//
// Created by luis on 7/15/18.
//

#include "ZMPKickLearningAgent.h"

#include "tools/rlearning3d/utils/EnvironmentUtils.h"
#include "tools/rlearning3d/utils/LearningConstants.h"
#include "external/easylogging++.h"

using itandroids_lib::math::sgn;

// Constants
// Robot
const double MAX_ANGLE_JOINTS = M_PI;

// RL
const int NUMBER_OF_STATE_DIM = 1;
const int NUMBER_OF_ACTION_DIM = representations::NaoJoints::NUM_JOINTS;
const int NUMBER_OF_STEPS_PER_EPISODE = 5000 * LearningConstants::NUM_STEP_SAME_INPUT;

ZMPKickLearningAgent::ZMPKickLearningAgent(std::string host, int serverPort, int monitorPort, int agentNumber,
                                           int robotType, std::string teamName)
        : BaseLearningAgent(host, serverPort, monitorPort, agentNumber, robotType, teamName),
          controller(0, 180.0 / (M_PI * 7.0)) {
    featEx = make_unique<FeatureExtractor>(wiz, 0.0, 0.0, 0.0);

    jointsWeight.neckPitch = 1;
    jointsWeight.neckYaw = 1;
    jointsWeight.leftShoulderPitch = 1;
    jointsWeight.leftShoulderYaw = 1;
    jointsWeight.leftArmRoll = 1;
    jointsWeight.leftArmYaw = 1;
    jointsWeight.rightShoulderPitch = 1;
    jointsWeight.rightShoulderYaw = 1;
    jointsWeight.rightArmRoll = 1;
    jointsWeight.rightArmYaw = 1;
    jointsWeight.leftHipYawPitch = 2;
    jointsWeight.leftHipRoll = 2;
    jointsWeight.leftHipPitch = 2;
    jointsWeight.leftKneePitch = 2;
    jointsWeight.leftFootPitch = 2;
    jointsWeight.leftFootRoll = 2;
    jointsWeight.rightHipYawPitch = 3;
    jointsWeight.rightHipRoll = 3;
    jointsWeight.rightHipPitch = 3;
    jointsWeight.rightKneePitch = 3;
    jointsWeight.rightFootPitch = 3;
    jointsWeight.rightFootRoll = 3;

    //std::ifstream config_doc("config/zmp_kick.json");
    //config_doc >> config;
}

State ZMPKickLearningAgent::newEpisode() {
    LOG(INFO) << "##############################################";
    LOG(INFO) << "Starting Episode: " << ++iEpi;
    nbEpisodeSteps = 0;
    episodeAvgReward = 0;
    learningFile.open("angles.txt"); // open file
    // set initial position in the field
    wiz.setPlayMode(representations::RawPlayMode::PLAY_ON);
    wiz.setBallPosition(Vector3<double>(2,2,0));
    wiz.setAgentPositionAndDirection(agentNumber, representations::PlaySide::LEFT,
                                     Vector3<double>(0, 0, 0.3), 0);
    // set initial joints pos
    for(int i = 0; i < 30; i++) {
        controller.update(initialJointsPos, perception.getAgentPerception().getNaoJoints());
        controlStub.naoJoints = controller.getCommands();
        step();
        if(i % 5 != 0)
            wiz.setAgentPositionAndDirection(agentNumber, representations::PlaySide::LEFT,
                                             Vector3<double>(0, 0, 0.3), 0);
    }

    return state();
}

SimulationResponse ZMPKickLearningAgent::runStep(Action action) {
    LOG(INFO) << "#######################";
    LOG(INFO) << "Step " << nbEpisodeSteps;

    commandedJointsPos.neckPitch = action.action(0);
    commandedJointsPos.neckYaw = action.action(1);
    commandedJointsPos.leftShoulderPitch = action.action(2);
    commandedJointsPos.leftShoulderYaw = action.action(3);
    commandedJointsPos.leftArmRoll = action.action(4);
    commandedJointsPos.leftArmYaw = action.action(5);
    commandedJointsPos.rightShoulderPitch = action.action(6);
    commandedJointsPos.rightShoulderYaw = action.action(7);
    commandedJointsPos.rightArmRoll = action.action(8);
    commandedJointsPos.rightArmYaw = action.action(9);
    commandedJointsPos.leftHipYawPitch = action.action(10);
    commandedJointsPos.leftHipRoll = action.action(11);
    commandedJointsPos.leftHipPitch = action.action(12);
    commandedJointsPos.leftKneePitch = action.action(13);
    commandedJointsPos.leftFootPitch = action.action(14);
    commandedJointsPos.leftFootRoll = action.action(15);
    commandedJointsPos.rightHipYawPitch = action.action(16);
    commandedJointsPos.rightHipRoll = action.action(17);
    commandedJointsPos.rightHipPitch = action.action(18);
    commandedJointsPos.rightKneePitch = action.action(19);
    commandedJointsPos.rightFootPitch = action.action(20);
    commandedJointsPos.rightFootRoll = action.action(21);

    if(iEpi % 1000 == 0){
        LOG(INFO) << "commanded joints positions: ";
        printJoints(commandedJointsPos);
    }


    // make 1 simulation step
    controller.update(commandedJointsPos, perception.getAgentPerception().getNaoJoints());
    controlStub.naoJoints = controller.getCommands();
    step();

    // compute reward
    currReward = reward();

    // update steps count
    nbEpisodeSteps++;
    nbTotalSteps++;

    ////////////////////////
    SimulationResponse stepUpdate;
    stepUpdate.mutable_state()->CopyFrom(state()); // end in next state S_t+1
    stepUpdate.set_reward(currReward);
    stepUpdate.set_done(episodeOver());

    return stepUpdate;
}

SetupEnvResponse ZMPKickLearningAgent::setup() {
    SetupEnvResponse initialInformation;

    initialInformation.set_num_state_dim(NUMBER_OF_STATE_DIM);
    initialInformation.set_num_action_dim(NUMBER_OF_ACTION_DIM);

    // Limits for actions
    // angle for each joint
    for (int i = 0; i < representations::NaoJoints::NUM_JOINTS; i++)
        initialInformation.add_action_bound(MAX_ANGLE_JOINTS);

    return initialInformation;
}

bool ZMPKickLearningAgent::episodeOver() {
    Vector3<double> selfPos = wiz.getAgentTranslation(agentNumber);
    if(selfPos.z < 0.2) {
        learningFile.close();
        LOG(INFO) << "Episode finishing because agent fell";
        LOG(INFO) << "Episode Average reward: " << episodeAvgReward / nbEpisodeSteps;
        LOG(INFO) << "Steps Until Fall: " << nbEpisodeSteps;
        return true;
    }

    if(learningFile.eof()){
        learningFile.close();
        LOG(INFO) << "Episode finishing because reached end of file";
        LOG(INFO) << "Episode Average reward: " << episodeAvgReward / nbEpisodeSteps;
        return true;
    }
    return false;
}

double ZMPKickLearningAgent::reward() {
    double reward = 0;

    // reward the time standing without fall
    reward += 20;

    Vector3<double> selfPos = wiz.getAgentTranslation(agentNumber);
    if(selfPos.z < 0.2)
        reward -= 1000;

    representations::NaoJoints referenceFrame;
    learningFile >> referenceFrame.neckPitch;
    learningFile >> referenceFrame.neckYaw;

    learningFile >> referenceFrame.leftShoulderPitch;
    learningFile >> referenceFrame.leftShoulderYaw;
    learningFile >> referenceFrame.leftArmRoll;
    learningFile >> referenceFrame.leftArmYaw;

    learningFile >> referenceFrame.rightShoulderPitch;
    learningFile >> referenceFrame.rightShoulderYaw;
    learningFile >> referenceFrame.rightArmRoll;
    learningFile >> referenceFrame.rightArmYaw;

    learningFile >> referenceFrame.leftHipYawPitch;
    learningFile >> referenceFrame.leftHipRoll;
    learningFile >> referenceFrame.leftHipPitch;
    learningFile >> referenceFrame.leftKneePitch;
    learningFile >> referenceFrame.leftFootPitch;
    learningFile >> referenceFrame.leftFootRoll;

    learningFile >> referenceFrame.rightHipYawPitch;
    learningFile >> referenceFrame.rightHipRoll;
    learningFile >> referenceFrame.rightHipPitch;
    learningFile >> referenceFrame.rightKneePitch;
    learningFile >> referenceFrame.rightFootPitch;
    learningFile >> referenceFrame.rightFootRoll;

    reward += -getJointsDiffNorm(commandedJointsPos, referenceFrame);
    episodeAvgReward += -getJointsDiffNorm(commandedJointsPos, referenceFrame);

    LOG(INFO) << "current reward: " << reward;
    
    return reward;
}

State ZMPKickLearningAgent::state() {
    // return next timestep
    State st;
    // read next time step from file
    string timeStep;
    learningFile >> timeStep;
    currState = timeStep.empty() ? currState + 0.02 : std::stod(timeStep);
    st.add_observation(currState);
    LOG(INFO) << "next state: " << st.observation(0);
    return st;
}

void ZMPKickLearningAgent::step() {
    action.act(decisionMaking, controlStub);
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
    communication.receiveMessage();
    perception.perceive(communication);
}

void ZMPKickLearningAgent::drawEnvironment() {

}

void ZMPKickLearningAgent::drawStats() {

}

double ZMPKickLearningAgent::getJointsDiffNorm(const representations::NaoJoints &frame1, const representations::NaoJoints &frame2) {
    using namespace itandroids_lib::math;

    double norm = 0;
    representations::NaoJoints frameDiff = frame1 - frame2;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.neckPitch)) * jointsWeight.neckPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.neckYaw)) * jointsWeight.neckYaw;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftShoulderPitch)) * jointsWeight.leftShoulderPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftShoulderYaw)) * jointsWeight.leftShoulderYaw;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftArmRoll)) * jointsWeight.leftArmRoll;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftArmYaw)) * jointsWeight.leftArmYaw;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightShoulderPitch)) * jointsWeight.rightShoulderPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightShoulderYaw)) * jointsWeight.rightShoulderYaw;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightArmRoll)) * jointsWeight.rightArmRoll;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightArmYaw)) * jointsWeight.rightArmYaw;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftHipYawPitch)) * jointsWeight.leftHipYawPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftHipRoll)) * jointsWeight.leftHipRoll;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftHipPitch)) * jointsWeight.leftHipPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftKneePitch)) * jointsWeight.leftKneePitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftFootPitch)) * jointsWeight.leftFootPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftFootRoll)) * jointsWeight.leftFootRoll;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightHipYawPitch)) * jointsWeight.rightHipYawPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightHipRoll)) * jointsWeight.rightHipRoll;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightHipPitch)) * jointsWeight.rightHipPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightKneePitch)) * jointsWeight.rightKneePitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightFootPitch)) * jointsWeight.rightFootPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightFootRoll)) * jointsWeight.rightFootRoll;

    return norm;
}

void ZMPKickLearningAgent::printJoints(representations::NaoJoints &frame) {
    LOG(INFO) << frame.neckPitch << " ";
    LOG(INFO) << frame.neckYaw << " ";

    LOG(INFO) << frame.leftShoulderPitch << " ";
    LOG(INFO) << frame.leftShoulderYaw << " ";
    LOG(INFO) << frame.leftArmRoll << " ";
    LOG(INFO) << frame.leftArmYaw << " ";

    LOG(INFO) << frame.rightShoulderPitch << " ";
    LOG(INFO) << frame.rightShoulderYaw << " ";
    LOG(INFO) << frame.rightArmRoll << " ";
    LOG(INFO) << frame.rightArmYaw << " ";

    LOG(INFO) << frame.leftHipYawPitch << " ";
    LOG(INFO) << frame.leftHipRoll << " ";
    LOG(INFO) << frame.leftHipPitch << " ";
    LOG(INFO) << frame.leftKneePitch << " ";
    LOG(INFO) << frame.leftFootPitch << " ";
    LOG(INFO) << frame.leftFootRoll << " ";

    LOG(INFO) << frame.rightHipYawPitch << " ";
    LOG(INFO) << frame.rightHipRoll << " ";
    LOG(INFO) << frame.rightHipPitch << " ";
    LOG(INFO) << frame.rightKneePitch << " ";
    LOG(INFO) << frame.rightFootPitch << " ";
    LOG(INFO) << frame.rightFootRoll << std::endl;
}


