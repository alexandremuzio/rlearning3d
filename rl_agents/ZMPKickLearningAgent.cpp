//
// Created by luis on 7/15/18.
//

#include "ZMPKickLearningAgent.h"

#include "tools/rlearning3d/utils/EnvironmentUtils.h"
#include "tools/rlearning3d/utils/LearningConstants.h"
#include "external/easylogging++.h"
#include "string.h"

using itandroids_lib::math::sgn;

// Constants
// Robot
const double MAX_ANGLE_JOINTS = M_PI;
const double JOINTS_POSITIONS_WEIGHT = 0.65;
const double CENTER_OF_MASS_WEIGHT = 0.2;
const double END_EFFECTOR_WEIGHT = 0.15;

// RL
const int NUMBER_OF_STATE_DIM = 1;
const int NUMBER_OF_ACTION_DIM = representations::NaoJoints::NUM_JOINTS;
const int NUMBER_OF_STEPS_PER_EPISODE = 5000 * LearningConstants::NUM_STEP_SAME_INPUT;

ZMPKickLearningAgent::ZMPKickLearningAgent(std::string host, int serverPort, int monitorPort, int agentNumber,
                                           int robotType, std::string teamName)
        : BaseLearningAgent(host, serverPort, monitorPort, agentNumber, robotType, teamName),
          controller(0, 180.0 / (M_PI * 7.0)), bodyUtils(serverPort),
          inverseKinematics(control::walking::RobotPhysicalParameters::getNaoPhysicalParameters()),
          kick(control::kick::KickZMPParams::getDefaultKickZMPParams(), false,
               control::walking::RobotPhysicalParameters::getNaoPhysicalParameters(), &inverseKinematics) {

    featEx = make_unique<FeatureExtractor>(wiz, 0.0, 0.0, 0.0);
    learningFile.open("angles.txt"); // open file
    rewardFile.open("../../plots/rewards" + std::to_string(serverPort) + ".txt");
    referenceMovement = bodyUtils.preProcessFile(learningFile);
    //std::ifstream config_doc("config/zmp_kick.json");
    //config_doc >> config;
}

State ZMPKickLearningAgent::newEpisode() {
    LOG(INFO) << "##############################################";
    LOG(INFO) << "Starting Episode: " << ++iEpi;
    nbEpisodeSteps = 0;
    episodeAvgReward = 0;
    iterator = 0;

    // set initial position in the field
    wiz.setGameTime(0);
    wiz.setPlayMode(representations::RawPlayMode::PLAY_ON);
    wiz.setBallPosition(Vector3<double>(2, 2, 0));
    wiz.setAgentPositionAndDirection(agentNumber, representations::PlaySide::LEFT,
                                     Vector3<double>(0.0, 0.0, 0.35), 0);
    // set initial joints pos
    decisionMaking.movementRequest == nullptr;
    for (int i = 0; i < 20; i++)
        fullStep();

    // print episode number
    string episodeString = "new episode " + to_string(iEpi);
    string epiString = "stats.epi";
    roboviz->drawAnnotation(&episodeString, 0, 0.3, 0.0, 0.0, 0.0, 1.0, &epiString);
    std::string buffer(epiString);
    roboviz->swapBuffers(&buffer);
    // check initial positions
    while (!bodyUtils.initialPosSanityCheck(perception.getAgentPerception().getNaoJoints())) {
        for (int i = 0; i < 50; i++)
            fullStep();
    }
    wiz.setAgentPositionAndDirection(agentNumber, representations::PlaySide::LEFT,
                                     Vector3<double>(0.0, 0.0, 0.35), 0);
    // go to initial kick position
    for(; iterator < 100; iterator++){
        kick.update(0.0,commandedJointsPos);
        commandedJointsPos.scale(iterator * 0.02 / 2.0);
        commandedJointsPos *= representations::NaoJoints::getNaoDirectionsFixing();
        step();
    }
    return state();
}

SimulationResponse ZMPKickLearningAgent::runStep(Action action) {
//    LOG(INFO) << "#######################";
//    LOG(INFO) << "Step " << nbEpisodeSteps;

    // read commanded joints positions
    commandedJointsPos = bodyUtils.readAction(action);
    // make 1 simulation step
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
    if (modeling.getAgentModel().hasFallen() /*selfPos.z < 0.27*/ ) {
        LOG(INFO) << "Episode finishing because agent fell";
        LOG(INFO) << "Episode Average reward: " << episodeAvgReward / nbEpisodeSteps;
        LOG(INFO) << "Steps Until Fall: " << nbEpisodeSteps;
        decisionMaking.movementRequest = nullptr;
        for (int i = 0; i < 80; i++)
            fullStep();
        return true;
    }
    if (iterator >= referenceMovement.size()) {
        LOG(INFO) << "Episode finishing because reached end of file";
        LOG(INFO) << "Episode Average reward: " << episodeAvgReward / nbEpisodeSteps;
        decisionMaking.movementRequest = nullptr;
        for (int i = 0; i < 80; i++)
            fullStep();
        return true;
    }
    return false;
}

double ZMPKickLearningAgent::reward() {
    double reward = 0;

    // read reference frame from file
    representations::NaoJoints referenceFrame = referenceMovement[iterator++];
    // get actual frame
    representations::NaoJoints actualFrame = perception.getAgentPerception().getNaoJoints();

    // get joints difference norm
    double jointsDiffNorm = bodyUtils.getJointsDiffNorm(actualFrame, referenceFrame);

    // add joints position diff factor in reward
    reward += JOINTS_POSITIONS_WEIGHT * exp(-2 * jointsDiffNorm);

    // update body models
    selfBodyModel.update(actualFrame);
    referenceBodyModel.update(referenceFrame);
    // get center of mass in the support foot referential
    Pose3D selfFactorTransform = selfBodyModel.getLeftLegEndEffectorTransform();
    Pose3D referenceFactorTransform = referenceBodyModel.getLeftLegEndEffectorTransform();
    Vector3<double> selfCenterOfMass = selfFactorTransform.invert() * selfBodyModel.computeCenterOfMass();
    Vector3<double> refCenterOfMass = referenceFactorTransform.invert() * referenceBodyModel.computeCenterOfMass();
    // get center of mass difference
    Vector3<double> cmDiff = selfCenterOfMass - refCenterOfMass;

    // add CM diff factor in reward
    reward += CENTER_OF_MASS_WEIGHT * exp(-10 * cmDiff.squareAbs());

    // now the end effector contribution
    Vector3<double> selfEndEffector = selfBodyModel.getRightLegEndEffectorTransform().translation;
    Vector3<double> refEndEffector = referenceBodyModel.getRightLegEndEffectorTransform().translation;
    Vector3<double> endEffectorDiff = selfEndEffector - refEndEffector;
    // add End Effector in reward
    reward += END_EFFECTOR_WEIGHT * exp(-40 * endEffectorDiff.squareAbs());

//    LOG(INFO) << "current reward: " << reward;

    episodeAvgReward += reward;

    return reward;
}

State ZMPKickLearningAgent::state() {
    // return next timestep
    State st;
    st.add_observation(iterator * 0.02 - 2);
    return st;
}

void ZMPKickLearningAgent::step() {
    // setup control
    controller.update(commandedJointsPos, perception.getAgentPerception().getNaoJoints());
    controlStub.naoJoints = controller.getCommands();
    // remaining part of cycle
    action.act(decisionMaking, controlStub);
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
    modeling.model(perception, controlStub);
}

void ZMPKickLearningAgent::fullStep() {
    control.control(perception, modeling, decisionMaking);
    action.act(decisionMaking, control);
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
    modeling.model(perception, control);
}

void ZMPKickLearningAgent::drawEnvironment() {

}

void ZMPKickLearningAgent::drawStats() {

}
