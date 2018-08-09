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
          controller(0, 180.0 / (M_PI * 7.0)) {
    featEx = make_unique<FeatureExtractor>(wiz, 0.0, 0.0, 0.0);
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

    // read commanded joints positions
    commandedJointsPos = bodyUtils.readAction(action);

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
    //Vector3<double> selfPos = wiz.getAgentTranslation(agentNumber);
    if(modeling.getAgentModel().hasFallen()) {
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

    // read reference frame from file
    representations::NaoJoints referenceFrame = bodyUtils.readJointsFromFile(learningFile);
    // get actual frame
    representations::NaoJoints actualFrame = perception.getAgentPerception().getNaoJoints();

    // get joints difference norm
    double jointsDiffNorm = bodyUtils.getJointsDiffNorm(actualFrame, referenceFrame);

    // add joints position diff factor in reward
    reward += JOINTS_POSITIONS_WEIGHT * exp(- 2 * jointsDiffNorm);

    // update body models
    selfBodyModel.update(actualFrame);
    referenceBodyModel.update(referenceFrame);
    // get center of mass in the support foot referential
    Pose3D selfFactorTransform = selfBodyModel.getLeftLegEndEffectorTransform();
    Pose3D referenceFactorTransform = referenceBodyModel.getLeftLegEndEffectorTransform();
    Vector3<double> selfCenterOfMass = selfFactorTransform.invert() * selfBodyModel.computeCenterOfMass();
    Vector3<double> refCenterOfMass = referenceFactorTransform.invert() * referenceBodyModel.computeCenterOfMass();
    // get center of mass difference
    Vector3<double> CMdiff = selfCenterOfMass - refCenterOfMass;

    // add CM diff factor in reward
    reward += CENTER_OF_MASS_WEIGHT * exp(-10 * CMdiff.squareAbs());

    // now the end effector contribution
    Vector3<double> selfEndEffector = selfBodyModel.getRightLegEndEffectorTransform().translation;
    Vector3<double> refEndEffector = referenceBodyModel.getRightLegEndEffectorTransform().translation;
    Vector3<double> endEffectorDiff = selfEndEffector - refEndEffector;
    // add End Effector in reward
    reward += END_EFFECTOR_WEIGHT * exp(-40 * endEffectorDiff.squareAbs());

    LOG(INFO) << "current reward: " << reward;

    episodeAvgReward += reward;

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
    modeling.model(perception, controlStub);
}

void ZMPKickLearningAgent::drawEnvironment() {

}

void ZMPKickLearningAgent::drawStats() {

}
