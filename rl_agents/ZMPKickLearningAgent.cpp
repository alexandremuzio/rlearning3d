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

    commandedJointsPos = bodyUtils.readAction(action);

    if(iEpi % 1000 == 0){
        LOG(INFO) << "commanded joints positions: ";
        bodyUtils.printJoints(commandedJointsPos);
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

    representations::NaoJoints referenceFrame = bodyUtils.readJointsFromFile(learningFile);

    double jointsDiffNorm = bodyUtils.getJointsDiffNorm(commandedJointsPos, referenceFrame);

    reward += -jointsDiffNorm;

    episodeAvgReward += -jointsDiffNorm;

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
