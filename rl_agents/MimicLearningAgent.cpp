//
// Created by luis on 9/2/18.
//

#include "MimicLearningAgent.h"
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

MimicLearningAgent::MimicLearningAgent(string host, int serverPort, int monitorPort, int agentNumber, int robotType,
                                       string teamName) {
    learningFile.open("angles.txt"); // open file
    rewardFile.open("../../plots/rewards.txt");
    referenceMovement = bodyUtils.preProcessFile(learningFile);
}

State MimicLearningAgent::newEpisode() {
    LOG(INFO) << "##############################################";
    LOG(INFO) << "Starting Episode: " << ++iEpi;
    nbEpisodeSteps = 0;
    episodeAvgReward = 0;
    iterator = 0;
    return state();
}

SimulationResponse MimicLearningAgent::runStep(Action action) {
//    LOG(INFO) << "#######################";
//    LOG(INFO) << "Step " << nbEpisodeSteps;
    // read commanded joints positions
    commandedJointsPos = bodyUtils.readAction(action);
    //bodyUtils.printJoints(referenceMovement[iterator]);
    currReward = reward();
    //LOG(INFO) << "end up in state " << iterator << " " << state().observation(0);
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

SetupEnvResponse MimicLearningAgent::setup() {
    SetupEnvResponse initialInformation;

    initialInformation.set_num_state_dim(NUMBER_OF_STATE_DIM);
    initialInformation.set_num_action_dim(NUMBER_OF_ACTION_DIM);

    // Limits for actions
    // angle for each joint
    for (int i = 0; i < representations::NaoJoints::NUM_JOINTS; i++)
        initialInformation.add_action_bound(MAX_ANGLE_JOINTS);

    return initialInformation;
}

bool MimicLearningAgent::episodeOver() {
    if(iterator >= referenceMovement.size()){
        LOG(INFO) << "Episode finishing because reached end of file";
        LOG(INFO) << "Episode Average reward: " << episodeAvgReward / nbEpisodeSteps;
        rewardFile << episodeAvgReward / nbEpisodeSteps;
        return true;
    }
    return false;
}

double MimicLearningAgent::reward() {
    // read reference frame from file
    if(iEpi % 10 == 0)
        bodyUtils.printJoints(referenceMovement[iterator], commandedJointsPos);
    // get joints commands difference norm
    double jointsDiffNorm = bodyUtils.getJointsDiffNorm(commandedJointsPos, referenceMovement[iterator++]);
    double reward = -jointsDiffNorm;
    episodeAvgReward += reward;
    return reward;
}

State MimicLearningAgent::state() {
    // return next timestep
    State st;
    st.add_observation(iterator * 0.02);
//    LOG(INFO) << "next state: " << st.observation(0);
    return st;
}
