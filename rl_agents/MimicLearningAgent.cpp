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
}

State MimicLearningAgent::newEpisode() {
    LOG(INFO) << "##############################################";
    LOG(INFO) << "Starting Episode: " << ++iEpi;
    nbEpisodeSteps = 0;
    episodeAvgReward = 0;
    learningFile.open("angles.txt"); // open file

    return state();
}

SimulationResponse MimicLearningAgent::runStep(Action action) {
//    LOG(INFO) << "#######################";
//    LOG(INFO) << "Step " << nbEpisodeSteps;
    // read commanded joints positions
    commandedJointsPos = bodyUtils.readAction(action);
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
    if(learningFile.eof()){
        learningFile.close();
        LOG(INFO) << "Episode finishing because reached end of file";
        LOG(INFO) << "Episode Average reward: " << episodeAvgReward / nbEpisodeSteps;
        return true;
    }
    return false;
}

double MimicLearningAgent::reward() {
    // read reference frame from file
    representations::NaoJoints referenceFrame = bodyUtils.readJointsFromFile(learningFile);
    if(iEpi % 10 == 0)
        bodyUtils.printJoints(referenceFrame, commandedJointsPos);
    // get joints commands difference norm
    double jointsDiffNorm = bodyUtils.getJointsDiffNorm(commandedJointsPos, referenceFrame);
    double reward = -jointsDiffNorm;
    episodeAvgReward += reward;
    return  reward;
}

State MimicLearningAgent::state() {
    // return next timestep
    State st;
    // read next time step from file
    string timeStep;
    learningFile >> timeStep;
    currState = timeStep.empty() ? currState + 0.02 : std::stod(timeStep);
    st.add_observation(currState);
//    LOG(INFO) << "next state: " << st.observation(0);
    return st;
}
