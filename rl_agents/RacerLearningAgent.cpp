#include "RacerLearningAgent.h"

#include "external/easylogging++.h"

#include <random>

// Constants
// Robot
const double MAX_X_SPEED = 0.9;
const double MAX_Y_SPEED = 0.4;
const double MAX_THETA_SPEED = 300.0 * M_PI / 180.0;

// Environment
const double RACE_SIZE = 18;
const double LANE_HALF_WIDTH = 2;

// RL
const int NUMBER_OF_STATE_DIM = 7;
const int NUMBER_OF_ACTION_DIM = 3;
const int NUM_STEP_SAME_INPUT = 1;
const int NUMBER_OF_STEPS_PER_EPISODE = 2000 / NUM_STEP_SAME_INPUT;
const double SERVER_STEP_TIME = 1 / 60.0;

////////////////////////
// Helper utils methods
void RacerLearningAgent::drawEnvironment() {
    std::string selfStr("pos.field");
//    roboviz->drawLine(0, INITIAL_ROBOT_Y, 0, 0, RACE_SIZE + INITIAL_ROBOT_Y, 0, 5, 1, 1, 1, &selfStr);
//    roboviz->drawLine(-LANE_HALF_WIDTH, INITIAL_ROBOT_Y, 0, -LANE_HALF_WIDTH, RACE_SIZE + INITIAL_ROBOT_Y, 0, 5, 0, 0, 0, &selfStr);
//    roboviz->drawLine(LANE_HALF_WIDTH, INITIAL_ROBOT_Y, 0, LANE_HALF_WIDTH, RACE_SIZE + INITIAL_ROBOT_Y, 0, 5, 0, 0, 0, &selfStr);
//
//    roboviz->drawLine(-LANE_HALF_WIDTH, INITIAL_ROBOT_Y, 0, LANE_HALF_WIDTH, INITIAL_ROBOT_Y, 0, 5, 0, 1, 0, &selfStr);
//    roboviz->drawLine(-LANE_HALF_WIDTH, RACE_SIZE + INITIAL_ROBOT_Y, 0, LANE_HALF_WIDTH, RACE_SIZE + INITIAL_ROBOT_Y,0, 5, 1, 0, 0, &selfStr);
//
//    std::string buffer("pos.field");
//    roboviz->swapBuffers(&buffer);
}
void RacerLearningAgent::drawStats() {
//    string stats = "stats";
//    string episodeString = "Episode: " + to_string(iEpi);
//    string stepsString = "Number of steps: " + to_string(nbSteps);
//    string rewardString = "Reward: " + to_string(reward());
//    string actionSpeedString = "Action Speed: (" + to_string(actionXSpeed) + ", " + to_string(actionYSpeed) + ", " + to_string(
//            actionThetaSpeed) + ")";
//    string currPosition = "Curr Pose: (" + to_string(currentPos.x) + ", " + to_string(currentPos.y) + ", " + to_string(
//            currentTheta) + ")";
//    string currSpeedString = "Curr Speed: (" + to_string(currentXSpeed) + ", " + to_string(currentYSpeed) + ", " + to_string(
//            currentThetaSpeed) + ")";
//
//
//    string epiString = stats + ".epi";
//    roboviz->drawAnnotation(&episodeString, 3*(agentNumber- 1), 0.3, 0.0, 0.0, 0.0, 1.0, &epiString);
//    roboviz->drawAnnotation(&stepsString, 3*(agentNumber- 1), 0.0, 0.0, 0.0, 0.0, 1.0, &stats);
//    roboviz->drawAnnotation(&rewardString, 3*(agentNumber- 1), -0.3, 0.0, 0.0, 0.0, 1.0, &stats);
//    roboviz->drawAnnotation(&currSpeedString, 3*(agentNumber- 1), -0.9, 0.0, 0.0, 0.0, 1.0, &stats);
//    roboviz->drawAnnotation(&actionSpeedString, 3*(agentNumber- 1), -1.2, 0.0, 0.0, 0.0, 1.0, &stats);
//    roboviz->drawAnnotation(&currPosition, 3*(agentNumber- 1), -1.5, 0.0, 0.0, 0.0, 1.0, &stats);
//
//    std::string buffer(stats);
//    roboviz->swapBuffers(&buffer);
}
bool RacerLearningAgent::robotLeftTrack() {
    return fabs(currentPos.x) > LANE_HALF_WIDTH;
}
bool RacerLearningAgent::robotReachedFinishLine() {
    return currentPos.y > RACE_SIZE;
}
////////////////////////

RacerLearningAgent::RacerLearningAgent(std::string host, int serverPort, int monitorPort, int agentNumber,
                                           int robotType, std::string teamName)
        : BaseLearningAgent(host, serverPort, monitorPort, agentNumber, robotType, teamName) {
}


State RacerLearningAgent::newEpisode() {
    LOG(INFO) << "#######################################################";
    LOG(INFO) << "Starting Episode: " << ++iEpi;

    INITIAL_ROBOT_Y = -10.0;
    INITIAL_ROBOT_THETA = M_PI_2;

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0, MAX_X_SPEED);


    // starting position
    wiz.setPlayMode(representations::RawPlayMode::PLAY_ON);
    wiz.setAgentPositionAndDirection(agentNumber, representations::PlaySide::LEFT,
                                     Vector3<double>(0.0, INITIAL_ROBOT_Y, 0.3), M_PI_2);

    wiz.setBallPosition(Vector3<double>(10.0, 10.0, 0.0));

    decisionMaking.movementRequest = std::make_shared<control::WalkRequest>(Vector3<double>(0.0, 0.0, 0.0));
    for (int i =0; i < 30; i++)
        step();


    lastPos = Vector3<double>(0.0, 0.0, 0.0);
    lastTheta = 0.0;

    currentPos = getAgentPos();
    currentTheta = getAgentAngle();

    nbSteps = 0;

    return state();
}

SimulationResponse RacerLearningAgent::runStep(Action action) {
    LOG(INFO) << "#######################";
    LOG(INFO) << "Step " << nbSteps;
    LOG(INFO) << "Pos: " << currentPos << " " << currentTheta;
    LOG(INFO) << "Speed: " << currentXSpeed << " " << currentYSpeed << " " << currentThetaSpeed;

    actionXSpeed = action.action(0);
    actionYSpeed = action.action(1);
    actionThetaSpeed = action.action(2);

    LOG(INFO) << "Action speedX: " << actionXSpeed << " speedY: " << actionYSpeed << " speedTheta: " << actionThetaSpeed;

    // Execute action from server
    decisionMaking.movementRequest = std::make_shared<control::WalkRequest>(Vector3<double>(actionXSpeed, actionYSpeed, actionThetaSpeed));

    // Follow line
    // behaviorFactory.getFollowLine().setLine(M_PI_2, Vector2<double>(0.0, 0.0));
    // behaviorFactory.getFollowLine().behave(modeling);
    // decisionMaking.movementRequest = behaviorFactory.getFollowLine().getMovementRequest();

    ////////////////////////
    // SOCCER 3D SIMULATION STEP
    // Uses same input for some steps
    for (int i = 0; i < NUM_STEP_SAME_INPUT; i++) {
        step();
    }
    nbSteps++;

    // Update speed
    lastPos = currentPos;
    lastTheta = currentTheta;

    currentPos = getAgentPos();
    currentTheta = getAgentAngle();


    double xSpeed =  (currentPos - lastPos).x / (SERVER_STEP_TIME * NUM_STEP_SAME_INPUT);
    double ySpeed = (currentPos - lastPos).y / (SERVER_STEP_TIME * NUM_STEP_SAME_INPUT);
    double thetaSpeed = (currentTheta - lastTheta) / (SERVER_STEP_TIME * NUM_STEP_SAME_INPUT);

    // Filter speeds
    double alpha = 0.5;
    currentXSpeed += alpha * (xSpeed - currentXSpeed);
    currentYSpeed += alpha * (ySpeed - currentYSpeed);
    currentThetaSpeed += alpha * (thetaSpeed - currentThetaSpeed);


    drawEnvironment();
    drawStats();
    ////////////////////////

    SimulationResponse stepUpdate;
    stepUpdate.mutable_state()->CopyFrom(state());
    stepUpdate.set_reward(reward());
    stepUpdate.set_done(episodeOver());

    return stepUpdate;
}

SetupEnvResponse RacerLearningAgent::setup() {
    SetupEnvResponse initialInformation;

    initialInformation.set_num_state_dim(NUMBER_OF_STATE_DIM);
    initialInformation.set_num_action_dim(NUMBER_OF_ACTION_DIM);

    // Limits for actions
    // x speed
    initialInformation.add_action_bound(MAX_X_SPEED);
    initialInformation.add_action_bound(MAX_Y_SPEED);
    initialInformation.add_action_bound(MAX_THETA_SPEED);

    return initialInformation;
}

bool RacerLearningAgent::episodeOver() {

    if (robotReachedFinishLine()) {
        LOG(INFO) << "Episode finishing because robot reached finish line!!!";
        return true;
    }

    if (robotFallen()) {
        LOG(INFO) << "Episode finishing because robot fell";
        return true;
    }

    if (robotLeftTrack()) {
        LOG(INFO) << "Episode finishing because left track";
        return true;
    }

    if (nbSteps >= NUMBER_OF_STEPS_PER_EPISODE) {
        LOG(INFO) << "Episode finishing because of number of steps";
        return true;
    }

    return false;
}

double RacerLearningAgent::reward() {
    double dt = (SERVER_STEP_TIME * NUM_STEP_SAME_INPUT);

    double reward = 0;

    double vx = (currentPos.y - lastPos.y) / dt;
    double vy = (currentPos.x - lastPos.x) / dt;
    double vtheta = (currentTheta - lastTheta) / dt;
    double actionNorm = Vector3<double>(actionXSpeed, actionYSpeed, actionThetaSpeed).squareAbs();

    reward += min(vx, MAX_X_SPEED);

    reward -= 0.005 * (vy * vy + vtheta * vtheta);

    reward -= 0.05 * actionNorm;
//    reward -= 0.01 *  max(actionXSpeed - MAX_X_SPEED, 0.0); ///////////////

    reward -= 0.05 * currentPos.x * currentPos.x;

    if (robotReachedFinishLine()) {
        reward += 50;
    }

    // Greatly punish robot falling.
    if (robotFallen()) {
        reward += -10;
    }

    if (robotLeftTrack()) {
        reward += -10;
    }

    LOG(INFO) << "Reward: " << reward;

    return reward;
}

State RacerLearningAgent::state() const {
    State st;

    st.add_observation(currentPos.x);
    st.add_observation(currentPos.y);
    st.add_observation(currentPos.z);
    st.add_observation(currentTheta);
    st.add_observation(currentXSpeed);
    st.add_observation(currentYSpeed);
    st.add_observation(currentThetaSpeed);

    return st;
}

void RacerLearningAgent::step() {
    control.control(perception, modeling, decisionMaking);
    action.act(decisionMaking, control);
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
//    modeling.model(perception, control);
}
