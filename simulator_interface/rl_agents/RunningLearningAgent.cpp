#include "RunningLearningAgent.h"

#include "external/easylogging++.h"

#include <random>

// Constants
// Robot
const double MAX_X_SPEED = 0.9;

// Environment
const double MAX_FIELD_X = 4;
const double MAX_FIELD_Y = 3;

// RL
const int NUMBER_OF_STATE_DIM = 1;
const int NUMBER_OF_ACTION_DIM = 1;
const int NUM_STEP_SAME_INPUT = 1;
const int NUMBER_OF_STEPS_PER_EPISODE = 2000 / NUM_STEP_SAME_INPUT;
const double SERVER_STEP_TIME = 1 / 60.0;

////////////////////////
// Helper utils methods
static void drawEnvironment(utils::roboviz::Roboviz& roboviz) {
    std::string selfStr("pos.field");
//    roboviz.drawLine(-MAX_FIELD_X, -MAX_FIELD_Y, 0, -MAX_FIELD_X, MAX_FIELD_Y, 0, 5, 1, 1, 0, &selfStr);
    roboviz.drawLine(-0.2, 0.0, 0, -0.2, 5.0, 0, 5, 1, 1, 0, &selfStr);
//    roboviz.drawLine(-MAX_FIELD_X, -MAX_FIELD_Y, 0, MAX_FIELD_X, -MAX_FIELD_Y, 0, 5, 1, 1, 0, &selfStr);
    roboviz.drawLine(0.2, 0.0, 0, 0.2, 5.0,0, 5, 1, 1, 0, &selfStr);

    std::string buffer("pos.field");
    roboviz.swapBuffers(&buffer);
}

////////////////////////
void RunningLearningAgent::drawStats() {
    string stats = "stats." + to_string(agentNumber);
    string rewardString = "Reward: " + to_string(reward());
    string refXSpeedString = "Ref X Speed: " + to_string(referenceXSpeed);
    string actionXSpeedString = "Action X Speed: " + to_string(actionXSpeed);
    string currXSpeedString = "Curr X Speed: " + to_string(currentXSpeed);
    string currPosition1 = "Curr Position: (" + to_string(getAgentPos().x) + ", " + to_string(getAgentPos().y) + ", " + to_string(
            getAgentPos().z) + ")";

    roboviz->drawAnnotation(&rewardString, 3*(agentNumber- 1), MAX_FIELD_Y - 0.3, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&refXSpeedString, 3*(agentNumber- 1), MAX_FIELD_Y - 0.6, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&actionXSpeedString, 3*(agentNumber- 1), MAX_FIELD_Y - 0.9, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&currXSpeedString, 3*(agentNumber- 1), MAX_FIELD_Y - 1.2, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&currPosition1, 3*(agentNumber- 1), MAX_FIELD_Y - 1.5, 0.0, 0.0, 0.0, 1.0, &stats);

    std::string buffer(stats);
    roboviz->swapBuffers(&buffer);
}
////////////////////////

RunningLearningAgent::RunningLearningAgent(std::string host, int serverPort, int monitorPort, int agentNumber,
                                         int robotType, std::string teamName)
        : BaseLearningAgent(host, serverPort, monitorPort, agentNumber, robotType, teamName) {
}


State RunningLearningAgent::newEpisode() {
    static int iEpi = 0;
    LOG(INFO) << "#######################################################";
    LOG(INFO) << "Starting Episode: " << ++iEpi;
    LOG(INFO) << "Robot starting position: 0.0, 0.0";

    INITIAL_ROBOT_THETA = M_PI_2;


    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0, MAX_X_SPEED);

    // random reference speed
    referenceXSpeed = 0.4; //dis(gen);

    LOG(INFO) << "Reference speed: " << referenceXSpeed;


    // starting position
    wiz.setPlayMode(representations::RawPlayMode::PLAY_ON);
    wiz.setAgentPositionAndDirection(agentNumber, representations::PlaySide::LEFT,
                                     Vector3<double>(0.0, agentNumber - 1, 0.3), M_PI_2);

    wiz.setBallPosition(Vector3<double>(10.0, 10.0, 0.0));

    decisionMaking.movementRequest = std::make_shared<control::WalkRequest>(Vector3<double>(0.0, 0.0, 0.0));
    for (int i =0; i < 30; i++)
        step();

    lastPose = Pose2D(0.0, 0.0, 0.0);
    currentPose = Pose2D(getAgentAngle(), getAgentPos().x, getAgentPos().y);
    currentXSpeed = 0.0;

    nbSteps = 0;

    return state();
}

SimulationResponse RunningLearningAgent::runStep(Action action) {
    LOG(INFO) << "#######################";
    LOG(INFO) << "Step " << nbSteps;
    LOG(INFO) << "Pos " << getAgentPos() << " " << getAgentAngle();
    LOG(INFO) << "SpeedX " << currentXSpeed;

    actionXSpeed = action.action(0);

    LOG(INFO) << "Action speedX: " << actionXSpeed;

    //Execute action from server
    decisionMaking.movementRequest = std::make_shared<control::WalkRequest>(Vector3<double>(actionXSpeed, 0.0, 0.0));

    ////////////////////////
    // SOCCER 3D SIMULATION STEP
    // Uses same input for some steps
    for (int i = 0; i < NUM_STEP_SAME_INPUT; i++) {
        step();
    }
    nbSteps++;

    // Update speed
    lastPose = currentPose;
    currentPose = Pose2D(getAgentAngle(), getAgentPos().x, getAgentPos().y);


    double alpha = 0.8;
    currentXSpeed += alpha * (currentPose.globalToRelative(lastPose).translation.y / (SERVER_STEP_TIME * NUM_STEP_SAME_INPUT) - currentXSpeed);

    // drawEnvironment(roboviz);
    drawStats();
    ////////////////////////

    SimulationResponse stepUpdate;
    stepUpdate.mutable_state()->CopyFrom(state());
    stepUpdate.set_reward(reward());
    stepUpdate.set_done(episodeOver());

    return stepUpdate;
}

SetupEnvResponse RunningLearningAgent::setup() {
    SetupEnvResponse initialInformation;

    initialInformation.set_num_state_dim(NUMBER_OF_STATE_DIM);
    initialInformation.set_num_action_dim(NUMBER_OF_ACTION_DIM);

    // Limits for actions
    // x speed
    initialInformation.add_action_bound(MAX_X_SPEED);

    return initialInformation;
}

bool RunningLearningAgent::episodeOver() {
    if (robotFallen()) {
        LOG(INFO) << "Episode finishing because robot fell";
        return true;
    }

    if (nbSteps >= NUMBER_OF_STEPS_PER_EPISODE) {
        LOG(INFO) << "Episode finishing because of number of steps";
        return true;
    }

    return false;
}

double RunningLearningAgent::reward() {
    double reward = 0;

//    reward += - pow(referenceXSpeed - currentXSpeed, 2);
    if (abs(currentXSpeed - referenceXSpeed) < 0.05) {
        reward += 1;
    }

    // Greatly punish robot falling.
    if (robotFallen()) {
        reward += -100;
    }

    LOG(INFO) << "Reward: " << reward;

    return reward;
}

State RunningLearningAgent::state() const {
    State st;

    st.add_observation(currentXSpeed);

    return st;
}

void RunningLearningAgent::step() {
    control.control(perception, modeling, decisionMaking);
    action.act(decisionMaking, control);
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
//    modeling.model(perception, control);
}
