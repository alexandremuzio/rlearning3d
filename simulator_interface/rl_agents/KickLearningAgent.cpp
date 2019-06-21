//
// Created by luckeciano on 6/1/18.
//

#include "external/easylogging++.h"
#include "KickLearningAgent.h"

// RL
const int NUMBER_OF_STATE_DIM = 1;
const int NUMBER_OF_ACTION_DIM = representations::NaoJoints::NUM_JOINTS + 1;
const int NUM_STEP_SAME_INPUT = 1;
const double STEP_TIME = 0.02;
const double VELOCITY_THRESHOLD = 0.1;
double EPISODE_STEPS = 200; //reference_motion time
const Vector3<double> INITIAL_BALL_POS = Vector3<double>(-10.0, 0.0, 0.0);
double jointsErrorWeights[representations::NaoJoints::NUM_JOINTS] = {1.0, 1.0, 2.0, 2.0, 2.0, 1.0, 2.0, 2.0, 2.0, 1.0,
                                                                     5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

const double MAX_JOINTS_ERROR = 1000.0;

std::vector<std::string> split(string phrase, string delimiter){
    vector<string> list;
    string s = phrase;
    size_t pos = 0;
    string token;
    while ((pos = s.find(delimiter)) != string::npos) {
        token = s.substr(0, pos);
        list.push_back(token);
        s.erase(0, pos + delimiter.length());
    }
    return list;
}


void KickLearningAgent::drawEnvironment() {
    // Use this method to draw with roboviz
}

void KickLearningAgent::drawStats() {
    string stats = "stats";
    string episodeString = "Episode: " + to_string(iEpi);
    string stepsString = "Number of steps: " + to_string(nbSteps);
    string rewardString = "Reward: " + to_string(reward());
    string robotFallenString = "Robot Fallen: " + to_string(robotFallen());
    string distanceString = "Distance: (" + to_string(xDistance) + ", " + to_string(yDistance) + ")" ;
    string maxHeightString = "Max Height: (" + to_string(maxHeight) + ")";
    string ballVelocityString = "Ball Velocity: (" + to_string(ballVelocity.abs()) + ")";


    string epiString = stats + ".epi";
    roboviz->drawAnnotation(&episodeString, 3*(agentNumber- 1), 0.3, 0.0, 0.0, 0.0, 1.0, &epiString);
    roboviz->drawAnnotation(&stepsString, 3*(agentNumber- 1), 0.0, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&rewardString, 3*(agentNumber- 1), -0.3, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&robotFallenString, 3*(agentNumber- 1), -0.6, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&distanceString, 3*(agentNumber- 1), -0.9, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&maxHeightString, 3*(agentNumber- 1), -1.2, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&ballVelocityString, 3*(agentNumber- 1), -1.5, 0.0, 0.0, 0.0, 1.0, &stats);

    std::string buffer(stats);
    roboviz->swapBuffers(&buffer);
}

KickLearningAgent::KickLearningAgent(std::string host, int serverPort, int monitorPort, int agentNumber,
                                       int robotType, std::string teamName)
        : BaseLearningAgent(host, serverPort, monitorPort, agentNumber, robotType, teamName) {
    std::ifstream infile;
    infile.open("reference_kick_itandroids.txt");
    string aux="";
    getline(infile,aux);
    int nbStepSim = 0;
    lastRefError = 0.0;
    while(!infile.eof()) {
        aux = "";
        getline(infile, aux);
        if(aux.size() == 0) break;
        vector<string> values = split(aux, "::");
        std::vector<double> jointsVector;
        referenceMotionMap[nbStepSim] = jointsVector;
        for (int i = 1; i <= representations::NaoJoints::NUM_JOINTS; i++)
            referenceMotionMap[nbStepSim].push_back(stod(values[i]));
        nbStepSim++;
    }
}

State KickLearningAgent::newEpisode() {
    LOG(INFO) << "#######################################################";
    LOG(INFO) << "Starting Episode: " << ++iEpi;

    // starting position
    wiz.setGameTime(0);
    wiz.setPlayMode(representations::RawPlayMode::PLAY_ON);
    //setAgentRandomPosition();
    wiz.setAgentPositionAndDirection(
            agentNumber,
            representations::PlaySide::LEFT,
            Vector3<double>(INITIAL_BALL_POS.x - 0.2, INITIAL_BALL_POS.y, 0.3),
            0.0);

    wiz.setBallPosition(INITIAL_BALL_POS);


    nbSteps = 0;
    keyframeTime = -1;
    xDistance = yDistance = maxHeight = 0.000;
    ballVelocity = Vector3<double>(0,0,0);
    hasEnded = false;
    ballKicked = false;

    control.RestartController();

    decisionMaking.movementRequest = nullptr;
    for (int i = 0; i < 80; i++ )
        step();
    behaviorFactory.getKickBehavior().setStartedKick(false);
    while (!behaviorFactory.getKickBehavior().hasStartedKick()) {
        if (!decisionMaking.decideFall(modeling)) {
            kick(); // Agent always try to kick the ball
            look(); // Agent look movement

        }
        step();
    }


    lastBallTranslation = wiz.getBallTranslation();

    return state();
}

SimulationResponse KickLearningAgent::runStep(Action action) {
    if (!ballKicked && ballVelocity.abs() > VELOCITY_THRESHOLD)
        ballKicked = true;

    if (!hasEnded) {
        desiredJoints.clear();
        for (int i = 0; i < representations::NaoJoints::NUM_JOINTS; i++) {
            double x = action.action(i);
            desiredJoints.push_back(x);
        }

        keyframeStage = action.action(representations::NaoJoints::NUM_JOINTS);
        hasEnded = (keyframeStage > 0.9);
        LOG(INFO) << "Has Ended: " << hasEnded;

        if (hasEnded && keyframeTime < 0) {
            keyframeTime = nbSteps * STEP_TIME;
        }

    }


    ////////////////////////
    // SOCCER 3D SIMULATION STEP
    // Uses same input for some steps
    for (int i = 0; i < NUM_STEP_SAME_INPUT; i++) {
        stepControl();
    }

    nbSteps++;
    currentPos = wiz.getAgentTranslation();

    if (nbSteps % 10 == 0) {
        ballVelocity.x = (wiz.getBallTranslation().x - lastBallTranslation.x) / (10*STEP_TIME);
        ballVelocity.y = (wiz.getBallTranslation().y - lastBallTranslation.y) / (10*STEP_TIME);
        ballVelocity.z = (wiz.getBallTranslation().z - lastBallTranslation.z) / (10*STEP_TIME);
        lastBallTranslation = wiz.getBallTranslation();
    }
    xDistance = wiz.getBallTranslation().x - INITIAL_BALL_POS.x;
    yDistance = wiz.getBallTranslation().y - INITIAL_BALL_POS.y;
    maxHeight = max(maxHeight, wiz.getBallTranslation().z);


    drawEnvironment();
    drawStats();
    ////////////////////////

    SimulationResponse stepUpdate;
    stepUpdate.mutable_state()->CopyFrom(state());
    stepUpdate.set_reward(reward());
    stepUpdate.set_done(episodeOver());

    return stepUpdate;
}

SetupEnvResponse KickLearningAgent::setup() {
    SetupEnvResponse initialInformation;

    initialInformation.set_num_state_dim(NUMBER_OF_STATE_DIM);
    initialInformation.set_num_action_dim(NUMBER_OF_ACTION_DIM);

    // Limits for actions
    // x speed
    for (int i = 0; i < representations::NaoJoints::NUM_JOINTS; i++)
        initialInformation.add_action_bound(M_PI);
    initialInformation.add_action_bound(1.05);

    return initialInformation;
}

bool KickLearningAgent::episodeOver() {
    if ((ballKicked) || robotFallen()
            || nbSteps > EPISODE_STEPS) {
        decisionMaking.movementRequest = nullptr;
        for (int i = 0; i < 80; i++ )
            step();
        wiz.setBallVelocity(Vector3<double>(0.0, 0.0, 0.0));
        return true;
    }

    return false;
}

double KickLearningAgent::reward() {

    double reward = 0.0;
    reward += 5.0*fabs(ballVelocity.x);

    reward -= 5.0*fabs(ballVelocity.y);

    reward += 5.0*fabs(ballVelocity.z);

    //reward -= 0.05 * nbSteps * 0.02;

    //double refMotionError = getReferenceMotionError();
    //reward -= min(refMotionError, MAX_JOINTS_ERROR);

    return reward;
}

double KickLearningAgent::getReferenceMotionError() {
    double error = 0;
    int timestep = nbSteps - 1;
    if (referenceMotionMap.find(timestep) == referenceMotionMap.end()) {
        LOG(INFO) << "No reference motion in timestep" << timestep << endl;
        return lastRefError;
    }

    for (int i = 0; i < representations::NaoJoints::NUM_JOINTS; i++) {
        if (referenceMotionMap[timestep].size() <= i) continue; //sanity check
        error += jointsErrorWeights[i] * fabs(desiredJoints[i] - referenceMotionMap[timestep][i]);
        LOG(INFO) << "Joint Weight | Desired | Reference " << jointsErrorWeights[i] << " | "
                << desiredJoints[i] << " | " << referenceMotionMap[timestep][i];
    }

    if (isnan(error)) {
        LOG(INFO) << "NAN DETECTED!";
        for (int i = 0; i < referenceMotionMap.size(); i++) {
            for (int j = 0; j < referenceMotionMap[i].size(); j++) {
                LOG(INFO) << referenceMotionMap[i][j];
            }
        }
        return 0.0;
    }
    lastRefError = error;
    return error;
}


State KickLearningAgent::state() const {
    State st;
    st.add_observation(nbSteps * STEP_TIME);

    return st;
}

void KickLearningAgent::stepControl() {
    control.control(perception, desiredJoints);
    action.act(decisionMaking, control);
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
}

void KickLearningAgent::step() {
    control.control(perception, modeling, decisionMaking);
    action.act(decisionMaking, control);
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
    modeling.model(perception, control);
}

void KickLearningAgent::look() {

    if (modeling.getWorldModel().getBall().isRelevant()) {
        /*behaviorFactory.getActiveVision().behave(modeling);
        decisionMaking.lookRequest =
                behaviorFactory.getActiveVision().getLookRequest();
        */
        behaviorFactory.getFocusBall().behave(modeling);
        decisionMaking.lookRequest =
                behaviorFactory.getFocusBall().getLookRequest();
    } else {
        behaviorFactory.getFocusBall().behave(modeling);
        decisionMaking.lookRequest =
                behaviorFactory.getFocusBall().getLookRequest();
        decisionMaking.movementRequest = std::make_shared<control::WalkRequest>(
                Pose2D(8.72664625997165 / 4, 0, 0));
    }

}

void KickLearningAgent::kick() {
    behaviorFactory.getKickBehavior().setTarget(Vector2<double>(
            modeling.getWorldModel().getTheirGoalPosition().x,
            modeling.getWorldModel().getTheirGoalPosition().y));
    behaviorFactory.getKickBehavior().behave(modeling);
    decisionMaking.movementRequest =
            behaviorFactory.getKickBehavior().getMovementRequest();
}

void KickLearningAgent::setAgentRandomPosition() {
    auto generator = itandroids_lib::math::MathUtils::getUniformRandomGenerator(-1.0, 1.0);

    double x = generator();
    double y = generator();

    wiz.setAgentPositionAndDirection(
            agentNumber,
            representations::PlaySide::LEFT,
            Vector3<double>(INITIAL_BALL_POS.x + x, INITIAL_BALL_POS.y + y, 0.3),
            0.0);
}