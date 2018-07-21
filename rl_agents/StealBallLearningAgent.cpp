#include "StealBallLearningAgent.h"

#include "EnvironmentUtils.h"
#include "LearningConstants.h"
#include "external/easylogging++.h"

using itandroids_lib::math::sgn;

// Constants
// Robot
const double MAX_X_SPEED = 0.9;
const double MAX_Y_SPEED = 0.4;
const double MAX_W_SPEED = 300 * M_PI / 180;

// Environment
const double MAX_FIELD_X = 4;
const double MAX_FIELD_Y = 3;

const double AGENT_MIN_X = 1.5;
const double AGENT_MAX_X = 3.2;
const double AGENT_MIN_Y = 1.5;
const double AGENT_MAX_Y = 2.5;
const double AGENT_MAX_THETA = M_PI;

// RL
const int NUMBER_OF_STATE_DIM = 9;
const int NUMBER_OF_ACTION_DIM = 3;
const int NUMBER_OF_STEPS_PER_EPISODE = 5000 * LearningConstants::NUM_STEP_SAME_INPUT;


// Helper utils methods
// TODO add this to a drawing class.
void StealBallLearningAgent::drawEnvironment() {
    std::string selfStr("pos.field");
    roboviz->drawLine(-MAX_FIELD_X, -MAX_FIELD_Y, 0, -MAX_FIELD_X, MAX_FIELD_Y, 0, 5, 1, 1, 0, &selfStr);
    roboviz->drawLine(MAX_FIELD_X, -MAX_FIELD_Y, 0, MAX_FIELD_X, MAX_FIELD_Y, 0, 5, 1, 1, 0, &selfStr);
    roboviz->drawLine(-MAX_FIELD_X, -MAX_FIELD_Y, 0, MAX_FIELD_X, -MAX_FIELD_Y, 0, 5, 1, 1, 0, &selfStr);
    roboviz->drawLine(-MAX_FIELD_X, MAX_FIELD_Y, 0, MAX_FIELD_X, MAX_FIELD_Y,0, 5, 1, 1, 0, &selfStr);

    double angle = featEx->agentAngle();
    roboviz->drawLine(featEx->agentPos().x, featEx->agentPos().y, 0,
                      featEx->agentPos().x + cos(angle), featEx->agentPos().y + sin(angle), 0,
                     5, 0, 0.2, 1, &selfStr);

    double oppAngle = featEx->oppAngle();
    roboviz->drawLine(featEx->oppPos().x, featEx->oppPos().y, 0,
                      featEx->oppPos().x + cos(oppAngle), featEx->oppPos().y + sin(oppAngle), 0,
                     5, 1, 0.2, 0, &selfStr);

    std::string buffer("");
    roboviz->swapBuffers(&buffer);
}
void StealBallLearningAgent::drawStats() {
    string stats = "stats";
    string episodeString = "Episode: " + to_string(iEpi);
    string stepsString = "Number of steps: " + to_string(nbEpisodeSteps);
    string rewardString = "Reward: " + to_string(currReward);

    string agentPositionStr =
            "Curr Pose: (" + to_string(featEx->agentPos().x) + ", " + to_string(featEx->agentPos().y) + ", " +
            to_string(
                    featEx->agentAngle() * 180 / M_PI) + ")";
    string agentSpeedStr = "Curr Speed: (" + to_string(featEx->agentSpeed().translation.x) + ", " +
                           to_string(featEx->agentSpeed().translation.y) + ", " + to_string(
            featEx->agentSpeed().rotation) + ")";
    string actionSpeedString =
            "Action Speed: (" + to_string(commandedXSpeed) + ", " + to_string(commandedYSpeed) + ", " + to_string(
                    commandedThetaSpeed) + ")";
    string oppPositionStr =
            "Opp Pose: (" + to_string(featEx->oppPos().x) + ", " + to_string(featEx->oppPos().y) + ", " +
            to_string(
                    featEx->oppAngle() * 180 / M_PI) + ")";
    string ballPosStr =
            "Ball Pos: (" + to_string(featEx->ballPos().x) + ", " + to_string(featEx->ballPos().y);
    string ballSpeedStr =
            "Ball Speed: (" + to_string(featEx->ballSpeed().x) + ", " + to_string(featEx->ballSpeed().y);
    string distBallStr = "Dist to Ball: " + to_string(featEx->distanceAgentToBall());
    string distOppBallStr = "Dist to Ball: " + to_string(featEx->distanceOppToBall());

    string epiString = stats + ".epi";
    roboviz->drawAnnotation(&episodeString, 0, 0.3, 0.0, 0.0, 0.0, 1.0, &epiString);
    roboviz->drawAnnotation(&stepsString, 0, 0.0, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&rewardString, 0, -0.3, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&agentPositionStr, 0, -0.7, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&agentSpeedStr, 0, -1.1, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&actionSpeedString, 0, -1.5, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&oppPositionStr, 0, -1.9, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&ballPosStr, 0, -2.3, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&ballSpeedStr, 0, -2.7, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&distBallStr, 0, -3.1, 0.0, 0.0, 0.0, 1.0, &stats);
    roboviz->drawAnnotation(&distOppBallStr, 0, -3.5, 0.0, 0.0, 0.0, 1.0, &stats);

    std::string buffer(stats);
    roboviz->swapBuffers(&buffer);
}
////////////////////////

StealBallLearningAgent::StealBallLearningAgent(std::string host, int serverPort, int monitorPort, int agentNumber,
                                         int robotType, std::string teamName) 
                                         : BaseLearningAgent(host, serverPort, monitorPort, agentNumber, robotType, teamName) {
    featEx = make_unique<FeatureExtractor>(wiz, 0.0, 0.0, 0.0);

    std::ifstream config_doc("config/steal_ball.json");
    config_doc >> config;
}


State StealBallLearningAgent::newEpisode() {
    LOG(INFO) << "##############################################";
    LOG(INFO) << "Starting Episode: " << ++iEpi;

    featEx = std::make_unique<FeatureExtractor>(wiz, 0.0, 0.0, 0.0);
    nbEpisodeSteps = 0;

    double curriculumScalar = 0;
    if (nbTotalSteps < 300000)
        curriculumScalar = 0.2;
    else if (nbTotalSteps < 600000)
        curriculumScalar = 0.6;
    else if (nbTotalSteps < 1000000)
        curriculumScalar = 1.0;
    else
        curriculumScalar = 1.2;
    Vector2<double> selfPos = EnvironmentUtils::getRandomPointInRectangle(0.0, 0.0, curriculumScalar * AGENT_MAX_X, curriculumScalar * AGENT_MAX_Y,
                                                                          curriculumScalar * AGENT_MIN_X, curriculumScalar * AGENT_MIN_Y);
    double selfTheta = EnvironmentUtils::getRandomAngle(AGENT_MAX_THETA);

    Vector2<double> oppPos = EnvironmentUtils::getRandomPointInRectangle(0.0, 0.0, 4, 3, 2, 1.5);

    Vector2<double> ballPos = EnvironmentUtils::getRandomPointInCircle(0.0, 0.0, 1.0);

    double oppAngle = (ballPos - oppPos).angle();

    LOG(INFO) << "Robot starting position: " << selfPos.x << " " << selfPos.y << " " << selfTheta;
    LOG(INFO) << "Opp starting position: " << oppPos.x << " " << oppPos.y << " " << 0.0;


    // TODO Remove. This should be in an environment specific class.
    // starting position
    wiz.setPlayMode(representations::RawPlayMode::PLAY_ON);
    wiz.setAgentPositionAndDirection(agentNumber, representations::PlaySide::LEFT,
                                     Vector3<double>(selfPos.x, selfPos.y, 0.3), selfTheta);
    wiz.setAgentPositionAndDirection(1, representations::PlaySide::RIGHT,
                                     Vector3<double>(oppPos.x, oppPos.y, 0.3), oppAngle);

    wiz.setBallPosition(Vector3<double>(ballPos.x, ballPos.y, 0.0));

    decisionMaking.movementRequest = std::make_shared<control::WalkRequest>(Vector3<double>(0.0, 0.0, 0.0));
    for (int i =0; i < 30; i++)
        step();
    featEx->extractFeatures();

    return state();
}

SimulationResponse StealBallLearningAgent::runStep(Action action) {
    LOG(INFO) << "#######################";
    LOG(INFO) << "Step " << nbEpisodeSteps;
    LOG(INFO) << "AgentPos: " << featEx->agentPos() << " " << featEx->agentAngle();
    LOG(INFO) << "AgentLastPos: " << featEx->agentPos(1, -1) << " " << featEx->agentAngle(1, -1);
    LOG(INFO) << "OppPos: " << featEx->oppPos() << " " << featEx->oppAngle();
    LOG(INFO) << "Agent Speed: " << featEx->agentSpeed().translation.x << " "
              << featEx->agentSpeed().translation.y
              << " " << featEx->agentSpeed().rotation;
    LOG(INFO) << "Commanded Speed: " << commandedXSpeed << " "<< commandedYSpeed << " " << commandedThetaSpeed;
    LOG(INFO) << "BallPos: " << featEx->ballPos();
    LOG(INFO) << "BallSpeed: " << featEx->ballPos();

    if (nbTotalSteps % 100000 == 0)
        LOG(DEBUG) << "Total N Steps: " << nbTotalSteps;

    commandedXSpeed = action.action(0);
    commandedYSpeed= action.action(1);
    commandedThetaSpeed= action.action(2);

    //Execute action from server
    decisionMaking.movementRequest = std::make_shared<control::WalkRequest>(Vector3<double>(commandedXSpeed,
                                                                                            commandedYSpeed,
                                                                                            commandedThetaSpeed));

    ////////////////////////
    // SOCCER 3D SIMULATION STEP
    // Uses same input for some steps
    for (int i = 0; i < LearningConstants::NUM_STEP_SAME_INPUT; i++) {
        step();
        drawEnvironment();
        drawStats();
    }

    Vector3<double> oppPos = featEx->oppPos();
    if (featEx->oppFallen() || !EnvironmentUtils::pointInsideArea(oppPos.x, oppPos.y,
                                                                  1.2 * MAX_FIELD_X, 1.2 * MAX_FIELD_Y)) {
        wiz.setAgentPositionAndDirection(1, representations::PlaySide::RIGHT,
                                         Vector3<double>(sgn(oppPos.x) * MAX_FIELD_X, sgn(oppPos.y) * MAX_FIELD_Y, 0.3),
                                         (sgn(oppPos.x) > 0)? 180 : 0.0);
    }

    featEx->extractFeatures();
    currReward = reward();
    nbEpisodeSteps++;
    nbTotalSteps++;
    ////////////////////////

    SimulationResponse stepUpdate;
    stepUpdate.mutable_state()->CopyFrom(state());
    stepUpdate.set_reward(currReward);
    stepUpdate.set_done(episodeOver());

    return stepUpdate;
}

SetupEnvResponse StealBallLearningAgent::setup() {
    SetupEnvResponse initialInformation;

    wiz.setPlayMode(representations::RawPlayMode::PLAY_ON);

    initialInformation.set_num_state_dim(NUMBER_OF_STATE_DIM);
    initialInformation.set_num_action_dim(NUMBER_OF_ACTION_DIM);

    // Limits for actions
    // x speed
    initialInformation.add_action_bound(MAX_X_SPEED);

    // y speed
    initialInformation.add_action_bound(MAX_Y_SPEED);

    // angular speed
    initialInformation.add_action_bound(MAX_W_SPEED);

    return initialInformation;
}

bool StealBallLearningAgent::episodeOver() {
    Vector3<double> robotPos = featEx->agentPos();
    Vector3<double> ballPos = featEx->ballPos(0);

    if (featEx->agentFallen()) {
        LOG(INFO) << "Episode finishing because robot fell";
        return true;
    }
    //if (featEx->oppFallen()) {
    //    LOG(INFO) << "Episode finishing because opponent fell";
    //    return true;
    //}
    if (!EnvironmentUtils::pointInsideArea(robotPos.x, robotPos.y,
                                           MAX_FIELD_X, MAX_FIELD_Y)) {
        LOG(INFO) << "Episode finishing because robot left the area";
        return true;
    }
    if (!EnvironmentUtils::pointInsideArea(ballPos.x, ballPos.y,
                                           MAX_FIELD_X, MAX_FIELD_Y)) {

        if (ballPos.x > MAX_FIELD_X)
            LOG(INFO) << "PLAYER DRIBBLED";
        if (ballPos.x < -MAX_FIELD_X)
            LOG(INFO) << "OPPONENT DRIBBLED";

        LOG(INFO) << "Episode finishing because ball left the area";
        return true;
    }

    if (nbEpisodeSteps >= NUMBER_OF_STEPS_PER_EPISODE) {
        LOG(INFO) << "Episode finishing because of number of steps";
        return true;
    }

    return false;
}

double StealBallLearningAgent::reward() {
    double lastDistSelfToBall = featEx->distanceAgentToBall(1, -1);
    double distSelfToBall = featEx->distanceAgentToBall(1);
    double angleSelfToBall = (featEx->ballPos() - featEx->agentPos()).getAlpha();
    double lastDistOppToBall = featEx->distanceOppToBall(1, -1);
    double distOppToBall = featEx->distanceOppToBall(1);

    double controlIntensity = Vector3<double>(commandedXSpeed,
                                              commandedYSpeed,
                                              commandedThetaSpeed).squareAbs();

    double deltaXBall = featEx->ballPos().x - featEx->ballPos(-1).x;

    bool ballWasKickableLastStep = featEx->ballKickable(1, -1);

    double reward = 0.0;


    //std::this_thread::sleep_for(std::chrono::milliseconds(200));

    LOG(INFO) << "DIST SELF-BALL " << lastDistSelfToBall - distSelfToBall;
    LOG(INFO) << "DIST OPP-BALL " << (lastDistOppToBall - distOppToBall);
    LOG(INFO) << "DELTA X BALL " << deltaXBall;
    LOG(INFO) << "CONTROL INTENSITY" << controlIntensity;
    LOG(INFO) << "NEGATIVE SPEED PEN" << max(fabs(commandedXSpeed) - MAX_X_SPEED, 0.0);
    LOG(INFO) << "COMMANDED SPEED PEN " <<  max(fabs(commandedXSpeed) - MAX_X_SPEED, 0.0);

    reward += (lastDistSelfToBall - distSelfToBall);
    reward += 0.05 * exp(-distSelfToBall);

    if (featEx->distanceAgentToBall() < 0.75)
        reward += -(lastDistOppToBall - distOppToBall);

    reward += 5 * deltaXBall;
    //reward += -0.05 *  max(fabs(commandedXSpeed) - MAX_X_SPEED, 0.0);
    //reward += -0.005 * controlIntensity;
    reward += 0.1 * min(commandedXSpeed, 0.0);

    if (featEx->ballPos().x > MAX_FIELD_X)
        reward += 10;

    if (featEx->agentFallen())
        reward += -1;

    // TODO This should actually test fouls
    if (config["r"]["f_penalize_opp_fall"].asBool()
        && featEx->oppFallen())
        reward += -5;

    if (config["r"]["f_use_kickable"].asBool()
        && featEx->ballKickable(1))
        reward += 0.1;

    LOG(INFO) << "Reward: " << reward;

    //LOG(DEBUG) << modeling.getAgentModel().getAgentPerception().getRightFootForceResistanceData().getForce().z;
    return reward;
}

State StealBallLearningAgent::state() {
    State st;

    double selfAngle = featEx->agentAngle();
    Vector3<double> relToSelfBallPos = featEx->ballPos() - featEx->agentPos();
    Vector3<double> relToSelfOppPos = featEx->oppPos() - featEx->agentPos();

    Vector3<double> selfAngleVector = Vector3<double>(cos(selfAngle), sin(selfAngle), 0.0);
    double relToSelfBallAngle = (selfAngleVector - relToSelfBallPos).getAlpha();
    double relToSelfOppAngle = (selfAngleVector - relToSelfOppPos).getAlpha();

    double distSelfToBall = relToSelfOppPos.absXY();
    double distOppToBall = (featEx->oppPos() - featEx->ballPos(0)).absXY();

    st.add_observation(selfAngle);
    st.add_observation(relToSelfBallPos.x);
    st.add_observation(relToSelfBallPos.y);
    st.add_observation(relToSelfBallAngle);
    st.add_observation(relToSelfOppPos.x);
    st.add_observation(relToSelfOppPos.y);
    st.add_observation(relToSelfOppAngle);
    st.add_observation(distSelfToBall);
    st.add_observation(distOppToBall);

    //st.add_observation(modeling.getAgentModel().getAgentPerception().getRightFootForceResistanceData().getForce().z);
    //st.add_observation(modeling.getAgentModel().getAgentPerception().getLeftFootForceResistanceData().getForce().z);

    return st;
}

void StealBallLearningAgent::step() {
    control.control(perception, modeling, decisionMaking);
    action.act(decisionMaking, control);
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
//    modeling.model(perception, control);
}
