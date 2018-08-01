#include "FeatureExtractor.h"
#include "tools/rlearning3d/utils/LearningConstants.h"
#include "external/easylogging++.h"

FeatureExtractor::FeatureExtractor(utils::wizard::Wizard &wizard, double initialX, double initialY,
                                   double initialTheta) : wiz(wizard),
                                                          COORD_REF_X(initialX),
                                                          COORD_REF_Y(initialY),
                                                          COORD_REF_THETA(initialTheta) {
    reset();
}

void FeatureExtractor::extractFeatures() {
    timestep++;

    // Update features form previous timestep
    for (int i = 0; i < NUM_OF_AGENTS; i++) {
        lastAgentPos_[i] = agentPos_[i];
        lastAgentAngle_[i] = agentAngle_[i];
        lastAgentSpeed_[i] = agentSpeed_[i];
    }
    lastBallPos_ = ballPos_;
    lastBallSpeed_ = ballSpeed_;


    // Update features
    for (int i = 0; i < NUM_OF_AGENTS; i++) {
        agentPos_[i] = calcAgentPos(i);
        agentAngle_[i] = calcAgentAngle(i);
        agentSpeed_[i] = calcAgentSpeed(i);
    }
    ballPos_ = calcBallPos();
    ballSpeed_ = calcBallSpeed();
}

void FeatureExtractor::reset() {
    timestep = 0;
    for (int i = 0; i < NUM_OF_AGENTS; i++) {
        agentPos_[i] = Vector3<double>();
        lastAgentPos_[i] = Vector3<double>();
        agentAngle_[i] = 0.0;
        lastAgentAngle_[i] = 0.0;
        agentSpeed_[i] = Pose2D();
        lastAgentSpeed_[i] = Pose2D();

        ballPos_ = Vector3<double>();
        lastBallPos_ = Vector3<double>();
        ballSpeed_ = Vector2<double>();
        lastBallSpeed_ = Vector2<double>();
    }
}

double FeatureExtractor::agentAngle(int agentNum, int t) {
    if (t == 0)
        return agentAngle_[agentNum - 1];
    if (t == -1)
        return lastAgentAngle_[agentNum - 1];
}

double FeatureExtractor::oppAngle(int agentNum, int t) {
    return agentAngle(agentNum + 11, t);
}

Vector3<double> FeatureExtractor::agentPos(int agentNum, int t) {
    if (t == 0)
        return agentPos_[agentNum - 1];
    if (t == -1)
        return lastAgentPos_[agentNum - 1];
}

Vector3<double> FeatureExtractor::oppPos(int agentNum, int t) {
    return agentPos(agentNum + 11, t);
}

Pose2D FeatureExtractor::agentSpeed(int agentNum, int t) {
    if (t == 0)
        return agentSpeed_[agentNum - 1];
    if (t == -1)
        return lastAgentSpeed_[agentNum - 1];
}

Pose2D FeatureExtractor::oppSpeed(int agentNum, int t) {
    return agentSpeed(agentNum + 11, t);
}

bool FeatureExtractor::agentFallen(int agentNum, int t) {
    if (t == 0)
        return agentPos_[agentNum - 1].z < 0.2;
    else
        return agentPos_[agentNum - 1].z < 0.2;
}

bool FeatureExtractor::oppFallen(int agentNum, int t) {
    return agentFallen(agentNum + 11, t);
}

Vector3<double> FeatureExtractor::ballPos(int t) {
    if (t == 0)
        return ballPos_;
    if (t == -1)
        return lastBallPos_;
}

Vector2<double> FeatureExtractor::ballSpeed(int t) {
    if (t == 0)
        return ballSpeed_;
    if (t == -1)
        return lastBallSpeed_;
}

double FeatureExtractor::distanceAgentToBall(int agentNum, int t) {
    if (t == 0)
        return (agentPos_[agentNum - 1] - ballPos_).absXY();
    else
        return (lastAgentPos_[agentNum - 1] - lastBallPos_).absXY();
}

double FeatureExtractor::distanceOppToBall(int agentNum, int t) {
    return distanceAgentToBall(agentNum + 11, t);
}

double FeatureExtractor::ballKickable(int agentNum, int t) {
    if (t == 0)
        return distanceAgentToBall(agentNum, t) < 0.2;
    else
        return distanceAgentToBall(agentNum, -1) < 0.2;
}

// Calculations
double FeatureExtractor::calcAgentAngle(int idx) {
    double angle = wiz.getAgentRotation(idx + 1).getZAngle() - M_PI_2;

    if (std::isnan(angle))
        return 0.0;

    if (angle > M_PI)
        angle -= 2* M_PI;
    if (angle < -M_PI)
        angle += 2* M_PI;

    return angle;
}

Vector3<double> FeatureExtractor::calcAgentPos(int idx) {
    Vector3<double> position = wiz.getAgentTranslation(idx + 1);

    position.x = position.x - COORD_REF_X;
    position.y = position.y - COORD_REF_Y;
    return position;
}

Pose2D FeatureExtractor::calcAgentSpeed(int idx) {
    double den = LearningConstants::SERVER_STEP_TIME * LearningConstants::NUM_STEP_SAME_INPUT;
    double xSpeed = (agentPos_[idx] - lastAgentPos_[idx]).x / den;
    double ySpeed = (agentPos_[idx] - lastAgentPos_[idx]).y / den;
    double thetaSpeed = (agentAngle_[idx] - lastAgentAngle_[idx]) / den;

    agentSpeed_[idx].translation.x += LOW_PASS_FILTER_ALPHA * (xSpeed - agentSpeed_[idx].translation.x);
    agentSpeed_[idx].translation.y += LOW_PASS_FILTER_ALPHA * (ySpeed - agentSpeed_[idx].translation.y);
    agentSpeed_[idx].rotation += LOW_PASS_FILTER_ALPHA * (thetaSpeed - agentSpeed_[idx].rotation);
    return agentSpeed_[idx];
}

Vector3<double> FeatureExtractor::calcBallPos() {
    return wiz.getBallTranslation();
}

Vector2<double> FeatureExtractor::calcBallSpeed() {
    double den = LearningConstants::SERVER_STEP_TIME * LearningConstants::NUM_STEP_SAME_INPUT;

    double xSpeed =  (ballSpeed_ - lastBallSpeed_).x / den;
    double ySpeed =  (ballSpeed_ - lastBallSpeed_).y / den;

    ballSpeed_.x += LOW_PASS_FILTER_ALPHA * (xSpeed -  ballSpeed_.x);
    ballSpeed_.y += LOW_PASS_FILTER_ALPHA * (xSpeed -  ballSpeed_.y);

    return ballSpeed_;
}
