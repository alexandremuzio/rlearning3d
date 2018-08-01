//
// Created by muzio on 3/3/18.
//

#ifndef ITANDROIDS_SOCCER3D_CPP_FEATUREEXTRACTOR_H
#define ITANDROIDS_SOCCER3D_CPP_FEATUREEXTRACTOR_H

#include "math/Pose2D.h"
#include "core/utils/wizard/Wizard.h"

using itandroids_lib::math::Pose2D;
using itandroids_lib::math::Vector3;
using itandroids_lib::math::Vector2;


class FeatureExtractor {

public:
    FeatureExtractor(utils::wizard::Wizard &wizard, double initialX, double initialY, double initialTheta);

    // Computes all the features from the current timestep.
    // Should only be called once per timestep.
    void extractFeatures();

    // Clear all features
    void reset();

    // Access API
    // agentNum is the agent number [1,...]
    // t is the timestep that you want the feature in. I.e. t = -2 means you want 2 timesteps in the past
    double agentAngle(int agentNum = 1, int t = 0);
    double oppAngle(int agentNum = 1, int t = 0);
    Vector3<double> agentPos(int agentNum = 1, int t = 0);
    Vector3<double> oppPos(int agentNum = 1, int t = 0);
    Pose2D agentSpeed(int agentNum = 1, int t = 0);
    Pose2D oppSpeed(int agentNum = 1, int t = 0);
    bool agentFallen(int agentNum = 1, int t = 0);
    bool oppFallen(int agentNum = 1, int t = 0);
    Vector3<double> ballPos(int t = 0);
    Vector2<double> ballSpeed(int t = 0);

    double distanceAgentToBall(int agentNum = 1, int t = 0);
    double distanceOppToBall(int agentNum =1 , int t = 0);

    double ballKickable(int agentNum = 1, int t = 0);


private:
    double calcAgentAngle(int idx = 0);
    Vector3<double> calcAgentPos(int idx = 0);
    Pose2D calcAgentSpeed(int idx = 0);
    Vector3<double> calcBallPos();
    Vector2<double> calcBallSpeed();

    utils::wizard::Wizard& wiz;

    const double COORD_REF_X = 0.0;
    const double COORD_REF_Y = 0.0;
    const double COORD_REF_THETA = 0.0;
    const static int NUM_OF_AGENTS = 22;

    const double LOW_PASS_FILTER_ALPHA = 0.5;

    int timestep;

    // Agents
    Vector3<double> agentPos_[NUM_OF_AGENTS];
    Vector3<double> lastAgentPos_[NUM_OF_AGENTS];
    double agentAngle_[NUM_OF_AGENTS];
    double lastAgentAngle_[NUM_OF_AGENTS];

    Pose2D agentSpeed_[NUM_OF_AGENTS];
    Pose2D lastAgentSpeed_[NUM_OF_AGENTS];

    // Ball
    Vector3<double> ballPos_;
    Vector3<double> lastBallPos_;

    Vector2<double> ballSpeed_;
    Vector2<double> lastBallSpeed_;
};


#endif //ITANDROIDS_SOCCER3D_CPP_FEATUREEXTRACTOR_H
