#ifndef ITANDROIDS_SOCCER3D_CPP_LEARNINGCONSTANTS_H
#define ITANDROIDS_SOCCER3D_CPP_LEARNINGCONSTANTS_H


#include <math.h>

class LearningConstants {
public:
    // Robot
    static constexpr double MAX_X_SPEED = 0.9;
    static constexpr double MAX_Y_SPEED = 0.4;
    static constexpr double MAX_THETA_SPEED = 300.0 * M_PI / 180.0;;

    // Server
    static constexpr double SERVER_STEP_TIME =  0.020;

    // RL
    static constexpr int NUM_STEP_SAME_INPUT = 3;

};


#endif //ITANDROIDS_SOCCER3D_CPP_LEARNINGCONSTANTS_H
