//
// Created by luis on 8/1/18.
//

#ifndef ITANDROIDS_SOCCER3D_CPP_BODYFEATURESUTILS_H
#define ITANDROIDS_SOCCER3D_CPP_BODYFEATURESUTILS_H

#include "core/representations/NaoJoints.h"

#include "soccer3d.grpc.pb.h"

#include<iostream>

#include<fstream>

using api::Action;

class BodyFeaturesUtils {
public:
    BodyFeaturesUtils();

    ~BodyFeaturesUtils();

    representations::NaoJoints readJointsFromFile(std::ifstream &anglesFile);

    representations::NaoJoints readAction(Action action);

    void printJoints(representations::NaoJoints &frame);

    double getJointsDiffNorm(representations::NaoJoints &frame1, representations::NaoJoints &frame2);

private:
    representations::NaoJoints jointsWeight;
};


#endif //ITANDROIDS_SOCCER3D_CPP_BODYFEATURESUTILS_H
