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
    BodyFeaturesUtils(int serverPort);

    ~BodyFeaturesUtils();

    representations::NaoJoints readJointsFromFile(std::ifstream &anglesFile);

    representations::NaoJoints readAction(Action action);

    void printJoints(representations::NaoJoints &frame);

    bool initialPosSanityCheck(const representations::NaoJoints &frame);

    double getJointsDiffNorm(representations::NaoJoints &frame1, representations::NaoJoints &frame2);

    void printJoints(representations::NaoJoints &frame1, representations::NaoJoints &fram2);

    std::vector<representations::NaoJoints> preProcessFile(std::ifstream &anglesFile);

private:
    representations::NaoJoints jointsWeight;
    std::vector<std::string> jointsToPrint;
    std::vector<std::ofstream> filesToPrint;
};


#endif //ITANDROIDS_SOCCER3D_CPP_BODYFEATURESUTILS_H
