//
// Created by luis on 8/1/18.
//

#include <tools/rlearning3d/external/easylogging++.h>
#include "BodyFeaturesUtils.h"
#include "math/MathUtils.h"
#include "string.h"

BodyFeaturesUtils::BodyFeaturesUtils() {
    jointsToPrint = {"rightFootRoll","rightFootPitch","rightKneePitch","rightHipPitch","rightHipRoll","rightHipYawPitch"};
    filesToPrint.resize(jointsToPrint.size());
    std::string prefix = "../../plots/";
    for(int i = 0; i < jointsToPrint.size(); i++)
        filesToPrint[i].open(prefix+jointsToPrint[i]+".txt");

    jointsWeight.neckPitch = 1;
    jointsWeight.neckYaw = 1;
    jointsWeight.leftShoulderPitch = 1;
    jointsWeight.leftShoulderYaw = 1;
    jointsWeight.leftArmRoll = 1;
    jointsWeight.leftArmYaw = 1;
    jointsWeight.rightShoulderPitch = 1;
    jointsWeight.rightShoulderYaw = 1;
    jointsWeight.rightArmRoll = 1;
    jointsWeight.rightArmYaw = 1;
    jointsWeight.leftHipYawPitch = 2;
    jointsWeight.leftHipRoll = 2;
    jointsWeight.leftHipPitch = 2;
    jointsWeight.leftKneePitch = 2;
    jointsWeight.leftFootPitch = 2;
    jointsWeight.leftFootRoll = 2;
    jointsWeight.rightHipYawPitch = 3;
    jointsWeight.rightHipRoll = 3;
    jointsWeight.rightHipPitch = 3;
    jointsWeight.rightKneePitch = 3;
    jointsWeight.rightFootPitch = 3;
    jointsWeight.rightFootRoll = 3;
}

BodyFeaturesUtils::~BodyFeaturesUtils() {
    for(int i = 0; i < filesToPrint.size(); i++)
        filesToPrint[i].close();
}

representations::NaoJoints BodyFeaturesUtils::readJointsFromFile(std::ifstream &anglesFile){
    representations::NaoJoints referenceFrame;

    anglesFile >> referenceFrame.neckPitch;
    anglesFile >> referenceFrame.neckYaw;

    anglesFile >> referenceFrame.leftShoulderPitch;
    anglesFile >> referenceFrame.leftShoulderYaw;
    anglesFile >> referenceFrame.leftArmRoll;
    anglesFile >> referenceFrame.leftArmYaw;

    anglesFile >> referenceFrame.rightShoulderPitch;
    anglesFile >> referenceFrame.rightShoulderYaw;
    anglesFile >> referenceFrame.rightArmRoll;
    anglesFile >> referenceFrame.rightArmYaw;

    anglesFile >> referenceFrame.leftHipYawPitch;
    anglesFile >> referenceFrame.leftHipRoll;
    anglesFile >> referenceFrame.leftHipPitch;
    anglesFile >> referenceFrame.leftKneePitch;
    anglesFile >> referenceFrame.leftFootPitch;
    anglesFile >> referenceFrame.leftFootRoll;

    anglesFile >> referenceFrame.rightHipYawPitch;
    anglesFile >> referenceFrame.rightHipRoll;
    anglesFile >> referenceFrame.rightHipPitch;
    anglesFile >> referenceFrame.rightKneePitch;
    anglesFile >> referenceFrame.rightFootPitch;
    anglesFile >> referenceFrame.rightFootRoll;

    return referenceFrame;
}

representations::NaoJoints BodyFeaturesUtils::readAction(Action action){
    representations::NaoJoints actionFrame;

    actionFrame.neckPitch = action.action(0);
    actionFrame.neckYaw = action.action(1);
    actionFrame.leftShoulderPitch = action.action(2);
    actionFrame.leftShoulderYaw = action.action(3);
    actionFrame.leftArmRoll = action.action(4);
    actionFrame.leftArmYaw = action.action(5);
    actionFrame.rightShoulderPitch = action.action(6);
    actionFrame.rightShoulderYaw = action.action(7);
    actionFrame.rightArmRoll = action.action(8);
    actionFrame.rightArmYaw = action.action(9);
    actionFrame.leftHipYawPitch = action.action(10);
    actionFrame.leftHipRoll = action.action(11);
    actionFrame.leftHipPitch = action.action(12);
    actionFrame.leftKneePitch = action.action(13);
    actionFrame.leftFootPitch = action.action(14);
    actionFrame.leftFootRoll = action.action(15);
    actionFrame.rightHipYawPitch = action.action(16);
    actionFrame.rightHipRoll = action.action(17);
    actionFrame.rightHipPitch = action.action(18);
    actionFrame.rightKneePitch = action.action(19);
    actionFrame.rightFootPitch = action.action(20);
    actionFrame.rightFootRoll = action.action(21);

    return actionFrame;
}

void BodyFeaturesUtils::printJoints(representations::NaoJoints &frame){
    LOG(INFO) << frame.neckPitch << " ";
    LOG(INFO) << frame.neckYaw << " ";

    LOG(INFO) << frame.leftShoulderPitch << " ";
    LOG(INFO) << frame.leftShoulderYaw << " ";
    LOG(INFO) << frame.leftArmRoll << " ";
    LOG(INFO) << frame.leftArmYaw << " ";

    LOG(INFO) << frame.rightShoulderPitch << " ";
    LOG(INFO) << frame.rightShoulderYaw << " ";
    LOG(INFO) << frame.rightArmRoll << " ";
    LOG(INFO) << frame.rightArmYaw << " ";

    LOG(INFO) << frame.leftHipYawPitch << " ";
    LOG(INFO) << frame.leftHipRoll << " ";
    LOG(INFO) << frame.leftHipPitch << " ";
    LOG(INFO) << frame.leftKneePitch << " ";
    LOG(INFO) << frame.leftFootPitch << " ";
    LOG(INFO) << frame.leftFootRoll << " ";

    LOG(INFO) << frame.rightHipYawPitch << " ";
    LOG(INFO) << frame.rightHipRoll << " ";
    LOG(INFO) << frame.rightHipPitch << " ";
    LOG(INFO) << frame.rightKneePitch << " ";
    LOG(INFO) << frame.rightFootPitch << " ";
    LOG(INFO) << frame.rightFootRoll << std::endl;
}

double BodyFeaturesUtils::getJointsDiffNorm(representations::NaoJoints &frame1, representations::NaoJoints &frame2){
    using namespace itandroids_lib::math;

    double norm = 0;
    representations::NaoJoints frameDiff = frame1 - frame2;

    norm += fabs(MathUtils::normalizeAngle(frameDiff.neckPitch)) * jointsWeight.neckPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.neckYaw)) * jointsWeight.neckYaw;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftShoulderPitch)) * jointsWeight.leftShoulderPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftShoulderYaw)) * jointsWeight.leftShoulderYaw;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftArmRoll)) * jointsWeight.leftArmRoll;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftArmYaw)) * jointsWeight.leftArmYaw;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightShoulderPitch)) * jointsWeight.rightShoulderPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightShoulderYaw)) * jointsWeight.rightShoulderYaw;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightArmRoll)) * jointsWeight.rightArmRoll;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightArmYaw)) * jointsWeight.rightArmYaw;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftHipYawPitch)) * jointsWeight.leftHipYawPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftHipRoll)) * jointsWeight.leftHipRoll;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftHipPitch)) * jointsWeight.leftHipPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftKneePitch)) * jointsWeight.leftKneePitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftFootPitch)) * jointsWeight.leftFootPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.leftFootRoll)) * jointsWeight.leftFootRoll;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightHipYawPitch)) * jointsWeight.rightHipYawPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightHipRoll)) * jointsWeight.rightHipRoll;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightHipPitch)) * jointsWeight.rightHipPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightKneePitch)) * jointsWeight.rightKneePitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightFootPitch)) * jointsWeight.rightFootPitch;
    norm += fabs(MathUtils::normalizeAngle(frameDiff.rightFootRoll)) * jointsWeight.rightFootRoll;

    return norm;
}

bool BodyFeaturesUtils::initialPosSanityCheck(const representations::NaoJoints &frame) {
    if( (frame.neckPitch > 0.001)
        || (frame.neckYaw > 0.001)
        || (frame.leftShoulderPitch > 0.001)
        || (frame.leftShoulderYaw > 0.001)
        || (frame.leftArmRoll > 0.001)
        || (frame.leftArmYaw > 0.001)
        || (frame.rightShoulderPitch > 0.001)
        || (frame.rightShoulderYaw > 0.001)
        || (frame.rightArmRoll > 0.001)
        || (frame.rightArmYaw > 0.001)
        || (frame.leftHipYawPitch > 0.001)
        || (frame.leftHipRoll > 0.001)
        || (frame.leftHipPitch > 0.001)
        || (frame.leftKneePitch > 0.001)
        || (frame.leftFootPitch > 0.001)
        || (frame.leftFootRoll > 0.001)
        || (frame.rightHipYawPitch > 0.001)
        || (frame.rightHipRoll > 0.001)
        || (frame.rightHipPitch > 0.001)
        || (frame.rightKneePitch > 0.001)
        || (frame.rightFootPitch > 0.001)
        || (frame.rightFootRoll > 0.001) )
        return false;
    return true;
}

std::unordered_map<std::string,double> createJointsMap(representations::NaoJoints &frame) {
    std::unordered_map<std::string,double> frameMap;
    frameMap["rightFootRoll"] = frame.rightFootRoll;
    frameMap["rightFootPitch"] = frame.rightFootPitch;
    frameMap["rightKneePitch"] = frame.rightKneePitch;
    frameMap["rightHipPitch"] = frame.rightHipPitch;
    frameMap["rightHipRoll"] = frame.rightHipRoll;
    frameMap["rightHipYawPitch"] = frame.rightHipYawPitch;
    return frameMap;
}

void BodyFeaturesUtils::printJoints(representations::NaoJoints &frame1, representations::NaoJoints &frame2) {
    auto frameMap1 = createJointsMap(frame1);
    auto frameMap2 = createJointsMap(frame2);
    for(int i = 0; i < jointsToPrint.size(); i++) {
        filesToPrint[i] << frameMap1[jointsToPrint[i]] << " " << frameMap2[jointsToPrint[i]] << std::endl;
    }

}
