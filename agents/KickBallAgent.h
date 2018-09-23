//
// Created by luis on 7/17/18.
//

#ifndef ITANDROIDS_SOCCER3D_CPP_KICKBALLAGENT_H
#define ITANDROIDS_SOCCER3D_CPP_KICKBALLAGENT_H

#include "communication/Communication.h"
#include "utils/wizard/Wizard.h"
#include "action/Action.h"
#include "control/ControlStub.h"
#include "decision_making/DecisionMakingStub.h"
#include "utils/roboviz/Roboviz.h"
#include "utils/configure_tree/ParameterTreeCreator.h"

class KickBallAgent {
public:
    KickBallAgent(std::string host = "127.0.0.1", int agentPort = 3000, int monitorPort = 3300, int agentNumber = 1,
                  int robotType = 0, std::string teamName = std::string("ITAndroids"));

    ~KickBallAgent();

    void testLoop(int numberOfSteps);

    void printJoints(double time, representations::NaoJoints curJointsPos);

    void runStep();

    utils::wizard::Wizard wiz;

private:
    const double STEP_DT = 0.020;
    double accumulatedTime = 0.0;
    communication::Communication communication;
    communication::Communication wizCommunication;
    control::ControlStub controlStub;
    action::ActionImpl action;
    decision_making::DecisionMakingStub decisionMaking;
    modeling::Modeling modeling;
    perception::Perception perception;
    control::kick::KickZMPParams kickParams;
    control::kick::KickZMP kick;
    control::walking::InverseKinematics inverseKinematics;
    control::JointsController controller;
    representations::NaoJoints jointsTargets;
    bool finishedKick;

    std::ofstream anglesFile;
    // Runs simulation step in the environment
    void step();
};


#endif //ITANDROIDS_SOCCER3D_CPP_KICKBALLAGENT_H
