#ifndef ITANDROIDS_SOCCER3D_CPP_DUMMYAGENT_H
#define ITANDROIDS_SOCCER3D_CPP_DUMMYAGENT_H

#include "action/Action.h"
#include "control/ControlStub.h"
#include "communication/Communication.h"
#include "decision_making/DecisionMakingStub.h"
#include "utils/wizard/Wizard.h"

class DummyAgent {
public:
    DummyAgent(int agentNumber = 1, std::string teamName = std::string("Opponent"));

    ~DummyAgent();

    void run();

protected:
    const int AGENT_NUMBER;

    communication::Communication communication;
//    communication::Communication wizCommunication;
    decision_making::behavior::BehaviorFactory behaviorFactory;
    action::ActionImpl action;
//    utils::wizard::Wizard wiz;
    decision_making::DecisionMakingStub decisionMaking;
    control::ControlImpl control;
    perception::Perception perception;
    modeling::Modeling modeling;
};

#endif //ITANDROIDS_SOCCER3D_CPP_DUMMYAGENT_H
