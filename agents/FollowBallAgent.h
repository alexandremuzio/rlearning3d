#ifndef ITANDROIDS_SOCCER3D_CPP_FOLLOWBALLAGENT_H
#define ITANDROIDS_SOCCER3D_CPP_FOLLOWBALLAGENT_H


#include "action/Action.h"
#include "control/ControlStub.h"
#include "communication/Communication.h"
#include "decision_making/DecisionMakingStub.h"
#include "utils/wizard/Wizard.h"

class FollowBallAgent {
public:
    FollowBallAgent(int agentNumber = 1, std::string teamName = std::string("Opponent"), std::string curriculumStr = std::string("off"));

    ~FollowBallAgent();

    void run();


protected:
    const int AGENT_NUMBER;
    int nSteps = 0;
    bool curriculum = false;

    communication::Communication communication;
    communication::Communication wizCommunication;
    control::ControlImpl control;
    decision_making::behavior::BehaviorFactory behaviorFactory;
    action::ActionImpl action;
    decision_making::DecisionMakingStub decisionMaking;
    modeling::Modeling modeling;
    perception::Perception perception;
    utils::wizard::Wizard wiz;
};


#endif //ITANDROIDS_SOCCER3D_CPP_FOLLOWBALLAGENT_H
