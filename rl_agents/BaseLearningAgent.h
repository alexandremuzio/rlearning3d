#ifndef ITANDROIDS_SOCCER3D_CPP_BASELEARNINGAGENT_H
#define ITANDROIDS_SOCCER3D_CPP_BASELEARNINGAGENT_H

#include "tools/rlearning3d/utils/FeatureExtractor.h"
#include "soccer3d.grpc.pb.h"

#include "action/Action.h"
#include "communication/Communication.h"
#include "decision_making/DecisionMakingStub.h"
#include "core/utils/configure_tree/ParameterTreeCreator.h"
#include "core/utils/roboviz/Roboviz.h"
#include "core/utils/wizard/Wizard.h"

using api::Action;
using api::SetupEnvResponse;
using api::SimulationResponse;
using api::State;

class BaseLearningAgent {
public:
    BaseLearningAgent(std::string host = "127.0.0.1", int serverPort = 3100, int monitorPort = 3200, int agentNumber = 1,
                  int robotType = 0, std::string teamName = std::string("ITAndroids"));

    virtual ~BaseLearningAgent();

    /**
     * Method that setups a new episode.
     * @return State of episode start.
     */
    virtual State newEpisode() =0;

    /**
      * This abstract method is to be implemented with the initialization
      * logic for the agent.
      * @return
      */
     virtual SetupEnvResponse setup() = 0;

    /**
     * Method that simulates one step of the server.
     * @param action Action of the robot
     * @return Result of simulation
     */
    virtual SimulationResponse runStep(Action action) = 0;

protected:
    //helper methods
    Vector3<double> getAgentPos();
    double getAgentAngle();
    bool robotFallen();

    // coordinate system
    double INITIAL_ROBOT_X = 0.0;
    double INITIAL_ROBOT_Y = 0.0;
    double INITIAL_ROBOT_THETA = 0.0;

    // TODO Delete these variables from base class
    Vector3<double> currentPos;
    double currentTheta;

    // Control signal
    double commandedXSpeed;
    double commandedYSpeed;
    double commandedThetaSpeed;

    const int serverPort;
    const int monitorPort;

    int agentNumber;

    unique_ptr<FeatureExtractor> featEx;

    utils::wizard::Wizard wiz;

    // simulation variables
    communication::Communication communication;
    communication::Communication wizCommunication;
    decision_making::behavior::BehaviorFactory behaviorFactory;
    action::ActionImpl action;
    decision_making::DecisionMakingStub decisionMaking;
    control::ControlImpl control;
    perception::Perception perception;
    modeling::Modeling modeling;
    representations::HearData hearData;
    //std::shared_ptr<utils::roboviz::Roboviz> roboviz;
};

#endif //ITANDROIDS_SOCCER3D_CPP_BASELEARNINGAGENT_H
