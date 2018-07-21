#ifndef ITANDROIDS_SOCCER3D_CPP_SIMPLELEARNINGAGENT_H
#define ITANDROIDS_SOCCER3D_CPP_SIMPLELEARNINGAGENT_H

#include "BaseLearningAgent.h"

#include "soccer3d.grpc.pb.h"

#include "external/json.h"


using api::Action;
using api::SetupEnvResponse;
using api::SimulationResponse;
using api::State;

class StealBallLearningAgent : public BaseLearningAgent {
public:
    StealBallLearningAgent(std::string host = "127.0.0.1", int serverPort = 3100, int monitorPort = 3200, int agentNumber = 2,
                  int robotType = 0, std::string teamName = std::string("ITAndroids"));

    State newEpisode() override;
    SimulationResponse runStep(Action action) override;
    SetupEnvResponse setup() override;

private:
    // Checks whether the episode has finished
    bool episodeOver();

    // Reward signal
    double reward();

    // Environment state
    State state(); //TODO should be const

    // Runs simulation step in the environment
    void step();

    //helper methods
    void drawEnvironment();
    void drawStats();

    Json::Value config;
    int iEpi = 0;
    int nbEpisodeSteps;
    int nbTotalSteps = 0;
    double currReward;
};


#endif //ITANDROIDS_SOCCER3D_CPP_SIMPLELEARNINGAGENT_H
