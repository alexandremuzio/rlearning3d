#ifndef ITANDROIDS_SOCCER3D_CPP_RACINGLEARNINGAGENT_H
#define ITANDROIDS_SOCCER3D_CPP_RACINGLEARNINGAGENT_H

#include "BaseLearningAgent.h"

#include "soccer3d.grpc.pb.h"

using api::Action;
using api::SetupEnvResponse;
using api::SimulationResponse;
using api::State;

class RacerLearningAgent : public BaseLearningAgent {
public:
    RacerLearningAgent(std::string host = "127.0.0.1", int serverPort = 3100, int monitorPort = 3200, int agentNumber = 1,
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
    State state() const;

    // Runs simulation step in the environment
    void step();

    //helper methods
    void drawEnvironment();
    void drawStats();
    bool robotLeftTrack();
    bool robotReachedFinishLine();

    // actions
    double actionXSpeed;
    double actionYSpeed;
    double actionThetaSpeed;

    // speed
    double currentXSpeed;
    double currentYSpeed;
    double currentThetaSpeed;

    Vector3<double> lastPos;

    double lastTheta;

    int iEpi = 0;
    int nbSteps;

    std::vector<double> speeds;
};


#endif //ITANDROIDS_SOCCER3D_CPP_RACINGLEARNINGAGENT_H
