#ifndef ITANDROIDS_SOCCER3D_CPP_RUNNINGLEARNINGAGENT_H
#define ITANDROIDS_SOCCER3D_CPP_RUNNINGLEARNINGAGENT_H

#include "BaseLearningAgent.h"

#include "soccer3d.grpc.pb.h"

using api::Action;
using api::SetupEnvResponse;
using api::SimulationResponse;
using api::State;

class RunningLearningAgent : public BaseLearningAgent {
public:
    RunningLearningAgent(std::string host = "127.0.0.1", int serverPort = 3100, int monitorPort = 3200, int agentNumber = 1,
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
    void drawStats();

    double actionXSpeed;
    double currentXSpeed;
    double referenceXSpeed;
    Pose2D currentPose;
    Pose2D lastPose;
    int nbSteps;

    std::vector<double> speeds;
};


#endif //ITANDROIDS_SOCCER3D_CPP_RUNNINGLEARNINGAGENT_H
