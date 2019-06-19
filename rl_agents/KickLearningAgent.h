//
// Created by luckeciano on 6/1/18.
//

#ifndef ITANDROIDS_SOCCER3D_CPP_KICKLEARNINGAGENT_H
#define ITANDROIDS_SOCCER3D_CPP_KICKLEARNINGAGENT_H


#include "BaseLearningAgent.h"

#include "soccer3d.grpc.pb.h"

using api::Action;
using api::SetupEnvResponse;
using api::SimulationResponse;
using api::State;


class KickLearningAgent: public BaseLearningAgent {
public:
    KickLearningAgent(std::string host = "127.0.0.1", int serverPort = 3100, int monitorPort = 3200, int agentNumber = 1,
    int robotType = 0, std::string teamName = std::string("ITAndroids"));

    State newEpisode() override;
    SimulationResponse runStep(Action action) override;
    SetupEnvResponse setup() override;

private:
    // Checks whether the episode has finished
    bool episodeOver();

    // Reward signal
    double reward();

    double getReferenceMotionError();

    double getReferenceTorsoError();

    // Environment state
    State state() const;

    // Runs simulation step in the environment
    void step();
    void stepControl();

    //helper methods
    void drawEnvironment();
    void drawStats();

    void kick();
    void look();
    void setAgentRandomPosition();

    // distance
    double xDistance;
    double yDistance;
    double maxHeight;

    //ball
    Vector3<double> lastBallTranslation;
    Vector3<double> ballVelocity;
    bool ballKicked;

    //keyframe
    double keyframeTime;
    bool hasEnded;

    //reference motion
    std::map<int, vector<double> > referenceMotionMap;
   // std::map<int, double> referenceTorsoMap;
    double keyframeStage;
    double lastRefError;

    //control
    vector<double> desiredJoints;

    int iEpi = 0;
    int nbSteps;
};


#endif //ITANDROIDS_SOCCER3D_CPP_KICKLEARNINGAGENT_H
