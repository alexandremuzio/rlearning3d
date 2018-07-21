#ifndef ITANDROIDS_SOCCER3D_CPP_SIMPLEAGENTSERVER_H
#define ITANDROIDS_SOCCER3D_CPP_SIMPLEAGENTSERVER_H

#include "soccer3d.grpc.pb.h"
#include "RacerLearningAgent.h"
#include "RunningLearningAgent.h"
#include "StealBallLearningAgent.h"

#include <grpc++/grpc++.h>
#include <map>
#include <future>

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using api::DDPGTrainer;
using api::SetupEnvRequest;
using api::SetupEnvResponse;
using api::SimulationRequest;
using api::SimulationResponse;
using api::EpisodeRequest;
using api::EpisodeResponse;
using api::CloseRequest;
using api::CloseResponse;


class SoccerAgentService final : public DDPGTrainer::Service {
public:
    SoccerAgentService() { }
    SoccerAgentService(string serverIp, int serverPort, int monitorPort, string teamName);

    Status SetupEnvironment(ServerContext* context, const SetupEnvRequest* request, SetupEnvResponse* response) override;
    Status Simulate(ServerContext* context, const SimulationRequest* request, SimulationResponse* response) override;
    Status StartEpisode(ServerContext* context, const EpisodeRequest* request, EpisodeResponse* response) override;
    Status CloseEnvironment(ServerContext* context, const CloseRequest* request, CloseResponse* response) override;

    std::promise<void>& getClosePromise() {return closeRequested;}

private:
    std::promise<void> closeRequested;

    string serverIp = "127.0.0.1";
    int serverPort = 3100;
    int monitorPort = 3200;
    string teamName = "ITAndroids";

    int nbAgents = 1;

    unique_ptr<BaseLearningAgent> agent;
};

#endif //ITANDROIDS_SOCCER3D_CPP_SOCCERAGENTSERVER_H