#include "KickLearningAgent.h"
#include "StealBallLearningAgent.h"
#include "SoccerAgentService.h"

#include "external/easylogging++.h"


SoccerAgentService::SoccerAgentService(string serverIp, int serverPort, int monitorPort, string teamName)
        : serverIp(serverIp), serverPort(serverPort), monitorPort(monitorPort), teamName(teamName) {
    LOG(INFO) << "Starting Server...";
    LOG(INFO) << "S3D Server Port: " << serverPort;
    LOG(INFO) << "S3D Monitor Port: " << monitorPort;
}

Status SoccerAgentService::SetupEnvironment(ServerContext *context, const SetupEnvRequest *request,
                                           SetupEnvResponse *response) {
    agent = std::make_unique<StealBallLearningAgent>(serverIp, serverPort, monitorPort, nbAgents, 0,
                                                     std::string("ITAndroids"));
    response->CopyFrom(agent->setup());
    return Status::OK;
}

Status SoccerAgentService::Simulate(ServerContext *context, const SimulationRequest *request,
                                   SimulationResponse *response) {
    response->CopyFrom(agent->runStep(request->action()));
    return Status::OK;
}

Status SoccerAgentService::StartEpisode(ServerContext *context, const EpisodeRequest *request,
                                       EpisodeResponse *response) {
    response->mutable_state()->CopyFrom(agent->newEpisode());
    return Status::OK;
}

Status SoccerAgentService::CloseEnvironment(ServerContext *context, const CloseRequest *request,
                                            CloseResponse *response) {
    closeRequested.set_value();
    return Status::OK;
}

