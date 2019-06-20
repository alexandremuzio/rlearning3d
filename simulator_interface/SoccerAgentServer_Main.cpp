#include "SoccerAgentService.h"

#include "external/easylogging++.h"
#include "core/utils/command_line_parser/AgentCommandLineParser.h"

using core::utils::command_line_parser::AgentCommandLineParser;
using std::string;

INITIALIZE_EASYLOGGINGPP

void RunServer(string serverIp, int serverPort, int monitorPort, string teamName) {
    string serverAddress("0.0.0.0:" + std::to_string(5000));

    SoccerAgentService service(serverIp, serverPort, monitorPort, teamName);
    std::promise<void> &closeRequested = service.getClosePromise();

    ServerBuilder builder;
    builder.AddListeningPort(serverAddress, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    std::unique_ptr<Server> server(builder.BuildAndStart());
    LOG(INFO) << "Server listening on " << serverAddress;

    auto serveThread = [&]() {
        server->Wait();
    };

    std::thread servingThread(serveThread);
    auto f = closeRequested.get_future();
    f.wait();

    LOG(INFO) << "Server shutting down";
    server->Shutdown();
    servingThread.join();
}

int main(int argc, char** argv) {
    // Load configuration from file
    el::Configurations conf("config/log.conf");
    el::Loggers::reconfigureLogger("default", conf);;
    AgentCommandLineParser parser;
    parser.parse(argc, argv);

    RunServer(parser.getServerIP(), parser.getServerPort(), parser.getMonitorPort(), parser.getTeamName());
}
