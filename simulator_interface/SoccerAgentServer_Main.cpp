#include "SoccerAgentService.h"

#include "representations/ITAndroidsConstants.h"

#include "external/easylogging++.h"
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using std::string;

INITIALIZE_EASYLOGGINGPP

void parseArguments(int argc, char** argv, po::variables_map& variablesMap) {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("server-ip", po::value<std::string>()->default_value(ITAndroidsConstants::DEFAULT_IP),
            "set server ip")
        ("server-port", po::value<int>()->default_value(ITAndroidsConstants::DEFAULT_SERVER_PORT),
            "set agent port")
        ("monitor-port", po::value<int>()->default_value(ITAndroidsConstants::DEFAULT_MONITOR_PORT),
            "set monitor port")
        ("team-name", po::value<std::string>()->default_value(ITAndroidsConstants::TEAM_DEFAULT_NAME),
            "set team name")
        ("agent-type", po::value<std::string>()->default_value("steal"),
            "set team name")
        ;

    po::store(po::parse_command_line(argc, argv, desc), variablesMap);
    po::notify(variablesMap);

    if(variablesMap.count("help")){
        std::cout << desc << std::endl;
        exit(1);
    }

    LOG(INFO) << "COMMAND LINE PARAMETERS";
    for (const auto &it : variablesMap) {
        std::string param = it.first.c_str();
        auto &value = it.second.value();
        if (auto v = boost::any_cast<int>(&value))
            LOG(INFO) << param << " " << *v;
        else if (auto v = boost::any_cast<std::string>(&value))
            LOG(INFO) << param << " " << *v;
        else
            LOG(INFO) << "error";
    }
}

void runServer(string serverIp, int serverPort, int monitorPort, string teamName, string agentType) {
    string serverAddress("0.0.0.0:" + std::to_string(5000));

    SoccerAgentService service(serverIp, serverPort, monitorPort, teamName, agentType);
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
    el::Loggers::reconfigureLogger("default", conf);

    po::variables_map vm;
    parseArguments(argc, argv, vm);

    runServer(
        vm["server-ip"].as<std::string>(),
        vm["server-port"].as<int>(),
        vm["monitor-port"].as<int>(),
        vm["team-name"].as<std::string>(),
        vm["agent-type"].as<std::string>());
}
