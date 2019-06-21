#include "DummyAgent.h"

#include "representations/RepresentationsLoader.h"
#include "utils/configure_tree/ParameterTreeCreator.h"

DummyAgent::DummyAgent(int agentNumber, std::string teamName) : communication(
        new communication::ServerSocketImpl("127.0.0.1", 3100)),
//                                                                wizCommunication(
//                                                                        new communication::ServerSocketImpl("127.0.0.1",
//                                                                                                            3200)),
////                                                                wiz(wizCommunication),
                                                                control(static_cast<representations::RobotType::ROBOT_TYPE>(0)),
                                                                modeling(agentNumber, teamName),
                                                                AGENT_NUMBER(agentNumber) {
    auto root = utils::configure_tree::ParameterTreeCreator::getRoot<>();
    RepresentationsLoader representationsLoader;
    representationsLoader.setParameter(root->getChild("representations"));
//    control.setParameter(root->getChild("control"));
    communication.establishConnection();
//    wizCommunication.establishConnection();
    action.create(0); //number of robot type
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
    action.init(agentNumber, teamName);

//    wiz.run();
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
    modeling.model(perception, control);

    decisionMaking.beamRequest = NULL;
//    control.control(perception, modeling, decisionMaking);
    action.act(decisionMaking, control);
    communication.sendMessage(action.getServerMessage());
}

DummyAgent::~DummyAgent() {
    communication.closeConnection();
}

void DummyAgent::run() {
    while(true) {
        communication.receiveMessage();
        perception.perceive(communication);
        modeling.model(perception, control);

        //step

        control.control(perception, modeling, decisionMaking);
        action.act(decisionMaking, control);
        communication.sendMessage(action.getServerMessage());
    }
}

int main(int argc, char *argv[]) {
    std::string agentNumber(argv[1]);

    DummyAgent agent(std::stoi(agentNumber), argv[2]);
    agent.run();
}