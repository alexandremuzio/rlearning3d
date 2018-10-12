#include "BaseLearningAgent.h"
#include "representations/RepresentationsLoader.h"

using representations::RepresentationsLoader;

BaseLearningAgent::BaseLearningAgent(std::string host, int serverPort, int monitorPort, int agentNumber, int robotType,
                                     std::string teamName) :
        featEx(nullptr),
        communication(
                new communication::ServerSocketImpl(host, serverPort)),
        wizCommunication(
                new communication::ServerSocketImpl(host, monitorPort)),
        wiz(wizCommunication),
        control(static_cast<representations::RobotType::ROBOT_TYPE>(robotType)),
        modeling(agentNumber, teamName), agentNumber(agentNumber),
        serverPort(serverPort), monitorPort(monitorPort) {

    //roboviz = utils::roboviz::Roboviz::getInstance();
    auto root = utils::configure_tree::ParameterTreeCreator::getRoot<>();
    RepresentationsLoader representationsLoader;
    representationsLoader.setParameter(root->getChild("representations"));
    control.setParameter(root->getChild("control"));
    communication.establishConnection();
    wizCommunication.establishConnection();
    action.create(robotType); //number of robot type
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
    action.init(agentNumber, teamName);

    wiz.run();
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
    modeling.model(perception, control);

    decisionMaking.beamRequest = NULL;
    control.control(perception, modeling, decisionMaking);
    action.act(decisionMaking, control);
    communication.sendMessage(action.getServerMessage());
}


BaseLearningAgent::~BaseLearningAgent() {
    communication.closeConnection();
}

Vector3<double> BaseLearningAgent::getAgentPos() {
    Vector3<double> position = wiz.getAgentTranslation(1);
    position.x = position.x - INITIAL_ROBOT_X;
    position.y = position.y - INITIAL_ROBOT_Y;
    return position;
}
double BaseLearningAgent::getAgentAngle() {
    // TODO fix initial angle problem
    double angle = wiz.getAgentRotation().getZAngle() - M_PI_2 + INITIAL_ROBOT_THETA;

    if (std::isnan(angle))
        return 0.0;

    if (angle > M_PI)
        angle -= 2* M_PI;
    return angle;
}
bool BaseLearningAgent::robotFallen() {
    return currentPos.z < 0.2;
}






