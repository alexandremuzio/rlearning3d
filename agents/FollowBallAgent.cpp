#include "FollowBallAgent.h"

#include "representations/RepresentationsLoader.h"
#include "utils/configure_tree/ParameterTreeCreator.h"

const int NUM_STEP_SAME_INPUT = 3;
const double N_CURRICULUM_STEPS = 1400000 * NUM_STEP_SAME_INPUT;

FollowBallAgent::FollowBallAgent(int agentNumber, std::string teamName, std::string curriculumStr) : communication(
        new communication::ServerSocketImpl("127.0.0.1", 3100)),
                                                                          wizCommunication(
                                                                                  new communication::ServerSocketImpl(
                                                                                          "127.0.0.1", 3200)),
                                                                          wiz(wizCommunication),
                                                                          control(static_cast<representations::RobotType::ROBOT_TYPE>(0)),
                                                                          modeling(agentNumber, teamName),
                                                                          AGENT_NUMBER(agentNumber) {
    auto root = utils::configure_tree::ParameterTreeCreator::getRoot<>();
    RepresentationsLoader representationsLoader;
    representationsLoader.setParameter(root->getChild("representations"));
    control.setParameter(root->getChild("control"));
    communication.establishConnection();
    wizCommunication.establishConnection();
    action.create(0); //number of robot type
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

    if (curriculumStr == "on")
        this->curriculum = true;
}

FollowBallAgent::~FollowBallAgent() {
    communication.closeConnection();
}

void FollowBallAgent::run() {
    while(true) {
        nSteps += 1;
        communication.receiveMessage();
        perception.perceive(communication);
        modeling.model(perception, control);

        Vector3<double> ballPosition =  wiz.getBallTranslation();
        Vector3<double> myPosition = wiz.getAgentTranslation(1 + 11);
        double angle = (ballPosition - myPosition).getAlpha();

        Pose2D target = Pose2D(0.0, -ballPosition.x, -ballPosition.y);

        if (!curriculum || nSteps > N_CURRICULUM_STEPS) {
            behaviorFactory.getDribbleBall().behave(modeling);
            decisionMaking.movementRequest = behaviorFactory.getDribbleBall().getMovementRequest();
        }
        behaviorFactory.getActiveVision().behave(modeling);
        decisionMaking.lookRequest =  behaviorFactory.getActiveVision().getLookRequest();

        control.control(perception, modeling, decisionMaking);
        action.act(decisionMaking, control);
        communication.sendMessage(action.getServerMessage());
    }
}

int main(int argc, char *argv[]) {
    std::string agentNumber(argv[1]);
    std::string curriculum;

    if (argc < 4)
        curriculum = "off";
    else
        curriculum = argv[3];
    FollowBallAgent agent(std::stoi(agentNumber), argv[2], curriculum);
    agent.run();
}