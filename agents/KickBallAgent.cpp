//
// Created by luis on 7/17/18.
//

#include "KickBallAgent.h"

const int STEPS_DELAY = 0;

KickBallAgent::KickBallAgent(std::string host, int agentPort, int monitorPort,
                             int agentNumber,
                             int robotType, std::string teamName) : communication(
        new communication::ServerSocketImpl(host, agentPort)), wizCommunication(
        new communication::ServerSocketImpl(host, monitorPort)), wiz(wizCommunication), modeling(agentNumber, teamName),
                                                                    controller(0, 180.0 / (M_PI * 7.0)),
                                                                    inverseKinematics(
                                                                            control::walking::RobotPhysicalParameters::getNaoPhysicalParameters()),
                                                                    kick(control::kick::KickZMPParams::getDefaultKickZMPParams(),
                                                                         false,
                                                                         control::walking::RobotPhysicalParameters::getNaoPhysicalParameters(),
                                                                         &inverseKinematics) {
//    anglesFile.open("/home/luis/TG/itandroids-soccer3d/source/tools/rlearning3d/rl_agents/angles.txt");
    printCommands = false;
    bool useDevcloud = false;
    string file_suffix = printCommands ? "angles_commands" : "angles";
    string file_prefix = useDevcloud ? "~/itandroids-soccer3d/binaries/" : "";
    anglesFile.open(file_suffix + ".txt");
    communication.establishConnection();
    wizCommunication.establishConnection();
    action.create(0);
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
    action.init(1, "ITAndroids");
    communication.sendMessage(action.getServerMessage());
    finishedKick = false;

    wiz.run();
    wiz.setAgentPositionAndDirection(agentNumber, representations::PlaySide::LEFT, Vector3<double>(0, 0, 0.3), 0);
    wiz.setBallPosition(Vector3<double>(0.5,0,0));
}

KickBallAgent::~KickBallAgent() {
    communication.closeConnection();
    anglesFile.close();
};

void KickBallAgent::runStep() {
    if (kick.hasKicked()) {
        kick.reset();
        finishedKick = true;
    }
    jointsTargets.clear();
    communication.receiveMessage();
    perception.perceive(communication);
    if (accumulatedTime < 2.0) {
        kick.update(0.0, jointsTargets);
        jointsTargets.scale(accumulatedTime / 2.0);
    } else {
        kick.update(STEP_DT, jointsTargets);
    }
    jointsTargets = jointsTargets
                    * representations::NaoJoints::getNaoDirectionsFixing();

    controller.update(jointsTargets,
                      perception.getAgentPerception().getNaoJoints());
    controlStub.naoJoints = controller.getCommands();
}

void KickBallAgent::testLoop(int numberOfSteps) {
    accumulatedTime = 0;
    wiz.setPlayMode(representations::RawPlayMode::PLAY_ON);
    // print initial state
//    printJoints(accumulatedTime, perception.getAgentPerception().getNaoJoints());
    printJoints(accumulatedTime, jointsTargets);
    for (int currentStep = 0; currentStep < numberOfSteps && !finishedKick; currentStep++) {
        // update control data
        runStep();
        if(printCommands) // print final frame control
            printJoints(accumulatedTime, jointsTargets);
        // run all simulation cycle
        step();
        // reach next state
        accumulatedTime += STEP_DT;
        if(!printCommands){
            if(accumulatedTime - STEPS_DELAY >= 0)
                printJoints(accumulatedTime-STEPS_DELAY, perception.getAgentPerception().getNaoJoints());
        }
//
    }
}

void KickBallAgent::printJoints(double time, representations::NaoJoints curJointsPos) {
    anglesFile << time << " ";
    anglesFile << curJointsPos.neckPitch << " ";
    anglesFile << curJointsPos.neckYaw << " ";

    anglesFile << curJointsPos.leftShoulderPitch << " ";
    anglesFile << curJointsPos.leftShoulderYaw << " ";
    anglesFile << curJointsPos.leftArmRoll << " ";
    anglesFile << curJointsPos.leftArmYaw << " ";

    anglesFile << curJointsPos.rightShoulderPitch << " ";
    anglesFile << curJointsPos.rightShoulderYaw << " ";
    anglesFile << curJointsPos.rightArmRoll << " ";
    anglesFile << curJointsPos.rightArmYaw << " ";

    anglesFile << curJointsPos.leftHipYawPitch << " ";
    anglesFile << curJointsPos.leftHipRoll << " ";
    anglesFile << curJointsPos.leftHipPitch << " ";
    anglesFile << curJointsPos.leftKneePitch << " ";
    anglesFile << curJointsPos.leftFootPitch << " ";
    anglesFile << curJointsPos.leftFootRoll << " ";

    anglesFile << curJointsPos.rightHipYawPitch << " ";
    anglesFile << curJointsPos.rightHipRoll << " ";
    anglesFile << curJointsPos.rightHipPitch << " ";
    anglesFile << curJointsPos.rightKneePitch << " ";
    anglesFile << curJointsPos.rightFootPitch << " ";
    anglesFile << curJointsPos.rightFootRoll << std::endl;
    anglesFile.flush();
}

void KickBallAgent::step() {
    action.act(decisionMaking, controlStub);
    communication.sendMessage(action.getServerMessage());
    communication.receiveMessage();
    perception.perceive(communication);
    modeling.model(perception, controlStub);
}

int main() {
    KickBallAgent agent;
    agent.testLoop(200);
}