//
// Created by jh on 24. 7. 5.
//

#include "CommandLists.hpp"

CommandLists::CommandLists()
{
    sharedMemory = SharedMemory::getInstance();
}

void CommandLists::NoCommand()
{
    sharedMemory->udp.joyCommand = CMD_NO_INPUT;
}

void CommandLists::Start()
{
    sharedMemory->udp.joyCommand = CMD_START;
    printf("[CMD] : Robot Start\n");
    sleep(1);
}

void CommandLists::Restart()
{
    sharedMemory->udp.joyCommand = CMD_RESTART;
    printf("[CMD] : Restart CANINE Controller\n");
    sleep(4);
}

void CommandLists::HomeUp()
{
    sharedMemory->udp.joyCommand = CMD_STAND_UP;
    printf("[CMD] : Home Up\n");
    sleep(4);
}

void CommandLists::HomeDown()
{
    sharedMemory->udp.joyCommand = CMD_SIT_DOWN;
    printf("[CMD] : Home Down\n");
    sleep(7);
}

void CommandLists::TrotSlow()
{
    sharedMemory->udp.joyCommand = CMD_TROT_SLOW;
    printf("[CMD] : Trot Slow\n");
    sleep(2);
}

void CommandLists::TrotStop()
{
    double zeros[3];
    zeros[0] = 0.0;
    zeros[1] = 0.0;
    zeros[2] = 0.0;
    SetBodyVelocity(zeros, zeros);
    sleep(2);

    sharedMemory->udp.joyCommand = CMD_TROT_STOP;
    printf("[CMD] : Trot Stop\n");
    sleep(1);
}

void CommandLists::EmergencyStop()
{
    sharedMemory->udp.joyCommand = CMD_EMERGENCY_STOP;
    printf("[CMD] : Emergency Stop\n");
    sleep(2);
}

void CommandLists::SetBodyVelocity(double* refLinearVelocity, double* refAngularVelocity)
{
    Eigen::Vector3d refLinVel = Eigen::Vector3d (refLinearVelocity[0], refLinearVelocity[1], 0.0);
    Eigen::Vector3d refAngVel = Eigen::Vector3d (refAngularVelocity[0], refAngularVelocity[1], refAngularVelocity[2]);

    switch (sharedMemory->FSMState)
    {
    case FSM_STAND:
        refLinVel = refLinVel.array().max(0.0);
        refLinVel = refLinVel.array().min(0.0);

        refAngVel = refAngVel.array().max(-0.5);
        refAngVel = refAngVel.array().min(0.5);
        break;
    case FSM_TROT_SLOW:
        refLinVel = refLinVel.array().max(-0.8);
        refLinVel = refLinVel.array().min(0.8);

        refAngVel = refAngVel.array().max(-0.7);
        refAngVel = refAngVel.array().min(0.7);
        break;
    case FSM_TROT_FAST:
        refLinVel = refLinVel.array().max(-1.2);
        refLinVel = refLinVel.array().min(1.2);

        refAngVel = refAngVel.array().max(-0.7);
        refAngVel = refAngVel.array().min(0.7);
        break;
    case FSM_OVERLAP_TROT_FAST:
        refLinVel = refLinVel.array().max(-0.8);
        refLinVel = refLinVel.array().min(0.8);

        refAngVel = refAngVel.array().max(-0.7);
        refAngVel = refAngVel.array().min(0.7);
        break;
    default:
        break;

    }

    sharedMemory->udp.userLinVel = refLinVel;
    sharedMemory->udp.userAngVel = refAngVel;
}
