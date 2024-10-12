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
}

void CommandLists::Restart()
{
    sharedMemory->udp.joyCommand = CMD_RESTART;
    printf("[CMD] : Recovery\n");
}

void CommandLists::StandUp()
{
    sharedMemory->udp.joyCommand = CMD_STAND_UP;
    printf("[CMD] : Home Up\n");
}

void CommandLists::SitDown()
{
    sharedMemory->udp.joyCommand = CMD_SIT_DOWN;
    printf("[CMD] : Home Down\n");
}

void CommandLists::TrotSlow()
{
    sharedMemory->udp.joyCommand = CMD_TROT_SLOW;
    printf("[CMD] : Trot Slow\n");
}

void CommandLists::TrotStop()
{
//    if(sharedMemory->FSMState == FSM_TROT_SLOW)
//    {
//        sharedMemory->udp.joyCommand = CMD_TROT_STOP;
//    }
//    else
//    {
//        sharedMemory->udp.joyCommand = CMD_NO_INPUT;
//    }

    sharedMemory->udp.joyCommand = CMD_TROT_STOP;
    printf("[CMD] : Trot Stop\n");
}

void CommandLists::TrotForceStop()
{
//    if(sharedMemory->FSMState == FSM_TROT_STOP)
//    {
//        sharedMemory->udp.joyCommand = CMD_TROT_STOP;
//    }
//    else
//    {
//        sharedMemory->udp.joyCommand = CMD_NO_INPUT;
//    }
    sharedMemory->udp.joyCommand = CMD_TROT_STOP;
    printf("[CMD] : Trot Force Stop\n");
}

void CommandLists::EmergencyStop()
{
    sharedMemory->udp.joyCommand = CMD_EMERGENCY_STOP;
    printf("[CMD] : Emergency Stop\n");
}

void CommandLists::SetBodyVelocity(double refVelX, double refVelY, double refVelYaw)
{
    Eigen::Vector3d refLinVel = Eigen::Vector3d (refVelX, refVelY, 0);
    Eigen::Vector3d refAngVel = Eigen::Vector3d (0, 0, refVelYaw);

    switch (sharedMemory->FSMState)
    {
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
    case FSM_WHEEL_MOVE:
        refLinVel = refLinVel.array().max(-1.5);
        refLinVel = refLinVel.array().min(1.5);

        refAngVel.setZero();
        break;
    default:
        break;

    }

    sharedMemory->udp.userLinVel = refLinVel;
    sharedMemory->udp.userAngVel = refAngVel;
}