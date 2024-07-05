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
    sharedMemory->UDPCommand = CMD_NO_ACT;
}

void CommandLists::Start()
{
    sharedMemory->UDPCommand = CMD_START;
    printf("[CMD] : Robot Start\n");
}

void CommandLists::Recovery()
{
    sharedMemory->UDPCommand = CMD_RECOVERY;
    printf("[CMD] : Recovery\n");
}

void CommandLists::HomeUp()
{
    sharedMemory->UDPCommand = CMD_HOME_UP;
    printf("[CMD] : Home Up\n");
}

void CommandLists::HomeDown()
{
    sharedMemory->UDPCommand = CMD_HOME_DOWN;
    printf("[CMD] : Home Down\n");
}

void CommandLists::TrotFast()
{
    sharedMemory->UDPCommand = CMD_TROT_OVERLAP;
    printf("[CMD] : Trot Fast\n");
}

void CommandLists::TrotSlow()
{
    sharedMemory->UDPCommand = CMD_TROT_SLOW;
    printf("[CMD] : Trot Slow\n");
}

void CommandLists::TrotStop()
{
    sharedMemory->UDPCommand = CMD_TROT_STOP;
    printf("[CMD] : Trot Stop\n");
}

void CommandLists::EmergencyStop()
{
    sharedMemory->UDPCommand = CMD_E_STOP;
    printf("[CMD] : Emergency Stop\n");
}

void CommandLists::SetBodyVelocity(double *refVel)
{
    double velocityLimit[3] = {0.0};

    switch (sharedMemory->FSMState)
    {
        case FSM_TROT_SLOW:
        {
            velocityLimit[0] = 0.8;
            velocityLimit[1] = 0.4;
            velocityLimit[2] = 0.65;
            break;
        }
        case FSM_TROT_FAST:
        {
            velocityLimit[0] = 0.8;
            velocityLimit[1] = 0.4;
            velocityLimit[2] = 0.65;
            break;
        }
        case FSM_OVERLAP_TROT_FAST:
        {
            velocityLimit[0] = 1.2;
            velocityLimit[1] = 0.4;
            velocityLimit[2] = 0.65;
            break;
        }
        default:
            break;

    }

    sharedMemory->UDPRefBodyLinearVelocity_x = fmin(fmax(refVel[0], -velocityLimit[0]), velocityLimit[0]);
    sharedMemory->UDPRefBodyLinearVelocity_y = fmin(fmax(refVel[1], -velocityLimit[1]), velocityLimit[1]);
    sharedMemory->UDPRefBodyAngularVelocity_yaw = fmin(fmax(refVel[2], -velocityLimit[2]), velocityLimit[2]);
}