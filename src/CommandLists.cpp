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
    printf("(%02d:%02d:%02d) [CMD] : Robot Start\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(4);
}

void CommandLists::Recovery()
{
    sharedMemory->UDPCommand = CMD_RECOVERY;
    printf("(%02d:%02d:%02d) [CMD] : Recovery\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(5);
}

void CommandLists::HomeUp()
{
    sharedMemory->UDPCommand = CMD_HOME_UP;
    printf("(%02d:%02d:%02d) [CMD] : Home Up\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(4);
}

void CommandLists::HomeDown()
{
    sharedMemory->UDPCommand = CMD_HOME_DOWN;
    printf("(%02d:%02d:%02d) [CMD] : Home Down\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(4);
}

void CommandLists::TrotFast()
{
    sharedMemory->UDPCommand = CMD_TROT_OVERLAP;
    printf("(%02d:%02d:%02d) [CMD] : Trot Fast\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(2);
}

void CommandLists::TrotSlow()
{
    sharedMemory->UDPCommand = CMD_TROT_SLOW;
    printf("(%02d:%02d:%02d) [CMD] : Trot Slow\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(2);
}

void CommandLists::TrotStop()
{
    sharedMemory->UDPCommand = CMD_TROT_STOP;
    printf("(%02d:%02d:%02d) [CMD] : Trot Stop\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(2);
}

void CommandLists::EmergencyStop()
{
    sharedMemory->UDPCommand = CMD_E_STOP;
    printf("(%02d:%02d:%02d) [CMD] : Emergency Stop\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
}

void CommandLists::ArmHome()
{
    sharedMemory->UDPCommand = CMD_ARM_HOME;
    printf("(%02d:%02d:%02d) [CMD] : Arm Home\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
}

#include <iostream>
void CommandLists::ArmMove(double* goalPosition, double* goalEulerAngle)
{
    double goalPositionLimit[3][2];
    double goalEulerAngleLimit[3][2];

    goalPositionLimit[0][0] = 0.4;// x-axis position lower limit
    goalPositionLimit[0][1] = 0.6;// x-axis position upper limit
    goalPositionLimit[1][0] = -0.2;// y-axis position lower limit
    goalPositionLimit[1][1] = 0.2;// y-axis position upper limit
    goalPositionLimit[2][0] = 0.1;// z-axis position lower limit
    goalPositionLimit[2][1] = 0.5;// z-axis position upper limit

    goalEulerAngleLimit[0][0] = -30 * 3.141592 / 180.0;// roll angle lower limit
    goalEulerAngleLimit[0][1] = 30 * 3.141592 / 180.0;// roll angle upper limit
    goalEulerAngleLimit[1][0] = -45 * 3.141592 / 180.0;// pitch angle lower limit
    goalEulerAngleLimit[1][1] = 45 * 3.141592 / 180.0;// pitch angle upper limit
    goalEulerAngleLimit[2][0] = -45 * 3.141592 / 180.0;// yaw angle lower limit
    goalEulerAngleLimit[2][1] = 45 * 3.141592 / 180.0;// yaw angle upper limit

    sharedMemory->UDPDesiredEndEffectorPosition[0] = fmin(fmax(goalPosition[0], goalPositionLimit[0][0]), goalPositionLimit[0][1]);
    sharedMemory->UDPDesiredEndEffectorPosition[1] = fmin(fmax(goalPosition[1], goalPositionLimit[1][0]), goalPositionLimit[1][1]);
    sharedMemory->UDPDesiredEndEffectorPosition[2] = fmin(fmax(goalPosition[2], goalPositionLimit[2][0]), goalPositionLimit[2][1]);

    sharedMemory->UDPDesiredEndEffectorEulerAngle[0] = fmin(fmax(goalEulerAngle[0], goalEulerAngleLimit[0][0]), goalEulerAngleLimit[0][1]);
    sharedMemory->UDPDesiredEndEffectorEulerAngle[1] = fmin(fmax(goalEulerAngle[1], goalEulerAngleLimit[1][0]), goalEulerAngleLimit[1][1]);
    sharedMemory->UDPDesiredEndEffectorEulerAngle[2] = fmin(fmax(goalEulerAngle[2], goalEulerAngleLimit[2][0]), goalEulerAngleLimit[2][1]);

    sharedMemory->UDPCommand = CMD_ARM_MOVE;
    printf("(%02d:%02d:%02d) [CMD] : Arm Move\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(3);
}

void CommandLists::ArmTeleoperation(double* armLinearVelocityRef, double* armAngularVelocityRef)
{
    sharedMemory->UDPCommand = CMD_ARM_TELE;
    printf("[CMD] : Arm Teleoperation\n");
    printf("[CMD] : Notice: This mode needs joystick or other teleoperation devices.\n");

    double armLinearVelocityLimit[3] = {0.0};
    double armAngularVelocityLimit[3] = {0.0};

    switch(sharedMemory->armFSMState)
    {
        case ARM_FSM::ARM_TELE:
            {
                armAngularVelocityLimit[0] = 0.5;
                armAngularVelocityLimit[1] = 0.5;
                armAngularVelocityLimit[2] = 0.5;
                armLinearVelocityLimit[0] = 0.5;
                armLinearVelocityLimit[1] = 0.5;
                armLinearVelocityLimit[2] = 0.5;
                break;
            }
        default:
            break;
    }

    sharedMemory->UDPDesiredTeleOperationLinearVelocity[0] = fmin(fmax(armLinearVelocityRef[0], -armLinearVelocityLimit[0]), armLinearVelocityLimit[0]);
    sharedMemory->UDPDesiredTeleOperationLinearVelocity[1] = fmin(fmax(armLinearVelocityRef[1], -armLinearVelocityLimit[1]), armLinearVelocityLimit[1]);
    sharedMemory->UDPDesiredTeleOperationLinearVelocity[2] = fmin(fmax(armLinearVelocityRef[2], -armLinearVelocityLimit[2]), armLinearVelocityLimit[2]);
    sharedMemory->UDPDesiredTeleOperationAngularVelocity[0] = fmin(fmax(armAngularVelocityRef[0], -armAngularVelocityLimit[0]), armAngularVelocityLimit[0]);
    sharedMemory->UDPDesiredTeleOperationAngularVelocity[1] = fmin(fmax(armAngularVelocityRef[1], -armAngularVelocityLimit[1]), armAngularVelocityLimit[1]);
    sharedMemory->UDPDesiredTeleOperationAngularVelocity[2] = fmin(fmax(armAngularVelocityRef[2], -armAngularVelocityLimit[2]), armAngularVelocityLimit[2]);
}

void CommandLists::ArmGripperClose()
{
    sharedMemory->UDPCommand = CMD_ARM_GRIPPER_CLOSE;
    printf("(%02d:%02d:%02d) [CMD] : Arm Gripper Close\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(2);
}

void CommandLists::ArmGripperOpen()
{
    sharedMemory->UDPCommand = CMD_ARM_GRIPPER_OPEN;
    printf("(%02d:%02d:%02d) [CMD] : Arm Gripper Open\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(2);
}

void CommandLists::SetBodyVelocity(double *refVel)
{
    double velocityLimit[3] = {0.0};

    switch (sharedMemory->FSMState)
    {
        case FSM_TROT_SLOW:
        {
            velocityLimit[0] = 0.6;
            velocityLimit[1] = 0.4;
            velocityLimit[2] = 0.65;
            break;
        }
        case FSM_TROT_FAST:
        {
            velocityLimit[0] = 0.6;
            velocityLimit[1] = 0.4;
            velocityLimit[2] = 0.65;
            break;
        }
        case FSM_OVERLAP_TROT_FAST:
        {
            velocityLimit[0] = 1.0;
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