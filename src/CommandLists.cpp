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
    printf("[CMD] : Recovery\n");
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
    sleep(5);
}

void CommandLists::TrotSlow()
{
    sharedMemory->udp.joyCommand = CMD_TROT_SLOW;
    printf("[CMD] : Trot Slow\n");
    sleep(2);
}

void CommandLists::TrotStop()
{
    double refBodyVelocity[3];
    refBodyVelocity[0] = 0.0; // reference x-axis velocity in body frame. [m/s]
    refBodyVelocity[1] = 0.0; // reference y-axis velocity in body frame. [m/s]
    refBodyVelocity[2] = 0.0; // reference yaw velocity. [rad/s]
    SetBodyVelocity(refBodyVelocity);
    sleep(1);

    sharedMemory->udp.joyCommand = CMD_TROT_STOP;
    printf("[CMD] : Trot Stop\n");
    sleep(1);
}

void CommandLists::EmergencyStop()
{
    sharedMemory->udp.joyCommand = CMD_EMERGENCY_STOP;
    printf("[CMD] : Emergency Stop\n");
    sleep(1);
}

void CommandLists::SetBodyVelocity(double* refVel)
{
    Eigen::Vector3d refLinVel = Eigen::Vector3d (refVel[0], refVel[1], 0);
    Eigen::Vector3d refAngVel = Eigen::Vector3d (0, 0, refVel[2]);

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
    default:
        break;

    }

    sharedMemory->udp.userLinVel = refLinVel;
    sharedMemory->udp.userAngVel = refAngVel;
}

void CommandLists::ArmHome()
{
    sharedMemory->udp.joyCommand = CMD_ARM_HOME;
    printf("(%02d:%02d:%02d) [CMD] : Arm Home\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(3);
}

void CommandLists::ArmMove(double* goalPosition, double* goalEulerAngle)
{
    double goalPositionLimit[3][2];
    double goalEulerAngleLimit[3][2];

    goalPositionLimit[0][0] = 0.4;  // x-axis position lower limit
    goalPositionLimit[0][1] = 0.6;  // x-axis position upper limit
    goalPositionLimit[1][0] = -0.2; // y-axis position lower limit
    goalPositionLimit[1][1] = 0.2;  // y-axis position upper limit
    goalPositionLimit[2][0] = 0.1;  // z-axis position lower limit
    goalPositionLimit[2][1] = 0.5;  // z-axis position upper limit

    goalEulerAngleLimit[0][0] = -30 * 3.141592 / 180.0; // roll angle lower limit
    goalEulerAngleLimit[0][1] = 30 * 3.141592 / 180.0;  // roll angle upper limit
    goalEulerAngleLimit[1][0] = -45 * 3.141592 / 180.0; // pitch angle lower limit
    goalEulerAngleLimit[1][1] = 45 * 3.141592 / 180.0;  // pitch angle upper limit
    goalEulerAngleLimit[2][0] = -45 * 3.141592 / 180.0; // yaw angle lower limit
    goalEulerAngleLimit[2][1] = 45 * 3.141592 / 180.0;  // yaw angle upper limit

    sharedMemory->udp.desiredEndEffectorPosition[0] = fmin(fmax(goalPosition[0], goalPositionLimit[0][0]), goalPositionLimit[0][1]);
    sharedMemory->udp.desiredEndEffectorPosition[1] = fmin(fmax(goalPosition[1], goalPositionLimit[1][0]), goalPositionLimit[1][1]);
    sharedMemory->udp.desiredEndEffectorPosition[2] = fmin(fmax(goalPosition[2], goalPositionLimit[2][0]), goalPositionLimit[2][1]);

    sharedMemory->udp.desiredEndEffectorEulerAngle[0] = fmin(fmax(goalEulerAngle[0], goalEulerAngleLimit[0][0]), goalEulerAngleLimit[0][1]);
    sharedMemory->udp.desiredEndEffectorEulerAngle[1] = fmin(fmax(goalEulerAngle[1], goalEulerAngleLimit[1][0]), goalEulerAngleLimit[1][1]);
    sharedMemory->udp.desiredEndEffectorEulerAngle[2] = fmin(fmax(goalEulerAngle[2], goalEulerAngleLimit[2][0]), goalEulerAngleLimit[2][1]);

    sharedMemory->udp.joyCommand = CMD_ARM_MOVE;
    printf("(%02d:%02d:%02d) [CMD] : Arm Move\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(3);
}

void CommandLists::SetArmTeleoperationVelocity(double* armLinearVelocityRef, double* armAngularVelocityRef)
{
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

    sharedMemory->udp.desiredTeleOperationLinearVelocity[0] = fmin(fmax(armLinearVelocityRef[0], -armLinearVelocityLimit[0]), armLinearVelocityLimit[0]);
    sharedMemory->udp.desiredTeleOperationLinearVelocity[1] = fmin(fmax(armLinearVelocityRef[1], -armLinearVelocityLimit[1]), armLinearVelocityLimit[1]);
    sharedMemory->udp.desiredTeleOperationLinearVelocity[2] = fmin(fmax(armLinearVelocityRef[2], -armLinearVelocityLimit[2]), armLinearVelocityLimit[2]);
    sharedMemory->udp.desiredTeleOperationAngularVelocity[0] = fmin(fmax(armAngularVelocityRef[0], -armAngularVelocityLimit[0]), armAngularVelocityLimit[0]);
    sharedMemory->udp.desiredTeleOperationAngularVelocity[1] = fmin(fmax(armAngularVelocityRef[1], -armAngularVelocityLimit[1]), armAngularVelocityLimit[1]);
    sharedMemory->udp.desiredTeleOperationAngularVelocity[2] = fmin(fmax(armAngularVelocityRef[2], -armAngularVelocityLimit[2]), armAngularVelocityLimit[2]);
}

void CommandLists::ArmTeleOn()
{
    sharedMemory->udp.joyCommand = CMD_ARM_TELE_ON;
    printf("(%02d:%02d:%02d) [CMD] : Arm Teleoperation On\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(1);
}

void CommandLists::ArmTeleOff()
{
    double armLinearVelocityRef[3];
    double armAngularVelocityRef[3];
    armLinearVelocityRef[0] = 0.0; // reference x-axis velocity in arm-base frame. [m/s]
    armLinearVelocityRef[1] = 0.0; // reference y-axis velocity in arm-base frame. [m/s]
    armLinearVelocityRef[2] = 0.0; // reference z-axis velocity in arm-base frame. [m/s]
    armAngularVelocityRef[0] = 0.0; // reference x-axis angular velocity in arm-base frame. [rad/s]
    armAngularVelocityRef[1] = 0.0; // reference y-axis angular velocity in arm-base frame. [rad/s]
    armAngularVelocityRef[2] = 0.0; // reference z-axis angular velocity in arm-base frame. [rad/s]
    SetArmTeleoperationVelocity(armLinearVelocityRef,armAngularVelocityRef);
    sharedMemory->udp.joyCommand = CMD_ARM_TELE_OFF;
    printf("(%02d:%02d:%02d) [CMD] : Arm Teleoperation Off\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(1);
}

void CommandLists::ArmGripperClose()
{
    sharedMemory->udp.joyCommand = CMD_ARM_GRP_CLOSE;
    printf("(%02d:%02d:%02d) [CMD] : Arm Gripper Close\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(2);
}

void CommandLists::ArmGripperOpen()
{
    sharedMemory->udp.joyCommand = CMD_ARM_GRP_OPEN;
    printf("(%02d:%02d:%02d) [CMD] : Arm Gripper Open\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    sleep(2);
}
