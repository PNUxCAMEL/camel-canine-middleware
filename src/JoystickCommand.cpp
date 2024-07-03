#include "JoystickCommand.hpp"

JoystickCommand::JoystickCommand()
{
    gamepad = Gamepad::getInstance();
    sharedMemory = SharedMemory::getInstance();
}

void JoystickCommand::copyJoystickData()
{
#ifdef ONEXPLAYER
    mJoystickAxis[0] = gamepad->joy_axis[0];                                // LeftStickX
    mJoystickAxis[1] = gamepad->joy_axis[1];                                // LeftStickY
    mJoystickAxis[2] = gamepad->joy_axis[2];                                // LeftTrigger
    mJoystickAxis[3] = gamepad->joy_axis[3];                                // RightStickX
    mJoystickAxis[4] = gamepad->joy_axis[4];                                // RightStickY
    mJoystickAxis[5] = gamepad->joy_axis[5];                                // RightTrigger
    mJoystickAxis[6] = gamepad->joy_axis[6];                                // DpadX
    mJoystickAxis[7] = gamepad->joy_axis[7];                                // DpadY
    mJoystickButton[0] = gamepad->joy_button[0];                            // FaceButtonA
    mJoystickButton[1] = gamepad->joy_button[1];                            // FaceButtonB
    mJoystickButton[2] = gamepad->joy_button[2];                            // FaceButtonY
    mJoystickButton[3] = gamepad->joy_button[3];                            // FaceButtonX
    mJoystickButton[4] = gamepad->joy_button[4];                            // LeftBumper
    mJoystickButton[5] = gamepad->joy_button[5];                            // RightBumper
    mJoystickButton[6] = gamepad->joy_button[6];                            // Back
    mJoystickButton[7] = gamepad->joy_button[7];                            // Start
    mJoystickButton[8] = gamepad->joy_button[8];                            // Guide
    mJoystickButton[9] = gamepad->joy_button[9];                            // LeftStick
    mJoystickButton[10] = gamepad->joy_button[10];                          // RightStick
#endif
#ifdef STEAMDECK
    mJoystickAxis[0] = gamepad->joy_axis[0];                                // LeftStickX
    mJoystickAxis[1] = gamepad->joy_axis[1];                                // LeftStickY
    mJoystickAxis[2] = gamepad->joy_axis[2];                                // LeftTrigger
    mJoystickAxis[3] = gamepad->joy_axis[3];                                // RightStickX
    mJoystickAxis[4] = gamepad->joy_axis[9];                                // RightStickY
    mJoystickAxis[5] = gamepad->joy_axis[8];                                // RightTrigger
    mJoystickAxis[6] = gamepad->joy_button[19] - gamepad->joy_button[18];   // DpadX
    mJoystickAxis[7] = gamepad->joy_button[17] - gamepad->joy_button[16];   // DpadY
    mJoystickButton[0] = gamepad->joy_button[3];                            // FaceButtonA
    mJoystickButton[1] = gamepad->joy_button[4];                            // FaceButtonB
    mJoystickButton[2] = gamepad->joy_button[5];                            // FaceButtonY
    mJoystickButton[3] = gamepad->joy_button[6];                            // FaceButtonX
    mJoystickButton[4] = gamepad->joy_button[7];                            // LeftBumper
    mJoystickButton[5] = gamepad->joy_button[8];                            // RightBumper
    mJoystickButton[6] = gamepad->joy_button[11];                           // Back
    mJoystickButton[7] = gamepad->joy_button[12];                           // Start
    //mJoystickButton[8] = gamepad->joy_button[8];;                         // Guide
    mJoystickButton[9] = gamepad->joy_button[14];;                          // LeftStick
    mJoystickButton[10] = gamepad->joy_button[15];                          // RightStick
#endif

#ifdef DUALSENSE

    mJoystickAxis[0] = gamepad->joy_axis[0];                                // LeftStickX
    mJoystickAxis[1] = gamepad->joy_axis[1];                                // LeftStickY
    mJoystickAxis[2] = gamepad->joy_axis[2];                                // LeftTrigger
    mJoystickAxis[3] = gamepad->joy_axis[3];                                // RightStickX
    mJoystickAxis[4] = gamepad->joy_axis[4];                                // RightStickY
    mJoystickAxis[5] = gamepad->joy_axis[5];                                // RightTrigger
    mJoystickAxis[6] = gamepad->joy_axis[6];                                // DpadX
    mJoystickAxis[7] = gamepad->joy_axis[7];                                // DpadY
    mJoystickButton[0] = gamepad->joy_button[0];                            // FaceButtonA
    mJoystickButton[1] = gamepad->joy_button[1];                            // FaceButtonB
    mJoystickButton[2] = gamepad->joy_button[2];                            // FaceButtonY
    mJoystickButton[3] = gamepad->joy_button[3];                            // FaceButtonX
    mJoystickButton[4] = gamepad->joy_button[4];                            // LeftBumper
    mJoystickButton[5] = gamepad->joy_button[5];                            // RightBumper
    mJoystickButton[6] = gamepad->joy_button[8];                            // Back
    mJoystickButton[7] = gamepad->joy_button[9];                            // Start
    mJoystickButton[8] = gamepad->joy_button[10];                           // Guide
    mJoystickButton[9] = gamepad->joy_button[11];                           // LeftStick
    mJoystickButton[10] = gamepad->joy_button[12];                          // RightStick
#endif

    if (mJoystickAxis[7] > 0)
    {
        mJoystickLeftButtons[0] = true;
    }
    else
    {
        mJoystickLeftButtons[0] = false;
    }

    if (mJoystickAxis[6] > 0)
    {
        mJoystickLeftButtons[1] = true;
    }
    else
    {
        mJoystickLeftButtons[1] = false;
    }

    if (mJoystickAxis[6] < 0)
    {
        mJoystickLeftButtons[2] = true;
    }
    else
    {
        mJoystickLeftButtons[2] = false;
    }

    if (mJoystickAxis[7] < 0)
    {
        mJoystickLeftButtons[3] = true;
    }
    else
    {
        mJoystickLeftButtons[3] = false;
    }

    //Normalizing Axis Value (-30767 ~ 30767 -> -1 ~ 1)
    mJoystickLeftAxis[0] = (double)mJoystickAxis[0] / 30767.0;
    mJoystickLeftAxis[1] = -(double)mJoystickAxis[1] / 30767.0;
    mJoystickRightAxis[0] = (double)mJoystickAxis[3] / 30767.0;
    mJoystickRightAxis[1] = -(double)mJoystickAxis[4] / 30767.0;
    // A, B, X, Y
    mJoystickRightButtons[0] = mJoystickButton[0];
    mJoystickRightButtons[1] = mJoystickButton[1];
    mJoystickRightButtons[2] = mJoystickButton[2];
    mJoystickRightButtons[3] = mJoystickButton[3];
    //LB, RB
    mJoystickRearButtons[0] = mJoystickButton[4];
    mJoystickRearButtons[1] = mJoystickButton[5];
    //LT, RT
    if (mJoystickAxis[2] > 0)
    {
        mJoystickRearButtons[2] = true;
    }
    else
    {
        mJoystickRearButtons[2] = false;
    }
    if (mJoystickAxis[5] > 0)
    {
        mJoystickRearButtons[3] = true;
    }
    else
    {
        mJoystickRearButtons[3] = false;
    }
    //Start, End
    mJoystickFunctionButtons[0] = mJoystickButton[6];
    mJoystickFunctionButtons[1] = mJoystickButton[7];

    mappingJoystickCommand();
}

void JoystickCommand::mappingJoystickCommand()
{
    sharedMemory->UDPCommand = CMD_NO_ACT;
//    printf("[CMD] %d\n",joystickCommand);
    mappingStart();
    mappingEmergencyStop();
    mappingStandUp();
    mappingStandDown();
    mappingMotorOff();
    mappingConstStand();
    mappingTrotSlow();
    mappingTrotFast();
    mappingTrotOverlap();
    mappingSlopeMode();
    mappingRecovery();
    mappingJoystick();
    mappingRestart();
}

void JoystickCommand::mappingStart()
{
    if (mJoystickFunctionButtons[1])
    {
        sharedMemory->UDPCommand = CMD_START;
    }
}

void JoystickCommand::mappingEmergencyStop()
{
    if ((mJoystickRearButtons[2] && mJoystickRearButtons[3]))
    {
        sharedMemory->UDPCommand = CMD_E_STOP;
    }
}

void JoystickCommand::mappingStandUp()
{
    if (mJoystickRearButtons[3] && mJoystickLeftButtons[3])
    {
        sharedMemory->UDPCommand = CMD_HOME_UP;
    }
}

void JoystickCommand::mappingStandDown()
{
    if ((mJoystickRearButtons[3] && mJoystickLeftButtons[0]))
    {
        sharedMemory->UDPCommand = CMD_HOME_DOWN;
    }
}

void JoystickCommand::mappingMotorOff()
{

}

void JoystickCommand::mappingConstStand()
{
    if ((mJoystickRearButtons[1] && mJoystickLeftButtons[1]))
    {
        sharedMemory->UDPCommand = CMD_TROT_STOP;
    }
}

void JoystickCommand::mappingTrotSlow()
{
    if ((mJoystickRearButtons[1] && mJoystickLeftButtons[2]))
    {
        sharedMemory->UDPCommand = CMD_TROT_SLOW;
    }
}

void JoystickCommand::mappingTrotFast()
{
    if ((mJoystickRearButtons[1] && mJoystickLeftButtons[0]))
    {
        sharedMemory->UDPCommand = CMD_TROT_FAST;

    }
}

void JoystickCommand::mappingTrotOverlap()
{
    if ((mJoystickRearButtons[1] && mJoystickLeftButtons[3]))
    {
        sharedMemory->UDPCommand = CMD_TROT_OVERLAP;
    }
}

void JoystickCommand::mappingRecovery()
{
    if (mJoystickButton[1])
    {
        sharedMemory->UDPCommand = CMD_RECOVERY;
    }
}

void JoystickCommand::mappingSlopeMode()
{
    if ((mJoystickButton[0] && !mbPrevSlopeInput))
    {
//        sharedMemory->UDPCommand = GAMEPAD_MODE_SLOPE;
    }
}

void JoystickCommand::mappingRestart()
{
//    if (sharedParameter->isRobotRestart)
//    {
//        sharedMemory->UDPCommand = GAMEPAD_RESTART;
//    }
}

void JoystickCommand::mappingJoystick()
{
    switch (sharedMemory->FSMState)
    {
    case FSM_CONST_STAND:
    {
        sharedMemory->UDPRefBodyLinearVelocity_x = 0.0;
        sharedMemory->UDPRefBodyLinearVelocity_y = 0.0;
        sharedMemory->UDPRefBodyAngularVelocity_yaw = 0.0;
        break;
    }
    case FSM_TROT_SLOW:
    {
        sharedMemory->UDPRefBodyLinearVelocity_x = mJoystickLeftAxis[1] * 0.6;
        sharedMemory->UDPRefBodyLinearVelocity_y = -mJoystickLeftAxis[0] * 0.4;
        sharedMemory->UDPRefBodyAngularVelocity_yaw = -mJoystickRightAxis[0] * 0.65;
        break;
    }
    case FSM_TROT_FAST:
    {
        sharedMemory->UDPRefBodyLinearVelocity_x = mJoystickLeftAxis[1] * 1.0; // 1 m/s
        sharedMemory->UDPRefBodyLinearVelocity_y = -mJoystickLeftAxis[0] * 0.5;
        sharedMemory->UDPRefBodyAngularVelocity_yaw = -mJoystickRightAxis[0] * 0.65;
        break;
    }
    case FSM_OVERLAP_TROT_FAST:
    {
        sharedMemory->UDPRefBodyLinearVelocity_x = mJoystickLeftAxis[1] * 1.0;
        sharedMemory->UDPRefBodyAngularVelocity_yaw = -mJoystickRightAxis[0] * 0.65;
        break;
    }
    default:
        break;

    }
}