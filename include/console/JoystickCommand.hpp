#ifndef RAISIM_COMMAND_H
#define RAISIM_COMMAND_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include "ConsoleType.hpp"
#include "GamepadDescription.hpp"
#include "Gamepad.h"
#include "SharedMemory.hpp"


class JoystickCommand
{
public:
    JoystickCommand();
    void copyJoystickData();
    void mappingJoystickCommand();

    double bodyLinVel_ref[3];
    double bodyAngVel_ref[3];
    int8_t joystickCommand;
    int FSMState;

private:
    void mappingStart();
    void mappingEmergencyStop();
    void mappingStandUp();
    void mappingStandDown();
    void mappingMotorOff();
    void mappingConstStand();
    void mappingTrotSlow();
    void mappingTrotFast();
    void mappingTrotOverlap();
    void mappingRecovery();
    void mappingSlopeMode();
    void mappingRestart();
    void mappingJoystick();

private:
    bool mJoystickButton[11];
    int mJoystickAxis[8];

    bool mJoystickRightButtons[4]; // (A, B, X, Y) (X, O, nemo, semo)
    bool mJoystickLeftButtons[4]; // down, right, left, up
    bool mJoystickRearButtons[4]; // L1, R1, L2, R2
    bool mJoystickFunctionButtons[2]; // option, start
    double mJoystickLeftAxis[2]; // left-right, down-up
    double mJoystickRightAxis[2]; // left-right, down-up
    bool mbPrevSlopeInput;
    bool mbPrevSlopeSelected;

    int mIsolation;
    Gamepad* gamepad;
    SharedMemory* sharedMemory;

    enum ISOLATION
    {
        ISOLATION_STAND,
        ISOLATION_READY
    };

};


#endif //RAISIM_COMMAND_H
