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
#include "Setup.hpp"
#include "CommandLists.hpp"
#include "EnumClasses.hpp"
#include "Gamepad.h"
#include "SharedMemory.hpp"


class JoystickCommand
{
public:
    JoystickCommand();
    void copyJoystickData();
    void mappingJoystickCommand();

private:
    void mappingStart();
    void mappingEmergencyStop();
    void mappingStandUp();
    void mappingStandDown();
    void mappingConstStand();
    void mappingTrotSlow();
    void mappingRestart();
    void mappingFoot2WheelChange();
    void mappingWheel2FootChange();
    void mappingWheelMove();
    void mappingWheelStand();
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

    Gamepad* gamepad;
    SharedMemory* sharedMemory;
    CommandLists commandLists;
};


#endif //RAISIM_COMMAND_H
