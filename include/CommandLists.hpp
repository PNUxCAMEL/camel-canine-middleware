//
// Created by camel on 22. 9. 21.
//

#ifndef CAMEL_CONSOLE_GAMEPADDESCRIPTION_HPP
#define CAMEL_CONSOLE_GAMEPADDESCRIPTION_HPP

#include "SharedMemory.hpp"
#include "EnumClasses.hpp"
#include <unistd.h>

class CommandLists
{
public:
    CommandLists();
    void NoCommand();
    void Start();
    void Restart();
    void StandUp();
    void SitDown();
    void TrotSlow();
    void TrotStop();
    void TrotForceStop();
    void EmergencyStop();
    void SetBodyVelocity(double refVelX, double refVelY, double refVelYaw);

private:
    SharedMemory* sharedMemory;
    enum COMMAND
    {
        CMD_NO_INPUT,
        CMD_START,
        CMD_EMERGENCY_STOP,
        CMD_STAND_UP,
        CMD_SIT_DOWN,
        CMD_TROT_STOP,
        CMD_TROT_SLOW,
        CMD_TROT_FAST,
        CMD_TROT_OVERLAP,
        CMD_RESTART,
        CMD_FOOT2WHEEL_CHANGE,
        CMD_WHEEL2FOOT_CHANGE,
        CMD_WHEEL_MOVE,
        CMD_WHEEL_STAND
    };
};


#endif //CAMEL_CONSOLE_GAMEPADDESCRIPTION_HPP
