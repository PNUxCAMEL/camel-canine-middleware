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
    void Recovery();
    void HomeUp();
    void HomeDown();
    void TrotFast();
    void TrotSlow();
    void TrotStop();
    void EmergencyStop();
    void SetBodyVelocity(double* refVel);

private:
    SharedMemory* sharedMemory;
    enum COMMAND
    {
        CMD_NO_ACT,
        CMD_START,
        CMD_RECOVERY,
        CMD_RESET,
        CMD_HOME_UP,
        CMD_HOME_DOWN,
        CMD_TROT_OVERLAP,
        CMD_TROT_SLOW,
        CMD_TROT_FAST,
        CMD_TROT_STOP,
        CMD_E_STOP
    };
};


#endif //CAMEL_CONSOLE_GAMEPADDESCRIPTION_HPP
