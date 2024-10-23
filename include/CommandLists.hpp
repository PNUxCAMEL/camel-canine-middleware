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
    void ArmHome();
    void ArmMove(double* goalPosition, double* goalEulerAngle);
    void ArmTeleoperation(double* armLinearVelocityRef, double* armAngularVelocityRef);
    void ArmGripperOpen();
    void ArmGripperClose();
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
        CMD_E_STOP,
        CMD_ARM_HOME,
        CMD_ARM_MOVE,
        CMD_ARM_TELE,
        CMD_ARM_GRIPPER_OPEN,
        CMD_ARM_GRIPPER_CLOSE
    };
};


#endif //CAMEL_CONSOLE_GAMEPADDESCRIPTION_HPP
