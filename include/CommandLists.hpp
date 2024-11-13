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
    void HomeUp();
    void HomeDown();
    void TrotSlow();
    void TrotStop();
    void EmergencyStop();
    void ArmHome();
    void ArmMove(double* goalPosition, double* goalEulerAngle);
    void ArmTeleOn();
    void ArmTeleOff();
    void ArmGripperOpen();
    void ArmGripperClose();
    void SetBodyVelocity(double* refVel);
    void SetArmTeleoperationVelocity(double* armLinearVelocityRef, double* armAngularVelocityRef);

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
        CMD_ARM_HOME,
        CMD_ARM_MOVE,
        CMD_ARM_TELE_ON,
        CMD_ARM_TELE_OFF,
        CMD_ARM_GRP_OPEN,
        CMD_ARM_GRP_CLOSE
    };
};


#endif //CAMEL_CONSOLE_GAMEPADDESCRIPTION_HPP
