//
// Created by ys on 24. 5. 6.
//

#include "SharedMemory.hpp"


_HWD_* _HWD_::getInstance()
{
    static _HWD_ instace;
    return &instace;
}
_HWD_::_HWD_()
{
    for (int idx = 0; idx < MOTOR_NUM; idx++)
    {
        sensor.motor[idx].vel = 0;
        sensor.motor[idx].pos = 0;
        sensor.motor[idx].torque = 0;
        sensor.motor[idx].temp = 0;
        motorDesiredTorque[idx] = 0;
    }
    for (int idx = 0; idx < 11; idx++)
    {
        threadElapsedTime[idx] = 0;
    }

    sensor.imu.acc.setZero();
    sensor.imu.gyro.setZero();
    sensor.imu.quat.setZero();
    sensor.imu.rpy.setZero();
    sensor.imu.imu_offset_pitch = 0;
    sensor.imu.imu_offset_roll = 0;

    gamepad.gui.isGUISlope = false;
    gamepad.gui.GUIButton = 0;

    gamepad.joystick.LeftStickX = 0;
    gamepad.joystick.LeftStickY = 0;
    gamepad.joystick.LeftTrigger = -30767;
    gamepad.joystick.RightStickX = 0;
    gamepad.joystick.RightStickY = 0;
    gamepad.joystick.LeftTrigger = -30767;
    gamepad.joystick.DpadX = 0;
    gamepad.joystick.DpadY = 0;

    gamepad.button.A = 0;
    gamepad.button.B = 0;
    gamepad.button.X = 0;
    gamepad.button.Y = 0;
    gamepad.button.LB = 0;
    gamepad.button.RB = 0;
    gamepad.button.Back = 0;
    gamepad.button.Start = 0;
    gamepad.button.Guide = 0;
    gamepad.button.LeftStick = 0;
    gamepad.button.RightStick = 0;

    motorCheckFlag = true;
    motorALLStatus = false;

    for (int idx = 0; idx < 11; idx++)
    {
        threadElapsedTime[idx] = 0.0;
    }

}

SharedMemory* SharedMemory::getInstance()
{
    static SharedMemory instace;
    return &instace;
}

SharedMemory::SharedMemory()
{
    consoleConnection = false;
    for (int i = 0; i < MPC_HORIZON * 4; i++)
    {
        gaitTable[i] = 1;
    }

    gaitState = STAND;
    gaitChangeFlag = false;
    throwFlag = false;

    isNan = false;
    isRamp = false;
    isSimulation = false;
    bIsEndHome = false;
    newCommand = false;

    canFLStatus = false;
    canFRStatus = false;
    canHLStatus = false;
    canHRStatus = false;

    motorStatus = false;
    motorFLStatus = true;
    motorFRStatus = true;
    motorHLStatus = true;
    motorHRStatus = true;

    FSMState = 0;
    LowControlState = STATE_LOW_CONTROL_STOP;
    HighControlState = STATE_HIGH_CONTROL_STOP;
    visualState = STATE_VISUAL_STOP;

    canFLState = CAN_NO_ACT;
    canFRState = CAN_NO_ACT;
    canHLState = CAN_NO_ACT;
    canHRState = CAN_NO_ACT;

    localTime = 0;

    for (int index = 0; index < MOTOR_NUM; index++)
    {
        motorErrorStatus[index] = 0;
        motorTemp[index] = 0;
        motorVoltage[index] = 0;
        motorPosition[index] = 0;
        motorVelocity[index] = 0;
        motorTorque[index] = 0;
        motorDesiredPosition[index] = 0;
        motorDesiredVelocity[index] = 0;
        motorDesiredTorque[index] = 0;
    }

    globalBasePosition.setZero();
    globalBaseVelocity.setZero();
    globalBaseQuaternion[0] = 1.0;
    globalBaseQuaternion[1] = 0.0;
    globalBaseQuaternion[2] = 0.0;
    globalBaseQuaternion[3] = 0.0;
    globalBaseDesiredPosition.setZero();
    globalBaseDesiredVelocity.setZero();
    globalBaseDesiredQuaternion[0] = 1.0;
    globalBaseDesiredQuaternion[1] = 0.0;
    globalBaseDesiredQuaternion[2] = 0.0;
    globalBaseDesiredQuaternion[3] = 0.0;
    globalBaseDesiredEulerAngle.setZero();
    bodyBaseDesiredAngularVelocity.setZero();
    bodyBaseDesiredVelocity.setZero();
    bodyBaseVelocity.setZero();
    globalBaseEulerAngle.setZero();
    bodyBaseAngularVelocity.setZero();
    globalBaseAcceleration.setZero();
    for (int index = 0; index < 3; index++)
    {
        testBasePos[index] = 0;
        testBaseVel[index] = 0;
    }

    for (int index = 0; index < 4; index++)
    {
        pdTorque[index].setZero();
        mpcTorque[index].setZero();
        globalFootPosition[index].setZero();
        bodyBase2FootPosition[index].setZero();
        bodyBase2FootVelocity[index].setZero();
        bodyBase2FootDesiredPosition[index].setZero();
        bodyBase2FootDesiredVelocity[index].setZero();
        contactState[index] = true;
        tempRes[index] = 0;
        solvedGRF[index].setZero();
        swingPgain[index] << 700.0, 700.0, 700.0;
        swingDgain[index] << 60.0, 60.0, 60.0;
        isFirstHome[index] = true;
        contactResidualTorque[index] = 0.0;
    }

    for (int index = 0; index < 13; index++)
    {
        threadElapsedTime[index] = 0.0;
    }


    for (int i = 0; i < MPC_HORIZON; i++)
    {
        mpcDesiredPos[i].setZero();
    }

    yamlParams.isFirstYamlInputFlag = true;
    yamlParams.isValueChanged = false;
    yamlParams.isYamlSave = false;

    paramChangedFlag = false;
    mpcParamChangedFlag = false;
    swingParamChangedFlag = false;
    wbcParamChangedFlag = false;

    isRobotRestart = false;

    UDPRefBodyLinearVelocity_x = 0.0;
    UDPRefBodyLinearVelocity_y = 0.0;
    UDPRefBodyAngularVelocity_yaw = 0.0;
    UDPCommand = 0;
}