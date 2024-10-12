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
    udp.joyCommand = int8_t(0);
    udp.userAngVel.setZero();
    udp.userLinVel.setZero();

    for (int i = 0; i < MPC_HORIZON * 4; i++)
    {
        gaitTable[i] = 1;
    }

    motorStatus = false;

    FSMState = 0;

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
    bodyBaseVelocity.setZero();
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
    globalBaseEulerAngle.setZero();
    bodyBaseAngularVelocity.setZero();
    globalBaseAcceleration.setZero();

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
        solvedGRF[index].setZero();
        contactResidualTorque[index] = 0.0;
    }

    for (int index = 0; index < 11; index++)
    {
        threadElapsedTime[index] = 0.0;
    }

    isTCPConnected = false;
}