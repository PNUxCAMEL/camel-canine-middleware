//
// Created by ys on 24. 2. 16.
//

#ifndef RBCANINE_SHAREDMEMORY_HPP
#define RBCANINE_SHAREDMEMORY_HPP

#include <Eigen/Dense>
#include "custom_struct.hpp"
#include "EnumClasses.hpp"
#define MPC_HORIZON         5

typedef class _HWD_
{
public:
    _HWD_(const _HWD_&) = delete;
    _HWD_& operator=(const _HWD_&) = delete;
    static _HWD_* getInstance();
private:
    _HWD_();
public:
    GAMEPAD gamepad;
} HWD, * pHWD;

typedef class SharedMemory
{
public:
    SharedMemory(const SharedMemory&) = delete;
    SharedMemory& operator=(const SharedMemory&) = delete;

    static SharedMemory* getInstance();
private:
    SharedMemory();
public:
    /// UDP: Command
    int8_t UDPCommand;

    /// UDP: Command velocity
    double UDPRefBodyLinearVelocity_x;
    double UDPRefBodyLinearVelocity_y;
    double UDPRefBodyAngularVelocity_yaw;

    /// Motor states
    double motorPosition[MOTOR_NUM];
    double motorVelocity[MOTOR_NUM];
    double motorTorque[MOTOR_NUM];
    double motorVoltage[MOTOR_NUM];
    int motorTemp[MOTOR_NUM];
    int motorErrorStatus[MOTOR_NUM];
    double motorDesiredPosition[MOTOR_NUM];
    double motorDesiredVelocity[MOTOR_NUM];
    double motorDesiredTorque[MOTOR_NUM];

    /// State estimator: global frame
    Eigen::Vector3d globalBasePosition;
    Eigen::Vector3d globalBaseVelocity;

    /// State estimator: body frame
    Eigen::Vector3d bodyBaseVelocity;
    Eigen::Vector3d bodyBase2FootPosition[4];
    Eigen::Vector3d bodyBase2FootVelocity[4];

    /// State estimator: contact estimator
    bool contactState[4];

    /// IMU
    Eigen::Vector4d globalBaseQuaternion;
    Eigen::Vector3d globalBaseEulerAngle;
    Eigen::Vector3d globalBaseAngularVelocity;
    Eigen::Vector3d bodyBaseAngularVelocity;

    //Control params
    Eigen::Vector3d globalBaseDesiredPosition;
    Eigen::Vector3d globalBaseDesiredVelocity;
    Eigen::Vector4d globalBaseDesiredQuaternion;
    Eigen::Vector3d globalBaseDesiredEulerAngle;
    Eigen::Vector3d globalBaseDesiredAngularVelocity;
    Eigen::Vector3d bodyBaseDesiredAngularVelocity;
    Eigen::Vector3d bodyBaseDesiredVelocity;
    Eigen::Vector3d globalBaseAcceleration; // TODO: acc. states will be deleted.
    Eigen::Vector3d pdTorque[4];
    Eigen::Vector3d mpcTorque[4];
    Eigen::Vector3d globalFootPosition[4];
    Eigen::Vector3d bodyBase2FootDesiredPosition[4];
    Eigen::Vector3d bodyBase2FootDesiredVelocity[4];
    Eigen::Vector3d solvedGRF[4];

    /// For debug
    UI_COMMAND command;
    bool isNan;
    bool isRamp;
    bool newCommand;
    bool motorStatus;
    int FSMState;
    int LowControlState;
    int HighControlState;
    int gaitTable[MPC_HORIZON*4];
    double testBasePos[3];
    double testBaseVel[3];
    double threadElapsedTime[11];
    double localTime;

} SHM, * pSHM;


#endif //RBCANINE_SHAREDMEMORY_HPP
