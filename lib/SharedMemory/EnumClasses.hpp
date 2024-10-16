//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_ROBOTDESCRIPTION_H
#define RAISIM_ROBOTDESCRIPTION_H


enum LEG_INDEX
{
    FL_IDX = 0,
    FR_IDX,
    HL_IDX,
    HR_IDX
};

enum HIGH_CONTROL_STATE
{
    STATE_HIGH_CONTROL_STOP,
    STATE_DEFAULT_CONTROL,
    STATE_HIGH_HOME_STAND_UP_READY,
    STATE_HIGH_HOME_STAND_DOWN_READY,
    STATE_HIGH_DO_CONTROL,
};

enum LOW_CONTROL_STATE
{
    STATE_LOW_CONTROL_STOP,
    STATE_LOW_CONTROL_START,
    STATE_LOW_HOME_STAND_UP_READY,
    STATE_LOW_HOME_STAND_DOWN_READY,
    STATE_LOW_HOME_CONTROL,
    STATE_LOW_BACK_FLIP_READY,
    STATE_LOW_BACK_FLIP_CONTROL,
    STATE_LOW_RECOVERY_READY,
    STATE_LOW_RECOVERY_CONTROL,
};

enum CAN_STATE
{
    CAN_NO_ACT,
    CAN_INIT,
    CAN_MOTOR_ON,
    CAN_MOTOR_OFF,
    CAN_SET_TORQUE,
    CAN_READ_ERROR,
    CAN_READ_ENCODER
};

enum VISUAL_STATE
{
    STATE_VISUAL_STOP,
    STATE_OPEN_RAISIM,
    STATE_UPDATE_VISUAL
};

enum GAIT_TYPE
{
    STAND = 0,
    TROT_SLOW,
    TROT_FAST,
    OVERLAP_TROT_FAST
};

enum GAIT_COMMAND
{
    DESIRED_GAIT_STAND,
    DESIRED_GAIT_TROT_SLOW,
    DESIRED_GAIT_TROT_FAST,
    DESIRED_GAIT_OVERLAP_TROT_FAST
};

enum FSM
{
    FSM_INITIAL,
    FSM_EMERGENCY_STOP,
    FSM_READY,
    FSM_ISOLATION,
    FSM_CONST_STAND,
    FSM_STAND,
    FSM_TROT_SLOW,
    FSM_TROT_FAST,
    FSM_OVERLAP_TROT_FAST
};

#endif //RAISIM_ROBOTDESCRIPTION_H
