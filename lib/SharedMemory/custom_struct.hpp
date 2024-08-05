//
// Created by ys on 24. 2. 16.
//

#ifndef RBCANINE_CUSTOM_STRUCT_HPP
#define RBCANINE_CUSTOM_STRUCT_HPP

#include<Eigen/Dense>
#include"JoystickInfo.hpp"

#define MOTOR_NUM 12
#define MAX_COMMAND_DATA 10
#define MAX_CUSTOM_DATA     20

typedef struct _GAMEPAD_
{
    AXIS joystick;
    BUTTON button;
    GUI_BUTTON gui;
    Eigen::Vector3d userLinVel;
    Eigen::Vector3d userAngVel;
    bool newCommand;
    uint8_t joyCommand;
    uint8_t state;
} GAMEPAD, * pGAMEPAD;

typedef struct _UI_COMMAND_
{
    int userCommand;
    int gaitCommand;
    char userParamChar[MAX_COMMAND_DATA];
    int userParamInt[MAX_COMMAND_DATA];
    double userParamDouble[MAX_COMMAND_DATA];
} UI_COMMAND, * pUI_COMMAND;

typedef struct _CUSTOM_DATA_
{
    double customVariableDouble[MAX_CUSTOM_DATA];
    int customVariableInt[MAX_CUSTOM_DATA];
} CUSTOM_DATA, * pCUSTOM_DATA;

#endif //RBCANINE_CUSTOM_STRUCT_HPP
