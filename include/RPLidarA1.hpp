//
// Created by jh on 25. 1. 27.
//

#ifndef RPLIDARA1_HPP
#define RPLIDARA1_HPP

#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <iomanip>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include "SharedMemory.hpp"

using namespace sl;

class RPLidarA1
{
public:
    RPLidarA1();
    ~RPLidarA1();
    void ReadLidarPoint();
    void Initialize();

private:

private:
    SharedMemory* sharedMemory;

    sl_result op_result;
    ILidarDriver * drv;
    const char* argv[5];
    const char* opt_is_channel;
    const char* opt_channel;
    const char* opt_channel_param_first;
    sl_u32 opt_channel_param_second;
    sl_u32 baudrateArray[2];
    bool useArgcBaudrate;
    int opt_channel_type;
    bool mbInitialized;
    std::chrono::time_point<std::chrono::_V2::system_clock> mCurrentTime;
    std::chrono::time_point<std::chrono::_V2::system_clock> mPrevTime;
    std::chrono::duration<double> mTimeInterval;
    double mParsingHz;
};



#endif //RPLIDARA1_HPP
