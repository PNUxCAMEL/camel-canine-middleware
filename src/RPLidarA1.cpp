//
// Created by jh on 25. 1. 27.
//

#include "RPLidarA1.hpp"


#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


static inline void delay(sl_word_size_t ms)
{
    while (ms >= 1000)
    {
        usleep(1000 * 1000);
        ms -= 1000;
    };
    if (ms != 0)
        usleep(ms * 1000);
}

bool checkSLAMTECLIDARHealth(ILidarDriver* drv)
{
    sl_result op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result))
    {
        // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("[RPLIDAR] SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR)
        {
            fprintf(stderr, "[RPLIDAR] Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        fprintf(stderr, "[RPLIDAR] Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

RPLidarA1::RPLidarA1()
{
    sharedMemory = SharedMemory::getInstance();

    argv[0] = "./code";
    argv[1] = "--channel";
    argv[2] = "--serial";
    argv[3] = "/dev/ttyUSB0"; // Serial port name
    argv[4] = "115200"; // Baudrate

    opt_is_channel = NULL;
    opt_channel = NULL;
    opt_channel_param_first = NULL;
    opt_channel_param_second = 0;

    baudrateArray[0] = 115200;
    baudrateArray[1] = 256000;
    opt_channel_type = CHANNEL_TYPE_SERIALPORT;

    useArgcBaudrate = false;

    mCurrentTime = std::chrono::high_resolution_clock::now();
    mPrevTime = std::chrono::high_resolution_clock::now();
}

RPLidarA1::~RPLidarA1()
{
    if (drv)
    {
        drv->stop();
        delay(200);
        if (opt_channel_type == CHANNEL_TYPE_SERIALPORT)
            drv->setMotorSpeed(0);
        delete drv;
        drv = NULL;
    }
}

void RPLidarA1::Initialize()
{
    IChannel* _channel;

    opt_is_channel = argv[1];


    if (strcmp(opt_is_channel, "--channel") == 0)
    {
        opt_channel = argv[2];
        if (strcmp(opt_channel, "-s") == 0 || strcmp(opt_channel, "--serial") == 0)
        {
            // read serial port from the command line...
            opt_channel_param_first = argv[3]; // or set to a fixed value: e.g. "com3"
            // read baud rate from the command line if specified...

            opt_channel_param_second = strtoul(argv[4], NULL, 10);
            useArgcBaudrate = true;
        }
        else if (strcmp(opt_channel, "-u") == 0 || strcmp(opt_channel, "--udp") == 0)
        {
            // read ip addr from the command line...
            opt_channel_param_first = argv[3]; //or set to a fixed value: e.g. "192.168.11.2"

            opt_channel_param_second = strtoul(argv[4], NULL, 10); //e.g. "8089"
            opt_channel_type = CHANNEL_TYPE_UDP;
        }
    }

    if (opt_channel_type == CHANNEL_TYPE_SERIALPORT)
    {
        if (!opt_channel_param_first)
        {
            opt_channel_param_first = argv[3];
        }
    }


    // create the driver instance
    drv = *createLidarDriver();

    if (!drv)
    {
        fprintf(stderr, "[RPLIDAR] insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    if (opt_channel_type == CHANNEL_TYPE_SERIALPORT)
    {
        if (useArgcBaudrate)
        {
            _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
            if (SL_IS_OK((drv)->connect(_channel)))
            {
                op_result = drv->getDeviceInfo(devinfo);

                if (SL_IS_OK(op_result))
                {
                    connectSuccess = true;
                }
                else
                {
                    delete drv;
                    drv = NULL;
                }
            }
        }
        else
        {
            size_t baudRateArraySize = (sizeof(baudrateArray)) / (sizeof(baudrateArray[0]));
            for (size_t i = 0; i < baudRateArraySize; ++i)
            {
                _channel = (*createSerialPortChannel(opt_channel_param_first, baudrateArray[i]));
                if (SL_IS_OK((drv)->connect(_channel)))
                {
                    op_result = drv->getDeviceInfo(devinfo);

                    if (SL_IS_OK(op_result))
                    {
                        connectSuccess = true;
                        break;
                    }
                    else
                    {
                        delete drv;
                        drv = NULL;
                    }
                }
            }
        }
    }
    else if (opt_channel_type == CHANNEL_TYPE_UDP)
    {
        _channel = *createUdpChannel(opt_channel_param_first, opt_channel_param_second);
        if (SL_IS_OK((drv)->connect(_channel)))
        {
            op_result = drv->getDeviceInfo(devinfo);

            if (SL_IS_OK(op_result))
            {
                connectSuccess = true;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }


    if (!connectSuccess)
    {
        (opt_channel_type == CHANNEL_TYPE_SERIALPORT)
            ? (fprintf(stderr, "[RPLIDAR] Error, cannot bind to the specified serial port %s.\n"
                       , opt_channel_param_first))
            : (fprintf(stderr, "[RPLIDAR] Error, cannot connect to the specified ip addr %s.\n"
                       , opt_channel_param_first));
    }

    // print out the device serial number, firmware and hardware version number..
    printf("[RPLIDAR] SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16; ++pos)
    {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
           "[RPLIDAR] Firmware Ver: %d.%02d\n"
           "[RPLIDAR] Hardware Rev: %d\n"
           , devinfo.firmware_version >> 8
           , devinfo.firmware_version & 0xFF
           , (int)devinfo.hardware_version);


    // check health...
    if (!checkSLAMTECLIDARHealth(drv))
    {
        mbInitialized = false;
    }
    else
    {
        mbInitialized = true;
    }

    if (opt_channel_type == CHANNEL_TYPE_SERIALPORT)
        drv->setMotorSpeed();


    // drv->startScanExpress(0,1);
    drv->startScan(0, 1);
}

void RPLidarA1::ReadLidarPoint()
{
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);

    op_result = drv->grabScanDataHq(nodes, count);
    if (SL_IS_OK(op_result))
    {
        int data_idx = 0;
        mCurrentTime = std::chrono::high_resolution_clock::now();
        mTimeInterval = mCurrentTime - mPrevTime;
        mParsingHz = 1.0 / mTimeInterval.count();
        drv->ascendScanData(nodes, count);
        sharedMemory->lidarData.scanSize = (int)count;

        for (int index = 0; index < 8192; index++)
        {
            sharedMemory->lidarData.scanAngle[index] = 0.0;
            sharedMemory->lidarData.scanDistance[index] = 0.0;
        }

        for (int pos = 0; pos < (int)count; ++pos)
        {
            // printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
            //        (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "  ",
            //        (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
            //        nodes[pos].dist_mm_q2 / 4.0f,
            //        nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            int quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
            // if (quality == 0)
            // {
            //     data_idx--;
            //     sharedMemory->lidarData.scanSize--;
            // }
            // else
            // {
            //     sharedMemory->lidarData.scanAngle[data_idx] = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
            //     sharedMemory->lidarData.scanDistance[data_idx] = nodes[pos].dist_mm_q2 / 4.0f / 1000.0f;
            // }
            sharedMemory->lidarData.scanAngle[data_idx] = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
            sharedMemory->lidarData.scanDistance[data_idx] = nodes[pos].dist_mm_q2 / 4.0f / 1000.0f;
            data_idx++;
        }
        // std::cout << "total num. of point : " << count << std::endl;
        // std::cout << "good-Q num : " << sharedMemory->lidarData.scanSize << std::endl;
        // std::cout << "parsing Hz : " << mParsingHz << std::endl;
        for (int pos = 0; pos < sharedMemory->lidarData.scanSize; ++pos)
        {
            // std::cout<<"theta : "<<sharedMemory->lidarData.scanAngle[pos]<<"\tDist : "<<sharedMemory->lidarData.scanDistance[pos]<<std::endl;
        }
        std::cout<<"theta : "<<sharedMemory->lidarData.scanAngle[0]<<"\tDist : "<<sharedMemory->lidarData.scanDistance[0]<<std::endl;


        mPrevTime = mCurrentTime;
    }
    else
    {
        std::cerr<<"[RPLIDAR] Error reading lidar data."<<std::endl;
        // drv->reset();
        // delete drv;
        // drv = NULL;
        // this->Initialize();
    }
}
