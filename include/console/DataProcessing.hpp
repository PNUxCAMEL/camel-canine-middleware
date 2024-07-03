//
// Created by ys on 24. 6. 9.
//

#ifndef RBQ_CONSOLE_DATAPROCESSING_HPP
#define RBQ_CONSOLE_DATAPROCESSING_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "SharedMemory.hpp"


uint64_t htonll(uint64_t value) {
    if (__BYTE_ORDER == __LITTLE_ENDIAN) {
        return ((uint64_t)htonl(value & 0xFFFFFFFF) << 32) | htonl(value >> 32);
    } else {
        return value;
    }
}

uint64_t ntohll(uint64_t value) {
    if (__BYTE_ORDER == __LITTLE_ENDIAN) {
        return ((uint64_t)ntohl(value & 0xFFFFFFFF) << 32) | ntohl(value >> 32);
    } else {
        return value;
    }
}

// Converts a double to network byte order
uint64_t htond(double value) {
    uint64_t temp;
    memcpy(&temp, &value, sizeof(temp));
    temp = htonll(temp);
    return temp;
}

// Converts a network byte order double to host byte order
double ntohd(uint64_t value) {
    value = ntohll(value);
    double temp;
    memcpy(&temp, &value, sizeof(temp));
    return temp;
}

void unpackingEigenVector3d(const char*& dataPtr, Eigen::Vector3d& vec) {
    for (int i = 0; i < 3; ++i) {
        uint64_t networkOrder;
        memcpy(&networkOrder, dataPtr, sizeof(networkOrder));
        vec[i] = ntohd(networkOrder);
        dataPtr += sizeof(networkOrder);
    }
}

void unpackingEigenVector4d(const char*& dataPtr, Eigen::Vector4d& vec) {
    for (int i = 0; i < 4; ++i) {
        uint64_t networkOrder;
        memcpy(&networkOrder, dataPtr, sizeof(networkOrder));
        vec[i] = ntohd(networkOrder);
        dataPtr += sizeof(networkOrder);
    }
}

void unpackingTCPmsg(const std::vector<char>& dataBuffer) {
    const char* dataPtr = dataBuffer.data();
    SharedMemory* sharedMemory = SharedMemory::getInstance();

    // Unpack sharedMemory->userCommand
    int userCommandNetworkOrder;
    memcpy(&userCommandNetworkOrder, dataPtr, sizeof(userCommandNetworkOrder));
    sharedMemory->command.userCommand = ntohl(userCommandNetworkOrder);
    dataPtr += sizeof(userCommandNetworkOrder);


    // Unpack sharedMemory->gaitCommand
    int gaitCommandNetworkOrder;
    memcpy(&gaitCommandNetworkOrder, dataPtr, sizeof(gaitCommandNetworkOrder));
    sharedMemory->command.gaitCommand = ntohl(gaitCommandNetworkOrder);
    dataPtr += sizeof(gaitCommandNetworkOrder);

    // Unpack sharedMemory->userParamInt
    for (int i = 0; i < MAX_COMMAND_DATA; ++i)
    {
        int userParamIntNetworkOrder;
        memcpy(&userParamIntNetworkOrder, dataPtr, sizeof(userParamIntNetworkOrder));
        sharedMemory->command.userParamInt[i] = ntohl(userParamIntNetworkOrder);
        dataPtr += sizeof(userParamIntNetworkOrder);
    }

    // Unpack sharedMemory->userParamDouble
    for (int i = 0; i < MAX_COMMAND_DATA; ++i)
    {
        uint64_t userParamDoubleNetworkOrder;
        memcpy(&userParamDoubleNetworkOrder, dataPtr, sizeof(userParamDoubleNetworkOrder));
        sharedMemory->command.userParamDouble[i] = ntohd(userParamDoubleNetworkOrder);
        dataPtr += sizeof(userParamDoubleNetworkOrder);
    }

    // Unpack sharedMemory->gaitTable
    for (int i = 0; i < MPC_HORIZON * 4; ++i)
    {
        int gaitTableNetworkOrder;
        memcpy(&gaitTableNetworkOrder, dataPtr, sizeof(gaitTableNetworkOrder));
        sharedMemory->gaitTable[i] = ntohl(gaitTableNetworkOrder);
        dataPtr += sizeof(gaitTableNetworkOrder);
    }

    // Unpack remaining boolean and integer fields in sharedMemory
    memcpy(&sharedMemory->isNan, dataPtr, sizeof(sharedMemory->isNan));
    dataPtr += sizeof(sharedMemory->isNan);

    memcpy(&sharedMemory->isRamp, dataPtr, sizeof(sharedMemory->isRamp));
    dataPtr += sizeof(sharedMemory->isRamp);

    memcpy(&sharedMemory->newCommand, dataPtr, sizeof(sharedMemory->newCommand));
    dataPtr += sizeof(sharedMemory->newCommand);

    memcpy(&sharedMemory->motorStatus, dataPtr, sizeof(sharedMemory->motorStatus));
    dataPtr += sizeof(sharedMemory->motorStatus);

    int stateNetworkOrder;
    memcpy(&stateNetworkOrder, dataPtr, sizeof(stateNetworkOrder));
    sharedMemory->FSMState = ntohl(stateNetworkOrder);
    dataPtr += sizeof(stateNetworkOrder);

    memcpy(&stateNetworkOrder, dataPtr, sizeof(stateNetworkOrder));
    sharedMemory->LowControlState = ntohl(stateNetworkOrder);
    dataPtr += sizeof(stateNetworkOrder);

    memcpy(&stateNetworkOrder, dataPtr, sizeof(stateNetworkOrder));
    sharedMemory->HighControlState = ntohl(stateNetworkOrder);
    dataPtr += sizeof(stateNetworkOrder);

    uint64_t localTimeNetworkOrder;
    memcpy(&localTimeNetworkOrder, dataPtr, sizeof(localTimeNetworkOrder));
    sharedMemory->localTime = ntohd(localTimeNetworkOrder);
    dataPtr += sizeof(localTimeNetworkOrder);

    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        int motorErrorStatusNetworkOrder;
        memcpy(&motorErrorStatusNetworkOrder, dataPtr, sizeof(motorErrorStatusNetworkOrder));
        sharedMemory->motorErrorStatus[i] = ntohl(motorErrorStatusNetworkOrder);
        dataPtr += sizeof(motorErrorStatusNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        int motorTempNetworkOrder;
        memcpy(&motorTempNetworkOrder, dataPtr, sizeof(motorTempNetworkOrder));
        sharedMemory->motorTemp[i] = ntohl(motorTempNetworkOrder);
        dataPtr += sizeof(motorTempNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorVoltageNetworkOrder;
        memcpy(&motorVoltageNetworkOrder, dataPtr, sizeof(motorVoltageNetworkOrder));
        sharedMemory->motorVoltage[i] = ntohd(motorVoltageNetworkOrder);
        dataPtr += sizeof(motorVoltageNetworkOrder);
    }
    const double PI = 3.141592;
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorPositionNetworkOrder;
        memcpy(&motorPositionNetworkOrder, dataPtr, sizeof(motorPositionNetworkOrder));
        sharedMemory->motorPosition[i] = ntohd(motorPositionNetworkOrder);
        sharedMemory->motorPosition[i] = sharedMemory->motorPosition[i] / PI * 180;
        dataPtr += sizeof(motorPositionNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorVelocityNetworkOrder;
        memcpy(&motorVelocityNetworkOrder, dataPtr, sizeof(motorVelocityNetworkOrder));
        sharedMemory->motorVelocity[i] = ntohd(motorVelocityNetworkOrder);
        dataPtr += sizeof(motorVelocityNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorTorqueNetworkOrder;
        memcpy(&motorTorqueNetworkOrder, dataPtr, sizeof(motorTorqueNetworkOrder));
        sharedMemory->motorTorque[i] = ntohd(motorTorqueNetworkOrder);
        dataPtr += sizeof(motorTorqueNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorDesiredPositionNetworkOrder;
        memcpy(&motorDesiredPositionNetworkOrder, dataPtr, sizeof(motorDesiredPositionNetworkOrder));
        sharedMemory->motorDesiredPosition[i] = ntohd(motorDesiredPositionNetworkOrder);
        dataPtr += sizeof(motorDesiredPositionNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorDesiredVelocityNetworkOrder;
        memcpy(&motorDesiredVelocityNetworkOrder, dataPtr, sizeof(motorDesiredVelocityNetworkOrder));
        sharedMemory->motorDesiredVelocity[i] = ntohd(motorDesiredVelocityNetworkOrder);
        dataPtr += sizeof(motorDesiredVelocityNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorDesiredTorqueNetworkOrder;
        memcpy(&motorDesiredTorqueNetworkOrder, dataPtr, sizeof(motorDesiredTorqueNetworkOrder));
        sharedMemory->motorDesiredTorque[i] = ntohd(motorDesiredTorqueNetworkOrder);
        dataPtr += sizeof(motorDesiredTorqueNetworkOrder);
    }

    unpackingEigenVector3d(dataPtr, sharedMemory->globalBasePosition);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseVelocity);
    unpackingEigenVector4d(dataPtr, sharedMemory->globalBaseQuaternion);

    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseDesiredPosition);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseDesiredVelocity);
    unpackingEigenVector4d(dataPtr, sharedMemory->globalBaseDesiredQuaternion);

    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseDesiredEulerAngle);
    unpackingEigenVector3d(dataPtr, sharedMemory->bodyBaseDesiredAngularVelocity);
    unpackingEigenVector3d(dataPtr, sharedMemory->bodyBaseDesiredVelocity);

    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseEulerAngle);
    unpackingEigenVector3d(dataPtr, sharedMemory->bodyBaseAngularVelocity);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseAcceleration);

    for (int i = 0; i < 4; ++i)
    {
        unpackingEigenVector3d(dataPtr, sharedMemory->pdTorque[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->mpcTorque[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->globalFootPosition[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->bodyBase2FootPosition[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->bodyBase2FootVelocity[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->bodyBase2FootDesiredPosition[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->bodyBase2FootDesiredVelocity[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->solvedGRF[i]);
    }

    memcpy(sharedMemory->contactState, dataPtr, 4 * sizeof(bool));
    dataPtr += 4 * sizeof(bool);

    for (int i = 0; i < 3; ++i)
    {
        uint64_t testBasePosNetworkOrder;
        memcpy(&testBasePosNetworkOrder, dataPtr, sizeof(testBasePosNetworkOrder));
        sharedMemory->testBasePos[i] = ntohd(testBasePosNetworkOrder);
        dataPtr += sizeof(testBasePosNetworkOrder);
    }

    for (int i = 0; i < 3; ++i)
    {
        uint64_t testBaseVelNetworkOrder;
        memcpy(&testBaseVelNetworkOrder, dataPtr, sizeof(testBaseVelNetworkOrder));
        sharedMemory->testBaseVel[i] = ntohd(testBaseVelNetworkOrder);
        dataPtr += sizeof(testBaseVelNetworkOrder);
    }

    for (int i = 0; i < 11; ++i)
    {
        uint64_t threadElapsedTimeNetworkOrder;
        memcpy(&threadElapsedTimeNetworkOrder, dataPtr, sizeof(threadElapsedTimeNetworkOrder));
        sharedMemory->threadElapsedTime[i] = ntohd(threadElapsedTimeNetworkOrder);
        dataPtr += sizeof(threadElapsedTimeNetworkOrder);
    }
}

#endif //RBQ_CONSOLE_DATAPROCESSING_HPP
