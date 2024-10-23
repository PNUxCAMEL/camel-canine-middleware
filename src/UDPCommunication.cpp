//
// Created by jh on 24. 7. 3.
//

#include "UDPCommunication.hpp"

UDPCommunication::UDPCommunication()
    : mPrevFSMState(FSM_INITIAL)
    , mPrevArmFSMState(ARM_FSM::ARM_INITIAL)
    , mbPrevSlopeMode(false)
    , mbPrevArmTeleState(false)
{

}

void UDPCommunication::Initialize()
{
    memset(msg, 0, sizeof(msg));
    clientSocket = socket(PF_INET, SOCK_DGRAM, 0);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(60000);
    serverAddr.sin_addr.s_addr = inet_addr(INIT_IP.c_str());
    memset(serverAddr.sin_zero, '\0', sizeof(serverAddr.sin_zero));
    addr_size = sizeof(serverAddr);
}

void UDPCommunication::Send()
{
    memset(msg, 0, sizeof(msg));
    packageUDPmsg(msg);
    sendto(clientSocket, msg, sizeof(msg), 0, (struct sockaddr*)&serverAddr, addr_size);
}

void UDPCommunication::packageUDPmsg(unsigned char* msg)
{
    SharedMemory* sharedMemory = SharedMemory::getInstance();
    clearUDPCommand();
    uint guiButtonState = sharedMemory->UDPCommand;
    unsigned char gb_u = (0xFF00 & guiButtonState) >> 8;
    unsigned char gb_l = 0x00FF & guiButtonState;
    // header
    msg[0] = 0xFF;
    msg[1] = 0xFE;
    mempcpy(&msg[16], &gb_u, 1);
    mempcpy(&msg[17], &gb_l, 1);
    mempcpy(&msg[19], &sharedMemory->UDPRefBodyLinearVelocity_x, sizeof(double));
    mempcpy(&msg[27], &sharedMemory->UDPRefBodyLinearVelocity_y, sizeof(double));
    mempcpy(&msg[35], &sharedMemory->UDPRefBodyAngularVelocity_yaw, sizeof(double));
    mempcpy(&msg[43], &sharedMemory->UDPDesiredEndEffectorPosition, sizeof(double)*3);
    mempcpy(&msg[67], &sharedMemory->UDPDesiredEndEffectorEulerAngle, sizeof(double)*3);
    mempcpy(&msg[91], &sharedMemory->UDPDesiredTeleOperationLinearVelocity, sizeof(double)*3);
    mempcpy(&msg[115], &sharedMemory->UDPDesiredTeleOperationAngularVelocity, sizeof(double)*3);
}

void UDPCommunication::clearUDPCommand()
{
    SharedMemory* sharedMemory = SharedMemory::getInstance();
    if ((mPrevFSMState != sharedMemory->FSMState) || (mbPrevSlopeMode != sharedMemory->isRamp) || (mPrevArmFSMState != sharedMemory->armFSMState)
         || (mbPrevArmTeleState != sharedMemory->isArmTele))
    {
        sharedMemory->UDPCommand = 0;
    }
    mbPrevSlopeMode = sharedMemory->isRamp;
    mPrevFSMState = sharedMemory->FSMState;
    mPrevArmFSMState = sharedMemory->armFSMState;
    mbPrevArmTeleState = sharedMemory->isArmTele;
}
