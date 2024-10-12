//
// Created by jh on 24. 7. 3.
//

#include "UDPCommunication.hpp"

UDPCommunication::UDPCommunication()
{
    UDP_Open = open();
    sharedMemory = SharedMemory::getInstance();
}

bool UDPCommunication::open(){

    memset(msg, '0', sizeof(msg));

    clientSocket = socket(PF_INET, SOCK_DGRAM, 0);

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(60000);
    serverAddr.sin_addr.s_addr = inet_addr(INIT_IP.c_str());
    memset(serverAddr.sin_zero, '\0', sizeof(serverAddr.sin_zero));
    addr_size = sizeof(serverAddr);
    return 1;
}

//data sender!!
void UDPCommunication::packageUDPmsg(char* msg)
{
    SharedMemory* sharedMemory = SharedMemory::getInstance();
    // header
    msg[0] = 0xFF; // len: 1
    msg[1] = 0xFE; // len: 1
    memcpy(&msg[2], &sharedMemory->udp.joyCommand, sizeof(int8_t)); // len: 1
    memcpy(&msg[3], sharedMemory->udp.userLinVel.data(), sizeof(double[3])); // len: 24
    memcpy(&msg[27], sharedMemory->udp.userAngVel.data(), sizeof(double[3])); // len: 24
    // tail
    msg[51] = 0x00; // len: 1
    msg[52] = 0x01; // len: 1
}

void UDPCommunication::SendData()
{
    while (true)
    {
        packageUDPmsg(msg);
        sendto(clientSocket, msg, sizeof(msg), 0, (struct sockaddr*)&serverAddr, addr_size);
        usleep(100);
    }
}