//
// Created by jh on 24. 7. 3.
//

#ifndef CANINE_MIDDLEWARE_UDPCOMMUNICATION_HPP
#define CANINE_MIDDLEWARE_UDPCOMMUNICATION_HPP


#include <arpa/inet.h>
#include "Setup.hpp"
#include "SharedMemory.hpp"
#include "CommandLists.hpp"

class UDPCommunication
{
public:
    UDPCommunication();
    void Initialize();
    void Send();

private:
    void packageUDPmsg(unsigned char* msg);
    void clearUDPCommand();

private:
    int clientSocket;
    struct sockaddr_in serverAddr;
    socklen_t addr_size;

    unsigned char msg[141];
    int mPrevFSMState;
    int mPrevArmFSMState;
    bool mbPrevArmTeleState;
    bool mbPrevSlopeMode;
};


#endif //CANINE_MIDDLEWARE_UDPCOMMUNICATION_HPP
