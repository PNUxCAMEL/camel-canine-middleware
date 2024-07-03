//
// Created by jh on 24. 7. 3.
//

#ifndef CANINE_MIDDLEWARE_UDPCOMMUNICATION_HPP
#define CANINE_MIDDLEWARE_UDPCOMMUNICATION_HPP


#include <arpa/inet.h>
#include "Setup.hpp"
#include "SharedMemory.hpp"

class UDPCommunication
{
public:
    UDPCommunication();
    void Initialize();
    void Send();

private:
    void packageUDPmsg(unsigned char* msg);

private:
    int clientSocket;
    struct sockaddr_in serverAddr;
    socklen_t addr_size;

    unsigned char msg[45];
};


#endif //CANINE_MIDDLEWARE_UDPCOMMUNICATION_HPP
