//
// Created by jh on 24. 7. 3.
//

#ifndef CANINE_MIDDLEWARE_UDPCOMMUNICATION_HPP
#define CANINE_MIDDLEWARE_UDPCOMMUNICATION_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "Setup.hpp"
#include "SharedMemory.hpp"

class UDPCommunication
{
public:
    UDPCommunication();
    void SendData();
private:
    SharedMemory* sharedMemory;
    int clientSocket;
    struct sockaddr_in serverAddr;
    socklen_t addr_size;

    char msg[53];
    bool open();
    bool UDP_Open;
    void packageUDPmsg(char* msg);
};


#endif //CANINE_MIDDLEWARE_UDPCOMMUNICATION_HPP
