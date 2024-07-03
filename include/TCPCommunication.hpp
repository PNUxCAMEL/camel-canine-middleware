//
// Created by jh on 24. 7. 3.
//

#ifndef CANINE_MIDDLEWARE_TCPCOMMUNICATION_HPP
#define CANINE_MIDDLEWARE_TCPCOMMUNICATION_HPP

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "Setup.hpp"
#include "SharedMemory.hpp"

class TCPCommunication
{
public:
    TCPCommunication();
    int Initialize();
    void Read();
    void CloseServer();

private:
    void unpackingTCPmsg(const std::vector<char>& dataBuffer);
    uint64_t htonll(uint64_t value);
    uint64_t ntohll(uint64_t value);
    uint64_t htond(double value);
    double ntohd(uint64_t value);
    void unpackingEigenVector3d(const char*& dataPtr, Eigen::Vector3d& vec);
    void unpackingEigenVector4d(const char*& dataPtr, Eigen::Vector4d& vec);

private:
    int serverSocket, clientSocket;
    struct sockaddr_in serverAddr;
    struct sockaddr_storage serverStorage;
    socklen_t addr_size;

};


#endif //CANINE_MIDDLEWARE_TCPCOMMUNICATION_HPP
