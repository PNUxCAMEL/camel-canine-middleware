//
// Created by ys on 24. 6. 6.
//
#include "threadGenerator.hpp"
#include "Setup.hpp"
#include "Gamepad.h"
#include "JoystickCommand.hpp"
#include "DataProcessing.hpp"
#include "SharedMemory.hpp"


SharedMemory* sharedMemory = SharedMemory::getInstance();
Gamepad* gamepad = Gamepad::getInstance();
HWD* HWData = HWD::getInstance();

JoystickCommand joystickCommand;

void* readJoystick(void* arg);
void* sendRobotCommand_udp(void* arg);
void* receiveRobotStatus_tcp(void* arg);

int main()
{
    pthread_t joystickThread;
    pthread_t UDPthread;
    pthread_t TCPthread;

    generateNrtThread(joystickThread, readJoystick, "JoyThread", 4, NULL);
    generateNrtThread(UDPthread, sendRobotCommand_udp, "UDP_send", 5, NULL);
    generateNrtThread(TCPthread, receiveRobotStatus_tcp, "TCP_receive", 6, NULL);
    while (true)
    {
        sleep(10000);
    }
}

void packageUDPmsg(unsigned char* msg)
{
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
    // tail
    msg[43] = 0x00;
    msg[44] = 0x01;
}

void* readJoystick(void* arg)
{
    while (true)
    {
        gamepad->Read();
        joystickCommand.copyJoystickData();
    }
}

void* sendRobotCommand_udp(void* arg)
{
    int clientSocket;
    struct sockaddr_in serverAddr;
    socklen_t addr_size;

    unsigned char msg[45];
    memset(msg, 0, sizeof(msg));

    clientSocket = socket(PF_INET, SOCK_DGRAM, 0);

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(60000);
    serverAddr.sin_addr.s_addr = inet_addr(INIT_IP.c_str());
    memset(serverAddr.sin_zero, '\0', sizeof(serverAddr.sin_zero));
    addr_size = sizeof(serverAddr);
    while (true)
    {
        memset(msg, 0, sizeof(msg));
        packageUDPmsg(msg);
        sendto(clientSocket, msg, sizeof(msg), 0, (struct sockaddr*)&serverAddr, addr_size);
    }
    return 0;
}

void* receiveRobotStatus_tcp(void* arg)
{
    int serverSocket, clientSocket;
    struct sockaddr_in serverAddr;
    struct sockaddr_storage serverStorage;
    socklen_t addr_size;

    // 서버 소켓 생성
    serverSocket = socket(PF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1)
    {
        std::cerr << "Cannot create socket" << std::endl;
        return (void*)1;
    }

    int opt = 1;
    if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) {
        std::cerr << "setsockopt(SO_REUSEADDR) failed" << std::endl;
        close(serverSocket);
        return (void *) 1;
    }

    // 서버 주소 구조체 설정
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(60001);
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    memset(serverAddr.sin_zero, '\0', sizeof(serverAddr.sin_zero));

    // 소켓에 주소 바인드
    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1)
    {
        std::cerr << "Bind failed" << std::endl;
        close(serverSocket);
        return (void*)1;
    }

    // 연결 대기
    if (listen(serverSocket, 50) == -1)
    {
        std::cerr << "Listen failed" << std::endl;
        close(serverSocket);
        return (void*)1;
    }

    std::cout << "Server listening on port 60001" << std::endl;

    while (true)
    {
        addr_size = sizeof serverStorage;
        clientSocket = accept(serverSocket, (struct sockaddr*)&serverStorage, &addr_size);

        if (clientSocket == -1)
        {
            std::cerr << "Accept failed" << std::endl;
            continue;
        }


        // 클라이언트로부터 데이터 길이를 먼저 수신
        int64_t expectedBytesNetworkOrder = 0;
        int bytesRead = recv(clientSocket, &expectedBytesNetworkOrder, sizeof(expectedBytesNetworkOrder), 0);
        if (bytesRead != sizeof(expectedBytesNetworkOrder))
        {
            std::cerr << "Error reading data size. Bytes read: " << bytesRead << std::endl;
            close(clientSocket);
            continue;
        }

        int64_t expectedBytes64 = ntohll(expectedBytesNetworkOrder);  // 네트워크 바이트 오더를 호스트 바이트 오더로 변환
        if (expectedBytes64 > INT_MAX)
        {
            std::cerr << "Data size too large: " << expectedBytes64 << std::endl;
            close(clientSocket);
            continue;
        }
        int expectedBytes = static_cast<int>(expectedBytes64);

        if (expectedBytes <= 0)
        {
            std::cerr << "Invalid data size: " << expectedBytes << std::endl;
            close(clientSocket);
            continue;
        }

        // expectedBytes만큼 데이터를 수신
        std::vector<char> dataBuffer(expectedBytes);
        int totalBytesRead = 0;

        while (totalBytesRead < expectedBytes)
        {
            bytesRead = recv(clientSocket, dataBuffer.data() + totalBytesRead, expectedBytes - totalBytesRead, 0);
            if (bytesRead <= 0)
            {
                std::cerr << "Failed to receive full data. Bytes read: " << bytesRead << std::endl;
                close(clientSocket);
                continue;
            }
            totalBytesRead += bytesRead;
        }

        if (totalBytesRead == expectedBytes)
        {
            unpackingTCPmsg(dataBuffer);
        }

        close(clientSocket);
    }

    close(serverSocket);
    return nullptr;
}