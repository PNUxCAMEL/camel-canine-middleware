//
// Created by jh on 24. 7. 3.
//
#include "Setup.hpp"
#include "Gamepad.h"
#include "SharedMemory.hpp"
#include "threadGenerator.hpp"
#include "TCPCommunication.hpp"
#include "UDPCommunication.hpp"
#include "JoystickCommand.hpp"

SharedMemory* sharedMemory = SharedMemory::getInstance();
Gamepad* gamepad = Gamepad::getInstance();
HWD* HWData = HWD::getInstance();

JoystickCommand joystickCommand;
TCPCommunication tcpCommunication;
UDPCommunication udpCommunication;

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
    udpCommunication.Initialize();
    while (true)
    {
        udpCommunication.Send();
        usleep(20000);
    }
}

void* receiveRobotStatus_tcp(void* arg)
{
    tcpCommunication.Initialize();
    while (true)
    {
        tcpCommunication.Read();
        usleep(20000);
    }
    tcpCommunication.CloseServer();
}