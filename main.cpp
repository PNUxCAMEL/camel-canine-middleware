//
// Created by jh on 24. 7. 3.
//
#include "Setup.hpp"
#include "CommandLists.hpp"
#include "SharedMemory.hpp"
#include "threadGenerator.hpp"
#include "TCPCommunication.hpp"
#include "UDPCommunication.hpp"
//#include "Gamepad.h"
//#include "JoystickCommand.hpp"

SharedMemory* sharedMemory = SharedMemory::getInstance();
//Gamepad* gamepad = Gamepad::getInstance();
HWD* HWData = HWD::getInstance();

//JoystickCommand joystickCommand;
TCPCommunication tcpCommunication;
UDPCommunication udpCommunication;

void* readJoystick(void* arg);
void* sendRobotCommand_udp(void* arg);
void* receiveRobotStatus_tcp(void* arg);
void* highController(void* arg);

int main()
{
    pthread_t joystickThread;
    pthread_t UDPthread;
    pthread_t TCPthread;
    pthread_t HighControlThread;

    generateRtThread(HighControlThread, highController, "RT_Controller", 4, 99, NULL);
    generateNrtThread(UDPthread, sendRobotCommand_udp, "UDP_send", 5, NULL);
    generateNrtThread(TCPthread, receiveRobotStatus_tcp, "TCP_receive", 6, NULL);
//    generateNrtThread(joystickThread, readJoystick, "NRTJoyThread", 4, NULL);

    while (true)
    {
        sleep(10000);
    }
}

//void* readJoystick(void* arg)
//{
//    std::cout << "[MAIN] Generated Joystick Thread : " <<std::endl;
//    while (true)
//    {
//        gamepad->Read();
//        joystickCommand.copyJoystickData();
//    }
//}

void* sendRobotCommand_udp(void* arg)
{
    std::cout << "[MAIN] Generated UDP Thread." <<std::endl;
    udpCommunication.Initialize();
    while (true)
    {
        udpCommunication.Send();
        usleep(20000);
    }
}

void* receiveRobotStatus_tcp(void* arg)
{
    std::cout << "[MAIN] Generated TCP Thread." <<std::endl;
    tcpCommunication.Initialize();
    while (true)
    {
        tcpCommunication.Read();
        usleep(20000);
    }
    tcpCommunication.CloseServer();
}

void* highController(void* arg)
{
    double dT = 0.01; // 10Hz Real-time thread
    const long threadPeriod = long(dT * 1e6);
    struct timespec time1;
    struct timespec time2;
    std::cout << "[MAIN] Generated Real-Time High Controller Thread : " << 1 / double(threadPeriod) * 1e6 << " Hz" <<std::endl;


    sleep(2);
    sharedMemory->UDPCommand = CMD_START;
    sleep(2);
    sharedMemory->UDPCommand = CMD_HOME_UP;
    sleep(5);
    sharedMemory->UDPCommand = CMD_TROT_SLOW;
    sleep(2);

    clock_gettime(CLOCK_REALTIME, &time1);
    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &time2);
        timeAddus(&time1, threadPeriod);

        // Functions -- start

        sharedMemory->UDPRefBodyLinearVelocity_x = sin(sharedMemory->localTime * 0.5 * 3.141592);
        std::cout << "==== local time: " << sharedMemory->localTime << std::endl;
        std::cout << "ref x: " << sharedMemory->UDPRefBodyLinearVelocity_x << std::endl;

        // Functions -- end

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &time1, NULL);
        if (timeCmp(&time2, &time1) > 0)
        {
            std::cout << "[MAIN] Deadline Miss, High Controller Real-Time Thread : " << timeDifferentMs(&time1, &time2) * 0.001 << " ms" << std::endl;
        }
    }
}