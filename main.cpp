//
// Created by jh on 24. 7. 3.
//
#include "Setup.hpp"
#include "CommandLists.hpp"
#include "SharedMemory.hpp"
#include "threadGenerator.hpp"
#include "TCPCommunication.hpp"
#include "UDPCommunication.hpp"

SharedMemory* sharedMemory = SharedMemory::getInstance();
HWD* HWData = HWD::getInstance();

TCPCommunication tcpCommunication;
UDPCommunication udpCommunication;
CommandLists commandLists;

void* sendRobotCommand_udp(void* arg);
void* receiveRobotStatus_tcp(void* arg);
void* highController(void* arg);

int main()
{
    pthread_t UDPthread;
    pthread_t TCPthread;
    pthread_t HighControlThread;

    generateRtThread(HighControlThread, highController, "RT_Controller", 5, 99, NULL);
    generateNrtThread(UDPthread, sendRobotCommand_udp, "UDP_send", 6, NULL);
    generateNrtThread(TCPthread, receiveRobotStatus_tcp, "TCP_receive", 7, NULL);

    while (true)
    {
        sleep(10000);
    }
}

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
    commandLists.Start();
    sleep(4);
    commandLists.HomeUp();
    sleep(4);
    commandLists.TrotSlow();
    sleep(1);

    clock_gettime(CLOCK_REALTIME, &time1);
    while (true)
    {
        clock_gettime(CLOCK_REALTIME, &time2);
        timeAddus(&time1, threadPeriod);

        // Functions -- start

        double refBodyVelocity[3];
        refBodyVelocity[0] = sin(sharedMemory->localTime * 0.5 * 3.141592); // reference x-axis velocity in body frame. [m/s]
        refBodyVelocity[1] = 0.0; // reference y-axis velocity in body frame. [m/s]
        refBodyVelocity[2] = 0.0; // reference yaw velocity. [rad/s]
        commandLists.SetBodyVelocity(refBodyVelocity);

        // Functions -- end

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &time1, NULL);
        if (timeCmp(&time2, &time1) > 0)
        {
            std::cout << "[MAIN] Deadline Miss, High Controller Real-Time Thread : " << timeDifferentMs(&time1, &time2) * 0.001 << " ms" << std::endl;
        }
    }
}