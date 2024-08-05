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
    double localTime = 0.0;
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
        localTime += dT;
        clock_gettime(CLOCK_REALTIME, &time2);
        timeAddus(&time1, threadPeriod);

        // Functions -- start

        double refBodyVelocity[3];
        refBodyVelocity[0] = 0.1; // reference x-axis velocity in body frame. [m/s]
        refBodyVelocity[1] = 0.0; // reference y-axis velocity in body frame. [m/s]
        refBodyVelocity[2] = 0.0; // reference yaw velocity. [rad/s]
        commandLists.SetBodyVelocity(refBodyVelocity);

        std::cout << "[shared memory] local time in controller: " << sharedMemory->localTime << " sec" << std::endl;
        std::cout << "[shared memory] base velocity in body frame (x,y): " << sharedMemory->bodyBaseVelocity[0] << ", " << sharedMemory->bodyBaseVelocity[1] << std::endl;
        std::cout << "[shared memory] base yaw rate in body frame (yaw_dot): " << sharedMemory->bodyBaseAngularVelocity[2] << std::endl;
        std::cout << "[shared memory] base Euler angle (roll, pitch, yaw): " << sharedMemory->globalBaseEulerAngle[0] << ", " << sharedMemory->globalBaseEulerAngle[1] << ", " << sharedMemory->globalBaseEulerAngle[2] << std::endl;

        if(localTime > 5.0)
        {
            break;
        }

        // Functions -- end

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &time1, NULL);
        if (timeCmp(&time2, &time1) > 0)
        {
            std::cout << "[MAIN] Deadline Miss, High Controller Real-Time Thread : " << timeDifferentMs(&time1, &time2) * 0.001 << " ms" << std::endl;
        }
    }
    commandLists.TrotStop();
    sleep(4);
    commandLists.HomeDown();
    sleep(4);
    commandLists.EmergencyStop();
    sleep(2);
}