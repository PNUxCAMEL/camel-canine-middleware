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

CommandLists commandLists;

double referenceTime;
bool bIsReferenceTimeUpdate;
int prevFSMState = 1000;
int upORoff = 0; // 0: up, 1: off
int trotORdown = 0; //0: trot, 1: down

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
    UDPCommunication udpSender;
    udpSender.SendData();
}

void* receiveRobotStatus_tcp(void* arg)
{
    std::cout << "[MAIN] Generated TCP Thread." <<std::endl;
    try
    {
        boost::asio::io_context io_context;
        std::shared_ptr<TCPCommunication> server = std::make_shared<TCPCommunication>(io_context, 60001);
        io_context.run();
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}

void* highController(void* arg)
{
    double dT = 0.01; // 10Hz Real-time thread
    double localTime = 0.0;
    const long threadPeriod = long(dT * 1e6);
    struct timespec time1;
    struct timespec time2;
    std::cout << "[MAIN] Generated Real-Time High Controller Thread : " << 1 / double(threadPeriod) * 1e6 << " Hz" <<std::endl;

    clock_gettime(CLOCK_REALTIME, &time1);
    bIsReferenceTimeUpdate = true;
    referenceTime = 0.0;
    while (true)
    {
        localTime += dT;
        clock_gettime(CLOCK_REALTIME, &time2);
        timeAddus(&time1, threadPeriod);

        // Functions -- start
        if(sharedMemory->isTCPConnected)
        {
            if(prevFSMState != sharedMemory->FSMState)
            {
                bIsReferenceTimeUpdate = true;
            }
            else
            {
                bIsReferenceTimeUpdate = false;
            }
            switch(sharedMemory->FSMState)
            {
            case FSM_INITIAL:
                if(bIsReferenceTimeUpdate)
                {
                    referenceTime = sharedMemory->localTime + 2;
                }
                if(sharedMemory->localTime > referenceTime)
                {
                    upORoff = 0;
                    commandLists.Start();
                }
                break;
            case FSM_READY:
                if(bIsReferenceTimeUpdate)
                {
                    if(upORoff == 0)
                    {
                        referenceTime = sharedMemory->localTime + 2;
                    }
                    else if (upORoff == 1)
                    {
                        referenceTime = sharedMemory->localTime + 0.1;
                    }
                }
                if(sharedMemory->localTime > referenceTime)
                {
                    if(upORoff == 0)
                    {
                        commandLists.StandUp();
                    }
                    else if (upORoff == 1)
                    {
                        commandLists.EmergencyStop();
                    }
                }
                break;
            case FSM_STAND:
                if(bIsReferenceTimeUpdate)
                {
                    referenceTime = sharedMemory->localTime + 0.1;
                }
                if(sharedMemory->localTime > referenceTime)
                {
                    if(trotORdown == 0)
                    {
                        commandLists.TrotSlow();
                    }
                    else if(trotORdown == 1)
                    {
                        commandLists.SitDown();
                    }
                }
                break;
            case FSM_TROT_SLOW:
                if(bIsReferenceTimeUpdate)
                {
                    referenceTime = sharedMemory->localTime + 3.0;
                    trotORdown = 1;
                }

                commandLists.SetBodyVelocity(0.1, 0.0, 0.0);

                std::cout << "[shared memory] base velocity in body frame (x,y): " << sharedMemory->bodyBaseVelocity[0] << ", " << sharedMemory->bodyBaseVelocity[1] << std::endl;
                std::cout << "[shared memory] base yaw rate (yaw_dot): " << sharedMemory->bodyBaseAngularVelocity[2] << std::endl;
                std::cout << "[shared memory] base Euler angle (roll, pitch, yaw): " << sharedMemory->globalBaseEulerAngle[0] << ", " << sharedMemory->globalBaseEulerAngle[1] << ", " << sharedMemory->globalBaseEulerAngle[2] << std::endl;

                if(sharedMemory->localTime > referenceTime)
                {
                    commandLists.TrotStop();
                }
                break;
            case FSM_STAND_UP:
                trotORdown = 0;
                break;
            case FSM_SIT_DOWN:
                upORoff = 1;
                break;
            case FSM_EMERGENCY_STOP:
                if(bIsReferenceTimeUpdate)
                {
                    referenceTime = sharedMemory->localTime + 1.0;
                    trotORdown = 1;
                }
                if(sharedMemory->localTime > referenceTime)
                {
                    commandLists.Restart();
                }
                break;
            default:
                commandLists.NoCommand();
                commandLists.SetBodyVelocity(0, 0, 0);
                break;
            }
            prevFSMState = sharedMemory->FSMState;
        }
        else
        {
            std::cout<<"waiting TCP connection..."<<std::endl;
        }
        // Functions -- end
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &time1, NULL);
        if (timeCmp(&time2, &time1) > 0)
        {
            std::cout << "[MAIN] Deadline Miss, High Controller Real-Time Thread : " << timeDifferentMs(&time1, &time2) * 0.001 << " ms" << std::endl;
        }
    }

//    if(sharedMemory->isTCPConnected)
//    {
//        std::cout<<"go..."<<std::endl;
//        sleep(2);
//        commandLists.Start();
//        sleep(4);
//        commandLists.HomeUp();
//        sleep(4);
//        commandLists.TrotSlow();
//        sleep(1);
//
//        clock_gettime(CLOCK_REALTIME, &time1);
//        while (true)
//        {
//            localTime += dT;
//            clock_gettime(CLOCK_REALTIME, &time2);
//            timeAddus(&time1, threadPeriod);
//
//            // Functions -- start
//
//            double refBodyVelocityX = 0.1; // reference x-axis velocity in body frame. [m/s]
//            double refBodyVelocityY = 0.0; // reference y-axis velocity in body frame. [m/s]
//            double refBodyVelocityYaw = 0.0; // reference yaw velocity. [rad/s]
//            commandLists.SetBodyVelocity(refBodyVelocityX, refBodyVelocityY, refBodyVelocityYaw);
//
//            std::cout << "[shared memory] base velocity in body frame (x,y): " << sharedMemory->bodyBaseVelocity[0] << ", " << sharedMemory->bodyBaseVelocity[1] << std::endl;
//            std::cout << "[shared memory] base yaw rate (yaw_dot): " << sharedMemory->bodyBaseAngularVelocity[2] << std::endl;
//            std::cout << "[shared memory] base Euler angle (roll, pitch, yaw): " << sharedMemory->globalBaseEulerAngle[0] << ", " << sharedMemory->globalBaseEulerAngle[1] << ", " << sharedMemory->globalBaseEulerAngle[2] << std::endl;
//
//            if(localTime > 5.0)
//            {
//                break;
//            }
//
//            // Functions -- end
//
//            clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &time1, NULL);
//            if (timeCmp(&time2, &time1) > 0)
//            {
//                std::cout << "[MAIN] Deadline Miss, High Controller Real-Time Thread : " << timeDifferentMs(&time1, &time2) * 0.001 << " ms" << std::endl;
//            }
//        }
//        commandLists.TrotStop();
//        sleep(4);
//        commandLists.HomeDown();
//        sleep(4);
//        commandLists.EmergencyStop();
//        sleep(2);
//    }
//    else
//    {
//    }
}