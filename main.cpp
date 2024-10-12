//
// Created by jh on 24. 7. 3.
//
#include <poll.h>
#include <unistd.h>

#include "Setup.hpp"
#include "CommandLists.hpp"
#include "SharedMemory.hpp"
#include "threadGenerator.hpp"
#include "TCPCommunication.hpp"
#include "UDPCommunication.hpp"

SharedMemory* sharedMemory = SharedMemory::getInstance();

CommandLists commandLists;

double referenceTime;
bool bIsReferenceTimeUpdate;
int prevFSMState = 1000;
int upORoff = 0; // 0: up, 1: off
int trotORdown = 0; //0: trot, 1: down

void* sendRobotCommand_udp(void* arg);
void* receiveRobotStatus_tcp(void* arg);
void* highController(void* arg);
void* KeyListener(void* arg);

int main()
{
    pthread_t UDPthread;
    pthread_t TCPthread;
    pthread_t HighControlThread;
    pthread_t KeyListenerThread;

    generateRtThread(HighControlThread, highController, "RT_Controller", 5, 99, NULL);
    generateNrtThread(UDPthread, sendRobotCommand_udp, "UDP_send", 6, NULL);
    generateNrtThread(TCPthread, receiveRobotStatus_tcp, "TCP_receive", 7, NULL);
    generateNrtThread(KeyListenerThread, KeyListener, "key_board", 4, NULL);

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

void* KeyListener(void* arg) {
    struct pollfd fds[1];
    fds[0].fd = STDIN_FILENO; // 표준 입력 (키보드)
    fds[0].events = POLLIN;   // 읽기 이벤트 감지

    while (true) {
        if(sharedMemory->isTCPConnected)
        {
            int ret = poll(fds, 1, 100); // 100ms 타임아웃
            if (ret > 0 && (fds[0].revents & POLLIN)) {
                char ch;
                read(STDIN_FILENO, &ch, 1); // 키 입력 읽기
                if (ch == 'e') {
                    std::cout << "'e' key pressed! e-stop" << std::endl;
                    commandLists.EmergencyStop();
                }
            }
        }
        tcflush(STDIN_FILENO, TCIFLUSH);
        usleep(50000); // CPU 사용량을 줄이기 위해 잠시 대기
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
}