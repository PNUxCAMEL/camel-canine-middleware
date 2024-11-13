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

void printBaseState()
{
    printf("(%02d:%02d:%02d) Print base states\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    std::cout << "\t\t\t\tbase velocity in body frame:\n"
                << "\t\t\t\t\tv_x:   " << sharedMemory->bodyBaseVelocity[0] << "m/s\n"
                << "\t\t\t\t\tv_y:   " << sharedMemory->bodyBaseVelocity[1] << "m/s\n"
                << "\t\t\t\t\tw_yaw: " << sharedMemory->bodyBaseAngularVelocity[2] << "rad/s"<< std::endl;
    std::cout << "\t\t\t\tbase Euler angle:\n"
                << "\t\t\t\t\troll:  " << sharedMemory->globalBaseEulerAngle[0] << "rad\n"
                << "\t\t\t\t\tpitch: " << sharedMemory->globalBaseEulerAngle[1] << "rad\n"
                << "\t\t\t\t\tyaw:   " << sharedMemory->globalBaseEulerAngle[2] << "rad\n"<< std::endl;
}

void printEndEffectorState()
{
    printf("(%02d:%02d:%02d) Print end-effector states\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    std::cout << "\t\t\t\t End effector position in arm base frame:\n"
                << "\t\t\t\t\tx: "<< sharedMemory->currentEndEffectorPosition[0] << "m\n"
                << "\t\t\t\t\ty: "<< sharedMemory->currentEndEffectorPosition[1] << "m\n"
                << "\t\t\t\t\tz: "<< sharedMemory->currentEndEffectorPosition[2] << "m"<< std::endl;
    std::cout << "\t\t\t\t End effector orientation in arm base frame:\n"
                << "\t\t\t\t\troll:  " << sharedMemory->currentEndEffectorEulerAngle[0] * 180.0 / 3.141592 << "rad\n"
                << "\t\t\t\t\tpitch: " << sharedMemory->currentEndEffectorEulerAngle[1] * 180.0 / 3.141592 << "rad\n"
                << "\t\t\t\t\tyaw:   " << sharedMemory->currentEndEffectorEulerAngle[2] * 180.0 / 3.141592 << "rad\n"<< std::endl;
}

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

    // Initialize controller
    commandLists.Start();

    /// CMD: Home up
    commandLists.HomeUp();

    /// CMD: Trot slow
    commandLists.TrotSlow();

    /// CMD: Base velocity control in body frame
    double refBodyVelocity[3];

    refBodyVelocity[0] = 0.1; // reference x-axis velocity in body frame. [m/s]
    refBodyVelocity[1] = 0.0; // reference y-axis velocity in body frame. [m/s]
    refBodyVelocity[2] = 0.0; // reference yaw velocity. [rad/s]
    commandLists.SetBodyVelocity(refBodyVelocity);
    for(int i = 0 ; i < 5 ; i ++)
    {
        printBaseState();
        sleep(1);
    }
    refBodyVelocity[0] = 0.0; // reference x-axis velocity in body frame. [m/s]
    refBodyVelocity[1] = 0.0; // reference y-axis velocity in body frame. [m/s]
    refBodyVelocity[2] = 0.0; // reference yaw velocity. [rad/s]
    commandLists.SetBodyVelocity(refBodyVelocity);
    sleep(1);

    /// CMD: Trot stop
    commandLists.TrotStop();

    /// CMD: Arm goal position control
    double armGoalPosition[3];
    double armGoalEulerAngle[3];

    armGoalPosition[0] = 0.50;
    armGoalPosition[1] = 0.10;
    armGoalPosition[2] = 0.45;
    armGoalEulerAngle[0] = 0.0;
    armGoalEulerAngle[1] = 0.0;
    armGoalEulerAngle[2] = 0.0;
    commandLists.ArmMove(armGoalPosition, armGoalEulerAngle);
    printEndEffectorState();

    armGoalPosition[1] = -0.20;
    commandLists.ArmMove(armGoalPosition, armGoalEulerAngle);
    printEndEffectorState();
    commandLists.ArmHome();
    armGoalPosition[0] = 0.45;
    armGoalPosition[1] = 0.0;
    armGoalPosition[2] = 0.35;
    commandLists.ArmMove(armGoalPosition, armGoalEulerAngle);
    printEndEffectorState();

    armGoalEulerAngle[2] = 30 * 3.141592 / 180;
    commandLists.ArmMove(armGoalPosition, armGoalEulerAngle);
    printEndEffectorState();

    armGoalEulerAngle[2] = -30 * 3.141592 / 180;
    commandLists.ArmMove(armGoalPosition, armGoalEulerAngle);
    printEndEffectorState();

    armGoalEulerAngle[1] = 30.0 * 3.141592 / 180;;
    armGoalEulerAngle[2] = 0.0;
    commandLists.ArmMove(armGoalPosition, armGoalEulerAngle);
    printEndEffectorState();

    armGoalEulerAngle[1] = -30.0 * 3.141592 / 180;;
    commandLists.ArmMove(armGoalPosition, armGoalEulerAngle);
    printEndEffectorState();

    /// CMD: Arm gripper open
    commandLists.ArmGripperOpen();

    /// CMD: Arm gripper close
    commandLists.ArmGripperClose();

    commandLists.ArmHome();

    /// CMD: Home down
    commandLists.HomeDown();

    /// CMD: Emergency stop (Controller stop)
    commandLists.EmergencyStop();
}