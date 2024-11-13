//
// Created by jh on 24. 7. 3.
//
#include "Setup.hpp"
#include "CommandLists.hpp"
#include "SharedMemory.hpp"
#include "threadGenerator.hpp"
#include "TCPCommunication.hpp"
#include "UDPCommunication.hpp"
#include <lcm/lcm-cpp.hpp>
#include "lcm_type/joystick_sub.hpp"


SharedMemory* sharedMemory = SharedMemory::getInstance();
HWD* HWData = HWD::getInstance();

TCPCommunication tcpCommunication;
UDPCommunication udpCommunication;
CommandLists commandLists;

void* sendRobotCommand_udp(void* arg);
void* receiveRobotStatus_tcp(void* arg);
void* highController(void* arg);
void* receiveLCM(void* arg);

double refBodyVelocity[3];
float bt_X = 0.0;
float bt_Y = 0.0;
float bt_A = 0.0;
float bt_B = 0.0;
float prev_bt_X = 0.0;
float prev_bt_Y = 0.0;
float prev_bt_A = 0.0;
float prev_bt_B = 0.0;
float joyStick[4] = {0,};

void checkFSMStart()
{
//    if(1)
//    {
//        commandLists.Start();
//    }
}

void checkFSMEmergencyStop()
{
    if(prev_bt_Y!=bt_Y)
    {
        commandLists.EmergencyStop();
    }
}

void checkFSMStandUp()
{
//    if(prev_bt_Y!=bt_Y)
//    {
//        commandLists.HomeUp();
//    }
}

void checkFSMStandDown()
{
    if(prev_bt_X!=bt_X)
    {
        commandLists.HomeDown();
    }
}

void checkFSMTrotSlow()
{
    if(prev_bt_A!=bt_A)
    {
        commandLists.TrotSlow();
    }
}

void checkFSMCONSTStand()
{
    if(prev_bt_B!=bt_B)
    {
        commandLists.TrotStop();
    }
}

void FSMTrotSlowFunction()
{
    refBodyVelocity[0] = joyStick[0] * 0.4;
    refBodyVelocity[1] = 0.0;
    refBodyVelocity[2] = joyStick[3] * 0.5;
    commandLists.SetBodyVelocity(refBodyVelocity);
}

// JoystickHandler 클래스 정의
class JoystickHandler {
public:
    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const joystick_sub* msg) {
        std::cout << "[RECEIVED] Data on channel: " << chan << std::endl;

        // joystick 데이터 출력
        for (int i = 0; i < 4; i++) {
            std::cout << "joystick[" << i << "]: " << msg->joystick[i] << std::endl;
            joyStick[i] = msg->joystick[i];
        }

        // 버튼 데이터 출력
        std::cout << "vr_botton_X: " << msg->vr_botton_X << std::endl;
        std::cout << "vr_botton_Y: " << msg->vr_botton_Y << std::endl;
        std::cout << "vr_botton_A: " << msg->vr_botton_A << std::endl;
        std::cout << "vr_botton_B: " << msg->vr_botton_B << std::endl;
        bt_X = msg->vr_botton_X;
        bt_Y = msg->vr_botton_Y;
        bt_A = msg->vr_botton_A;
        bt_B = msg->vr_botton_B;
        switch(sharedMemory->FSMState)
        {
            case FSM_INITIAL:
            {
                checkFSMStart();
                break;
            }
            case FSM_EMERGENCY_STOP:
            {
                break;
            }
            case FSM_READY:
            {
                checkFSMEmergencyStop();
                checkFSMStandUp();
                break;
            }
            case FSM_ISOLATION:
            {
                checkFSMEmergencyStop();
                break;
            }
            case FSM_CONST_STAND:
            {
                checkFSMEmergencyStop();
                checkFSMStandDown();
                checkFSMTrotSlow();
                break;
            }
            case FSM_STAND:
            {
                checkFSMEmergencyStop();
                checkFSMCONSTStand();
                break;
            }
            case FSM_TROT_SLOW:
            {
                FSMTrotSlowFunction();
                checkFSMEmergencyStop();
                checkFSMCONSTStand();
                break;
            }
            default:
            {
                checkFSMEmergencyStop();
                break;
            }
        }
        prev_bt_A = bt_A;
        prev_bt_B = bt_B;
        prev_bt_X = bt_X;
        prev_bt_Y = bt_Y;
    }
};


int main()
{
    pthread_t UDPthread;
    pthread_t TCPthread;
    pthread_t HighControlThread;
    pthread_t LCMthread;

    generateRtThread(HighControlThread, highController, "RT_Controller", 3, 99, NULL);
    generateNrtThread(UDPthread, sendRobotCommand_udp, "UDP_send", 6, NULL);
    generateNrtThread(TCPthread, receiveRobotStatus_tcp, "TCP_receive", 3, NULL);
    generateNrtThread(LCMthread, receiveLCM, "LCM_thread", 3, NULL);

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
    std::cout << "[MAIN] Generated Real-Time High Controller Thread : " << 1 / double(threadPeriod) * 1e6 << " Hz" <<std::endl;

    refBodyVelocity[0] = 0.0; // reference x-axis velocity in body frame. [m/s]
    refBodyVelocity[1] = 0.0; // reference y-axis velocity in body frame. [m/s]
    refBodyVelocity[2] = 0.0; // reference yaw velocity. [rad/s]

    sleep(2);
    commandLists.Start();
    sleep(4);
    commandLists.HomeUp();
    while (true)
    {
        localTime += dT;

        // Functions -- start

        usleep(10000);
    }
}

void* receiveLCM(void* arg)
{
    std::cout << "[MAIN] Generated LCM Communication Thread."<<std::endl;

    lcm::LCM lc;
    if (!lc.good()) {
        std::cerr << "[ERROR] Unable to initialize LCM." << std::endl;
        return nullptr;
    }
    JoystickHandler handlerObject;  // JoystickHandler 객체 생성

    // "ARM_CHANNEL" 채널 구독 설정
    lc.subscribe("ARM_CHANNEL", &JoystickHandler::handleMessage, &handlerObject);
    while(true)
    {

        if (lc.handle() != 0) {
            std::cerr << "[ERROR] LCM handle encountered an issue." << std::endl;
        }

        usleep(10000);
    }
    return nullptr;
}