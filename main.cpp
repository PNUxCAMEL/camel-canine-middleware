//
// Created by jh on 24. 7. 3.
//
#include <poll.h>
#include <unistd.h>
#include <lcm/lcm-cpp.hpp>

#include "Setup.hpp"
#include "CommandLists.hpp"
#include "SharedMemory.hpp"
#include "threadGenerator.hpp"
#include "TCPCommunication.hpp"
#include "UDPCommunication.hpp"
#include "lcm_type/joystick_sub.hpp"

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
void* receiveLCM(void* arg);

double refBodyLinearVelocity[3];
double refBodyAngularVelocity[3];

float bt_X = 0.0;
float bt_Y = 0.0;
float bt_A = 0.0;
float bt_B = 0.0;
float prev_bt_X = 0.0;
float prev_bt_Y = 0.0;
float prev_bt_A = 0.0;
float prev_bt_B = 0.0;
float joyStick[4] = {0,};

void checkFSMEmergencyStop()
{
    if(prev_bt_Y!=bt_Y)
    {
        commandLists.EmergencyStop();
    }
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
    refBodyLinearVelocity[0] = joyStick[0] * 0.4;
    refBodyLinearVelocity[1] = 0.0;
    refBodyLinearVelocity[2] = 0.0;
    refBodyAngularVelocity[0] = 0.0;
    refBodyAngularVelocity[1] = 0.0;
    refBodyAngularVelocity[2] = joyStick[3] * 0.5;
    commandLists.SetBodyVelocity(refBodyLinearVelocity,refBodyAngularVelocity);
}

void FSMStandFunction()
{
    refBodyLinearVelocity[0] = 0.0;
    refBodyLinearVelocity[1] = 0.0;
    refBodyLinearVelocity[2] = 0.0;
    refBodyAngularVelocity[0] = 0.0;
    refBodyAngularVelocity[1] = joyStick[0] * 0.4;
    refBodyAngularVelocity[2] = 0.0;
    commandLists.SetBodyVelocity(refBodyLinearVelocity,refBodyAngularVelocity);
}

// JoystickHandler 클래스 정의
class JoystickHandler {
public:
    void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const joystick_sub* msg) {
        std::cout << "[RECEIVED] Data on channel: " << chan << std::endl;

        double joystick_threshold = 0.3;

        // joystick 데이터 출력
        for (int i = 0; i < 4; i++) {
            std::cout << "original_joystick[" << i << "]: " << msg->joystick[i] << std::endl;

            if(abs(msg->joystick[i]) < joystick_threshold)
            {
                joyStick[i] = 0.0;
            }
            else
            {
                joyStick[i] = msg->joystick[i];
            }
//            std::cout << "joystick[" << i << "]: " << joyStick[i] << std::endl;
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

                break;
            }
            case FSM_EMERGENCY_STOP:
            {
                break;
            }
            case FSM_READY:
            {
                checkFSMEmergencyStop();

                break;
            }
            case FSM_STAND_UP:
            {
                checkFSMEmergencyStop();
                break;
            }
            case FSM_SIT_DOWN:
            {
                checkFSMEmergencyStop();
                break;
            }
            case FSM_STAND:
            {
                FSMStandFunction();
                checkFSMEmergencyStop();
                checkFSMStandDown();
                checkFSMTrotSlow();
                break;
            }
            case FSM_TROT_STOP:
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

int main()
{
    pthread_t UDPthread;
    pthread_t TCPthread;
    pthread_t HighControlThread;
    pthread_t KeyListenerThread;
    pthread_t LCMthread;

    generateRtThread(HighControlThread, highController, "RT_Controller", 5, 99, NULL);
    generateNrtThread(UDPthread, sendRobotCommand_udp, "UDP_send", 6, NULL);
    generateNrtThread(TCPthread, receiveRobotStatus_tcp, "TCP_receive", 7, NULL);
    generateNrtThread(KeyListenerThread, KeyListener, "key_board", 4, NULL);
    generateNrtThread(LCMthread, receiveLCM, "LCM_thread", 3, NULL);

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
    std::cout << "[MAIN] Generated Real-Time High Controller Thread : " << 1 / double(threadPeriod) * 1e6 << " Hz" <<std::endl;

    refBodyLinearVelocity[0] = 0.0;
    refBodyLinearVelocity[1] = 0.0;
    refBodyLinearVelocity[2] = 0.0;
    refBodyAngularVelocity[0] = 0.0;
    refBodyAngularVelocity[1] = 0.0;
    refBodyAngularVelocity[2] = 0.0;

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

        if(lc.handleTimeout(2000000) <= 0)
        {
            std::cerr << "[ERROR] LCM handle Time Out." << std::endl;
            joyStick[0] = 0.0;
            joyStick[1] = 0.0;
            joyStick[2] = 0.0;
            joyStick[3] = 0.0;
            commandLists.EmergencyStop();
        }

        usleep(2000);
    }
    return nullptr;
}