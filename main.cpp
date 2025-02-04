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
#include "RPLidarA1.hpp"
#include "ROSCommunication.hpp"

SharedMemory* sharedMemory = SharedMemory::getInstance();

CommandLists commandLists;

void* sendRobotCommand_udp(void* arg);
void* receiveRobotStatus_tcp(void* arg);
void* highController(void* arg);
void* KeyListener(void* arg);
void* receiveLidarData(void* arg);

void printBaseState()
{
    printf("(%02d:%02d:%02d) Print base states\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    std::cout << "\t\t\t\tbase velocity in body frame:\n"
                << "\t\t\t\t\tv_x:   " << sharedMemory->bodyBaseVelocity[0] << "m/s\n"
                << "\t\t\t\t\tv_y:   " << sharedMemory->bodyBaseVelocity[1] << "m/s\n"
                << "\t\t\t\t\tw_yaw: " << sharedMemory->bodyBaseAngularVelocity[2] << "rad/s"<< std::endl;
    std::cout << "\t\t\t\tbase Euler angle:\n"
                << "\t\t\t\t\troll:  " << sharedMemory->globalBaseEulerAngle[0] * 180.0 / 3.141592 << "deg\n"
                << "\t\t\t\t\tpitch: " << sharedMemory->globalBaseEulerAngle[1] * 180.0 / 3.141592 << "deg\n"
                << "\t\t\t\t\tyaw:   " << sharedMemory->globalBaseEulerAngle[2] * 180.0 / 3.141592 << "deg\n"<< std::endl;
}

void printEndEffectorState()
{
    printf("(%02d:%02d:%02d) Print end-effector states\n",(int)(sharedMemory->localTime/3600),((int)sharedMemory->localTime%3600)/60,(int)sharedMemory->localTime%60);
    std::cout << "\t\t\t\t End effector position in arm base frame:\n"
                << "\t\t\t\t\tx: "<< sharedMemory->currentEndEffectorPosition[0] << "m\n"
                << "\t\t\t\t\ty: "<< sharedMemory->currentEndEffectorPosition[1] << "m\n"
                << "\t\t\t\t\tz: "<< sharedMemory->currentEndEffectorPosition[2] << "m"<< std::endl;
    std::cout << "\t\t\t\t End effector orientation in arm base frame:\n"
                << "\t\t\t\t\troll:  " << sharedMemory->currentEndEffectorEulerAngle[0] * 180.0 / 3.141592 << "deg\n"
                << "\t\t\t\t\tpitch: " << sharedMemory->currentEndEffectorEulerAngle[1] * 180.0 / 3.141592 << "deg\n"
                << "\t\t\t\t\tyaw:   " << sharedMemory->currentEndEffectorEulerAngle[2] * 180.0 / 3.141592 << "deg\n"<< std::endl;
}

int main(int argc, char** argv)
{
    pthread_t UDPthread;
    pthread_t TCPthread;
    pthread_t HighControlThread;
    pthread_t KeyListenerThread;
    pthread_t LidarThread;

    generateRtThread(LidarThread, receiveLidarData, "lidar", 1, 99, NULL);
    generateNrtThread(HighControlThread, highController, "highController", 5, NULL);
    generateNrtThread(UDPthread, sendRobotCommand_udp, "UDP_send", 6, NULL);
    generateNrtThread(TCPthread, receiveRobotStatus_tcp, "TCP_receive", 7, NULL);
    generateNrtThread(KeyListenerThread, KeyListener, "key_board", 4, NULL);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROSCommunication>());
    rclcpp::shutdown();

    while (true)
    {
        sleep(10000);
    }

    return 0;
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

    while (true)
    {
        char ch;
        read(STDIN_FILENO, &ch, 1); // 키 입력 읽기
        if (ch == 'e')
        {
            std::cerr << "[MAIN] 'e' key pressed! e-stop" << std::endl;
            commandLists.EmergencyStop();
            sleep(2);
            std::cerr << "[MAIN] Exit canine-middleware" << std::endl;
            exit(0);
        }
        tcflush(STDIN_FILENO, TCIFLUSH);
        usleep(5000); // CPU 사용량을 줄이기 위해 잠시 대기
    }
}

void* receiveLidarData(void* arg)
{
    std::cout << "[MAIN] Generated Lidar Thread." <<std::endl;
    RPLidarA1 rplidar;
    rplidar.Initialize();

    while (true)
    {
        rplidar.ReadLidarPoint();
    }
}

void* highController(void* arg)
{
    double dT = 0.05; // 20Hz Real-time thread
    const long threadPeriod = long(dT * 1e6);
    std::cout << "[MAIN] Generated Real-Time High Controller Thread : " << 1 / double(threadPeriod) * 1e6 << " Hz" <<std::endl;
    int prevRosCMD = -1;
    while (true)
    {
        int incomingRosCMD = sharedMemory->rosCommand.joyCommand;
        if (incomingRosCMD != prevRosCMD)
        {
            switch (incomingRosCMD)
            {
                case CommandLists::ROS_COMMAND::ROS_CMD_NO_INPUT:
                {
                    break;
                }
                case CommandLists::ROS_COMMAND::ROS_CMD_START:
                {
                    commandLists.Start();
                    break;
                }
                case CommandLists::ROS_COMMAND::ROS_CMD_EMERGENCY_STOP:
                {
                    commandLists.EmergencyStop();
                    break;
                }
                case CommandLists::ROS_COMMAND::ROS_CMD_STAND_UP:
                {
                    commandLists.HomeUp();
                    break;
                }
                case CommandLists::ROS_COMMAND::ROS_CMD_SIT_DOWN:
                {
                    commandLists.HomeDown();
                    break;
                }
                case CommandLists::ROS_COMMAND::ROS_CMD_TROT_STOP:
                {
                    commandLists.TrotStop();
                    break;
                }
                case CommandLists::ROS_COMMAND::ROS_CMD_TROT_SLOW:
                {
                    commandLists.TrotSlow();
                    break;
                }
                case CommandLists::ROS_COMMAND::ROS_CMD_RESTART:
                {
                    commandLists.Restart();
                    break;
                }
                case CommandLists::ROS_COMMAND::ROS_CMD_ARM_HOME:
                {
                    commandLists.ArmHome();
                    break;
                }
                case CommandLists::ROS_COMMAND::ROS_CMD_ARM_MOVE:
                {
                    double armGoalPosition[3];
                    double armGoalEulerAngle[3];

                    armGoalPosition[0] = sharedMemory->rosCommand.desiredEndEffectorPosition[0];
                    armGoalPosition[1] = sharedMemory->rosCommand.desiredEndEffectorPosition[1];
                    armGoalPosition[2] = sharedMemory->rosCommand.desiredEndEffectorPosition[2];
                    armGoalEulerAngle[0] = sharedMemory->rosCommand.desiredEndEffectorEulerAngle[0];
                    armGoalEulerAngle[1] = sharedMemory->rosCommand.desiredEndEffectorEulerAngle[1];
                    armGoalEulerAngle[2] = sharedMemory->rosCommand.desiredEndEffectorEulerAngle[2];
                    commandLists.ArmMove(armGoalPosition, armGoalEulerAngle);
                    break;
                }
                case CommandLists::ROS_COMMAND::ROS_CMD_ARM_TELE_ON:
                {
                    commandLists.ArmTeleOn();
                    break;
                }
                case CommandLists::ROS_COMMAND::ROS_CMD_ARM_TELE_OFF:
                {
                    commandLists.ArmTeleOff();
                    break;
                }
                case CommandLists::ROS_COMMAND::ROS_CMD_ARM_GRP_OPEN:
                {
                    commandLists.ArmGripperOpen();
                    break;
                }
                case CommandLists::ROS_COMMAND::ROS_CMD_ARM_GRP_CLOSE:
                {
                    commandLists.ArmGripperClose();
                    break;
                }
                default:
                {
                    std::perror("[MAIN] invalid ROS command.");
                    break;
                }
            }
        }

        switch (sharedMemory->FSMState)
        {
            case FSM::FSM_TROT_SLOW:
            {
                double refBodyVelocity[3];
                refBodyVelocity[0] = sharedMemory->rosCommand.userLinVel[0]; // reference x-axis velocity in body frame. [m/s]
                refBodyVelocity[1] = sharedMemory->rosCommand.userLinVel[1]; // reference y-axis velocity in body frame. [m/s]
                refBodyVelocity[2] = sharedMemory->rosCommand.userAngVel[2]; // reference yaw velocity. [rad/s]
                commandLists.SetBodyVelocity(refBodyVelocity);
                break;
            }
            default:
            {
                double refBodyVelocity[3];
                refBodyVelocity[0] = 0.0; // reference x-axis velocity in body frame. [m/s]
                refBodyVelocity[1] = 0.0; // reference y-axis velocity in body frame. [m/s]
                refBodyVelocity[2] = 0.0; // reference yaw velocity. [rad/s]
                commandLists.SetBodyVelocity(refBodyVelocity);
                break;
            }
        }

        switch (sharedMemory->armFSMState)
        {
            case ARM_FSM::ARM_TELE:
            {
                double armLinearVelocityRef[3];
                double armAngularVelocityRef[3];
                armLinearVelocityRef[0] = sharedMemory->rosCommand.desiredTeleOperationLinearVelocity[0]; // reference x-axis velocity in arm-base frame. [m/s]
                armLinearVelocityRef[1] = sharedMemory->rosCommand.desiredTeleOperationLinearVelocity[1]; // reference y-axis velocity in arm-base frame. [m/s]
                armLinearVelocityRef[2] = sharedMemory->rosCommand.desiredTeleOperationLinearVelocity[2]; // reference z-axis velocity in arm-base frame. [m/s]
                armAngularVelocityRef[0] = sharedMemory->rosCommand.desiredTeleOperationAngularVelocity[0]; // reference x-axis angular velocity in arm-base frame. [rad/s]
                armAngularVelocityRef[1] = sharedMemory->rosCommand.desiredTeleOperationAngularVelocity[1]; // reference y-axis angular velocity in arm-base frame. [rad/s]
                armAngularVelocityRef[2] = sharedMemory->rosCommand.desiredTeleOperationAngularVelocity[2]; // reference z-axis angular velocity in arm-base frame. [rad/s]
                commandLists.SetArmTeleoperationVelocity(armLinearVelocityRef,armAngularVelocityRef);
                break;
            }
            default:
            {
                double armLinearVelocityRef[3];
                double armAngularVelocityRef[3];
                armLinearVelocityRef[0] = 0.0; // reference x-axis velocity in arm-base frame. [m/s]
                armLinearVelocityRef[1] = 0.0; // reference y-axis velocity in arm-base frame. [m/s]
                armLinearVelocityRef[2] = 0.0; // reference z-axis velocity in arm-base frame. [m/s]
                armAngularVelocityRef[0] = 0.0; // reference x-axis angular velocity in arm-base frame. [rad/s]
                armAngularVelocityRef[1] = 0.0; // reference y-axis angular velocity in arm-base frame. [rad/s]
                armAngularVelocityRef[2] = 0.0; // reference z-axis angular velocity in arm-base frame. [rad/s]
                commandLists.SetArmTeleoperationVelocity(armLinearVelocityRef,armAngularVelocityRef);
                break;
            }
        }

        prevRosCMD = incomingRosCMD;
        usleep(dT * 1e6);
    }

    printBaseState();
    printEndEffectorState();
}