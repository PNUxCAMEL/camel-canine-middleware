//
// Created by jh on 24. 7. 3.
//

#include "TCPCommunication.hpp"

TCPCommunication::TCPCommunication()
{

}

int TCPCommunication::Initialize()
{
    // 서버 소켓 생성
    serverSocket = socket(PF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1)
    {
        std::cerr << "Cannot create socket" << std::endl;
        return -1;
    }

    int opt = 1;
    if (setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) {
        std::cerr << "setsockopt(SO_REUSEADDR) failed" << std::endl;
        close(serverSocket);
        return -1;
    }

    // 서버 주소 구조체 설정
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(60001);
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    memset(serverAddr.sin_zero, '\0', sizeof(serverAddr.sin_zero));

    // 소켓에 주소 바인드
    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1)
    {
        std::cerr << "Bind failed" << std::endl;
        close(serverSocket);
        return -1;
    }

    // 연결 대기
    if (listen(serverSocket, 50) == -1)
    {
        std::cerr << "Listen failed" << std::endl;
        close(serverSocket);
        return -1;
    }

    std::cout << "Server listening on port 60001" << std::endl;
    return 1;
}

void TCPCommunication::Read()
{
        addr_size = sizeof serverStorage;
        clientSocket = accept(serverSocket, (struct sockaddr*)&serverStorage, &addr_size);

        if (clientSocket == -1)
        {
            std::cerr << "Accept failed" << std::endl;
            return;
        }


        // 클라이언트로부터 데이터 길이를 먼저 수신
        int64_t expectedBytesNetworkOrder = 0;
        int bytesRead = recv(clientSocket, &expectedBytesNetworkOrder, sizeof(expectedBytesNetworkOrder), 0);
        if (bytesRead != sizeof(expectedBytesNetworkOrder))
        {
            std::cerr << "Error reading data size. Bytes read: " << bytesRead << std::endl;
            close(clientSocket);
            return;
        }

        int64_t expectedBytes64 = ntohll(expectedBytesNetworkOrder);  // 네트워크 바이트 오더를 호스트 바이트 오더로 변환
        if (expectedBytes64 > INT_MAX)
        {
            std::cerr << "Data size too large: " << expectedBytes64 << std::endl;
            close(clientSocket);
            return;
        }
        int expectedBytes = static_cast<int>(expectedBytes64);

        if (expectedBytes <= 0)
        {
            std::cerr << "Invalid data size: " << expectedBytes << std::endl;
            close(clientSocket);
            return;
        }

        // expectedBytes만큼 데이터를 수신
        std::vector<char> dataBuffer(expectedBytes);
        int totalBytesRead = 0;

        while (totalBytesRead < expectedBytes)
        {
            bytesRead = recv(clientSocket, dataBuffer.data() + totalBytesRead, expectedBytes - totalBytesRead, 0);
            if (bytesRead <= 0)
            {
                std::cerr << "Failed to receive full data. Bytes read: " << bytesRead << std::endl;
                close(clientSocket);
                continue;
            }
            totalBytesRead += bytesRead;
        }

        if (totalBytesRead == expectedBytes)
        {
            unpackingTCPmsg(dataBuffer);
        }

        close(clientSocket);
}

void TCPCommunication::CloseServer()
{
    close(serverSocket);
}

uint64_t TCPCommunication::htonll(uint64_t value) {
    if (__BYTE_ORDER == __LITTLE_ENDIAN) {
        return ((uint64_t)htonl(value & 0xFFFFFFFF) << 32) | htonl(value >> 32);
    } else {
        return value;
    }
}

uint64_t TCPCommunication::ntohll(uint64_t value) {
    if (__BYTE_ORDER == __LITTLE_ENDIAN) {
        return ((uint64_t)ntohl(value & 0xFFFFFFFF) << 32) | ntohl(value >> 32);
    } else {
        return value;
    }
}

// Converts a double to network byte order
uint64_t TCPCommunication::htond(double value) {
    uint64_t temp;
    memcpy(&temp, &value, sizeof(temp));
    temp = htonll(temp);
    return temp;
}

// Converts a network byte order double to host byte order
double TCPCommunication::ntohd(uint64_t value) {
    value = ntohll(value);
    double temp;
    memcpy(&temp, &value, sizeof(temp));
    return temp;
}

void TCPCommunication::unpackingEigenVector3d(const char*& dataPtr, Eigen::Vector3d& vec) {
    for (int i = 0; i < 3; ++i) {
        uint64_t networkOrder;
        memcpy(&networkOrder, dataPtr, sizeof(networkOrder));
        vec[i] = ntohd(networkOrder);
        dataPtr += sizeof(networkOrder);
    }
}

void TCPCommunication::unpackingEigenVector4d(const char*& dataPtr, Eigen::Vector4d& vec) {
    for (int i = 0; i < 4; ++i) {
        uint64_t networkOrder;
        memcpy(&networkOrder, dataPtr, sizeof(networkOrder));
        vec[i] = ntohd(networkOrder);
        dataPtr += sizeof(networkOrder);
    }
}

void TCPCommunication::unpackingTCPmsg(const std::vector<char>& dataBuffer)
{
    const char* dataPtr = dataBuffer.data();
    SharedMemory* sharedMemory = SharedMemory::getInstance();

    // Unpack sharedMemory->userCommand
    int userCommandNetworkOrder;
    memcpy(&userCommandNetworkOrder, dataPtr, sizeof(userCommandNetworkOrder));
    sharedMemory->command.userCommand = ntohl(userCommandNetworkOrder);
    dataPtr += sizeof(userCommandNetworkOrder);


    // Unpack sharedMemory->gaitCommand
    int gaitCommandNetworkOrder;
    memcpy(&gaitCommandNetworkOrder, dataPtr, sizeof(gaitCommandNetworkOrder));
    sharedMemory->command.gaitCommand = ntohl(gaitCommandNetworkOrder);
    dataPtr += sizeof(gaitCommandNetworkOrder);

    // Unpack sharedMemory->userParamInt
    for (int i = 0; i < MAX_COMMAND_DATA; ++i)
    {
        int userParamIntNetworkOrder;
        memcpy(&userParamIntNetworkOrder, dataPtr, sizeof(userParamIntNetworkOrder));
        sharedMemory->command.userParamInt[i] = ntohl(userParamIntNetworkOrder);
        dataPtr += sizeof(userParamIntNetworkOrder);
    }

    // Unpack sharedMemory->userParamDouble
    for (int i = 0; i < MAX_COMMAND_DATA; ++i)
    {
        uint64_t userParamDoubleNetworkOrder;
        memcpy(&userParamDoubleNetworkOrder, dataPtr, sizeof(userParamDoubleNetworkOrder));
        sharedMemory->command.userParamDouble[i] = ntohd(userParamDoubleNetworkOrder);
        dataPtr += sizeof(userParamDoubleNetworkOrder);
    }

    // Unpack sharedMemory->gaitTable
    for (int i = 0; i < MPC_HORIZON * 4; ++i)
    {
        int gaitTableNetworkOrder;
        memcpy(&gaitTableNetworkOrder, dataPtr, sizeof(gaitTableNetworkOrder));
        sharedMemory->gaitTable[i] = ntohl(gaitTableNetworkOrder);
        dataPtr += sizeof(gaitTableNetworkOrder);
    }

    // Unpack remaining boolean and integer fields in sharedMemory
    memcpy(&sharedMemory->isNan, dataPtr, sizeof(sharedMemory->isNan));
    dataPtr += sizeof(sharedMemory->isNan);

    memcpy(&sharedMemory->isRamp, dataPtr, sizeof(sharedMemory->isRamp));
    dataPtr += sizeof(sharedMemory->isRamp);

    memcpy(&sharedMemory->newCommand, dataPtr, sizeof(sharedMemory->newCommand));
    dataPtr += sizeof(sharedMemory->newCommand);

    memcpy(&sharedMemory->motorStatus, dataPtr, sizeof(sharedMemory->motorStatus));
    dataPtr += sizeof(sharedMemory->motorStatus);

    int stateNetworkOrder;
    memcpy(&stateNetworkOrder, dataPtr, sizeof(stateNetworkOrder));
    sharedMemory->FSMState = ntohl(stateNetworkOrder);
    dataPtr += sizeof(stateNetworkOrder);

    memcpy(&stateNetworkOrder, dataPtr, sizeof(stateNetworkOrder));
    sharedMemory->LowControlState = ntohl(stateNetworkOrder);
    dataPtr += sizeof(stateNetworkOrder);

    memcpy(&stateNetworkOrder, dataPtr, sizeof(stateNetworkOrder));
    sharedMemory->HighControlState = ntohl(stateNetworkOrder);
    dataPtr += sizeof(stateNetworkOrder);

    uint64_t localTimeNetworkOrder;
    memcpy(&localTimeNetworkOrder, dataPtr, sizeof(localTimeNetworkOrder));
    sharedMemory->localTime = ntohd(localTimeNetworkOrder);
    dataPtr += sizeof(localTimeNetworkOrder);

    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        int motorErrorStatusNetworkOrder;
        memcpy(&motorErrorStatusNetworkOrder, dataPtr, sizeof(motorErrorStatusNetworkOrder));
        sharedMemory->motorErrorStatus[i] = ntohl(motorErrorStatusNetworkOrder);
        dataPtr += sizeof(motorErrorStatusNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        int motorTempNetworkOrder;
        memcpy(&motorTempNetworkOrder, dataPtr, sizeof(motorTempNetworkOrder));
        sharedMemory->motorTemp[i] = ntohl(motorTempNetworkOrder);
        dataPtr += sizeof(motorTempNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorVoltageNetworkOrder;
        memcpy(&motorVoltageNetworkOrder, dataPtr, sizeof(motorVoltageNetworkOrder));
        sharedMemory->motorVoltage[i] = ntohd(motorVoltageNetworkOrder);
        dataPtr += sizeof(motorVoltageNetworkOrder);
    }
    const double PI = 3.141592;
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorPositionNetworkOrder;
        memcpy(&motorPositionNetworkOrder, dataPtr, sizeof(motorPositionNetworkOrder));
        sharedMemory->motorPosition[i] = ntohd(motorPositionNetworkOrder);
        sharedMemory->motorPosition[i] = sharedMemory->motorPosition[i] / PI * 180;
        dataPtr += sizeof(motorPositionNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorVelocityNetworkOrder;
        memcpy(&motorVelocityNetworkOrder, dataPtr, sizeof(motorVelocityNetworkOrder));
        sharedMemory->motorVelocity[i] = ntohd(motorVelocityNetworkOrder);
        dataPtr += sizeof(motorVelocityNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorTorqueNetworkOrder;
        memcpy(&motorTorqueNetworkOrder, dataPtr, sizeof(motorTorqueNetworkOrder));
        sharedMemory->motorTorque[i] = ntohd(motorTorqueNetworkOrder);
        dataPtr += sizeof(motorTorqueNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorDesiredPositionNetworkOrder;
        memcpy(&motorDesiredPositionNetworkOrder, dataPtr, sizeof(motorDesiredPositionNetworkOrder));
        sharedMemory->motorDesiredPosition[i] = ntohd(motorDesiredPositionNetworkOrder);
        dataPtr += sizeof(motorDesiredPositionNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorDesiredVelocityNetworkOrder;
        memcpy(&motorDesiredVelocityNetworkOrder, dataPtr, sizeof(motorDesiredVelocityNetworkOrder));
        sharedMemory->motorDesiredVelocity[i] = ntohd(motorDesiredVelocityNetworkOrder);
        dataPtr += sizeof(motorDesiredVelocityNetworkOrder);
    }
    for (int i = 0; i < MOTOR_NUM; ++i)
    {
        uint64_t motorDesiredTorqueNetworkOrder;
        memcpy(&motorDesiredTorqueNetworkOrder, dataPtr, sizeof(motorDesiredTorqueNetworkOrder));
        sharedMemory->motorDesiredTorque[i] = ntohd(motorDesiredTorqueNetworkOrder);
        dataPtr += sizeof(motorDesiredTorqueNetworkOrder);
    }

    unpackingEigenVector3d(dataPtr, sharedMemory->globalBasePosition);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseVelocity);
    unpackingEigenVector4d(dataPtr, sharedMemory->globalBaseQuaternion);

    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseDesiredPosition);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseDesiredVelocity);
    unpackingEigenVector4d(dataPtr, sharedMemory->globalBaseDesiredQuaternion);

    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseDesiredEulerAngle);
    unpackingEigenVector3d(dataPtr, sharedMemory->bodyBaseDesiredAngularVelocity);
    unpackingEigenVector3d(dataPtr, sharedMemory->bodyBaseDesiredVelocity);
    unpackingEigenVector3d(dataPtr, sharedMemory->bodyBaseVelocity);

    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseEulerAngle);
    unpackingEigenVector3d(dataPtr, sharedMemory->bodyBaseAngularVelocity);
    unpackingEigenVector3d(dataPtr, sharedMemory->globalBaseAcceleration);

    for (int i = 0; i < 4; ++i)
    {
        unpackingEigenVector3d(dataPtr, sharedMemory->pdTorque[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->mpcTorque[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->globalFootPosition[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->bodyBase2FootPosition[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->bodyBase2FootVelocity[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->bodyBase2FootDesiredPosition[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->bodyBase2FootDesiredVelocity[i]);
        unpackingEigenVector3d(dataPtr, sharedMemory->solvedGRF[i]);
    }

    memcpy(sharedMemory->contactState, dataPtr, 4 * sizeof(bool));
    dataPtr += 4 * sizeof(bool);

    for (int i = 0; i < 3; ++i)
    {
        uint64_t testBasePosNetworkOrder;
        memcpy(&testBasePosNetworkOrder, dataPtr, sizeof(testBasePosNetworkOrder));
        sharedMemory->testBasePos[i] = ntohd(testBasePosNetworkOrder);
        dataPtr += sizeof(testBasePosNetworkOrder);
    }

    for (int i = 0; i < 3; ++i)
    {
        uint64_t testBaseVelNetworkOrder;
        memcpy(&testBaseVelNetworkOrder, dataPtr, sizeof(testBaseVelNetworkOrder));
        sharedMemory->testBaseVel[i] = ntohd(testBaseVelNetworkOrder);
        dataPtr += sizeof(testBaseVelNetworkOrder);
    }

    for (int i = 0; i < 11; ++i)
    {
        uint64_t threadElapsedTimeNetworkOrder;
        memcpy(&threadElapsedTimeNetworkOrder, dataPtr, sizeof(threadElapsedTimeNetworkOrder));
        sharedMemory->threadElapsedTime[i] = ntohd(threadElapsedTimeNetworkOrder);
        dataPtr += sizeof(threadElapsedTimeNetworkOrder);
    }
}