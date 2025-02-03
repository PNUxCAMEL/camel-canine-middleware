cmake_minimum_required(VERSION 3.21)
project(canine_middleware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(canine_msgs REQUIRED)

set(RPLIDAR_INCLUDE_PATH "$ENV{HOME}/Library/rplidar_sdk/sdk/include")
set(RPLIDAR_LIB "$ENV{HOME}/Library/rplidar_sdk/output/Linux/Release/libsl_lidar_sdk.a")

add_subdirectory(lib)

add_executable(${PROJECT_NAME}
        main.cpp
        src/Gamepad.cpp
        src/JoystickCommand.cpp
        src/TCPCommunication.cpp
        src/UDPCommunication.cpp
        src/CommandLists.cpp
        src/RPLidarA1.cpp
        src/ROSCommunication.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
        ${RPLIDAR_INCLUDE_PATH}
)

target_link_libraries(${PROJECT_NAME}
        threadGenerator
        SharedMemory
        Eigen3::Eigen
        ${RPLIDAR_LIB}
)

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        geometry_msgs
        canine_msgs
)

ament_package()