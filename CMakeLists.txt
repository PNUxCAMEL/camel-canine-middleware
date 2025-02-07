cmake_minimum_required(VERSION 3.21)
project(canine_middleware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(canine_msgs REQUIRED)

add_subdirectory(lib)

add_executable(${PROJECT_NAME}
        main.cpp
        src/Gamepad.cpp
        src/JoystickCommand.cpp
        src/TCPCommunication.cpp
        src/UDPCommunication.cpp
        src/CommandLists.cpp
        src/ROSCommunication.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
)

target_link_libraries(${PROJECT_NAME}
        threadGenerator
        SharedMemory
        Eigen3::Eigen
)

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        geometry_msgs
        canine_msgs
)

ament_package()