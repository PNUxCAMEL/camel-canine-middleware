cmake_minimum_required(VERSION 3.21)
project(canine_high_level_api)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)

add_subdirectory(lib)

add_executable(${PROJECT_NAME}
        src/JoystickCommand.cpp
        src/Gamepad.cpp
        src/RobotController.cpp
        include/DataProcessing.hpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
)

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        threadGenerator
        SharedMemory
        Eigen3::Eigen
)