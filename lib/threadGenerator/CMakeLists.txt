cmake_minimum_required(VERSION 3.21)
project(threadGenerator)


add_library(${PROJECT_NAME} STATIC
        threadGenerator.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        .
)
target_link_libraries(${PROJECT_NAME}
        pthread
)