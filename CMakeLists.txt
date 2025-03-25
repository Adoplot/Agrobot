cmake_minimum_required(VERSION 3.26)
project(robotarm)

set(CMAKE_CXX_STANDARD 20)

# Add libraries
include_directories(${CMAKE_SOURCE_DIR}/Dependencies/json/include/)
include_directories(${CMAKE_SOURCE_DIR}/Dependencies/Eigen)
set(CODEGEN_DIR "${CMAKE_SOURCE_DIR}/Dependencies/matlab-ik-ARM")
include_directories(${CODEGEN_DIR})

# Find all source files for compilation
file(GLOB GENERATED_SRC "${CODEGEN_DIR}/*.c" "${CODEGEN_DIR}/*.cpp")


add_executable(robot main.cpp
        connection_handler.cpp
        connection_handler.h
        enet_handler.cpp
        enet_handler.h
        onltrack_handler.cpp
        onltrack_handler.h
        compv_handler.cpp
        compv_handler.h
        transform_calc.cpp
        transform_calc.h
        robot_api.cpp
        robot_api.h
        ${GENERATED_SRC}
        ik_wrapper.cpp
        ik_wrapper.h
)


