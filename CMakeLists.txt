set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(SOURCES
    motor_controller.cpp
    ${SRC_DIR}/bimotor.cpp
    ${SRC_DIR}/pwm.cpp
    ${SRC_DIR}/events.cpp
    ${SRC_DIR}/mecanumbot.cpp
    ${INCLUDE_DIR}/bimotor.hpp 
    ${INCLUDE_DIR}/pwm.hpp
    ${INCLUDE_DIR}/events.hpp
    ${INCLUDE_DIR}/mecanumbot.hpp
)
cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)
project(motor_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(motor_controller
    ${SOURCES}
)
target_include_directories(motor_controller PRIVATE ${INCLUDE_DIR})
target_link_libraries(motor_controller pico_stdlib hardware_pwm)
pico_add_extra_outputs(motor_controller)
