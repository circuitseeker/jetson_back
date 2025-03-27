cmake_minimum_required(VERSION 2.8)
project(SCServo)

add_library(SCServo SHARED
    SCS.cpp
    SCSCL.cpp
    SCSerial.cpp
    SMSBL.cpp
    SMSCL.cpp
    SMS_STS.cpp
)

target_include_directories(SCServo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})