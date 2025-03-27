# Create the library target
add_library(scservo_lib STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/TTROBOT_SERVO/SCServo_Linux/SCS.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/TTROBOT_SERVO/SCServo_Linux/SCSCL.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/TTROBOT_SERVO/SCServo_Linux/SCSerial.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/TTROBOT_SERVO/SCServo_Linux/SMSBL.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/TTROBOT_SERVO/SCServo_Linux/SMSCL.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/TTROBOT_SERVO/SCServo_Linux/SMS_STS.cpp
)

# Set include directories for the library
target_include_directories(scservo_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/TTROBOT_SERVO/SCServo_Linux
)

# Add compiler flags
target_compile_options(scservo_lib PUBLIC
  -std=c++11
  -O3
)

# Link any system libraries needed by your library
target_link_libraries(scservo_lib
  pthread  # For serial communication
) 