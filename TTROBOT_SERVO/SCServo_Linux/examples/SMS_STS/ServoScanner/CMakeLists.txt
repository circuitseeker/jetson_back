cmake_minimum_required(VERSION 3.0.0)
project(ServoScanner VERSION 1.0.0)

# Include SCServo headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../)

# Create ServoScanner executable
add_executable(ServoScanner ServoScanner.cpp)

# Link against SCServo library
target_link_libraries(ServoScanner SCServo)

# Install the executable
install(TARGETS ServoScanner DESTINATION bin) 