cmake_minimum_required(VERSION 3.8)
project(tt_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Create the servo control node executable
add_executable(servo_control_node
  servo_control_main.cpp
  servo_control_node.cpp
)

# Link libraries
target_link_libraries(servo_control_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# Install targets
install(TARGETS
  servo_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package() 