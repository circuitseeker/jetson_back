cmake_minimum_required(VERSION 3.8)
project(scservoros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add the external library as a subdirectory
add_subdirectory(third_party/scservo)

# Include directories
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/scservo/TTROBOT_SERVO/SCServo_Linux
)

# Create the ROS2 node executables
add_executable(scservo_node src/scservo_node.cpp)
add_executable(joint_states_monitor_node src/joint_states_monitor_node.cpp)

target_link_libraries(scservo_node
  scservo_lib
)

# Add ROS2 dependencies
ament_target_dependencies(scservo_node
  rclcpp
  std_msgs
  sensor_msgs
)

ament_target_dependencies(joint_states_monitor_node
  rclcpp
  std_msgs
  sensor_msgs
)

# Install targets
install(TARGETS
  scservo_node
  joint_states_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

