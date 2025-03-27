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
  src/SMS_STS/TTRobot/servo_control_main.cpp
  src/SMS_STS/TTRobot/servo_control_node.cpp
)

# Set C++ standard
target_compile_features(servo_control_node PUBLIC c_std_99 cxx_std_17)

# Set include directories
target_include_directories(servo_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SMS_STS/TTRobot
)

# Link dependencies properly
ament_target_dependencies(servo_control_node
  rclcpp
  std_msgs
)

# Install targets
install(TARGETS
  servo_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
  include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
