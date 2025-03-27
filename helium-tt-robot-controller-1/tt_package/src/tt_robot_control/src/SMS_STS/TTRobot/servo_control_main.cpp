#include <rclcpp/rclcpp.hpp>
#include "tt_robot_control/servo_control_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tt_robot_control::ServoControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 