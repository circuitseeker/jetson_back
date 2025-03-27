#ifndef TT_ROBOT_CONTROL_SERVO_CONTROL_NODE_HPP_
#define TT_ROBOT_CONTROL_SERVO_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "SCServo.h"

namespace tt_robot_control {

class ServoControlNode : public rclcpp::Node
{
public:
    ServoControlNode();
    ~ServoControlNode();
    
private:
    void joint_angles_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    int16_t convertToPosition(float angle);
    
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_angles_sub_;
    SMS_STS sm_st;
};

} // namespace tt_robot_control

#endif // TT_ROBOT_CONTROL_SERVO_CONTROL_NODE_HPP_ 