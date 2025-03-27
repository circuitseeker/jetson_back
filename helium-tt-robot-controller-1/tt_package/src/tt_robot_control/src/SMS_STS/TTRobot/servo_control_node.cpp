#include "tt_robot_control/servo_control_node.hpp"

namespace tt_robot_control {

ServoControlNode::ServoControlNode() : Node("servo_control_node")
{
    // Declare parameters
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("baud_rate", 1000000);
    
    // Get parameters
    std::string serial_port = this->get_parameter("serial_port").as_string();
    int baud_rate = this->get_parameter("baud_rate").as_int();
    
    // Initialize servo controller
    if(!sm_st.begin(baud_rate, serial_port.c_str())) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize servo controller!");
        return;
    }
    
    // Create subscriber for joint angles
    joint_angles_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "joint_angles", 10,
        std::bind(&ServoControlNode::joint_angles_callback, this, std::placeholders::_1));
        
    RCLCPP_INFO(this->get_logger(), "Servo control node initialized successfully");
}

ServoControlNode::~ServoControlNode()
{
    sm_st.end();
}

void ServoControlNode::joint_angles_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // Check if we have enough angles (we need 4 angles: 1,3,5,7)
    if (msg->data.size() < 4) {
        RCLCPP_ERROR(this->get_logger(), "Received %zu angles, need 4 angles", msg->data.size());
        return;
    }
    
    float angle1 = msg->data[0];
    float angle3 = msg->data[1];
    float angle5 = msg->data[2];
    float angle7 = msg->data[3];
    
    // Calculate mirrored angles for servos 2, 4, and 6
    float angle2 = 360.0f - angle1;
    float angle4 = 360.0f - angle3;
    float angle6 = 360.0f - angle5;
    
    // Convert angles to positions (0-4095)
    int16_t pos1 = convertToPosition(angle1);
    int16_t pos2 = convertToPosition(angle2);
    int16_t pos3 = convertToPosition(angle3);
    int16_t pos4 = convertToPosition(angle4);
    int16_t pos5 = convertToPosition(angle5);
    int16_t pos6 = convertToPosition(angle6);
    int16_t pos7 = convertToPosition(angle7);
    
    // Define servo speeds and acceleration
    const uint16_t speed = 1500;
    const uint8_t acc = 50;
    
    // Move servos to the specified positions
    sm_st.WritePosEx(1, pos1, speed, acc);
    sm_st.WritePosEx(2, pos2, speed, acc);
    sm_st.WritePosEx(3, pos3, speed, acc);
    sm_st.WritePosEx(4, pos4, speed, acc);
    sm_st.WritePosEx(5, pos5, speed, acc);
    sm_st.WritePosEx(6, pos6, speed, acc);
    sm_st.WritePosEx(7, pos7, speed, acc);
    
    RCLCPP_INFO(this->get_logger(), "Moved servos to angles: %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f",
                angle1, angle2, angle3, angle4, angle5, angle6, angle7);
}

int16_t ServoControlNode::convertToPosition(float angle) {
    // Clamp angle to 0-360 range
    angle = std::max(0.0f, std::min(360.0f, angle));
    // Map 0-360 degrees to 0-4095
    return static_cast<int16_t>(round((angle / 360.0f) * 4095.0f));
}

} // namespace tt_robot_control 