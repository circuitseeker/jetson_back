#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "SMS_STS.h"
#include <cmath>
#include <vector>

class SCServoNode : public rclcpp::Node
{
public:
    SCServoNode() : Node("scservo_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 1000000);

        // Get parameters
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();

        // Initialize servo communication
        if (!sm_st_.begin(baud_rate, serial_port.c_str())) {
            RCLCPP_ERROR(this->get_logger(), "Failed to init SMS/STS motor!");
            return;
        }

        // Scan for connected servos and subscribe to joint_states
        scan_and_subscribe();
    }

    ~SCServoNode()
    {
        sm_st_.end();
    }

private:
    void scan_and_subscribe()
    {
        std::vector<uint8_t> connectedIDs;
        RCLCPP_INFO(this->get_logger(), "Scanning for servos with IDs 1-6...");

        for (int id = 1; id <= 6; id++) {
            if (sm_st_.Ping(id) != -1) {
                RCLCPP_INFO(this->get_logger(), "Found servo ID: %d", id);
                connectedIDs.push_back(id);
            }
        }

        if (connectedIDs.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No servos found! Check connections and try again.");
            return;
        }

        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&SCServoNode::joint_states_callback, this, std::placeholders::_1));
    }

    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Ensure we have at least 5 joint positions (as expected)
        if (msg->position.size() < 5) {
            RCLCPP_ERROR(this->get_logger(), "Received insufficient joint positions. Expected at least 5, got %zu", msg->position.size());
            return;
        }

        // Mapping:
        // msg->position[2] is for servo 1
        // msg->position[3] is for servo 3
        // msg->position[4] is for servo 5
        // Servos 2, 4, and 6 mirror servos 1, 3, and 5 respectively.

        // Convert radians to degrees
        // Note: The conversion for servo 1 uses a negative multiplier to account for its orientation
        float angle1 = (msg->position[2] * -57.2958f) + 180.0f;   // Servo 1
        float angle3 = (msg->position[3] * 57.2958f) + 180.0f;      // Servo 3
        float angle5 = (msg->position[4] * -57.2958f) + 180.0f;      // Servo 5

        // Calculate mirrored angles for servos 2, 4, and 6
        float angle2 = 360.0f - angle1; // Mirror of servo 1
        float angle4 = 360.0f - angle3; // Mirror of servo 3
        float angle6 = 360.0f - angle5; // Mirror of servo 5

        // Convert all angles to servo positions (0-4095 range)
        int16_t pos1 = convertToPosition(angle1);
        int16_t pos2 = convertToPosition(angle2);
        int16_t pos3 = convertToPosition(angle3);
        int16_t pos4 = convertToPosition(angle4);
        int16_t pos5 = convertToPosition(angle5);
        int16_t pos6 = convertToPosition(angle6);

        // Define servo speed and acceleration parameters
        const uint16_t speed = 3400;
        const uint8_t acc = 150;

        // Command servos 1 to 6
        sm_st_.WritePosEx(1, pos1, speed, acc);
        sm_st_.WritePosEx(2, pos2, speed, acc);
        sm_st_.WritePosEx(3, pos3, speed, acc);
        sm_st_.WritePosEx(4, pos4, speed, acc);
        sm_st_.WritePosEx(5, pos5, speed, acc);
        sm_st_.WritePosEx(6, pos6, speed, acc);

        RCLCPP_DEBUG(this->get_logger(), "Servo positions: 1:%d, 2:%d, 3:%d, 4:%d, 5:%d, 6:%d", 
                     pos1, pos2, pos3, pos4, pos5, pos6);
    }

    // Function to convert a degree angle (0-360) to a servo position value (0-4095)
    int16_t convertToPosition(float angle)
    {
        // Clamp angle between 0 and 360 degrees
        if (angle < 0.0f) {
            angle = 0.0f;
        } else if (angle > 360.0f) {
            angle = 360.0f;
        }
        // Map 0-360 degrees to 0-4095
        return static_cast<int16_t>(round((angle / 360.0f) * 4095.0f));
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    SMS_STS sm_st_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SCServoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

