#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

class JointStatesMonitorNode : public rclcpp::Node
{
public:
    JointStatesMonitorNode() : Node("joint_states_monitor_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Joint States Monitor Node...");
        
        // Declare parameters
        this->declare_parameter<std::string>("esp32_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);

        // Get parameters
        std::string esp32_port = this->get_parameter("esp32_port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Using ESP32 port: %s, baud rate: %d", esp32_port.c_str(), baud_rate);

        // Initialize serial communication
        serial_fd_ = open(esp32_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open ESP32 port: %s (Error: %s)", 
                        esp32_port.c_str(), strerror(errno));
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully opened ESP32 port");

        // Configure serial port
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get serial port attributes: %s", strerror(errno));
            close(serial_fd_);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Got serial port attributes");

        // Set baud rate
        speed_t baud = B115200;  // Default to 115200
        if (baud_rate == 9600) baud = B9600;
        else if (baud_rate == 19200) baud = B19200;
        else if (baud_rate == 38400) baud = B38400;
        else if (baud_rate == 57600) baud = B57600;
        else if (baud_rate == 115200) baud = B115200;
        else if (baud_rate == 230400) baud = B230400;
        else if (baud_rate == 460800) baud = B460800;
        else if (baud_rate == 921600) baud = B921600;

        if (cfsetispeed(&tty, baud) < 0 || cfsetospeed(&tty, baud) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set baud rate: %s", strerror(errno));
            close(serial_fd_);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Set baud rate successfully");

        // Set other serial port parameters
        tty.c_cflag &= ~PARENB;  // No parity
        tty.c_cflag &= ~CSTOPB;  // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;      // 8 data bits
        tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Enable receiver and ignore modem control lines
        tty.c_lflag &= ~ICANON;  // Disable canonical mode
        tty.c_lflag &= ~ECHO;    // Disable echo
        tty.c_lflag &= ~ECHOE;   // Disable erasure
        tty.c_lflag &= ~ECHONL;  // Disable new-line echo
        tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
        tty.c_oflag &= ~OPOST;   // Disable output processing
        tty.c_oflag &= ~ONLCR;   // Don't translate CR to NL

        // Apply the settings
        if (tcsetattr(serial_fd_, TCSANOW, &tty) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to apply serial port settings: %s", strerror(errno));
            close(serial_fd_);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Applied serial port settings successfully");

        // Subscribe to joint_states topic
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&JointStatesMonitorNode::joint_states_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Joint States Monitor Node initialized successfully");
    }

    ~JointStatesMonitorNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            RCLCPP_INFO(this->get_logger(), "Closed ESP32 serial port");
        }
    }

private:
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received joint_states message");
        
        // Ensure we have at least 2 joint positions
        if (msg->position.size() < 2) {
            RCLCPP_ERROR(this->get_logger(), "Received insufficient joint positions. Expected at least 2, got %zu", msg->position.size());
            return;
        }

        // Joint 1 is prismatic (linear motion in cm)
        // Joint 2 is revolute (angular motion in degrees)
        float position1 = msg->position[0] * 100.0f;  // Convert to cm
        float angle2 = msg->position[1] * 57.2958f;   // Convert radians to degrees

        // Format the command string
        std::string command = std::to_string(position1) + " " + std::to_string(angle2) + "\n";

        // Send to ESP32
        ssize_t bytes_written = write(serial_fd_, command.c_str(), command.length());
        if (bytes_written < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write to ESP32: %s", strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent %zd bytes to ESP32: %s", bytes_written, command.c_str());
        }

        // Print the joint positions for debugging
        RCLCPP_INFO(this->get_logger(), "Joint positions - Joint 1 (cm): %.2f, Joint 2 (degrees): %.2f°", position1, angle2);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    int serial_fd_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStatesMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 