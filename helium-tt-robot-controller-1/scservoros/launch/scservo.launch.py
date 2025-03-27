from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the serial port parameter
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for servo communication'
    )
    
    # Declare the baud rate parameter
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='1000000',
        description='Baud rate for servo communication'
    )

    # Declare the ESP32 serial port parameter
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for ESP32 communication'
    )
    
    # Create the scservo node
    scservo_node = Node(
        package='scservoros',
        executable='scservo_node',
        name='scservo_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate')
        }]
    )
    
    # Create the joint states monitor node
    joint_states_monitor_node = Node(
        package='scservoros',
        executable='joint_states_monitor_node',
        name='joint_states_monitor_node',
        parameters=[{
            'esp32_port': LaunchConfiguration('esp32_port'),
            'baud_rate': 115200  # Fixed baud rate for ESP32
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        esp32_port_arg,
        scservo_node,
        joint_states_monitor_node
    ]) 