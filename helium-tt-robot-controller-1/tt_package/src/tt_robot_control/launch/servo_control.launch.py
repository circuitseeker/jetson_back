from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the serial port parameter
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the servo controller'
    )
    
    # Declare the baud rate parameter
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='1000000',
        description='Baud rate for serial communication'
    )
    
    # Create the servo control node
    servo_node = Node(
        package='tt_robot_control',
        executable='servo_control_node',
        name='servo_control_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate')
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        servo_node
    ]) 