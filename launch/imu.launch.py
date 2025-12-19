from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the WitMotion IMU driver node with configurable parameters."""

    # Declare launch arguments for the serial port and baud rate
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/imu_usb',
        description='Serial port for the IMU device.'
    )

    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200', # Assuming IMU is configured to 115200 after one-time setup
        description='Baud rate for serial communication with the IMU.'
    )

    # IMU driver node
    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu_driver_node', # A descriptive name
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud')
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data') # Remap the raw topic to a more common one
        ]
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        imu_node
    ])
