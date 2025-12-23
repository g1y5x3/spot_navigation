from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch the thermal camera node with arguments."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_id',
            default_value='0',
            description='The V4L2 device ID for the thermal camera.'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='thermal_link',
            description='The frame ID for the published thermal image.'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='30.0',
            description='The rate at which to publish the thermal image (in Hz).'
        ),

        Node(
            package='thermal_tc001',
            executable='thermal_node',
            name='thermal_camera_node',
            output='screen',
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }]
        )
    ])
