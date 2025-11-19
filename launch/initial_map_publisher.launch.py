from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_to_fiducial_file',
            default_value='config/map2fiducial.yaml',
            description='Path to the map to fiducial file.'
        ),
        DeclareLaunchArgument(
            'output_file',
            default_value='config/localization.yaml',
            description='Path to the output file for the initial transform.'
        ),
        Node(
            package='spot_navigation',
            executable='initial_map_publisher',
            name='initial_map_publisher',
            output='screen',
            parameters=[{
                'map_to_fiducial_file': LaunchConfiguration('map_to_fiducial_file'),
                'output_file': LaunchConfiguration('output_file')
            }]
        )
    ])
