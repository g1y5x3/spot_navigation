from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch file that combines Velodyne, IMU, and Thermal camera drivers,
    and provides static transforms between them.
    """
    spot_nav_pkg = FindPackageShare('spot_navigation')

    # Include Velodyne VLP-16 launch file
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [spot_nav_pkg, 'launch', 'velodyne.VLP16.launch.py']
            )
        )
    )

    # Include IMU launch file
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [spot_nav_pkg, 'launch', 'imu.launch.py']
            )
        )
    )

    # Include thermal camera launch file
    thermal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [spot_nav_pkg, 'launch', 'thermal.launch.py']
            )
        )
    )

    # Static transform for Velodyne LiDAR from base_link
    # Using values from velodyne.VLP32C.launch.py as a reference
    static_transform_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_broadcaster_base_to_velodyne',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.2327',
            '--yaw', '1.5708',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'sensor_base',
            '--child-frame-id', 'velodyne'
        ]
    )

    # Static transform for IMU from base_link
    static_transform_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_broadcaster_base_to_imu',
        arguments=[
            '--x', '0.0',
            '--y', '-0.085',
            '--z', '0.005',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '1.5708',
            '--frame-id', 'sensor_base',
            '--child-frame-id', 'imu_link'
        ]
    )

    # Static transform for Thermal Camera from base_link
    static_transform_thermal = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_broadcaster_base_to_thermal',
        arguments=[
            '--x', '-0.00394',
            '--y', '0.0829',
            '--z', '0.096925',
            '--roll', '0.0',
            '--pitch', '1.5708',
            '--yaw', '1.5708',
            '--frame-id', 'sensor_base',
            '--child-frame-id', 'thermal_link'
        ]
    )

    return LaunchDescription([
        velodyne_launch,
        imu_launch,
        thermal_launch,
        static_transform_velodyne,
        static_transform_imu,
        static_transform_thermal,
    ])
