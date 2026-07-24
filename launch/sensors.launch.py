from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch the Velodyne, IMU, thermal camera, and SCD41 drivers, and provide
    static transforms for the mounted sensors.
    """
    spot_nav_pkg = FindPackageShare('spot_navigation')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz with sensor outputs and the static TF tree.'
    )

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

    scd41_node = Node(
        package='scd41_reader',
        executable='scd41_reader',
        name='scd41_reader',
        output='screen'
    )

    # Static transform from base to sensor_base
    static_transform_base_to_mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_broadcaster_base_to_sensor_base',
        arguments=[
            '--x', '-0.1015',
            '--y', '0.0',
            '--z', '0.0805',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'sensor_base'
        ]
    )

    # Static transform for Velodyne LiDAR from sensor_base
    static_transform_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_broadcaster_sensor_base_to_velodyne',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.2169',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'sensor_base',
            '--child-frame-id', 'velodyne'
        ]
    )

    # Static transform for IMU from sensor_base
    static_transform_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_broadcaster_sensor_base_to_imu',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0875',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'sensor_base',
            '--child-frame-id', 'imu_link'
        ]
    )

    # Static transform for Thermal Camera from sensor_base
    static_transform_thermal = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_broadcaster_sensor_base_to_thermal',
        arguments=[
            '--x', '0.0876',
            '--y', '-0.00356',
            '--z', '0.1418',
            '--roll', '-1.5708',
            '--pitch', '0.0',
            '--yaw', '-1.5708',
            '--frame-id', 'sensor_base',
            '--child-frame-id', 'thermal_link'
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='sensors_rviz',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([spot_nav_pkg, 'rviz', 'sensors.rviz'])
        ],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        static_transform_base_to_mount,
        static_transform_velodyne,
        static_transform_imu,
        static_transform_thermal,
        velodyne_launch,
        imu_launch,
        thermal_launch,
        scd41_node,
        rviz_node,
    ])
