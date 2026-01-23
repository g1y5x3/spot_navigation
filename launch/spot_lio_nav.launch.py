from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    spot_nav_pkg = FindPackageShare('spot_navigation')
    spot_driver_pkg = FindPackageShare('spot_driver')

    # Arguments for spot driver
    password_arg = DeclareLaunchArgument(
        'password',
        default_value='6mpxu4ezpaqs',
        description='Password for Spot robot authentication'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odometry_frame',
        default_value='odom_lidar',
        description='Frame to use for odometry'
    )

    # 1. Launch Sensors (Velodyne, IMU, Thermal)
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([spot_nav_pkg, 'launch', 'sensors.launch.py'])
        )
    )

    # 2. Launch Fast-LIO Localization
    lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([spot_nav_pkg, 'launch', 'fast_lio_localization.launch.py'])
        ),
        launch_arguments={
            'rviz': 'false' # Usually we don't want multiple rviz instances if launching from a top level
        }.items()
    )

    # 3. Launch Spot Driver (Delayed by 5 seconds)
    spot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([spot_driver_pkg, 'launch', 'spot_driver.launch.py'])
        ),
        launch_arguments={
            'password': LaunchConfiguration('password'),
            'odometry_frame': LaunchConfiguration('odometry_frame')
        }.items()
    )

    delayed_spot_driver = TimerAction(
        period=5.0,
        actions=[spot_driver_launch]
    )

    return LaunchDescription([
        password_arg,
        odom_frame_arg,
        sensors_launch,
        lio_launch,
        delayed_spot_driver
    ])