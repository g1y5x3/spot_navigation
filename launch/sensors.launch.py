from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    Launch file that combines the Velodyne and IMU drivers and publishes
    static transforms derived from the CAD mount geometry.
    """
    spot_nav_pkg = FindPackageShare('spot_navigation')
    imu_port = LaunchConfiguration('port')
    imu_baud = LaunchConfiguration('baud')
    radio_port = LaunchConfiguration('radio_port')
    radio_baud = LaunchConfiguration('radio_baud')
    owon_mac = LaunchConfiguration('owon_mac_address')
    owon_model = LaunchConfiguration('owon_model')

    # Include the Velodyne launch file for the VLP32C variant used on this rig.
    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [spot_nav_pkg, 'launch', 'velodyne.VLP32C.launch.py']
            )
        )
    )

    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu_driver_node',
        output='screen',
        parameters=[{
            'port': imu_port,
            'baud': imu_baud,
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data')
        ]
    )

    # Static transform from base_link to the sensor mount reference frame.
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

    # CAD-derived sensor_base -> velodyne transform.
    static_transform_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_broadcaster_sensor_base_to_velodyne',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.1145',
            '--yaw', '0.0',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'sensor_base',
            '--child-frame-id', 'velodyne'
        ]
    )

    # CAD-derived sensor_base -> imu_link transform.
    static_transform_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_broadcaster_sensor_base_to_imu',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0195',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'sensor_base',
            '--child-frame-id', 'imu_link'
        ]
    )

    owon_node = Node(
        package='owon_driver',
        executable='owon_node',
        name='owon_driver_node',
        output='screen',
        parameters=[{
            'mac_address': owon_mac,
            'model': owon_model,
            'odom_topic': '/odometry_map',
            'target_frame': 'map',
        }]
    )

    radio_bridge_node = Node(
        package='spot_navigation',
        executable='radio_bridge',
        name='radio_bridge',
        output='screen',
        parameters=[{
            'port': radio_port,
            'baud': radio_baud,
            'odom_topic': '/odometry_map',
            'voltage_topic': '/owon/value',
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/imu_usb',
            description='Serial port for the IMU device.'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Baud rate for serial communication with the IMU.'
        ),
        DeclareLaunchArgument(
            'radio_port',
            default_value='/dev/radio_usb',
            description='Serial port connected to the long-range radio transmitter.'
        ),
        DeclareLaunchArgument(
            'radio_baud',
            default_value='57600',
            description='Baud rate for the long-range radio transmitter.'
        ),
        DeclareLaunchArgument(
            'owon_mac_address',
            default_value='A6:C0:80:91:58:C2',
            description='Bluetooth MAC address for the OWON multimeter.'
        ),
        DeclareLaunchArgument(
            'owon_model',
            default_value='cm2100b',
            description='OWON multimeter model identifier.'
        ),
        static_transform_base_to_mount,
        static_transform_velodyne,
        static_transform_imu,
        velodyne_launch,
        imu_node,
        owon_node,
        radio_bridge_node,
    ])
