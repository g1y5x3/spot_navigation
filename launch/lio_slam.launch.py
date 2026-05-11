from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    spot_nav_pkg = "spot_navigation"

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (bag) time if true",
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="lio_localization.yaml",
        description="Config file name (in spot_navigation/config folder)",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false", description="Start RViz"
    )

    config_path = PathJoinSubstitution(
        [
            get_package_share_directory(spot_nav_pkg),
            "config",
            LaunchConfiguration("config_file"),
        ]
    )

    static_transform_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_broadcaster_map_to_odom_lidar",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "map",
            "--child-frame-id", "odom_lidar",
        ],
    )

    terrain_processor_node = Node(
        package="terrain_analysis",
        executable="terrain_processor",
        name="terrain_processor",
        output="screen",
        parameters=[
            config_path,
            {
                "map_path": "",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[("/scan_cloud", "/cloud_registered_body")],
    )

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[
            config_path,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "publish.map_en": False,
                "pcd_save.pcd_save_en": False,
                "mapping.extrinsic_est_en": False,
                "publish.scan_publish_en": True,
                "publish.scan_bodyframe_pub_en": True,
                "publish.dense_publish_en": False,
            },
        ],
        remappings=[("/Odometry", "/odometry_map")],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [get_package_share_directory(spot_nav_pkg), "rviz", "localization.rviz"]
            ),
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            config_file_arg,
            rviz_arg,
            static_transform_map_to_odom,
            terrain_processor_node,
            fast_lio_node,
            rviz_node,
        ]
    )
