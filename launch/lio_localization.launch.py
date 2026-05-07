from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    spot_nav_pkg = "spot_navigation"

    # Arguments
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value=PathJoinSubstitution(
            [get_package_share_directory(spot_nav_pkg), "map", "microgrid_transformed.pcd"]
        ),
        description="Path to the global PCD map file",
    )

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

    # Config path (shared by all nodes)
    config_path = PathJoinSubstitution(
        [
            get_package_share_directory(spot_nav_pkg),
            "config",
            LaunchConfiguration("config_file"),
        ]
    )

    # 1. Terrain Processor (publishes /global_map and /terrain_cloud that has segmented ground points and obstacle points)
    terrain_processor_node = Node(
        package="terrain_analysis",
        executable="terrain_processor",
        name="terrain_processor",
        output="screen",
        parameters=[
            config_path,
            {
                "map_path": LaunchConfiguration("map_path"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[("/scan_cloud", "/cloud_registered_body")],
    )

    # 2. FAST-LIO (Odometry Mode)
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
        remappings=[("/Odometry", "/odometry_lio")],
    )

    # 3. NDT Localization Node
    localization_node = Node(
        package="ndt_localization",
        executable="localization_node",
        name="localization_node",
        output="screen",
        parameters=[
            config_path,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # RViz
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
            map_path_arg,
            use_sim_time_arg,
            config_file_arg,
            rviz_arg,
            terrain_processor_node,
            fast_lio_node,
            localization_node,
            rviz_node,
        ]
    )
