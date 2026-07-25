from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_name = "spot_navigation"

    pcd_file_arg = DeclareLaunchArgument(
        "pcd_file",
        description="Container-visible path to the PCD map",
    )
    boundary_file_arg = DeclareLaunchArgument(
        "boundary_file",
        default_value="",
        description=(
            "Boundary PLY path; defaults to <pcd_file stem>_boundary.ply"
        ),
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="map",
        description="Frame assigned to the PCD and captured poses",
    )
    output_mission_file_arg = DeclareLaunchArgument(
        "output_mission_file",
        description="Output mission YAML path",
    )
    output_preview_file_arg = DeclareLaunchArgument(
        "output_preview_file",
        default_value="",
        description="Output mission PNG; defaults to the mission YAML stem",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz with mission-authoring displays and tools",
    )

    pcd_publisher_node = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        output="screen",
        parameters=[
            {
                "file_name": ParameterValue(
                    LaunchConfiguration("pcd_file"),
                    value_type=str,
                ),
                "tf_frame": ParameterValue(
                    LaunchConfiguration("frame_id"),
                    value_type=str,
                ),
                "publishing_period_ms": 1000,
            }
        ],
    )

    boundary_publisher_node = Node(
        package=package_name,
        executable="boundary_marker_publisher",
        name="boundary_marker_publisher",
        output="screen",
        parameters=[
            {
                "pcd_file": ParameterValue(
                    LaunchConfiguration("pcd_file"),
                    value_type=str,
                ),
                "boundary_file": ParameterValue(
                    LaunchConfiguration("boundary_file"),
                    value_type=str,
                ),
                "frame_id": ParameterValue(
                    LaunchConfiguration("frame_id"),
                    value_type=str,
                ),
            }
        ],
    )

    recorder_node = Node(
        package=package_name,
        executable="mission_recorder",
        name="mission_recorder",
        output="screen",
        parameters=[
            {
                "frame_id": ParameterValue(
                    LaunchConfiguration("frame_id"),
                    value_type=str,
                ),
                "output_mission_file": ParameterValue(
                    LaunchConfiguration("output_mission_file"),
                    value_type=str,
                ),
                "pcd_file": ParameterValue(
                    LaunchConfiguration("pcd_file"),
                    value_type=str,
                ),
                "output_preview_file": ParameterValue(
                    LaunchConfiguration("output_preview_file"),
                    value_type=str,
                ),
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="mission_recorder_rviz",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    get_package_share_directory(package_name),
                    "rviz",
                    "mission_recorder.rviz",
                ]
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            pcd_file_arg,
            boundary_file_arg,
            frame_id_arg,
            output_mission_file_arg,
            output_preview_file_arg,
            rviz_arg,
            pcd_publisher_node,
            boundary_publisher_node,
            recorder_node,
            rviz_node,
        ]
    )
