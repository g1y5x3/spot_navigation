import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    fast_lio_pkg = 'fast_lio'
    spot_nav_pkg = 'spot_navigation'

    # Arguments
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=PathJoinSubstitution([
            get_package_share_directory(spot_nav_pkg),
            'map',
            'experimental_mine.pcd'
        ]),
        description='Path to the global PCD map file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bag) time if true'
    )

    publish_goals_arg = DeclareLaunchArgument(
        'publish_goals',
        default_value='false',
        description='Whether to publish goal markers from goals.yaml for visualization'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='velodyne_vlp16.yaml',
        description='FAST-LIO config file name (in spot_navigation/config folder)'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz'
    )

    # Config Path - NOW pointing to spot_navigation
    fast_lio_config_path = PathJoinSubstitution([
        get_package_share_directory(spot_nav_pkg),
        'config',
        LaunchConfiguration('config_file')
    ])

    # 1. Global Map Server
    map_server_node = Node(
        package=fast_lio_pkg,
        executable='global_map_server',
        name='global_map_server',
        output='screen',
        parameters=[
            fast_lio_config_path,
            {
                'map_path': LaunchConfiguration('map_path'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    # 2. FAST-LIO (Odometry Mode)
    # The config file in spot_navigation already has odom_frame_id set to "odom_lidar",
    # but we keep the override here just in case or for explicit clarity if parameters are merged.
    fast_lio_node = Node(
        package=fast_lio_pkg,
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            fast_lio_config_path,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Overrides for Localization Mode
                'publish.map_en': False,
                'pcd_save.pcd_save_en': False,
                'mapping.extrinsic_est_en': False,
                'publish.scan_publish_en': True,
                'publish.scan_bodyframe_pub_en': True,
                'publish.dense_publish_en': False,
            }
        ]
    )

    # 3. Localization Node (NDT)
    localization_node = Node(
        package=fast_lio_pkg,
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[
            fast_lio_config_path,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    # 4. Goal Publisher (Optional)
    # We explicitly point it to the goals.yaml in spot_navigation
    goal_publisher_node = Node(
        package=fast_lio_pkg,
        executable='publish_goals.py',
        name='goal_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_goals')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'goals_file': PathJoinSubstitution([
                get_package_share_directory(spot_nav_pkg),
                'config',
                'goals.yaml'
            ])
        }]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([get_package_share_directory(spot_nav_pkg), 'rviz', 'nav.rviz'])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        map_path_arg,
        use_sim_time_arg,
        publish_goals_arg,
        config_file_arg,
        rviz_arg,
        map_server_node,
        fast_lio_node,
        localization_node,
        goal_publisher_node,
        rviz_node
    ])
