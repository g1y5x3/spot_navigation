import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    # --- Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false',
        description='Use simulation time'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file', 
        default_value='far_planner.yaml',
        description='FAR Planner config file name (in spot_navigation/config folder)'
    )

    # Prior map path argument
    prior_map_path_arg = DeclareLaunchArgument(
        'prior_map_path',
        default_value=PathJoinSubstitution([
            get_package_share_directory('spot_navigation'),
            'map',
            'experimental_mine.vgh'
        ]),
        description='Path to prior map .vgh file (auto-loaded after 5s delay)'
    )

    # Boolean to enable/disable auto-load
    load_prior_map_arg = DeclareLaunchArgument(
        'load_prior_map',
        default_value='true',
        description='Auto-load prior map on startup'
    )

    # Config Path - now points to spot_navigation package
    far_planner_config_path = PathJoinSubstitution([
        get_package_share_directory('spot_navigation'),
        'config',
        LaunchConfiguration('config_file')
    ])

    # FAR Planner Node
    far_planner_node = Node(
        package='far_planner',
        executable='far_planner',
        name='far_planner',
        output='screen',
        parameters=[
            far_planner_config_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # 1. Odometry: Connect FAR to FAST-LIO's global odometry
            ('/odom_world', '/odometry_map'),
            # 2. Global Mapping Input:
            # We keep this as the LIO-registered cloud. It is drift-corrected and ideal 
            # for building the consistent global visibility graph.
            # ('/terrain_cloud', '/cloud_registered'), 

            # 3. Dynamic Obstacle Detection Input:
            # We use the RAW cloud here as requested. This allows FAR to see 
            # obstacles that LIO might have filtered out.
            # FAR Planner will transform this to world frame using TF.
            # ('/scan_cloud', '/velodyne_points'),

            # 4. Local Clearance Check Input:
            # Uses the body-frame registered cloud for immediate collision checking.
            ('/terrain_local_cloud', '/terrain_cloud'),

            # 5. Output:
            # This topic (/way_point) needs to be consumed by your local controller.
            # ('/way_point', '/goal_manager/target_point') 
        ]
    )

    # Goal Markers Publisher Node
    goal_markers_node = Node(
        package='spot_navigation',
        executable='publish_goal_markers',
        name='goal_markers_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Graph Decoder - used for loading/saving prior maps
    graph_decoder_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('graph_decoder'), 
            '/launch/decoder.launch.py'
        ]),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # Timer to auto-load prior map after graph decoder initializes
    load_prior_map_timer = TimerAction(
        period=5.0,  # Wait 5 seconds for graph decoder to be ready
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once', '/read_file_dir', 
                    'std_msgs/msg/String', 
                    Command([
                        '"data: \'',
                        LaunchConfiguration('prior_map_path'),
                        '\'"'
                    ])
                ],
                shell=False,
                output='screen',
                condition=IfCondition(LaunchConfiguration('load_prior_map'))
            )
        ]
    )

    # RViz (Optional)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', PathJoinSubstitution([
    #         get_package_share_directory('spot_navigation'), 
    #         'rviz', 
    #         'spot.rviz'
    #     ])],
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )

    # Pure Pursuit Controller (from mpl_planner) - executes the local path
    pure_pursuit_controller_node = Node(
        package='mpl_planner',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'lookahead_distance': 0.5},
            {'linear_velocity': 0.2},
            {'goal_tolerance': 0.3},
            {'max_angular_velocity': 1.0},
            {'robot_frame': 'base_link'}
        ],
        remappings=[
            ('/local_path', '/far_path')
        ]
    )

    return LaunchDescription([
        # Sets use_sim_time for all nodes that don't explicitly override it, 
        # though passing it explicitly in parameters is also good practice.
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        
        use_sim_time_arg,
        config_file_arg,
        prior_map_path_arg,
        load_prior_map_arg,
        graph_decoder_launch,
        far_planner_node,
        goal_markers_node,
        load_prior_map_timer,
        pure_pursuit_controller_node,
        # rviz_node
    ])
