import os
import yaml
import math

import ament_index_python.packages
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    spot_nav_dir = ament_index_python.packages.get_package_share_directory('spot_navigation')
    driver_params_file = os.path.join(spot_nav_dir, 'config', 'VLP32C-velodyne_driver_node-params.yaml')
    with open(driver_params_file, 'r') as f:
        driver_params = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']

    convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(spot_nav_dir, 'config', 'VLP32C-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VeloView-VLP-32C.yaml')

    container = ComposableNodeContainer(
            name='velodyne_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='velodyne_driver',
                    plugin='velodyne_driver::VelodyneDriver',
                    name='velodyne_driver_node',
                    parameters=[driver_params]),
                ComposableNode(
                    package='velodyne_pointcloud',
                    plugin='velodyne_pointcloud::Transform',
                    name='velodyne_transform_node',
                    parameters=[convert_params]),
            ],
            output='both',
    )

    # Measured TF from LiDAR to base_link
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_broadcaster_lidar',
        arguments=[
            '--x',     '-0.127',
            '--y',     '0.0',
            '--z',     '0.3048',
            '--yaw',   '-1.5708',
            '--pitch', '0.0',
            '--roll',  '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'velodyne'
        ]
    )

    return LaunchDescription([
        container, 
        static_transform_node
    ])