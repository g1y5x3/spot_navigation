import os
import yaml
import math

import ament_index_python.packages
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    driver_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_driver')
    driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP32C-velodyne_driver_node-params.yaml')
    with open(driver_params_file, 'r') as f:
        driver_params = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']

    convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(convert_share_dir, 'config', 'VLP32C-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VeloView-VLP-32C.yaml')

    # Static transform from robot to lidar
    x_translation = -0.127
    y_translation = 0.0
    z_translation = 0.3048
    
    roll  = 0.0
    pitch = 0.0
    yaw   = -1.5708
    
    # Define the static transform node
    static_transform_node = ComposableNode(
        package='tf2_ros',
        plugin='tf2_ros::StaticTransformBroadcasterNode',
        name='static_transform_broadcaster_lidar',
        parameters=[{
            'frame_id': 'base_link',         # Assumed parent frame
            'child_frame_id': 'velodyne',    # Assumed lidar frame
            'translation.x': x_translation,
            'translation.y': y_translation,
            'translation.z': z_translation,
            'rotation.roll':  roll,
            'rotation.pitch': pitch,
            'rotation.yaw':   yaw
        }]
    )

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

    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_broadcaster_lidar',
        arguments=[
            '--x', '-0.127',
            '--y', '0.0',
            '--z', '0.3048',
            '--yaw', '-1.5708',
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'velodyne'
        ]
    )

    return LaunchDescription([container, static_transform_node])