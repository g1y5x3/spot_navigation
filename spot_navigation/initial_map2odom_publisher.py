import numpy as np
import yaml

from geometry_msgs.msg import PoseStamped, TransformStamped
import rclpy
from rclpy.node import Node
import tf_transformations
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class InitialMap2OdomPublisher(Node):

    def __init__(self):
        super().__init__('initial_map2odom_publisher')
        self.declare_parameter('map_to_fiducial_file', 'src/spot_navigation/config/map2fiducial.yaml')
        self.declare_parameter('output_file', 'src/spot_navigation/config/localization.yaml')

        self.map_to_fiducial_file = self.get_parameter('map_to_fiducial_file').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            PoseStamped,
            '/fiducial_pose',
            self.fiducial_callback,
            10)

        self.static_broadcaster = StaticTransformBroadcaster(self)

        self.map_to_fiducial = self.load_map_to_fiducial()
        self.work_done = False

    def load_map_to_fiducial(self):
        try:
            with open(self.map_to_fiducial_file, 'r') as file:
                transform_data = yaml.safe_load(file)
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'map'
                t.child_frame_id = 'fiducial'
                t.transform.translation.x = float(transform_data['translation']['x'])
                t.transform.translation.y = float(transform_data['translation']['y'])
                t.transform.translation.z = float(transform_data['translation']['z'])
                t.transform.rotation.x = float(transform_data['rotation']['x'])
                t.transform.rotation.y = float(transform_data['rotation']['y'])
                t.transform.rotation.z = float(transform_data['rotation']['z'])
                t.transform.rotation.w = float(transform_data['rotation']['w'])
                return t
        except FileNotFoundError:
            self.get_logger().error(f"Fiducial map file not found at {self.map_to_fiducial_file}")
            return None

    def fiducial_callback(self, msg):
        if self.map_to_fiducial is None:
            return

        # Convert PoseStamped to TransformStamped for odom_lidar -> fiducial
        odom_lidar_to_fiducial = TransformStamped()
        odom_lidar_to_fiducial.header.stamp = msg.header.stamp
        odom_lidar_to_fiducial.header.frame_id = 'odom_lidar'
        odom_lidar_to_fiducial.child_frame_id = 'fiducial'
        odom_lidar_to_fiducial.transform.translation.x = msg.pose.position.x
        odom_lidar_to_fiducial.transform.translation.y = msg.pose.position.y
        odom_lidar_to_fiducial.transform.translation.z = msg.pose.position.z
        odom_lidar_to_fiducial.transform.rotation.x = msg.pose.orientation.x
        odom_lidar_to_fiducial.transform.rotation.y = msg.pose.orientation.y
        odom_lidar_to_fiducial.transform.rotation.z = msg.pose.orientation.z
        odom_lidar_to_fiducial.transform.rotation.w = msg.pose.orientation.w


        odom_lidar_to_fiducial_mat = self.transform_to_matrix(odom_lidar_to_fiducial.transform)
        fiducial_to_odom_lidar_mat = np.linalg.inv(odom_lidar_to_fiducial_mat)
        map_to_fiducial_mat = self.transform_to_matrix(self.map_to_fiducial.transform)
        map_to_odom_lidar_mat = np.dot(map_to_fiducial_mat, fiducial_to_odom_lidar_mat)

        # Convert matrix back to TransformStamped
        map_to_odom_lidar = self.matrix_to_transform(map_to_odom_lidar_mat, 'map', 'odom_lidar')

        # Publish the static transform
        self.static_broadcaster.sendTransform(map_to_odom_lidar)
        self.get_logger().info("Published map -> odom_lidar transform")

        # Save the transform to a YAML file
        self.save_transform_to_yaml(map_to_odom_lidar)

    def transform_to_matrix(self, transform):
        translation = np.array([
            transform.translation.x,
            transform.translation.y,
            transform.translation.z
        ])
        rotation = np.array([
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        ])
        return np.dot(
            tf_transformations.translation_matrix(translation),
            tf_transformations.quaternion_matrix(rotation)
        )

    def matrix_to_transform(self, matrix, parent_frame, child_frame):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        translation = tf_transformations.translation_from_matrix(matrix)
        rotation = tf_transformations.quaternion_from_matrix(matrix)
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]
        return t

    def save_transform_to_yaml(self, transform):
        transform_data = {
            '/**': {
                'ros__parameters': {
                    'dlo/localizationNode/initial_pose_use': True,
                    'dlo/localizationNode/initial_position/x': float(transform.transform.translation.x),
                    'dlo/localizationNode/initial_position/y': float(transform.transform.translation.y),
                    'dlo/localizationNode/initial_position/z': float(transform.transform.translation.z),
                    'dlo/localizationNode/initial_orientation/w': float(transform.transform.rotation.w),
                    'dlo/localizationNode/initial_orientation/x': float(transform.transform.rotation.x),
                    'dlo/localizationNode/initial_orientation/y': float(transform.transform.rotation.y),
                    'dlo/localizationNode/initial_orientation/z': float(transform.transform.rotation.z),
                }
            }
        }
        with open(self.output_file, 'w') as file:
            yaml.dump(transform_data, file, default_flow_style=False, sort_keys=False)
        self.get_logger().info(f"Saved transform to {self.output_file}")

def main(args=None):
    rclpy.init(args=args)
    node = InitialMap2OdomPublisher()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
