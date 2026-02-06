#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory
import yaml
import os

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        self.declare_parameter('goals_file', '')
        goals_file = self.get_parameter('goals_file').value
        
        # Default path if parameter not set
        if not goals_file:
            try:
                pkg_share = get_package_share_directory('spot_navigation')
                goals_file = os.path.join(pkg_share, 'config', 'target_goals.yaml')
            except Exception as e:
                self.get_logger().error(f"Could not find package share directory: {e}")
                return

        self.get_logger().info(f"Loading goals from: {goals_file}")
        
        self.publisher_ = self.create_publisher(MarkerArray, 'goal_markers', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.markers = MarkerArray()
        self.load_goals(goals_file)

    def load_goals(self, file_path):
        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
                
            goals = data.get('goals', [])
            for i, goal in enumerate(goals):
                pos = goal['position']
                desc = goal.get('description', f"G{i+1}")
                
                # Sphere Marker
                m = Marker()
                m.header.frame_id = "map"
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "goals_pos"
                m.id = i * 2
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(pos[0])
                m.pose.position.y = float(pos[1])
                m.pose.position.z = float(pos[2])
                m.pose.orientation.w = 1.0
                m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 0.5
                m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0
                m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
                self.markers.markers.append(m)
                
                # Text Marker
                t = Marker()
                t.header.frame_id = "map"
                t.header.stamp = self.get_clock().now().to_msg()
                t.ns = "goals_text"
                t.id = i * 2 + 1
                t.type = Marker.TEXT_VIEW_FACING
                t.text = desc
                t.pose.position.x = float(pos[0])
                t.pose.position.y = float(pos[1])
                t.pose.position.z = float(pos[2]) + 0.6
                t.scale.z = 0.5
                t.color.r = 1.0; t.color.g = 1.0; t.color.b = 0.0; t.color.a = 1.0
                t.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
                self.markers.markers.append(t)
                
            self.get_logger().info(f"Loaded {len(goals)} goals.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load goals: {e}")

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        for m in self.markers.markers:
            m.header.stamp = now
        self.publisher_.publish(self.markers)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
