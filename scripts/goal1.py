#!/usr/bin/env python3
"""
Publish Goal 1 pose to /goal_pose topic
Convenience script for repetitive missions
Goal 1: [-2.496, 0.546, 0.002]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal1_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Goal 1 coordinates
        msg.pose.position.x = -2.496135711669922
        msg.pose.position.y = 0.5463771820068359
        msg.pose.position.z = 0.0019300878047943115
        msg.pose.orientation.w = 1.0
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published Goal 1: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    node.publish_goal()
    # Give time for message to be sent
    import time
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
