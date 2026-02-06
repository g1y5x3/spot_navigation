#!/usr/bin/env python3
"""
Publish Entrance pose to /goal_pose topic
Convenience script for repetitive missions
Entrance: [6.22024, -6.27788, 0.0]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('entrance_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Entrance coordinates
        msg.pose.position.x = 6.22024
        msg.pose.position.y = -6.27788
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published Entrance: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}')

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
