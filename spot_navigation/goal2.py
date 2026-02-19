#!/usr/bin/env python3
"""
Publish Goal 2 pose to /goal_pose topic
Convenience script for repetitive missions
Goal 2: [-7.328, -9.266, -0.156]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal2_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Goal 2 coordinates
        msg.pose.position.x = -7.328216552734375
        msg.pose.position.y = -9.265711784362793
        msg.pose.position.z = -0.15630733966827393
        msg.pose.orientation.w = 1.0
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published Goal 2: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}')

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
