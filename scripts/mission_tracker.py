#!/usr/bin/env python3
"""
Mission Tracker Node
Tracks robot position relative to goals and entrance with timestamps.
Exports data to CSV for mission analysis.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import csv
import os
from datetime import datetime
import yaml
import math


class MissionTracker(Node):
    def __init__(self):
        super().__init__('mission_tracker')
        
        # Parameters
        self.declare_parameter('goals_file', '/home/yg5d6/spot_ws/src/spot_navigation/config/target_goals.yaml')
        self.declare_parameter('output_dir', '/home/yg5d6/spot_ws/src/spot_navigation/logs')
        self.declare_parameter('log_rate', 1.0)  # Hz
        self.declare_parameter('odom_topic', '/odometry_map')
        self.declare_parameter('goal_tolerance', 1.0)  # meters
        
        goals_file = self.get_parameter('goals_file').value
        self.output_dir = self.get_parameter('output_dir').value
        log_rate = self.get_parameter('log_rate').value
        odom_topic = self.get_parameter('odom_topic').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        # Create output directory if needed
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Load goals and entrance
        self.goals = []
        self.entrance = None
        self.load_locations(goals_file)
        
        # Subscribe to odometry
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        
        # Subscribe to goal updates (to track current goal)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.current_goal_idx = None
        
        # RViz marker publisher for mission status display
        self.marker_pub = self.create_publisher(Marker, '/mission_status_marker', 10)
        self.markers_enabled = True
        
        # Image overlay setup
        self.bridge = CvBridge()
        self.create_subscription(Image, '/thermal/image', self.image_callback, 10)
        self.annotated_image_pub = self.create_publisher(Image, '/thermal/image_annotated', 10)
        
        # Store latest distances for overlay
        self.latest_distances = {}
        self.latest_nearest_goal = ""
        
        # Initialize CSV file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.output_dir, f'mission_{timestamp}.csv')
        self.init_csv()
        
        # Timer for logging
        self.current_pose = None
        self.create_timer(1.0 / log_rate, self.log_callback)
        
        self.get_logger().info(f'Mission tracker started. Logging to {self.csv_path}')
        self.get_logger().info(f'Tracking {len(self.goals)} goals and entrance')
    
    def load_locations(self, goals_file):
        """Load goals from YAML and set entrance from config."""
        # Entrance coordinates (hardcoded from entrance.py)
        self.entrance = {
            'name': 'Entrance',
            'position': [6.22024, -6.27788, 0.0]
        }
        
        # Load goals from YAML
        try:
            with open(goals_file, 'r') as f:
                config = yaml.safe_load(f)
                for i, goal in enumerate(config.get('goals', [])):
                    self.goals.append({
                        'name': goal.get('description', f'Goal {i+1}'),
                        'position': goal['position']
                    })
        except Exception as e:
            self.get_logger().error(f'Failed to load goals: {e}')
            # Fallback to hardcoded goals
            self.goals = [
                {'name': 'Goal 1', 'position': [-2.496, 0.546, 0.002]},
                {'name': 'Goal 2', 'position': [-7.328, -9.266, -0.156]},
                {'name': 'Goal 3', 'position': [-20.756, -11.535, -0.320]},
            ]
    
    def init_csv(self):
        """Initialize CSV file with headers."""
        headers = ['timestamp', 'robot_x', 'robot_y', 'robot_z']
        
        # Add distance columns for each goal and entrance
        for goal in self.goals:
            headers.append(f'dist_{goal["name"].replace(" ", "_")}')
        headers.append('dist_Entrance')
        
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
        
        # Track which goals have been reached
        self.goals_reached = [False] * len(self.goals)
        self.reach_time = [None] * len(self.goals)
    
    def odom_callback(self, msg):
        """Store current pose from odometry."""
        self.current_pose = msg.pose.pose
    
    def goal_callback(self, msg):
        """Track which goal is currently active."""
        # Find closest goal to the published goal pose
        min_dist = float('inf')
        closest_idx = None
        
        for i, goal in enumerate(self.goals):
            dx = msg.pose.position.x - goal['position'][0]
            dy = msg.pose.position.y - goal['position'][1]
            dz = msg.pose.position.z - goal['position'][2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        if min_dist < self.goal_tolerance:
            self.current_goal_idx = closest_idx
    
    def log_callback(self):
        """Log current state to CSV."""
        if self.current_pose is None:
            return
        
        robot_pos = [
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z
        ]
        
        # Calculate distances to all goals and entrance
        distances = []
        
        for i, goal in enumerate(self.goals):
            dx = robot_pos[0] - goal['position'][0]
            dy = robot_pos[1] - goal['position'][1]
            dz = robot_pos[2] - goal['position'][2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            distances.append(dist)
            
            # Check if goal reached
            if dist < self.goal_tolerance and not self.goals_reached[i]:
                self.goals_reached[i] = True
                self.reach_time[i] = datetime.now().isoformat()
                self.get_logger().info(f"{goal['name']} reached!")
        
        # Distance to entrance
        dx = robot_pos[0] - self.entrance['position'][0]
        dy = robot_pos[1] - self.entrance['position'][1]
        dz = robot_pos[2] - self.entrance['position'][2]
        dist_entrance = math.sqrt(dx*dx + dy*dy + dz*dz)
        distances.append(dist_entrance)
        
        # Build CSV row
        row = [
            datetime.now().isoformat(),
            robot_pos[0],
            robot_pos[1],
            robot_pos[2],
        ]
        
        # Add distances
        row.extend(distances)
        
        # Add reached status
        row.extend([1 if r else 0 for r in self.goals_reached])
        
        # Store for image overlay
        self.latest_distances = {
            'entrance': dist_entrance,
            'goals': {self.goals[i]['name']: distances[i] for i in range(len(self.goals))},
            'reached': sum(self.goals_reached),
            'total': len(self.goals)
        }
        
        # Write to CSV
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
    
    def image_callback(self, msg):
        """Draw mission stats on thermal image."""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Prepare text lines - only distances to goals and entrance
            lines = []
            
            # Add distances to each goal (1, 2, 3)
            goal_distances = self.latest_distances.get('goals', {})
            for goal_name, dist in goal_distances.items():
                lines.append(f"{goal_name}: {dist:.1f}m")
            
            # Add entrance distance
            entrance_dist = self.latest_distances.get('entrance', 0)
            lines.append(f"Entrance: {entrance_dist:.1f}m")
            
            # Draw text on image (top-left corner)
            y_offset = 30
            for i, line in enumerate(lines):
                y = y_offset + i * 25
                # Draw black background for readability
                cv2.putText(cv_image, line, (10, y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                # Draw white text
                cv2.putText(cv_image, line, (10, y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            # Convert back to ROS Image and publish
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().warning(f'Failed to annotate image: {e}')
            # Publish original if annotation fails
            self.annotated_image_pub.publish(msg)
    
    def publish_markers(self, robot_pos, nearest_goal, nearest_dist, 
                       dist_entrance, distances):
        """Publish text markers for RViz display."""
        # Calculate x^2 + y^2 (distance squared from origin in XY plane)
        dist_squared_xy = robot_pos[0]**2 + robot_pos[1]**2
        
        # Count reached goals
        reached_count = sum(self.goals_reached)
        
        # Create main status marker (follows robot)
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mission_status'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Position above the robot
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 2.5  # 2.5m above robot
        marker.pose.orientation.w = 1.0
        
        # Text content with mission metrics
        lines = []
        lines.append(f'=== MISSION STATUS ===')
        lines.append(f'Goals: {reached_count}/{len(self.goals)} reached')
        
        if self.current_goal_idx is not None:
            current = self.goals[self.current_goal_idx]
            lines.append(f'Current: {current["name"]}')
            lines.append(f'  Distance: {distances[self.current_goal_idx]:.2f}m')
        else:
            lines.append(f'Nearest: {nearest_goal}')
            lines.append(f'  Distance: {nearest_dist:.2f}m')
        
        lines.append(f'Dist to Entrance: {dist_entrance:.2f}m')
        lines.append(f'x²+y²: {dist_squared_xy:.2f}m²')
        lines.append(f'Robot: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f})')
        
        marker.text = '\n'.join(lines)
        marker.scale.z = 0.3  # Text size
        
        # Color based on progress (green = all goals reached)
        if reached_count == len(self.goals):
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif reached_count > 0:
            marker.color.r = 1.0
            marker.color.g = 0.65
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)
        
        # Create markers for each goal showing distance
        for i, goal in enumerate(self.goals):
            if i == self.current_goal_idx:
                continue  # Skip current goal, shown in main marker
            
            goal_marker = Marker()
            goal_marker.header.frame_id = 'map'
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = 'goal_distances'
            goal_marker.id = i + 1
            goal_marker.type = Marker.TEXT_VIEW_FACING
            goal_marker.action = Marker.ADD
            
            # Position above the goal
            goal_marker.pose.position.x = goal['position'][0]
            goal_marker.pose.position.y = goal['position'][1]
            goal_marker.pose.position.z = goal['position'][2] + 1.0
            goal_marker.pose.orientation.w = 1.0
            
            dist = distances[i]
            status = '✓' if self.goals_reached[i] else '○'
            goal_marker.text = f'{status} {dist:.1f}m'
            goal_marker.scale.z = 0.25
            
            if self.goals_reached[i]:
                goal_marker.color.r = 0.0
                goal_marker.color.g = 1.0
                goal_marker.color.b = 0.0
            else:
                goal_marker.color.r = 1.0
                goal_marker.color.g = 1.0
                goal_marker.color.b = 1.0
            goal_marker.color.a = 0.8
            
            self.marker_pub.publish(goal_marker)


def main(args=None):
    rclpy.init(args=args)
    node = MissionTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Print summary
        node.get_logger().info('\n=== Mission Summary ===')
        for i, goal in enumerate(node.goals):
            status = 'REACHED' if node.goals_reached[i] else 'NOT REACHED'
            reach_time = f" at {node.reach_time[i]}" if node.reach_time[i] else ""
            node.get_logger().info(f"{goal['name']}: {status}{reach_time}")
        node.get_logger().info(f'\nData saved to: {node.csv_path}')
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
