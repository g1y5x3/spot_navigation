#!/usr/bin/env python3

import math
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class RouteManager(Node):
    def __init__(self) -> None:
        super().__init__("route_manager")

        default_route = (
            get_package_share_directory("spot_navigation") + "/config/midpoint_route.yaml"
        )

        self.declare_parameter("route_file", default_route)
        self.declare_parameter("odom_topic", "/odometry_map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("reach_tolerance_xy", 0.75)
        self.declare_parameter("reach_tolerance_yaw", 0.75)
        self.declare_parameter("goal_republish_period", 2.0)
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("loop_forever", True)

        route_file = self.get_parameter("route_file").value
        odom_topic = self.get_parameter("odom_topic").value
        goal_topic = self.get_parameter("goal_topic").value
        self.reach_tolerance_xy = float(
            self.get_parameter("reach_tolerance_xy").value
        )
        self.reach_tolerance_yaw = float(
            self.get_parameter("reach_tolerance_yaw").value
        )
        self.goal_republish_period = float(
            self.get_parameter("goal_republish_period").value
        )
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.loop_forever = bool(self.get_parameter("loop_forever").value)

        self.pre_loop = []
        self.loop_points = []
        self._load_route(route_file)

        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)

        self.current_pose = None
        self.active_section = "pre_loop" if self.pre_loop else "loop"
        self.active_index = 0
        self.active_goal = None
        self.route_started = False
        self.last_publish_time = None
        self.start_time = self.get_clock().now()

        self.create_timer(0.2, self._tick)

        self.get_logger().info(
            f"Loaded route with {len(self.pre_loop)} pre-loop points and "
            f"{len(self.loop_points)} loop points from {route_file}"
        )

    def _load_route(self, route_file: str) -> None:
        import yaml

        with open(route_file, "r", encoding="utf-8") as handle:
            config = yaml.safe_load(handle) or {}

        route = config.get("route", {})
        self.pre_loop = route.get("pre_loop", [])
        self.loop_points = route.get("loop", [])

        if not self.pre_loop and not self.loop_points:
            raise RuntimeError(f"No waypoints found in route file: {route_file}")

    def _odom_callback(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose

    def _tick(self) -> None:
        if self.current_pose is None:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if not self.route_started and elapsed < self.start_delay_sec:
            return
        self.route_started = True

        if self.active_goal is None:
            self.active_goal = self._current_waypoint()
            if self.active_goal is None:
                return
            self._publish_active_goal(force=True)
            return

        if self._goal_reached(self.active_goal):
            self.get_logger().info(
                f"Reached waypoint {self.active_goal['name']} in section {self.active_section}"
            )
            next_goal = self._advance_goal()
            if next_goal is None:
                self.get_logger().info("Route complete; no more waypoints to publish")
                self.active_goal = None
                return
            self.active_goal = next_goal
            self._publish_active_goal(force=True)
            return

        self._publish_active_goal(force=False)

    def _current_waypoint(self) -> dict[str, Any] | None:
        if self.active_section == "pre_loop":
            if self.active_index < len(self.pre_loop):
                return self.pre_loop[self.active_index]
            self.active_section = "loop"
            self.active_index = 0

        if self.active_section == "loop" and self.active_index < len(self.loop_points):
            return self.loop_points[self.active_index]

        return None

    def _advance_goal(self) -> dict[str, Any] | None:
        self.active_index += 1

        if self.active_section == "pre_loop":
            if self.active_index < len(self.pre_loop):
                return self.pre_loop[self.active_index]
            self.active_section = "loop"
            self.active_index = 0
            if self.loop_points:
                return self.loop_points[self.active_index]
            return None

        if not self.loop_points:
            return None

        if self.active_index >= len(self.loop_points):
            if not self.loop_forever:
                return None
            self.active_index = 0

        return self.loop_points[self.active_index]

    def _goal_reached(self, waypoint: dict[str, Any]) -> bool:
        position = waypoint["position"]
        dx = self.current_pose.position.x - float(position["x"])
        dy = self.current_pose.position.y - float(position["y"])
        distance_xy = math.hypot(dx, dy)

        orientation = waypoint.get("orientation")
        if not orientation:
            return distance_xy <= self.reach_tolerance_xy

        yaw_goal = float(orientation["yaw"])
        q = self.current_pose.orientation
        yaw_robot = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        yaw_error = abs(_normalize_angle(yaw_robot - yaw_goal))
        return (
            distance_xy <= self.reach_tolerance_xy
            and yaw_error <= self.reach_tolerance_yaw
        )

    def _publish_active_goal(self, force: bool) -> None:
        now = self.get_clock().now()
        if not force and self.last_publish_time is not None:
            dt = (now - self.last_publish_time).nanoseconds / 1e9
            if dt < self.goal_republish_period:
                return

        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = now.to_msg()
        msg.pose.position.x = float(self.active_goal["position"]["x"])
        msg.pose.position.y = float(self.active_goal["position"]["y"])
        msg.pose.position.z = float(self.active_goal["position"].get("z", 0.0))

        yaw = float(self.active_goal.get("orientation", {}).get("yaw", 0.0))
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(msg)
        self.last_publish_time = now
        self.get_logger().info(
            f"Published waypoint {self.active_goal['name']}: "
            f"x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, yaw={yaw:.3f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
