#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class Route:
    name: str
    pre_loop: tuple[Waypoint, ...] = ()
    loop: tuple[Waypoint, ...] = ()
    loop_forever: bool = False


INITIAL_WAYPOINT = Waypoint("initial", 1.06585, -0.984649, 0.0, 2.71661)
MIDPOINT_WAYPOINT = Waypoint("midpoint", 15.5887, -3.37175, 0.0, -1.58109)
LOOP_WAYPOINTS = (
    Waypoint("loop_1", 4.38347, 0.94021, 0.0, 0.445908),
    Waypoint("loop_2", 12.1475, 2.7388, 0.0, -0.174207),
    Waypoint("loop_3", 15.4377, -2.95677, 0.0, -1.61901),
    Waypoint("loop_4", 8.70354, -5.94047, 0.0, 3.13688),
    Waypoint("loop_5", -0.323454, -4.71842, 0.0, 1.62359),
)

ROUTES = {
    "initial": Route("initial", pre_loop=(INITIAL_WAYPOINT,)),
    "midpoint": Route("midpoint", pre_loop=(MIDPOINT_WAYPOINT,)),
    "loop": Route("loop", loop=LOOP_WAYPOINTS, loop_forever=True),
    "initial_loop": Route(
        "initial_loop",
        pre_loop=(INITIAL_WAYPOINT,),
        loop=LOOP_WAYPOINTS,
        loop_forever=True,
    ),
}


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class RouteManager(Node):
    def __init__(self) -> None:
        super().__init__("route_manager")

        self.declare_parameter("route_name", "midpoint")
        route_name = str(self.get_parameter("route_name").value)
        self.route = self._select_route(route_name)

        self.declare_parameter("odom_topic", "/odometry_map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("marker_topic", "/goal_markers")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("reach_tolerance_xy", 0.75)
        self.declare_parameter("reach_tolerance_yaw", 0.75)
        self.declare_parameter("require_yaw_tolerance", False)
        self.declare_parameter("goal_republish_period", 2.0)
        self.declare_parameter("start_delay_sec", 2.0)
        self.declare_parameter("loop_forever", self.route.loop_forever)

        odom_topic = str(self.get_parameter("odom_topic").value)
        goal_topic = str(self.get_parameter("goal_topic").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.reach_tolerance_xy = float(self.get_parameter("reach_tolerance_xy").value)
        self.reach_tolerance_yaw = float(self.get_parameter("reach_tolerance_yaw").value)
        self.require_yaw_tolerance = bool(
            self.get_parameter("require_yaw_tolerance").value
        )
        self.goal_republish_period = float(
            self.get_parameter("goal_republish_period").value
        )
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.loop_forever = bool(self.get_parameter("loop_forever").value)

        self.pre_loop = list(self.route.pre_loop)
        self.loop_points = list(self.route.loop)
        if not self.pre_loop and not self.loop_points:
            raise RuntimeError(f"Route '{self.route.name}' has no waypoints")

        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 1)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)

        self.current_pose = None
        self.active_section = "pre_loop" if self.pre_loop else "loop"
        self.active_index = 0
        self.active_goal: Waypoint | None = None
        self.route_started = False
        self.last_publish_time = None
        self.start_time = self.get_clock().now()

        self.create_timer(0.2, self._tick)
        self.create_timer(1.0, self._publish_route_markers)

        self.get_logger().info(
            f"Route manager loaded '{self.route.name}' with "
            f"{len(self.pre_loop)} pre-loop points and "
            f"{len(self.loop_points)} loop points"
        )

    def _select_route(self, route_name: str) -> Route:
        if route_name not in ROUTES:
            valid_routes = ", ".join(sorted(ROUTES))
            raise RuntimeError(
                f"Unknown route_name '{route_name}'. Valid routes: {valid_routes}"
            )
        return ROUTES[route_name]

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
                f"Reached waypoint {self.active_goal.name} "
                f"in section {self.active_section}"
            )
            next_goal = self._advance_goal()
            if next_goal is None:
                self.get_logger().info("Route complete; no more waypoints to publish")
                self.active_goal = None
                self._publish_route_markers()
                return
            self.active_goal = next_goal
            self._publish_active_goal(force=True)
            self._publish_route_markers()
            return

        self._publish_active_goal(force=False)

    def _current_waypoint(self) -> Waypoint | None:
        if self.active_section == "pre_loop":
            if self.active_index < len(self.pre_loop):
                return self.pre_loop[self.active_index]
            self.active_section = "loop"
            self.active_index = 0

        if self.active_section == "loop" and self.active_index < len(self.loop_points):
            return self.loop_points[self.active_index]

        return None

    def _advance_goal(self) -> Waypoint | None:
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

    def _goal_reached(self, waypoint: Waypoint) -> bool:
        dx = self.current_pose.position.x - waypoint.x
        dy = self.current_pose.position.y - waypoint.y
        distance_xy = math.hypot(dx, dy)
        if distance_xy > self.reach_tolerance_xy:
            return False

        if not self.require_yaw_tolerance:
            return True

        yaw_robot = _yaw_from_quaternion(self.current_pose.orientation)
        yaw_error = abs(_normalize_angle(yaw_robot - waypoint.yaw))
        return yaw_error <= self.reach_tolerance_yaw

    def _publish_active_goal(self, force: bool) -> None:
        if self.active_goal is None:
            return

        now = self.get_clock().now()
        if not force and self.last_publish_time is not None:
            dt = (now - self.last_publish_time).nanoseconds / 1e9
            if dt < self.goal_republish_period:
                return

        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = now.to_msg()
        msg.pose.position.x = self.active_goal.x
        msg.pose.position.y = self.active_goal.y
        msg.pose.position.z = self.active_goal.z
        msg.pose.orientation.z = math.sin(self.active_goal.yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.active_goal.yaw / 2.0)

        self.goal_pub.publish(msg)
        self.last_publish_time = now
        self.get_logger().info(
            f"Published waypoint {self.active_goal.name}: "
            f"x={msg.pose.position.x:.3f}, "
            f"y={msg.pose.position.y:.3f}, "
            f"yaw={self.active_goal.yaw:.3f}"
        )

    def _all_waypoints(self) -> list[Waypoint]:
        return self.pre_loop + self.loop_points

    def _publish_route_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        route_points = self._all_waypoints()
        if route_points:
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = now
            line.ns = "route_manager"
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.06
            line.color.r = 0.1
            line.color.g = 0.4
            line.color.b = 1.0
            line.color.a = 0.8
            for waypoint in route_points:
                point = Point()
                point.x = waypoint.x
                point.y = waypoint.y
                point.z = waypoint.z + 0.2
                line.points.append(point)
            markers.markers.append(line)

        active_name = self.active_goal.name if self.active_goal is not None else ""
        for marker_index, waypoint in enumerate(route_points, start=1):
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = now
            sphere.ns = "route_manager_goals"
            sphere.id = marker_index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = waypoint.x
            sphere.pose.position.y = waypoint.y
            sphere.pose.position.z = waypoint.z + 0.25
            sphere.scale.x = 0.35
            sphere.scale.y = 0.35
            sphere.scale.z = 0.35
            if waypoint.name == active_name:
                sphere.color.r = 0.0
                sphere.color.g = 1.0
                sphere.color.b = 0.2
            else:
                sphere.color.r = 0.1
                sphere.color.g = 0.4
                sphere.color.b = 1.0
            sphere.color.a = 0.9
            markers.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = now
            text.ns = "route_manager_labels"
            text.id = marker_index + 1000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = waypoint.x
            text.pose.position.y = waypoint.y
            text.pose.position.z = waypoint.z + 0.75
            text.scale.z = 0.35
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 0.95
            text.text = waypoint.name
            markers.markers.append(text)

        self.marker_pub.publish(markers)


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
