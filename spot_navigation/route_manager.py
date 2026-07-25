#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import Any

import rclpy
import yaml
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool


@dataclass(frozen=True)
class Mission:
    frame_id: str
    initial_pose: Pose
    waypoints: tuple[Pose, ...]


class MissionEvent(Enum):
    IGNORED = auto()
    GOAL_ACCEPTED = auto()
    WAYPOINT_REACHED = auto()
    MISSION_COMPLETE = auto()


class MissionProgress:
    def __init__(self, waypoints: tuple[Pose, ...]) -> None:
        if not waypoints:
            raise ValueError("Mission must contain at least one waypoint")

        self._waypoints = waypoints
        self._active_index = 0
        self._goal_accepted = False

    @property
    def active_pose(self) -> Pose | None:
        if self.is_complete:
            return None
        return self._waypoints[self._active_index]

    @property
    def active_index(self) -> int:
        return self._active_index

    @property
    def is_complete(self) -> bool:
        return self._active_index >= len(self._waypoints)

    def reset_goal_acknowledgement(self) -> None:
        self._goal_accepted = False

    def handle_far_status(self, reached: bool) -> MissionEvent:
        if self.is_complete:
            return MissionEvent.IGNORED

        if not reached:
            if self._goal_accepted:
                return MissionEvent.IGNORED
            self._goal_accepted = True
            return MissionEvent.GOAL_ACCEPTED

        if not self._goal_accepted:
            return MissionEvent.IGNORED

        self._active_index += 1
        self._goal_accepted = False
        if self.is_complete:
            return MissionEvent.MISSION_COMPLETE
        return MissionEvent.WAYPOINT_REACHED


def _require_mapping(
    value: Any,
    field_name: str,
    numeric_keys: tuple[str, ...] = (),
) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"'{field_name}' must be a mapping")

    result = dict(value)
    for key in numeric_keys:
        numeric_value = result.get(key)
        if isinstance(numeric_value, bool) or not isinstance(
            numeric_value, (int, float)
        ):
            raise ValueError(f"'{field_name}.{key}' must be numeric")

        numeric_value = float(numeric_value)
        if not math.isfinite(numeric_value):
            raise ValueError(f"'{field_name}.{key}' must be finite")
        result[key] = numeric_value
    return result


def _load_pose(value: Any, field_name: str) -> Pose:
    mapping = _require_mapping(value, field_name)
    position = _require_mapping(
        mapping.get("position"),
        f"{field_name}.position",
        ("x", "y", "z"),
    )
    orientation = _require_mapping(
        mapping.get("orientation"),
        f"{field_name}.orientation",
        ("x", "y", "z", "w"),
    )

    qx = orientation["x"]
    qy = orientation["y"]
    qz = orientation["z"]
    qw = orientation["w"]
    quaternion_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if quaternion_norm < 1.0e-9:
        raise ValueError(f"'{field_name}.orientation' quaternion cannot be zero")

    pose = Pose()
    pose.position.x = position["x"]
    pose.position.y = position["y"]
    pose.position.z = position["z"]
    pose.orientation.x = qx / quaternion_norm
    pose.orientation.y = qy / quaternion_norm
    pose.orientation.z = qz / quaternion_norm
    pose.orientation.w = qw / quaternion_norm
    return pose


def load_mission(path: str | Path) -> Mission:
    mission_path = Path(path).expanduser()
    if not mission_path.is_file():
        raise ValueError(f"Mission file does not exist: {mission_path}")

    try:
        raw_document = yaml.safe_load(mission_path.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid mission YAML '{mission_path}': {error}") from error

    document = _require_mapping(raw_document, "mission")
    frame_id = document.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("'frame_id' must be a non-empty string")

    initial_pose = _load_pose(document.get("initial_pose"), "initial_pose")

    raw_waypoints = document.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("'waypoints' must be a non-empty list")

    waypoints: list[Pose] = []
    for index, raw_waypoint in enumerate(raw_waypoints):
        field_name = f"waypoints[{index}]"
        waypoints.append(_load_pose(raw_waypoint, field_name))

    return Mission(
        frame_id=frame_id,
        initial_pose=initial_pose,
        waypoints=tuple(waypoints),
    )


class RouteManagerState(Enum):
    WAITING_FOR_ODOMETRY = auto()
    WAITING_FOR_INITIAL_POSE = auto()
    WAITING_FOR_FAR = auto()
    WAITING_FOR_GOAL_ACCEPTANCE = auto()
    NAVIGATING = auto()
    COMPLETE = auto()
    FAILED = auto()


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class RouteManager(Node):
    def __init__(self) -> None:
        super().__init__("route_manager")

        self.declare_parameter("mission_file", "")
        self.declare_parameter("odom_topic", "/odometry_map")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("far_status_topic", "/far_reach_goal_status")
        self.declare_parameter("publish_initial_pose", True)
        self.declare_parameter("initial_pose_tolerance_xy", 0.75)
        self.declare_parameter("initial_pose_tolerance_yaw", 0.35)
        self.declare_parameter("initial_pose_retry_period", 2.0)
        self.declare_parameter("far_ready_delay", 6.0)
        self.declare_parameter("goal_retry_period", 2.0)
        self.declare_parameter("status_silence_timeout", 5.0)
        self.declare_parameter("max_goal_retries", 3)

        mission_file = str(self.get_parameter("mission_file").value)
        if not mission_file:
            raise RuntimeError("Parameter 'mission_file' must name a mission YAML file")

        try:
            self.mission: Mission = load_mission(mission_file)
        except ValueError as error:
            raise RuntimeError(str(error)) from error

        odom_topic = str(self.get_parameter("odom_topic").value)
        initial_pose_topic = str(self.get_parameter("initial_pose_topic").value)
        goal_topic = str(self.get_parameter("goal_topic").value)
        far_status_topic = str(self.get_parameter("far_status_topic").value)
        self.far_status_topic = far_status_topic
        self.publish_initial_pose = bool(
            self.get_parameter("publish_initial_pose").value
        )
        self.initial_pose_tolerance_xy = float(
            self.get_parameter("initial_pose_tolerance_xy").value
        )
        self.initial_pose_tolerance_yaw = float(
            self.get_parameter("initial_pose_tolerance_yaw").value
        )
        self.initial_pose_retry_period = float(
            self.get_parameter("initial_pose_retry_period").value
        )
        self.far_ready_delay = float(
            self.get_parameter("far_ready_delay").value
        )
        self.goal_retry_period = float(
            self.get_parameter("goal_retry_period").value
        )
        self.status_silence_timeout = float(
            self.get_parameter("status_silence_timeout").value
        )
        self.max_goal_retries = int(
            self.get_parameter("max_goal_retries").value
        )
        self._validate_parameters()

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, initial_pose_topic, 1
        )
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
        self.create_subscription(Bool, far_status_topic, self._far_status_callback, 10)

        self.progress = MissionProgress(self.mission.waypoints)
        self.state = RouteManagerState.WAITING_FOR_ODOMETRY
        self.current_pose: Pose | None = None
        self.odom_count = 0
        self.initial_pose_publish_odom_count = 0
        self.last_initial_pose_publish_time: Time | None = None
        self.far_available_since: Time | None = None
        self.last_goal_publish_time: Time | None = None
        self.last_far_status_time: Time | None = None
        self.goal_retry_count = 0

        self.create_timer(0.2, self._tick)

        self.get_logger().info(
            f"Loaded mission '{mission_file}' with "
            f"{len(self.mission.waypoints)} waypoints"
        )
        self.get_logger().info("Waiting for localization odometry")

    def _validate_parameters(self) -> None:
        positive_parameters = {
            "initial_pose_tolerance_xy": self.initial_pose_tolerance_xy,
            "initial_pose_tolerance_yaw": self.initial_pose_tolerance_yaw,
            "initial_pose_retry_period": self.initial_pose_retry_period,
            "goal_retry_period": self.goal_retry_period,
            "status_silence_timeout": self.status_silence_timeout,
        }
        for name, value in positive_parameters.items():
            if value <= 0.0:
                raise RuntimeError(f"Parameter '{name}' must be greater than zero")
        if self.far_ready_delay < 0.0:
            raise RuntimeError("Parameter 'far_ready_delay' cannot be negative")
        if self.max_goal_retries < 0:
            raise RuntimeError("Parameter 'max_goal_retries' cannot be negative")

    def _odom_callback(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose
        self.odom_count += 1

    def _far_status_callback(self, msg: Bool) -> None:
        if self.state not in (
            RouteManagerState.WAITING_FOR_GOAL_ACCEPTANCE,
            RouteManagerState.NAVIGATING,
        ):
            return

        if self.progress.is_complete:
            return
        waypoint_index = self.progress.active_index

        event = self.progress.handle_far_status(bool(msg.data))
        now = self.get_clock().now()

        if not msg.data:
            self.last_far_status_time = now

        if event is MissionEvent.GOAL_ACCEPTED:
            self.state = RouteManagerState.NAVIGATING
            self.get_logger().info(
                f"FAR accepted waypoint {waypoint_index}; waiting for success"
            )
            return

        if event is MissionEvent.WAYPOINT_REACHED:
            self.get_logger().info(f"FAR reached waypoint {waypoint_index}")
            self.goal_retry_count = 0
            self.last_far_status_time = None
            self.state = RouteManagerState.WAITING_FOR_FAR
            return

        if event is MissionEvent.MISSION_COMPLETE:
            self.get_logger().info(
                f"FAR reached waypoint {waypoint_index}; mission complete"
            )
            self.state = RouteManagerState.COMPLETE
            return

        if msg.data and event is MissionEvent.IGNORED:
            self.get_logger().warning(
                "Ignored FAR success received before current goal acknowledgement"
            )

    def _tick(self) -> None:
        if self.state in (RouteManagerState.COMPLETE, RouteManagerState.FAILED):
            return

        if self.state is RouteManagerState.WAITING_FOR_ODOMETRY:
            self._tick_waiting_for_odometry()
            return

        if self.state is RouteManagerState.WAITING_FOR_INITIAL_POSE:
            self._tick_waiting_for_initial_pose()
            return

        if self.state is RouteManagerState.WAITING_FOR_FAR:
            self._tick_waiting_for_far()
            return

        if self.state is RouteManagerState.WAITING_FOR_GOAL_ACCEPTANCE:
            self._tick_waiting_for_goal_acceptance()
            return

        if self.state is RouteManagerState.NAVIGATING:
            self._tick_navigating()

    def _tick_waiting_for_odometry(self) -> None:
        if self.current_pose is None:
            return
        if not self.publish_initial_pose:
            self.get_logger().warning(
                "Initial-pose publication disabled; using existing localization"
            )
            self.state = RouteManagerState.WAITING_FOR_FAR
            return
        if self.initial_pose_pub.get_subscription_count() == 0:
            return

        self._publish_initial_pose()
        self.state = RouteManagerState.WAITING_FOR_INITIAL_POSE

    def _tick_waiting_for_initial_pose(self) -> None:
        if (
            self.current_pose is not None
            and self.odom_count > self.initial_pose_publish_odom_count
            and self._initial_pose_matches(self.current_pose)
        ):
            self.get_logger().info("Initial pose applied by localization")
            self.state = RouteManagerState.WAITING_FOR_FAR
            return

        if self.initial_pose_pub.get_subscription_count() == 0:
            return
        if self._elapsed_since(self.last_initial_pose_publish_time) < (
            self.initial_pose_retry_period
        ):
            return

        self.get_logger().warning(
            "Initial pose not yet reflected in /odometry_map; publishing it again"
        )
        self._publish_initial_pose()

    def _tick_waiting_for_far(self) -> None:
        if not self._far_available():
            self.far_available_since = None
            return
        if self.far_available_since is None:
            self.far_available_since = self.get_clock().now()
            return
        if self._elapsed_since(self.far_available_since) < self.far_ready_delay:
            return
        if self.progress.active_pose is None:
            self.state = RouteManagerState.COMPLETE
            return

        self._publish_active_goal()
        self.state = RouteManagerState.WAITING_FOR_GOAL_ACCEPTANCE

    def _tick_waiting_for_goal_acceptance(self) -> None:
        if not self._far_available():
            self.far_available_since = None
            self.state = RouteManagerState.WAITING_FOR_FAR
            return
        if self._elapsed_since(self.last_goal_publish_time) < self.goal_retry_period:
            return

        self.get_logger().warning(
            "FAR has not acknowledged current goal; publishing it again"
        )
        self._publish_active_goal()

    def _tick_navigating(self) -> None:
        if not self._far_available():
            self.progress.reset_goal_acknowledgement()
            self.far_available_since = None
            self.state = RouteManagerState.WAITING_FOR_FAR
            return
        if (
            self._elapsed_since(self.last_far_status_time)
            < self.status_silence_timeout
        ):
            return

        self.goal_retry_count += 1
        if self.goal_retry_count > self.max_goal_retries:
            self.get_logger().error(
                f"FAR status became silent at waypoint "
                f"{self.progress.active_index}; "
                f"mission failed after {self.max_goal_retries} retries"
            )
            self.state = RouteManagerState.FAILED
            return

        self.get_logger().warning(
            "FAR status became silent; retrying current goal "
            f"({self.goal_retry_count}/{self.max_goal_retries})"
        )
        self.progress.reset_goal_acknowledgement()
        self.last_far_status_time = None
        self._publish_active_goal()
        self.state = RouteManagerState.WAITING_FOR_GOAL_ACCEPTANCE

    def _far_available(self) -> bool:
        return (
            self.goal_pub.get_subscription_count() > 0
            and self.count_publishers(self.far_status_topic) > 0
        )

    def _elapsed_since(self, timestamp: Time | None) -> float:
        if timestamp is None:
            return math.inf
        return (self.get_clock().now() - timestamp).nanoseconds / 1.0e9

    def _initial_pose_matches(self, current_pose: Pose) -> bool:
        initial_pose = self.mission.initial_pose
        distance_xy = math.hypot(
            current_pose.position.x - initial_pose.position.x,
            current_pose.position.y - initial_pose.position.y,
        )
        if distance_xy > self.initial_pose_tolerance_xy:
            return False

        current_yaw = _yaw_from_quaternion(
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        )
        initial_yaw = _yaw_from_quaternion(
            initial_pose.orientation.x,
            initial_pose.orientation.y,
            initial_pose.orientation.z,
            initial_pose.orientation.w,
        )
        yaw_error = abs(_normalize_angle(current_yaw - initial_yaw))
        return yaw_error <= self.initial_pose_tolerance_yaw

    def _publish_initial_pose(self) -> None:
        now = self.get_clock().now()
        initial_pose = self.mission.initial_pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.mission.frame_id
        msg.header.stamp = now.to_msg()
        msg.pose.pose = initial_pose
        self.initial_pose_pub.publish(msg)
        self.initial_pose_publish_odom_count = self.odom_count
        self.last_initial_pose_publish_time = now
        self.get_logger().info(
            "Published initial pose: "
            f"x={initial_pose.position.x:.3f}, "
            f"y={initial_pose.position.y:.3f}"
        )

    def _publish_active_goal(self) -> None:
        active_pose = self.progress.active_pose
        if active_pose is None:
            return
        waypoint_index = self.progress.active_index

        now = self.get_clock().now()
        msg = PoseStamped()
        msg.header.frame_id = self.mission.frame_id
        msg.header.stamp = now.to_msg()
        msg.pose = active_pose

        self.goal_pub.publish(msg)
        self.last_goal_publish_time = now
        self.get_logger().info(
            f"Published waypoint {waypoint_index}: "
            f"x={active_pose.position.x:.3f}, "
            f"y={active_pose.position.y:.3f}"
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
