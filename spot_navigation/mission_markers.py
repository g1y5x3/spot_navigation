from __future__ import annotations

from collections.abc import Sequence
from copy import deepcopy

from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker, MarkerArray

MISSION_MARKER_TOPIC = "/mission_recorder/markers"

START_COLOR = "#22c55e"
WAYPOINT_COLOR = "#f97316"
ROUTE_COLOR = "#2563eb"


def _marker_color(value: str) -> tuple[float, float, float]:
    return (
        int(value[1:3], 16) / 255,
        int(value[3:5], 16) / 255,
        int(value[5:7], 16) / 255,
    )


_START_COLOR = _marker_color(START_COLOR)
_WAYPOINT_COLOR = _marker_color(WAYPOINT_COLOR)
_ROUTE_COLOR = _marker_color(ROUTE_COLOR)


def _marker(
    frame_id: str,
    stamp,
    namespace: str,
    marker_id: int,
    marker_type: int,
    color: tuple[float, float, float],
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 1.0
    return marker


def build_mission_markers(
    frame_id: str,
    stamp,
    initial_pose: Pose | None,
    waypoints: Sequence[Pose],
) -> MarkerArray:
    marker_array = MarkerArray()
    if initial_pose is None:
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        return marker_array

    if waypoints:
        route_marker = _marker(
            frame_id,
            stamp,
            "mission_route",
            0,
            Marker.LINE_STRIP,
            _ROUTE_COLOR,
        )
        route_marker.scale.x = 0.10
        for pose in (initial_pose, *waypoints):
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = pose.position.z + 0.15
            route_marker.points.append(point)
        marker_array.markers.append(route_marker)

    for index, pose in enumerate((initial_pose, *waypoints)):
        is_start = index == 0
        color = _START_COLOR if is_start else _WAYPOINT_COLOR

        pose_marker = _marker(
            frame_id,
            stamp,
            "mission_initial_pose" if is_start else "mission_waypoints",
            index,
            Marker.ARROW,
            color,
        )
        pose_marker.pose = deepcopy(pose)
        pose_marker.pose.position.z += 0.25
        pose_marker.scale.x = 1.0
        pose_marker.scale.y = 0.28
        pose_marker.scale.z = 0.20

        label_marker = _marker(
            frame_id,
            stamp,
            "mission_labels",
            index,
            Marker.TEXT_VIEW_FACING,
            color,
        )
        label_marker.pose.position = deepcopy(pose.position)
        label_marker.pose.position.z += 0.85
        label_marker.pose.orientation.w = 1.0
        label_marker.scale.z = 0.45
        label_marker.text = "START" if is_start else f"W{index}"

        marker_array.markers.extend((pose_marker, label_marker))
    return marker_array
