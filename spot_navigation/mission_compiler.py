#!/usr/bin/env python3

"""Compile manually recorded missions against a FAR prior visibility graph."""

from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import tempfile
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
import yaml
from pypcd4 import PointCloud


EPSILON = 1.0e-7


class FreeDirection(IntEnum):
    UNKNOWN = 0
    CONVEX = 1
    CONCAVE = 2
    PILLAR = 3


@dataclass(frozen=True)
class MissionPose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float

    @property
    def xy(self) -> np.ndarray:
        return np.asarray((self.x, self.y), dtype=np.float64)

    def as_mapping(self) -> dict[str, dict[str, float]]:
        return {
            "position": {"x": self.x, "y": self.y, "z": self.z},
            "orientation": {
                "x": self.qx,
                "y": self.qy,
                "z": self.qz,
                "w": self.qw,
            },
        }


@dataclass(frozen=True)
class MissionDocument:
    frame_id: str
    initial_pose: MissionPose
    waypoints: tuple[MissionPose, ...]


@dataclass(frozen=True)
class VghNode:
    node_id: int
    free_direction: FreeDirection
    position: np.ndarray
    surface_directions: tuple[np.ndarray, np.ndarray]
    is_covered: bool
    is_frontier: bool
    is_navpoint: bool
    is_boundary: bool
    connections: tuple[int, ...]
    polygon_connections: tuple[int, ...]
    contour_connections: tuple[int, ...]
    trajectory_connections: tuple[int, ...]


@dataclass(frozen=True)
class FarParameters:
    voxel_dim: float
    robot_dim: float
    vehicle_height: float
    sensor_range: float
    local_planner_range: float
    angle_noise: float
    converge_distance: float
    floor_height: float
    is_multi_layer: bool

    @property
    def nav_clear_distance(self) -> float:
        return self.robot_dim / 2.0 + self.voxel_dim

    @property
    def height_tolerance(self) -> float:
        return self.floor_height - self.voxel_dim * 2.0


@dataclass
class CandidateResult:
    accepted: list[int]
    rejected: dict[str, list[int]]

    def as_report(self) -> dict[str, object]:
        return {
            "accepted_count": len(self.accepted),
            "accepted_node_ids": self.accepted,
            "rejected_counts": {
                reason: len(node_ids)
                for reason, node_ids in sorted(self.rejected.items())
            },
            "rejected_node_ids": {
                reason: node_ids
                for reason, node_ids in sorted(self.rejected.items())
            },
        }


@dataclass
class LegPlan:
    route: list[np.ndarray]
    graph_path: list[int]
    mode: str
    start_candidates: CandidateResult
    goal_candidates: CandidateResult
    selected_goal_node: int | None


class CompilationError(RuntimeError):
    def __init__(self, message: str, report: dict[str, object]) -> None:
        super().__init__(message)
        self.report = report


class BoundarySegments:
    def __init__(
        self,
        segments: Iterable[tuple[np.ndarray, np.ndarray]],
    ) -> None:
        segment_list = list(segments)
        if segment_list:
            self.starts = np.asarray(
                [segment[0][:2] for segment in segment_list], dtype=np.float64
            )
            self.ends = np.asarray(
                [segment[1][:2] for segment in segment_list], dtype=np.float64
            )
        else:
            self.starts = np.empty((0, 2), dtype=np.float64)
            self.ends = np.empty((0, 2), dtype=np.float64)

    def intersects(self, start: np.ndarray, end: np.ndarray) -> bool:
        if len(self.starts) == 0:
            return False

        start = np.asarray(start[:2], dtype=np.float64)
        end = np.asarray(end[:2], dtype=np.float64)
        minimum = np.minimum(start, end)
        maximum = np.maximum(start, end)
        candidates = (
            (np.maximum(self.starts[:, 0], self.ends[:, 0]) >= minimum[0])
            & (np.minimum(self.starts[:, 0], self.ends[:, 0]) <= maximum[0])
            & (np.maximum(self.starts[:, 1], self.ends[:, 1]) >= minimum[1])
            & (np.minimum(self.starts[:, 1], self.ends[:, 1]) <= maximum[1])
        )
        if not np.any(candidates):
            return False

        first = self.starts[candidates]
        second = self.ends[candidates]
        first_side = _cross(end - start, first - start)
        second_side = _cross(end - start, second - start)
        third_side = _cross(second - first, start - first)
        fourth_side = _cross(second - first, end - first)

        proper = (
            ((first_side > EPSILON) & (second_side < -EPSILON))
            | ((first_side < -EPSILON) & (second_side > EPSILON))
        ) & (
            ((third_side > EPSILON) & (fourth_side < -EPSILON))
            | ((third_side < -EPSILON) & (fourth_side > EPSILON))
        )
        if np.any(proper):
            return True

        return bool(
            np.any(
                (np.abs(first_side) <= EPSILON)
                & _points_on_segment(first, start, end)
            )
            or np.any(
                (np.abs(second_side) <= EPSILON)
                & _points_on_segment(second, start, end)
            )
            or np.any(
                (np.abs(third_side) <= EPSILON)
                & _point_on_segments(start, first, second)
            )
            or np.any(
                (np.abs(fourth_side) <= EPSILON)
                & _point_on_segments(end, first, second)
            )
        )


class PriorGraph:
    def __init__(
        self,
        nodes: dict[int, VghNode],
        parameters: FarParameters,
    ) -> None:
        if not nodes:
            raise ValueError("VGH graph is empty")
        self.nodes = nodes
        self.parameters = parameters
        self.boundaries = BoundarySegments(self._boundary_segments())

    def _boundary_segments(self) -> Iterable[tuple[np.ndarray, np.ndarray]]:
        seen: set[tuple[int, int]] = set()
        for node in self.nodes.values():
            if not node.is_boundary:
                continue
            for neighbor_id in node.contour_connections:
                neighbor = self.nodes[neighbor_id]
                if not neighbor.is_boundary:
                    continue
                edge = tuple(sorted((node.node_id, neighbor_id)))
                if edge in seen:
                    continue
                seen.add(edge)
                if (
                    np.linalg.norm(node.position[:2] - neighbor.position[:2])
                    < self.parameters.voxel_dim
                ):
                    continue
                yield node.position, neighbor.position

    def plan_leg(
        self,
        start: MissionPose,
        goal: MissionPose,
    ) -> LegPlan | None:
        start_position = self._synthetic_position(start)
        goal_position = self._synthetic_position(goal)
        start_candidates = self._start_candidates(start_position)
        distances, parents = self._distances_from(
            start_position,
            start_candidates,
        )
        goal_candidates = self._goal_candidates(goal_position, distances)

        direct_clear = self._direct_connection_clear(
            start_position,
            goal_position,
        )
        direct_cost = (
            float(np.linalg.norm(goal_position[:2] - start_position[:2]))
            if direct_clear
            else math.inf
        )

        selected_goal_node: int | None = None
        selected_cost = direct_cost
        for node_id in goal_candidates.accepted:
            node = self.nodes[node_id]
            cost = distances[node_id] + float(
                np.linalg.norm(goal_position[:2] - node.position[:2])
            )
            if cost < selected_cost:
                selected_cost = cost
                selected_goal_node = node_id

        mode = "direct"
        if selected_goal_node is None and not math.isfinite(selected_cost):
            bridge = self._find_approach_bridge(goal_position, distances)
            if bridge is None:
                return None
            selected_goal_node, selected_cost = bridge
            mode = "approach_bridge"
        elif selected_goal_node is not None:
            mode = "graph"

        if selected_goal_node is None:
            graph_path: list[int] = []
            route = [start_position, goal_position]
        else:
            graph_path = _reconstruct_path(selected_goal_node, parents)
            route = [start_position]
            route.extend(
                self._anchor_position(self.nodes[node_id])
                for node_id in graph_path
            )
            route.append(goal_position)

        route = _deduplicate_points(route)
        route = self._shortcut_route(route)
        return LegPlan(
            route=route,
            graph_path=graph_path,
            mode=mode,
            start_candidates=start_candidates,
            goal_candidates=goal_candidates,
            selected_goal_node=selected_goal_node,
        )

    def _synthetic_position(self, pose: MissionPose) -> np.ndarray:
        xy = pose.xy
        nearest = min(
            self.nodes.values(),
            key=lambda node: float(np.linalg.norm(node.position[:2] - xy)),
        )
        return np.asarray(
            (pose.x, pose.y, float(nearest.position[2])),
            dtype=np.float64,
        )

    def _start_candidates(self, start: np.ndarray) -> CandidateResult:
        result = CandidateResult([], {})
        for node_id, node in self.nodes.items():
            difference = start - node.position
            distance = float(np.linalg.norm(difference))
            if distance >= self.parameters.sensor_range:
                _reject(result, "outside_sensor_range", node_id)
            elif (
                node.is_navpoint
                and distance < self.parameters.nav_clear_distance
            ):
                result.accepted.append(node_id)
            elif node.free_direction == FreeDirection.CONCAVE:
                _reject(result, "concave", node_id)
            elif not _is_outside_reduced_directions(
                difference,
                node.surface_directions,
                self.parameters,
            ):
                _reject(result, "direction", node_id)
            elif not self._node_endpoint_connection_clear(node, start):
                _reject(result, "boundary_collision", node_id)
            else:
                result.accepted.append(node_id)
        return result

    def _distances_from(
        self,
        start: np.ndarray,
        candidates: CandidateResult,
    ) -> tuple[dict[int, float], dict[int, int | None]]:
        distances = {node_id: math.inf for node_id in self.nodes}
        parents: dict[int, int | None] = {}
        queue: list[tuple[float, int]] = []
        for node_id in candidates.accepted:
            distance = float(
                np.linalg.norm(self.nodes[node_id].position - start)
            )
            if distance < distances[node_id]:
                distances[node_id] = distance
                parents[node_id] = None
                heapq.heappush(queue, (distance, node_id))

        while queue:
            distance, node_id = heapq.heappop(queue)
            if distance > distances[node_id]:
                continue
            node = self.nodes[node_id]
            for neighbor_id in node.connections:
                neighbor = self.nodes[neighbor_id]
                candidate_distance = distance + float(
                    np.linalg.norm(neighbor.position - node.position)
                )
                if candidate_distance < distances[neighbor_id]:
                    distances[neighbor_id] = candidate_distance
                    parents[neighbor_id] = node_id
                    heapq.heappush(queue, (candidate_distance, neighbor_id))
        return distances, parents

    def _goal_candidates(
        self,
        goal: np.ndarray,
        distances: dict[int, float],
    ) -> CandidateResult:
        result = CandidateResult([], {})
        for node_id, node in self.nodes.items():
            if not math.isfinite(distances[node_id]):
                _reject(result, "unreachable", node_id)
                continue
            difference = goal - node.position
            distance = float(np.linalg.norm(difference))
            if node.is_navpoint and distance < self.parameters.robot_dim:
                result.accepted.append(node_id)
            elif node.free_direction == FreeDirection.CONCAVE:
                _reject(result, "concave", node_id)
            elif not _is_outside_reduced_directions(
                difference,
                node.surface_directions,
                self.parameters,
            ):
                _reject(result, "direction", node_id)
            elif not self._node_endpoint_connection_clear(node, goal):
                _reject(result, "boundary_collision", node_id)
            else:
                result.accepted.append(node_id)
        return result

    def _find_approach_bridge(
        self,
        goal: np.ndarray,
        distances: dict[int, float],
    ) -> tuple[int, float] | None:
        best: tuple[int, float] | None = None
        for node_id, distance in distances.items():
            if not math.isfinite(distance):
                continue
            node = self.nodes[node_id]
            anchor = self._anchor_position(node)
            if not self._direct_connection_clear(anchor, goal):
                continue
            cost = distance + float(np.linalg.norm(anchor[:2] - goal[:2]))
            if best is None or cost < best[1]:
                best = (node_id, cost)
        return best

    def _node_endpoint_connection_clear(
        self,
        node: VghNode,
        endpoint: np.ndarray,
    ) -> bool:
        distance = float(np.linalg.norm(endpoint[:2] - node.position[:2]))
        if distance < self.parameters.nav_clear_distance:
            return True
        projection_distance = min(
            distance * 0.4,
            self.parameters.voxel_dim,
        )
        projected_node = node.position[:2] + (
            self._node_project_direction(node) * projection_distance
        )
        return not self.boundaries.intersects(projected_node, endpoint[:2])

    def _direct_connection_clear(
        self,
        start: np.ndarray,
        goal: np.ndarray,
    ) -> bool:
        if np.linalg.norm(goal[:2] - start[:2]) < EPSILON:
            return True
        return not self.boundaries.intersects(start[:2], goal[:2])

    def _node_project_direction(self, node: VghNode) -> np.ndarray:
        if node.free_direction in (
            FreeDirection.UNKNOWN,
            FreeDirection.PILLAR,
        ):
            return np.zeros(2, dtype=np.float64)
        topology = _normalize_xy(
            node.surface_directions[0][:2] + node.surface_directions[1][:2]
        )
        if node.free_direction == FreeDirection.CONCAVE:
            return topology
        return -topology

    def _anchor_position(self, node: VghNode) -> np.ndarray:
        anchor = np.asarray(node.position, dtype=np.float64).copy()
        # Prior-map visibility edges are serialized from a half-voxel corner
        # projection. Keep generated goals on that same free-space side.
        anchor[:2] += (
            self._node_project_direction(node)
            * self.parameters.voxel_dim
            * 0.5
        )
        return anchor

    def _shortcut_route(self, route: list[np.ndarray]) -> list[np.ndarray]:
        if len(route) <= 2:
            return route
        result = [route[0]]
        current = 0
        while current < len(route) - 1:
            selected = current + 1
            for candidate in range(len(route) - 1, current, -1):
                if self._direct_connection_clear(
                    route[current],
                    route[candidate],
                ):
                    selected = candidate
                    break
            if selected == current + 1 and not self._direct_connection_clear(
                route[current], route[selected]
            ):
                raise ValueError(
                    "Serialized VGH path cannot be converted into "
                    "collision-free mission segments"
                )
            result.append(route[selected])
            current = selected
        return result


def _cross(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    return first[..., 0] * second[..., 1] - first[..., 1] * second[..., 0]


def _points_on_segment(
    points: np.ndarray,
    start: np.ndarray,
    end: np.ndarray,
) -> np.ndarray:
    return (
        (points[:, 0] >= min(start[0], end[0]) - EPSILON)
        & (points[:, 0] <= max(start[0], end[0]) + EPSILON)
        & (points[:, 1] >= min(start[1], end[1]) - EPSILON)
        & (points[:, 1] <= max(start[1], end[1]) + EPSILON)
    )


def _point_on_segments(
    point: np.ndarray,
    starts: np.ndarray,
    ends: np.ndarray,
) -> np.ndarray:
    return (
        (point[0] >= np.minimum(starts[:, 0], ends[:, 0]) - EPSILON)
        & (point[0] <= np.maximum(starts[:, 0], ends[:, 0]) + EPSILON)
        & (point[1] >= np.minimum(starts[:, 1], ends[:, 1]) - EPSILON)
        & (point[1] <= np.maximum(starts[:, 1], ends[:, 1]) + EPSILON)
    )


def _normalize_xy(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector[:2]))
    if norm <= EPSILON:
        return np.zeros(2, dtype=np.float64)
    return np.asarray(vector[:2], dtype=np.float64) / norm


def _is_outside_reduced_directions(
    difference: np.ndarray,
    surface_directions: tuple[np.ndarray, np.ndarray],
    parameters: FarParameters,
) -> bool:
    direction = _normalize_xy(difference)
    distance = float(np.linalg.norm(difference[:2]))
    margin = parameters.angle_noise
    if distance * math.sin(margin) < parameters.robot_dim:
        margin = math.asin(
            parameters.robot_dim / max(distance, parameters.robot_dim)
        )

    first = _normalize_xy(surface_directions[0])
    second = _normalize_xy(surface_directions[1])
    opposite = -second
    threshold = _noise_cosine(float(np.dot(first, opposite)), margin)
    if (
        float(np.dot(direction, first)) > threshold
        and float(np.dot(direction, opposite)) > threshold
    ):
        return True

    opposite = -first
    threshold = _noise_cosine(float(np.dot(second, opposite)), margin)
    return (
        float(np.dot(direction, second)) > threshold
        and float(np.dot(direction, opposite)) > threshold
    )


def _noise_cosine(dot_value: float, noise: float) -> float:
    theta = math.acos(max(-1.0, min(1.0, dot_value)))
    return math.cos(max(0.0, min(math.pi, theta + noise)))


def _reject(result: CandidateResult, reason: str, node_id: int) -> None:
    result.rejected.setdefault(reason, []).append(node_id)


def _reconstruct_path(
    node_id: int,
    parents: dict[int, int | None],
) -> list[int]:
    path = [node_id]
    while parents[path[-1]] is not None:
        parent = parents[path[-1]]
        assert parent is not None
        path.append(parent)
    path.reverse()
    return path


def _deduplicate_points(points: Sequence[np.ndarray]) -> list[np.ndarray]:
    result: list[np.ndarray] = []
    for point in points:
        if result and np.linalg.norm(point[:2] - result[-1][:2]) < EPSILON:
            continue
        result.append(point)
    return result


def load_mission_document(path: Path) -> MissionDocument:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid mission YAML '{path}': {error}") from error
    if not isinstance(document, dict):
        raise ValueError("Mission YAML must contain a mapping")
    frame_id = document.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("'frame_id' must be a non-empty string")
    initial_pose = _load_pose(document.get("initial_pose"), "initial_pose")
    raw_waypoints = document.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("'waypoints' must be a non-empty list")
    waypoints = tuple(
        _load_pose(value, f"waypoints[{index}]")
        for index, value in enumerate(raw_waypoints)
    )
    return MissionDocument(frame_id.strip(), initial_pose, waypoints)


def _load_pose(value: object, field_name: str) -> MissionPose:
    if not isinstance(value, dict):
        raise ValueError(f"'{field_name}' must be a mapping")
    position = _numeric_mapping(
        value.get("position"),
        f"{field_name}.position",
        ("x", "y", "z"),
    )
    orientation = _numeric_mapping(
        value.get("orientation"),
        f"{field_name}.orientation",
        ("x", "y", "z", "w"),
    )
    quaternion = np.asarray(
        [
            orientation["x"],
            orientation["y"],
            orientation["z"],
            orientation["w"],
        ],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-9:
        raise ValueError(
            f"'{field_name}.orientation' quaternion cannot be zero"
        )
    quaternion /= norm
    return MissionPose(
        position["x"],
        position["y"],
        position["z"],
        float(quaternion[0]),
        float(quaternion[1]),
        float(quaternion[2]),
        float(quaternion[3]),
    )


def _numeric_mapping(
    value: object,
    field_name: str,
    keys: Sequence[str],
) -> dict[str, float]:
    if not isinstance(value, dict):
        raise ValueError(f"'{field_name}' must be a mapping")
    result: dict[str, float] = {}
    for key in keys:
        raw = value.get(key)
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"'{field_name}.{key}' must be numeric")
        result[key] = float(raw)
        if not math.isfinite(result[key]):
            raise ValueError(f"'{field_name}.{key}' must be finite")
    return result


def load_far_parameters(path: Path) -> FarParameters:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise ValueError(
            f"Invalid FAR configuration '{path}': {error}"
        ) from error
    try:
        values = document["far_planner"]["ros__parameters"]
    except (KeyError, TypeError) as error:
        raise ValueError(
            f"{path} must contain far_planner.ros__parameters"
        ) from error

    def number(name: str, default: float | None = None) -> float:
        raw = values.get(name, default)
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"FAR parameter '{name}' must be numeric")
        result = float(raw)
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError(
                f"FAR parameter '{name}' must be positive and finite"
            )
        return result

    parameters = FarParameters(
        voxel_dim=number("voxel_dim", 0.2),
        robot_dim=number("robot_dim", 0.8),
        vehicle_height=number("vehicle_height", 0.75),
        sensor_range=number("sensor_range", 30.0),
        local_planner_range=number("local_planner_range", 5.0),
        angle_noise=math.radians(number("util/angle_noise", 15.0)),
        converge_distance=number("g_planner/converge_distance", 1.0),
        floor_height=number("map_handler/floor_height", 2.0),
        is_multi_layer=bool(values.get("is_multi_layer", False)),
    )
    if parameters.is_multi_layer:
        raise ValueError(
            "Mission compiler currently supports FAR single-layer prior maps "
            "only"
        )
    return parameters


def load_vgh(path: Path) -> dict[int, VghNode]:
    nodes: dict[int, VghNode] = {}
    for line_number, raw_line in enumerate(
        path.read_text(encoding="utf-8").splitlines(),
        1,
    ):
        tokens = raw_line.split()
        if not tokens:
            continue
        if len(tokens) < 18:
            raise ValueError(f"{path}:{line_number}: incomplete VGH node")
        try:
            node_id = int(tokens[0])
            if node_id in nodes:
                raise ValueError(
                    f"{path}:{line_number}: duplicate VGH node id {node_id}"
                )
            groups: list[list[int]] = [[]]
            for token in tokens[15:]:
                if token == "|":
                    groups.append([])
                else:
                    groups[-1].append(int(token))
            if len(groups) != 4:
                raise ValueError(
                    f"{path}:{line_number}: expected three VGH separators"
                )
            nodes[node_id] = VghNode(
                node_id=node_id,
                free_direction=FreeDirection(int(tokens[1])),
                position=np.asarray(
                    [float(value) for value in tokens[2:5]],
                    dtype=np.float64,
                ),
                surface_directions=(
                    np.asarray(
                        [float(value) for value in tokens[5:8]],
                        dtype=np.float64,
                    ),
                    np.asarray(
                        [float(value) for value in tokens[8:11]],
                        dtype=np.float64,
                    ),
                ),
                is_covered=bool(int(tokens[11])),
                is_frontier=bool(int(tokens[12])),
                is_navpoint=bool(int(tokens[13])),
                is_boundary=bool(int(tokens[14])),
                connections=tuple(groups[0]),
                polygon_connections=tuple(groups[1]),
                contour_connections=tuple(groups[2]),
                trajectory_connections=tuple(groups[3]),
            )
        except (ValueError, IndexError) as error:
            if (
                isinstance(error, ValueError)
                and str(error).startswith(str(path))
            ):
                raise
            raise ValueError(
                f"{path}:{line_number}: invalid VGH node: {error}"
            ) from error

    for node in nodes.values():
        for field_name, references in (
            ("connections", node.connections),
            ("polygon_connections", node.polygon_connections),
            ("contour_connections", node.contour_connections),
            ("trajectory_connections", node.trajectory_connections),
        ):
            missing = [
                node_id for node_id in references if node_id not in nodes
            ]
            if missing:
                raise ValueError(
                    f"VGH node {node.node_id} has missing {field_name}: "
                    f"{missing}"
                )
        for neighbor_id in node.connections:
            if node.node_id not in nodes[neighbor_id].connections:
                raise ValueError(
                    f"VGH connection {node.node_id}<->{neighbor_id} is "
                    "asymmetric"
                )
    return nodes


def compile_mission(
    mission: MissionDocument,
    graph: PriorGraph,
    pcd_bounds: tuple[np.ndarray, np.ndarray],
    max_goal_spacing: float,
) -> tuple[dict[str, object], dict[str, object]]:
    if not math.isfinite(max_goal_spacing) or max_goal_spacing <= 0.0:
        raise ValueError("max_goal_spacing must be positive and finite")
    minimum, maximum = pcd_bounds
    source_poses = [mission.initial_pose, *mission.waypoints]
    for index, pose in enumerate(source_poses):
        if np.any(pose.xy < minimum[:2]) or np.any(
            pose.xy > maximum[:2]
        ):
            label = (
                "initial_pose"
                if index == 0
                else f"waypoints[{index - 1}]"
            )
            raise ValueError(f"{label} lies outside the PCD XY bounds")

    compiled_waypoints: list[MissionPose] = []
    report: dict[str, object] = {
        "status": "VALIDATING_STATIC",
        "assumption": (
            "Each leg starts with the robot at the preceding compiled "
            "waypoint; no recorded odometry is used."
        ),
        "validation_scope": (
            "Static VGH topology, FAR endpoint direction predicates, "
            "serialized boundary contours, sensor-range start attachment, "
            "and PCD coverage."
        ),
        "original_waypoint_count": len(mission.waypoints),
        "max_goal_spacing": max_goal_spacing,
        "far_parameters": {
            "voxel_dim": graph.parameters.voxel_dim,
            "robot_dim": graph.parameters.robot_dim,
            "vehicle_height": graph.parameters.vehicle_height,
            "sensor_range": graph.parameters.sensor_range,
            "local_planner_range": graph.parameters.local_planner_range,
            "angle_noise_degrees": math.degrees(
                graph.parameters.angle_noise
            ),
            "converge_distance": graph.parameters.converge_distance,
            "floor_height": graph.parameters.floor_height,
            "is_multi_layer": graph.parameters.is_multi_layer,
            "derived_nav_clear_distance": (
                graph.parameters.nav_clear_distance
            ),
        },
        "legs": [],
    }
    legs = report["legs"]
    assert isinstance(legs, list)

    current_pose = mission.initial_pose
    for index, target_pose in enumerate(mission.waypoints):
        plan = graph.plan_leg(current_pose, target_pose)
        if plan is None:
            start_position = graph._synthetic_position(current_pose)
            goal_position = graph._synthetic_position(target_pose)
            start_candidates = graph._start_candidates(start_position)
            distances, _ = graph._distances_from(
                start_position,
                start_candidates,
            )
            goal_candidates = graph._goal_candidates(goal_position, distances)
            leg_report = {
                "original_leg_index": index,
                "status": "REJECTED_STATIC",
                "start": _pose_position_report(current_pose),
                "goal": _pose_position_report(target_pose),
                "start_candidates": start_candidates.as_report(),
                "goal_candidates": goal_candidates.as_report(),
                "direct_connection": graph._direct_connection_clear(
                    start_position, goal_position
                ),
            }
            legs.append(leg_report)
            report["status"] = "REJECTED_STATIC"
            report["failure"] = (
                f"Original leg {index} has no FAR-compatible route"
            )
            raise CompilationError(str(report["failure"]), report)

        dense_points = _densify_route(plan.route, max_goal_spacing)
        inserted = dense_points[1:-1]
        generated_poses = _poses_for_points(inserted, target_pose)
        compiled_waypoints.extend(generated_poses)
        compiled_waypoints.append(target_pose)
        legs.append(
            {
                "original_leg_index": index,
                "status": "VALIDATED_STATIC",
                "mode": plan.mode,
                "direct_connection": plan.mode == "direct",
                "start": _pose_position_report(current_pose),
                "goal": _pose_position_report(target_pose),
                "start_candidates": plan.start_candidates.as_report(),
                "goal_candidates": plan.goal_candidates.as_report(),
                "selected_goal_node": plan.selected_goal_node,
                "selected_graph_path": plan.graph_path,
                "route_length": _route_length(plan.route),
                "inserted_waypoint_count": len(generated_poses),
                "inserted_waypoints": [
                    _pose_position_report(pose) for pose in generated_poses
                ],
            }
        )
        current_pose = target_pose

    report["status"] = "VALIDATED_STATIC"
    report["compiled_waypoint_count"] = len(compiled_waypoints)
    report["inserted_waypoint_count"] = (
        len(compiled_waypoints) - len(mission.waypoints)
    )
    compiled_document: dict[str, object] = {
        "frame_id": mission.frame_id,
        "initial_pose": mission.initial_pose.as_mapping(),
        "waypoints": [pose.as_mapping() for pose in compiled_waypoints],
    }
    return compiled_document, report


def _densify_route(
    route: Sequence[np.ndarray],
    max_spacing: float,
) -> list[np.ndarray]:
    result = [np.asarray(route[0], dtype=np.float64)]
    for start, end in zip(route, route[1:]):
        distance = float(np.linalg.norm(end[:2] - start[:2]))
        segment_count = max(1, int(math.ceil(distance / max_spacing)))
        for segment in range(1, segment_count + 1):
            ratio = segment / segment_count
            result.append(start + (end - start) * ratio)
    return _deduplicate_points(result)


def _poses_for_points(
    points: Sequence[np.ndarray],
    target_pose: MissionPose,
) -> list[MissionPose]:
    if not points:
        return []
    destinations = [*points[1:], target_pose.xy]
    poses: list[MissionPose] = []
    for point, destination in zip(points, destinations):
        yaw = math.atan2(destination[1] - point[1], destination[0] - point[0])
        poses.append(
            MissionPose(
                x=float(point[0]),
                y=float(point[1]),
                z=target_pose.z,
                qx=0.0,
                qy=0.0,
                qz=math.sin(yaw / 2.0),
                qw=math.cos(yaw / 2.0),
            )
        )
    return poses


def _route_length(route: Sequence[np.ndarray]) -> float:
    return float(
        sum(
            np.linalg.norm(end[:2] - start[:2])
            for start, end in zip(route, route[1:])
        )
    )


def _pose_position_report(pose: MissionPose) -> dict[str, float]:
    return {"x": pose.x, "y": pose.y, "z": pose.z}


def render_compiled_preview(
    pcd_points: np.ndarray,
    output_path: Path,
    mission: MissionDocument,
    compiled_document: dict[str, object],
    report: dict[str, object],
    graph: PriorGraph,
    *,
    force: bool = False,
) -> None:
    """Render the optimized route over the PCD and serialized VGH."""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    if output_path.exists() and not force:
        raise FileExistsError(
            f"{output_path} already exists; use --force to replace it"
        )
    raw_waypoints = compiled_document.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("Compiled mission contains no waypoints to preview")

    compiled_positions = np.asarray(
        [
            [
                float(waypoint["position"]["x"]),
                float(waypoint["position"]["y"]),
            ]
            for waypoint in raw_waypoints
        ],
        dtype=np.float64,
    )
    route_positions = np.vstack((mission.initial_pose.xy, compiled_positions))
    graph_positions = np.asarray(
        [node.position[:2] for node in graph.nodes.values()],
        dtype=np.float64,
    )
    visibility_segments = _vgh_visibility_segments(graph)
    boundary_segments = np.stack(
        (graph.boundaries.starts, graph.boundaries.ends),
        axis=1,
    )
    original_indices = _original_compiled_indices(report)
    inserted_indices = sorted(
        set(range(len(compiled_positions))) - set(original_indices)
    )

    maximum_preview_points = 500_000
    stride = max(1, math.ceil(len(pcd_points) / maximum_preview_points))
    preview_points = pcd_points[::stride]
    x_min = min(
        float(np.min(preview_points[:, 0])),
        float(np.min(route_positions[:, 0])),
        float(np.min(graph_positions[:, 0])),
    )
    x_max = max(
        float(np.max(preview_points[:, 0])),
        float(np.max(route_positions[:, 0])),
        float(np.max(graph_positions[:, 0])),
    )
    y_min = min(
        float(np.min(preview_points[:, 1])),
        float(np.min(route_positions[:, 1])),
        float(np.min(graph_positions[:, 1])),
    )
    y_max = max(
        float(np.max(preview_points[:, 1])),
        float(np.max(route_positions[:, 1])),
        float(np.max(graph_positions[:, 1])),
    )
    span = max(x_max - x_min, y_max - y_min, 1.0)
    margin = max(0.5, span * 0.025)

    figure, axes = plt.subplots(figsize=(14.0, 11.0), dpi=200)
    figure.patch.set_facecolor("#f8fafc")
    axes.set_facecolor("#ffffff")
    try:
        axes.scatter(
            preview_points[:, 0],
            preview_points[:, 1],
            s=0.45,
            c="#475569",
            alpha=0.24,
            linewidths=0,
            rasterized=True,
            label="Prior map",
            zorder=1,
        )
        if len(visibility_segments):
            axes.add_collection(
                LineCollection(
                    visibility_segments,
                    colors="#0f766e",
                    linewidths=0.28,
                    alpha=0.10,
                    label="VGH visibility graph",
                    rasterized=True,
                    zorder=2,
                )
            )
        axes.scatter(
            graph_positions[:, 0],
            graph_positions[:, 1],
            s=1.4,
            c="#0f766e",
            alpha=0.26,
            linewidths=0,
            rasterized=True,
            zorder=2,
        )
        if len(boundary_segments):
            axes.add_collection(
                LineCollection(
                    boundary_segments,
                    colors="#dc2626",
                    linewidths=0.9,
                    alpha=0.82,
                    label="VGH boundary contours",
                    rasterized=True,
                    zorder=3,
                )
            )
        axes.plot(
            route_positions[:, 0],
            route_positions[:, 1],
            color="#2563eb",
            linewidth=1.8,
            alpha=0.94,
            label="Compiled route",
            zorder=4,
        )
        if inserted_indices:
            inserted = compiled_positions[inserted_indices]
            axes.scatter(
                inserted[:, 0],
                inserted[:, 1],
                s=10,
                c="#0891b2",
                edgecolors="#ffffff",
                linewidths=0.3,
                label="Inserted goals",
                zorder=5,
            )
        original = compiled_positions[original_indices]
        axes.scatter(
            original[:, 0],
            original[:, 1],
            s=34,
            c="#c2410c",
            edgecolors="#ffffff",
            linewidths=0.7,
            label="Recorded goals",
            zorder=6,
        )
        axes.scatter(
            [mission.initial_pose.x],
            [mission.initial_pose.y],
            s=80,
            c="#15803d",
            marker="*",
            edgecolors="#ffffff",
            linewidths=0.8,
            label="Initial pose",
            zorder=7,
        )
        for goal_number, point in enumerate(original, 1):
            axes.annotate(
                f"G{goal_number}",
                xy=point,
                xytext=(5, 5),
                textcoords="offset points",
                fontsize=7,
                color="#7c2d12",
                zorder=8,
            )

        axes.set_xlim(x_min - margin, x_max + margin)
        axes.set_ylim(y_min - margin, y_max + margin)
        axes.set_aspect("equal", adjustable="box")
        axes.set_xlabel("Map X (m)")
        axes.set_ylabel("Map Y (m)")
        preview_name = output_path.stem.removesuffix("_compiled")
        axes.set_title(
            f"{preview_name.replace('_', ' ').title()} - Compiled Mission"
        )
        axes.grid(True, color="#cbd5e1", linewidth=0.45, alpha=0.6)
        axes.legend(loc="best", framealpha=0.94)
        figure.tight_layout()

        output_path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{output_path.stem}.",
            suffix=".png",
            dir=output_path.parent,
        )
        os.close(descriptor)
        temporary_path = Path(temporary_name)
        try:
            figure.savefig(temporary_path, format="png", bbox_inches="tight")
            if output_path.exists() and not force:
                raise FileExistsError(
                    f"{output_path} already exists; use --force to replace it"
                )
            temporary_path.replace(output_path)
        except BaseException:
            temporary_path.unlink(missing_ok=True)
            raise
    finally:
        plt.close(figure)


def _vgh_visibility_segments(graph: PriorGraph) -> np.ndarray:
    segments: list[np.ndarray] = []
    for node in graph.nodes.values():
        for neighbor_id in node.connections:
            if neighbor_id <= node.node_id:
                continue
            segments.append(
                np.asarray(
                    (
                        node.position[:2],
                        graph.nodes[neighbor_id].position[:2],
                    ),
                    dtype=np.float64,
                )
            )
    if not segments:
        return np.empty((0, 2, 2), dtype=np.float64)
    return np.asarray(segments, dtype=np.float64)


def _original_compiled_indices(report: dict[str, object]) -> list[int]:
    legs = report.get("legs")
    if not isinstance(legs, list):
        raise ValueError("Compilation report has no leg records")
    indices: list[int] = []
    cursor = 0
    for leg in legs:
        if not isinstance(leg, dict):
            raise ValueError("Compilation report contains an invalid leg")
        inserted_count = leg.get("inserted_waypoint_count")
        if not isinstance(inserted_count, int) or inserted_count < 0:
            raise ValueError(
                "Compilation report has an invalid inserted count"
            )
        cursor += inserted_count
        indices.append(cursor)
        cursor += 1
    return indices


def _atomic_write(path: Path, content: str, force: bool) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and not force:
        raise FileExistsError(
            f"{path} already exists; use --force to replace it"
        )
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        dir=path.parent,
        text=True,
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists() and not force:
            raise FileExistsError(
                f"{path} already exists; use --force to replace it"
            )
        temporary_path.replace(path)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise


def _default_output_path(mission_path: Path) -> Path:
    return mission_path.with_name(f"{mission_path.stem}_compiled.yaml")


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate and densify a mission using FAR's serialized prior "
            "graph and endpoint connection criteria."
        )
    )
    parser.add_argument("--mission", required=True, type=Path)
    parser.add_argument("--pcd", required=True, type=Path)
    parser.add_argument("--vgh", required=True, type=Path)
    parser.add_argument("--far-config", required=True, type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--report", type=Path)
    preview_group = parser.add_mutually_exclusive_group()
    preview_group.add_argument(
        "--preview",
        type=Path,
        help="Preview PNG path; defaults beside the compiled mission",
    )
    preview_group.add_argument(
        "--no-preview",
        action="store_true",
        help="Do not render a compiled mission preview",
    )
    parser.add_argument(
        "--max-goal-spacing",
        type=float,
        help=(
            "Maximum compiled waypoint spacing; defaults to "
            "local_planner_range"
        ),
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Replace existing output and report files",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    mission_path = args.mission.expanduser().resolve()
    pcd_path = args.pcd.expanduser().resolve()
    vgh_path = args.vgh.expanduser().resolve()
    config_path = args.far_config.expanduser().resolve()
    output_path = (
        args.output.expanduser().resolve()
        if args.output
        else _default_output_path(mission_path)
    )
    report_path = (
        args.report.expanduser().resolve()
        if args.report
        else output_path.with_suffix(".report.json")
    )
    preview_path = (
        None
        if args.no_preview
        else (
            args.preview.expanduser().resolve()
            if args.preview
            else output_path.with_suffix(".png")
        )
    )

    try:
        mission = load_mission_document(mission_path)
        parameters = load_far_parameters(config_path)
        nodes = load_vgh(vgh_path)
        cloud = PointCloud.from_path(pcd_path)
        missing_fields = {"x", "y", "z"} - set(cloud.fields)
        if missing_fields:
            raise ValueError(f"{pcd_path} must contain x, y, z fields")
        pcd_points = np.asarray(
            cloud.numpy(("x", "y", "z")),
            dtype=np.float32,
        )
        pcd_points = pcd_points[np.isfinite(pcd_points).all(axis=1)]
        if len(pcd_points) == 0:
            raise ValueError(f"{pcd_path} contains no finite points")
        spacing = (
            args.max_goal_spacing
            if args.max_goal_spacing is not None
            else parameters.local_planner_range
        )
        graph = PriorGraph(nodes, parameters)
        compiled, report = compile_mission(
            mission,
            graph,
            (pcd_points.min(axis=0), pcd_points.max(axis=0)),
            spacing,
        )
        report["assets"] = {
            "mission": str(mission_path),
            "pcd": str(pcd_path),
            "pcd_point_count": int(len(pcd_points)),
            "vgh": str(vgh_path),
            "vgh_node_count": len(nodes),
            "far_config": str(config_path),
            "preview": str(preview_path) if preview_path is not None else None,
        }
        _atomic_write(
            output_path,
            yaml.safe_dump(compiled, sort_keys=False),
            args.force,
        )
        _atomic_write(
            report_path,
            json.dumps(report, indent=2) + "\n",
            args.force,
        )
        if preview_path is not None:
            render_compiled_preview(
                pcd_points,
                preview_path,
                mission,
                compiled,
                report,
                graph,
                force=args.force,
            )
    except CompilationError as error:
        error.report["assets"] = {
            "mission": str(mission_path),
            "pcd": str(pcd_path),
            "vgh": str(vgh_path),
            "far_config": str(config_path),
        }
        _atomic_write(
            report_path,
            json.dumps(error.report, indent=2) + "\n",
            args.force,
        )
        parser.exit(2, f"mission_compiler: {error}; report: {report_path}\n")
    except (FileExistsError, OSError, ValueError) as error:
        parser.exit(2, f"mission_compiler: {error}\n")

    print(f"Compiled mission: {output_path}")
    print(f"Validation report: {report_path}")
    if preview_path is not None:
        print(f"Compiled preview: {preview_path}")
    print(
        f"Waypoints: {report['original_waypoint_count']} original, "
        f"{report['inserted_waypoint_count']} inserted, "
        f"{report['compiled_waypoint_count']} total"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
