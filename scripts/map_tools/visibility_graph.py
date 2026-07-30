"""FAR-compatible visibility graph construction."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum

import numpy as np


class FreeDirection(IntEnum):
    """FAR Planner free-space classification for a graph vertex."""

    UNKNOWN = 0
    CONVEX = 1
    CONCAVE = 2
    PILLAR = 3


@dataclass
class VisibilityNode:
    """One node in FAR Planner's serialized visibility graph."""

    node_id: int
    position: np.ndarray
    free_direction: FreeDirection
    surface_directions: tuple[np.ndarray, np.ndarray]
    connections: list[int]
    contour_connections: list[int]


def _normalize_xy(vector: np.ndarray) -> np.ndarray:
    norm = math.hypot(float(vector[0]), float(vector[1]))
    if norm <= 1e-7:
        return np.zeros(3, dtype=np.float32)
    return np.asarray(
        [float(vector[0]) / norm, float(vector[1]) / norm, 0.0],
        dtype=np.float32,
    )


def point_inside_polygon(polygon: np.ndarray, point: np.ndarray) -> bool:
    """Return whether a point lies inside a polygon using FAR's rule."""
    if len(polygon) < 3:
        return False

    inside = False
    previous = len(polygon) - 1
    point_x = float(point[0])
    point_y = float(point[1])
    for current in range(len(polygon)):
        current_x = float(polygon[current, 0])
        current_y = float(polygon[current, 1])
        previous_x = float(polygon[previous, 0])
        previous_y = float(polygon[previous, 1])
        if (
            (current_y <= point_y < previous_y)
            or (previous_y <= point_y < current_y)
        ):
            intersection_x = (
                (previous_x - current_x)
                * (point_y - current_y)
                / (previous_y - current_y)
                + current_x
            )
            if point_x < intersection_x:
                inside = not inside
        previous = current
    return inside


def _point_on_segment(
    start: np.ndarray,
    point: np.ndarray,
    end: np.ndarray,
) -> bool:
    return (
        float(point[0]) <= max(float(start[0]), float(end[0]))
        and float(point[0]) >= min(float(start[0]), float(end[0]))
        and float(point[1]) <= max(float(start[1]), float(end[1]))
        and float(point[1]) >= min(float(start[1]), float(end[1]))
    )


def _orientation(
    first: np.ndarray,
    second: np.ndarray,
    third: np.ndarray,
) -> int:
    value = (
        (float(second[1]) - float(first[1]))
        * (float(third[0]) - float(second[0]))
        - (float(second[0]) - float(first[0]))
        * (float(third[1]) - float(second[1]))
    )
    if abs(value) < 1e-7:
        return 0
    return 1 if value > 0.0 else 2


def segments_intersect(
    first_start: np.ndarray,
    first_end: np.ndarray,
    second_start: np.ndarray,
    second_end: np.ndarray,
) -> bool:
    """Return whether two closed 2D segments intersect."""
    first_orientation = _orientation(first_start, first_end, second_start)
    second_orientation = _orientation(first_start, first_end, second_end)
    third_orientation = _orientation(second_start, second_end, first_start)
    fourth_orientation = _orientation(second_start, second_end, first_end)

    if (
        first_orientation != second_orientation
        and third_orientation != fourth_orientation
    ):
        return True
    if first_orientation == 0 and _point_on_segment(
        first_start,
        second_start,
        first_end,
    ):
        return True
    if second_orientation == 0 and _point_on_segment(
        first_start,
        second_end,
        first_end,
    ):
        return True
    if third_orientation == 0 and _point_on_segment(
        second_start,
        first_start,
        second_end,
    ):
        return True
    return fourth_orientation == 0 and _point_on_segment(
        second_start,
        first_end,
        second_end,
    )


def _surface_topology_direction(
    surface_directions: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, bool]:
    topology_direction = surface_directions[0] + surface_directions[1]
    if math.hypot(
        float(topology_direction[0]),
        float(topology_direction[1]),
    ) <= 1e-7:
        return np.zeros(3, dtype=np.float32), True
    return _normalize_xy(topology_direction), False


def _is_outside_reduced_directions(
    difference: np.ndarray,
    surface_directions: tuple[np.ndarray, np.ndarray],
) -> bool:
    normalized_direction = _normalize_xy(difference)
    first_direction, second_direction = surface_directions

    opposite_direction = -second_direction
    threshold = float(np.dot(first_direction, opposite_direction))
    if (
        float(np.dot(normalized_direction, first_direction)) > threshold
        and float(np.dot(normalized_direction, opposite_direction)) > threshold
    ):
        return True

    opposite_direction = -first_direction
    threshold = float(np.dot(second_direction, opposite_direction))
    return (
        float(np.dot(normalized_direction, second_direction)) > threshold
        and float(np.dot(normalized_direction, opposite_direction)) > threshold
    )


def _connect_contour(first: VisibilityNode, second: VisibilityNode) -> None:
    if first.node_id == second.node_id:
        return
    if second.node_id in first.contour_connections:
        return
    first.contour_connections.append(second.node_id)
    second.contour_connections.append(first.node_id)


def _connect_visibility(first: VisibilityNode, second: VisibilityNode) -> None:
    if first.node_id == second.node_id:
        return
    if second.node_id not in first.connections:
        first.connections.append(second.node_id)
        second.connections.append(first.node_id)


def _reproject_visibility_edge(
    first: VisibilityNode,
    second: VisibilityNode,
) -> tuple[np.ndarray, np.ndarray]:
    first_topology, _ = _surface_topology_direction(first.surface_directions)
    second_topology, _ = _surface_topology_direction(second.surface_directions)
    first_sign = -1.0 if first.free_direction == FreeDirection.CONVEX else 1.0
    second_sign = (
        -1.0 if second.free_direction == FreeDirection.CONVEX else 1.0
    )
    return (
        first.position + first_topology * first_sign * 0.05,
        second.position + second_topology * second_sign * 0.05,
    )


def _edge_intersects_polygons(
    edge: tuple[np.ndarray, np.ndarray],
    polygons: list[np.ndarray],
) -> bool:
    for polygon in polygons:
        for index in range(len(polygon)):
            if segments_intersect(
                polygon[index],
                polygon[(index + 1) % len(polygon)],
                edge[0],
                edge[1],
            ):
                return True
    return False


def _is_valid_visibility_connection(
    first: VisibilityNode,
    second: VisibilityNode,
    polygons: list[np.ndarray],
    height_tolerance: float,
) -> bool:
    if second.node_id in first.contour_connections:
        return True
    if abs(float(first.position[2] - second.position[2])) > height_tolerance:
        return False
    if (
        first.free_direction == FreeDirection.CONCAVE
        or second.free_direction == FreeDirection.CONCAVE
    ):
        return False
    if (
        first.free_direction != FreeDirection.PILLAR
        and not _is_outside_reduced_directions(
            second.position - first.position,
            first.surface_directions,
        )
    ):
        return False
    if (
        second.free_direction != FreeDirection.PILLAR
        and not _is_outside_reduced_directions(
            first.position - second.position,
            second.surface_directions,
        )
    ):
        return False
    edge = _reproject_visibility_edge(first, second)
    return not _edge_intersects_polygons(edge, polygons)


def build_visibility_graph(
    polygons: list[np.ndarray],
    free_point: tuple[float, float, float],
    boundary_z: float,
    height_tolerance: float = 2.0,
) -> list[VisibilityNode]:
    """Build FAR-compatible visibility nodes from boundary polygons."""
    free_position = np.asarray(free_point, dtype=np.float32)
    if free_position.shape != (3,) or not np.isfinite(free_position).all():
        raise ValueError("free_point must contain three finite values")
    if not math.isfinite(boundary_z):
        raise ValueError("boundary_z must be finite")
    if not math.isfinite(height_tolerance) or height_tolerance < 0.0:
        raise ValueError("height_tolerance must be finite and non-negative")

    free_position = np.round(free_position, decimals=6)
    graph_boundary_z = round(float(boundary_z), 6)
    graph: list[VisibilityNode] = []
    graph_polygons: list[np.ndarray] = []
    for polygon_index, polygon in enumerate(polygons):
        polygon_array = np.asarray(polygon, dtype=np.float32)
        if (
            polygon_array.ndim != 2
            or polygon_array.shape[1] < 2
            or len(polygon_array) < 3
            or not np.isfinite(polygon_array[:, :2]).all()
        ):
            raise ValueError(
                f"polygon {polygon_index} must contain three finite "
                "XY vertices"
            )
        polygon_array = np.round(polygon_array, decimals=6)
        vertices = np.column_stack(
            (
                polygon_array[:, :2],
                np.full(
                    len(polygon_array),
                    graph_boundary_z,
                    dtype=np.float32,
                ),
            )
        ).astype(np.float32, copy=False)
        graph_polygons.append(vertices)
        polygon_nodes = [
            VisibilityNode(
                node_id=len(graph) + index,
                position=vertex.copy(),
                free_direction=FreeDirection.UNKNOWN,
                surface_directions=(
                    np.zeros(3, dtype=np.float32),
                    np.zeros(3, dtype=np.float32),
                ),
                connections=[],
                contour_connections=[],
            )
            for index, vertex in enumerate(vertices)
        ]

        free_point_inside = point_inside_polygon(vertices, free_position)
        for index, node in enumerate(polygon_nodes):
            previous_node = polygon_nodes[(index - 1) % len(polygon_nodes)]
            next_node = polygon_nodes[(index + 1) % len(polygon_nodes)]
            node.surface_directions = (
                _normalize_xy(previous_node.position - node.position),
                _normalize_xy(next_node.position - node.position),
            )
            topology_direction, is_wall = _surface_topology_direction(
                node.surface_directions
            )
            if is_wall:
                node.free_direction = FreeDirection.CONCAVE
            else:
                evaluation_inside = point_inside_polygon(
                    vertices,
                    node.position + topology_direction * 0.25,
                )
                node.free_direction = (
                    FreeDirection.CONVEX
                    if evaluation_inside != free_point_inside
                    else FreeDirection.CONCAVE
                )
            _connect_contour(next_node, node)
        graph.extend(polygon_nodes)

    for first_index in range(len(graph)):
        for second_index in range(first_index):
            first = graph[first_index]
            second = graph[second_index]
            if _is_valid_visibility_connection(
                first,
                second,
                graph_polygons,
                height_tolerance,
            ):
                _connect_visibility(first, second)
    return graph
