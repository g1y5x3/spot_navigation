"""Boundary contour extraction and polygon simplification."""

from __future__ import annotations

import math
from operator import itemgetter

import cv2
import numpy as np

from map_tools.far_grid import (
    make_kernel,
    rasterize_clear_trajectory,
    select_free_space_component,
)


def shoelace_area(polygon: np.ndarray) -> float:
    if len(polygon) < 3:
        return 0.0
    x = polygon[:, 0]
    y = polygon[:, 1]
    return 0.5 * float(
        np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))
    )


def world_from_contour(
    contour: np.ndarray,
    origin: np.ndarray,
    resolution: float,
) -> np.ndarray:
    points = contour.reshape(-1, 2).astype(np.float32)
    world = np.empty_like(points)
    world[:, 0] = origin[0] + (points[:, 0] + 0.5) * resolution
    world[:, 1] = origin[1] + (points[:, 1] + 0.5) * resolution
    return world


def simplify_contour(
    contour: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    epsilon_m: float,
    max_vertices: int,
) -> np.ndarray:
    epsilon_px = max(epsilon_m / resolution, 0.5)
    approximation = cv2.approxPolyDP(contour, epsilon_px, True)
    while len(approximation) > max_vertices:
        epsilon_px *= 1.35
        approximation = cv2.approxPolyDP(contour, epsilon_px, True)
    return world_from_contour(approximation, origin, resolution)


def extract_boundary_polygons(
    points: np.ndarray,
    bounds_points: np.ndarray,
    resolution: float,
    padding: float,
    close_radius: float,
    inflate_radius: float,
    simplify_epsilon: float,
    min_area: float,
    max_polygons: int,
    max_vertices: int,
    add_outer_boundary: bool,
    clear_trajectory_xy: np.ndarray | None,
    clear_radius: float,
    requested_free_point_xy: np.ndarray | None = None,
) -> tuple[
    list[np.ndarray],
    dict[str, object],
    np.ndarray,
    np.ndarray,
    tuple[int, int],
]:
    min_xy = bounds_points[:, :2].min(axis=0) - padding
    max_xy = bounds_points[:, :2].max(axis=0) + padding
    origin = np.floor(min_xy / resolution) * resolution
    grid_max = np.ceil(max_xy / resolution) * resolution
    cols = int(math.ceil((grid_max[0] - origin[0]) / resolution)) + 1
    rows = int(math.ceil((grid_max[1] - origin[1]) / resolution)) + 1

    indices = np.floor((points[:, :2] - origin) / resolution).astype(np.int32)
    valid = (
        (indices[:, 0] >= 0)
        & (indices[:, 0] < cols)
        & (indices[:, 1] >= 0)
        & (indices[:, 1] < rows)
    )
    indices = indices[valid]
    occupancy = np.zeros((rows, cols), dtype=np.uint8)
    occupancy[indices[:, 1], indices[:, 0]] = 255

    close_kernel = make_kernel(close_radius, resolution)
    if close_kernel is not None:
        occupancy = cv2.morphologyEx(
            occupancy,
            cv2.MORPH_CLOSE,
            close_kernel,
        )
    inflate_kernel = make_kernel(inflate_radius, resolution)
    if inflate_kernel is not None:
        occupancy = cv2.dilate(occupancy, inflate_kernel)

    clear_stats = None
    if clear_trajectory_xy is not None:
        clear_mask, clear_stats = rasterize_clear_trajectory(
            occupancy.shape,
            origin,
            resolution,
            clear_trajectory_xy,
            clear_radius,
        )
        clear_cells = clear_mask > 0
        clear_stats["occupied_cells_removed"] = int(
            np.count_nonzero(occupancy[clear_cells])
        )
        occupancy[clear_cells] = 0

    free_component, free_space_stats = select_free_space_component(
        occupancy,
        origin,
        resolution,
        requested_free_point_xy,
    )
    contours, hierarchy = cv2.findContours(
        free_component,
        cv2.RETR_CCOMP,
        cv2.CHAIN_APPROX_NONE,
    )
    outer_records: list[tuple[float, np.ndarray]] = []
    obstacle_records: list[tuple[float, np.ndarray]] = []
    for contour_index, contour in enumerate(contours):
        area = abs(cv2.contourArea(contour)) * resolution * resolution
        if area < min_area:
            continue
        polygon = simplify_contour(
            contour,
            origin,
            resolution,
            simplify_epsilon,
            max_vertices,
        )
        area = abs(shoelace_area(polygon))
        if len(polygon) < 3 or area < min_area:
            continue
        parent = -1 if hierarchy is None else int(
            hierarchy[0, contour_index, 3]
        )
        (outer_records if parent == -1 else obstacle_records).append(
            (area, polygon)
        )

    outer_records.sort(key=itemgetter(0), reverse=True)
    obstacle_records.sort(key=itemgetter(0), reverse=True)
    outer_polygons = [polygon for _, polygon in outer_records]
    obstacle_polygons = [
        polygon for _, polygon in obstacle_records[:max_polygons]
    ]
    polygons = (
        (outer_polygons if add_outer_boundary else []) + obstacle_polygons
    )

    stats: dict[str, object] = {
        "grid": {"rows": rows, "cols": cols, "resolution": resolution},
        "origin": origin.tolist(),
        "points_projected": int(len(indices)),
        "occupied_cells": int(np.count_nonzero(occupancy)),
        "free_space": free_space_stats,
        "outer_boundary_polygons": (
            len(outer_polygons) if add_outer_boundary else 0
        ),
        "candidate_obstacle_polygons": len(obstacle_records),
        "obstacle_polygons": len(obstacle_polygons),
        "discarded_obstacle_polygons": max(
            0,
            len(obstacle_records) - len(obstacle_polygons),
        ),
        "total_polygons": len(polygons),
        "total_vertices": int(sum(len(polygon) for polygon in polygons)),
        "outer_boundary_areas_m2": [
            round(float(area), 3) for area, _ in outer_records
        ],
        "obstacle_areas_m2": [
            round(float(area), 3)
            for area, _ in obstacle_records[:max_polygons]
        ],
    }
    if clear_stats is not None:
        stats["trajectory_clear"] = clear_stats
    return polygons, stats, occupancy, origin, (rows, cols)
