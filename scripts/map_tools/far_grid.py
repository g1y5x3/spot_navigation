"""Occupancy, ground-height, and free-space grid operations."""

from __future__ import annotations

import math

import cv2
import numpy as np


def make_kernel(radius_m: float, resolution: float) -> np.ndarray | None:
    cells = int(round(radius_m / resolution))
    if cells <= 0:
        return None
    size = cells * 2 + 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))


def fill_nan_nearest(values: np.ndarray) -> np.ndarray:
    valid = np.isfinite(values)
    if np.all(valid):
        return values.astype(np.float32, copy=True)
    if not np.any(valid):
        raise ValueError("Cannot fill an all-NaN grid")

    invalid_mask = np.where(valid, 0, 255).astype(np.uint8)
    _, labels = cv2.distanceTransformWithLabels(
        invalid_mask,
        cv2.DIST_L2,
        cv2.DIST_MASK_PRECISE,
        labelType=cv2.DIST_LABEL_PIXEL,
    )
    valid_rows, valid_cols = np.where(valid)
    filled = values.astype(np.float32, copy=True)
    invalid_rows, invalid_cols = np.where(~valid)
    nearest = labels[invalid_rows, invalid_cols] - 1
    filled[invalid_rows, invalid_cols] = values[
        valid_rows[nearest],
        valid_cols[nearest],
    ]
    return filled


def rasterize_clear_trajectory(
    occupancy_shape: tuple[int, int],
    origin: np.ndarray,
    resolution: float,
    trajectory_xy: np.ndarray,
    clear_radius: float,
) -> tuple[np.ndarray, dict[str, object]]:
    rows, cols = occupancy_shape
    mask = np.zeros((rows, cols), dtype=np.uint8)
    if trajectory_xy.size == 0:
        return mask, {
            "points_total": 0,
            "points_in_bounds": 0,
            "clear_radius": clear_radius,
            "clear_cells": 0,
        }

    indices = np.floor((trajectory_xy - origin) / resolution).astype(np.int32)
    in_bounds = (
        (indices[:, 0] >= 0)
        & (indices[:, 0] < cols)
        & (indices[:, 1] >= 0)
        & (indices[:, 1] < rows)
    )
    radius_cells = max(1, int(math.ceil(clear_radius / resolution)))
    thickness = radius_cells * 2 + 1
    for start, end in zip(indices[:-1], indices[1:]):
        cv2.line(
            mask,
            (int(start[0]), int(start[1])),
            (int(end[0]), int(end[1])),
            255,
            thickness,
            cv2.LINE_8,
        )
    for col, row in indices[in_bounds]:
        cv2.circle(
            mask,
            (int(col), int(row)),
            radius_cells,
            255,
            -1,
            cv2.LINE_8,
        )

    return mask, {
        "points_total": int(len(indices)),
        "points_in_bounds": int(np.count_nonzero(in_bounds)),
        "clear_radius": clear_radius,
        "clear_cells": int(np.count_nonzero(mask)),
    }


def select_free_space_component(
    occupancy: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    requested_free_point_xy: np.ndarray | None = None,
) -> tuple[np.ndarray, dict[str, object]]:
    """Select the requested or largest connected free-space component."""
    free_mask = (occupancy == 0).astype(np.uint8)
    component_count, labels, component_stats, _ = (
        cv2.connectedComponentsWithStats(free_mask, connectivity=8)
    )
    if component_count <= 1:
        raise ValueError("Occupancy grid contains no free-space component")

    selection = "largest"
    if requested_free_point_xy is not None:
        requested = np.asarray(requested_free_point_xy, dtype=np.float64)
        if requested.shape != (2,) or not np.isfinite(requested).all():
            raise ValueError(
                "requested free point must contain two finite XY values"
            )
        index = np.floor((requested - origin) / resolution).astype(np.int32)
        row, col = int(index[1]), int(index[0])
        if not (
            0 <= row < occupancy.shape[0]
            and 0 <= col < occupancy.shape[1]
        ):
            raise ValueError(
                "requested free point lies outside the occupancy grid"
            )
        selected_label = int(labels[row, col])
        if selected_label == 0:
            raise ValueError("requested free point lies on an occupied cell")
        selection = "requested"
    else:
        selected_label = 1 + int(
            np.argmax(component_stats[1:, cv2.CC_STAT_AREA])
        )

    x = int(component_stats[selected_label, cv2.CC_STAT_LEFT])
    y = int(component_stats[selected_label, cv2.CC_STAT_TOP])
    width = int(component_stats[selected_label, cv2.CC_STAT_WIDTH])
    height = int(component_stats[selected_label, cv2.CC_STAT_HEIGHT])
    selected_cells = int(component_stats[selected_label, cv2.CC_STAT_AREA])
    component = cv2.compare(labels, selected_label, cv2.CMP_EQ)
    return component, {
        "components": component_count - 1,
        "selection": selection,
        "selected_label": selected_label,
        "selected_cells": selected_cells,
        "touches_map_edge": (
            x == 0
            or y == 0
            or x + width == occupancy.shape[1]
            or y + height == occupancy.shape[0]
        ),
    }


def _ground_percentile_grid(
    points: np.ndarray,
    cell_size: float,
    percentile: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    min_xy = points[:, :2].min(axis=0)
    max_xy = points[:, :2].max(axis=0)
    cols = int(math.ceil((max_xy[0] - min_xy[0]) / cell_size)) + 1
    rows = int(math.ceil((max_xy[1] - min_xy[1]) / cell_size)) + 1
    indices = np.floor((points[:, :2] - min_xy) / cell_size).astype(np.int32)
    flat = indices[:, 1] * cols + indices[:, 0]
    order = np.argsort(flat)
    sorted_flat = flat[order]
    sorted_z = points[order, 2]
    starts = np.r_[0, np.flatnonzero(np.diff(sorted_flat)) + 1]
    ends = np.r_[starts[1:], len(order)]
    ground = np.full(rows * cols, np.nan, dtype=np.float32)
    for start, end in zip(starts, ends):
        ground[sorted_flat[start]] = np.percentile(
            sorted_z[start:end],
            percentile,
        )
    return ground.reshape(rows, cols), indices, flat, len(starts)


def classify_obstacle_points_pmf(
    points: np.ndarray,
    obstacle_height: float,
    max_obstacle_height: float,
    cell_size: float,
    max_window_size: int,
    slope: float,
    initial_distance: float,
    max_distance: float,
    ground_percentile: float,
) -> tuple[np.ndarray, dict[str, object]]:
    ground_image, indices, _, ground_cell_count = _ground_percentile_grid(
        points,
        cell_size,
        ground_percentile,
    )
    observed_cells = np.isfinite(ground_image)
    ground_estimate = fill_nan_nearest(ground_image)
    non_ground_cells = np.zeros(ground_image.shape, dtype=bool)
    odd_max_window_size = max(3, int(max_window_size) | 1)
    window_records = []
    for window_size in range(3, odd_max_window_size + 1, 2):
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (window_size, window_size),
        )
        opened_ground = cv2.morphologyEx(
            ground_estimate,
            cv2.MORPH_OPEN,
            kernel,
        )
        radius_m = (window_size - 1) * 0.5 * cell_size
        threshold = min(max_distance, initial_distance + slope * radius_m)
        newly_non_ground = (
            (ground_estimate - opened_ground > threshold)
            & observed_cells
        )
        new_cell_count = int(
            np.count_nonzero(newly_non_ground & ~non_ground_cells)
        )
        non_ground_cells |= newly_non_ground
        np.copyto(ground_estimate, opened_ground, where=newly_non_ground)
        window_records.append(
            {
                "window_size": window_size,
                "radius_m": round(float(radius_m), 6),
                "height_threshold": round(float(threshold), 6),
                "new_non_ground_cells": new_cell_count,
            }
        )

    ground_z = ground_estimate[indices[:, 1], indices[:, 0]]
    height_above_ground = points[:, 2] - ground_z
    mask = (
        (height_above_ground >= obstacle_height)
        & (height_above_ground <= max_obstacle_height)
        & np.isfinite(height_above_ground)
    )
    return points[mask], {
        "mode": "pmf_opening",
        "obstacle_height": obstacle_height,
        "max_obstacle_height": max_obstacle_height,
        "pmf_cell_size": cell_size,
        "pmf_max_window_size": odd_max_window_size,
        "pmf_slope": slope,
        "pmf_initial_distance": initial_distance,
        "pmf_max_distance": max_distance,
        "ground_percentile": ground_percentile,
        "ground_cells": ground_cell_count,
        "pmf_non_ground_cells": int(np.count_nonzero(non_ground_cells)),
        "windows": window_records,
    }


def classify_obstacle_points(
    points: np.ndarray,
    height_mode: str,
    min_z: float,
    max_z: float,
    obstacle_height: float,
    max_obstacle_height: float,
    ground_resolution: float,
    ground_percentile: float,
    pmf_cell_size: float,
    pmf_max_window_size: int,
    pmf_slope: float,
    pmf_initial_distance: float,
    pmf_max_distance: float,
) -> tuple[np.ndarray, dict[str, object]]:
    if height_mode == "absolute":
        mask = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
        return points[mask], {
            "mode": "absolute",
            "min_z": min_z,
            "max_z": max_z,
        }
    if height_mode == "pmf":
        return classify_obstacle_points_pmf(
            points,
            obstacle_height,
            max_obstacle_height,
            pmf_cell_size,
            pmf_max_window_size,
            pmf_slope,
            pmf_initial_distance,
            pmf_max_distance,
            ground_percentile,
        )

    ground_image, _, flat, ground_cell_count = _ground_percentile_grid(
        points,
        ground_resolution,
        ground_percentile,
    )
    ground_z = ground_image.ravel()[flat]
    height_above_ground = points[:, 2] - ground_z
    mask = (
        (height_above_ground >= obstacle_height)
        & (height_above_ground <= max_obstacle_height)
        & np.isfinite(height_above_ground)
    )
    return points[mask], {
        "mode": "local_ground",
        "obstacle_height": obstacle_height,
        "max_obstacle_height": max_obstacle_height,
        "ground_resolution": ground_resolution,
        "ground_percentile": ground_percentile,
        "ground_cells": ground_cell_count,
    }


def choose_free_point_from_component(
    free_component: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    z: float,
) -> tuple[float, float, float]:
    padded = cv2.copyMakeBorder(
        free_component,
        1,
        1,
        1,
        1,
        cv2.BORDER_CONSTANT,
        value=0,
    )
    distance = cv2.distanceTransform(padded, cv2.DIST_L2, 5)[1:-1, 1:-1]
    row, col = np.unravel_index(int(np.argmax(distance)), distance.shape)
    return (
        float(origin[0] + (col + 0.5) * resolution),
        float(origin[1] + (row + 0.5) * resolution),
        z,
    )


def choose_free_point(
    occupancy: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    z: float,
) -> tuple[float, float, float]:
    """Choose the maximum-clearance point in largest free-space component."""
    free_component, _ = select_free_space_component(
        occupancy,
        origin,
        resolution,
    )
    return choose_free_point_from_component(
        free_component,
        origin,
        resolution,
        z,
    )
