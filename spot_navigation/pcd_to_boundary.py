#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from enum import IntEnum
from operator import itemgetter
from pathlib import Path
from typing import Iterable, TextIO, cast

import cv2
import numpy as np


def _pcd_dtype(field_sizes: list[int], field_types: list[str]) -> list[tuple[str, str]]:
    type_map = {
        ("F", 4): "<f4",
        ("F", 8): "<f8",
        ("I", 1): "i1",
        ("I", 2): "<i2",
        ("I", 4): "<i4",
        ("I", 8): "<i8",
        ("U", 1): "u1",
        ("U", 2): "<u2",
        ("U", 4): "<u4",
        ("U", 8): "<u8",
    }
    dtype = []
    for idx, (size, field_type) in enumerate(zip(field_sizes, field_types)):
        key = (field_type.upper(), size)
        if key not in type_map:
            raise ValueError(f"Unsupported PCD field type/size: {field_type}{size}")
        dtype.append((f"f{idx}", type_map[key]))
    return dtype


def load_pcd_xyz(path: Path) -> np.ndarray:
    header: list[str] = []
    with path.open("rb") as handle:
        while True:
            line = handle.readline()
            if not line:
                raise ValueError(f"{path} ended before a DATA line")
            decoded = line.decode("ascii", errors="ignore").strip()
            header.append(decoded)
            if decoded.startswith("DATA"):
                data = handle.read()
                break

    meta: dict[str, list[str]] = {}
    for line in header:
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        meta[parts[0].upper()] = parts[1:]

    fields = meta.get("FIELDS")
    if not fields or not {"x", "y", "z"}.issubset(fields):
        raise ValueError(f"{path} must contain x, y, z fields")

    points = int(meta.get("POINTS", ["0"])[0])
    sizes = [int(v) for v in meta.get("SIZE", [])]
    types = meta.get("TYPE", [])
    counts = [int(v) for v in meta.get("COUNT", ["1"] * len(fields))]
    data_mode = meta.get("DATA", [""])[0].lower()

    if len(fields) != len(sizes) or len(fields) != len(types):
        raise ValueError("Malformed PCD header: FIELDS/SIZE/TYPE lengths differ")
    if any(count != 1 for count in counts):
        raise ValueError("This extractor supports scalar PCD fields only")

    xyz_indices = [fields.index(axis) for axis in ("x", "y", "z")]

    if data_mode == "binary":
        dtype = np.dtype(_pcd_dtype(sizes, types))
        cloud = np.frombuffer(data, dtype=dtype, count=points)
        xyz = np.column_stack([cloud[f"f{idx}"] for idx in xyz_indices])
    elif data_mode == "ascii":
        table = np.loadtxt(data.splitlines(), dtype=np.float32)
        if table.ndim == 1:
            table = table.reshape(1, -1)
        xyz = table[:, xyz_indices]
    else:
        raise ValueError(f"Unsupported PCD DATA mode: {data_mode}")

    xyz = np.asarray(xyz, dtype=np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def shoelace_area(poly: np.ndarray) -> float:
    if len(poly) < 3:
        return 0.0
    x = poly[:, 0]
    y = poly[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


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
    filled[invalid_rows, invalid_cols] = values[valid_rows[nearest], valid_cols[nearest]]
    return filled


def read_trajectory_xy(path: Path) -> np.ndarray:
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        raise ValueError(f"{path} is empty")

    first_line = text.splitlines()[0]
    delimiter = "," if "," in first_line else None
    first_values = first_line.split(",") if delimiter == "," else first_line.split()
    try:
        for value in first_values:
            float(value)
    except ValueError:
        has_header = True
    else:
        has_header = False
    if has_header:
        rows: list[tuple[float, float]] = []
        if delimiter == ",":
            with path.open("r", encoding="utf-8", newline="") as handle:
                reader = csv.DictReader(handle)
                fieldnames = reader.fieldnames or []
                lower_names = {name.lower(): name for name in fieldnames}
                if "x" in lower_names and "y" in lower_names:
                    x_key = lower_names["x"]
                    y_key = lower_names["y"]
                elif "position_x" in lower_names and "position_y" in lower_names:
                    x_key = lower_names["position_x"]
                    y_key = lower_names["position_y"]
                else:
                    raise ValueError(
                        f"{path} must contain x/y or position_x/position_y columns"
                    )
                for row in reader:
                    rows.append((float(row[x_key]), float(row[y_key])))
        else:
            lines = text.splitlines()
            fieldnames = lines[0].split()
            lower_names = {name.lower(): index for index, name in enumerate(fieldnames)}
            if "x" in lower_names and "y" in lower_names:
                x_index = lower_names["x"]
                y_index = lower_names["y"]
            elif "position_x" in lower_names and "position_y" in lower_names:
                x_index = lower_names["position_x"]
                y_index = lower_names["position_y"]
            else:
                raise ValueError(
                    f"{path} must contain x/y or position_x/position_y columns"
                )
            for line in lines[1:]:
                values = line.split()
                rows.append((float(values[x_index]), float(values[y_index])))
        if not rows:
            raise ValueError(f"{path} has no trajectory rows")
        trajectory = np.asarray(rows, dtype=np.float64)
    else:
        data = np.loadtxt(path, delimiter=delimiter, dtype=np.float64)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[1] < 2:
            raise ValueError(f"{path} must contain at least x and y columns")
        trajectory = data[:, :2]

    return trajectory[np.isfinite(trajectory).all(axis=1)]


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
        cv2.circle(mask, (int(col), int(row)), radius_cells, 255, -1, cv2.LINE_8)

    return mask, {
        "points_total": int(len(indices)),
        "points_in_bounds": int(np.count_nonzero(in_bounds)),
        "clear_radius": clear_radius,
        "clear_cells": int(np.count_nonzero(mask)),
    }


def world_from_contour(contour: np.ndarray, origin: np.ndarray, resolution: float) -> np.ndarray:
    pts = contour.reshape(-1, 2).astype(np.float32)
    world = np.empty_like(pts)
    world[:, 0] = origin[0] + (pts[:, 0] + 0.5) * resolution
    world[:, 1] = origin[1] + (pts[:, 1] + 0.5) * resolution
    return world


def simplify_contour(
    contour: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    epsilon_m: float,
    max_vertices: int,
) -> np.ndarray:
    epsilon_px = max(epsilon_m / resolution, 0.5)
    approx = cv2.approxPolyDP(contour, epsilon_px, True)
    while len(approx) > max_vertices:
        epsilon_px *= 1.35
        approx = cv2.approxPolyDP(contour, epsilon_px, True)
    return world_from_contour(approx, origin, resolution)


def select_free_space_component(
    occupancy: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    requested_free_point_xy: np.ndarray | None = None,
) -> tuple[np.ndarray, dict[str, object]]:
    """Select requested or largest connected free-space component."""

    free_mask = np.where(occupancy == 0, 1, 0).astype(np.uint8)
    component_count, labels, component_stats, _ = (
        cv2.connectedComponentsWithStats(free_mask, connectivity=8)
    )
    if component_count <= 1:
        raise ValueError("Occupancy grid contains no free-space component")

    selection = "largest"
    if requested_free_point_xy is not None:
        requested = np.asarray(requested_free_point_xy, dtype=np.float64)
        if requested.shape != (2,) or not np.isfinite(requested).all():
            raise ValueError("requested free point must contain two finite XY values")
        index = np.floor((requested - origin) / resolution).astype(np.int32)
        row = int(index[1])
        col = int(index[0])
        if not (0 <= row < occupancy.shape[0] and 0 <= col < occupancy.shape[1]):
            raise ValueError("requested free point lies outside the occupancy grid")
        selected_label = int(labels[row, col])
        if selected_label == 0:
            raise ValueError("requested free point lies on an occupied cell")
        selection = "requested"
    else:
        selected_label = 1
        selected_area = int(component_stats[1, cv2.CC_STAT_AREA])
        for label in range(2, component_count):
            area = int(component_stats[label, cv2.CC_STAT_AREA])
            if area > selected_area:
                selected_label = label
                selected_area = area

    x = int(component_stats[selected_label, cv2.CC_STAT_LEFT])
    y = int(component_stats[selected_label, cv2.CC_STAT_TOP])
    width = int(component_stats[selected_label, cv2.CC_STAT_WIDTH])
    height = int(component_stats[selected_label, cv2.CC_STAT_HEIGHT])
    selected_cells = int(component_stats[selected_label, cv2.CC_STAT_AREA])
    touches_map_edge = (
        x == 0
        or y == 0
        or x + width == occupancy.shape[1]
        or y + height == occupancy.shape[0]
    )
    component = np.where(labels == selected_label, 255, 0).astype(np.uint8)
    stats = {
        "components": component_count - 1,
        "selection": selection,
        "selected_label": selected_label,
        "selected_cells": selected_cells,
        "touches_map_edge": touches_map_edge,
    }
    return component, stats


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
) -> tuple[list[np.ndarray], dict[str, object], np.ndarray, np.ndarray, tuple[int, int]]:
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
        occupancy = cv2.morphologyEx(occupancy, cv2.MORPH_CLOSE, close_kernel)

    inflate_kernel = make_kernel(inflate_radius, resolution)
    if inflate_kernel is not None:
        occupancy = cv2.dilate(occupancy, inflate_kernel)

    clear_stats = None
    if clear_trajectory_xy is not None:
        clear_mask, clear_stats = rasterize_clear_trajectory(
            (int(occupancy.shape[0]), int(occupancy.shape[1])),
            origin,
            resolution,
            clear_trajectory_xy,
            clear_radius,
        )
        occupied_before_clear = occupancy > 0
        clear_cells = clear_mask > 0
        clear_stats["occupied_cells_removed"] = int(np.count_nonzero(occupied_before_clear & clear_cells))
        occupancy[clear_cells] = 0

    free_component, free_space_stats = select_free_space_component(
        occupancy,
        origin,
        resolution,
        requested_free_point_xy,
    )
    contours, hierarchy = cv2.findContours(
        free_component.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE
    )
    outer_records: list[tuple[float, np.ndarray]] = []
    obstacle_records: list[tuple[float, np.ndarray]] = []
    for contour_index, contour in enumerate(contours):
        area = abs(cv2.contourArea(contour)) * resolution * resolution
        if area < min_area:
            continue
        polygon = simplify_contour(contour, origin, resolution, simplify_epsilon, max_vertices)
        area = abs(shoelace_area(polygon))
        if len(polygon) < 3 or area < min_area:
            continue
        parent = -1 if hierarchy is None else int(hierarchy[0, contour_index, 3])
        if parent == -1:
            outer_records.append((area, polygon))
        else:
            obstacle_records.append((area, polygon))

    outer_records.sort(key=itemgetter(0), reverse=True)
    obstacle_records.sort(key=itemgetter(0), reverse=True)
    outer_polygons = [polygon for _, polygon in outer_records]
    obstacle_polygons = [polygon for _, polygon in obstacle_records[:max_polygons]]
    polygons = (outer_polygons if add_outer_boundary else []) + obstacle_polygons

    stats = {
        "grid": {"rows": rows, "cols": cols, "resolution": resolution},
        "origin": origin.tolist(),
        "points_projected": int(len(indices)),
        "occupied_cells": int(np.count_nonzero(occupancy)),
        "free_space": free_space_stats,
        "outer_boundary_polygons": len(outer_polygons) if add_outer_boundary else 0,
        "candidate_obstacle_polygons": len(obstacle_records),
        "obstacle_polygons": len(obstacle_polygons),
        "discarded_obstacle_polygons": max(
            0, len(obstacle_records) - len(obstacle_polygons)
        ),
        "total_polygons": len(polygons),
        "total_vertices": int(sum(len(poly) for poly in polygons)),
        "outer_boundary_areas_m2": [
            round(float(area), 3) for area, _ in outer_records
        ],
        "obstacle_areas_m2": [
            round(float(area), 3) for area, _ in obstacle_records[:max_polygons]
        ],
    }
    if clear_stats is not None:
        stats["trajectory_clear"] = clear_stats
    return polygons, stats, occupancy, origin, (rows, cols)


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
    min_xy = points[:, :2].min(axis=0)
    max_xy = points[:, :2].max(axis=0)
    cols = int(math.ceil((max_xy[0] - min_xy[0]) / cell_size)) + 1
    rows = int(math.ceil((max_xy[1] - min_xy[1]) / cell_size)) + 1
    indices = np.floor((points[:, :2] - min_xy) / cell_size).astype(np.int32)
    flat = indices[:, 1] * cols + indices[:, 0]

    ground_grid = np.full(rows * cols, np.nan, dtype=np.float32)
    order = np.argsort(flat)
    sorted_flat = flat[order]
    starts = np.r_[0, np.flatnonzero(np.diff(sorted_flat)) + 1]
    ends = np.r_[starts[1:], len(order)]
    for start, end in zip(starts, ends):
        cell = sorted_flat[start]
        z_values = points[order[start:end], 2]
        ground_grid[cell] = np.percentile(z_values, ground_percentile)

    ground_image = ground_grid.reshape(rows, cols)
    observed_cells = np.isfinite(ground_image)
    ground_estimate = fill_nan_nearest(ground_image)
    non_ground_cells = np.zeros((rows, cols), dtype=bool)
    odd_max_window_size = max(3, int(max_window_size) | 1)
    window_records = []
    for window_size in range(3, odd_max_window_size + 1, 2):
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (window_size, window_size))
        opened_ground = cv2.morphologyEx(ground_estimate, cv2.MORPH_OPEN, kernel)
        radius_m = (window_size - 1) * 0.5 * cell_size
        threshold = min(max_distance, initial_distance + slope * radius_m)
        residual = ground_estimate - opened_ground
        newly_non_ground = (residual > threshold) & observed_cells
        new_cell_count = int(np.count_nonzero(newly_non_ground & ~non_ground_cells))
        non_ground_cells |= newly_non_ground
        ground_estimate = np.where(newly_non_ground, opened_ground, ground_estimate)
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
    stats = {
        "mode": "pmf_opening",
        "obstacle_height": obstacle_height,
        "max_obstacle_height": max_obstacle_height,
        "pmf_cell_size": cell_size,
        "pmf_max_window_size": odd_max_window_size,
        "pmf_slope": slope,
        "pmf_initial_distance": initial_distance,
        "pmf_max_distance": max_distance,
        "ground_percentile": ground_percentile,
        "ground_cells": int(np.count_nonzero(np.isfinite(ground_grid))),
        "pmf_non_ground_cells": int(np.count_nonzero(non_ground_cells & observed_cells)),
        "windows": window_records,
    }
    return points[mask], stats


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

    min_xy = points[:, :2].min(axis=0)
    max_xy = points[:, :2].max(axis=0)
    cols = int(math.ceil((max_xy[0] - min_xy[0]) / ground_resolution)) + 1
    rows = int(math.ceil((max_xy[1] - min_xy[1]) / ground_resolution)) + 1
    indices = np.floor((points[:, :2] - min_xy) / ground_resolution).astype(np.int32)
    flat = indices[:, 1] * cols + indices[:, 0]

    ground = np.full(rows * cols, np.nan, dtype=np.float32)
    order = np.argsort(flat)
    sorted_flat = flat[order]
    starts = np.r_[0, np.flatnonzero(np.diff(sorted_flat)) + 1]
    ends = np.r_[starts[1:], len(order)]
    for start, end in zip(starts, ends):
        cell = sorted_flat[start]
        z_values = points[order[start:end], 2]
        ground[cell] = np.percentile(z_values, ground_percentile)

    ground_z = ground[flat]
    height_above_ground = points[:, 2] - ground_z
    mask = (
        (height_above_ground >= obstacle_height)
        & (height_above_ground <= max_obstacle_height)
        & np.isfinite(height_above_ground)
    )
    stats = {
        "mode": "local_ground",
        "obstacle_height": obstacle_height,
        "max_obstacle_height": max_obstacle_height,
        "ground_resolution": ground_resolution,
        "ground_percentile": ground_percentile,
        "ground_cells": int(np.count_nonzero(np.isfinite(ground))),
    }
    return points[mask], stats


def choose_free_point(
    occupancy: np.ndarray,
    origin: np.ndarray,
    resolution: float,
    z: float,
) -> tuple[float, float, float]:
    """Choose maximum-clearance point in largest free-space component."""

    free_component, _ = select_free_space_component(
        occupancy,
        origin,
        resolution,
    )
    padded_component = cv2.copyMakeBorder(
        free_component,
        1,
        1,
        1,
        1,
        cv2.BORDER_CONSTANT,
        value=0,
    )
    padded_distance = cv2.distanceTransform(
        padded_component, cv2.DIST_L2, 5
    )
    dist = padded_distance[1:-1, 1:-1]
    row, col = np.unravel_index(int(np.argmax(dist)), dist.shape)
    return (
        float(origin[0] + (col + 0.5) * resolution),
        float(origin[1] + (row + 0.5) * resolution),
        z,
    )


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
    polygon_connections: list[int]
    contour_connections: list[int]
    trajectory_connections: list[int]


def _normalize_xy(vector: np.ndarray) -> np.ndarray:
    norm = math.hypot(float(vector[0]), float(vector[1]))
    if norm <= 1e-7:
        return np.zeros(3, dtype=np.float32)
    return np.asarray(
        [float(vector[0]) / norm, float(vector[1]) / norm, 0.0],
        dtype=np.float32,
    )


def point_inside_polygon(polygon: np.ndarray, point: np.ndarray) -> bool:
    """Return whether point lies inside polygon using FAR's ray-casting rule."""

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
        crosses_y = (
            (current_y <= point_y < previous_y)
            or (previous_y <= point_y < current_y)
        )
        if crosses_y:
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
        first_start, second_start, first_end
    ):
        return True
    if second_orientation == 0 and _point_on_segment(
        first_start, second_end, first_end
    ):
        return True
    if third_orientation == 0 and _point_on_segment(
        second_start, first_start, second_end
    ):
        return True
    if fourth_orientation == 0 and _point_on_segment(
        second_start, first_end, second_end
    ):
        return True
    return False


def _surface_topology_direction(
    surface_directions: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, bool]:
    topology_direction = surface_directions[0] + surface_directions[1]
    if math.hypot(
        float(topology_direction[0]), float(topology_direction[1])
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
    if second.node_id not in first.polygon_connections:
        first.polygon_connections.append(second.node_id)
        second.polygon_connections.append(first.node_id)
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
            second.position - first.position, first.surface_directions
        )
    ):
        return False
    if (
        second.free_direction != FreeDirection.PILLAR
        and not _is_outside_reduced_directions(
            first.position - second.position, second.surface_directions
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
    """Build FAR-compatible visibility nodes directly from boundary polygons."""

    free_position = np.asarray(free_point, dtype=np.float32)
    if free_position.shape != (3,) or not np.isfinite(free_position).all():
        raise ValueError("free_point must contain three finite values")
    if not math.isfinite(boundary_z):
        raise ValueError("boundary_z must be finite")
    if not math.isfinite(height_tolerance) or height_tolerance < 0.0:
        raise ValueError("height_tolerance must be finite and non-negative")

    # Match values previously passed through six-decimal PLY and trajectory files.
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
                f"polygon {polygon_index} must contain three finite XY vertices"
            )
        polygon_array = np.round(polygon_array, decimals=6)

        vertices = np.column_stack(
            (
                polygon_array[:, :2],
                np.full(
                    len(polygon_array), graph_boundary_z, dtype=np.float32
                ),
            )
        ).astype(np.float32, copy=False)
        graph_polygons.append(vertices)
        polygon_nodes: list[VisibilityNode] = []
        for vertex in vertices:
            node = VisibilityNode(
                node_id=len(graph) + len(polygon_nodes),
                position=vertex.copy(),
                free_direction=FreeDirection.UNKNOWN,
                surface_directions=(
                    np.zeros(3, dtype=np.float32),
                    np.zeros(3, dtype=np.float32),
                ),
                connections=[],
                polygon_connections=[],
                contour_connections=[],
                trajectory_connections=[],
            )
            polygon_nodes.append(node)

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
                evaluation_point = node.position + topology_direction * 0.25
                evaluation_inside = point_inside_polygon(
                    vertices, evaluation_point
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
                first, second, graph_polygons, height_tolerance
            ):
                _connect_visibility(first, second)
    return graph


def _write_vgh_ids(handle: TextIO, identifiers: list[int]) -> None:
    for identifier in identifiers:
        handle.write(f"{identifier} ")


def write_vgh(path: Path, graph: Iterable[VisibilityNode]) -> int:
    """Write graph in boundary_handler's text VGH format."""

    nodes = list(graph)
    if not nodes:
        raise ValueError("Cannot write an empty visibility graph")

    with path.open("w", encoding="utf-8") as handle:
        for node in nodes:
            values = (
                node.position[0],
                node.position[1],
                node.position[2],
                node.surface_directions[0][0],
                node.surface_directions[0][1],
                node.surface_directions[0][2],
                node.surface_directions[1][0],
                node.surface_directions[1][1],
                node.surface_directions[1][2],
            )
            handle.write(f"{node.node_id} {int(node.free_direction)} ")
            for value in values:
                handle.write(f"{float(value):.6f} ")
            handle.write("1 0 0 1 ")
            _write_vgh_ids(handle, node.connections)
            handle.write("| ")
            _write_vgh_ids(handle, node.polygon_connections)
            handle.write("| ")
            _write_vgh_ids(handle, node.contour_connections)
            handle.write("| ")
            _write_vgh_ids(handle, node.trajectory_connections)
            handle.write("\n")
    return len(nodes)


def write_boundary_ply(
    path: Path,
    polygons: Iterable[np.ndarray],
    z: float,
    has_outer_boundary: bool = True,
) -> int:
    rows: list[tuple[float, float, float, int]] = []
    first_polygon_index = 0 if has_outer_boundary else 1
    for poly_index, polygon in enumerate(polygons, start=first_polygon_index):
        for x, y in polygon:
            rows.append((float(x), float(y), z, poly_index))

    with path.open("w", encoding="utf-8") as handle:
        handle.write("ply\n")
        handle.write("format ascii 1.0\n")
        handle.write(f"element vertex {len(rows)}\n")
        handle.write("property float x\n")
        handle.write("property float y\n")
        handle.write("property float z\n")
        handle.write("property float poly_index\n")
        handle.write("end_header\n")
        for x, y, z_value, poly_index in rows:
            handle.write(f"{x:.6f}\t{y:.6f}\t{z_value:.6f}\t{poly_index}\n")
    return len(rows)


def write_trajectory(path: Path, free_point: tuple[float, float, float]) -> None:
    x, y, z = free_point
    with path.open("w", encoding="utf-8") as handle:
        handle.write(f"{x:.6f} {y:.6f} {z:.6f} 0 0 0 0\n")


def write_preview(
    path: Path,
    occupancy: np.ndarray,
    polygons: list[np.ndarray],
    origin: np.ndarray,
    resolution: float,
    free_point: tuple[float, float, float],
    scale: int,
    clear_trajectory_xy: np.ndarray | None = None,
    has_outer_boundary: bool = True,
) -> None:
    scale = max(int(scale), 1)
    preview = np.full((*occupancy.shape, 3), 255, dtype=np.uint8)
    preview[occupancy > 0] = (70, 70, 70)
    if scale > 1:
        preview = cv2.resize(
            preview,
            (preview.shape[1] * scale, preview.shape[0] * scale),
            interpolation=cv2.INTER_NEAREST,
        )

    def to_px(point: np.ndarray) -> tuple[int, int]:
        col = int(round((float(point[0]) - origin[0]) / resolution * scale))
        row = int(round((float(point[1]) - origin[1]) / resolution * scale))
        return col, row

    thickness = max(1, scale)
    for idx, polygon in enumerate(polygons):
        pts = np.array([to_px(point) for point in polygon], dtype=np.int32).reshape(-1, 1, 2)
        color = (0, 180, 0) if has_outer_boundary and idx == 0 else (0, 0, 220)
        cv2.polylines(preview, [pts], True, color, thickness, cv2.LINE_AA)

    fp = np.array(free_point[:2], dtype=np.float32)
    cv2.circle(preview, to_px(fp), max(4, 4 * scale), (255, 0, 0), -1, cv2.LINE_AA)
    if clear_trajectory_xy is not None and len(clear_trajectory_xy) > 0:
        pts = np.array([to_px(point) for point in clear_trajectory_xy], dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(preview, [pts], False, (255, 0, 255), max(1, scale), cv2.LINE_AA)
    cv2.imwrite(str(path), cv2.flip(preview, 0))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert a PCD map into FAR boundary, preview, and VGH files."
    )
    parser.add_argument("pcd", type=Path, help="Input PCD file")
    parser.add_argument("--output-dir", type=Path, help="Output directory")
    parser.add_argument("--name", help="Output stem; defaults to the PCD stem")
    parser.add_argument("--resolution", type=float, default=0.15, help="2D grid resolution in meters")
    parser.add_argument(
        "--height-mode",
        choices=("local", "absolute", "pmf"),
        default="local",
        help="Classify obstacles by local height-above-ground, absolute z, or PMF-style opening",
    )
    parser.add_argument("--min-z", type=float, default=0.2, help="Minimum point z for absolute mode")
    parser.add_argument("--max-z", type=float, default=2.0, help="Maximum point z for absolute mode")
    parser.add_argument(
        "--obstacle-height",
        type=float,
        default=0.35,
        help="Minimum height above local/PMF ground to project as obstacle",
    )
    parser.add_argument(
        "--max-obstacle-height",
        type=float,
        default=2.5,
        help="Maximum height above local/PMF ground to project as obstacle",
    )
    parser.add_argument(
        "--ground-resolution",
        type=float,
        default=0.75,
        help="XY cell size used to estimate local floor height",
    )
    parser.add_argument(
        "--ground-percentile",
        type=float,
        default=15.0,
        help="Z percentile used as local/PMF floor sample in each ground cell",
    )
    parser.add_argument(
        "--pmf-cell-size",
        type=float,
        default=0.25,
        help="XY cell size for PMF-style ground opening in meters",
    )
    parser.add_argument(
        "--pmf-max-window-size",
        type=int,
        default=7,
        help="Maximum odd PMF opening window size in cells",
    )
    parser.add_argument(
        "--pmf-slope",
        type=float,
        default=0.5,
        help="PMF slope term used in progressive height thresholding",
    )
    parser.add_argument(
        "--pmf-initial-distance",
        type=float,
        default=0.2,
        help="Initial PMF ground distance threshold in meters",
    )
    parser.add_argument(
        "--pmf-max-distance",
        type=float,
        default=0.6,
        help="Maximum PMF ground distance threshold in meters",
    )
    parser.add_argument("--boundary-z", type=float, default=0.75, help="Z value written to boundary vertices")
    parser.add_argument("--padding", type=float, default=0.5, help="Outer map padding in meters")
    parser.add_argument("--close-radius", type=float, default=0.25, help="Morphological close radius in meters")
    parser.add_argument("--inflate-radius", type=float, default=0.10, help="Obstacle inflation radius in meters")
    parser.add_argument("--simplify", type=float, default=0.25, help="Contour simplification in meters")
    parser.add_argument("--min-area", type=float, default=0.25, help="Minimum obstacle polygon area in m^2")
    parser.add_argument(
        "--max-polygons",
        type=int,
        default=80,
        help="Maximum obstacle polygons retained by area; raise for large maps",
    )
    parser.add_argument("--max-vertices", type=int, default=80, help="Maximum vertices per obstacle polygon")
    parser.add_argument(
        "--free-point",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="Known traversable point; selects free-space component and VGH side",
    )
    parser.add_argument(
        "--clear-trajectory-csv",
        type=Path,
        help="CSV/whitespace file containing x,y trajectory points to clear from obstacle occupancy",
    )
    parser.add_argument(
        "--clear-radius",
        type=float,
        default=0.55,
        help="Radius in meters to clear around --clear-trajectory-csv path",
    )
    parser.add_argument(
        "--no-outer-boundary",
        action="store_true",
        help="Do not include selected free-space outer boundary",
    )
    parser.add_argument("--no-preview", action="store_true", help="Skip PNG preview generation")
    parser.add_argument(
        "--preview-scale",
        type=int,
        default=6,
        help="Integer scale factor for preview PNG resolution",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    pcd_path = args.pcd.resolve()
    output_dir = (args.output_dir or pcd_path.parent).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    name = args.name or pcd_path.stem

    points = load_pcd_xyz(pcd_path)
    projected, height_stats = classify_obstacle_points(
        points,
        args.height_mode,
        args.min_z,
        args.max_z,
        args.obstacle_height,
        args.max_obstacle_height,
        args.ground_resolution,
        args.ground_percentile,
        args.pmf_cell_size,
        args.pmf_max_window_size,
        args.pmf_slope,
        args.pmf_initial_distance,
        args.pmf_max_distance,
    )
    if len(projected) < 3:
        raise RuntimeError(
            f"Height filter kept only {len(projected)} points. Adjust height threshold options."
        )

    clear_trajectory_xy = None
    if args.clear_trajectory_csv is not None:
        clear_trajectory_xy = read_trajectory_xy(args.clear_trajectory_csv.resolve())
        if len(clear_trajectory_xy) == 0:
            raise RuntimeError(f"No finite x,y trajectory points in {args.clear_trajectory_csv}")

    requested_free_point = tuple(args.free_point) if args.free_point else None
    requested_free_point_xy = (
        np.asarray(requested_free_point[:2], dtype=np.float64)
        if requested_free_point is not None
        else None
    )
    polygons, stats, occupancy, origin, _ = extract_boundary_polygons(
        projected,
        points,
        args.resolution,
        args.padding,
        args.close_radius,
        args.inflate_radius,
        args.simplify,
        args.min_area,
        args.max_polygons,
        args.max_vertices,
        not args.no_outer_boundary,
        clear_trajectory_xy,
        args.clear_radius,
        requested_free_point_xy,
    )

    free_point = requested_free_point or choose_free_point(
        occupancy,
        origin,
        args.resolution,
        args.boundary_z,
    )
    stats["input_pcd"] = str(pcd_path)
    stats["height_filter"] = height_stats
    stats["points_total"] = int(len(points))
    stats["points_after_height_filter"] = int(len(projected))
    stats["free_point"] = [round(float(v), 6) for v in free_point]
    stats["preview_scale"] = None if args.no_preview else max(int(args.preview_scale), 1)

    boundary_path = output_dir / f"{name}_boundary.ply"
    trajectory_path = output_dir / f"{name}_trajectory.txt"
    vgh_path = output_dir / f"{name}.vgh"
    stats_path = output_dir / f"{name}_boundary_stats.json"
    preview_path = output_dir / f"{name}_boundary_preview.png"

    vertices = write_boundary_ply(
        boundary_path,
        polygons,
        args.boundary_z,
        has_outer_boundary=not args.no_outer_boundary,
    )
    write_trajectory(trajectory_path, free_point)
    graph = build_visibility_graph(polygons, free_point, args.boundary_z)
    graph_nodes = write_vgh(vgh_path, graph)
    graph_edges = sum(len(node.connections) for node in graph) // 2
    contour_edges = sum(len(node.contour_connections) for node in graph) // 2
    stats["written_vertices"] = vertices
    stats["visibility_graph"] = {
        "nodes": graph_nodes,
        "edges": graph_edges,
        "contour_edges": contour_edges,
    }
    stats_path.write_text(json.dumps(stats, indent=2) + "\n", encoding="utf-8")
    if not args.no_preview:
        write_preview(
            preview_path,
            occupancy,
            polygons,
            origin,
            args.resolution,
            free_point,
            args.preview_scale,
            clear_trajectory_xy,
            not args.no_outer_boundary,
        )

    print(f"Wrote {boundary_path}")
    print(f"Wrote {trajectory_path}")
    print(f"Wrote {vgh_path}")
    print(f"Wrote {stats_path}")
    if not args.no_preview:
        print(f"Wrote {preview_path}")
    discarded_polygons = cast(int, stats["discarded_obstacle_polygons"])
    if discarded_polygons > 0:
        print(
            f"Warning: discarded {discarded_polygons} smaller obstacle polygons; "
            "increase --max-polygons to retain more."
        )


if __name__ == "__main__":
    main()
