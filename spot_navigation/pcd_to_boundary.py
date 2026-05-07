#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Iterable

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

    contours, _ = cv2.findContours(occupancy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contour_records = []
    for contour in contours:
        area = abs(cv2.contourArea(contour)) * resolution * resolution
        if area < min_area:
            continue
        polygon = simplify_contour(contour, origin, resolution, simplify_epsilon, max_vertices)
        area = abs(shoelace_area(polygon))
        if len(polygon) < 3 or area < min_area:
            continue
        contour_records.append((area, polygon))

    contour_records.sort(key=lambda item: item[0], reverse=True)
    obstacle_polygons = [polygon for _, polygon in contour_records[:max_polygons]]

    polygons: list[np.ndarray] = []
    if add_outer_boundary:
        outer_min = origin
        outer_max = origin + np.array([cols * resolution, rows * resolution])
        polygons.append(
            np.array(
                [
                    [outer_max[0], outer_max[1]],
                    [outer_max[0], outer_min[1]],
                    [outer_min[0], outer_min[1]],
                    [outer_min[0], outer_max[1]],
                ],
                dtype=np.float32,
            )
        )
    polygons.extend(obstacle_polygons)

    stats = {
        "grid": {"rows": rows, "cols": cols, "resolution": resolution},
        "origin": origin.tolist(),
        "points_projected": int(len(indices)),
        "occupied_cells": int(np.count_nonzero(occupancy)),
        "obstacle_polygons": len(obstacle_polygons),
        "total_polygons": len(polygons),
        "total_vertices": int(sum(len(poly) for poly in polygons)),
        "obstacle_areas_m2": [round(float(area), 3) for area, _ in contour_records[:max_polygons]],
    }
    return polygons, stats, occupancy, origin, (rows, cols)


def classify_obstacle_points(
    points: np.ndarray,
    height_mode: str,
    min_z: float,
    max_z: float,
    obstacle_height: float,
    max_obstacle_height: float,
    ground_resolution: float,
    ground_percentile: float,
) -> tuple[np.ndarray, dict[str, object]]:
    if height_mode == "absolute":
        mask = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
        return points[mask], {
            "mode": "absolute",
            "min_z": min_z,
            "max_z": max_z,
        }

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
    free_mask = np.where(occupancy > 0, 0, 255).astype(np.uint8)
    if not np.any(free_mask):
        rows, cols = occupancy.shape
        return (
            float(origin[0] + cols * resolution * 0.5),
            float(origin[1] + rows * resolution * 0.5),
            z,
        )

    dist = cv2.distanceTransform(free_mask, cv2.DIST_L2, 5)
    row, col = np.unravel_index(int(np.argmax(dist)), dist.shape)
    return (
        float(origin[0] + (col + 0.5) * resolution),
        float(origin[1] + (row + 0.5) * resolution),
        z,
    )


def write_boundary_ply(path: Path, polygons: Iterable[np.ndarray], z: float) -> int:
    rows: list[tuple[float, float, float, int]] = []
    for poly_index, polygon in enumerate(polygons):
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
        color = (0, 180, 0) if idx == 0 else (0, 0, 220)
        cv2.polylines(preview, [pts], True, color, thickness, cv2.LINE_AA)

    fp = np.array(free_point[:2], dtype=np.float32)
    cv2.circle(preview, to_px(fp), max(4, 4 * scale), (255, 0, 0), -1, cv2.LINE_AA)
    cv2.imwrite(str(path), cv2.flip(preview, 0))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract FAR boundary_handler polygons from a PCD map."
    )
    parser.add_argument("pcd", type=Path, help="Input PCD file")
    parser.add_argument("--output-dir", type=Path, help="Output directory")
    parser.add_argument("--name", help="Output stem; defaults to the PCD stem")
    parser.add_argument("--resolution", type=float, default=0.15, help="2D grid resolution in meters")
    parser.add_argument(
        "--height-mode",
        choices=("local", "absolute"),
        default="local",
        help="Classify obstacles by local height-above-ground or absolute z",
    )
    parser.add_argument("--min-z", type=float, default=0.2, help="Minimum point z for absolute mode")
    parser.add_argument("--max-z", type=float, default=2.0, help="Maximum point z for absolute mode")
    parser.add_argument(
        "--obstacle-height",
        type=float,
        default=0.35,
        help="Minimum height above local ground to project as obstacle",
    )
    parser.add_argument(
        "--max-obstacle-height",
        type=float,
        default=2.5,
        help="Maximum height above local ground to project as obstacle",
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
        help="Z percentile used as local floor height in each ground cell",
    )
    parser.add_argument("--boundary-z", type=float, default=0.75, help="Z value written to boundary vertices")
    parser.add_argument("--padding", type=float, default=0.5, help="Outer map padding in meters")
    parser.add_argument("--close-radius", type=float, default=0.25, help="Morphological close radius in meters")
    parser.add_argument("--inflate-radius", type=float, default=0.10, help="Obstacle inflation radius in meters")
    parser.add_argument("--simplify", type=float, default=0.25, help="Contour simplification in meters")
    parser.add_argument("--min-area", type=float, default=0.25, help="Minimum obstacle polygon area in m^2")
    parser.add_argument("--max-polygons", type=int, default=80, help="Maximum obstacle polygons")
    parser.add_argument("--max-vertices", type=int, default=80, help="Maximum vertices per obstacle polygon")
    parser.add_argument("--free-point", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--no-outer-boundary", action="store_true", help="Do not add map bounding box polygon")
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
    )
    if len(projected) < 3:
        raise RuntimeError(
            f"Height filter kept only {len(projected)} points. Adjust height threshold options."
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
    )

    free_point = tuple(args.free_point) if args.free_point else choose_free_point(
        occupancy, origin, args.resolution, args.boundary_z
    )
    stats["input_pcd"] = str(pcd_path)
    stats["height_filter"] = height_stats
    stats["points_total"] = int(len(points))
    stats["points_after_height_filter"] = int(len(projected))
    stats["free_point"] = [round(float(v), 6) for v in free_point]
    stats["preview_scale"] = None if args.no_preview else max(int(args.preview_scale), 1)

    boundary_path = output_dir / f"{name}_boundary.ply"
    trajectory_path = output_dir / f"{name}_trajectory.txt"
    stats_path = output_dir / f"{name}_boundary_stats.json"
    preview_path = output_dir / f"{name}_boundary_preview.png"

    vertices = write_boundary_ply(boundary_path, polygons, args.boundary_z)
    write_trajectory(trajectory_path, free_point)
    stats["written_vertices"] = vertices
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
        )

    print(f"Wrote {boundary_path}")
    print(f"Wrote {trajectory_path}")
    print(f"Wrote {stats_path}")
    if not args.no_preview:
        print(f"Wrote {preview_path}")
    print(
        "Boundary and trajectory files are intermediate inputs for "
        "boundary_handler when regenerating a .vgh."
    )


if __name__ == "__main__":
    main()
