"""Output paths and serialization for offline map preparation."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, TextIO

import numpy as np

from map_tools.visibility_graph import VisibilityNode


@dataclass(frozen=True)
class AlignmentOutputPaths:
    pointcloud: Path
    transform: Path
    debug_plot: Path


@dataclass(frozen=True)
class FarOutputPaths:
    boundary: Path
    trajectory: Path
    visibility_graph: Path
    stats: Path
    preview: Path


def alignment_output_paths(
    output_dir: Path,
    name: str | None = None,
) -> AlignmentOutputPaths:
    if name is None:
        return AlignmentOutputPaths(
            output_dir / "transformed.pcd",
            output_dir / "estimated_transform.json",
            output_dir / "transform_debug.png",
        )
    return AlignmentOutputPaths(
        output_dir / f"{name}_transformed.pcd",
        output_dir / f"{name}_transform.json",
        output_dir / f"{name}_transform_debug.png",
    )


def far_output_paths(output_dir: Path, name: str) -> FarOutputPaths:
    return FarOutputPaths(
        output_dir / f"{name}_boundary.ply",
        output_dir / f"{name}_trajectory.txt",
        output_dir / f"{name}.vgh",
        output_dir / f"{name}_boundary_stats.json",
        output_dir / f"{name}_boundary_preview.png",
    )


def numpy_to_list(value: object) -> object:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: numpy_to_list(item) for key, item in value.items()}
    if isinstance(value, list):
        return [numpy_to_list(item) for item in value]
    return value


def read_trajectory_xy(path: Path) -> np.ndarray:
    text = path.read_text(encoding="utf-8").strip()
    if not text:
        raise ValueError(f"{path} is empty")

    first_line = text.splitlines()[0]
    delimiter = "," if "," in first_line else None
    first_values = (
        first_line.split(",") if delimiter == "," else first_line.split()
    )
    try:
        for value in first_values:
            float(value)
    except ValueError:
        has_header = True
    else:
        has_header = False

    if not has_header:
        data = np.loadtxt(path, delimiter=delimiter, dtype=np.float64)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        if data.shape[1] < 2:
            raise ValueError(f"{path} must contain at least x and y columns")
        trajectory = data[:, :2]
        return trajectory[np.isfinite(trajectory).all(axis=1)]

    rows: list[tuple[float, float]] = []
    if delimiter == ",":
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            fieldnames = reader.fieldnames or []
            lower_names = {name.lower(): name for name in fieldnames}
            if "x" in lower_names and "y" in lower_names:
                x_key, y_key = lower_names["x"], lower_names["y"]
            elif "position_x" in lower_names and "position_y" in lower_names:
                x_key = lower_names["position_x"]
                y_key = lower_names["position_y"]
            else:
                raise ValueError(
                    f"{path} must contain x/y or position_x/position_y columns"
                )
            rows.extend(
                (float(row[x_key]), float(row[y_key])) for row in reader
            )
    else:
        lines = text.splitlines()
        fieldnames = lines[0].split()
        lower_names = {
            name.lower(): index for index, name in enumerate(fieldnames)
        }
        if "x" in lower_names and "y" in lower_names:
            x_index, y_index = lower_names["x"], lower_names["y"]
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
    return trajectory[np.isfinite(trajectory).all(axis=1)]


def _write_vgh_ids(handle: TextIO, identifiers: Iterable[int]) -> None:
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
                *node.position,
                *node.surface_directions[0],
                *node.surface_directions[1],
            )
            handle.write(f"{node.node_id} {int(node.free_direction)} ")
            for value in values:
                handle.write(f"{float(value):.6f} ")
            handle.write("1 0 0 1 ")
            for identifiers in (
                node.connections,
                node.connections,
                node.contour_connections,
            ):
                _write_vgh_ids(handle, identifiers)
                handle.write("| ")
            handle.write("\n")
    return len(nodes)


def write_boundary_ply(
    path: Path,
    polygons: Iterable[np.ndarray],
    z: float,
    has_outer_boundary: bool = True,
) -> int:
    polygon_list = list(polygons)
    vertex_count = sum(len(polygon) for polygon in polygon_list)
    first_polygon_index = 0 if has_outer_boundary else 1
    with path.open("w", encoding="utf-8") as handle:
        handle.write("ply\n")
        handle.write("format ascii 1.0\n")
        handle.write(f"element vertex {vertex_count}\n")
        handle.write("property float x\n")
        handle.write("property float y\n")
        handle.write("property float z\n")
        handle.write("property float poly_index\n")
        handle.write("end_header\n")
        for polygon_index, polygon in enumerate(
            polygon_list,
            start=first_polygon_index,
        ):
            for x, y in polygon:
                handle.write(
                    f"{float(x):.6f}\t{float(y):.6f}\t"
                    f"{z:.6f}\t{polygon_index}\n"
                )
    return vertex_count


def write_trajectory(
    path: Path,
    free_point: tuple[float, float, float],
) -> None:
    x, y, z = free_point
    path.write_text(
        f"{x:.6f} {y:.6f} {z:.6f} 0 0 0 0\n",
        encoding="utf-8",
    )


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
    import cv2

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
        col = int(
            round((float(point[0]) - origin[0]) / resolution * scale)
        )
        row = int(
            round((float(point[1]) - origin[1]) / resolution * scale)
        )
        return col, row

    thickness = max(1, scale)
    for index, polygon in enumerate(polygons):
        points = np.array(
            [to_px(point) for point in polygon],
            dtype=np.int32,
        ).reshape(-1, 1, 2)
        color = (
            (0, 180, 0)
            if has_outer_boundary and index == 0
            else (0, 0, 220)
        )
        cv2.polylines(
            preview,
            [points],
            True,
            color,
            thickness,
            cv2.LINE_AA,
        )

    free_position = np.array(free_point[:2], dtype=np.float32)
    cv2.circle(
        preview,
        to_px(free_position),
        max(4, 4 * scale),
        (255, 0, 0),
        -1,
        cv2.LINE_AA,
    )
    if clear_trajectory_xy is not None and len(clear_trajectory_xy) > 0:
        points = np.array(
            [to_px(point) for point in clear_trajectory_xy],
            dtype=np.int32,
        ).reshape(-1, 1, 2)
        cv2.polylines(
            preview,
            [points],
            False,
            (255, 0, 255),
            max(1, scale),
            cv2.LINE_AA,
        )
    cv2.imwrite(str(path), cv2.flip(preview, 0))
