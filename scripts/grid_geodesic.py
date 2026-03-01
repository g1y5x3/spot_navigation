#!/usr/bin/env python3
"""
Grid-based geodesic distance computation for SPL evaluation.

Builds a 2D traversability grid from /terrain_cloud messages in a ROS 2 bag,
then computes shortest-path distance via A* on the grid.

The /terrain_cloud topic is published in base_link frame with PMF classification
encoded as intensity (0.0 = ground, 1.0 = obstacle). Points are transformed to
the map frame using /odometry_map poses.

Usage as standalone (test on a single bag):
    python3 grid_geodesic.py --bag /path/to/mine_nav1_r1 \\
        --start 5.8 -6.3 --goal -2.5 0.55 --plot

Usage as module:
    from grid_geodesic import build_grid_from_bag, TraversabilityGrid
    grid = build_grid_from_bag(bag_path, goal_timestamp=t0)
    dist, path = grid.astar([5.8, -6.3], [-2.5, 0.55])
"""

import argparse
import heapq
import math
import re
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import lzf
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg


# Custom message registration for visibility_graph_msg
_VG_MSG_DIR = Path(__file__).resolve().parent.parent.parent / (
    "far_planner/src/visibility_graph_msg/msg"
)


def _register_vg_msgs(typestore):
    """Register visibility_graph_msg/msg/{Node,Graph} with the typestore."""
    node_path = _VG_MSG_DIR / "Node.msg"
    graph_path = _VG_MSG_DIR / "Graph.msg"
    if not node_path.exists() or not graph_path.exists():
        return
    add_types = {}
    add_types.update(
        get_types_from_msg(node_path.read_text(), "visibility_graph_msg/msg/Node")
    )
    add_types.update(
        get_types_from_msg(graph_path.read_text(), "visibility_graph_msg/msg/Graph")
    )
    typestore.register(add_types)


# ---------------------------------------------------------------------------
# Bresenham line rasterization
# ---------------------------------------------------------------------------


def _bresenham(r0: int, c0: int, r1: int, c1: int) -> list[tuple[int, int]]:
    """Return list of (row, col) cells along a line from (r0,c0) to (r1,c1)."""
    cells = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r1 > r0 else -1
    sc = 1 if c1 > c0 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells


# Extract contour segments from /robot_vgraph in a bag
def extract_contour_from_bag(
    bag_path: Path,
    typestore=None,
) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """
    Extract contour_connects segments from the last /robot_vgraph message.

    Returns list of ((x1, y1), (x2, y2)) segments in map frame.
    """
    if typestore is None:
        typestore = get_typestore(Stores.ROS2_HUMBLE)
    _register_vg_msgs(typestore)

    with Reader(bag_path) as reader:
        connections = {c.topic: c for c in reader.connections}
        if "/robot_vgraph" not in connections:
            print(f"  WARNING: No /robot_vgraph in {bag_path.name}, skipping contour")
            return []

        conn = connections["/robot_vgraph"]
        last_raw = None
        for _, _, rawdata in reader.messages(connections=[conn]):
            last_raw = rawdata

    if last_raw is None:
        return []

    msg = typestore.deserialize_cdr(last_raw, conn.msgtype)
    node_map = {int(n.id): (n.position.x, n.position.y) for n in msg.nodes}

    segments = []
    seen: set[tuple[int, int]] = set()
    for n in msg.nodes:
        nid = int(n.id)
        for cid in n.contour_connects:
            cid = int(cid)
            if cid not in node_map:
                continue
            edge = (min(nid, cid), max(nid, cid))
            if edge in seen:
                continue
            seen.add(edge)
            segments.append((node_map[nid], node_map[cid]))

    print(f"  Extracted {len(segments)} contour segments from /robot_vgraph")
    return segments


# ---------------------------------------------------------------------------
# Quaternion → rotation matrix (avoid scipy dependency)
# ---------------------------------------------------------------------------


def quat_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    r = np.array(
        [
            [
                1 - 2 * (qy * qy + qz * qz),
                2 * (qx * qy - qz * qw),
                2 * (qx * qz + qy * qw),
            ],
            [
                2 * (qx * qy + qz * qw),
                1 - 2 * (qx * qx + qz * qz),
                2 * (qy * qz - qx * qw),
            ],
            [
                2 * (qx * qz - qy * qw),
                2 * (qy * qz + qx * qw),
                1 - 2 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float64,
    )
    return r


# ---------------------------------------------------------------------------
# TraversabilityGrid
# ---------------------------------------------------------------------------


@dataclass
class TraversabilityGrid:
    """2D traversability grid with world ↔ cell coordinate conversion and A* queries."""

    traversable: np.ndarray  # bool array, shape (rows, cols). True = free.
    resolution: float  # meters per cell
    x_min: float  # world X of left edge of column 0
    y_min: float  # world Y of bottom edge of row 0
    elevation: np.ndarray | None = (
        None  # float array, shape (rows, cols). NaN = non-traversable.
    )

    @property
    def rows(self) -> int:
        return self.traversable.shape[0]

    @property
    def cols(self) -> int:
        return self.traversable.shape[1]

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """Convert world (x, y) to grid (row, col). Row = Y axis, Col = X axis."""
        col = int((x - self.x_min) / self.resolution)
        row = int((y - self.y_min) / self.resolution)
        return row, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        """Convert grid (row, col) to world (x, y) at cell center."""
        x = self.x_min + (col + 0.5) * self.resolution
        y = self.y_min + (row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.rows and 0 <= col < self.cols

    def astar(
        self,
        start_xy: list[float] | tuple[float, float],
        goal_xy: list[float] | tuple[float, float],
    ) -> tuple[float, list[tuple[float, float]]]:
        """
        A* shortest path on the traversability grid (8-connected).

        Parameters
        ----------
        start_xy : (x, y) in world coordinates
        goal_xy  : (x, y) in world coordinates

        Returns
        -------
        distance : float — geodesic distance in meters (inf if no path)
        path     : list of (x, y) world coordinates along the path
        """
        sr, sc = self.world_to_cell(start_xy[0], start_xy[1])
        gr, gc = self.world_to_cell(goal_xy[0], goal_xy[1])

        # Clamp to grid bounds
        sr = max(0, min(sr, self.rows - 1))
        sc = max(0, min(sc, self.cols - 1))
        gr = max(0, min(gr, self.rows - 1))
        gc = max(0, min(gc, self.cols - 1))

        # If start or goal cell is not traversable, snap to nearest traversable cell
        sr, sc = self._snap_to_traversable(sr, sc)
        gr, gc = self._snap_to_traversable(gr, gc)

        if sr < 0 or gr < 0:
            return float("inf"), []

        # A* with 8-connected neighbors
        SQRT2 = math.sqrt(2.0)
        # Neighbors: (dr, dc, cost_multiplier)
        neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, SQRT2),
            (-1, 1, SQRT2),
            (1, -1, SQRT2),
            (1, 1, SQRT2),
        ]

        def heuristic(r: int, c: int) -> float:
            # Octile distance heuristic (admissible for 8-connected)
            dr = abs(r - gr)
            dc = abs(c - gc)
            return (dr + dc) + (SQRT2 - 2.0) * min(dr, dc)

        # g_score map — use a dict for sparse storage (grid can be large)
        g_score: dict[tuple[int, int], float] = {(sr, sc): 0.0}
        parent: dict[tuple[int, int], tuple[int, int] | None] = {(sr, sc): None}

        # Open set: (f_score, g, row, col)
        open_set: list[tuple[float, float, int, int]] = [
            (heuristic(sr, sc), 0.0, sr, sc)
        ]
        closed: set[tuple[int, int]] = set()

        while open_set:
            f, g, r, c = heapq.heappop(open_set)

            if (r, c) in closed:
                continue
            closed.add((r, c))

            if r == gr and c == gc:
                # Reconstruct path
                path_cells = []
                cur: tuple[int, int] | None = (r, c)
                while cur is not None:
                    path_cells.append(cur)
                    cur = parent[cur]
                path_cells.reverse()
                path_world = [self.cell_to_world(pr, pc) for pr, pc in path_cells]
                distance = g * self.resolution
                return distance, path_world

            for dr, dc, cost in neighbors:
                nr, nc = r + dr, c + dc
                if not self.in_bounds(nr, nc):
                    continue
                if not self.traversable[nr, nc]:
                    continue
                if (nr, nc) in closed:
                    continue
                # For diagonal moves, also check the two adjacent cells to prevent
                # cutting through diagonal wall corners
                if dr != 0 and dc != 0:
                    if (
                        not self.traversable[r + dr, c]
                        or not self.traversable[r, c + dc]
                    ):
                        continue
                new_g = g + cost
                key = (nr, nc)
                if key not in g_score or new_g < g_score[key]:
                    g_score[key] = new_g
                    parent[key] = (r, c)
                    heapq.heappush(open_set, (new_g + heuristic(nr, nc), new_g, nr, nc))

        return float("inf"), []

    def _snap_to_traversable(
        self, row: int, col: int, max_radius: int = 20
    ) -> tuple[int, int]:
        """Find the nearest traversable cell within max_radius (BFS spiral)."""
        if self.in_bounds(row, col) and self.traversable[row, col]:
            return row, col
        # BFS expanding ring search
        for radius in range(1, max_radius + 1):
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if abs(dr) != radius and abs(dc) != radius:
                        continue  # only check ring perimeter
                    nr, nc = row + dr, col + dc
                    if self.in_bounds(nr, nc) and self.traversable[nr, nc]:
                        return nr, nc
        return -1, -1  # no traversable cell found


# ---------------------------------------------------------------------------
# Load PCL binary_compressed PCD files
# ---------------------------------------------------------------------------


def _load_pcd(pcd_path: str | Path) -> dict[str, np.ndarray]:
    """
    Load a PCL .pcd file (binary, binary_compressed, or ascii).

    Returns a dict mapping field names to 1-D numpy arrays (float32).
    """
    pcd_path = Path(pcd_path)
    with open(pcd_path, "rb") as f:
        # Parse header lines
        fields: list[str] = []
        sizes: list[int] = []
        types: list[str] = []
        counts: list[int] = []
        n_points = 0
        data_format = ""
        while True:
            line = f.readline().decode("ascii", errors="replace").strip()
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("SIZE"):
                sizes = [int(x) for x in line.split()[1:]]
            elif line.startswith("TYPE"):
                types = line.split()[1:]
            elif line.startswith("COUNT"):
                counts = [int(x) for x in line.split()[1:]]
            elif line.startswith("POINTS"):
                n_points = int(line.split()[1])
            elif line.startswith("DATA"):
                data_format = line.split()[1]
                break

        if not fields or n_points == 0:
            raise ValueError(f"Invalid PCD header in {pcd_path}")

        # Build numpy dtype for a single point
        point_step = sum(s * c for s, c in zip(sizes, counts))
        dtype_map = {"F": "f", "U": "u", "I": "i"}
        np_fields: list[tuple[str, str]] = []
        for name, sz, tp, cnt in zip(fields, sizes, types, counts):
            np_dtype = f"<{dtype_map[tp]}{sz}"
            if cnt == 1:
                np_fields.append((name, np_dtype))
            else:
                for ci in range(cnt):
                    np_fields.append((f"{name}_{ci}", np_dtype))
        dt = np.dtype(np_fields)

        if data_format == "binary_compressed":
            # Read compressed and uncompressed sizes (2 uint32 LE)
            compressed_size = struct.unpack("<I", f.read(4))[0]
            uncompressed_size = struct.unpack("<I", f.read(4))[0]
            compressed_data = f.read(compressed_size)
            raw = lzf.decompress(compressed_data, uncompressed_size)
            # PCL binary_compressed stores each field contiguously (column-major),
            # not interleaved per point.
            result: dict[str, np.ndarray] = {}
            offset = 0
            for name, sz, tp, cnt in zip(fields, sizes, types, counts):
                np_dtype = f"<{dtype_map[tp]}{sz}"
                field_bytes = sz * cnt * n_points
                arr = np.frombuffer(
                    raw, dtype=np_dtype, count=n_points * cnt, offset=offset
                )
                if cnt == 1:
                    result[name] = arr.copy()
                else:
                    for ci in range(cnt):
                        result[f"{name}_{ci}"] = arr[ci::cnt].copy()
                offset += field_bytes
            return result

        elif data_format == "binary":
            raw = f.read(point_step * n_points)
            arr = np.frombuffer(raw, dtype=dt, count=n_points)
            return {name: arr[name].copy() for name in arr.dtype.names}

        elif data_format == "ascii":
            lines_data = f.read().decode("ascii").strip().split("\n")
            all_vals = np.array(
                [[float(v) for v in line.split()] for line in lines_data[:n_points]],
                dtype=np.float32,
            )
            result = {}
            col = 0
            for name, cnt in zip(fields, counts):
                if cnt == 1:
                    result[name] = all_vals[:, col]
                    col += 1
                else:
                    for ci in range(cnt):
                        result[f"{name}_{ci}"] = all_vals[:, col]
                        col += 1
            return result
        else:
            raise ValueError(f"Unsupported PCD DATA format: {data_format}")


# ---------------------------------------------------------------------------
# Build grid from a static PCD map (local ground surface estimation)
# ---------------------------------------------------------------------------


def build_grid_from_pcd(
    pcd_path: str | Path,
    resolution: float = 0.15,
    margin: float = 2.0,
    ground_height_max: float = 0.5,
    ground_height_min: float = -0.2,
    ground_ratio_threshold: float = 0.1,
    coarse_factor: int = 5,
    robot_radius: float = 0.0,
) -> "TraversabilityGrid":
    """
    Build a 2D traversability grid from a static PCD map.

    Estimates a local ground surface from the point cloud, then classifies
    points as ground (within a height band above local surface) or obstacle
    (walls, ceiling). Rasterizes the classification into a 2D grid.

    This approach works well for underground mine PCD maps where the terrain
    is not level and the point cloud includes floor, walls, and ceiling returns.

    Parameters
    ----------
    pcd_path : Path to a .pcd file (PCL format).
    resolution : Grid cell size in meters.
    margin : Extra margin around point bounds in meters.
    ground_height_max : Max height above local ground surface for a point to be
                        classified as ground (meters).
    ground_height_min : Min height above local ground surface (allows small
                        negative values for noise tolerance).
    ground_ratio_threshold : Cells with ground_count/total > this are traversable.
    coarse_factor : Factor for coarse ground surface estimation grid. The coarse
                    grid cell size is resolution * coarse_factor.

    Returns
    -------
    TraversabilityGrid
    """
    from scipy.ndimage import (
        binary_closing,
        binary_opening,
        label,
        median_filter,
        zoom,
    )

    # Load PCD
    pcd = _load_pcd(pcd_path)
    x = pcd["x"]
    y = pcd["y"]
    z = pcd["z"]
    n_total = len(x)
    print(f"  PCD loaded: {n_total} points")

    # --- Step 1: Estimate local ground surface ---
    # Build a coarse min-Z grid, smooth it, then upsample to fine resolution.
    coarse_res = resolution * coarse_factor
    x_min = float(x.min()) - margin
    y_min = float(y.min()) - margin
    x_max = float(x.max()) + margin
    y_max = float(y.max()) + margin
    n_cols_coarse = int(math.ceil((x_max - x_min) / coarse_res))
    n_rows_coarse = int(math.ceil((y_max - y_min) / coarse_res))

    col_idx_c = np.clip(
        ((x - x_min) / coarse_res).astype(np.int32), 0, n_cols_coarse - 1
    )
    row_idx_c = np.clip(
        ((y - y_min) / coarse_res).astype(np.int32), 0, n_rows_coarse - 1
    )

    # Min-Z per coarse cell = approximate ground surface
    z_min_coarse = np.full((n_rows_coarse, n_cols_coarse), np.inf, dtype=np.float64)
    np.minimum.at(z_min_coarse, (row_idx_c, col_idx_c), z.astype(np.float64))

    # Smooth with median filter to remove outlier dips
    z_ground = np.where(z_min_coarse == np.inf, np.nan, z_min_coarse)
    z_smooth = median_filter(np.nan_to_num(z_ground, nan=0), size=3)
    z_smooth = np.where(np.isnan(z_ground) & (z_smooth == 0), np.nan, z_smooth)

    # Upsample to fine grid resolution
    n_cols = int(math.ceil((x_max - x_min) / resolution))
    n_rows = int(math.ceil((y_max - y_min) / resolution))
    z_surface_fine = zoom(np.nan_to_num(z_smooth, nan=0), coarse_factor, order=1)
    # Crop to match fine grid dimensions
    z_surface_fine = z_surface_fine[:n_rows, :n_cols]
    if z_surface_fine.shape[0] < n_rows or z_surface_fine.shape[1] < n_cols:
        padded = np.zeros((n_rows, n_cols), dtype=z_surface_fine.dtype)
        padded[: z_surface_fine.shape[0], : z_surface_fine.shape[1]] = z_surface_fine
        z_surface_fine = padded

    # --- Step 2: Classify points as ground/obstacle ---
    col_idx = np.clip(((x - x_min) / resolution).astype(np.int32), 0, n_cols - 1)
    row_idx = np.clip(((y - y_min) / resolution).astype(np.int32), 0, n_rows - 1)

    local_z = z_surface_fine[row_idx, col_idx]
    height_above = z.astype(np.float64) - local_z
    is_ground = (height_above >= ground_height_min) & (
        height_above <= ground_height_max
    )

    n_ground = int(is_ground.sum())
    print(
        f"  Ground points: {n_ground}/{n_total} "
        f"({100 * n_ground / n_total:.1f}%), "
        f"height band: [{ground_height_min}, {ground_height_max}]m"
    )

    # --- Step 3: Rasterize into traversability grid ---
    ground_count = np.zeros((n_rows, n_cols), dtype=np.int32)
    obstacle_count = np.zeros((n_rows, n_cols), dtype=np.int32)

    np.add.at(ground_count, (row_idx[is_ground], col_idx[is_ground]), 1)
    np.add.at(obstacle_count, (row_idx[~is_ground], col_idx[~is_ground]), 1)

    # Compute mean ground-point Z per cell for elevation map
    z_sum = np.zeros((n_rows, n_cols), dtype=np.float64)
    np.add.at(
        z_sum, (row_idx[is_ground], col_idx[is_ground]), z[is_ground].astype(np.float64)
    )
    with np.errstate(divide="ignore", invalid="ignore"):
        elevation = np.where(ground_count > 0, z_sum / ground_count, np.nan)

    total_count = ground_count + obstacle_count
    with np.errstate(divide="ignore", invalid="ignore"):
        ground_ratio = np.where(total_count > 0, ground_count / total_count, 0.0)

    traversable = ground_ratio > ground_ratio_threshold

    # --- Step 4: Morphological cleanup ---
    traversable = binary_opening(traversable, structure=np.ones((3, 3)))
    traversable = binary_closing(traversable, structure=np.ones((5, 5)))

    # Keep only the largest connected component
    labeled, n_features = label(traversable)
    if n_features > 0:
        component_sizes = np.bincount(labeled.ravel())
        component_sizes[0] = 0  # background
        largest = np.argmax(component_sizes)
        traversable = labeled == largest

    traversable = traversable.astype(bool)

    # --- Step 4b: Inflate obstacles by robot radius ---
    if robot_radius > 0:
        from scipy.ndimage import binary_erosion

        radius_cells = int(math.ceil(robot_radius / resolution))
        # Build a disk structuring element
        y_k, x_k = np.ogrid[
            -radius_cells : radius_cells + 1, -radius_cells : radius_cells + 1
        ]
        disk = (x_k * x_k + y_k * y_k) <= radius_cells * radius_cells
        traversable = binary_erosion(traversable, structure=disk).astype(bool)
        # Re-extract largest connected component after erosion
        labeled, n_features = label(traversable)
        if n_features > 0:
            component_sizes = np.bincount(labeled.ravel())
            component_sizes[0] = 0
            largest = np.argmax(component_sizes)
            traversable = (labeled == largest).astype(bool)
        n_free_inflated = int(traversable.sum())
        print(
            f"  After robot inflation (r={robot_radius}m, "
            f"{radius_cells} cells): traversable={n_free_inflated}"
        )

    # Fill NaN gaps in elevation within traversable region using iterative
    # neighbor averaging, then smooth with 5x5 median filter.
    from scipy.ndimage import uniform_filter

    elev = elevation.copy()
    # Iteratively fill NaN holes inside the traversable mask
    for _ in range(10):
        nan_mask = np.isnan(elev) & traversable
        if not nan_mask.any():
            break
        # Average of valid neighbors
        filled = np.where(np.isnan(elev), 0.0, elev)
        counts = uniform_filter(
            (~np.isnan(elev)).astype(np.float64), size=5, mode="constant"
        )
        avg = uniform_filter(filled, size=5, mode="constant")
        with np.errstate(divide="ignore", invalid="ignore"):
            avg = np.where(counts > 0, avg / counts, np.nan)
        elev = np.where(nan_mask, avg, elev)

    # Apply 5x5 median filter for final smoothing
    has_data = ~np.isnan(elev)
    elev_for_filter = np.nan_to_num(elev, nan=0.0)
    elev = np.where(has_data, median_filter(elev_for_filter, size=5), np.nan)

    # Mask elevation to traversable cells only
    elevation = np.where(traversable, elev, np.nan)

    n_free = int(traversable.sum())
    print(
        f"  PCD grid: {n_rows}x{n_cols}, resolution={resolution}m, "
        f"traversable={n_free}/{traversable.size} "
        f"({100 * n_free / traversable.size:.1f}%)"
    )

    return TraversabilityGrid(
        traversable=traversable,
        resolution=resolution,
        x_min=x_min,
        y_min=y_min,
        elevation=elevation,
    )
    min_z_grid = np.where(np.isnan(filled), min_z_grid, filled)

    # Look up local ground height for each point and filter ceiling
    local_ground = min_z_grid[row_idx_c, col_idx_c]
    height_above_ground = z.astype(np.float64) - local_ground
    ceiling_mask = height_above_ground <= ceiling_height
    # Also remove points where we couldn't determine ground (inf)
    ceiling_mask &= local_ground != np.inf

    x_f = x[ceiling_mask]
    y_f = y[ceiling_mask]
    z_f = z[ceiling_mask]
    print(
        f"  After ceiling filter ({ceiling_height}m): {len(x_f)} points ({100 * len(x_f) / n_total:.1f}%)"
    )

    # --- Step 2: PMF ground segmentation ---
    # Progressive Morphological Filter on the full 2D-projected point cloud.
    # We work on a height grid: for each cell, store min Z.
    # Then iteratively apply opening (erosion + dilation) with increasing window
    # to build an estimated ground surface, and classify points as ground if
    # they are within distance threshold of the estimated surface.

    pmf_res = resolution  # use same resolution as output grid
    x_min = float(x_f.min()) - margin
    y_min = float(y_f.min()) - margin
    x_max = float(x_f.max()) + margin
    y_max = float(y_f.max()) + margin
    n_cols = int(math.ceil((x_max - x_min) / pmf_res))
    n_rows = int(math.ceil((y_max - y_min) / pmf_res))

    col_idx = np.clip(((x_f - x_min) / pmf_res).astype(np.int32), 0, n_cols - 1)
    row_idx = np.clip(((y_f - y_min) / pmf_res).astype(np.int32), 0, n_rows - 1)

    # Min-Z grid for PMF
    z_min_grid = np.full((n_rows, n_cols), np.inf, dtype=np.float64)
    np.minimum.at(z_min_grid, (row_idx, col_idx), z_f.astype(np.float64))

    # PMF: iterative morphological opening with increasing window
    from scipy.ndimage import grey_opening

    surface = z_min_grid.copy()
    # Replace inf with a large value for morphological ops
    no_data_mask = surface == np.inf
    surface[no_data_mask] = 999.0

    # Compute window sizes: 1, 2, 4, ..., up to max_window_size (in cells)
    max_win_cells = max(1, int(pmf_max_window_size / pmf_res))
    window_sizes = []
    w = 1
    while w <= max_win_cells:
        window_sizes.append(w)
        w *= 2
    if window_sizes[-1] < max_win_cells:
        window_sizes.append(max_win_cells)

    for k, win in enumerate(window_sizes):
        if win < 1:
            continue
        sz = 2 * win + 1  # full window diameter in cells
        opened = grey_opening(surface, size=(sz, sz))

        # Distance threshold increases linearly with iteration
        if k == 0:
            d_thresh = pmf_initial_distance
        else:
            d_thresh = min(
                pmf_slope * (win * pmf_res) + pmf_initial_distance,
                pmf_max_distance,
            )

        # Update surface: where current surface is too far above opened,
        # pull it down to opened (these cells have obstacles on top of ground)
        diff = surface - opened
        surface = np.where(diff > d_thresh, opened, surface)

    # Restore no-data cells
    surface[no_data_mask] = np.inf

    # --- Step 3: Classify points as ground/obstacle ---
    # A point is ground if its Z is within pmf_max_distance of the estimated surface
    surface_z = surface[row_idx, col_idx]
    point_height = z_f.astype(np.float64) - surface_z
    is_ground = (point_height <= pmf_max_distance) & (surface_z != np.inf)

    # --- Step 4: Rasterize into traversability grid ---
    ground_count = np.zeros((n_rows, n_cols), dtype=np.int32)
    obstacle_count = np.zeros((n_rows, n_cols), dtype=np.int32)

    ground_rows = row_idx[is_ground]
    ground_cols = col_idx[is_ground]
    np.add.at(ground_count, (ground_rows, ground_cols), 1)

    obs_rows = row_idx[~is_ground]
    obs_cols = col_idx[~is_ground]
    np.add.at(obstacle_count, (obs_rows, obs_cols), 1)

    total_count = ground_count + obstacle_count
    with np.errstate(divide="ignore", invalid="ignore"):
        ground_ratio = np.where(total_count > 0, ground_count / total_count, 0.0)

    traversable = ground_ratio > ground_ratio_threshold

    # --- Step 5: Morphological cleanup ---
    traversable = binary_opening(traversable, structure=np.ones((3, 3)))
    traversable = binary_closing(traversable, structure=np.ones((5, 5)))

    # Keep only the largest connected component
    labeled, n_features = label(traversable)
    if n_features > 0:
        component_sizes = np.bincount(labeled.ravel())
        component_sizes[0] = 0  # background
        largest = np.argmax(component_sizes)
        traversable = labeled == largest

    traversable = traversable.astype(bool)
    n_free = int(traversable.sum())
    print(
        f"  PCD grid: {n_rows}x{n_cols}, resolution={pmf_res}m, "
        f"traversable={n_free}/{traversable.size} ({100 * n_free / traversable.size:.1f}%)"
    )

    return TraversabilityGrid(
        traversable=traversable,
        resolution=pmf_res,
        x_min=x_min,
        y_min=y_min,
    )


# ---------------------------------------------------------------------------
# Build grid from a ROS 2 bag
# ---------------------------------------------------------------------------


def build_grid_from_bag(
    bag_path: Path,
    goal_timestamp: int | None = None,
    resolution: float = 0.15,
    margin: float = 2.0,
    max_range: float = 5.0,
    ground_ratio_threshold: float = 0.3,
    contour_segments: list | None = None,
    typestore=None,
) -> TraversabilityGrid:
    """
    Build a 2D traversability grid from /terrain_cloud in a nav bag.

    Extracts terrain points from goal_timestamp to end of bag, transforms
    them from base_link to map frame using /odometry_map, classifies by
    PMF intensity, and rasterizes into a 2D grid.

    Optionally burns contour segments (from /robot_vgraph contour_connects)
    onto the grid as non-traversable walls, so A* respects mine geometry.

    Parameters
    ----------
    bag_path : Path to the bag directory.
    goal_timestamp : Only use messages at or after this time (nanoseconds).
                     If None, use all messages.
    resolution : Grid cell size in meters.
    margin : Extra margin around point bounds in meters.
    max_range : Max horizontal distance from robot in base_link frame (meters).
    ground_ratio_threshold : Cells with ground_count / total > this are traversable.
    contour_segments : List of ((x1,y1),(x2,y2)) wall segments to burn as obstacles.
    typestore : rosbags typestore (created if None).

    Returns
    -------
    TraversabilityGrid
    """
    if typestore is None:
        typestore = get_typestore(Stores.ROS2_HUMBLE)

    # --- Pass 1: collect all /odometry_map poses for interpolation ---
    odom_times: list[int] = []
    odom_positions: list[np.ndarray] = []
    odom_orientations: list[np.ndarray] = []  # (qx, qy, qz, qw)

    with Reader(bag_path) as reader:
        connections = {c.topic: c for c in reader.connections}

        if "/odometry_map" in connections:
            conn = connections["/odometry_map"]
            for _, ts, rawdata in reader.messages(connections=[conn]):
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                p = msg.pose.pose.position
                o = msg.pose.pose.orientation
                odom_times.append(ts)
                odom_positions.append(np.array([p.x, p.y, p.z]))
                odom_orientations.append(np.array([o.x, o.y, o.z, o.w]))

    if len(odom_times) == 0:
        raise RuntimeError(f"No /odometry_map messages in {bag_path}")

    odom_times_arr = np.array(odom_times, dtype=np.int64)

    # --- Pass 2: extract terrain points, transform to map frame ---
    all_map_xy: list[np.ndarray] = []  # (N, 2) arrays
    all_intensities: list[np.ndarray] = []  # (N,) arrays

    with Reader(bag_path) as reader:
        connections = {c.topic: c for c in reader.connections}

        if "/terrain_cloud" not in connections:
            raise RuntimeError(f"No /terrain_cloud topic in {bag_path}")

        conn = connections["/terrain_cloud"]
        msg_count = 0

        for _, ts, rawdata in reader.messages(connections=[conn]):
            if goal_timestamp is not None and ts < goal_timestamp:
                continue

            msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
            n_points = msg.width * msg.height
            if n_points == 0:
                continue

            # Decode points from raw bytes
            point_step = msg.point_step
            data = bytes(msg.data)
            # Vectorized decode: extract x, y, z, intensity
            points_bl = np.zeros((n_points, 4), dtype=np.float32)
            for i in range(n_points):
                off = i * point_step
                points_bl[i, 0] = struct.unpack_from("<f", data, off)[0]  # x
                points_bl[i, 1] = struct.unpack_from("<f", data, off + 4)[0]  # y
                points_bl[i, 2] = struct.unpack_from("<f", data, off + 8)[0]  # z
                points_bl[i, 3] = struct.unpack_from("<f", data, off + 16)[
                    0
                ]  # intensity

            # Filter out ceiling points (z > 1.5m in base_link) — should already
            # be filtered, but just in case
            mask = points_bl[:, 2] <= 1.5
            # Filter by horizontal distance from robot in base_link (keep within max_range)
            dist_sq = points_bl[:, 0] ** 2 + points_bl[:, 1] ** 2
            mask &= dist_sq <= max_range**2
            points_bl = points_bl[mask]
            if len(points_bl) == 0:
                continue

            # Find nearest odometry pose (by timestamp)
            idx = int(np.searchsorted(odom_times_arr, ts))
            idx = max(0, min(idx, len(odom_times) - 1))

            pos = odom_positions[idx]
            quat = odom_orientations[idx]
            R = quat_to_rotation_matrix(quat[0], quat[1], quat[2], quat[3])

            # Transform base_link → map:  p_map = R @ p_bl + t
            xyz_bl = points_bl[:, :3].T  # (3, N)
            xyz_map = (R @ xyz_bl).T + pos  # (N, 3)

            all_map_xy.append(xyz_map[:, :2].astype(np.float32))
            all_intensities.append(points_bl[:, 3])
            msg_count += 1

    if msg_count == 0:
        raise RuntimeError(
            f"No /terrain_cloud messages after goal_timestamp in {bag_path}"
        )

    print(f"  Processed {msg_count} terrain_cloud messages")

    # Concatenate all points
    map_xy = np.concatenate(all_map_xy, axis=0)  # (N, 2)
    intensities = np.concatenate(all_intensities)  # (N,)

    # --- Rasterize into grid ---
    x_min = float(map_xy[:, 0].min()) - margin
    x_max = float(map_xy[:, 0].max()) + margin
    y_min = float(map_xy[:, 1].min()) - margin
    y_max = float(map_xy[:, 1].max()) + margin

    n_cols = int(math.ceil((x_max - x_min) / resolution))
    n_rows = int(math.ceil((y_max - y_min) / resolution))

    ground_count = np.zeros((n_rows, n_cols), dtype=np.int32)
    obstacle_count = np.zeros((n_rows, n_cols), dtype=np.int32)

    # Compute cell indices for all points
    col_idx = ((map_xy[:, 0] - x_min) / resolution).astype(np.int32)
    row_idx = ((map_xy[:, 1] - y_min) / resolution).astype(np.int32)

    # Clamp to valid range
    col_idx = np.clip(col_idx, 0, n_cols - 1)
    row_idx = np.clip(row_idx, 0, n_rows - 1)

    # Classify: intensity < 0.5 = ground, >= 0.5 = obstacle
    is_ground = intensities < 0.5

    # Accumulate counts (vectorized with np.add.at)
    ground_rows = row_idx[is_ground]
    ground_cols = col_idx[is_ground]
    np.add.at(ground_count, (ground_rows, ground_cols), 1)

    obs_rows = row_idx[~is_ground]
    obs_cols = col_idx[~is_ground]
    np.add.at(obstacle_count, (obs_rows, obs_cols), 1)

    # Classify cells
    total_count = ground_count + obstacle_count
    with np.errstate(divide="ignore", invalid="ignore"):
        ground_ratio = np.where(total_count > 0, ground_count / total_count, 0.0)

    traversable = ground_ratio > ground_ratio_threshold

    # --- Burn contour segments as obstacles ---
    if contour_segments:
        n_burned = 0
        for (x1, y1), (x2, y2) in contour_segments:
            c0 = int((x1 - x_min) / resolution)
            r0 = int((y1 - y_min) / resolution)
            c1 = int((x2 - x_min) / resolution)
            r1 = int((y2 - y_min) / resolution)
            for r, c in _bresenham(r0, c0, r1, c1):
                if 0 <= r < n_rows and 0 <= c < n_cols:
                    traversable[r, c] = False
                    n_burned += 1
        print(
            f"  Burned {n_burned} cells from {len(contour_segments)} contour segments"
        )

    # --- Morphological cleanup ---
    from scipy.ndimage import binary_opening, binary_closing, label

    # Opening removes small isolated traversable pixels (noise)
    traversable = binary_opening(traversable, structure=np.ones((3, 3)))
    # Closing fills small holes in traversable regions
    traversable = binary_closing(traversable, structure=np.ones((5, 5)))

    # Keep only the largest connected component
    labeled, n_features = label(traversable)
    if n_features > 0:
        component_sizes = np.bincount(labeled.ravel())
        # component 0 is background
        component_sizes[0] = 0
        largest = np.argmax(component_sizes)
        traversable = labeled == largest

    traversable = traversable.astype(bool)
    n_free = int(traversable.sum())
    print(
        f"  Grid: {n_rows}x{n_cols}, resolution={resolution}m, "
        f"traversable={n_free}/{traversable.size} ({100 * n_free / traversable.size:.1f}%)"
    )

    return TraversabilityGrid(
        traversable=traversable,
        resolution=resolution,
        x_min=x_min,
        y_min=y_min,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Grid-based geodesic distance from a nav bag's terrain cloud"
    )
    parser.add_argument("--bag", type=str, required=True, help="Path to bag directory")
    parser.add_argument(
        "--start", type=float, nargs=2, required=True, help="Start position: x y"
    )
    parser.add_argument(
        "--goal", type=float, nargs=2, required=True, help="Goal position: x y"
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.15,
        help="Grid resolution in meters (default: 0.15)",
    )
    parser.add_argument("--plot", action="store_true", help="Plot the grid and A* path")
    default_pcd = str(
        Path(__file__).resolve().parent.parent / "map" / "experimental_mine.pcd"
    )
    parser.add_argument(
        "--pcd",
        type=str,
        default=default_pcd,
        help="Path to .pcd map file for static background in plots (default: map/experimental_mine.pcd)",
    )
    args = parser.parse_args()

    bag_path = Path(args.bag)
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # Get goal_timestamp from /goal_pose
    goal_timestamp = None
    with Reader(bag_path) as reader:
        connections = {c.topic: c for c in reader.connections}
        if "/goal_pose" in connections:
            conn = connections["/goal_pose"]
            for _, ts, _ in reader.messages(connections=[conn]):
                goal_timestamp = ts
                break

    if goal_timestamp is not None:
        print(f"Goal timestamp: {goal_timestamp}")
    else:
        print("WARNING: No /goal_pose found, using all terrain_cloud messages")

    grid = build_grid_from_bag(
        bag_path,
        goal_timestamp=goal_timestamp,
        resolution=args.resolution,
        typestore=typestore,
    )

    print(
        f"\nRunning A* from ({args.start[0]:.2f}, {args.start[1]:.2f}) "
        f"to ({args.goal[0]:.2f}, {args.goal[1]:.2f})..."
    )
    dist, path = grid.astar(args.start, args.goal)

    euclidean = math.sqrt(
        (args.start[0] - args.goal[0]) ** 2 + (args.start[1] - args.goal[1]) ** 2
    )
    print(f"Euclidean distance: {euclidean:.3f}m")
    print(f"Grid A* distance:   {dist:.3f}m")
    if euclidean > 0:
        print(f"Ratio (A*/Eucl):    {dist / euclidean:.3f}")

    if args.plot:
        import matplotlib.pyplot as plt

        # Build static PCD map grid for consistent background
        print("Building static map grid from PCD for plot background...")
        map_grid = build_grid_from_pcd(args.pcd, resolution=args.resolution)

        fig, ax = plt.subplots(figsize=(14, 10))

        # Static mine map from PCD — elevation heatmap background
        map_extent = [
            map_grid.x_min,
            map_grid.x_min + map_grid.cols * map_grid.resolution,
            map_grid.y_min,
            map_grid.y_min + map_grid.rows * map_grid.resolution,
        ]
        if map_grid.elevation is not None:
            elev = map_grid.elevation.copy()
            elev_masked = np.ma.masked_invalid(elev)
            im = ax.imshow(
                elev_masked,
                origin="lower",
                extent=map_extent,
                cmap="Greens",
                alpha=0.7,
                aspect="equal",
                zorder=0,
            )
            cbar = fig.colorbar(im, ax=ax, shrink=0.6, pad=0.02)
            cbar.set_label("Elevation (m)", fontsize=10)
        else:
            ax.imshow(
                map_grid.traversable,
                origin="lower",
                extent=map_extent,
                cmap="Greens",
                alpha=0.5,
                aspect="equal",
                zorder=0,
            )

        # Plot A* path
        if path:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            ax.plot(px, py, "b-", linewidth=1.5, label=f"A* path ({dist:.1f}m)")

        # Plot start / goal
        ax.plot(args.start[0], args.start[1], "ro", markersize=10, label="Start")
        ax.plot(args.goal[0], args.goal[1], "r*", markersize=15, label="Goal")

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_xlim(-25, 10)
        ax.set_ylim(-25, 10)
        ax.set_aspect("equal")
        ax.legend(fontsize=9)
        # Parse goal/rep from bag name (e.g. mine_nav2_r3 -> Goal 2, Rep 3)
        _m = re.match(r"mine_nav(\d+)_r(\d+)", bag_path.name)
        if _m:
            ax.set_title(f"Goal {_m.group(1)}, Rep {_m.group(2)}", pad=12)
        else:
            ax.set_title(bag_path.name, pad=12)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()

        metrics_dir = Path(__file__).resolve().parent.parent / "metrics"
        metrics_dir.mkdir(exist_ok=True)
        save_path = metrics_dir / f"grid_geodesic_{bag_path.name}.pdf"
        plt.savefig(save_path, bbox_inches="tight")
        print(f"Saved plot to {save_path}")

        plt.show()


if __name__ == "__main__":
    main()
