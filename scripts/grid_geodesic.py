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
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore


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
# Build grid from a ROS 2 bag
# ---------------------------------------------------------------------------


def build_grid_from_bag(
    bag_path: Path,
    goal_timestamp: int | None = None,
    resolution: float = 0.15,
    margin: float = 2.0,
    ground_ratio_threshold: float = 0.3,
    typestore=None,
) -> TraversabilityGrid:
    """
    Build a 2D traversability grid from /terrain_cloud in a nav bag.

    Extracts terrain points from goal_timestamp to end of bag, transforms
    them from base_link to map frame using /odometry_map, classifies by
    PMF intensity, and rasterizes into a 2D grid.

    Parameters
    ----------
    bag_path : Path to the bag directory.
    goal_timestamp : Only use messages at or after this time (nanoseconds).
                     If None, use all messages.
    resolution : Grid cell size in meters.
    margin : Extra margin around point bounds in meters.
    ground_ratio_threshold : Cells with ground_count / total > this are traversable.
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

        fig, ax = plt.subplots(figsize=(14, 10))

        # Show traversability grid
        extent = [
            grid.x_min,
            grid.x_min + grid.cols * grid.resolution,
            grid.y_min,
            grid.y_min + grid.rows * grid.resolution,
        ]
        ax.imshow(
            grid.traversable,
            origin="lower",
            extent=extent,
            cmap="Greens",
            alpha=0.6,
            aspect="equal",
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
        ax.set_ylim(-25, 10)
        ax.set_aspect("equal")
        ax.legend(fontsize=9)
        ax.set_title(f"Traversability Grid — {bag_path.name}")
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
