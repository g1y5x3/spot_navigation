#!/usr/bin/env python3
"""
Benchmark extraction script.
Reads ROS 2 bag files from mine navigation experiments and computes:
  - SR  (Success Rate): human-verified, cross-checked with /far_reach_goal_status
  - SPL (Success weighted by Path Length): Anderson et al., 2018
  - Path Length Ratio (p_i / l_i): navigation efficiency
  - Completion Time: wall-clock seconds from first to last odometry message

The geodesic shortest-path distance l_i is computed per-trial via A* on a 2D
traversability grid built from /terrain_cloud (PMF classification) in the bag.

Usage:
    python3 benchmark.py --bags-dir /path/to/bags --goals 1

Bag naming convention: mine_nav{goal}_r{rep}
  goal 1: Goal 1 [-2.496, 0.546, 0.002]
  goal 2: Goal 2 [-7.328, -9.266, -0.156]
  goal 3: Goal 3 [-20.756, -11.535, -0.320]
  goal 4: Entrance [6.220, -6.278, 0.0]  (arbitrary start -> entrance)
"""

import argparse
import csv
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

from grid_geodesic import (
    build_grid_from_pcd,
)


# Goal definitions (from goal*.py and entrance.py)
GOALS = {
    1: {"name": "Goal 1", "position": [-2.496, 0.546, 0.002]},
    2: {"name": "Goal 2", "position": [-7.328, -9.266, -0.156]},
    3: {"name": "Goal 3", "position": [-20.756, -11.535, -0.320]},
    4: {"name": "Entrance", "position": [6.220, -6.278, 0.0]},
}

# Human-verified success for each trial.
# Fill in False for any trial that failed based on human observation.
# If a bag_name is not listed here, /far_reach_goal_status is used as fallback.
HUMAN_VERIFIED_SUCCESS = {
    # Goal 1
    "mine_nav1_r1": True,
    "mine_nav1_r2": True,
    "mine_nav1_r3": True,
    "mine_nav1_r4": True,
    "mine_nav1_r5": True,
    # Goal 2
    "mine_nav2_r1": True,
    "mine_nav2_r2": True,
    "mine_nav2_r3": True,
    "mine_nav2_r4": True,
    "mine_nav2_r5": True,
    # Goal 3
    "mine_nav3_r1": True,
    "mine_nav3_r2": True,
    "mine_nav3_r3": True,
    "mine_nav3_r4": True,
    "mine_nav3_r5": True,
    # Entrance
    "mine_nav4_r1": True,
    "mine_nav4_r2": True,
    "mine_nav4_r3": True,
    "mine_nav4_r4": True,
    "mine_nav4_r5": True,
}

GOAL_TOLERANCE = 0.5  # meters


@dataclass
class TrialResult:
    bag_name: str
    goal_id: int
    rep: int
    # Poses
    start_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    final_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    goal_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # Metrics
    path_length: float = 0.0  # p_i: actual odometry path length (m)
    geodesic_distance: float = 0.0  # l_i: grid A* shortest path (m)
    completion_time: float = 0.0  # seconds
    final_distance: float = 0.0  # self-reported distance to goal at termination (m)
    # Success
    far_goal_reached: bool = False  # from /far_reach_goal_status
    human_verified: bool = True  # human judgment override
    success: bool = False  # final determination
    # Derived
    spl: float = 0.0
    path_ratio: float = 0.0
    # Internal (not displayed)
    goal_timestamp: int | None = None  # nanoseconds, for grid building
    trajectory: np.ndarray = field(
        default_factory=lambda: np.empty((0, 3))
    )  # full odom path


# Bag processing
def process_bag(bag_path: Path, goal_id: int, rep: int, typestore) -> TrialResult:
    """Extract metrics from a single trial bag."""
    bag_name = bag_path.name
    result = TrialResult(bag_name=bag_name, goal_id=goal_id, rep=rep)
    result.goal_pos = np.array(GOALS[goal_id]["position"])

    with Reader(bag_path) as reader:
        connections = {c.topic: c for c in reader.connections}

        # 1. /goal_pose: get goal and its timestamp (navigation start) ---
        if "/goal_pose" in connections:
            conn = connections["/goal_pose"]
            for _, ts, rawdata in reader.messages(connections=[conn]):
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                bag_goal = np.array(
                    [
                        msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z,
                    ]
                )
                dist_to_expected = np.linalg.norm(bag_goal - result.goal_pos)
                if dist_to_expected > 2.0:
                    print(
                        f"  WARNING: {bag_name} /goal_pose {bag_goal} differs from "
                        f"expected {result.goal_pos} by {dist_to_expected:.2f}m"
                    )
                result.goal_pos = bag_goal  # use actual goal from bag
                result.goal_timestamp = ts
                break

        # 2. /odometry_map: trajectory, path length, timing
        # Only count data AFTER /goal_pose is received (navigation start).
        positions = np.empty((0, 3))
        timestamps: list[int] = []
        if "/odometry_map" in connections:
            conn = connections["/odometry_map"]
            pos_list: list[np.ndarray] = []
            for _, timestamp, rawdata in reader.messages(connections=[conn]):
                # Skip all odometry before goal was sent
                if (
                    result.goal_timestamp is not None
                    and timestamp < result.goal_timestamp
                ):
                    continue
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                pos = np.array(
                    [
                        msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z,
                    ]
                )
                pos_list.append(pos)
                timestamps.append(timestamp)

            if len(pos_list) >= 2:
                positions = np.array(pos_list)
                result.trajectory = positions
                result.start_pos = positions[0]
                result.final_pos = positions[-1]

                # p_i: sum of consecutive Euclidean distances
                diffs = np.diff(positions, axis=0)
                result.path_length = float(np.sum(np.linalg.norm(diffs, axis=1)))

                # Completion time (wall-clock from goal_pose to last odom)
                result.completion_time = (timestamps[-1] - timestamps[0]) / 1e9

                # Self-reported final distance to goal (map frame)
                result.final_distance = float(
                    np.linalg.norm(result.final_pos - result.goal_pos)
                )

        # 3. /far_reach_goal_status ---
        if "/far_reach_goal_status" in connections:
            conn = connections["/far_reach_goal_status"]
            for _, _, rawdata in reader.messages(connections=[conn]):
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                if msg.data:
                    result.far_goal_reached = True
                    # Once True in any message, done

        # 4. Determine success ---
        if bag_name in HUMAN_VERIFIED_SUCCESS:
            result.human_verified = HUMAN_VERIFIED_SUCCESS[bag_name]
        else:
            result.human_verified = result.far_goal_reached
        result.success = result.human_verified

        # SPL and path ratio are computed in main() after grid A* l_i is set.

    return result


# Display
# Column widths
_C = {"goal": 12, "n": 4, "sr": 6, "spl": 15, "pl": 15, "pi": 15, "time": 15}
_W = sum(_C.values()) + len(_C) - 1


def print_goal_summary(results: list[TrialResult], goal_ids: list[int]):
    print("\n" + "=" * _W)
    print("PER-GOAL SUMMARY (mean ± std)")
    print("=" * _W)
    print(
        f"{'Goal':<{_C['goal']}} {'N':>{_C['n']}} {'SR':>{_C['sr']}} "
        f"{'SPL':>{_C['spl']}} {'p/l':>{_C['pl']}} "
        f"{'p_i(m)':>{_C['pi']}} {'Time(s)':>{_C['time']}}"
    )
    print("-" * _W)

    all_results = []
    for goal_id in goal_ids:
        goal_results = [r for r in results if r.goal_id == goal_id]
        if not goal_results:
            continue
        all_results.extend(goal_results)
        _print_summary_row(GOALS[goal_id]["name"], goal_results)

    if all_results:
        print("-" * _W)
        _print_summary_row("ALL", all_results)


def _print_summary_row(label: str, results: list[TrialResult]):
    n = len(results)
    n_success = sum(1 for r in results if r.success)
    spls = np.array([r.spl for r in results])
    ratios = np.array([r.path_ratio for r in results if r.path_ratio > 0])
    path_lengths = np.array([r.path_length for r in results])
    times = np.array([r.completion_time for r in results])

    def fmt_mean_std(arr, val_fmt=".2f", w=15):
        if len(arr) == 0:
            return f"{'N/A':>{w}}"
        s = f"{np.mean(arr):{val_fmt}} ± {np.std(arr):{val_fmt}}"
        return f"{s:>{w}}"

    sr_str = f"{n_success}/{n}"
    print(
        f"{label:<{_C['goal']}} {n:>{_C['n']}} {sr_str:>{_C['sr']}} "
        f"{fmt_mean_std(spls, '.3f', _C['spl'])} "
        f"{fmt_mean_std(ratios, '.2f', _C['pl'])} "
        f"{fmt_mean_std(path_lengths, '.2f', _C['pi'])} "
        f"{fmt_mean_std(times, '.1f', _C['time'])}"
    )


# Main
def main():
    parser = argparse.ArgumentParser(description="Navigation Benchmark")
    # Default bags dir: workspace root (three levels up from this script)
    default_bags_dir = str(Path(__file__).resolve().parent.parent.parent.parent)
    parser.add_argument(
        "--bags-dir",
        type=str,
        default=default_bags_dir,
        help="Directory containing mine_nav*_r* bag directories (default: workspace root)",
    )
    parser.add_argument(
        "--goals",
        type=int,
        nargs="+",
        default=[1, 2, 3, 4],
        help="Goal IDs to process (default: 1 2 3 4)",
    )
    parser.add_argument(
        "--reps",
        type=int,
        default=5,
        help="Number of repetitions per goal (default: 5)",
    )
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.15,
        help="Traversability grid resolution in meters (default: 0.15)",
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        help="Export results to CSV file",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Save per-trial grid + A* path plots to metrics/ folder",
    )
    # Default PCD map: spot_navigation/map/experimental_mine.pcd
    default_pcd = str(
        Path(__file__).resolve().parent.parent / "map" / "experimental_mine.pcd"
    )
    # Default vgh: spot_navigation/map/experimental_mine.vgh
    default_vgh = str(
        Path(__file__).resolve().parent.parent / "map" / "experimental_mine.vgh"
    )
    parser.add_argument(
        "--pcd",
        type=str,
        default=default_pcd,
        help="Path to .pcd map file for static background in plots (default: map/experimental_mine.pcd)",
    )
    parser.add_argument(
        "--plot-vg",
        action="store_true",
        help="Save a visibility graph plot to metrics/ folder",
    )
    parser.add_argument(
        "--vgh",
        type=str,
        default=default_vgh,
        help="Path to .vgh file for visibility graph plot (default: map/experimental_mine.vgh)",
    )
    args = parser.parse_args()

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    bags_dir = Path(args.bags_dir)
    metrics_dir = Path(__file__).resolve().parent.parent / "metrics"

    if args.plot or args.plot_vg:
        metrics_dir.mkdir(exist_ok=True)

    # Build static map grid from PCD once — used for both geodesic (A*) and plot backgrounds.
    # Robot radius inflation ensures A* paths respect the robot's physical size.
    # Per Anderson et al. (2018), l_i is the shortest-path distance in the
    # environment (the full map), not in partial per-trial observations.
    print("Building static map grid from PCD...")
    map_grid = build_grid_from_pcd(
        args.pcd, resolution=args.resolution, robot_radius=0.5
    )
    # Also build an un-inflated grid for plot background (elevation heatmap
    # should show full traversable area, not the eroded planning grid)
    map_grid_visual = build_grid_from_pcd(args.pcd, resolution=args.resolution)

    if args.plot_vg:
        _save_vg_plot(args.vgh, metrics_dir)

    results: list[TrialResult] = []
    missing_bags: list[str] = []

    for goal_id in args.goals:
        for rep in range(1, args.reps + 1):
            bag_name = f"mine_nav{goal_id}_r{rep}"
            bag_path = bags_dir / bag_name

            if not bag_path.exists():
                missing_bags.append(bag_name)
                continue

            print(f"Processing {bag_name}...")
            try:
                result = process_bag(bag_path, goal_id, rep, typestore)

                # Compute l_i via A* on the static PCD map grid (with robot inflation)
                start_xy = [float(result.start_pos[0]), float(result.start_pos[1])]
                goal_xy = [float(result.goal_pos[0]), float(result.goal_pos[1])]
                geodesic_dist, astar_path = map_grid.astar(start_xy, goal_xy)
                result.geodesic_distance = geodesic_dist

                euclidean = float(
                    np.linalg.norm(result.start_pos[:2] - result.goal_pos[:2])
                )
                print(
                    f"  l_i(A*): {geodesic_dist:.2f}m  "
                    f"(Euclidean: {euclidean:.2f}m, ratio: {geodesic_dist / max(euclidean, 1e-6):.3f})"
                )

                # Compute SPL and path ratio
                if result.geodesic_distance > 0 and result.geodesic_distance != float(
                    "inf"
                ):
                    result.path_ratio = result.path_length / result.geodesic_distance
                    if result.success:
                        result.spl = result.geodesic_distance / max(
                            result.path_length, result.geodesic_distance
                        )
                    else:
                        result.spl = 0.0

                # Save per-trial plot
                if args.plot:
                    _save_trial_plot(
                        map_grid,
                        astar_path,
                        geodesic_dist,
                        result.trajectory,
                        result.path_length,
                        start_xy,
                        goal_xy,
                        bag_name,
                        metrics_dir,
                        map_grid=map_grid_visual,
                    )

                results.append(result)
            except Exception as e:
                print(f"  ERROR: {bag_name}: {e}")
                import traceback

                traceback.print_exc()

    if missing_bags:
        print(f"\nMissing bags ({len(missing_bags)}): {', '.join(missing_bags)}")

    if not results:
        print("No bags found. Nothing to report.")
        return

    print_goal_summary(results, args.goals)

    # CSV export
    if args.csv:
        csv_path = Path(args.csv)
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "bag_name",
                    "goal_id",
                    "goal_name",
                    "rep",
                    "success",
                    "far_goal_reached",
                    "path_length_m",
                    "geodesic_distance_m",
                    "path_ratio",
                    "spl",
                    "completion_time_s",
                    "final_distance_m",
                    "start_x",
                    "start_y",
                    "start_z",
                    "final_x",
                    "final_y",
                    "final_z",
                    "goal_x",
                    "goal_y",
                    "goal_z",
                ]
            )
            for r in results:
                writer.writerow(
                    [
                        r.bag_name,
                        r.goal_id,
                        GOALS[r.goal_id]["name"],
                        r.rep,
                        int(r.success),
                        int(r.far_goal_reached),
                        f"{r.path_length:.4f}",
                        f"{r.geodesic_distance:.4f}",
                        f"{r.path_ratio:.4f}",
                        f"{r.spl:.4f}",
                        f"{r.completion_time:.2f}",
                        f"{r.final_distance:.4f}",
                        f"{r.start_pos[0]:.4f}",
                        f"{r.start_pos[1]:.4f}",
                        f"{r.start_pos[2]:.4f}",
                        f"{r.final_pos[0]:.4f}",
                        f"{r.final_pos[1]:.4f}",
                        f"{r.final_pos[2]:.4f}",
                        f"{r.goal_pos[0]:.4f}",
                        f"{r.goal_pos[1]:.4f}",
                        f"{r.goal_pos[2]:.4f}",
                    ]
                )
        print(f"\nCSV exported to: {csv_path}")


def _parse_vgh(vgh_path: str) -> tuple[dict[int, dict], list]:
    """
    Parse a .vgh file.

    Returns (nodes_dict, contour_segments) where contour_segments is a list of
    [(x1,y1), (x2,y2)] line segments from contour_connects (polygon boundary edges).
    """
    nodes: dict[int, dict] = {}
    with open(vgh_path, "r") as f:
        for line in f:
            tokens = line.strip().split()
            if len(tokens) < 15:
                continue
            node_id = int(tokens[0])
            free_direct = int(tokens[1])
            position = (float(tokens[2]), float(tokens[3]))
            is_navpoint = int(tokens[13]) != 0

            # Parse pipe-delimited sections after token 15:
            # section 0: connect_nodes, 1: poly_connects, 2: contour_connects, 3: trajectory
            sections: list[list[int]] = []
            current: list[int] = []
            for t in tokens[15:]:
                if t == "|":
                    sections.append(current)
                    current = []
                else:
                    current.append(int(t))
            sections.append(current)

            contour_ids = sections[2] if len(sections) > 2 else []

            nodes[node_id] = {
                "pos": position,
                "type": free_direct,
                "is_navpoint": is_navpoint,
                "contour_connects": contour_ids,
            }

    # Build contour segments (deduplicated)
    contour_segments = []
    seen: set[tuple[int, int]] = set()
    for nid, node in nodes.items():
        for cid in node["contour_connects"]:
            if cid not in nodes:
                continue
            edge = (min(nid, cid), max(nid, cid))
            if edge in seen:
                continue
            seen.add(edge)
            contour_segments.append([node["pos"], nodes[cid]["pos"]])

    return nodes, contour_segments


def _save_vg_plot(vgh_path: str, metrics_dir: Path):
    """Parse a .vgh visibility graph file and save a 2D contour plot to metrics/."""
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    nodes, contour_segments = _parse_vgh(vgh_path)

    fig, ax = plt.subplots(figsize=(14, 10))

    # Contour edges (polygon boundary)
    if contour_segments:
        ax.add_collection(
            LineCollection(contour_segments, colors="0.4", linewidths=0.8, zorder=1)
        )

    # Nodes by type (skip navpoints)
    type_styles = {
        1: ("red", "CONVEX"),
        2: ("blue", "CONCAVE"),
        3: ("orange", "PILLAR"),
    }
    for tid, (color, label) in type_styles.items():
        xs = [
            n["pos"][0]
            for n in nodes.values()
            if n["type"] == tid and not n["is_navpoint"]
        ]
        ys = [
            n["pos"][1]
            for n in nodes.values()
            if n["type"] == tid and not n["is_navpoint"]
        ]
        ax.scatter(xs, ys, c=color, s=8, label=label, zorder=2, alpha=0.7)

    n_boundary = sum(1 for n in nodes.values() if not n["is_navpoint"])
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_ylim(-25, 10)
    ax.set_aspect("equal")
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title(
        f"Mine Contour ({n_boundary} boundary nodes, {len(contour_segments)} edges)"
    )
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    save_path = metrics_dir / "visibility_graph.pdf"
    plt.savefig(save_path, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved visibility graph to {save_path}")


def _save_trial_plot(
    grid,
    astar_path,
    geodesic_dist,
    trajectory,
    path_length,
    start_xy,
    goal_xy,
    bag_name,
    metrics_dir,
    map_grid=None,
):
    """Save a per-trial traversability grid + A* path + robot trajectory plot to metrics/."""
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(14, 10))

    # Static mine map from PCD — elevation heatmap background
    if map_grid is not None:
        map_extent = [
            map_grid.x_min,
            map_grid.x_min + map_grid.cols * map_grid.resolution,
            map_grid.y_min,
            map_grid.y_min + map_grid.rows * map_grid.resolution,
        ]
        if map_grid.elevation is not None:
            # Higher z = greener, lower z = whiter
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

    # Robot's actual traversed path (p_i)
    if len(trajectory) >= 2:
        ax.plot(
            trajectory[:, 0],
            trajectory[:, 1],
            "r-",
            linewidth=1.5,
            alpha=0.8,
            zorder=3,
            label=f"$p_i$ = {path_length:.1f}m",
        )

    # Shortest path (l_i)
    if astar_path:
        px = [p[0] for p in astar_path]
        py = [p[1] for p in astar_path]
        ax.plot(
            px,
            py,
            "b--",
            linewidth=1.5,
            zorder=3,
            label=f"$l_i$ (A*) = {geodesic_dist:.1f}m",
        )

    ax.plot(start_xy[0], start_xy[1], "ko", markersize=10, zorder=5, label="Start")
    ax.plot(goal_xy[0], goal_xy[1], "k*", markersize=15, zorder=5, label="Goal")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_xlim(-25, 10)
    ax.set_ylim(-25, 10)
    ax.set_aspect("equal")
    ax.legend(fontsize=9)
    # Parse goal/rep from bag name (e.g. mine_nav2_r3 -> Goal 2, Rep 3)
    _m = re.match(r"mine_nav(\d+)_r(\d+)", bag_name)
    if _m:
        ax.set_title(f"Goal {_m.group(1)}, Rep {_m.group(2)}", pad=12)
    else:
        ax.set_title(bag_name, pad=12)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    save_path = metrics_dir / f"grid_geodesic_{bag_name}.pdf"
    plt.savefig(save_path, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved plot to {save_path}")


if __name__ == "__main__":
    main()
