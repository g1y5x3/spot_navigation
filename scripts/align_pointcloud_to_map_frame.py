#!/usr/bin/env python3
"""
align_pointcloud_to_map_frame.py

Interactively align a point cloud to a user-defined local map frame.

Default workflow:
1. Load a PCD/PLY point cloud.
2. Downsample it and remove sparse statistical/radius outliers.
3. Project to top-down XY.
4. Detect wall-like lines with RANSAC.
5. Open a top-down plot.
6. Click the local origin.
7. Click a second point for the positive X direction.
8. Fit/snap origin Z to the ground plane.
9. Save:
      estimated_transform.json
      transformed.pcd
      transform_debug.png
10. Open a quick 3D QC view.

3D QC defaults:
    axis_length = 5.0
    origin_marker_radius = 0.04

Only exposed CLI options:
    --voxel-size
    --statistical-neighbors
    --statistical-std-ratio
    --radius-outlier-neighbors
    --radius-outlier-radius
    --no-noise-filter
    --ransac-threshold
    --max-lines
    --no-show-3d-qc

Install:
    pip install open3d numpy matplotlib

Usage:
    python align_pointcloud_to_map_frame.py input.pcd

Example with tuning:
    python align_pointcloud_to_map_frame.py input.pcd \
        --voxel-size 0.05 \
        --ransac-threshold 0.08 \
        --max-lines 16

Skip the Open3D QC view:
    python align_pointcloud_to_map_frame.py input.pcd --no-show-3d-qc
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

try:
    import open3d as o3d
except ModuleNotFoundError:
    o3d = None


EPS = 1e-9


@dataclass
class Line2D:
    normal: np.ndarray       # [a, b] in ax + by + c = 0
    c: float
    direction: np.ndarray    # unit direction vector
    point: np.ndarray        # point on line, usually centroid of inliers
    p0: np.ndarray           # segment endpoint for debug plot
    p1: np.ndarray           # segment endpoint for debug plot
    theta: float             # line orientation in radians, modulo pi
    length: float
    inlier_count: int
    score: float


@dataclass
class CornerCandidate:
    xy: np.ndarray
    score: float
    line_i: int
    line_j: int


@dataclass
class RankedOrigin:
    xy: np.ndarray
    score: float
    line_score: float
    positive_ratio: float
    x_axis_angle_rad: float
    x_axis_2d: np.ndarray
    y_axis_2d: np.ndarray
    source_candidate: CornerCandidate


def filter_finite_points(points: np.ndarray) -> np.ndarray:
    finite_mask = np.isfinite(points).all(axis=1)
    if not np.any(finite_mask):
        raise ValueError("No finite points remain in the point cloud.")
    return points[finite_mask]


def filter_noise_points(
    pcd: Any,
    statistical_neighbors: int,
    statistical_std_ratio: float,
    radius_outlier_neighbors: int,
    radius_outlier_radius: float,
) -> tuple[Any, dict[str, Any]]:
    """Remove sparse outliers while preserving coherent scene geometry."""
    if statistical_neighbors < 2:
        raise ValueError("statistical neighbors must be at least 2")
    if statistical_std_ratio <= 0:
        raise ValueError(
            "statistical standard-deviation ratio must be positive"
        )
    if radius_outlier_neighbors < 1:
        raise ValueError("radius outlier neighbors must be at least 1")
    if radius_outlier_radius <= 0:
        raise ValueError("radius outlier radius must be positive")

    input_points = len(pcd.points)
    if input_points < 3:
        raise ValueError("Need at least 3 points for noise filtering")

    effective_statistical_neighbors = min(
        statistical_neighbors,
        input_points - 1,
    )
    statistically_filtered, _ = pcd.remove_statistical_outlier(
        nb_neighbors=effective_statistical_neighbors,
        std_ratio=statistical_std_ratio,
    )
    after_statistical = len(statistically_filtered.points)
    if after_statistical == 0:
        raise ValueError("Statistical outlier filtering removed every point")

    radius_filtered, _ = statistically_filtered.remove_radius_outlier(
        nb_points=radius_outlier_neighbors,
        radius=radius_outlier_radius,
    )
    output_points = len(radius_filtered.points)
    if output_points == 0:
        raise ValueError("Radius outlier filtering removed every point")

    stats = {
        "enabled": True,
        "input_points": input_points,
        "statistical_neighbors": statistical_neighbors,
        "effective_statistical_neighbors": effective_statistical_neighbors,
        "statistical_std_ratio": statistical_std_ratio,
        "after_statistical_points": after_statistical,
        "statistical_removed_points": input_points - after_statistical,
        "radius_outlier_neighbors": radius_outlier_neighbors,
        "radius_outlier_radius": radius_outlier_radius,
        "output_points": output_points,
        "radius_removed_points": after_statistical - output_points,
        "removed_points": input_points - output_points,
    }
    return radius_filtered, stats


def require_finite(name: str, values: np.ndarray) -> None:
    if not np.isfinite(values).all():
        raise ValueError(f"{name} must be finite.")


def normalize(v: np.ndarray, name: str = "vector") -> np.ndarray:
    n = np.linalg.norm(v)
    if n < EPS:
        raise ValueError(f"Cannot normalize near-zero {name}.")
    return v / n


def angle_mod_pi(theta: float) -> float:
    theta = theta % math.pi
    if theta < 0:
        theta += math.pi
    return theta


def angle_diff_mod_pi(a: float, b: float) -> float:
    """Smallest angle difference for unoriented 2D lines."""
    d = abs((a - b + math.pi / 2.0) % math.pi - math.pi / 2.0)
    return d


def line_from_two_points(p1: np.ndarray, p2: np.ndarray) -> tuple[np.ndarray, float] | None:
    d = p2 - p1
    norm = np.linalg.norm(d)
    if norm < EPS:
        return None
    d = d / norm
    n = np.array([-d[1], d[0]], dtype=float)
    c = -float(np.dot(n, p1))
    return n, c


def distances_to_line(xy: np.ndarray, normal: np.ndarray, c: float) -> np.ndarray:
    return np.abs(xy @ normal + c)


def fit_line_pca(xy: np.ndarray) -> Line2D:
    """Refit a line to inlier points using PCA/SVD."""
    if len(xy) < 2:
        raise ValueError("Need at least two points to fit a line.")

    centroid = np.mean(xy, axis=0)
    centered = xy - centroid

    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    direction = normalize(vh[0], "line direction")

    # Keep line orientation canonical.
    theta = angle_mod_pi(math.atan2(direction[1], direction[0]))
    direction = np.array([math.cos(theta), math.sin(theta)], dtype=float)

    normal = np.array([-direction[1], direction[0]], dtype=float)
    c = -float(np.dot(normal, centroid))

    projections = centered @ direction
    p0 = centroid + np.min(projections) * direction
    p1 = centroid + np.max(projections) * direction
    length = float(np.linalg.norm(p1 - p0))

    score = float(len(xy) * max(length, EPS))
    return Line2D(
        normal=normal,
        c=c,
        direction=direction,
        point=centroid,
        p0=p0,
        p1=p1,
        theta=theta,
        length=length,
        inlier_count=int(len(xy)),
        score=score,
    )


def ransac_detect_lines(
    xy: np.ndarray,
    threshold: float,
    min_inliers: int,
    max_lines: int,
    iterations: int,
    seed: int,
) -> list[Line2D]:
    """
    Iteratively detect 2D lines with RANSAC.

    This intentionally avoids OpenCV so the script stays close to your current
    Open3D + NumPy workflow.
    """
    if len(xy) < min_inliers:
        return []

    rng = np.random.default_rng(seed)
    remaining = xy.copy()
    lines: list[Line2D] = []

    for line_idx in range(max_lines):
        if len(remaining) < min_inliers:
            break

        best_inliers_mask = None
        best_count = 0

        for _ in range(iterations):
            ids = rng.choice(len(remaining), size=2, replace=False)
            p1, p2 = remaining[ids[0]], remaining[ids[1]]
            model = line_from_two_points(p1, p2)
            if model is None:
                continue

            normal, c = model
            dists = distances_to_line(remaining, normal, c)
            mask = dists < threshold
            count = int(np.count_nonzero(mask))

            if count > best_count:
                best_count = count
                best_inliers_mask = mask

        if best_inliers_mask is None or best_count < min_inliers:
            break

        inliers = remaining[best_inliers_mask]
        refined = fit_line_pca(inliers)

        # Reject very short line segments, even if dense.
        if refined.length > threshold * 8.0:
            lines.append(refined)

        # Remove inliers and keep finding other walls/edges.
        remaining = remaining[~best_inliers_mask]

        print(
            f"Detected line {line_idx + 1}: "
            f"inliers={best_count}, length={refined.length:.3f}, "
            f"theta={math.degrees(refined.theta):.2f} deg"
        )

    return lines


def estimate_manhattan_base_angle(lines: list[Line2D]) -> float:
    """
    Estimate dominant building orientation.

    Lines separated by 90 degrees are considered equivalent, so use 4*theta
    circular averaging.
    """
    if not lines:
        raise ValueError("No lines available for Manhattan angle estimation.")

    weights = np.array([max(line.score, EPS) for line in lines], dtype=float)
    thetas = np.array([line.theta for line in lines], dtype=float)

    s = np.sum(weights * np.sin(4.0 * thetas))
    c = np.sum(weights * np.cos(4.0 * thetas))
    base = 0.25 * math.atan2(s, c)

    # Canonical modulo 90 degrees.
    base = base % (math.pi / 2.0)
    return base


def line_group(line: Line2D, base_angle: float) -> str:
    """
    Classify a detected line as parallel to the base axis or the perpendicular axis.
    """
    d0 = angle_diff_mod_pi(line.theta, base_angle)
    d1 = angle_diff_mod_pi(line.theta, base_angle + math.pi / 2.0)
    return "x_parallel" if d0 <= d1 else "y_parallel"


def intersect_lines(l1: Line2D, l2: Line2D) -> np.ndarray | None:
    A = np.vstack([l1.normal, l2.normal])
    b = -np.array([l1.c, l2.c], dtype=float)

    det = np.linalg.det(A)
    if abs(det) < 1e-6:
        return None

    return np.linalg.solve(A, b)


def generate_corner_candidates(
    lines: list[Line2D],
    base_angle: float,
    angle_tolerance_deg: float,
) -> list[CornerCandidate]:
    tol = math.radians(angle_tolerance_deg)
    x_lines: list[tuple[int, Line2D]] = []
    y_lines: list[tuple[int, Line2D]] = []

    for i, line in enumerate(lines):
        group = line_group(line, base_angle)
        expected = base_angle if group == "x_parallel" else base_angle + math.pi / 2.0
        if angle_diff_mod_pi(line.theta, expected) <= tol:
            if group == "x_parallel":
                x_lines.append((i, line))
            else:
                y_lines.append((i, line))

    candidates: list[CornerCandidate] = []
    for i, lx in x_lines:
        for j, ly in y_lines:
            xy = intersect_lines(lx, ly)
            if xy is None or not np.all(np.isfinite(xy)):
                continue

            angle_quality = 1.0 - min(
                angle_diff_mod_pi(lx.theta, ly.theta + math.pi / 2.0) / max(tol, EPS),
                1.0,
            )
            score = float((lx.score + ly.score) * max(angle_quality, 0.1))
            candidates.append(CornerCandidate(xy=xy, score=score, line_i=i, line_j=j))

    return candidates


def merge_close_candidates(
    candidates: list[CornerCandidate],
    radius: float,
) -> list[CornerCandidate]:
    if not candidates:
        return []

    candidates = sorted(candidates, key=lambda c: c.score, reverse=True)
    merged: list[CornerCandidate] = []

    for cand in candidates:
        duplicate = False
        for existing in merged:
            if np.linalg.norm(cand.xy - existing.xy) < radius:
                # Keep existing coordinate from the stronger candidate,
                # but accumulate score so repeated intersections help ranking.
                existing.score += cand.score
                duplicate = True
                break
        if not duplicate:
            merged.append(cand)

    return sorted(merged, key=lambda c: c.score, reverse=True)


def best_axes_for_origin(
    xy_points: np.ndarray,
    origin_xy: np.ndarray,
    base_angle: float,
    positive_margin: float,
) -> tuple[float, np.ndarray, np.ndarray, float]:
    """
    Choose X-axis direction such that most points lie in +X/+Y from the origin.

    We try four possibilities:
      base, -base, base+90, -(base+90)
    and keep the one that makes the point cloud mostly positive in local frame.
    """
    candidate_angles = [
        base_angle,
        base_angle + math.pi,
        base_angle + math.pi / 2.0,
        base_angle + 3.0 * math.pi / 2.0,
    ]

    best = None
    centered = xy_points - origin_xy

    for angle in candidate_angles:
        x_axis = np.array([math.cos(angle), math.sin(angle)], dtype=float)
        # For z=[0,0,1], y = z cross x => [-x_y, x_x].
        y_axis = np.array([-x_axis[1], x_axis[0]], dtype=float)

        local_x = centered @ x_axis
        local_y = centered @ y_axis

        positive_mask = (local_x >= -positive_margin) & (local_y >= -positive_margin)
        positive_ratio = float(np.mean(positive_mask))

        # Penalize cases where a lot of data falls far negative.
        negative_extent = max(
            0.0,
            -float(np.percentile(local_x, 1.0)),
            -float(np.percentile(local_y, 1.0)),
        )

        score = positive_ratio * 1000.0 - negative_extent

        if best is None or score > best[0]:
            best = (score, x_axis, y_axis, positive_ratio, angle)

    assert best is not None
    _, x_axis, y_axis, positive_ratio, angle = best
    return angle, x_axis, y_axis, positive_ratio


def rank_origin_candidates(
    xy_points: np.ndarray,
    candidates: list[CornerCandidate],
    base_angle: float,
    positive_margin: float,
) -> list[RankedOrigin]:
    ranked: list[RankedOrigin] = []

    if not candidates:
        return ranked

    max_line_score = max(c.score for c in candidates)

    for cand in candidates:
        angle, x_axis, y_axis, positive_ratio = best_axes_for_origin(
            xy_points=xy_points,
            origin_xy=cand.xy,
            base_angle=base_angle,
            positive_margin=positive_margin,
        )

        # Combine geometric line confidence and quadrant/origin plausibility.
        line_score_norm = cand.score / max(max_line_score, EPS)
        total_score = positive_ratio * 10.0 + line_score_norm

        ranked.append(
            RankedOrigin(
                xy=cand.xy,
                score=float(total_score),
                line_score=float(cand.score),
                positive_ratio=float(positive_ratio),
                x_axis_angle_rad=float(angle),
                x_axis_2d=x_axis,
                y_axis_2d=y_axis,
                source_candidate=cand,
            )
        )

    return sorted(ranked, key=lambda r: r.score, reverse=True)


def build_global_to_local_transform(
    origin: np.ndarray,
    x_axis: np.ndarray,
    z_axis: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Build transform T such that:

        local_point_h = T @ global_point_h

    The rotation rows are the local frame axes expressed in the global frame.
    """
    z_axis = normalize(z_axis.astype(float), "z_axis")
    x_axis = normalize(x_axis.astype(float), "x_axis")

    # Critical: remove any accidental vertical component from X.
    x_axis = x_axis - np.dot(x_axis, z_axis) * z_axis
    x_axis = normalize(x_axis, "x_axis projected perpendicular to z_axis")

    # Right-handed frame. For z=[0,0,1] and x=[1,0,0], this gives y=[0,1,0].
    y_axis = np.cross(z_axis, x_axis)
    y_axis = normalize(y_axis, "y_axis")

    R = np.vstack([x_axis, y_axis, z_axis])
    t = -R @ origin

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T, x_axis, y_axis, z_axis


def compute_diagnostics(
    T: np.ndarray,
    x_axis: np.ndarray,
    y_axis: np.ndarray,
    z_axis: np.ndarray,
) -> dict[str, Any]:
    R = T[:3, :3]
    return {
        "det_R": float(np.linalg.det(R)),
        "dot_x_y": float(np.dot(x_axis, y_axis)),
        "dot_x_z": float(np.dot(x_axis, z_axis)),
        "dot_y_z": float(np.dot(y_axis, z_axis)),
    }


def save_debug_plot(
    xy_points: np.ndarray,
    lines: list[Line2D],
    ranked: list[RankedOrigin],
    selected_index: int,
    output_path: str | Path,
    max_plot_points: int,
    seed: int,
) -> None:
    rng = np.random.default_rng(seed)

    if len(xy_points) > max_plot_points:
        ids = rng.choice(len(xy_points), size=max_plot_points, replace=False)
        plot_points = xy_points[ids]
    else:
        plot_points = xy_points

    plt.figure(figsize=(11, 9))
    plt.scatter(plot_points[:, 0], plot_points[:, 1], s=1, alpha=0.35, label="XY points")

    for idx, line in enumerate(lines):
        plt.plot(
            [line.p0[0], line.p1[0]],
            [line.p0[1], line.p1[1]],
            linewidth=2,
            label=f"line {idx}" if idx < 5 else None,
        )

    top_to_show = min(12, len(ranked))
    for i in range(top_to_show):
        r = ranked[i]
        plt.scatter([r.xy[0]], [r.xy[1]], s=40)
        plt.text(r.xy[0], r.xy[1], f" {i}", fontsize=10)

    if ranked:
        selected = ranked[selected_index]
        ox, oy = selected.xy
        scale = max(
            np.ptp(xy_points[:, 0]),
            np.ptp(xy_points[:, 1]),
            1.0,
        ) * 0.08

        plt.scatter([ox], [oy], s=100, marker="x", label="selected origin")
        plt.arrow(
            ox,
            oy,
            selected.x_axis_2d[0] * scale,
            selected.x_axis_2d[1] * scale,
            width=scale * 0.015,
            length_includes_head=True,
        )
        plt.text(
            ox + selected.x_axis_2d[0] * scale,
            oy + selected.x_axis_2d[1] * scale,
            " +X",
            fontsize=11,
        )

        plt.arrow(
            ox,
            oy,
            selected.y_axis_2d[0] * scale,
            selected.y_axis_2d[1] * scale,
            width=scale * 0.015,
            length_includes_head=True,
        )
        plt.text(
            ox + selected.y_axis_2d[0] * scale,
            oy + selected.y_axis_2d[1] * scale,
            " +Y",
            fontsize=11,
        )

    plt.gca().set_aspect("equal", adjustable="box")
    plt.grid(True)
    plt.xlabel("Global X")
    plt.ylabel("Global Y")
    plt.title("Detected wall lines, ranked corner candidates, and selected local frame")
    plt.legend(loc="best", fontsize=8)
    plt.tight_layout()
    plt.savefig(output_path, dpi=180)
    plt.close()


def make_ranked_origin_from_manual_click(
    xy_points: np.ndarray,
    origin_xy: np.ndarray,
    base_angle: float,
    positive_margin: float,
    x_axis_hint_xy: np.ndarray | None = None,
) -> RankedOrigin:
    """
    Build a RankedOrigin from a manually clicked origin.

    If x_axis_hint_xy is provided, the vector from origin -> hint is used as +X.
    Otherwise, +X is selected from the detected Manhattan axes so that most of
    the cloud lies in the positive local X/Y quadrant.
    """
    if x_axis_hint_xy is not None:
        x_axis = normalize(x_axis_hint_xy - origin_xy, "manual x-axis hint")
        angle = math.atan2(x_axis[1], x_axis[0])
        y_axis = np.array([-x_axis[1], x_axis[0]], dtype=float)

        centered = xy_points - origin_xy
        local_x = centered @ x_axis
        local_y = centered @ y_axis
        positive_mask = (local_x >= -positive_margin) & (local_y >= -positive_margin)
        positive_ratio = float(np.mean(positive_mask))
    else:
        angle, x_axis, y_axis, positive_ratio = best_axes_for_origin(
            xy_points=xy_points,
            origin_xy=origin_xy,
            base_angle=base_angle,
            positive_margin=positive_margin,
        )

    fake_candidate = CornerCandidate(
        xy=origin_xy,
        score=0.0,
        line_i=-1,
        line_j=-1,
    )

    return RankedOrigin(
        xy=origin_xy,
        score=9999.0,
        line_score=0.0,
        positive_ratio=positive_ratio,
        x_axis_angle_rad=float(angle),
        x_axis_2d=x_axis,
        y_axis_2d=y_axis,
        source_candidate=fake_candidate,
    )


def pick_origin_from_topdown_view(
    xy_points: np.ndarray,
    lines: list[Line2D],
    ranked: list[RankedOrigin],
    base_angle: float,
    positive_margin: float,
    max_plot_points: int,
    seed: int,
    pick_x_axis: bool,
) -> RankedOrigin:
    """
    Interactive 2D top-down origin picker using Matplotlib.

    This opens a regular Matplotlib window. Click once for the origin.
    If pick_x_axis=True, click a second point in the desired +X direction.
    """
    rng = np.random.default_rng(seed)

    if len(xy_points) > max_plot_points:
        ids = rng.choice(len(xy_points), size=max_plot_points, replace=False)
        plot_points = xy_points[ids]
    else:
        plot_points = xy_points

    fig, ax = plt.subplots(figsize=(11, 9))
    ax.scatter(plot_points[:, 0], plot_points[:, 1], s=1, alpha=0.35, label="XY points")

    for idx, line in enumerate(lines):
        ax.plot(
            [line.p0[0], line.p1[0]],
            [line.p0[1], line.p1[1]],
            linewidth=2,
            label=f"line {idx}" if idx < 5 else None,
        )

    top_to_show = min(12, len(ranked))
    for i in range(top_to_show):
        r = ranked[i]
        ax.scatter([r.xy[0]], [r.xy[1]], s=35)
        ax.text(r.xy[0], r.xy[1], f" {i}", fontsize=10)

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.set_xlabel("Global X")
    ax.set_ylabel("Global Y")

    title = "Click the desired ORIGIN point"
    if pick_x_axis:
        title += ", then click a second point in the +X direction"
    ax.set_title(title)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()

    print("\n--- Manual top-down origin selection ---")
    print("A Matplotlib window should open.")
    print("Click the desired origin point on the top-down map.")
    if pick_x_axis:
        print("Then click a second point in the desired positive X direction.")
    print("Close the plot only after the click selection is complete.")

    needed_clicks = 2 if pick_x_axis else 1
    clicked = plt.ginput(needed_clicks, timeout=0)
    plt.close(fig)

    if len(clicked) < needed_clicks:
        raise RuntimeError(
            f"Expected {needed_clicks} click(s), but only received {len(clicked)}."
        )

    origin_xy = np.array(clicked[0], dtype=float)
    x_axis_hint = np.array(clicked[1], dtype=float) if pick_x_axis else None

    selected = make_ranked_origin_from_manual_click(
        xy_points=xy_points,
        origin_xy=origin_xy,
        base_angle=base_angle,
        positive_margin=positive_margin,
        x_axis_hint_xy=x_axis_hint,
    )

    print(f"Manual origin selected: x={origin_xy[0]:.6f}, y={origin_xy[1]:.6f}")
    if pick_x_axis:
        print(
            "Manual +X hint selected: "
            f"x={x_axis_hint[0]:.6f}, y={x_axis_hint[1]:.6f}, "
            f"x-axis angle={math.degrees(selected.x_axis_angle_rad):.3f} deg"
        )
    else:
        print(
            "X/Y axes inferred from detected Manhattan directions; "
            f"x-axis angle={math.degrees(selected.x_axis_angle_rad):.3f} deg"
        )

    return selected


def percentile_origin_z(
    z_values: np.ndarray,
    percentile: float,
    offset: float,
) -> tuple[float, dict[str, Any]]:
    z = float(np.percentile(z_values, percentile)) + float(offset)
    info = {
        "method": "percentile",
        "percentile": float(percentile),
        "offset": float(offset),
        "z_before_offset": float(np.percentile(z_values, percentile)),
        "z": z,
    }
    return z, info


def fit_ground_plane_near_floor(
    points: np.ndarray,
    distance_threshold: float,
    ransac_iterations: int,
    max_points: int,
    lower_percentile: float,
    upper_percentile: float,
    min_normal_z: float,
    seed: int,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """
    Fit a floor/ground plane from the lower slice of the point cloud.

    Returns:
        plane coefficients [a, b, c, d] for ax + by + cz + d = 0
        unit normal, oriented so normal[2] >= 0
        diagnostic info
    """
    if len(points) < 3:
        raise ValueError("Need at least 3 points to fit a ground plane.")

    z = points[:, 2]
    z_low = float(np.percentile(z, lower_percentile))
    z_high = float(np.percentile(z, upper_percentile))

    if z_high <= z_low:
        z_high = z_low + 1e-3

    candidate_mask = (z >= z_low) & (z <= z_high)
    candidates = points[candidate_mask]

    # Fallback if the lower slice is too small.
    if len(candidates) < 200:
        candidates = points

    if len(candidates) > max_points:
        rng = np.random.default_rng(seed)
        ids = rng.choice(len(candidates), size=max_points, replace=False)
        candidates = candidates[ids]

    floor_pcd = o3d.geometry.PointCloud()
    floor_pcd.points = o3d.utility.Vector3dVector(candidates)

    plane_model, inliers = floor_pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=ransac_iterations,
    )

    coeffs = np.asarray(plane_model, dtype=float)
    normal_norm = np.linalg.norm(coeffs[:3])
    if normal_norm < EPS:
        raise ValueError("Ground plane fit returned a near-zero normal.")

    coeffs = coeffs / normal_norm

    # Flip so +Z component is positive.
    if coeffs[2] < 0:
        coeffs *= -1.0

    normal = coeffs[:3]
    normal_z = float(abs(normal[2]))

    if normal_z < min_normal_z:
        raise ValueError(
            f"Detected plane normal is not floor-like enough: |normal_z|={normal_z:.3f}. "
            f"Try lowering --floor-min-normal-z or use --origin-z manually."
        )

    info = {
        "method": "ground-plane",
        "plane_coefficients": coeffs.tolist(),
        "normal": normal.tolist(),
        "normal_z_abs": normal_z,
        "inlier_count": int(len(inliers)),
        "candidate_point_count": int(len(candidates)),
        "z_slice_low": z_low,
        "z_slice_high": z_high,
        "distance_threshold": float(distance_threshold),
        "ransac_iterations": int(ransac_iterations),
        "lower_percentile": float(lower_percentile),
        "upper_percentile": float(upper_percentile),
    }

    return coeffs, normal, info


def z_on_plane_at_xy(plane_coeffs: np.ndarray, xy: np.ndarray) -> float:
    a, b, c, d = [float(v) for v in plane_coeffs]
    if abs(c) < EPS:
        raise ValueError("Cannot compute Z from a near-vertical plane.")
    x, y = float(xy[0]), float(xy[1])
    return float(-(a * x + b * y + d) / c)


def resolve_origin_z_and_z_axis(
    origin_z_arg: str,
    z_values: np.ndarray,
    downsampled_points: np.ndarray,
    origin_xy: np.ndarray,
    auto_method: str,
    z_axis_source: str,
    origin_z_offset: float,
    origin_z_percentile: float,
    floor_distance_threshold: float,
    floor_ransac_iterations: int,
    floor_max_points: int,
    floor_lower_percentile: float,
    floor_upper_percentile: float,
    floor_min_normal_z: float,
    seed: int,
) -> tuple[float, np.ndarray, dict[str, Any]]:
    """
    Resolve origin Z and the local Z axis.

    If --origin-z is numeric, it wins. Otherwise:
      - ground-plane: fit floor plane and evaluate it at clicked origin XY
      - percentile: use percentile of all Z values

    z_axis_source controls whether local Z is global [0,0,1] or the fitted
    ground-plane normal.
    """
    if origin_z_arg.lower() != "auto":
        try:
            z = float(origin_z_arg) + float(origin_z_offset)
        except ValueError as exc:
            raise ValueError("--origin-z must be 'auto' or a numeric value") from exc

        info = {
            "method": "manual",
            "origin_z_arg": origin_z_arg,
            "offset": float(origin_z_offset),
            "z": z,
        }
        return z, np.array([0.0, 0.0, 1.0], dtype=float), info

    if auto_method == "percentile":
        z, info = percentile_origin_z(
            z_values=z_values,
            percentile=origin_z_percentile,
            offset=origin_z_offset,
        )
        return z, np.array([0.0, 0.0, 1.0], dtype=float), info

    if auto_method != "ground-plane":
        raise ValueError(f"Unknown origin Z auto method: {auto_method}")

    try:
        plane_coeffs, ground_normal, info = fit_ground_plane_near_floor(
            points=downsampled_points,
            distance_threshold=floor_distance_threshold,
            ransac_iterations=floor_ransac_iterations,
            max_points=floor_max_points,
            lower_percentile=floor_lower_percentile,
            upper_percentile=floor_upper_percentile,
            min_normal_z=floor_min_normal_z,
            seed=seed,
        )
        z_before_offset = z_on_plane_at_xy(plane_coeffs, origin_xy)
        z = z_before_offset + float(origin_z_offset)
        info["z_before_offset"] = float(z_before_offset)
        info["offset"] = float(origin_z_offset)
        info["z"] = float(z)

        if z_axis_source == "ground-plane":
            z_axis = ground_normal
        elif z_axis_source == "world":
            z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
        else:
            raise ValueError(f"Unknown z-axis source: {z_axis_source}")

        return float(z), z_axis, info

    except Exception as exc:
        print(f"Warning: ground-plane origin Z estimation failed: {exc}")
        print("Falling back to percentile-based origin Z.")
        z, info = percentile_origin_z(
            z_values=z_values,
            percentile=origin_z_percentile,
            offset=origin_z_offset,
        )
        info["fallback_reason"] = str(exc)
        return z, np.array([0.0, 0.0, 1.0], dtype=float), info


def create_axis_lineset(
    origin: np.ndarray,
    x_axis: np.ndarray,
    y_axis: np.ndarray,
    z_axis: np.ndarray,
    axis_length: float,
) -> o3d.geometry.LineSet:
    """
    Create a simple RGB axis overlay in the original/global point-cloud frame.

    X = red, Y = green, Z = blue.
    """
    origin = np.asarray(origin, dtype=float)
    x_end = origin + normalize(x_axis, "x_axis for visualization") * axis_length
    y_end = origin + normalize(y_axis, "y_axis for visualization") * axis_length
    z_end = origin + normalize(z_axis, "z_axis for visualization") * axis_length

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector([origin, x_end, y_end, z_end])
    line_set.lines = o3d.utility.Vector2iVector([[0, 1], [0, 2], [0, 3]])
    line_set.colors = o3d.utility.Vector3dVector(
        [
            [1.0, 0.0, 0.0],  # X red
            [0.0, 1.0, 0.0],  # Y green
            [0.0, 0.0, 1.0],  # Z blue
        ]
    )
    return line_set


def create_origin_marker(origin: np.ndarray, radius: float) -> o3d.geometry.TriangleMesh:
    marker = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    marker.translate(origin)
    marker.paint_uniform_color([1.0, 1.0, 0.0])
    marker.compute_vertex_normals()
    return marker


def show_3d_axis_quality_check(
    pcd: o3d.geometry.PointCloud,
    origin: np.ndarray,
    x_axis: np.ndarray,
    y_axis: np.ndarray,
    z_axis: np.ndarray,
    axis_length: float,
    origin_marker_radius: float,
) -> None:
    """
    Open a quick 3D view showing the selected local frame in the original cloud.

    This visualizes the chosen coordinate frame before/alongside applying the
    global-to-local transform:
      - yellow sphere = selected origin
      - red line      = +X
      - green line    = +Y
      - blue line     = +Z
    """
    axis_lines = create_axis_lineset(
        origin=origin,
        x_axis=x_axis,
        y_axis=y_axis,
        z_axis=z_axis,
        axis_length=axis_length,
    )
    origin_marker = create_origin_marker(origin, origin_marker_radius)

    print("\n--- 3D axis quality-check view ---")
    print("Yellow sphere = selected origin")
    print("Red = +X, Green = +Y, Blue = +Z")
    print("Close the Open3D window to continue/exit.")

    o3d.visualization.draw_geometries(
        [pcd, axis_lines, origin_marker],
        window_name="3D Quality Check: Selected Local Frame",
    )


def load_and_prepare_points(
    path: str | Path,
    voxel_size: float,
    noise_filter: bool,
    statistical_neighbors: int,
    statistical_std_ratio: float,
    radius_outlier_neighbors: int,
    radius_outlier_radius: float,
    height_min: float | None,
    height_max: float | None,
    max_detection_points: int,
    seed: int,
) -> tuple[Any, np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
    pcd = o3d.io.read_point_cloud(str(path))
    if not pcd.has_points():
        raise ValueError(f"No points found in {path}")

    raw_points = np.asarray(pcd.points)
    finite_mask = np.isfinite(raw_points).all(axis=1)
    finite_points = filter_finite_points(raw_points)
    if len(finite_points) != len(raw_points):
        removed_count = len(raw_points) - len(finite_points)
        pcd = pcd.select_by_index(np.flatnonzero(finite_mask).tolist())
        print(f"Removed {removed_count} non-finite points from {path}")

    print(f"Loaded {len(pcd.points)} finite points from {path}")

    if voxel_size > 0:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    else:
        pcd_down = pcd

    points_after_downsampling = len(pcd_down.points)
    if noise_filter:
        print("\n--- Removing sparse point-cloud outliers ---")
        pcd_down, noise_filter_stats = filter_noise_points(
            pcd=pcd_down,
            statistical_neighbors=statistical_neighbors,
            statistical_std_ratio=statistical_std_ratio,
            radius_outlier_neighbors=radius_outlier_neighbors,
            radius_outlier_radius=radius_outlier_radius,
        )
        print(
            "Statistical filter removed "
            f"{noise_filter_stats['statistical_removed_points']} points"
        )
        print(
            "Radius filter removed "
            f"{noise_filter_stats['radius_removed_points']} additional points"
        )
        print(
            f"Retained {noise_filter_stats['output_points']} / "
            f"{noise_filter_stats['input_points']} downsampled points"
        )
    else:
        noise_filter_stats = {
            "enabled": False,
            "input_points": points_after_downsampling,
            "output_points": points_after_downsampling,
            "removed_points": 0,
        }
        print("Noise filtering disabled")

    points = filter_finite_points(np.asarray(pcd_down.points))
    print(f"Using {len(points)} points for alignment after preprocessing")

    all_z = points[:, 2].copy()

    mask = np.ones(len(points), dtype=bool)
    if height_min is not None:
        mask &= points[:, 2] >= height_min
    if height_max is not None:
        mask &= points[:, 2] <= height_max

    filtered = points[mask]
    if len(filtered) == 0:
        raise ValueError(
            "No points left after height filtering. "
            "Adjust --height-min/--height-max."
        )

    print(f"Using {len(filtered)} points after height filtering")

    xy = filtered[:, :2]

    if len(xy) > max_detection_points:
        rng = np.random.default_rng(seed)
        ids = rng.choice(len(xy), size=max_detection_points, replace=False)
        xy_detection = xy[ids]
        print(f"Subsampled to {len(xy_detection)} XY points for line detection")
    else:
        xy_detection = xy

    return pcd_down, points, all_z, xy_detection, noise_filter_stats


def numpy_to_list(obj: Any) -> Any:
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.float32, np.float64)):
        return float(obj)
    if isinstance(obj, (np.int32, np.int64)):
        return int(obj)
    if isinstance(obj, dict):
        return {k: numpy_to_list(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [numpy_to_list(v) for v in obj]
    return obj


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Align a point cloud to a user-defined local map frame."
    )
    parser.add_argument(
        "pointcloud",
        help="Input point cloud file, for example .pcd, .ply, .xyz",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.05,
        help="Voxel downsample size in meters. Use 0 to disable.",
    )
    parser.add_argument(
        "--statistical-neighbors",
        type=int,
        default=20,
        help="Neighbors used by statistical outlier removal (default: 20).",
    )
    parser.add_argument(
        "--statistical-std-ratio",
        type=float,
        default=2.0,
        help="Statistical outlier standard-deviation ratio (default: 2.0).",
    )
    parser.add_argument(
        "--radius-outlier-neighbors",
        type=int,
        default=3,
        help="Minimum neighbors inside the outlier radius (default: 3).",
    )
    parser.add_argument(
        "--radius-outlier-radius",
        type=float,
        default=0.30,
        help=(
            "Radius used for sparse outlier removal in meters (default: 0.30)."
        ),
    )
    parser.add_argument(
        "--no-noise-filter",
        dest="noise_filter",
        action="store_false",
        default=True,
        help="Disable statistical and radius outlier removal.",
    )
    parser.add_argument(
        "--ransac-threshold",
        type=float,
        default=0.08,
        help="Line inlier threshold in meters.",
    )
    parser.add_argument(
        "--max-lines",
        type=int,
        default=16,
        help="Maximum wall/edge lines to detect.",
    )
    parser.add_argument(
        "--no-show-3d-qc",
        dest="show_3d_qc",
        action="store_false",
        default=True,
        help="Disable the Open3D 3D axis quality-check view.",
    )

    args = parser.parse_args()
    if o3d is None:
        parser.error("Open3D is required; install it with 'pip install open3d'")

    # Internal defaults intentionally not exposed as CLI arguments.
    # Keep the command line simple; adjust these constants here only if needed.
    args.height_min = None
    args.height_max = None
    args.max_detection_points = 60000
    args.ransac_iterations = 1200
    args.min_inliers = 250
    args.angle_tolerance_deg = 15.0
    args.candidate_merge_radius = 0.25
    args.origin_candidate = 0

    # Manual top-down origin + manual positive-X selection are now the default workflow.
    args.manual_origin = True
    args.manual_x_axis = True

    # QC view defaults requested.
    args.axis_length = 5.0
    args.origin_marker_radius = 0.04

    # Origin Z / floor snapping defaults.
    args.origin_z = "auto"
    args.origin_z_auto_method = "ground-plane"
    args.origin_z_percentile = 10.0
    args.origin_z_offset = 0.0
    args.z_axis_source = "world"
    args.floor_distance_threshold = 0.06
    args.floor_ransac_iterations = 2000
    args.floor_max_points = 120000
    args.floor_lower_percentile = 1.0
    args.floor_upper_percentile = 35.0
    args.floor_min_normal_z = 0.65

    # Ranking / output / reproducibility defaults.
    args.positive_margin = 0.20
    args.output_transform = "estimated_transform.json"
    args.output_pcd = "transformed.pcd"
    args.debug_plot = "transform_debug.png"
    args.max_plot_points = 30000
    args.seed = 7

    random.seed(args.seed)
    np.random.seed(args.seed)

    (
        pcd_down,
        downsampled_points,
        z_values,
        xy_detection,
        noise_filter_stats,
    ) = load_and_prepare_points(
        path=args.pointcloud,
        voxel_size=args.voxel_size,
        noise_filter=args.noise_filter,
        statistical_neighbors=args.statistical_neighbors,
        statistical_std_ratio=args.statistical_std_ratio,
        radius_outlier_neighbors=args.radius_outlier_neighbors,
        radius_outlier_radius=args.radius_outlier_radius,
        height_min=args.height_min,
        height_max=args.height_max,
        max_detection_points=args.max_detection_points,
        seed=args.seed,
    )

    print("\n--- Detecting wall/edge lines with RANSAC ---")
    lines = ransac_detect_lines(
        xy=xy_detection,
        threshold=args.ransac_threshold,
        min_inliers=args.min_inliers,
        max_lines=args.max_lines,
        iterations=args.ransac_iterations,
        seed=args.seed,
    )

    if len(lines) < 2:
        raise RuntimeError(
            "Not enough lines detected. Try increasing --ransac-threshold, "
            "lowering --min-inliers, changing --height-min/--height-max, "
            "or increasing --voxel-size."
        )

    base_angle = estimate_manhattan_base_angle(lines)
    print(f"\nEstimated Manhattan base angle: {math.degrees(base_angle):.3f} deg")

    print("\n--- Generating corner candidates ---")
    raw_candidates = generate_corner_candidates(
        lines=lines,
        base_angle=base_angle,
        angle_tolerance_deg=args.angle_tolerance_deg,
    )

    candidates = merge_close_candidates(
        candidates=raw_candidates,
        radius=args.candidate_merge_radius,
    )

    if not candidates:
        raise RuntimeError(
            "No corner candidates found from line intersections. "
            "Try increasing --angle-tolerance-deg or detecting more lines."
        )

    ranked = rank_origin_candidates(
        xy_points=xy_detection,
        candidates=candidates,
        base_angle=base_angle,
        positive_margin=args.positive_margin,
    )

    print("\nTop ranked automatic origin candidates:")
    print(" idx |        x         y   | score  | +quadrant | x-axis deg | lines")
    print("-----+----------------------+--------+-----------+------------+----------")
    for i, r in enumerate(ranked[:12]):
        print(
            f"{i:4d} | {r.xy[0]:9.3f} {r.xy[1]:9.3f} | "
            f"{r.score:6.3f} | {r.positive_ratio:9.3f} | "
            f"{math.degrees(r.x_axis_angle_rad):10.3f} | "
            f"{r.source_candidate.line_i},{r.source_candidate.line_j}"
        )

    manual_selected = False
    if args.manual_x_axis and not args.manual_origin:
        print("Note: --manual-x-axis is ignored because --no-manual-origin was set.")

    if args.manual_origin:
        selected = pick_origin_from_topdown_view(
            xy_points=xy_detection,
            lines=lines,
            ranked=ranked,
            base_angle=base_angle,
            positive_margin=args.positive_margin,
            max_plot_points=args.max_plot_points,
            seed=args.seed,
            pick_x_axis=args.manual_x_axis,
        )
        manual_selected = True
    else:
        if args.origin_candidate < 0 or args.origin_candidate >= len(ranked):
            raise ValueError(
                f"--origin-candidate must be between 0 and {len(ranked) - 1}, "
                f"got {args.origin_candidate}"
            )
        selected = ranked[args.origin_candidate]
    origin_z, z_axis_3d, origin_z_info = resolve_origin_z_and_z_axis(
        origin_z_arg=args.origin_z,
        z_values=z_values,
        downsampled_points=downsampled_points,
        origin_xy=selected.xy,
        auto_method=args.origin_z_auto_method,
        z_axis_source=args.z_axis_source,
        origin_z_offset=args.origin_z_offset,
        origin_z_percentile=args.origin_z_percentile,
        floor_distance_threshold=args.floor_distance_threshold,
        floor_ransac_iterations=args.floor_ransac_iterations,
        floor_max_points=args.floor_max_points,
        floor_lower_percentile=args.floor_lower_percentile,
        floor_upper_percentile=args.floor_upper_percentile,
        floor_min_normal_z=args.floor_min_normal_z,
        seed=args.seed,
    )

    origin = np.array([selected.xy[0], selected.xy[1], origin_z], dtype=float)
    x_axis_3d = np.array([selected.x_axis_2d[0], selected.x_axis_2d[1], 0.0], dtype=float)

    T, x_axis_3d, y_axis_3d, z_axis_3d = build_global_to_local_transform(
        origin=origin,
        x_axis=x_axis_3d,
        z_axis=z_axis_3d,
    )

    require_finite("origin", origin)
    require_finite("x axis", x_axis_3d)
    require_finite("y axis", y_axis_3d)
    require_finite("z axis", z_axis_3d)
    require_finite("transform", T)
    diagnostics = compute_diagnostics(T, x_axis_3d, y_axis_3d, z_axis_3d)

    print("\n--- Selected local frame ---")
    print(f"Origin: {origin}")
    print(
        f"Origin Z method: {origin_z_info.get('method')} -> "
        f"z={origin_z_info.get('z'):.6f}"
    )
    if origin_z_info.get("method") == "ground-plane":
        print(f"Ground plane normal: {origin_z_info.get('normal')}")
        print(
            f"Ground plane inliers: {origin_z_info.get('inlier_count')} / "
            f"candidates: {origin_z_info.get('candidate_point_count')}"
        )
    print(f"X axis: {x_axis_3d}")
    print(f"Y axis: {y_axis_3d}")
    print(f"Z axis: {z_axis_3d}")
    print("\n--- Global-to-local transform ---")
    np.set_printoptions(precision=8, suppress=True)
    print(T)

    print("\n--- Diagnostics ---")
    for k, v in diagnostics.items():
        print(f"{k}: {v:.8f}")

    if args.show_3d_qc:
        show_3d_axis_quality_check(
            pcd=pcd_down,
            origin=origin,
            x_axis=x_axis_3d,
            y_axis=y_axis_3d,
            z_axis=z_axis_3d,
            axis_length=args.axis_length,
            origin_marker_radius=args.origin_marker_radius,
        )

    output = {
        "frame": "global_to_local",
        "description": "local_point_h = matrix @ global_point_h",
        "source_pointcloud": str(args.pointcloud),
        "origin": origin,
        "origin_z_info": origin_z_info,
        "x_axis": x_axis_3d,
        "y_axis": y_axis_3d,
        "z_axis": z_axis_3d,
        "matrix": T,
        "manual_origin_used": manual_selected,
        "manual_x_axis_used": bool(args.manual_x_axis and manual_selected),
        "selected_origin_candidate": None if manual_selected else args.origin_candidate,
        "selected_origin_positive_ratio": selected.positive_ratio,
        "selected_origin_line_score": selected.line_score,
        "show_3d_qc": bool(args.show_3d_qc),
        "axis_length": float(args.axis_length),
        "origin_marker_radius": float(args.origin_marker_radius),
        "manhattan_base_angle_degrees": math.degrees(base_angle),
        "noise_filter": noise_filter_stats,
        "diagnostics": diagnostics,
        "parameters": vars(args),
    }

    with open(args.output_transform, "w", encoding="utf-8") as f:
        json.dump(numpy_to_list(output), f, indent=2, allow_nan=False)

    print(f"\nSaved transform JSON to: {args.output_transform}")

    if args.output_pcd:
        pcd_out = pcd_down
        pcd_out.transform(T)
        ok = o3d.io.write_point_cloud(args.output_pcd, pcd_out)
        if not ok:
            raise RuntimeError(f"Failed to write transformed point cloud: {args.output_pcd}")
        print(f"Saved transformed point cloud to: {args.output_pcd}")

    if args.debug_plot:
        debug_ranked = [selected] + ranked if manual_selected else ranked
        debug_selected_index = 0 if manual_selected else args.origin_candidate

        save_debug_plot(
            xy_points=xy_detection,
            lines=lines,
            ranked=debug_ranked,
            selected_index=debug_selected_index,
            output_path=args.debug_plot,
            max_plot_points=args.max_plot_points,
            seed=args.seed,
        )
        print(f"Saved debug plot to: {args.debug_plot}")

    print("\nDone.")
    print(
        "Review the 3D QC view and debug plot. If the selected origin or X "
        "axis is wrong, rerun and click them again."
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
