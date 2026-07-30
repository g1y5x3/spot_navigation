"""Matplotlib and Open3D interaction for point-cloud alignment."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from map_tools.alignment import (
    Line2D,
    RankedOrigin,
    make_ranked_origin_from_manual_click,
    normalize,
)
from map_tools.pointcloud import o3d


def _sample_plot_points(
    xy_points: np.ndarray,
    max_plot_points: int,
    seed: int,
) -> np.ndarray:
    if len(xy_points) <= max_plot_points:
        return xy_points
    ids = np.random.default_rng(seed).choice(
        len(xy_points),
        size=max_plot_points,
        replace=False,
    )
    return xy_points[ids]


def _draw_topdown_context(
    axes: Any,
    xy_points: np.ndarray,
    lines: list[Line2D],
    ranked: list[RankedOrigin],
    candidate_marker_size: int,
) -> None:
    axes.scatter(
        xy_points[:, 0],
        xy_points[:, 1],
        s=1,
        alpha=0.35,
        label="XY points",
    )
    for index, line in enumerate(lines):
        axes.plot(
            [line.p0[0], line.p1[0]],
            [line.p0[1], line.p1[1]],
            linewidth=2,
            label=f"line {index}" if index < 5 else None,
        )
    for index, candidate in enumerate(ranked[:12]):
        axes.scatter(
            [candidate.xy[0]],
            [candidate.xy[1]],
            s=candidate_marker_size,
        )
        axes.text(
            candidate.xy[0],
            candidate.xy[1],
            f" {index}",
            fontsize=10,
        )


def save_debug_plot(
    xy_points: np.ndarray,
    lines: list[Line2D],
    ranked: list[RankedOrigin],
    selected_index: int,
    output_path: str | Path,
    max_plot_points: int,
    seed: int,
) -> None:
    plot_points = _sample_plot_points(xy_points, max_plot_points, seed)
    figure, axes = plt.subplots(figsize=(11, 9))
    _draw_topdown_context(axes, plot_points, lines, ranked, 40)

    if ranked:
        selected = ranked[selected_index]
        origin_x, origin_y = selected.xy
        scale = max(
            np.ptp(xy_points[:, 0]),
            np.ptp(xy_points[:, 1]),
            1.0,
        ) * 0.08
        axes.scatter(
            [origin_x],
            [origin_y],
            s=100,
            marker="x",
            label="selected origin",
        )
        for axis, label in (
            (selected.x_axis_2d, " +X"),
            (selected.y_axis_2d, " +Y"),
        ):
            axes.arrow(
                origin_x,
                origin_y,
                axis[0] * scale,
                axis[1] * scale,
                width=scale * 0.015,
                length_includes_head=True,
            )
            axes.text(
                origin_x + axis[0] * scale,
                origin_y + axis[1] * scale,
                label,
                fontsize=11,
            )

    axes.set_aspect("equal", adjustable="box")
    axes.grid(True)
    axes.set_xlabel("Global X")
    axes.set_ylabel("Global Y")
    axes.set_title(
        "Detected wall lines, ranked corner candidates, "
        "and selected local frame"
    )
    axes.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


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
    """Open a top-down picker for local origin and optional positive X."""
    plot_points = _sample_plot_points(xy_points, max_plot_points, seed)
    figure, axes = plt.subplots(figsize=(11, 9))
    _draw_topdown_context(axes, plot_points, lines, ranked, 35)
    axes.set_aspect("equal", adjustable="box")
    axes.grid(True)
    axes.set_xlabel("Global X")
    axes.set_ylabel("Global Y")
    title = "Click the desired ORIGIN point"
    if pick_x_axis:
        title += ", then click a second point in the +X direction"
    axes.set_title(title)
    axes.legend(loc="best", fontsize=8)
    figure.tight_layout()

    print("\n--- Manual top-down origin selection ---")
    print("A Matplotlib window should open.")
    print("Click the desired origin point on the top-down map.")
    if pick_x_axis:
        print("Then click a second point in the desired positive X direction.")
    print("Close the plot only after the click selection is complete.")

    needed_clicks = 2 if pick_x_axis else 1
    clicked = plt.ginput(needed_clicks, timeout=0)
    plt.close(figure)
    if len(clicked) < needed_clicks:
        raise RuntimeError(
            f"Expected {needed_clicks} click(s), but only received "
            f"{len(clicked)}."
        )

    origin_xy = np.array(clicked[0], dtype=float)
    x_axis_hint = np.array(clicked[1], dtype=float) if pick_x_axis else None
    selected = make_ranked_origin_from_manual_click(
        xy_points,
        origin_xy,
        base_angle,
        positive_margin,
        x_axis_hint,
    )

    print(
        f"Manual origin selected: x={origin_xy[0]:.6f}, "
        f"y={origin_xy[1]:.6f}"
    )
    if x_axis_hint is not None:
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


def create_axis_lineset(
    origin: np.ndarray,
    x_axis: np.ndarray,
    y_axis: np.ndarray,
    z_axis: np.ndarray,
    axis_length: float,
) -> Any:
    if o3d is None:
        raise RuntimeError("Open3D is required for the 3D quality check")
    endpoints = [
        origin,
        origin + normalize(x_axis, "x_axis for visualization") * axis_length,
        origin + normalize(y_axis, "y_axis for visualization") * axis_length,
        origin + normalize(z_axis, "z_axis for visualization") * axis_length,
    ]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(endpoints)
    line_set.lines = o3d.utility.Vector2iVector([[0, 1], [0, 2], [0, 3]])
    line_set.colors = o3d.utility.Vector3dVector(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    )
    return line_set


def create_origin_marker(origin: np.ndarray, radius: float) -> Any:
    if o3d is None:
        raise RuntimeError("Open3D is required for the 3D quality check")
    marker = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    marker.translate(origin)
    marker.paint_uniform_color([1.0, 1.0, 0.0])
    marker.compute_vertex_normals()
    return marker


def show_3d_axis_quality_check(
    pcd: Any,
    origin: np.ndarray,
    x_axis: np.ndarray,
    y_axis: np.ndarray,
    z_axis: np.ndarray,
    axis_length: float,
    origin_marker_radius: float,
) -> None:
    if o3d is None:
        raise RuntimeError("Open3D is required for the 3D quality check")
    axis_lines = create_axis_lineset(
        origin,
        x_axis,
        y_axis,
        z_axis,
        axis_length,
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
