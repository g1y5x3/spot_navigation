#!/usr/bin/env python3
"""Interactively align a point cloud to a local map frame."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from map_tools.alignment import (  # noqa: E402
    build_global_to_local_transform,
    compute_diagnostics,
    estimate_manhattan_base_angle,
    generate_corner_candidates,
    merge_close_candidates,
    rank_origin_candidates,
    ransac_detect_lines,
    require_finite,
)
from map_tools.ground import resolve_origin_z_and_z_axis  # noqa: E402
from map_tools.interaction import (  # noqa: E402
    pick_origin_from_topdown_view,
    save_debug_plot,
    show_3d_axis_quality_check,
)
from map_tools.outputs import numpy_to_list  # noqa: E402
from map_tools.pointcloud import (  # noqa: E402,F401
    add_alignment_options,
    filter_finite_points as filter_finite_points,
    filter_noise_points as filter_noise_points,
    load_and_prepare_points,
    o3d,
)


_INTERNAL_DEFAULTS = {
    "height_min": None,
    "height_max": None,
    "max_detection_points": 60000,
    "ransac_iterations": 1200,
    "min_inliers": 250,
    "angle_tolerance_deg": 15.0,
    "candidate_merge_radius": 0.25,
    "origin_candidate": 0,
    "manual_origin": True,
    "manual_x_axis": True,
    "axis_length": 5.0,
    "origin_marker_radius": 0.04,
    "origin_z": "auto",
    "origin_z_auto_method": "ground-plane",
    "origin_z_percentile": 10.0,
    "origin_z_offset": 0.0,
    "z_axis_source": "world",
    "floor_distance_threshold": 0.06,
    "floor_ransac_iterations": 2000,
    "floor_max_points": 120000,
    "floor_lower_percentile": 1.0,
    "floor_upper_percentile": 35.0,
    "floor_min_normal_z": 0.65,
    "positive_margin": 0.20,
    "output_transform": "estimated_transform.json",
    "output_pcd": "transformed.pcd",
    "debug_plot": "transform_debug.png",
    "max_plot_points": 30000,
    "seed": 7,
}
_INTERNAL = SimpleNamespace(**_INTERNAL_DEFAULTS)


def parse_args() -> tuple[argparse.ArgumentParser, argparse.Namespace]:
    parser = argparse.ArgumentParser(
        description="Align a point cloud to a user-defined local map frame."
    )
    parser.add_argument(
        "pointcloud",
        help="Input point cloud file, for example .pcd, .ply, .xyz",
    )
    add_alignment_options(parser)
    return parser, parser.parse_args()


def _detect_and_rank_origins(
    xy_detection: np.ndarray,
    args: argparse.Namespace,
):
    print("\n--- Detecting wall/edge lines with RANSAC ---")
    lines = ransac_detect_lines(
        xy_detection,
        args.ransac_threshold,
        _INTERNAL.min_inliers,
        args.max_lines,
        _INTERNAL.ransac_iterations,
        _INTERNAL.seed,
    )
    if len(lines) < 2:
        raise RuntimeError(
            "Not enough lines detected. Try increasing --ransac-threshold, "
            "--max-lines, or --voxel-size."
        )

    base_angle = estimate_manhattan_base_angle(lines)
    print(
        f"\nEstimated Manhattan base angle: "
        f"{math.degrees(base_angle):.3f} deg"
    )
    candidates = merge_close_candidates(
        generate_corner_candidates(
            lines,
            base_angle,
            _INTERNAL.angle_tolerance_deg,
        ),
        _INTERNAL.candidate_merge_radius,
    )
    if not candidates:
        raise RuntimeError(
            "No corner candidates found. Try increasing --ransac-threshold "
            "or --max-lines."
        )
    ranked = rank_origin_candidates(
        xy_detection,
        candidates,
        base_angle,
        _INTERNAL.positive_margin,
    )
    return lines, base_angle, ranked


def _print_ranked_origins(ranked) -> None:
    print("\nTop ranked automatic origin candidates:")
    print(
        " idx |        x         y   | score  | +quadrant | "
        "x-axis deg | lines"
    )
    print(
        "-----+----------------------+--------+-----------+"
        "------------+----------"
    )
    for index, origin in enumerate(ranked[:12]):
        print(
            f"{index:4d} | {origin.xy[0]:9.3f} {origin.xy[1]:9.3f} | "
            f"{origin.score:6.3f} | {origin.positive_ratio:9.3f} | "
            f"{math.degrees(origin.x_axis_angle_rad):10.3f} | "
            f"{origin.source_candidate.line_i},"
            f"{origin.source_candidate.line_j}"
        )


def main() -> int:
    parser, args = parse_args()
    if o3d is None:
        parser.error(
            "Open3D is required; install it with 'pip install open3d'"
        )

    (
        pointcloud,
        downsampled_points,
        z_values,
        xy_detection,
        noise_filter_stats,
    ) = load_and_prepare_points(
        args.pointcloud,
        args.voxel_size,
        args.noise_filter,
        args.statistical_neighbors,
        args.statistical_std_ratio,
        args.radius_outlier_neighbors,
        args.radius_outlier_radius,
        _INTERNAL.height_min,
        _INTERNAL.height_max,
        _INTERNAL.max_detection_points,
        _INTERNAL.seed,
    )
    lines, base_angle, ranked = _detect_and_rank_origins(
        xy_detection,
        args,
    )
    _print_ranked_origins(ranked)
    selected = pick_origin_from_topdown_view(
        xy_detection,
        lines,
        ranked,
        base_angle,
        _INTERNAL.positive_margin,
        _INTERNAL.max_plot_points,
        _INTERNAL.seed,
        pick_x_axis=True,
    )

    origin_z, z_axis, origin_z_info = resolve_origin_z_and_z_axis(
        _INTERNAL.origin_z,
        z_values,
        downsampled_points,
        selected.xy,
        _INTERNAL.origin_z_auto_method,
        _INTERNAL.z_axis_source,
        _INTERNAL.origin_z_offset,
        _INTERNAL.origin_z_percentile,
        _INTERNAL.floor_distance_threshold,
        _INTERNAL.floor_ransac_iterations,
        _INTERNAL.floor_max_points,
        _INTERNAL.floor_lower_percentile,
        _INTERNAL.floor_upper_percentile,
        _INTERNAL.floor_min_normal_z,
        _INTERNAL.seed,
    )
    origin = np.array([*selected.xy, origin_z], dtype=float)
    x_axis = np.array([*selected.x_axis_2d, 0.0], dtype=float)
    transform, x_axis, y_axis, z_axis = build_global_to_local_transform(
        origin,
        x_axis,
        z_axis,
    )
    for name, value in (
        ("origin", origin),
        ("x axis", x_axis),
        ("y axis", y_axis),
        ("z axis", z_axis),
        ("transform", transform),
    ):
        require_finite(name, value)
    diagnostics = compute_diagnostics(transform, x_axis, y_axis, z_axis)

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
    print(f"X axis: {x_axis}")
    print(f"Y axis: {y_axis}")
    print(f"Z axis: {z_axis}")
    print("\n--- Global-to-local transform ---")
    np.set_printoptions(precision=8, suppress=True)
    print(transform)
    print("\n--- Diagnostics ---")
    for name, value in diagnostics.items():
        print(f"{name}: {value:.8f}")

    if args.show_3d_qc:
        show_3d_axis_quality_check(
            pointcloud,
            origin,
            x_axis,
            y_axis,
            z_axis,
            _INTERNAL.axis_length,
            _INTERNAL.origin_marker_radius,
        )

    output = {
        "frame": "global_to_local",
        "description": "local_point_h = matrix @ global_point_h",
        "source_pointcloud": str(args.pointcloud),
        "origin": origin,
        "origin_z_info": origin_z_info,
        "x_axis": x_axis,
        "y_axis": y_axis,
        "z_axis": z_axis,
        "matrix": transform,
        "manual_origin_used": True,
        "manual_x_axis_used": True,
        "selected_origin_candidate": None,
        "selected_origin_positive_ratio": selected.positive_ratio,
        "selected_origin_line_score": selected.line_score,
        "show_3d_qc": bool(args.show_3d_qc),
        "axis_length": float(_INTERNAL.axis_length),
        "origin_marker_radius": float(_INTERNAL.origin_marker_radius),
        "manhattan_base_angle_degrees": math.degrees(base_angle),
        "noise_filter": noise_filter_stats,
        "diagnostics": diagnostics,
        "parameters": {**vars(args), **_INTERNAL_DEFAULTS},
    }
    Path(_INTERNAL.output_transform).write_text(
        json.dumps(numpy_to_list(output), indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(f"\nSaved transform JSON to: {_INTERNAL.output_transform}")

    pointcloud.transform(transform)
    if not o3d.io.write_point_cloud(_INTERNAL.output_pcd, pointcloud):
        raise RuntimeError(
            f"Failed to write transformed point cloud: {_INTERNAL.output_pcd}"
        )
    print(f"Saved transformed point cloud to: {_INTERNAL.output_pcd}")

    save_debug_plot(
        xy_detection,
        lines,
        [selected, *ranked],
        0,
        _INTERNAL.debug_plot,
        _INTERNAL.max_plot_points,
        _INTERNAL.seed,
    )
    print(f"Saved debug plot to: {_INTERNAL.debug_plot}")
    print("\nDone.")
    print(
        "Review the 3D QC view and debug plot. If the selected origin or X "
        "axis is wrong, rerun and click them again."
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"\nERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
