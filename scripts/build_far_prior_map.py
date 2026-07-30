#!/usr/bin/env python3

"""Build FAR Planner prior-map assets from a PCD point cloud."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import cast

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from map_tools.far_contours import (  # noqa: E402,F401
    extract_boundary_polygons,
    shoelace_area as shoelace_area,
)
from map_tools.far_grid import (  # noqa: E402,F401
    choose_free_point,
    classify_obstacle_points,
    select_free_space_component as select_free_space_component,
)
from map_tools.outputs import (  # noqa: E402
    far_output_paths,
    read_trajectory_xy,
    write_boundary_ply,
    write_preview,
    write_trajectory,
    write_vgh,
)
from map_tools.pointcloud import load_pcd_xyz  # noqa: E402
from map_tools.visibility_graph import (  # noqa: E402,F401
    FreeDirection as FreeDirection,
    build_visibility_graph,
    point_inside_polygon as point_inside_polygon,
    segments_intersect as segments_intersect,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Build FAR boundary, preview, and VGH prior-map files from a PCD."
        )
    )
    parser.add_argument("pcd", type=Path, help="Input PCD file")
    parser.add_argument("--output-dir", type=Path, help="Output directory")
    parser.add_argument("--name", help="Output stem; defaults to the PCD stem")
    parser.add_argument(
        "--resolution",
        type=float,
        default=0.15,
        help="2D grid resolution in meters",
    )
    parser.add_argument(
        "--height-mode",
        choices=("local", "absolute", "pmf"),
        default="local",
        help=(
            "Classify obstacles by local height-above-ground, absolute z, "
            "or PMF-style opening"
        ),
    )
    parser.add_argument(
        "--min-z",
        type=float,
        default=0.2,
        help="Minimum point z for absolute mode",
    )
    parser.add_argument(
        "--max-z",
        type=float,
        default=2.0,
        help="Maximum point z for absolute mode",
    )
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
    parser.add_argument(
        "--boundary-z",
        type=float,
        default=0.75,
        help="Z value written to boundary vertices",
    )
    parser.add_argument(
        "--padding",
        type=float,
        default=0.5,
        help="Outer map padding in meters",
    )
    parser.add_argument(
        "--close-radius",
        type=float,
        default=0.25,
        help="Morphological close radius in meters",
    )
    parser.add_argument(
        "--inflate-radius",
        type=float,
        default=0.10,
        help="Obstacle inflation radius in meters",
    )
    parser.add_argument(
        "--simplify",
        type=float,
        default=0.25,
        help="Contour simplification in meters",
    )
    parser.add_argument(
        "--min-area",
        type=float,
        default=0.25,
        help="Minimum obstacle polygon area in m^2",
    )
    parser.add_argument(
        "--max-polygons",
        type=int,
        default=80,
        help=(
            "Maximum obstacle polygons retained by area; raise for large maps"
        ),
    )
    parser.add_argument(
        "--max-vertices",
        type=int,
        default=80,
        help="Maximum vertices per obstacle polygon",
    )
    parser.add_argument(
        "--free-point",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help=(
            "Known traversable point; selects free-space component "
            "and VGH side"
        ),
    )
    parser.add_argument(
        "--clear-trajectory-csv",
        type=Path,
        help=(
            "CSV/whitespace file containing x,y trajectory points to clear "
            "from obstacle occupancy"
        ),
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
    parser.add_argument(
        "--no-preview",
        action="store_true",
        help="Skip PNG preview generation",
    )
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
            f"Height filter kept only {len(projected)} points. "
            "Adjust height threshold options."
        )

    clear_trajectory_xy = None
    if args.clear_trajectory_csv is not None:
        clear_trajectory_xy = read_trajectory_xy(
            args.clear_trajectory_csv.resolve()
        )
        if len(clear_trajectory_xy) == 0:
            raise RuntimeError(
                f"No finite x,y trajectory points in "
                f"{args.clear_trajectory_csv}"
            )

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

    stats.update(
        input_pcd=str(pcd_path),
        height_filter=height_stats,
        points_total=int(len(points)),
        points_after_height_filter=int(len(projected)),
        free_point=[round(float(value), 6) for value in free_point],
        preview_scale=(
            None if args.no_preview else max(int(args.preview_scale), 1)
        ),
    )
    paths = far_output_paths(output_dir, name)
    vertices = write_boundary_ply(
        paths.boundary,
        polygons,
        args.boundary_z,
        has_outer_boundary=not args.no_outer_boundary,
    )
    write_trajectory(paths.trajectory, free_point)
    graph = build_visibility_graph(polygons, free_point, args.boundary_z)
    graph_nodes = write_vgh(paths.visibility_graph, graph)
    stats["written_vertices"] = vertices
    stats["visibility_graph"] = {
        "nodes": graph_nodes,
        "edges": sum(len(node.connections) for node in graph) // 2,
        "contour_edges": (
            sum(len(node.contour_connections) for node in graph) // 2
        ),
    }
    paths.stats.write_text(
        json.dumps(stats, indent=2) + "\n",
        encoding="utf-8",
    )
    if not args.no_preview:
        write_preview(
            paths.preview,
            occupancy,
            polygons,
            origin,
            args.resolution,
            free_point,
            args.preview_scale,
            clear_trajectory_xy,
            not args.no_outer_boundary,
        )

    written_paths = [
        paths.boundary,
        paths.trajectory,
        paths.visibility_graph,
        paths.stats,
    ]
    if not args.no_preview:
        written_paths.append(paths.preview)
    for path in written_paths:
        print(f"Wrote {path}")

    discarded = cast(int, stats["discarded_obstacle_polygons"])
    if discarded > 0:
        print(
            f"Warning: discarded {discarded} smaller obstacle polygons; "
            "increase --max-polygons to retain more."
        )


if __name__ == "__main__":
    main()
