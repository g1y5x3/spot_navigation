#!/usr/bin/env python3
"""Run point-cloud alignment followed by FAR prior-map generation."""

from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
from pathlib import Path
from typing import Sequence


SCRIPT_DIR = Path(__file__).resolve().parent


def parse_args(
    argv: Sequence[str] | None = None,
) -> tuple[argparse.ArgumentParser, argparse.Namespace, list[str]]:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if "--" in arguments:
        separator = arguments.index("--")
        wrapper_arguments = arguments[:separator]
        build_arguments = arguments[separator + 1:]
    else:
        wrapper_arguments = arguments
        build_arguments = []

    parser = argparse.ArgumentParser(
        description=(
            "Interactively align a point cloud, then build its FAR boundary "
            "and prior visibility graph."
        ),
        epilog=(
            "Arguments after '--' are passed unchanged to "
            "build_far_prior_map.py. Run that script with --help for its full "
            "option list."
        ),
    )
    parser.add_argument("pointcloud", type=Path, help="Raw input point cloud")
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="Directory for the aligned PCD and FAR map assets",
    )
    parser.add_argument(
        "--name",
        help=(
            "Base output stem; defaults to the raw point-cloud stem and gets "
            "a _transformed suffix for the aligned PCD and FAR assets"
        ),
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.05,
        help="Alignment voxel downsample size in meters",
    )
    parser.add_argument(
        "--statistical-neighbors",
        type=int,
        default=20,
        help="Neighbors used by alignment statistical outlier removal",
    )
    parser.add_argument(
        "--statistical-std-ratio",
        type=float,
        default=2.0,
        help="Alignment statistical outlier standard-deviation ratio",
    )
    parser.add_argument(
        "--radius-outlier-neighbors",
        type=int,
        default=3,
        help="Minimum neighbors used by alignment radius outlier removal",
    )
    parser.add_argument(
        "--radius-outlier-radius",
        type=float,
        default=0.30,
        help="Alignment radius outlier search distance in meters",
    )
    parser.add_argument(
        "--no-noise-filter",
        dest="noise_filter",
        action="store_false",
        default=True,
        help="Disable alignment statistical and radius outlier removal",
    )
    parser.add_argument(
        "--ransac-threshold",
        type=float,
        default=0.08,
        help="Alignment line-inlier threshold in meters",
    )
    parser.add_argument(
        "--max-lines",
        type=int,
        default=16,
        help="Maximum wall/edge lines detected during alignment",
    )
    parser.add_argument(
        "--no-show-3d-qc",
        dest="show_3d_qc",
        action="store_false",
        default=True,
        help="Skip the alignment tool's final Open3D quality-control view",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help=(
            "Print both underlying commands and output paths without running "
            "them"
        ),
    )
    return parser, parser.parse_args(wrapper_arguments), build_arguments


def output_paths(
    output_dir: Path,
    name: str,
    include_preview: bool = True,
) -> list[Path]:
    transformed_name = f"{name}_transformed"
    paths = [
        output_dir / f"{transformed_name}.pcd",
        output_dir / f"{name}_transform.json",
        output_dir / f"{name}_transform_debug.png",
        output_dir / f"{transformed_name}.vgh",
        output_dir / f"{transformed_name}_boundary.ply",
        output_dir / f"{transformed_name}_trajectory.txt",
        output_dir / f"{transformed_name}_boundary_stats.json",
    ]
    if include_preview:
        paths.append(output_dir / f"{transformed_name}_boundary_preview.png")
    return paths


def main(argv: Sequence[str] | None = None) -> int:
    parser, args, build_arguments = parse_args(argv)
    pointcloud = args.pointcloud.resolve()
    output_dir = args.output_dir.resolve()
    name = args.name or pointcloud.stem
    if not name or Path(name).name != name:
        parser.error("--name must be a single non-empty filename stem")

    transformed_name = f"{name}_transformed"
    aligned_pcd = output_dir / f"{transformed_name}.pcd"
    align_command = [
        sys.executable,
        str(SCRIPT_DIR / "align_pointcloud_to_map_frame.py"),
        str(pointcloud),
        "--voxel-size",
        str(args.voxel_size),
        "--statistical-neighbors",
        str(args.statistical_neighbors),
        "--statistical-std-ratio",
        str(args.statistical_std_ratio),
        "--radius-outlier-neighbors",
        str(args.radius_outlier_neighbors),
        "--radius-outlier-radius",
        str(args.radius_outlier_radius),
        "--ransac-threshold",
        str(args.ransac_threshold),
        "--max-lines",
        str(args.max_lines),
    ]
    if not args.show_3d_qc:
        align_command.append("--no-show-3d-qc")
    if not args.noise_filter:
        align_command.append("--no-noise-filter")

    build_command = [
        sys.executable,
        str(SCRIPT_DIR / "build_far_prior_map.py"),
        str(aligned_pcd),
        "--output-dir",
        str(output_dir),
        "--name",
        transformed_name,
        *build_arguments,
    ]
    outputs = output_paths(
        output_dir,
        name,
        include_preview="--no-preview" not in build_arguments,
    )

    if args.dry_run:
        print("[1/2] Align point cloud")
        print(f"$ {shlex.join(align_command)}")
        print(f"  aligned PCD: {aligned_pcd}")
        print("[2/2] Build FAR prior map")
        print(f"$ {shlex.join(build_command)}")
        print("Outputs:")
        for path in outputs:
            print(f"  {path}")
        return 0

    if not pointcloud.is_file():
        parser.error(f"point cloud does not exist: {pointcloud}")
    output_dir.mkdir(parents=True, exist_ok=True)

    print("[1/2] Align point cloud", flush=True)
    print(f"$ {shlex.join(align_command)}", flush=True)
    align_result = subprocess.run(align_command, cwd=output_dir, check=False)
    if align_result.returncode != 0:
        return align_result.returncode

    alignment_outputs = {
        output_dir / "transformed.pcd": aligned_pcd,
        output_dir / "estimated_transform.json": (
            output_dir / f"{name}_transform.json"
        ),
        output_dir / "transform_debug.png": (
            output_dir / f"{name}_transform_debug.png"
        ),
    }
    for source, destination in alignment_outputs.items():
        if not source.is_file():
            print(
                f"ERROR: alignment did not produce {source}",
                file=sys.stderr,
            )
            return 1
        source.replace(destination)

    print("[2/2] Build FAR prior map", flush=True)
    print(f"$ {shlex.join(build_command)}", flush=True)
    build_result = subprocess.run(build_command, cwd=output_dir, check=False)
    if build_result.returncode != 0:
        return build_result.returncode

    print("Outputs:")
    for path in outputs:
        print(f"  {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
