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
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from map_tools.outputs import (  # noqa: E402
    alignment_output_paths,
    far_output_paths,
)
from map_tools.pointcloud import add_alignment_options  # noqa: E402


_ALIGNMENT_VALUE_OPTIONS = (
    ("--voxel-size", "voxel_size"),
    ("--statistical-neighbors", "statistical_neighbors"),
    ("--statistical-std-ratio", "statistical_std_ratio"),
    ("--radius-outlier-neighbors", "radius_outlier_neighbors"),
    ("--radius-outlier-radius", "radius_outlier_radius"),
    ("--ransac-threshold", "ransac_threshold"),
    ("--max-lines", "max_lines"),
)


def parse_args(
    argv: Sequence[str] | None = None,
) -> tuple[argparse.ArgumentParser, argparse.Namespace, list[str]]:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if "--" in arguments:
        separator = arguments.index("--")
        wrapper_arguments = arguments[:separator]
        build_arguments = arguments[separator + 1:]
    else:
        wrapper_arguments, build_arguments = arguments, []

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
    add_alignment_options(parser)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print both commands and output paths without running them",
    )
    return parser, parser.parse_args(wrapper_arguments), build_arguments


def output_paths(
    output_dir: Path,
    name: str,
    include_preview: bool = True,
) -> list[Path]:
    alignment = alignment_output_paths(output_dir, name)
    far = far_output_paths(output_dir, f"{name}_transformed")
    paths = [
        alignment.pointcloud,
        alignment.transform,
        alignment.debug_plot,
        far.visibility_graph,
        far.boundary,
        far.trajectory,
        far.stats,
    ]
    if include_preview:
        paths.append(far.preview)
    return paths


def _alignment_command(
    pointcloud: Path,
    args: argparse.Namespace,
) -> list[str]:
    command = [
        sys.executable,
        str(SCRIPT_DIR / "align_pointcloud_to_map_frame.py"),
        str(pointcloud),
    ]
    for flag, attribute in _ALIGNMENT_VALUE_OPTIONS:
        command.extend((flag, str(getattr(args, attribute))))
    if not args.show_3d_qc:
        command.append("--no-show-3d-qc")
    if not args.noise_filter:
        command.append("--no-noise-filter")
    return command


def main(argv: Sequence[str] | None = None) -> int:
    parser, args, build_arguments = parse_args(argv)
    pointcloud = args.pointcloud.resolve()
    output_dir = args.output_dir.resolve()
    name = args.name or pointcloud.stem
    if not name or Path(name).name != name:
        parser.error("--name must be a single non-empty filename stem")

    transformed_name = f"{name}_transformed"
    alignment = alignment_output_paths(output_dir, name)
    default_alignment = alignment_output_paths(output_dir)
    align_command = _alignment_command(pointcloud, args)
    build_command = [
        sys.executable,
        str(SCRIPT_DIR / "build_far_prior_map.py"),
        str(alignment.pointcloud),
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
        print(f"  aligned PCD: {alignment.pointcloud}")
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
    result = subprocess.run(align_command, cwd=output_dir, check=False)
    if result.returncode != 0:
        return result.returncode

    for source, destination in zip(
        (
            default_alignment.pointcloud,
            default_alignment.transform,
            default_alignment.debug_plot,
        ),
        (
            alignment.pointcloud,
            alignment.transform,
            alignment.debug_plot,
        ),
    ):
        if not source.is_file():
            print(
                f"ERROR: alignment did not produce {source}",
                file=sys.stderr,
            )
            return 1
        source.replace(destination)

    print("[2/2] Build FAR prior map", flush=True)
    print(f"$ {shlex.join(build_command)}", flush=True)
    result = subprocess.run(build_command, cwd=output_dir, check=False)
    if result.returncode != 0:
        return result.returncode

    print("Outputs:")
    for path in outputs:
        print(f"  {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
