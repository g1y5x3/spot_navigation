"""Point-cloud loading, downsampling, and outlier filtering."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np

try:
    import open3d as o3d
except ModuleNotFoundError:
    o3d = None


def add_alignment_options(parser: argparse.ArgumentParser) -> None:
    """Add the alignment options shared by both map entry scripts."""
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
        help="Neighbors used by statistical outlier removal.",
    )
    parser.add_argument(
        "--statistical-std-ratio",
        type=float,
        default=2.0,
        help="Statistical outlier standard-deviation ratio.",
    )
    parser.add_argument(
        "--radius-outlier-neighbors",
        type=int,
        default=3,
        help="Minimum neighbors inside the outlier radius.",
    )
    parser.add_argument(
        "--radius-outlier-radius",
        type=float,
        default=0.30,
        help="Radius used for sparse outlier removal in meters.",
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
        help="Disable the final Open3D axis quality-check view.",
    )


def filter_finite_points(points: np.ndarray) -> np.ndarray:
    finite_mask = np.isfinite(points).all(axis=1)
    if not np.any(finite_mask):
        raise ValueError("No finite points remain in the point cloud.")
    return points[finite_mask]


def load_pcd_xyz(path: Path) -> np.ndarray:
    from pypcd4 import PointCloud

    cloud = PointCloud.from_path(path)
    if {"x", "y", "z"} - set(cloud.fields):
        raise ValueError(f"{path} must contain x, y, z fields")
    xyz = np.asarray(cloud.numpy(("x", "y", "z")), dtype=np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


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

    return radius_filtered, {
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
    if o3d is None:
        raise RuntimeError(
            "Open3D is required to load and prepare point clouds"
        )

    pcd = o3d.io.read_point_cloud(str(path))
    if not pcd.has_points():
        raise ValueError(f"No points found in {path}")

    raw_points = np.asarray(pcd.points)
    finite_mask = np.isfinite(raw_points).all(axis=1)
    finite_count = int(np.count_nonzero(finite_mask))
    if finite_count == 0:
        raise ValueError("No finite points remain in the point cloud.")
    if finite_count != len(raw_points):
        pcd = pcd.select_by_index(np.flatnonzero(finite_mask).tolist())
        print(
            f"Removed {len(raw_points) - finite_count} non-finite points "
            f"from {path}"
        )

    print(f"Loaded {len(pcd.points)} finite points from {path}")
    pcd_down = pcd.voxel_down_sample(voxel_size) if voxel_size > 0 else pcd
    downsampled_count = len(pcd_down.points)

    if noise_filter:
        print("\n--- Removing sparse point-cloud outliers ---")
        pcd_down, filter_stats = filter_noise_points(
            pcd_down,
            statistical_neighbors,
            statistical_std_ratio,
            radius_outlier_neighbors,
            radius_outlier_radius,
        )
        print(
            "Statistical filter removed "
            f"{filter_stats['statistical_removed_points']} points"
        )
        print(
            "Radius filter removed "
            f"{filter_stats['radius_removed_points']} additional points"
        )
        print(
            f"Retained {filter_stats['output_points']} / "
            f"{filter_stats['input_points']} downsampled points"
        )
    else:
        filter_stats = {
            "enabled": False,
            "input_points": downsampled_count,
            "output_points": downsampled_count,
            "removed_points": 0,
        }
        print("Noise filtering disabled")

    points = filter_finite_points(np.asarray(pcd_down.points))
    print(f"Using {len(points)} points for alignment after preprocessing")
    all_z = points[:, 2]

    if height_min is None and height_max is None:
        filtered = points
    else:
        height_mask = np.ones(len(points), dtype=bool)
        if height_min is not None:
            height_mask &= points[:, 2] >= height_min
        if height_max is not None:
            height_mask &= points[:, 2] <= height_max
        filtered = points[height_mask]

    if len(filtered) == 0:
        raise ValueError("No points left after internal height filtering.")
    print(f"Using {len(filtered)} points after height filtering")

    xy = filtered[:, :2]
    if len(xy) > max_detection_points:
        rng = np.random.default_rng(seed)
        ids = rng.choice(len(xy), size=max_detection_points, replace=False)
        xy = xy[ids]
        print(f"Subsampled to {len(xy)} XY points for line detection")

    return pcd_down, points, all_z, xy, filter_stats
