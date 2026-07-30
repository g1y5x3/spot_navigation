"""Ground-plane estimation for map-frame alignment."""

from __future__ import annotations

from typing import Any

import numpy as np

from map_tools.alignment import EPS
from map_tools.pointcloud import o3d


def percentile_origin_z(
    z_values: np.ndarray,
    percentile: float,
    offset: float,
) -> tuple[float, dict[str, Any]]:
    z_before_offset = float(np.percentile(z_values, percentile))
    z = z_before_offset + float(offset)
    return z, {
        "method": "percentile",
        "percentile": float(percentile),
        "offset": float(offset),
        "z_before_offset": z_before_offset,
        "z": z,
    }


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
    """Fit a floor plane from the lower slice of a point cloud."""
    if o3d is None:
        raise RuntimeError("Open3D is required for ground-plane fitting")
    if len(points) < 3:
        raise ValueError("Need at least 3 points to fit a ground plane.")

    z_values = points[:, 2]
    z_low, z_high = np.percentile(
        z_values,
        [lower_percentile, upper_percentile],
    )
    z_low = float(z_low)
    z_high = max(float(z_high), z_low + 1e-3)
    candidates = points[(z_values >= z_low) & (z_values <= z_high)]
    if len(candidates) < 200:
        candidates = points
    if len(candidates) > max_points:
        ids = np.random.default_rng(seed).choice(
            len(candidates),
            size=max_points,
            replace=False,
        )
        candidates = candidates[ids]

    floor_pcd = o3d.geometry.PointCloud()
    floor_pcd.points = o3d.utility.Vector3dVector(candidates)
    plane_model, inliers = floor_pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=ransac_iterations,
    )

    coefficients = np.asarray(plane_model, dtype=float)
    normal_norm = np.linalg.norm(coefficients[:3])
    if normal_norm < EPS:
        raise ValueError("Ground plane fit returned a near-zero normal.")
    coefficients = coefficients / normal_norm
    if coefficients[2] < 0:
        coefficients *= -1.0

    normal = coefficients[:3]
    normal_z = float(abs(normal[2]))
    if normal_z < min_normal_z:
        raise ValueError(
            "Detected plane is not sufficiently floor-like: "
            f"|normal_z|={normal_z:.3f}; adjust the internal floor constants."
        )

    return coefficients, normal, {
        "method": "ground-plane",
        "plane_coefficients": coefficients.tolist(),
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


def z_on_plane_at_xy(plane_coefficients: np.ndarray, xy: np.ndarray) -> float:
    a, b, c, d = [float(value) for value in plane_coefficients]
    if abs(c) < EPS:
        raise ValueError("Cannot compute Z from a near-vertical plane.")
    return float(-(a * float(xy[0]) + b * float(xy[1]) + d) / c)


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
    if origin_z_arg.lower() != "auto":
        try:
            z = float(origin_z_arg) + float(origin_z_offset)
        except ValueError as error:
            raise ValueError(
                "internal origin_z must be 'auto' or numeric"
            ) from error
        return z, np.array([0.0, 0.0, 1.0]), {
            "method": "manual",
            "origin_z_arg": origin_z_arg,
            "offset": float(origin_z_offset),
            "z": z,
        }

    if auto_method == "percentile":
        z, info = percentile_origin_z(
            z_values,
            origin_z_percentile,
            origin_z_offset,
        )
        return z, np.array([0.0, 0.0, 1.0]), info
    if auto_method != "ground-plane":
        raise ValueError(f"Unknown origin Z auto method: {auto_method}")

    try:
        coefficients, ground_normal, info = fit_ground_plane_near_floor(
            downsampled_points,
            floor_distance_threshold,
            floor_ransac_iterations,
            floor_max_points,
            floor_lower_percentile,
            floor_upper_percentile,
            floor_min_normal_z,
            seed,
        )
        z_before_offset = z_on_plane_at_xy(coefficients, origin_xy)
        z = z_before_offset + float(origin_z_offset)
        info.update(
            z_before_offset=float(z_before_offset),
            offset=float(origin_z_offset),
            z=float(z),
        )
        if z_axis_source == "ground-plane":
            z_axis = ground_normal
        elif z_axis_source == "world":
            z_axis = np.array([0.0, 0.0, 1.0])
        else:
            raise ValueError(f"Unknown z-axis source: {z_axis_source}")
        return float(z), z_axis, info
    except Exception as error:
        print(f"Warning: ground-plane origin Z estimation failed: {error}")
        print("Falling back to percentile-based origin Z.")
        z, info = percentile_origin_z(
            z_values,
            origin_z_percentile,
            origin_z_offset,
        )
        info["fallback_reason"] = str(error)
        return z, np.array([0.0, 0.0, 1.0]), info
