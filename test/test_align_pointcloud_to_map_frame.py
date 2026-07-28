import importlib.util
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest


SCRIPT_PATH = (
    Path(__file__).parents[1] / "scripts/align_pointcloud_to_map_frame.py"
)
SPEC = importlib.util.spec_from_file_location(
    "align_pointcloud_to_map_frame", SCRIPT_PATH
)
assert SPEC is not None and SPEC.loader is not None
map_frame_alignment = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = map_frame_alignment
SPEC.loader.exec_module(map_frame_alignment)


def test_help_does_not_require_optional_open3d(tmp_path: Path) -> None:
    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), "--help"],
        cwd=tmp_path,
        capture_output=True,
        check=False,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "Align a point cloud to a user-defined local map frame" in result.stdout
    assert "--statistical-neighbors" in result.stdout
    assert "--statistical-std-ratio" in result.stdout
    assert "--radius-outlier-neighbors" in result.stdout
    assert "--radius-outlier-radius" in result.stdout
    assert "--no-noise-filter" in result.stdout


def test_non_finite_points_are_removed_before_processing() -> None:
    points = np.asarray(
        [[1.0, 2.0, 3.0], [np.nan, 2.0, 3.0], [4.0, np.inf, 6.0]]
    )

    filtered = map_frame_alignment.filter_finite_points(points)

    np.testing.assert_array_equal(filtered, points[:1])
    with pytest.raises(ValueError, match="No finite points"):
        map_frame_alignment.filter_finite_points(points[1:])


def test_non_finite_transform_is_rejected() -> None:
    with pytest.raises(ValueError, match="transform must be finite"):
        map_frame_alignment.require_finite(
            "transform", np.asarray([[1.0, np.nan]])
        )


@pytest.mark.skipif(
    map_frame_alignment.o3d is None,
    reason="Open3D unavailable",
)
def test_noise_filter_removes_isolated_points_from_dense_geometry() -> None:
    axis = np.arange(5, dtype=np.float64) * 0.04
    dense_points = np.stack(
        np.meshgrid(axis, axis, axis, indexing="ij"),
        axis=-1,
    ).reshape(-1, 3)
    isolated_points = np.asarray([[2.0, 2.0, 2.0], [-2.0, -2.0, -2.0]])
    points = np.vstack([dense_points, isolated_points])
    pcd = map_frame_alignment.o3d.geometry.PointCloud()
    pcd.points = map_frame_alignment.o3d.utility.Vector3dVector(points)

    filtered, stats = map_frame_alignment.filter_noise_points(
        pcd,
        statistical_neighbors=8,
        statistical_std_ratio=2.0,
        radius_outlier_neighbors=3,
        radius_outlier_radius=0.12,
    )

    filtered_points = np.asarray(filtered.points)
    assert len(filtered_points) < len(points)
    assert len(filtered_points) >= 100
    assert np.max(np.linalg.norm(filtered_points, axis=1)) < 1.0
    assert stats["input_points"] == len(points)
    assert stats["output_points"] == len(filtered_points)
    assert stats["removed_points"] == len(points) - len(filtered_points)
