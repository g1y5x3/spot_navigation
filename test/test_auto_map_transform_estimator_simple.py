import importlib.util
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest


SCRIPT_PATH = (
    Path(__file__).parents[1] / "scripts/auto_map_transform_estimator_simple.py"
)
SPEC = importlib.util.spec_from_file_location("auto_map_transform", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
auto_map_transform = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = auto_map_transform
SPEC.loader.exec_module(auto_map_transform)


def test_help_does_not_require_optional_open3d(tmp_path: Path) -> None:
    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), "--help"],
        cwd=tmp_path,
        capture_output=True,
        check=False,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "point-cloud map transform estimator" in result.stdout


def test_non_finite_points_are_removed_before_processing() -> None:
    points = np.asarray(
        [[1.0, 2.0, 3.0], [np.nan, 2.0, 3.0], [4.0, np.inf, 6.0]]
    )

    filtered = auto_map_transform.filter_finite_points(points)

    np.testing.assert_array_equal(filtered, points[:1])
    with pytest.raises(ValueError, match="No finite points"):
        auto_map_transform.filter_finite_points(points[1:])


def test_non_finite_transform_is_rejected() -> None:
    with pytest.raises(ValueError, match="transform must be finite"):
        auto_map_transform.require_finite(
            "transform", np.asarray([[1.0, np.nan]])
        )
