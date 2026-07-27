from pathlib import Path

import numpy as np
from pypcd4 import PointCloud


def load_pcd_xyz(path: Path) -> np.ndarray:
    cloud = PointCloud.from_path(path)
    missing_fields = {"x", "y", "z"} - set(cloud.fields)
    if missing_fields:
        raise ValueError(f"{path} must contain x, y, z fields")
    xyz = np.asarray(cloud.numpy(("x", "y", "z")), dtype=np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]
