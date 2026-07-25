import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest

_STUBBED_MODULES: list[str] = []


def _install_module(name: str, attributes: dict[str, object]) -> ModuleType:
    module = ModuleType(name)
    for attribute_name, value in attributes.items():
        setattr(module, attribute_name, value)
    sys.modules[name] = module
    _STUBBED_MODULES.append(name)
    return module


if importlib.util.find_spec("rclpy") is None:
    geometry_messages = _install_module("geometry_msgs.msg", {"Point": object})
    _install_module("geometry_msgs", {"msg": geometry_messages})
    _install_module("rclpy", {})
    _install_module("rclpy.node", {"Node": object})
    _install_module(
        "rclpy.qos",
        {
            "DurabilityPolicy": object,
            "QoSProfile": object,
            "ReliabilityPolicy": object,
        },
    )
    visualization_messages = _install_module(
        "visualization_msgs.msg",
        {"Marker": object, "MarkerArray": object},
    )
    _install_module("visualization_msgs", {"msg": visualization_messages})


from spot_navigation.boundary_marker_publisher import (
    load_boundary_ply,
    resolve_boundary_path,
)

for module_name in reversed(_STUBBED_MODULES):
    del sys.modules[module_name]


def _write_ply(path: Path, rows: list[str], vertex_count: int | None = None) -> None:
    count = len(rows) if vertex_count is None else vertex_count
    path.write_text(
        "\n".join(
            [
                "ply",
                "format ascii 1.0",
                f"element vertex {count}",
                "property float x",
                "property float y",
                "property float z",
                "property float poly_index",
                "end_header",
                *rows,
                "",
            ]
        ),
        encoding="ascii",
    )


def test_resolve_boundary_path_uses_explicit_path() -> None:
    assert resolve_boundary_path("~/custom.ply", "/maps/site.pcd") == Path(
        "~/custom.ply"
    ).expanduser()


def test_resolve_boundary_path_infers_from_pcd() -> None:
    assert resolve_boundary_path("", "/maps/site.pcd") == Path(
        "/maps/site_boundary.ply"
    )


def test_resolve_boundary_path_requires_an_input() -> None:
    with pytest.raises(ValueError, match="boundary_file or pcd_file"):
        resolve_boundary_path("", "")


def test_load_boundary_ply_groups_polygons(tmp_path: Path) -> None:
    path = tmp_path / "boundary.ply"
    _write_ply(
        path,
        [
            "0 0 0.75 0",
            "4 0 0.75 0",
            "4 4 0.75 0",
            "1 1 0.75 1",
            "2 1 0.75 1",
            "1 2 0.75 1",
        ],
    )

    polygons = load_boundary_ply(path)

    assert list(polygons) == [0, 1]
    assert polygons[0] == [
        (0.0, 0.0, 0.75),
        (4.0, 0.0, 0.75),
        (4.0, 4.0, 0.75),
    ]
    assert len(polygons[1]) == 3


def test_load_boundary_ply_rejects_non_ascii_format(tmp_path: Path) -> None:
    path = tmp_path / "boundary.ply"
    path.write_text(
        "ply\nformat binary_little_endian 1.0\nelement vertex 0\nend_header\n",
        encoding="ascii",
    )

    with pytest.raises(ValueError, match="ASCII PLY"):
        load_boundary_ply(path)


def test_load_boundary_ply_rejects_short_polygon(tmp_path: Path) -> None:
    path = tmp_path / "boundary.ply"
    _write_ply(path, ["0 0 0.75 0", "1 0 0.75 0"])

    with pytest.raises(ValueError, match="at least three vertices"):
        load_boundary_ply(path)
