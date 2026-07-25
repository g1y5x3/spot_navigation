import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
import yaml

_STUBBED_MODULES: list[str] = []


def _install_module(name: str, attributes: dict[str, object]) -> ModuleType:
    module = ModuleType(name)
    for attribute_name, value in attributes.items():
        setattr(module, attribute_name, value)
    sys.modules[name] = module
    _STUBBED_MODULES.append(name)
    return module


class _Pose:
    def __init__(self) -> None:
        self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)


if importlib.util.find_spec("rclpy") is None:
    geometry_messages = _install_module(
        "geometry_msgs.msg",
        {
            "Pose": _Pose,
            "PoseStamped": object,
            "PoseWithCovarianceStamped": object,
        },
    )
    _install_module("geometry_msgs", {"msg": geometry_messages})
    _install_module("rclpy", {})
    _install_module(
        "rclpy.executors",
        {"ExternalShutdownException": KeyboardInterrupt},
    )
    _install_module("rclpy.node", {"Node": object})


from geometry_msgs.msg import Pose
from spot_navigation.mission_recorder import (
    MissionRecorder,
    render_mission_preview,
    resolve_preview_path,
    write_mission,
)

for module_name in reversed(_STUBBED_MODULES):
    del sys.modules[module_name]


def _pose(x: float, y: float, qz: float = 0.0, qw: float = 1.0) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def test_validate_pose_accepts_ros_pose() -> None:
    MissionRecorder._validate_pose(_pose(1.0, 2.0, 0.5, 0.5))


def test_validate_pose_rejects_zero_quaternion() -> None:
    with pytest.raises(ValueError, match="orientation quaternion cannot be zero"):
        MissionRecorder._validate_pose(_pose(1.0, 2.0, 0.0, 0.0))


def test_write_mission_preserves_capture_order(tmp_path: Path) -> None:
    mission_path = tmp_path / "mission.yaml"
    initial_pose = _pose(1.0, 2.0)
    waypoints = (_pose(3.0, 4.0), _pose(5.0, 6.0, 0.7071068, 0.7071068))

    write_mission(mission_path, "map", initial_pose, waypoints)

    document = yaml.safe_load(mission_path.read_text(encoding="utf-8"))
    assert document["frame_id"] == "map"
    assert document["initial_pose"]["position"] == {
        "x": 1.0,
        "y": 2.0,
        "z": 0.0,
    }
    assert [entry["position"]["x"] for entry in document["waypoints"]] == [
        3.0,
        5.0,
    ]
    assert document["waypoints"][1]["orientation"]["z"] == pytest.approx(
        0.7071068
    )


def test_write_mission_requires_frame_initial_pose_and_waypoint(
    tmp_path: Path,
) -> None:
    output_path = tmp_path / "mission.yaml"
    pose = _pose(0.0, 0.0)

    with pytest.raises(ValueError, match="frame_id must be non-empty"):
        write_mission(output_path, "", pose, (pose,))
    with pytest.raises(ValueError, match="at least one waypoint"):
        write_mission(output_path, "map", pose, ())


def test_resolve_preview_path_defaults_to_mission_stem() -> None:
    assert resolve_preview_path(Path("/maps/mcclay_mission.yaml"), "") == Path(
        "/maps/mcclay_mission.png"
    )
    assert resolve_preview_path(
        Path("/maps/mcclay_mission.yaml"),
        "/tmp/custom.png",
    ) == Path("/tmp/custom.png")


def test_render_mission_preview_writes_png(tmp_path: Path) -> None:
    pcd_path = tmp_path / "small.pcd"
    pcd_path.write_text(
        "\n".join(
            [
                "# .PCD v0.7 - Point Cloud Data file format",
                "VERSION 0.7",
                "FIELDS x y z",
                "SIZE 4 4 4",
                "TYPE F F F",
                "COUNT 1 1 1",
                "WIDTH 8",
                "HEIGHT 1",
                "VIEWPOINT 0 0 0 1 0 0 0",
                "POINTS 8",
                "DATA ascii",
                "0 0 0",
                "4 0 0",
                "4 4 0",
                "0 4 0",
                "1 1 1",
                "3 1 1",
                "3 3 1",
                "1 3 1",
                "",
            ]
        ),
        encoding="ascii",
    )
    output_path = tmp_path / "mission.png"

    render_mission_preview(
        pcd_path,
        output_path,
        "Test Mission — Top-Down XY Projection",
        _pose(1.0, 1.0),
        (_pose(2.0, 1.0), _pose(3.0, 3.0, 0.7071068, 0.7071068)),
    )

    assert output_path.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert output_path.stat().st_size > 10_000
