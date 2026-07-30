import ast
import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

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


class _Point:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Marker:
    ADD = 0
    ARROW = 0
    LINE_STRIP = 4
    TEXT_VIEW_FACING = 9
    DELETEALL = 3

    def __init__(self) -> None:
        self.header = SimpleNamespace(frame_id="", stamp=None)
        self.ns = ""
        self.id = 0
        self.type = 0
        self.action = 0
        self.pose = _Pose()
        self.scale = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.color = SimpleNamespace(r=0.0, g=0.0, b=0.0, a=0.0)
        self.points: list[_Point] = []
        self.text = ""


class _MarkerArray:
    def __init__(self) -> None:
        self.markers: list[_Marker] = []


if importlib.util.find_spec("rclpy") is None:
    geometry_messages = _install_module(
        "geometry_msgs.msg",
        {
            "Point": _Point,
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
    _install_module(
        "rclpy.qos",
        {
            "DurabilityPolicy": SimpleNamespace(TRANSIENT_LOCAL=1),
            "QoSProfile": lambda depth: SimpleNamespace(depth=depth),
            "ReliabilityPolicy": SimpleNamespace(RELIABLE=1),
        },
    )
    visualization_messages = _install_module(
        "visualization_msgs.msg",
        {"Marker": _Marker, "MarkerArray": _MarkerArray},
    )
    _install_module("visualization_msgs", {"msg": visualization_messages})


from geometry_msgs.msg import Pose  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402
from spot_navigation.mission_markers import (  # noqa: E402
    build_mission_markers,
)
from spot_navigation.mission_recorder import (  # noqa: E402
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


def test_launch_forwards_frame_id_to_rviz_fixed_frame() -> None:
    launch_path = (
        Path(__file__).parents[1] / "launch/mission_recorder.launch.py"
    )
    tree = ast.parse(launch_path.read_text(encoding="utf-8"))
    rviz_assignment = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        for node in node.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "rviz_node"
            for target in node.targets
        )
    )
    assert isinstance(rviz_assignment.value, ast.Call)
    arguments = next(
        keyword.value
        for keyword in rviz_assignment.value.keywords
        if keyword.arg == "arguments"
    )
    assert any(
        isinstance(node, ast.Constant) and node.value == "-f"
        for node in ast.walk(arguments)
    )
    assert any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "LaunchConfiguration"
        and len(node.args) == 1
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "frame_id"
        for node in ast.walk(arguments)
    )


def test_validate_pose_accepts_ros_pose() -> None:
    MissionRecorder._validate_pose(_pose(1.0, 2.0, 0.5, 0.5))


def test_validate_pose_rejects_zero_quaternion() -> None:
    with pytest.raises(
        ValueError,
        match="orientation quaternion cannot be zero",
    ):
        MissionRecorder._validate_pose(_pose(1.0, 2.0, 0.0, 0.0))


@pytest.mark.parametrize("callback_name", ["initial", "goal"])
def test_accepted_pose_callback_refreshes_visualization(
    callback_name: str,
) -> None:
    pose = _pose(1.0, 2.0)
    initial_pose = _pose(0.0, 0.0) if callback_name == "goal" else None
    publish_markers = Mock()
    logger = SimpleNamespace(error=Mock(), info=Mock(), warning=Mock())
    recorder = SimpleNamespace(
        frame_id="map",
        initial_pose=initial_pose,
        waypoints=[],
        _accept_frame=Mock(return_value=True),
        _validate_pose=MissionRecorder._validate_pose,
        _publish_markers=publish_markers,
        get_logger=Mock(return_value=logger),
    )
    message_pose = (
        SimpleNamespace(pose=pose) if callback_name == "initial" else pose
    )
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        pose=message_pose,
    )

    callback = getattr(MissionRecorder, f"_{callback_name}_pose_callback")
    callback(recorder, message)

    publish_markers.assert_called_once_with()
    if callback_name == "initial":
        assert recorder.initial_pose is pose
        assert recorder.waypoints == []
    else:
        assert recorder.initial_pose is initial_pose
        assert recorder.waypoints == [pose]


def test_build_mission_markers_shows_start_route_and_waypoints() -> None:
    stamp = Marker().header.stamp
    clear_markers = build_mission_markers("map", stamp, None, ())
    marker_array = build_mission_markers(
        "map",
        stamp,
        _pose(1.0, 2.0),
        (_pose(3.0, 4.0), _pose(5.0, 6.0, 0.7071068, 0.7071068)),
    )

    assert len(clear_markers.markers) == 1
    assert (
        clear_markers.markers[0].action
        == clear_markers.markers[0].DELETEALL
    )
    assert all(
        marker.action != Marker.DELETEALL
        for marker in marker_array.markers
    )
    markers_by_namespace = {
        marker.ns: marker
        for marker in marker_array.markers
        if marker.ns in {"mission_route", "mission_initial_pose"}
    }
    assert (
        markers_by_namespace["mission_initial_pose"].type == Marker.ARROW
    )
    assert markers_by_namespace["mission_route"].type == Marker.LINE_STRIP
    assert [
        (point.x, point.y)
        for point in markers_by_namespace["mission_route"].points
    ] == [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]
    assert [
        marker.text
        for marker in marker_array.markers
        if marker.ns == "mission_labels"
    ] == ["START", "W1", "W2"]
    assert [
        marker.id
        for marker in marker_array.markers
        if marker.ns == "mission_waypoints"
    ] == [1, 2]


def test_rviz_enables_recorded_mission_markers() -> None:
    rviz_path = Path(__file__).parents[1] / "rviz/mission_recorder.rviz"
    configuration = yaml.safe_load(rviz_path.read_text(encoding="utf-8"))
    displays = configuration["Visualization Manager"]["Displays"]

    mission_display = next(
        display
        for display in displays
        if display.get("Name") == "Recorded Mission"
    )
    assert mission_display["Class"] == "rviz_default_plugins/MarkerArray"
    assert mission_display["Enabled"] is True
    assert mission_display["Topic"]["Value"] == "/mission_recorder/markers"
    assert mission_display["Topic"]["Durability Policy"] == "Transient Local"


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
