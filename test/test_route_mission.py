import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


def _install_module(name: str, attributes: dict[str, object]) -> None:
    module = ModuleType(name)
    for attribute_name, value in attributes.items():
        setattr(module, attribute_name, value)
    sys.modules[name] = module


class _Pose:
    def __init__(self) -> None:
        self.position = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.orientation = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0)


if importlib.util.find_spec("rclpy") is None:
    _install_module("rclpy", {})
    _install_module("rclpy.node", {"Node": object})
    _install_module("rclpy.time", {"Time": object})
    _install_module(
        "geometry_msgs.msg",
        {
            "Pose": _Pose,
            "PoseStamped": object,
            "PoseWithCovarianceStamped": object,
        },
    )
    _install_module("nav_msgs.msg", {"Odometry": object})
    _install_module("std_msgs.msg", {"Bool": object})


from spot_navigation.route_manager import (
    MissionEvent,
    MissionProgress,
    Pose,
    load_mission,
)


MISSION_YAML = """
frame_id: map
initial_pose:
  position: {x: 0.5, y: -0.2, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.03, w: 0.99955}
waypoints:
  - position: {x: 1.0, y: 2.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - position: {x: 3.0, y: 4.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.7071068, w: 0.7071068}
"""


def _write_mission(tmp_path: Path, content: str = MISSION_YAML) -> Path:
    mission_path = tmp_path / "mission.yaml"
    mission_path.write_text(content, encoding="utf-8")
    return mission_path


def test_load_mission_preserves_initial_pose_and_waypoint_order(tmp_path: Path) -> None:
    mission = load_mission(_write_mission(tmp_path))

    assert mission.frame_id == "map"
    assert mission.initial_pose.position.x == pytest.approx(0.5)
    assert mission.initial_pose.position.y == pytest.approx(-0.2)
    assert [pose.position.x for pose in mission.waypoints] == [1.0, 3.0]
    assert mission.waypoints[1].orientation.z == pytest.approx(0.7071068)


def test_load_mission_strips_frame_id_whitespace(tmp_path: Path) -> None:
    mission = load_mission(
        _write_mission(
            tmp_path,
            MISSION_YAML.replace("frame_id: map", 'frame_id: " map "'),
        )
    )

    assert mission.frame_id == "map"


@pytest.mark.parametrize(
    ("invalid_yaml", "expected_error"),
    (
        (
            MISSION_YAML.replace(
                "  - position: {x: 1.0, y: 2.0, z: 0.0}",
                "  - position: invalid",
            ),
            "'waypoints[0].position' must be a mapping",
        ),
        (
            MISSION_YAML.replace("x: 1.0, y: 2.0", "x: invalid, y: 2.0"),
            "'waypoints[0].position.x' must be numeric",
        ),
    ),
)
def test_load_mission_validates_pose_fields(
    tmp_path: Path,
    invalid_yaml: str,
    expected_error: str,
) -> None:
    with pytest.raises(ValueError) as error:
        load_mission(_write_mission(tmp_path, invalid_yaml))

    assert str(error.value) == expected_error


def test_office_mission_configuration_is_valid() -> None:
    package_root = Path(__file__).parents[1]
    mission = load_mission(package_root / "map" / "office_mission.yaml")

    assert mission.frame_id == "map"
    assert [pose.position.x for pose in mission.waypoints] == [
        pytest.approx(5.79384),
        pytest.approx(5.79811),
        pytest.approx(1.59912),
        pytest.approx(1.43618),
    ]


def test_far_success_advances_only_after_current_goal_acknowledgement(
    tmp_path: Path,
) -> None:
    progress = MissionProgress(load_mission(_write_mission(tmp_path)).waypoints)

    assert progress.active_index == 0
    active_pose = progress.active_pose
    assert active_pose is not None
    assert active_pose.position.x == pytest.approx(1.0)
    assert progress.handle_far_status(True) is MissionEvent.IGNORED
    assert progress.active_index == 0

    assert progress.handle_far_status(False) is MissionEvent.GOAL_ACCEPTED
    assert progress.handle_far_status(False) is MissionEvent.IGNORED
    assert progress.handle_far_status(True) is MissionEvent.WAYPOINT_REACHED
    assert progress.active_index == 1
    active_pose = progress.active_pose
    assert active_pose is not None
    assert active_pose.position.x == pytest.approx(3.0)

    assert progress.handle_far_status(False) is MissionEvent.GOAL_ACCEPTED
    assert progress.handle_far_status(True) is MissionEvent.MISSION_COMPLETE
    assert progress.active_pose is None
    assert progress.is_complete


def test_progress_supports_arbitrary_waypoint_count() -> None:
    waypoints = []
    for index in range(20):
        pose = Pose()
        pose.position.x = float(index)
        pose.orientation.w = 1.0
        waypoints.append(pose)
    progress = MissionProgress(tuple(waypoints))

    for index in range(20):
        assert progress.active_index == index
        active_pose = progress.active_pose
        assert active_pose is not None
        assert active_pose.position.x == pytest.approx(float(index))
        assert progress.handle_far_status(False) is MissionEvent.GOAL_ACCEPTED
        expected_event = (
            MissionEvent.MISSION_COMPLETE
            if index == 19
            else MissionEvent.WAYPOINT_REACHED
        )
        assert progress.handle_far_status(True) is expected_event

    assert progress.is_complete


def test_retry_requires_a_new_far_acknowledgement(tmp_path: Path) -> None:
    progress = MissionProgress(load_mission(_write_mission(tmp_path)).waypoints)

    assert progress.handle_far_status(False) is MissionEvent.GOAL_ACCEPTED
    progress.reset_goal_acknowledgement()

    assert progress.handle_far_status(True) is MissionEvent.IGNORED
    assert progress.active_index == 0
    active_pose = progress.active_pose
    assert active_pose is not None
    assert active_pose.position.x == pytest.approx(1.0)
    assert progress.handle_far_status(False) is MissionEvent.GOAL_ACCEPTED
    assert progress.handle_far_status(True) is MissionEvent.WAYPOINT_REACHED
