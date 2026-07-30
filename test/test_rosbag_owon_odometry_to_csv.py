import csv
import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


SCRIPT_PATH = (
    Path(__file__).parents[1] / "scripts" / "rosbag_owon_odometry_to_csv.py"
)
SPEC = importlib.util.spec_from_file_location("rosbag_owon_odometry_to_csv", SCRIPT_PATH)
assert SPEC is not None
assert SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _vector(x: float, y: float, z: float) -> SimpleNamespace:
    return SimpleNamespace(x=x, y=y, z=z)


def _odometry(position_x: float = 1.0) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=345),
            frame_id="map",
        ),
        child_frame_id="base_link",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=_vector(position_x, 2.0, 3.0),
                orientation=SimpleNamespace(x=0.1, y=0.2, z=0.3, w=0.9),
            ),
        ),
    )


def test_odometry_pose_row_contains_only_pose_and_orientation() -> None:
    row = MODULE.odometry_pose_row(_odometry(), 200)

    assert row == {
        "odometry_bag_timestamp_ns": 200,
        "frame_id": "map",
        "child_frame_id": "base_link",
        "position_x": pytest.approx(1.0),
        "position_y": pytest.approx(2.0),
        "position_z": pytest.approx(3.0),
        "orientation_x": pytest.approx(0.1),
        "orientation_y": pytest.approx(0.2),
        "orientation_z": pytest.approx(0.3),
        "orientation_w": pytest.approx(0.9),
    }


def test_join_row_combines_voltage_with_odometry() -> None:
    odometry = MODULE.odometry_pose_row(_odometry(), 200)

    row = MODULE.join_row(51.5, 180, odometry)

    assert tuple(row) == MODULE.OUTPUT_COLUMNS
    assert row["timestamp_ns"] == 180
    assert [name for name in row if "timestamp" in name] == ["timestamp_ns"]
    assert row["voltage"] == pytest.approx(51.5)
    assert row["position_x"] == pytest.approx(1.0)
    assert row["orientation_w"] == pytest.approx(0.9)


def test_select_nearest_odometry_prefers_earlier_sample_on_tie() -> None:
    previous = MODULE.odometry_pose_row(_odometry(1.0), 100)
    following = MODULE.odometry_pose_row(_odometry(2.0), 200)

    assert MODULE.select_nearest_odometry(140, previous, following) is previous
    assert MODULE.select_nearest_odometry(160, previous, following) is following
    assert MODULE.select_nearest_odometry(150, previous, following) is previous


def test_select_nearest_odometry_handles_missing_samples() -> None:
    sample = MODULE.odometry_pose_row(_odometry(), 100)

    assert MODULE.select_nearest_odometry(100, sample, None) is sample
    assert MODULE.select_nearest_odometry(100, None, sample) is sample
    with pytest.raises(ValueError, match="No odometry sample"):
        MODULE.select_nearest_odometry(100, None, None)


def test_read_storage_id_from_metadata(tmp_path: Path) -> None:
    metadata_path = tmp_path / "metadata.yaml"
    metadata_path.write_text(
        "rosbag2_bagfile_information:\n  storage_identifier: sqlite3\n",
        encoding="utf-8",
    )

    assert MODULE.read_storage_id(tmp_path) == "sqlite3"


def test_default_output_path_is_bag_sibling(tmp_path: Path) -> None:
    bag_path = tmp_path / "rosbag2_run"

    assert MODULE.default_output_path(bag_path) == (
        tmp_path / "rosbag2_run_owon_odometry.csv"
    )


def test_finalize_csv_does_not_clobber_concurrently_created_output(
    tmp_path: Path,
) -> None:
    temporary_path = tmp_path / ".joined.csv.tmp"
    output_path = tmp_path / "joined.csv"
    temporary_path.write_text("generated\n", encoding="utf-8")
    output_path.write_text("created concurrently\n", encoding="utf-8")

    with pytest.raises(FileExistsError, match="Use --force"):
        MODULE._finalize_temporary_csv(temporary_path, output_path, force=False)

    assert output_path.read_text(encoding="utf-8") == "created concurrently\n"


def test_convert_bag_joins_each_voltage_to_nearest_odometry(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    bag_path = tmp_path / "bag"
    bag_path.mkdir()
    (bag_path / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n  storage_identifier: sqlite3\n",
        encoding="utf-8",
    )
    messages = [
        (MODULE.ODOMETRY_TOPIC, _odometry(1.0), 100),
        (MODULE.OWON_TOPIC, SimpleNamespace(data=50.0), 140),
        (MODULE.OWON_TOPIC, SimpleNamespace(data=51.0), 190),
        ("/unrelated", SimpleNamespace(data="ignored"), 195),
        (MODULE.ODOMETRY_TOPIC, _odometry(2.0), 200),
        (MODULE.OWON_TOPIC, SimpleNamespace(data=52.0), 260),
    ]
    filtered_topics: list[str] = []

    class FakeSequentialReader:
        def __init__(self) -> None:
            self.index = 0

        def open(self, storage_options, converter_options) -> None:
            assert storage_options.uri == str(bag_path.resolve())
            assert storage_options.storage_id == "sqlite3"

        def get_all_topics_and_types(self) -> list[SimpleNamespace]:
            return [
                SimpleNamespace(name=MODULE.OWON_TOPIC, type=MODULE.OWON_TYPE),
                SimpleNamespace(
                    name=MODULE.ODOMETRY_TOPIC,
                    type=MODULE.ODOMETRY_TYPE,
                ),
                SimpleNamespace(name="/unrelated", type="std_msgs/msg/String"),
            ]

        def set_filter(self, storage_filter) -> None:
            filtered_topics.extend(storage_filter.topics)

        def has_next(self) -> bool:
            return self.index < len(messages)

        def read_next(self):
            message = messages[self.index]
            self.index += 1
            return message

    class FakeStorageOptions:
        def __init__(self, uri: str, storage_id: str) -> None:
            self.uri = uri
            self.storage_id = storage_id

    class FakeConverterOptions:
        def __init__(self, input_format: str, output_format: str) -> None:
            self.input_format = input_format
            self.output_format = output_format

    class FakeStorageFilter:
        def __init__(self, topics: list[str]) -> None:
            self.topics = topics

    rosbag2_module = ModuleType("rosbag2_py")
    setattr(rosbag2_module, "SequentialReader", FakeSequentialReader)
    setattr(rosbag2_module, "StorageOptions", FakeStorageOptions)
    setattr(rosbag2_module, "ConverterOptions", FakeConverterOptions)
    setattr(rosbag2_module, "StorageFilter", FakeStorageFilter)
    nav_msgs_module = ModuleType("nav_msgs")
    nav_msgs_msg_module = ModuleType("nav_msgs.msg")
    setattr(nav_msgs_msg_module, "Odometry", object)
    setattr(nav_msgs_module, "msg", nav_msgs_msg_module)
    std_msgs_module = ModuleType("std_msgs")
    std_msgs_msg_module = ModuleType("std_msgs.msg")
    setattr(std_msgs_msg_module, "Float32", object)
    setattr(std_msgs_module, "msg", std_msgs_msg_module)
    rclpy_module = ModuleType("rclpy")
    serialization_module = ModuleType("rclpy.serialization")
    setattr(serialization_module, "deserialize_message", lambda data, kind: data)
    setattr(rclpy_module, "serialization", serialization_module)

    for name, module in (
        ("rosbag2_py", rosbag2_module),
        ("nav_msgs", nav_msgs_module),
        ("nav_msgs.msg", nav_msgs_msg_module),
        ("std_msgs", std_msgs_module),
        ("std_msgs.msg", std_msgs_msg_module),
        ("rclpy", rclpy_module),
        ("rclpy.serialization", serialization_module),
    ):
        monkeypatch.setitem(sys.modules, name, module)

    output_path = tmp_path / "joined.csv"
    result_path, voltage_count, odometry_count, joined_count, dropped_count = (
        MODULE.convert_bag(bag_path, output_path)
    )

    assert result_path == output_path.resolve()
    assert (voltage_count, odometry_count, joined_count, dropped_count) == (
        3,
        2,
        3,
        0,
    )
    assert filtered_topics == [MODULE.OWON_TOPIC, MODULE.ODOMETRY_TOPIC]
    with output_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert [row["voltage"] for row in rows] == ["50.0", "51.0", "52.0"]
    assert [row["timestamp_ns"] for row in rows] == ["140", "190", "260"]
    assert [row["position_x"] for row in rows] == ["1.0", "2.0", "2.0"]


def test_convert_bag_can_drop_matches_outside_maximum_gap() -> None:
    odometry = MODULE.odometry_pose_row(_odometry(), 1_000_000_000)

    assert MODULE.match_is_within_gap(900_000_000, odometry, 100_000_000)
    assert not MODULE.match_is_within_gap(899_999_999, odometry, 100_000_000)
    assert MODULE.match_is_within_gap(0, odometry, None)
