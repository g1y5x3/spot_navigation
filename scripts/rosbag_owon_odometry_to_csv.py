#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import os
import sys
import tempfile
from pathlib import Path
from typing import Any, Sequence

import yaml


OWON_TOPIC = "/owon/value"
ODOMETRY_TOPIC = "/odometry_map"
OWON_TYPE = "std_msgs/msg/Float32"
ODOMETRY_TYPE = "nav_msgs/msg/Odometry"

OUTPUT_COLUMNS = (
    "timestamp_ns",
    "frame_id",
    "child_frame_id",
    "voltage",
    "position_x",
    "position_y",
    "position_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
)


def odometry_pose_row(message: Any, bag_timestamp_ns: int) -> dict[str, Any]:
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return {
        "odometry_bag_timestamp_ns": int(bag_timestamp_ns),
        "frame_id": str(message.header.frame_id),
        "child_frame_id": str(message.child_frame_id),
        "position_x": float(position.x),
        "position_y": float(position.y),
        "position_z": float(position.z),
        "orientation_x": float(orientation.x),
        "orientation_y": float(orientation.y),
        "orientation_z": float(orientation.z),
        "orientation_w": float(orientation.w),
    }


def join_row(
    voltage: float,
    voltage_bag_timestamp_ns: int,
    odometry: dict[str, Any],
) -> dict[str, Any]:
    return {
        "timestamp_ns": int(voltage_bag_timestamp_ns),
        "frame_id": str(odometry["frame_id"]),
        "child_frame_id": str(odometry["child_frame_id"]),
        "voltage": float(voltage),
        "position_x": float(odometry["position_x"]),
        "position_y": float(odometry["position_y"]),
        "position_z": float(odometry["position_z"]),
        "orientation_x": float(odometry["orientation_x"]),
        "orientation_y": float(odometry["orientation_y"]),
        "orientation_z": float(odometry["orientation_z"]),
        "orientation_w": float(odometry["orientation_w"]),
    }


def select_nearest_odometry(
    voltage_bag_timestamp_ns: int,
    previous: dict[str, Any] | None,
    following: dict[str, Any] | None,
) -> dict[str, Any]:
    if following is None:
        if previous is None:
            raise ValueError("No odometry sample is available")
        return previous
    if previous is None:
        return following

    previous_gap = abs(
        int(previous["odometry_bag_timestamp_ns"])
        - int(voltage_bag_timestamp_ns)
    )
    following_gap = abs(
        int(following["odometry_bag_timestamp_ns"])
        - int(voltage_bag_timestamp_ns)
    )
    return previous if previous_gap <= following_gap else following


def match_is_within_gap(
    voltage_bag_timestamp_ns: int,
    odometry: dict[str, Any],
    maximum_gap_ns: int | None,
) -> bool:
    if maximum_gap_ns is None:
        return True
    offset = (
        int(odometry["odometry_bag_timestamp_ns"])
        - int(voltage_bag_timestamp_ns)
    )
    return abs(offset) <= maximum_gap_ns


def read_storage_id(bag_path: Path) -> str:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        raise ValueError(f"Bag metadata does not exist: {metadata_path}")
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    try:
        storage_id = metadata["rosbag2_bagfile_information"]["storage_identifier"]
    except (KeyError, TypeError) as error:
        raise ValueError(
            f"Bag metadata has no storage identifier: {metadata_path}"
        ) from error
    if not isinstance(storage_id, str) or not storage_id:
        raise ValueError(f"Invalid storage identifier in {metadata_path}")
    return storage_id


def default_output_path(bag_path: Path) -> Path:
    return bag_path.with_name(bag_path.name + "_owon_odometry.csv")


def convert_bag(
    bag_path: Path,
    output_path: Path,
    storage_id: str = "",
    force: bool = False,
    maximum_gap_ns: int | None = None,
) -> tuple[Path, int, int, int, int]:
    bag_path = bag_path.expanduser().resolve()
    output_path = output_path.expanduser().resolve()
    if not bag_path.is_dir():
        raise ValueError(f"Bag directory does not exist: {bag_path}")
    if maximum_gap_ns is not None and maximum_gap_ns < 0:
        raise ValueError("maximum_gap_ns cannot be negative")
    if output_path.exists() and not force:
        raise FileExistsError(
            f"Output file already exists: {output_path}. Use --force to replace."
        )
    selected_storage_id = storage_id or read_storage_id(bag_path)

    try:
        import rosbag2_py
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import Float32
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python modules are unavailable. Source the ROS 2 environment "
            "before running this script."
        ) from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_path),
            storage_id=selected_storage_id,
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    _validate_topic_type(topic_types, OWON_TOPIC, OWON_TYPE)
    _validate_topic_type(topic_types, ODOMETRY_TOPIC, ODOMETRY_TYPE)
    reader.set_filter(
        rosbag2_py.StorageFilter(topics=[OWON_TOPIC, ODOMETRY_TOPIC])
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    stream, temporary_path = _open_temporary_csv(output_path)
    try:
        writer = csv.DictWriter(stream, fieldnames=OUTPUT_COLUMNS)
        writer.writeheader()
        voltage_count = 0
        odometry_count = 0
        joined_count = 0
        dropped_count = 0
        previous_odometry: dict[str, Any] | None = None
        pending_voltages: list[tuple[int, float]] = []

        while reader.has_next():
            topic_name, serialized_data, bag_timestamp_ns = reader.read_next()
            if topic_name == OWON_TOPIC:
                voltage_message = deserialize_message(serialized_data, Float32)
                pending_voltages.append(
                    (int(bag_timestamp_ns), float(voltage_message.data))
                )
                voltage_count += 1
            elif topic_name == ODOMETRY_TOPIC:
                odometry_message = deserialize_message(serialized_data, Odometry)
                following_odometry = odometry_pose_row(
                    odometry_message,
                    bag_timestamp_ns,
                )
                odometry_count += 1
                joined, dropped = _write_pending_matches(
                    writer,
                    pending_voltages,
                    previous_odometry,
                    following_odometry,
                    maximum_gap_ns,
                )
                joined_count += joined
                dropped_count += dropped
                pending_voltages.clear()
                previous_odometry = following_odometry

        joined, dropped = _write_pending_matches(
            writer,
            pending_voltages,
            previous_odometry,
            None,
            maximum_gap_ns,
        )
        joined_count += joined
        dropped_count += dropped

        stream.flush()
        os.fsync(stream.fileno())
        stream.close()
        _finalize_temporary_csv(temporary_path, output_path, force)
    except BaseException:
        stream.close()
        temporary_path.unlink(missing_ok=True)
        raise

    return (
        output_path,
        voltage_count,
        odometry_count,
        joined_count,
        dropped_count,
    )


def _write_pending_matches(
    writer: csv.DictWriter,
    pending_voltages: Sequence[tuple[int, float]],
    previous_odometry: dict[str, Any] | None,
    following_odometry: dict[str, Any] | None,
    maximum_gap_ns: int | None,
) -> tuple[int, int]:
    if previous_odometry is None and following_odometry is None:
        return 0, len(pending_voltages)

    joined_count = 0
    dropped_count = 0
    for voltage_timestamp_ns, voltage in pending_voltages:
        odometry = select_nearest_odometry(
            voltage_timestamp_ns,
            previous_odometry,
            following_odometry,
        )
        if not match_is_within_gap(
            voltage_timestamp_ns,
            odometry,
            maximum_gap_ns,
        ):
            dropped_count += 1
            continue
        writer.writerow(join_row(voltage, voltage_timestamp_ns, odometry))
        joined_count += 1
    return joined_count, dropped_count


def _validate_topic_type(
    topic_types: dict[str, str],
    topic_name: str,
    expected_type: str,
) -> None:
    actual_type = topic_types.get(topic_name)
    if actual_type is None:
        raise ValueError(f"Bag does not contain required topic: {topic_name}")
    if actual_type != expected_type:
        raise ValueError(
            f"Topic {topic_name} has type {actual_type}; expected {expected_type}"
        )


def _open_temporary_csv(output_path: Path) -> tuple[Any, Path]:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output_path.name}.",
        suffix=".tmp",
        dir=output_path.parent,
        text=True,
    )
    stream = os.fdopen(descriptor, "w", encoding="utf-8", newline="")
    return stream, Path(temporary_name)


def _finalize_temporary_csv(
    temporary_path: Path,
    output_path: Path,
    force: bool,
) -> None:
    if force:
        temporary_path.replace(output_path)
        return

    try:
        os.link(temporary_path, output_path)
    except FileExistsError as error:
        raise FileExistsError(
            f"Output file already exists: {output_path}. Use --force to replace."
        ) from error
    temporary_path.unlink()


def _parse_arguments(arguments: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Join /owon/value samples to nearest /odometry_map poses from a "
            "ROS 2 bag and write one CSV file."
        )
    )
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory")
    parser.add_argument(
        "--output",
        type=Path,
        help="Output CSV; defaults beside bag as BAG_owon_odometry.csv",
    )
    parser.add_argument(
        "--max-gap-ms",
        type=float,
        help=(
            "Drop voltage samples whose nearest odometry is farther than this "
            "many milliseconds; default keeps every nearest match"
        ),
    )
    parser.add_argument(
        "--storage-id",
        default="",
        help="Override storage plugin; otherwise read it from metadata.yaml",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Atomically replace an existing output CSV",
    )
    return parser.parse_args(arguments)


def main(arguments: Sequence[str] | None = None) -> int:
    options = _parse_arguments(arguments)
    bag_path = options.bag.expanduser()
    output_path = options.output or default_output_path(bag_path)
    maximum_gap_ns = None
    if options.max_gap_ms is not None:
        if options.max_gap_ms < 0.0:
            print("ERROR: --max-gap-ms cannot be negative", file=sys.stderr)
            return 1
        maximum_gap_ns = int(round(options.max_gap_ms * 1_000_000.0))

    try:
        (
            result_path,
            voltage_count,
            odometry_count,
            joined_count,
            dropped_count,
        ) = convert_bag(
            bag_path,
            output_path,
            storage_id=options.storage_id,
            force=options.force,
            maximum_gap_ns=maximum_gap_ns,
        )
    except (FileExistsError, RuntimeError, ValueError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("Interrupted; incomplete temporary file removed.", file=sys.stderr)
        return 130

    print(f"{OWON_TOPIC}: {voltage_count} messages")
    print(f"{ODOMETRY_TOPIC}: {odometry_count} messages")
    print(f"Joined rows: {joined_count}")
    print(f"Dropped voltage samples: {dropped_count}")
    print(f"Output: {result_path}")
    if voltage_count == 0 or odometry_count == 0:
        print("WARNING: At least one requested topic contained zero messages.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
