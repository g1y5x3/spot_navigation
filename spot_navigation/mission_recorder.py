#!/usr/bin/env python3

from __future__ import annotations

import math
import os
import tempfile
from collections.abc import Sequence
from pathlib import Path
from typing import cast

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def _pose_as_mapping(pose: Pose) -> dict[str, dict[str, float]]:
    return {
        "position": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation": {
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
            "w": float(pose.orientation.w),
        },
    }


def write_mission(
    path: Path,
    frame_id: str,
    initial_pose: Pose,
    waypoints: Sequence[Pose],
) -> None:
    if not frame_id.strip():
        raise ValueError("frame_id must be non-empty")
    if not waypoints:
        raise ValueError("mission must contain at least one waypoint")

    document = {
        "frame_id": frame_id,
        "initial_pose": _pose_as_mapping(initial_pose),
        "waypoints": [_pose_as_mapping(pose) for pose in waypoints],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    serialized = yaml.safe_dump(
        document,
        default_flow_style=False,
        sort_keys=False,
    )
    _atomic_write_text(path, serialized)


def _atomic_write_text(path: Path, content: str) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        dir=path.parent,
        text=True,
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        temporary_path.replace(path)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise


def resolve_preview_path(mission_path: Path, preview_parameter: str) -> Path:
    preview_value = preview_parameter.strip()
    preview_path = (
        Path(preview_value).expanduser()
        if preview_value
        else mission_path.with_suffix(".png")
    )
    if preview_path.suffix.lower() != ".png":
        raise ValueError("mission preview output must use a .png extension")
    return preview_path


def _pose_yaw(pose: Pose) -> float:
    quaternion = pose.orientation
    return math.atan2(
        2.0
        * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0
        - 2.0
        * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def _preview_title(mission_path: Path) -> str:
    mission_name = mission_path.stem.replace("_", " ").replace("-", " ")
    return f"{mission_name.title()} — Top-Down XY Projection"


def _label_offset(yaw: float, index: int) -> tuple[int, int]:
    horizontal = -30 if math.cos(yaw) >= 0.0 else 7
    vertical = -17 if math.sin(yaw) >= 0.25 else 8
    if abs(math.sin(yaw)) < 0.25 and index % 2 == 0:
        vertical = -17
    return horizontal, vertical


def render_mission_preview(
    pcd_path: Path,
    output_path: Path,
    title: str,
    initial_pose: Pose,
    waypoints: Sequence[Pose],
) -> None:
    if not waypoints:
        raise ValueError("mission preview requires at least one waypoint")

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.axes import Axes
    from matplotlib.figure import Figure
    from matplotlib.lines import Line2D

    from spot_navigation.pcd_to_boundary import load_pcd_xyz

    points = load_pcd_xyz(pcd_path)
    if len(points) == 0:
        raise ValueError("mission preview PCD contains no points")

    maximum_preview_points = 500_000
    point_stride = max(1, math.ceil(len(points) / maximum_preview_points))
    preview_points = points[::point_stride]

    poses = [initial_pose, *waypoints]
    positions = np.asarray(
        [
            [float(pose.position.x), float(pose.position.y)]
            for pose in poses
        ],
        dtype=np.float64,
    )
    yaws = np.asarray([_pose_yaw(pose) for pose in poses], dtype=np.float64)

    cloud_x_min = float(np.min(points[:, 0]))
    cloud_x_max = float(np.max(points[:, 0]))
    cloud_y_min = float(np.min(points[:, 1]))
    cloud_y_max = float(np.max(points[:, 1]))
    x_min = min(cloud_x_min, float(np.min(positions[:, 0])))
    x_max = max(cloud_x_max, float(np.max(positions[:, 0])))
    y_min = min(cloud_y_min, float(np.min(positions[:, 1])))
    y_max = max(cloud_y_max, float(np.max(positions[:, 1])))
    x_span = max(x_max - x_min, 1.0)
    y_span = max(y_max - y_min, 1.0)
    margin = max(0.5, max(x_span, y_span) * 0.025)
    heading_length = min(2.0, max(0.6, max(x_span, y_span) * 0.025))
    figure_width = 14.0
    figure_height = min(12.0, max(7.0, figure_width * y_span / x_span))

    figure_object, axes_object = plt.subplots(
        figsize=(figure_width, figure_height),
        dpi=200,
    )
    figure = cast(Figure, figure_object)
    axes = cast(Axes, axes_object)
    figure.patch.set_facecolor("#f8fafc")
    axes.set_facecolor("#ffffff")
    try:
        axes.scatter(
            preview_points[:, 0],
            preview_points[:, 1],
            s=0.5,
            c="#475569",
            alpha=0.38,
            linewidths=0,
            rasterized=True,
            zorder=1,
        )

        for start, end in zip(positions[:-1], positions[1:]):
            axes.annotate(
                "",
                xy=end,
                xytext=start,
                arrowprops={
                    "arrowstyle": "-|>",
                    "color": "#2563eb",
                    "linewidth": 1.9,
                    "mutation_scale": 12,
                    "shrinkA": 6,
                    "shrinkB": 6,
                    "alpha": 0.9,
                },
                zorder=3,
            )

        axes.quiver(
            positions[1:, 0],
            positions[1:, 1],
            heading_length * np.cos(yaws[1:]),
            heading_length * np.sin(yaws[1:]),
            angles="xy",
            scale_units="xy",
            scale=1,
            color="#c2410c",
            width=0.0038,
            headwidth=4.3,
            headlength=5.2,
            headaxislength=4.6,
            zorder=5,
        )
        axes.quiver(
            [positions[0, 0]],
            [positions[0, 1]],
            [heading_length * math.cos(yaws[0])],
            [heading_length * math.sin(yaws[0])],
            angles="xy",
            scale_units="xy",
            scale=1,
            color="#15803d",
            width=0.0045,
            headwidth=4.3,
            headlength=5.2,
            headaxislength=4.6,
            zorder=6,
        )
        axes.scatter(
            positions[1:, 0],
            positions[1:, 1],
            s=65,
            c="#f97316",
            edgecolors="#7c2d12",
            linewidths=1.0,
            zorder=4,
        )
        axes.scatter(
            [positions[0, 0]],
            [positions[0, 1]],
            s=180,
            marker="*",
            c="#22c55e",
            edgecolors="#14532d",
            linewidths=1.2,
            zorder=6,
        )

        axes.annotate(
            "Start",
            xy=positions[0],
            xytext=_label_offset(float(yaws[0]), 0),
            textcoords="offset points",
            fontsize=8.5,
            fontweight="bold",
            color="#14532d",
            bbox={
                "boxstyle": "round,pad=0.23",
                "fc": "white",
                "ec": "#86efac",
                "alpha": 0.92,
            },
            zorder=7,
        )
        for index, (position, yaw) in enumerate(
            zip(positions[1:], yaws[1:]),
            start=1,
        ):
            axes.annotate(
                f"W{index}",
                xy=position,
                xytext=_label_offset(float(yaw), index),
                textcoords="offset points",
                fontsize=7.8,
                fontweight="bold",
                color="#7c2d12",
                bbox={
                    "boxstyle": "round,pad=0.18",
                    "fc": "white",
                    "ec": "#fdba74",
                    "alpha": 0.92,
                },
                zorder=7,
            )

        axes.set_xlim(x_min - margin, x_max + margin)
        axes.set_ylim(y_min - margin, y_max + margin)
        axes.set_aspect("equal", adjustable="box")
        axes.set_xlabel("X [m]", fontsize=10)
        axes.set_ylabel("Y [m]", fontsize=10)
        axes.set_title(title, fontsize=15, fontweight="bold", pad=14)
        axes.grid(True, color="#cbd5e1", linewidth=0.55, alpha=0.55)
        axes.set_axisbelow(True)
        for spine in axes.spines.values():
            spine.set_color("#94a3b8")

        legend_handles = [
            Line2D(
                [0],
                [0],
                marker="*",
                linestyle="None",
                markerfacecolor="#22c55e",
                markeredgecolor="#14532d",
                markersize=12,
                label="Initial pose",
            ),
            Line2D(
                [0],
                [0],
                marker="o",
                linestyle="None",
                markerfacecolor="#f97316",
                markeredgecolor="#7c2d12",
                markersize=7,
                label="Ordered waypoints",
            ),
            Line2D(
                [0],
                [0],
                color="#2563eb",
                linewidth=2,
                label="Mission order",
            ),
        ]
        axes.legend(
            handles=legend_handles,
            loc="lower right",
            frameon=True,
            framealpha=0.95,
            fontsize=8.5,
        )
        figure.tight_layout()

        output_path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{output_path.stem}.",
            suffix=".png",
            dir=output_path.parent,
        )
        os.close(descriptor)
        temporary_path = Path(temporary_name)
        try:
            figure.savefig(
                temporary_path,
                bbox_inches="tight",
                facecolor=figure.get_facecolor(),
            )
            temporary_path.replace(output_path)
        except BaseException:
            temporary_path.unlink(missing_ok=True)
            raise
    finally:
        plt.close(figure)


class MissionRecorder(Node):
    def __init__(self) -> None:
        super().__init__("mission_recorder")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("output_mission_file", "")
        self.declare_parameter("pcd_file", "")
        self.declare_parameter("output_preview_file", "")

        self.frame_id = str(self.get_parameter("frame_id").value).strip()
        if not self.frame_id:
            raise RuntimeError("Parameter 'frame_id' must be non-empty")

        mission_parameter = str(self.get_parameter("output_mission_file").value)
        if not mission_parameter:
            raise RuntimeError(
                "Parameter 'output_mission_file' must name a YAML file"
            )
        self.output_mission_path = Path(mission_parameter).expanduser()

        pcd_parameter = str(self.get_parameter("pcd_file").value).strip()
        preview_parameter = str(
            self.get_parameter("output_preview_file").value
        )
        self.pcd_path: Path | None = None
        self.output_preview_path: Path | None = None
        if pcd_parameter:
            self.pcd_path = Path(pcd_parameter).expanduser()
            if not self.pcd_path.is_file():
                raise RuntimeError(f"PCD map does not exist: {self.pcd_path}")
            try:
                self.output_preview_path = resolve_preview_path(
                    self.output_mission_path,
                    preview_parameter,
                )
            except ValueError as error:
                raise RuntimeError(str(error)) from error
        elif preview_parameter.strip():
            raise RuntimeError(
                "Parameter 'pcd_file' is required when output_preview_file is set"
            )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._initial_pose_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self._goal_pose_callback,
            10,
        )

        self.initial_pose: Pose | None = None
        self.waypoints: list[Pose] = []
        self.get_logger().info(
            "Use RViz '2D Pose Estimate' once, then '2D Goal Pose' in route order"
        )
        output_lines = [f"  {self.output_mission_path}"]
        if self.output_preview_path is not None:
            output_lines.append(f"  {self.output_preview_path}")
        self.get_logger().info(
            "Press Ctrl+C when finished; recorder will write:\n"
            + "\n".join(output_lines)
        )

    def _initial_pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        if not self._accept_frame(message.header.frame_id, "initial pose"):
            return
        pose = message.pose.pose
        try:
            self._validate_pose(pose)
        except ValueError as error:
            self.get_logger().error(f"Rejected initial pose: {error}")
            return

        self.initial_pose = pose
        self.get_logger().info(
            "Captured initial pose: "
            f"x={pose.position.x:.6f}, y={pose.position.y:.6f}"
        )

    def _goal_pose_callback(self, message: PoseStamped) -> None:
        if self.initial_pose is None:
            self.get_logger().warning(
                "Ignored goal pose because no initial pose has been captured"
            )
            return
        if not self._accept_frame(message.header.frame_id, "goal pose"):
            return
        pose = message.pose
        try:
            self._validate_pose(pose)
        except ValueError as error:
            self.get_logger().error(f"Rejected goal pose: {error}")
            return

        self.waypoints.append(pose)
        waypoint_index = len(self.waypoints) - 1
        self.get_logger().info(
            f"Captured waypoint {waypoint_index}: "
            f"x={pose.position.x:.6f}, y={pose.position.y:.6f}"
        )

    def _accept_frame(self, message_frame: str, pose_name: str) -> bool:
        if message_frame == self.frame_id:
            return True
        self.get_logger().error(
            f"Rejected {pose_name} in frame '{message_frame}'; "
            f"expected '{self.frame_id}'"
        )
        return False

    @staticmethod
    def _validate_pose(pose: Pose) -> None:
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pose contains a non-finite value")
        if sum(value * value for value in values[3:]) < 1.0e-18:
            raise ValueError("orientation quaternion cannot be zero")

    def write_outputs(self) -> None:
        if self.initial_pose is None:
            self.get_logger().error(
                "No initial pose captured; mission files were not written"
            )
            return
        if not self.waypoints:
            self.get_logger().error(
                "No waypoints captured; mission files were not written"
            )
            return

        write_mission(
            self.output_mission_path,
            self.frame_id,
            self.initial_pose,
            self.waypoints,
        )
        if self.pcd_path is not None and self.output_preview_path is not None:
            render_mission_preview(
                self.pcd_path,
                self.output_preview_path,
                _preview_title(self.output_mission_path),
                self.initial_pose,
                self.waypoints,
            )
        self.get_logger().info(
            f"Wrote mission with {len(self.waypoints)} waypoints:\n"
            f"  {self.output_mission_path}"
        )
        if self.output_preview_path is not None:
            self.get_logger().info(
                f"Wrote mission preview:\n  {self.output_preview_path}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node: MissionRecorder | None = None
    try:
        node = MissionRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.write_outputs()
            except Exception as error:
                node.get_logger().error(f"Failed to write mission outputs: {error}")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
