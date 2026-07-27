#!/usr/bin/env python3

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

Point3 = tuple[float, float, float]
BoundaryPolygons = dict[int, list[Point3]]


def resolve_boundary_path(boundary_file: str, pcd_file: str) -> Path:
    explicit_path = boundary_file.strip()
    if explicit_path:
        return Path(explicit_path).expanduser()

    pcd_path_value = pcd_file.strip()
    if not pcd_path_value:
        raise ValueError("boundary_file or pcd_file must be provided")
    pcd_path = Path(pcd_path_value).expanduser()
    return pcd_path.with_name(f"{pcd_path.stem}_boundary.ply")


def load_boundary_ply(path: Path) -> BoundaryPolygons:
    try:
        lines = path.read_text(encoding="ascii").splitlines()
    except UnicodeDecodeError as error:
        raise ValueError(f"Boundary map must be an ASCII PLY file: {path}") from error

    if not lines or lines[0].strip() != "ply":
        raise ValueError(f"Boundary map is not a PLY file: {path}")

    vertex_count: int | None = None
    vertex_properties: list[str] = []
    current_element = ""
    is_ascii = False
    data_start: int | None = None

    for line_index, raw_line in enumerate(lines[1:], start=1):
        parts = raw_line.strip().split()
        if not parts or parts[0] == "comment":
            continue
        if parts[0] == "format":
            is_ascii = len(parts) >= 2 and parts[1] == "ascii"
        elif parts[0] == "element" and len(parts) == 3:
            current_element = parts[1]
            if current_element == "vertex":
                vertex_count = int(parts[2])
        elif parts[0] == "property" and current_element == "vertex":
            if len(parts) != 3:
                raise ValueError("Boundary vertex properties must be scalar")
            vertex_properties.append(parts[2])
        elif parts[0] == "end_header":
            data_start = line_index + 1
            break

    if not is_ascii:
        raise ValueError(f"Boundary map must use ASCII PLY format: {path}")
    if data_start is None or vertex_count is None or vertex_count <= 0:
        raise ValueError(f"Boundary PLY has no vertex data: {path}")

    required_properties = ("x", "y", "z", "poly_index")
    missing_properties = [
        property_name
        for property_name in required_properties
        if property_name not in vertex_properties
    ]
    if missing_properties:
        raise ValueError(
            "Boundary PLY is missing properties: " + ", ".join(missing_properties)
        )
    if len(lines) - data_start < vertex_count:
        raise ValueError("Boundary PLY contains fewer vertices than its header declares")

    property_indices = {
        property_name: vertex_properties.index(property_name)
        for property_name in required_properties
    }
    polygons: BoundaryPolygons = {}
    for vertex_offset in range(vertex_count):
        values = lines[data_start + vertex_offset].split()
        if len(values) < len(vertex_properties):
            raise ValueError(
                f"Boundary vertex {vertex_offset} has fewer values than properties"
            )
        try:
            x = float(values[property_indices["x"]])
            y = float(values[property_indices["y"]])
            z = float(values[property_indices["z"]])
            polygon_value = float(values[property_indices["poly_index"]])
        except ValueError as error:
            raise ValueError(
                f"Boundary vertex {vertex_offset} contains a non-numeric value"
            ) from error
        if not all(math.isfinite(value) for value in (x, y, z, polygon_value)):
            raise ValueError(f"Boundary vertex {vertex_offset} is not finite")

        polygon_index = int(round(polygon_value))
        if polygon_index < 0 or not math.isclose(
            polygon_value,
            polygon_index,
            abs_tol=1.0e-6,
        ):
            raise ValueError(
                f"Boundary vertex {vertex_offset} has an invalid polygon index"
            )
        polygons.setdefault(polygon_index, []).append((x, y, z))

    for polygon_index, polygon in polygons.items():
        if len(polygon) < 3:
            raise ValueError(
                f"Boundary polygon {polygon_index} must contain at least three vertices"
            )
    return dict(sorted(polygons.items()))


def _point(coordinates: Point3) -> Point:
    point = Point()
    point.x, point.y, point.z = coordinates
    return point


def _line_marker(
    frame_id: str,
    stamp,
    namespace: str,
    polygons: list[list[Point3]],
    line_width: float,
    color: tuple[float, float, float],
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = line_width
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 1.0

    for polygon in polygons:
        for vertex_index, start in enumerate(polygon):
            end = polygon[(vertex_index + 1) % len(polygon)]
            marker.points.append(_point(start))
            marker.points.append(_point(end))
    return marker


def build_boundary_markers(
    polygons: BoundaryPolygons,
    frame_id: str,
    stamp,
    line_width: float,
) -> MarkerArray:
    marker_array = MarkerArray()
    if 0 in polygons:
        marker_array.markers.append(
            _line_marker(
                frame_id,
                stamp,
                "outer_boundary",
                [polygons[0]],
                line_width,
                (0.0, 1.0, 0.0),
            )
        )
    obstacle_polygons = [
        polygon for polygon_index, polygon in polygons.items() if polygon_index != 0
    ]
    if obstacle_polygons:
        marker_array.markers.append(
            _line_marker(
                frame_id,
                stamp,
                "obstacle_boundaries",
                obstacle_polygons,
                line_width,
                (1.0, 0.0, 0.0),
            )
        )
    return marker_array


class BoundaryMarkerPublisher(Node):
    def __init__(self) -> None:
        super().__init__("boundary_marker_publisher")
        self.declare_parameter("pcd_file", "")
        self.declare_parameter("boundary_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("boundary_topic", "/boundary_map")
        self.declare_parameter("line_width", 0.12)

        pcd_file = str(self.get_parameter("pcd_file").value)
        boundary_file = str(self.get_parameter("boundary_file").value)
        self.frame_id = str(self.get_parameter("frame_id").value).strip()
        boundary_topic = str(self.get_parameter("boundary_topic").value).strip()
        self.line_width = float(self.get_parameter("line_width").value)

        if not self.frame_id:
            raise RuntimeError("Parameter 'frame_id' must be non-empty")
        if not boundary_topic:
            raise RuntimeError("Parameter 'boundary_topic' must be non-empty")
        if not math.isfinite(self.line_width) or self.line_width <= 0.0:
            raise RuntimeError("Parameter 'line_width' must be finite and positive")

        try:
            self.boundary_path = resolve_boundary_path(boundary_file, pcd_file)
            self.polygons = load_boundary_ply(self.boundary_path)
        except (OSError, ValueError) as error:
            raise RuntimeError(f"Failed to load boundary map: {error}") from error

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.publisher = self.create_publisher(MarkerArray, boundary_topic, qos)
        self.publish_timer = self.create_timer(1.0, self.publish_boundary)
        self.publish_boundary()

        vertex_count = sum(len(polygon) for polygon in self.polygons.values())
        self.get_logger().info(
            f"Publishing {len(self.polygons)} boundary polygons "
            f"({vertex_count} vertices) from {self.boundary_path} "
            f"on {boundary_topic}"
        )

    def publish_boundary(self) -> None:
        markers = build_boundary_markers(
            self.polygons,
            self.frame_id,
            self.get_clock().now().to_msg(),
            self.line_width,
        )
        self.publisher.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: BoundaryMarkerPublisher | None = None
    try:
        node = BoundaryMarkerPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
