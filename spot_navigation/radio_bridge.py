import json
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

import serial


class RadioBridge(Node):
    def __init__(self):
        super().__init__("radio_bridge")

        self.declare_parameter("port", "/dev/radio_usb")
        self.declare_parameter("baud", 57600)
        self.declare_parameter("odom_topic", "/odometry_map")
        self.declare_parameter("voltage_topic", "/owon/value")
        self.declare_parameter("publish_rate_hz", 5.0)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.voltage_topic = self.get_parameter("voltage_topic").get_parameter_value().string_value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        self._serial = None
        self._latest_odom = None
        self._latest_voltage = None
        self._lock = threading.Lock()

        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)
        self.create_subscription(Float32, self.voltage_topic, self._voltage_callback, 10)
        self.create_timer(1.0 / max(self.publish_rate_hz, 0.1), self._publish_frame)

        self._connect_serial()

    def _connect_serial(self):
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Opened radio serial link on {self.port} at {self.baud} baud")
        except Exception as exc:
            self._serial = None
            self.get_logger().error(f"Failed to open {self.port}: {exc}")

    def _odom_callback(self, msg: Odometry):
        with self._lock:
            self._latest_odom = msg

    def _voltage_callback(self, msg: Float32):
        with self._lock:
            self._latest_voltage = float(msg.data)

    def _publish_frame(self):
        if self._serial is None or not self._serial.is_open:
            self._connect_serial()
            return

        with self._lock:
            odom = self._latest_odom
            voltage = self._latest_voltage

        if odom is None:
            frame = {
                "stamp": None,
                "frame_id": None,
                "child_frame_id": None,
                "position": None,
                "orientation": None,
                "voltage": voltage,
            }
        else:
            frame = {
                "stamp": {
                    "sec": int(odom.header.stamp.sec),
                    "nanosec": int(odom.header.stamp.nanosec),
                },
                "frame_id": odom.header.frame_id,
                "child_frame_id": odom.child_frame_id,
                "position": {
                    "x": odom.pose.pose.position.x,
                    "y": odom.pose.pose.position.y,
                    "z": odom.pose.pose.position.z,
                },
                "orientation": {
                    "x": odom.pose.pose.orientation.x,
                    "y": odom.pose.pose.orientation.y,
                    "z": odom.pose.pose.orientation.z,
                    "w": odom.pose.pose.orientation.w,
                },
                "voltage": voltage,
            }

        try:
            self._serial.write((json.dumps(frame, separators=(",", ":")) + "\n").encode("utf-8"))
        except Exception as exc:
            self.get_logger().error(f"Serial write failed: {exc}")
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def destroy_node(self):
        if self._serial is not None and self._serial.is_open:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RadioBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
