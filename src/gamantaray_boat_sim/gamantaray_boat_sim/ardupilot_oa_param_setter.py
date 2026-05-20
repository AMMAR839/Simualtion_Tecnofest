import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - handled at runtime on target system.
    mavutil = None


class ArduPilotOaParamSetter(Node):
    """Enable ArduPilot OA after SITL, EKF origin, and LiDAR are alive."""

    def __init__(self):
        super().__init__("ardupilot_oa_param_setter")
        self.enabled = self.as_bool(self.declare_parameter("enabled", True).value)
        self.mavlink_urls = list(
            self.declare_parameter(
                "mavlink_urls",
                ["tcp:127.0.0.1:5762", "tcp:127.0.0.1:5763", "tcp:127.0.0.1:5760"],
            ).value
        )
        self.scan_topic = self.declare_parameter("scan_topic", "/asv/lidar/scan").value
        self.start_delay_s = float(self.declare_parameter("start_delay_s", 55.0).value)
        self.connection_timeout_s = float(
            self.declare_parameter("connection_timeout_s", 8.0).value
        )
        self.param_timeout_s = float(self.declare_parameter("param_timeout_s", 2.0).value)
        self.require_scan = self.as_bool(
            self.declare_parameter("require_scan", True).value
        )

        self.params = {
            "OA_TYPE": float(self.declare_parameter("OA_TYPE", 3.0).value),
            "OA_BR_TYPE": float(self.declare_parameter("OA_BR_TYPE", 1.0).value),
            "OA_BR_LOOKAHEAD": float(
                self.declare_parameter("OA_BR_LOOKAHEAD", 5.0).value
            ),
            "OA_MARGIN_MAX": float(self.declare_parameter("OA_MARGIN_MAX", 1.5).value),
            "AVOID_ENABLE": float(self.declare_parameter("AVOID_ENABLE", 7.0).value),
            "AVOID_MARGIN": float(self.declare_parameter("AVOID_MARGIN", 1.5).value),
        }
        self.start_time = time.monotonic()
        self.scan_seen = False
        self.done = False

        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.create_timer(1.0, self.try_enable_oa)

    def on_scan(self, _msg):
        self.scan_seen = True

    def try_enable_oa(self):
        if self.done:
            return
        if not self.enabled:
            self.done = True
            self.get_logger().info("ArduPilot OA runtime setter disabled")
            return
        if mavutil is None:
            self.done = True
            self.get_logger().warn("pymavlink not available; OA params were not set")
            return
        if time.monotonic() - self.start_time < self.start_delay_s:
            return
        if self.require_scan and not self.scan_seen:
            self.get_logger().info("Waiting for filtered LiDAR before enabling OA")
            return

        connection = self.connect_mavlink()
        if connection is None:
            self.done = True
            self.get_logger().warn("Unable to connect MAVLink for OA param setup")
            return

        try:
            for name, value in self.params.items():
                self.set_param(connection, name, value)
            self.get_logger().info("ArduPilot OA runtime params applied")
        finally:
            connection.close()
        self.done = True

    def connect_mavlink(self):
        for url in self.mavlink_urls:
            try:
                connection = mavutil.mavlink_connection(
                    url,
                    source_system=255,
                    source_component=190,
                    autoreconnect=False,
                )
                connection.wait_heartbeat(timeout=self.connection_timeout_s)
                self.get_logger().info(f"MAVLink connected for OA params: {url}")
                return connection
            except Exception as exc:  # noqa: BLE001 - try next endpoint.
                self.get_logger().warn(f"MAVLink OA param endpoint failed {url}: {exc}")
        return None

    def set_param(self, connection, name, value):
        target_system = connection.target_system or 1
        target_component = connection.target_component or 1
        connection.mav.param_set_send(
            target_system,
            target_component,
            name.encode("ascii"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.monotonic() + self.param_timeout_s
        while time.monotonic() < deadline:
            msg = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
            if msg is None:
                continue
            param_id = msg.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode("ascii", errors="ignore")
            if param_id.rstrip("\x00") == name:
                self.get_logger().info(f"Set {name}={msg.param_value}")
                return
        self.get_logger().warn(f"No PARAM_VALUE ack for {name}; command was sent")

    @staticmethod
    def as_bool(value):
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = ArduPilotOaParamSetter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
