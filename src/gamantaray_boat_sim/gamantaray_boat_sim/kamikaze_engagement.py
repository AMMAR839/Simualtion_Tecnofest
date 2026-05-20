import math
import re
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class KamikazeEngagement(Node):
    """Course 3 target engagement after Nav2 waypoint mission succeeds."""

    TARGET_RE = re.compile(
        r"target=(?P<target>\w+)\s+visible=(?P<visible>true|false)"
        r"(?:\s+pixels=(?P<pixels>\d+)\s+offset=(?P<offset>[-+0-9.]+))?"
    )

    def __init__(self):
        super().__init__("kamikaze_engagement")
        self.target_color = str(
            self.declare_parameter("target_color", "green").value
        ).strip().lower()
        self.autostart = self.as_bool(self.declare_parameter("autostart", False).value)
        self.nav_status_topic = self.declare_parameter(
            "nav_status_topic", "/asv/navigation/status"
        ).value
        self.target_topic = self.declare_parameter(
            "target_topic", "/asv/perception/target_selection"
        ).value
        self.scan_topic = self.declare_parameter("scan_topic", "/asv/lidar/scan").value
        self.odom_topic = self.declare_parameter("odom_topic", "/asv/odom").value
        self.cmd_topic = self.declare_parameter(
            "cmd_topic", "/asv/kamikaze/cmd_vel"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/asv/kamikaze/status"
        ).value

        self.visible_timeout_s = float(
            self.declare_parameter("visible_timeout_s", 0.70).value
        )
        self.align_gain = float(self.declare_parameter("align_gain", 0.85).value)
        self.max_turn_radps = abs(
            float(self.declare_parameter("max_turn_radps", 0.75).value)
        )
        self.center_tolerance = abs(
            float(self.declare_parameter("center_tolerance", 0.18).value)
        )
        self.align_forward_speed = float(
            self.declare_parameter("align_forward_speed", 0.12).value
        )
        self.attack_forward_speed = float(
            self.declare_parameter("attack_forward_speed", 0.42).value
        )
        self.sweep_turn_speed = abs(
            float(self.declare_parameter("sweep_turn_speed", 0.32).value)
        )
        self.coarse_homing_enabled = self.as_bool(
            self.declare_parameter("coarse_homing_enabled", True).value
        )
        self.coarse_forward_speed = float(
            self.declare_parameter("coarse_forward_speed", 0.28).value
        )
        self.coarse_turn_gain = float(
            self.declare_parameter("coarse_turn_gain", 0.95).value
        )
        self.coarse_max_turn_radps = abs(
            float(self.declare_parameter("coarse_max_turn_radps", 0.58).value)
        )
        self.coarse_heading_tolerance_rad = abs(
            float(self.declare_parameter("coarse_heading_tolerance_rad", 0.22).value)
        )
        self.odom_timeout_s = float(self.declare_parameter("odom_timeout_s", 1.0).value)
        self.red_target_x = float(self.declare_parameter("red_target_x", 46.0).value)
        self.red_target_y = float(self.declare_parameter("red_target_y", 6.6).value)
        self.green_target_x = float(
            self.declare_parameter("green_target_x", 46.0).value
        )
        self.green_target_y = float(
            self.declare_parameter("green_target_y", 1.8).value
        )
        self.black_target_x = float(
            self.declare_parameter("black_target_x", 46.0).value
        )
        self.black_target_y = float(
            self.declare_parameter("black_target_y", -3.0).value
        )
        self.contact_distance_m = float(
            self.declare_parameter("contact_distance_m", 0.45).value
        )
        self.contact_half_angle_rad = math.radians(
            float(self.declare_parameter("contact_half_angle_deg", 12.0).value)
        )
        self.min_contact_points = int(
            self.declare_parameter("min_contact_points", 3).value
        )
        self.push_after_contact_s = float(
            self.declare_parameter("push_after_contact_s", 1.20).value
        )
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 20.0).value
        )

        self.active = self.autostart
        self.finished = False
        self.contact_started = None
        self.latest_scan = None
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_odom_time = 0.0
        self.target_visible = False
        self.target_offset = 0.0
        self.last_target_seen = 0.0
        self.last_sweep_sign = -1.0
        self.last_status = ""

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(String, self.nav_status_topic, self.on_nav_status, 10)
        self.create_subscription(String, self.target_topic, self.on_target, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos)
        self.create_timer(1.0 / max(1.0, self.publish_rate_hz), self.on_timer)

        if self.target_color not in ("red", "green", "black"):
            self.publish_status(f"unsupported_color:{self.target_color}")

    def on_nav_status(self, msg):
        if msg.data == "nav2_waypoints_succeeded" and not self.finished:
            self.active = True
            self.publish_status(f"kamikaze_started:target={self.target_color}")

    def on_target(self, msg):
        match = self.TARGET_RE.search(msg.data)
        if not match:
            return
        if match.group("target") != self.target_color:
            return

        visible = match.group("visible") == "true"
        self.target_visible = visible
        if visible and match.group("offset") is not None:
            self.target_offset = float(match.group("offset"))
            self.last_target_seen = time.monotonic()
            if abs(self.target_offset) > self.center_tolerance:
                self.last_sweep_sign = -1.0 if self.target_offset > 0.0 else 1.0

    def on_scan(self, msg):
        self.latest_scan = msg

    def on_odom(self, msg):
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)
        orientation = msg.pose.pose.orientation
        self.odom_yaw = self.quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        self.last_odom_time = time.monotonic()

    def on_timer(self):
        cmd = Twist()
        now = time.monotonic()

        if self.finished:
            self.cmd_pub.publish(cmd)
            return
        if not self.active:
            return
        if self.target_color not in ("red", "green", "black"):
            self.publish_status(f"unsupported_color:{self.target_color}")
            self.cmd_pub.publish(cmd)
            return

        if self.contact_started is not None:
            if now - self.contact_started <= self.push_after_contact_s:
                cmd.linear.x = self.attack_forward_speed
                self.publish_status("contact_push")
            else:
                self.finished = True
                self.publish_status("kamikaze_succeeded")
            self.cmd_pub.publish(cmd)
            return

        if self.contact_detected():
            self.contact_started = now
            self.publish_status("contact")
            cmd.linear.x = self.attack_forward_speed
            self.cmd_pub.publish(cmd)
            return

        target_recent = now - self.last_target_seen <= self.visible_timeout_s
        if self.target_visible and target_recent:
            turn = -self.align_gain * self.target_offset
            cmd.angular.z = max(-self.max_turn_radps, min(self.max_turn_radps, turn))
            if abs(self.target_offset) <= self.center_tolerance:
                cmd.linear.x = self.attack_forward_speed
                self.publish_status(
                    f"track:centered:target={self.target_color}:offset={self.target_offset:.2f}"
                )
            else:
                cmd.linear.x = self.align_forward_speed
                self.publish_status(
                    f"track:align:target={self.target_color}:offset={self.target_offset:.2f}"
                )
        elif self.coarse_homing_available(now):
            cmd = self.coarse_homing_cmd()
        else:
            cmd.angular.z = self.last_sweep_sign * self.sweep_turn_speed
            self.publish_status(f"search:target={self.target_color}")

        self.cmd_pub.publish(cmd)

    def coarse_homing_available(self, now):
        return (
            self.coarse_homing_enabled
            and self.target_color in ("red", "green", "black")
            and now - self.last_odom_time <= self.odom_timeout_s
        )

    def coarse_homing_cmd(self):
        target_x, target_y = self.target_position()
        dx = target_x - self.odom_x
        dy = target_y - self.odom_y
        distance = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        heading_error = self.angle_delta(desired_yaw, self.odom_yaw)

        cmd = Twist()
        turn = self.coarse_turn_gain * heading_error
        cmd.angular.z = max(
            -self.coarse_max_turn_radps, min(self.coarse_max_turn_radps, turn)
        )
        if abs(heading_error) <= self.coarse_heading_tolerance_rad:
            cmd.linear.x = self.coarse_forward_speed
        else:
            cmd.linear.x = 0.04 if distance > 2.0 else 0.0
        self.publish_status(
            (
                f"coarse_homing:target={self.target_color}:"
                f"dist={distance:.1f}:heading={heading_error:.2f}"
            )
        )
        return cmd

    def target_position(self):
        if self.target_color == "red":
            return self.red_target_x, self.red_target_y
        if self.target_color == "green":
            return self.green_target_x, self.green_target_y
        return self.black_target_x, self.black_target_y

    def contact_detected(self):
        if self.latest_scan is None:
            return False
        count = 0
        angle = self.latest_scan.angle_min
        for distance in self.latest_scan.ranges:
            if (
                math.isfinite(distance)
                and self.latest_scan.range_min <= distance <= self.contact_distance_m
                and abs(angle) <= self.contact_half_angle_rad
            ):
                count += 1
            angle += self.latest_scan.angle_increment
        return count >= self.min_contact_points

    def publish_status(self, status):
        if status == self.last_status:
            return
        self.status_pub.publish(String(data=status))
        self.last_status = status

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def angle_delta(first, second):
        return math.atan2(math.sin(first - second), math.cos(first - second))

    @staticmethod
    def as_bool(value):
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = KamikazeEngagement()
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
