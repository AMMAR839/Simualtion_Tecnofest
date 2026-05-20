import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class AsvNav2SafetyRecovery(Node):
    """Final Nav2 command guard for close obstacle stop, backup, and turn."""

    NORMAL = "normal"
    STOP = "stop"
    BACKUP = "backup"
    TURN = "turn"
    COOLDOWN = "recover"

    def __init__(self):
        super().__init__("asv_nav2_safety_recovery")
        self.nav_cmd_topic = self.declare_parameter(
            "nav_cmd_topic", "/cmd_vel_nav2_smoothed"
        ).value
        self.forward_intent_topic = self.declare_parameter(
            "forward_intent_topic", self.nav_cmd_topic
        ).value
        self.kamikaze_cmd_topic = self.declare_parameter(
            "kamikaze_cmd_topic", "/asv/kamikaze/cmd_vel"
        ).value
        self.output_cmd_topic = self.declare_parameter(
            "output_cmd_topic", "/cmd_vel"
        ).value
        self.scan_topic = self.declare_parameter(
            "scan_topic", "/asv/lidar/scan"
        ).value
        self.nav_status_topic = self.declare_parameter(
            "nav_status_topic", "/asv/navigation/status"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/asv/recovery/status"
        ).value

        self.recovery_enabled = self.as_bool(
            self.declare_parameter("recovery_enabled", True).value
        )
        self.kamikaze_enabled = self.as_bool(
            self.declare_parameter("kamikaze_enabled", True).value
        )
        self.danger_distance_m = float(
            self.declare_parameter("danger_distance_m", 1.60).value
        )
        self.front_half_angle_rad = math.radians(
            float(self.declare_parameter("front_half_angle_deg", 22.0).value)
        )
        self.min_valid_range_m = float(
            self.declare_parameter("min_valid_range_m", 0.75).value
        )
        self.min_cluster_points = int(
            self.declare_parameter("min_cluster_points", 4).value
        )
        self.front_obstacle_memory_s = float(
            self.declare_parameter("front_obstacle_memory_s", 1.50).value
        )
        self.stop_duration_s = float(
            self.declare_parameter("stop_duration_s", 0.40).value
        )
        self.backup_duration_s = float(
            self.declare_parameter("backup_duration_s", 1.20).value
        )
        self.turn_duration_s = float(
            self.declare_parameter("turn_duration_s", 1.40).value
        )
        self.cooldown_duration_s = float(
            self.declare_parameter("cooldown_duration_s", 0.60).value
        )
        self.backup_speed_mps = float(
            self.declare_parameter("backup_speed_mps", -0.18).value
        )
        self.turn_speed_radps = abs(
            float(self.declare_parameter("turn_speed_radps", 0.68).value)
        )
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 20.0).value
        )
        self.cmd_timeout_s = float(self.declare_parameter("cmd_timeout_s", 0.80).value)
        self.kamikaze_cmd_timeout_s = float(
            self.declare_parameter("kamikaze_cmd_timeout_s", 0.80).value
        )

        self.nav_cmd = Twist()
        self.forward_intent_cmd = Twist()
        self.kamikaze_cmd = Twist()
        self.latest_scan = None
        self.last_nav_cmd_time = 0.0
        self.last_forward_intent_time = 0.0
        self.last_kamikaze_cmd_time = 0.0
        self.nav_waypoints_done = False
        self.state = self.NORMAL
        self.state_started = time.monotonic()
        self.turn_sign = 1.0
        self.last_front_danger_time = 0.0
        self.last_front_danger_turn_sign = 1.0
        self.last_front_danger_min_range = float("inf")
        self.last_front_danger_points = 0
        self.last_status = ""

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Twist, self.nav_cmd_topic, self.on_nav_cmd, 10)
        self.create_subscription(
            Twist, self.forward_intent_topic, self.on_forward_intent_cmd, 10
        )
        self.create_subscription(Twist, self.kamikaze_cmd_topic, self.on_kamikaze_cmd, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos)
        self.create_subscription(String, self.nav_status_topic, self.on_nav_status, 10)
        self.create_timer(1.0 / max(1.0, self.publish_rate_hz), self.on_timer)

    def on_nav_cmd(self, msg):
        self.nav_cmd = msg
        self.last_nav_cmd_time = time.monotonic()

    def on_forward_intent_cmd(self, msg):
        self.forward_intent_cmd = msg
        self.last_forward_intent_time = time.monotonic()

    def on_kamikaze_cmd(self, msg):
        self.kamikaze_cmd = msg
        self.last_kamikaze_cmd_time = time.monotonic()

    def on_scan(self, msg):
        self.latest_scan = msg

    def on_nav_status(self, msg):
        if msg.data == "nav2_waypoints_succeeded":
            self.nav_waypoints_done = True

    def on_timer(self):
        now = time.monotonic()
        cmd = Twist()

        if self.kamikaze_active(now):
            cmd = self.kamikaze_cmd
            self.publish_cmd_and_status(cmd, "kamikaze")
            return

        if not self.recovery_enabled:
            cmd = self.nav_cmd if self.nav_cmd_recent(now) else Twist()
            self.publish_cmd_and_status(cmd, "normal_passthrough")
            return

        if self.state == self.NORMAL:
            if self.forward_intent_recent(now) and self.command_wants_forward(
                self.forward_intent_cmd
            ):
                danger, turn_sign, points, min_range = self.front_danger()
                if danger:
                    self.turn_sign = turn_sign
                    self.enter_state(
                        self.STOP,
                        f"danger:points={points}:range={min_range:.2f}:turn={turn_sign:.0f}",
                    )
            cmd = self.nav_cmd if self.nav_cmd_recent(now) else Twist()
        elif self.state == self.STOP:
            if now - self.state_started >= self.stop_duration_s:
                self.enter_state(self.BACKUP, "backup")
            cmd = Twist()
        elif self.state == self.BACKUP:
            if now - self.state_started >= self.backup_duration_s:
                self.enter_state(self.TURN, "turn")
            cmd.linear.x = self.backup_speed_mps
        elif self.state == self.TURN:
            if now - self.state_started >= self.turn_duration_s:
                self.enter_state(self.COOLDOWN, "cooldown")
            cmd.angular.z = self.turn_sign * self.turn_speed_radps
        elif self.state == self.COOLDOWN:
            if now - self.state_started >= self.cooldown_duration_s:
                self.enter_state(self.NORMAL, "normal")
            cmd = self.nav_cmd if self.nav_cmd_recent(now) else Twist()

        self.publish_cmd_and_status(cmd, self.state)

    def front_danger(self):
        if self.latest_scan is None:
            return False, 1.0, 0, float("inf")

        now = time.monotonic()
        count = 0
        min_range = float("inf")
        angle = self.latest_scan.angle_min
        for distance in self.latest_scan.ranges:
            if (
                math.isfinite(distance)
                and self.min_valid_range_m <= distance <= self.danger_distance_m
                and abs(angle) <= self.front_half_angle_rad
            ):
                count += 1
                min_range = min(min_range, float(distance))
            angle += self.latest_scan.angle_increment

        if count < self.min_cluster_points:
            if now - self.last_front_danger_time <= self.front_obstacle_memory_s:
                return (
                    True,
                    self.last_front_danger_turn_sign,
                    self.last_front_danger_points,
                    self.last_front_danger_min_range,
                )
            return False, 1.0, count, min_range

        left_clear = self.side_clearance(0.45, 1.80)
        right_clear = self.side_clearance(-1.80, -0.45)
        turn_sign = 1.0 if left_clear >= right_clear else -1.0
        self.last_front_danger_time = now
        self.last_front_danger_turn_sign = turn_sign
        self.last_front_danger_min_range = min_range
        self.last_front_danger_points = count
        return True, turn_sign, count, min_range

    def side_clearance(self, angle_min, angle_max):
        if self.latest_scan is None:
            return 0.0
        values = []
        angle = self.latest_scan.angle_min
        capped_max = min(self.latest_scan.range_max, 10.0)
        for distance in self.latest_scan.ranges:
            if angle_min <= angle <= angle_max and math.isfinite(distance):
                values.append(max(self.min_valid_range_m, min(float(distance), capped_max)))
            angle += self.latest_scan.angle_increment
        if not values:
            return capped_max
        values.sort(reverse=True)
        top = values[: max(1, len(values) // 3)]
        return sum(top) / len(top)

    def enter_state(self, state, reason):
        self.state = state
        self.state_started = time.monotonic()
        self.publish_status(f"{state}:{reason}")

    def publish_cmd_and_status(self, cmd, state):
        self.cmd_pub.publish(cmd)
        self.publish_status(state)

    def publish_status(self, status):
        if status == self.last_status:
            return
        self.status_pub.publish(String(data=status))
        self.last_status = status

    def nav_cmd_recent(self, now):
        return now - self.last_nav_cmd_time <= self.cmd_timeout_s

    def forward_intent_recent(self, now):
        return now - self.last_forward_intent_time <= self.cmd_timeout_s

    def kamikaze_active(self, now):
        if not self.kamikaze_enabled or not self.nav_waypoints_done:
            return False
        return now - self.last_kamikaze_cmd_time <= self.kamikaze_cmd_timeout_s

    @staticmethod
    def command_wants_forward(cmd):
        return math.isfinite(cmd.linear.x) and cmd.linear.x > 0.03

    @staticmethod
    def as_bool(value):
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = AsvNav2SafetyRecovery()
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
