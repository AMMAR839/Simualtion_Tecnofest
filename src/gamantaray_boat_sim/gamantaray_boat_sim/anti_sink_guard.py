import math
import subprocess

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class AntiSinkGuard(Node):
    def __init__(self):
        super().__init__("anti_sink_guard")
        self.odom_topic = self.declare_parameter("odom_topic", "/asv/odom").value
        self.service_name = self.declare_parameter(
            "set_pose_service", "/world/tecnofest_asv_course/set_pose"
        ).value
        self.entity_name = self.declare_parameter(
            "entity_name", "gamantaray_boat"
        ).value
        self.min_z = float(self.declare_parameter("min_z", -0.20).value)
        self.restore_z = float(self.declare_parameter("restore_z", 0.45).value)
        self.max_abs_x = float(self.declare_parameter("max_abs_x", 80.0).value)
        self.max_abs_y = float(self.declare_parameter("max_abs_y", 30.0).value)
        self.safe_x = float(self.declare_parameter("safe_x", -49.0).value)
        self.safe_y = float(self.declare_parameter("safe_y", -8.0).value)
        self.safe_yaw = float(self.declare_parameter("safe_yaw", 0.42).value)
        self.max_abs_roll_pitch = float(
            self.declare_parameter("max_abs_roll_pitch_rad", 1.05).value
        )
        self.min_call_period_s = float(
            self.declare_parameter("min_call_period_s", 0.50).value
        )
        self.service_timeout_s = float(
            self.declare_parameter("service_timeout_s", 1.0).value
        )
        self.last_call_time = None

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)

    def on_odom(self, msg):
        pose = msg.pose.pose
        roll, pitch, yaw = self.rpy_from_quaternion(pose.orientation)
        bad_position = (
            not math.isfinite(pose.position.x)
            or not math.isfinite(pose.position.y)
            or not math.isfinite(pose.position.z)
            or abs(pose.position.x) > self.max_abs_x
            or abs(pose.position.y) > self.max_abs_y
        )
        needs_restore = bad_position or pose.position.z < self.min_z
        needs_attitude_reset = (
            abs(roll) > self.max_abs_roll_pitch or abs(pitch) > self.max_abs_roll_pitch
        )
        if not needs_restore and not needs_attitude_reset:
            return
        if not self.can_call_now():
            return

        if bad_position:
            x = self.safe_x
            y = self.safe_y
            z = self.restore_z
            orientation = self.quaternion_from_rpy(0.0, 0.0, self.safe_yaw)
        else:
            x = pose.position.x
            y = pose.position.y
            z = self.restore_z if pose.position.z < self.min_z else pose.position.z
            if needs_attitude_reset:
                orientation = self.quaternion_from_rpy(0.0, 0.0, yaw)
            else:
                orientation = pose.orientation

        self.last_call_time = self.get_clock().now()
        self.call_gz_set_pose(x, y, z, orientation)
        self.get_logger().warn(
            f"Restoring {self.entity_name}: x={pose.position.x:.2f}, "
            f"y={pose.position.y:.2f}, z={pose.position.z:.2f}, "
            f"roll={roll:.2f}, pitch={pitch:.2f}, bad_position={bad_position}"
        )

    def call_gz_set_pose(self, x, y, z, orientation):
        request = (
            f'name: "{self.entity_name}" '
            f"position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }} "
            "orientation { "
            f"x: {orientation.x:.8f} y: {orientation.y:.8f} "
            f"z: {orientation.z:.8f} w: {orientation.w:.8f} "
            "}"
        )
        try:
            subprocess.run(
                [
                    "gz",
                    "service",
                    "-s",
                    self.service_name,
                    "--reqtype",
                    "gz.msgs.Pose",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    str(max(1, int(self.service_timeout_s * 1000.0))),
                    "--req",
                    request,
                ],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=self.service_timeout_s + 0.5,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warn(f"Gazebo restore service failed: {exc}")

    def can_call_now(self):
        if self.last_call_time is None:
            return True
        elapsed = (self.get_clock().now() - self.last_call_time).nanoseconds * 1e-9
        return elapsed >= self.min_call_period_s

    @staticmethod
    def rpy_from_quaternion(q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def quaternion_from_rpy(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


def main(args=None):
    rclpy.init(args=args)
    node = AntiSinkGuard()
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
