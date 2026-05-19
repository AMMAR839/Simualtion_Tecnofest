import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64, String


class CmdVelToThrusters(Node):
    """Map ROS 2 body velocity commands to left/right Gazebo thruster force."""

    def __init__(self):
        super().__init__("cmd_vel_to_thrusters")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "left_thrust_topic",
            "/model/gamantaray_boat/joint/left_propeller_joint/cmd_thrust",
        )
        self.declare_parameter(
            "right_thrust_topic",
            "/model/gamantaray_boat/joint/right_propeller_joint/cmd_thrust",
        )
        self.declare_parameter("max_forward_thrust_n", 100.0)
        self.declare_parameter("max_reverse_thrust_n", 55.0)
        self.declare_parameter("max_speed_cmd_mps", 0.65)
        self.declare_parameter("yaw_to_thrust_n_per_radps", 100.0)
        self.declare_parameter("max_yaw_rate_cmd_radps", 0.80)
        self.declare_parameter("cmd_timeout_s", 2.5)
        self.declare_parameter("publish_rate_hz", 30.0)

        self.max_forward = float(self.get_parameter("max_forward_thrust_n").value)
        self.max_reverse = float(self.get_parameter("max_reverse_thrust_n").value)
        self.max_speed = max(0.05, float(self.get_parameter("max_speed_cmd_mps").value))
        self.yaw_scale = float(self.get_parameter("yaw_to_thrust_n_per_radps").value)
        self.max_yaw_rate = max(
            0.05, float(self.get_parameter("max_yaw_rate_cmd_radps").value)
        )
        self.timeout = max(0.05, float(self.get_parameter("cmd_timeout_s").value))

        self.left_pub = self.create_publisher(
            Float64, str(self.get_parameter("left_thrust_topic").value), 10
        )
        self.right_pub = self.create_publisher(
            Float64, str(self.get_parameter("right_thrust_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, "/asv/control/thruster_status", 10
        )
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value), self.on_cmd_vel, 10
        )

        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.last_status = 0.0
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self.publish_thrusters)

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def on_cmd_vel(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    def publish_thrusters(self):
        if time.monotonic() - self.last_cmd_time > self.timeout:
            left = 0.0
            right = 0.0
            state = "timeout_stop"
        else:
            left, right = self.mix_command(self.last_cmd)
            state = "active"

        try:
            self.left_pub.publish(Float64(data=left))
            self.right_pub.publish(Float64(data=right))
        except RCLError:
            return

        now = time.monotonic()
        if now - self.last_status >= 0.5:
            try:
                self.status_pub.publish(
                    String(data=f"{state}: left={left:.2f}N right={right:.2f}N")
                )
            except RCLError:
                return
            self.last_status = now

    def mix_command(self, cmd):
        vx = cmd.linear.x if math.isfinite(cmd.linear.x) else 0.0
        wz = cmd.angular.z if math.isfinite(cmd.angular.z) else 0.0
        vx = self.clamp(vx, -self.max_speed, self.max_speed)
        wz = self.clamp(wz, -self.max_yaw_rate, self.max_yaw_rate)

        normalized = vx / self.max_speed
        if normalized >= 0.0:
            throttle = normalized * self.max_forward
        else:
            throttle = normalized * self.max_reverse

        turn = self.yaw_scale * wz
        left = self.clamp(throttle - turn, -self.max_reverse, self.max_forward)
        right = self.clamp(throttle + turn, -self.max_reverse, self.max_forward)
        return left, right


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToThrusters()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except RCLError:
            pass
        if rclpy.ok():
            rclpy.shutdown()
