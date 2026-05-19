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
        self.declare_parameter("nav_cmd_vel_topic", "/cmd_vel_smoothed")
        self.declare_parameter(
            "left_thrust_topic",
            "/model/gamantaray_boat/joint/left_propeller_joint/cmd_thrust",
        )
        self.declare_parameter(
            "right_thrust_topic",
            "/model/gamantaray_boat/joint/right_propeller_joint/cmd_thrust",
        )
        self.declare_parameter("max_forward_thrust_n", 18.0)
        self.declare_parameter("max_reverse_thrust_n", 8.0)
        self.declare_parameter("max_speed_cmd_mps", 0.60)
        self.declare_parameter("yaw_to_thrust_n_per_radps", 12.0)
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("max_yaw_rate_cmd_radps", 0.70)
        self.declare_parameter("cmd_timeout_s", 0.8)
        self.declare_parameter("thrust_slew_rate_nps", 28.0)
        self.declare_parameter("publish_rate_hz", 30.0)

        self.max_forward = float(self.get_parameter("max_forward_thrust_n").value)
        self.max_reverse = float(self.get_parameter("max_reverse_thrust_n").value)
        self.max_speed = max(0.05, float(self.get_parameter("max_speed_cmd_mps").value))
        self.yaw_scale = float(self.get_parameter("yaw_to_thrust_n_per_radps").value)
        self.yaw_sign = float(self.get_parameter("yaw_sign").value)
        self.max_yaw_rate = max(
            0.05, float(self.get_parameter("max_yaw_rate_cmd_radps").value)
        )
        self.timeout = max(0.05, float(self.get_parameter("cmd_timeout_s").value))
        self.slew_rate = max(
            0.1, float(self.get_parameter("thrust_slew_rate_nps").value)
        )

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
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            lambda msg: self.on_cmd_vel(msg, "cmd_vel"),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("nav_cmd_vel_topic").value),
            lambda msg: self.on_cmd_vel(msg, "nav_smoothed"),
            10,
        )

        self.commands = {
            "cmd_vel": Twist(),
            "nav_smoothed": Twist(),
        }
        self.command_times = {
            "cmd_vel": 0.0,
            "nav_smoothed": 0.0,
        }
        self.active_source = "none"
        self.current_left = 0.0
        self.current_right = 0.0
        self.last_publish_time = time.monotonic()
        self.last_status = 0.0
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self.publish_thrusters)

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def on_cmd_vel(self, msg, source):
        self.commands[source] = msg
        self.command_times[source] = time.monotonic()

    def publish_thrusters(self):
        now = time.monotonic()
        cmd, source = self.select_command(now)
        if cmd is None:
            target_left = 0.0
            target_right = 0.0
            state = "timeout_stop"
        else:
            target_left, target_right = self.mix_command(cmd)
            state = "active"

        dt = max(0.0, min(now - self.last_publish_time, 0.2))
        self.last_publish_time = now
        left = self.ramp(self.current_left, target_left, self.slew_rate * dt)
        right = self.ramp(self.current_right, target_right, self.slew_rate * dt)
        self.current_left = left
        self.current_right = right
        self.active_source = source

        try:
            self.left_pub.publish(Float64(data=left))
            self.right_pub.publish(Float64(data=right))
        except RCLError:
            return

        if now - self.last_status >= 0.5:
            try:
                self.status_pub.publish(
                    String(
                        data=(
                            f"{state}:{self.active_source}: "
                            f"left={left:.2f}N right={right:.2f}N"
                        )
                    )
                )
            except RCLError:
                return
            self.last_status = now

    def select_command(self, now):
        # Prefer /cmd_vel when it exists because it is the final safety output
        # after collision_monitor, and it is also the manual control topic.
        if now - self.command_times["cmd_vel"] <= self.timeout:
            return self.commands["cmd_vel"], "cmd_vel"
        if now - self.command_times["nav_smoothed"] <= self.timeout:
            return self.commands["nav_smoothed"], "nav_smoothed"
        return None, "none"

    @staticmethod
    def ramp(current, target, max_step):
        if target > current:
            return min(target, current + max_step)
        return max(target, current - max_step)

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

        turn = self.yaw_sign * self.yaw_scale * wz
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
