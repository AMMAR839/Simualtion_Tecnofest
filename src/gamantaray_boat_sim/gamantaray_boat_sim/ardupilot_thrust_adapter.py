import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64


class ArduPilotThrustAdapter(Node):
    """Map raw ArduPilot servo thrust commands into Gazebo thruster commands."""

    def __init__(self):
        super().__init__("ardupilot_thrust_adapter")
        self.raw_left_topic = self.declare_parameter(
            "raw_left_topic", "/asv/ardupilot/raw_left_thrust"
        ).value
        self.raw_right_topic = self.declare_parameter(
            "raw_right_topic", "/asv/ardupilot/raw_right_thrust"
        ).value
        self.direct_left_topic = self.declare_parameter(
            "direct_left_topic", "/asv/ardupilot/direct_left_thrust"
        ).value
        self.direct_right_topic = self.declare_parameter(
            "direct_right_topic", "/asv/ardupilot/direct_right_thrust"
        ).value
        self.left_output_topic = self.declare_parameter(
            "left_output_topic",
            "/model/gamantaray_boat/joint/left_propeller_joint/cmd_thrust",
        ).value
        self.right_output_topic = self.declare_parameter(
            "right_output_topic",
            "/model/gamantaray_boat/joint/right_propeller_joint/cmd_thrust",
        ).value

        self.swap_channels = self.as_bool(
            self.declare_parameter("swap_channels", True).value
        )
        self.thrust_scale = float(self.declare_parameter("thrust_scale", 1.0).value)
        self.deadband_n = abs(float(self.declare_parameter("deadband_n", 3.0).value))
        self.max_forward_thrust_n = abs(
            float(self.declare_parameter("max_forward_thrust_n", 190.0).value)
        )
        self.max_reverse_thrust_n = abs(
            float(self.declare_parameter("max_reverse_thrust_n", 90.0).value)
        )
        self.opposed_reverse_scale = self.clamp(
            float(self.declare_parameter("opposed_reverse_scale", 0.0).value),
            0.0,
            1.0,
        )
        self.slew_rate_nps = abs(float(self.declare_parameter("slew_rate_nps", 260.0).value))
        self.cmd_timeout_s = float(self.declare_parameter("cmd_timeout_s", 0.75).value)
        self.direct_cmd_timeout_s = float(
            self.declare_parameter("direct_cmd_timeout_s", 0.35).value
        )
        publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 30.0).value)

        self.raw_left = 0.0
        self.raw_right = 0.0
        self.direct_left = 0.0
        self.direct_right = 0.0
        self.output_left = 0.0
        self.output_right = 0.0
        self.last_raw_time = None
        self.last_direct_time = None
        self.last_publish_time = None

        self.left_pub = self.create_publisher(Float64, self.left_output_topic, 20)
        self.right_pub = self.create_publisher(Float64, self.right_output_topic, 20)
        self.create_subscription(Float64, self.raw_left_topic, self.on_left_raw, 20)
        self.create_subscription(Float64, self.raw_right_topic, self.on_right_raw, 20)
        self.create_subscription(Float64, self.direct_left_topic, self.on_left_direct, 20)
        self.create_subscription(Float64, self.direct_right_topic, self.on_right_direct, 20)
        self.create_timer(1.0 / max(1.0, publish_rate_hz), self.publish_thrust)

    def on_left_raw(self, msg):
        self.raw_left = float(msg.data)
        self.last_raw_time = self.get_clock().now()

    def on_right_raw(self, msg):
        self.raw_right = float(msg.data)
        self.last_raw_time = self.get_clock().now()

    def on_left_direct(self, msg):
        self.direct_left = float(msg.data)
        self.last_direct_time = self.get_clock().now()

    def on_right_direct(self, msg):
        self.direct_right = float(msg.data)
        self.last_direct_time = self.get_clock().now()

    def publish_thrust(self):
        now = self.get_clock().now()
        dt = 1.0 / 30.0
        if self.last_publish_time is not None:
            dt = max(1.0e-3, (now - self.last_publish_time).nanoseconds * 1.0e-9)
        self.last_publish_time = now

        timed_out = True
        if self.last_raw_time is not None:
            timed_out = (now - self.last_raw_time).nanoseconds * 1.0e-9 > self.cmd_timeout_s

        direct_ready = False
        if self.last_direct_time is not None:
            direct_ready = (
                (now - self.last_direct_time).nanoseconds * 1.0e-9
                <= self.direct_cmd_timeout_s
            )

        if direct_ready:
            target_left = self.limit(self.apply_deadband(self.direct_left) * self.thrust_scale)
            target_right = self.limit(self.apply_deadband(self.direct_right) * self.thrust_scale)
        elif timed_out:
            target_left = 0.0
            target_right = 0.0
        else:
            raw_left = self.raw_right if self.swap_channels else self.raw_left
            raw_right = self.raw_left if self.swap_channels else self.raw_right
            raw_left, raw_right = self.reduce_pivot_reverse(raw_left, raw_right)
            target_left = self.limit(self.apply_deadband(raw_left) * self.thrust_scale)
            target_right = self.limit(self.apply_deadband(raw_right) * self.thrust_scale)

        self.output_left = self.slew(self.output_left, target_left, dt)
        self.output_right = self.slew(self.output_right, target_right, dt)

        self.left_pub.publish(Float64(data=self.output_left))
        self.right_pub.publish(Float64(data=self.output_right))

    def apply_deadband(self, value):
        if not math.isfinite(value) or abs(value) < self.deadband_n:
            return 0.0
        return value

    def limit(self, value):
        if not math.isfinite(value):
            return 0.0
        return max(-self.max_reverse_thrust_n, min(self.max_forward_thrust_n, value))

    def reduce_pivot_reverse(self, left, right):
        if left * right >= 0.0:
            return left, right
        if left < 0.0:
            left *= self.opposed_reverse_scale
        if right < 0.0:
            right *= self.opposed_reverse_scale
        return left, right

    def slew(self, current, target, dt):
        max_step = self.slew_rate_nps * dt
        if target > current + max_step:
            return current + max_step
        if target < current - max_step:
            return current - max_step
        return target

    @staticmethod
    def as_bool(value):
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))


def main(args=None):
    rclpy.init(args=args)
    node = ArduPilotThrustAdapter()
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
