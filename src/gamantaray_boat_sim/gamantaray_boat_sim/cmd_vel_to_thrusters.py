import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelToThrusters(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_thrusters')
        self.declare_parameter('max_linear_speed', 1.2)
        self.declare_parameter('max_yaw_rate', 0.8)
        self.declare_parameter('command_timeout', 0.5)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)

        self.cmd_pub = self.create_publisher(
            Twist,
            '/model/gamantaray_boat/cmd_vel',
            10,
        )
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(1.0 / 30.0, self.publish_cmd)

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    def on_cmd_vel(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def publish_cmd(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        cmd = self.last_cmd if age <= self.command_timeout else Twist()

        out = Twist()
        out.linear.x = self.clamp(cmd.linear.x, self.max_linear_speed)
        out.angular.z = self.clamp(cmd.angular.z, self.max_yaw_rate)
        if not math.isfinite(out.linear.x) or not math.isfinite(out.angular.z):
            return

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToThrusters()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
