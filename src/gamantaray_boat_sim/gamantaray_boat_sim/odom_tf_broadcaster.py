import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__("odom_tf_broadcaster")
        self.declare_parameter("odom_topic", "/asv/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self.on_odom, 20
        )

    def on_odom(self, msg):
        transform = msg.pose.pose
        orientation = transform.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

        stamped = TransformStamped()
        stamped.header.stamp = msg.header.stamp
        stamped.header.frame_id = str(self.get_parameter("odom_frame").value) or "odom"
        stamped.child_frame_id = str(self.get_parameter("base_frame").value)
        stamped.transform.translation.x = transform.position.x
        stamped.transform.translation.y = transform.position.y
        stamped.transform.translation.z = 0.0
        stamped.transform.rotation.x = 0.0
        stamped.transform.rotation.y = 0.0
        stamped.transform.rotation.z = math.sin(yaw * 0.5)
        stamped.transform.rotation.w = math.cos(yaw * 0.5)
        self.broadcaster.sendTransform(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
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
