import math

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class TargetBuoyDetector(Node):
    """Lightweight color selector for Course 3 target buoys."""

    def __init__(self):
        super().__init__("target_buoy_detector")
        self.declare_parameter("image_topic", "/asv/camera/front/image")
        self.declare_parameter("target_color", "green")
        self.declare_parameter("min_pixels", 180)
        self.status_pub = self.create_publisher(
            String, "/asv/perception/target_selection", 10
        )
        self.create_subscription(
            Image, str(self.get_parameter("image_topic").value), self.on_image, 5
        )

    def publish_status(self, status):
        if not rclpy.ok():
            return
        try:
            self.status_pub.publish(String(data=status))
        except RCLError:
            return

    def on_image(self, msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            self.publish_status(f"unsupported_encoding:{msg.encoding}")
            return
        step = msg.step
        width = msg.width
        height = msg.height
        if width == 0 or height == 0 or step == 0:
            return

        target = str(self.get_parameter("target_color").value).lower()
        min_pixels = int(self.get_parameter("min_pixels").value)
        count = 0
        sum_x = 0
        stride = 4
        data = msg.data
        rgb = msg.encoding == "rgb8"

        for y in range(0, height, stride):
            row = y * step
            for x in range(0, width, stride):
                i = row + x * 3
                if i + 2 >= len(data):
                    continue
                if rgb:
                    r, g, b = data[i], data[i + 1], data[i + 2]
                else:
                    b, g, r = data[i], data[i + 1], data[i + 2]
                if self.match_color(target, r, g, b):
                    count += 1
                    sum_x += x

        if count < max(1, min_pixels // (stride * stride)):
            self.publish_status(f"target={target} visible=false")
            return

        centroid_x = sum_x / count
        normalized_offset = 2.0 * (centroid_x / max(width - 1, 1)) - 1.0
        self.publish_status(
            (
                f"target={target} visible=true pixels={count * stride * stride} "
                f"offset={normalized_offset:.3f}"
            )
        )

    @staticmethod
    def match_color(target, r, g, b):
        if target == "red":
            return r > 140 and r > 1.35 * max(g, b) and g < 130
        if target == "green":
            return g > 120 and g > 1.25 * max(r, b)
        if target == "black":
            return max(r, g, b) < 70 and math.hypot(r - g, g - b) < 40
        return False


def main(args=None):
    rclpy.init(args=args)
    node = TargetBuoyDetector()
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
