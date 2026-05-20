import math

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class LocalWindowMarker(Node):
    def __init__(self):
        super().__init__("local_window_marker")
        self.frame_id = self.declare_parameter("frame_id", "base_link").value
        self.radius_m = float(self.declare_parameter("radius_m", 10.0).value)
        self.z_offset_m = float(self.declare_parameter("z_offset_m", 0.04).value)
        self.publish_period_s = float(
            self.declare_parameter("publish_period_s", 0.5).value
        )

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            MarkerArray, "/asv/visualization/local_window", qos
        )
        self.timer = self.create_timer(
            max(0.1, self.publish_period_s), self.publish_marker
        )

    def publish_marker(self):
        markers = MarkerArray()
        markers.markers.append(self.delete_fill_marker())
        markers.markers.append(self.outline_marker())
        self.publisher.publish(markers)

    def base_marker(self, marker_id, marker_type, namespace):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = TimeMsg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.frame_locked = True
        marker.lifetime.sec = 0
        return marker

    def delete_fill_marker(self):
        marker = self.base_marker(0, Marker.CYLINDER, "local_window_fill")
        marker.action = Marker.DELETE
        return marker

    def outline_marker(self):
        marker = self.base_marker(1, Marker.LINE_STRIP, "local_window_outline")
        marker.pose.position.z = self.z_offset_m + 0.02
        marker.scale.x = 0.035
        marker.color.r = 0.95
        marker.color.g = 0.95
        marker.color.b = 0.95
        marker.color.a = 0.90

        segments = 96
        for index in range(segments + 1):
            angle = (2.0 * math.pi * index) / segments
            marker.points.append(
                Point(
                    x=self.radius_m * math.cos(angle),
                    y=self.radius_m * math.sin(angle),
                    z=0.0,
                )
            )
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = LocalWindowMarker()
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
