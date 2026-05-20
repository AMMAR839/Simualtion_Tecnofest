import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


class LidarRayMarker(Node):
    def __init__(self):
        super().__init__("lidar_ray_marker")
        self.scan_topic = self.declare_parameter(
            "scan_topic", "/asv/lidar/scan"
        ).value
        self.marker_topic = self.declare_parameter(
            "marker_topic", "/asv/visualization/lidar_rays"
        ).value
        self.marker_frame = self.declare_parameter("marker_frame", "lidar_link").value
        self.max_ray_range = float(self.declare_parameter("max_ray_range", 12.0).value)
        self.sample_step = int(self.declare_parameter("sample_step", 8).value)
        self.show_free_space = bool(
            self.declare_parameter("show_free_space", True).value
        )
        self.publish_period_s = float(
            self.declare_parameter("publish_period_s", 0.20).value
        )
        self.last_publish_time = None

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        marker_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Marker, self.marker_topic, marker_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, scan_qos)

    def on_scan(self, scan):
        now = self.get_clock().now()
        if self.last_publish_time is not None:
            elapsed = (now - self.last_publish_time).nanoseconds * 1e-9
            if elapsed < self.publish_period_s:
                return
        self.last_publish_time = now

        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = self.marker_frame or scan.header.frame_id or "lidar_link"
        marker.header.stamp.sec = 0
        marker.header.stamp.nanosec = 0
        marker.ns = "lidar_rays"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.color.r = 1.0
        marker.color.g = 0.12
        marker.color.b = 0.04
        marker.color.a = 0.45
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 250_000_000

        step = max(self.sample_step, 1)
        angle = scan.angle_min
        for index, distance in enumerate(scan.ranges):
            if index % step == 0:
                ray_range = self.ray_range(distance, scan)
                if ray_range is not None:
                    marker.points.append(Point(x=0.0, y=0.0, z=0.0))
                    marker.points.append(
                        Point(
                            x=ray_range * math.cos(angle),
                            y=ray_range * math.sin(angle),
                            z=0.0,
                        )
                    )
            angle += scan.angle_increment

        self.publisher.publish(marker)

    def ray_range(self, distance, scan):
        max_range = min(scan.range_max, self.max_ray_range)
        if math.isfinite(distance):
            if scan.range_min <= distance <= max_range:
                return distance
            return None
        if self.show_free_space:
            return max_range
        return None


def main(args=None):
    rclpy.init(args=args)
    node = LidarRayMarker()
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
