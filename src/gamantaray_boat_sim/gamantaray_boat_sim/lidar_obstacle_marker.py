import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


class LidarObstacleMarker(Node):
    def __init__(self):
        super().__init__("lidar_obstacle_marker")
        self.scan_topic = self.declare_parameter("scan_topic", "/asv/lidar/scan").value
        self.marker_topic = self.declare_parameter(
            "marker_topic", "/asv/perception/lidar_obstacles"
        ).value
        self.max_range = float(self.declare_parameter("max_range", 18.0).value)
        self.cluster_gap_m = float(self.declare_parameter("cluster_gap_m", 0.55).value)
        self.min_cluster_points = int(
            self.declare_parameter("min_cluster_points", 4).value
        )
        self.max_markers = int(self.declare_parameter("max_markers", 60).value)
        self.publish_period_s = float(
            self.declare_parameter("publish_period_s", 0.20).value
        )
        self.last_publish_time = None

        marker_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(
            MarkerArray, self.marker_topic, marker_qos
        )
        self.subscription = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, scan_qos
        )

    def scan_callback(self, scan):
        now = self.get_clock().now()
        if self.last_publish_time is not None:
            elapsed = (now - self.last_publish_time).nanoseconds * 1e-9
            if elapsed < self.publish_period_s:
                return
        self.last_publish_time = now

        clusters = self.cluster_scan(scan)
        markers = [self.delete_all_marker(scan)]
        for marker_id, cluster in enumerate(clusters[: self.max_markers], start=1):
            markers.append(self.cluster_marker(marker_id, scan, cluster))
        self.publisher.publish(MarkerArray(markers=markers))

    def cluster_scan(self, scan):
        clusters = []
        current = []
        previous = None
        angle = scan.angle_min

        for distance in scan.ranges:
            point = None
            if math.isfinite(distance):
                in_range = scan.range_min <= distance <= min(scan.range_max, self.max_range)
                if in_range:
                    point = (distance * math.cos(angle), distance * math.sin(angle))

            if point is None:
                self.flush_cluster(clusters, current)
                current = []
                previous = None
            else:
                if previous is not None and self.point_distance(previous, point) > self.cluster_gap_m:
                    self.flush_cluster(clusters, current)
                    current = []
                current.append(point)
                previous = point

            angle += scan.angle_increment

        self.flush_cluster(clusters, current)
        clusters.sort(key=lambda item: item["range"])
        return clusters

    def flush_cluster(self, clusters, points):
        if len(points) < self.min_cluster_points:
            return
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        spread = max(max(xs) - min(xs), max(ys) - min(ys), 0.25)
        clusters.append(
            {
                "x": cx,
                "y": cy,
                "range": math.hypot(cx, cy),
                "size": min(max(spread, 0.30), 1.20),
            }
        )

    def delete_all_marker(self, scan):
        marker = Marker()
        marker.header = scan.header
        if not marker.header.frame_id:
            marker.header.frame_id = "base_link"
        marker.ns = "lidar_cluster"
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    def cluster_marker(self, marker_id, scan, cluster):
        marker = Marker()
        marker.header = scan.header
        if not marker.header.frame_id:
            marker.header.frame_id = "base_link"
        marker.ns = "lidar_cluster"
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = cluster["x"]
        marker.pose.position.y = cluster["y"]
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = cluster["size"]
        marker.scale.y = cluster["size"]
        marker.scale.z = 0.50
        marker.color.r = 1.0
        marker.color.g = 0.84
        marker.color.b = 0.05
        marker.color.a = 0.90
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = int(self.publish_period_s * 3.0 * 1e9)
        return marker

    def point_distance(self, first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleMarker()
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
