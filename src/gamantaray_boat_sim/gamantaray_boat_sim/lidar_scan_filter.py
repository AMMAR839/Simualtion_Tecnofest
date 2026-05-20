import math

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class LidarScanFilter(Node):
    def __init__(self):
        super().__init__("lidar_scan_filter")
        self.raw_topic = self.declare_parameter(
            "raw_topic", "/asv/lidar/scan_raw"
        ).value
        self.filtered_topic = self.declare_parameter(
            "filtered_topic", "/asv/lidar/scan"
        ).value
        self.min_valid_range = float(
            self.declare_parameter("min_valid_range", 0.75).value
        )
        self.fallback_frame_id = self.declare_parameter(
            "fallback_frame_id", "lidar_link"
        ).value
        self.force_frame_id = bool(
            self.declare_parameter("force_frame_id", True).value
        )
        self.restamp_scan = bool(
            self.declare_parameter("restamp_scan", True).value
        )
        self.stamp_future_offset_s = float(
            self.declare_parameter("stamp_future_offset_s", 0.15).value
        )
        self.self_filter_x_min = float(
            self.declare_parameter("self_filter_x_min", -1.35).value
        )
        self.self_filter_x_max = float(
            self.declare_parameter("self_filter_x_max", 1.05).value
        )
        self.self_filter_y_abs = float(
            self.declare_parameter("self_filter_y_abs", 0.68).value
        )
        self.sensor_x = float(
            self.declare_parameter("sensor_x", 0.55).value
        )
        self.sensor_y = float(
            self.declare_parameter("sensor_y", 0.0).value
        )
        self.keep_angle_min = float(
            self.declare_parameter("keep_angle_min", -math.pi).value
        )
        self.keep_angle_max = float(
            self.declare_parameter("keep_angle_max", math.pi).value
        )
        self.cluster_gap_m = float(
            self.declare_parameter("cluster_gap_m", 0.35).value
        )
        self.min_cluster_points = int(
            self.declare_parameter("min_cluster_points", 3).value
        )
        self.min_cluster_width_m = float(
            self.declare_parameter("min_cluster_width_m", 0.10).value
        )
        self.max_cluster_width_m = float(
            self.declare_parameter("max_cluster_width_m", 1.25).value
        )
        self.max_cluster_points = int(
            self.declare_parameter("max_cluster_points", 80).value
        )

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(LaserScan, self.filtered_topic, scan_qos)
        self.create_subscription(LaserScan, self.raw_topic, self.on_scan, scan_qos)

    def on_scan(self, scan):
        filtered = LaserScan()
        filtered.header = scan.header
        if self.force_frame_id or not filtered.header.frame_id:
            filtered.header.frame_id = self.fallback_frame_id
        if self.restamp_scan:
            stamp = self.get_clock().now()
            if self.stamp_future_offset_s > 0.0:
                stamp += Duration(seconds=self.stamp_future_offset_s)
            filtered.header.stamp = stamp.to_msg()
        filtered.angle_min = scan.angle_min
        filtered.angle_max = scan.angle_max
        filtered.angle_increment = scan.angle_increment
        filtered.time_increment = scan.time_increment
        filtered.scan_time = scan.scan_time
        filtered.range_min = max(scan.range_min, self.min_valid_range)
        filtered.range_max = scan.range_max
        filtered.intensities = list(scan.intensities)

        ranges = []
        angle = scan.angle_min
        for distance in scan.ranges:
            ranges.append(self.filtered_range(distance, angle, filtered.range_min))
            angle += scan.angle_increment

        filtered.ranges = self.remove_small_clusters(ranges, scan, filtered.range_min)
        self.publisher.publish(filtered)

    def filtered_range(self, distance, angle, range_min):
        if not math.isfinite(distance):
            return distance
        if not self.angle_is_kept(angle):
            return math.inf
        if distance < range_min:
            return math.inf

        x = distance * math.cos(angle) + self.sensor_x
        y = distance * math.sin(angle) + self.sensor_y
        inside_boat_footprint = (
            self.self_filter_x_min <= x <= self.self_filter_x_max
            and abs(y) <= self.self_filter_y_abs
        )
        if inside_boat_footprint:
            return math.inf
        return distance

    def angle_is_kept(self, angle):
        if self.keep_angle_min <= self.keep_angle_max:
            return self.keep_angle_min <= angle <= self.keep_angle_max
        return angle >= self.keep_angle_min or angle <= self.keep_angle_max

    def remove_small_clusters(self, ranges, scan, range_min):
        clusters = []
        current = []
        previous = None
        angle = scan.angle_min

        for index, distance in enumerate(ranges):
            point = None
            if math.isfinite(distance) and range_min <= distance <= scan.range_max:
                point = (distance * math.cos(angle), distance * math.sin(angle))

            if point is None:
                self.flush_cluster(clusters, current)
                current = []
                previous = None
            else:
                if (
                    previous is not None
                    and self.point_distance(previous, point) > self.cluster_gap_m
                ):
                    self.flush_cluster(clusters, current)
                    current = []
                current.append((index, point))
                previous = point

            angle += scan.angle_increment

        self.flush_cluster(clusters, current)

        kept = [math.inf] * len(ranges)
        for cluster in clusters:
            if not self.cluster_is_valid(cluster):
                continue
            for index, _point in cluster:
                kept[index] = ranges[index]
        return kept

    def flush_cluster(self, clusters, points):
        if points:
            clusters.append(points)

    def cluster_is_valid(self, cluster):
        points = [point for _index, point in cluster]
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        width = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        return (
            len(cluster) >= self.min_cluster_points
            and len(cluster) <= self.max_cluster_points
            and width >= self.min_cluster_width_m
            and width <= self.max_cluster_width_m
        )

    @staticmethod
    def point_distance(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanFilter()
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
