import math

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class LidarLocalCostmap(Node):
    """Small RViz-only local obstacle grid built directly from filtered LiDAR."""

    def __init__(self):
        super().__init__("lidar_local_costmap")
        self.scan_topic = self.declare_parameter("scan_topic", "/asv/lidar/scan").value
        self.costmap_topic = self.declare_parameter(
            "costmap_topic", "/asv/lidar/local_costmap"
        ).value
        self.frame_id = self.declare_parameter("frame_id", "base_link").value
        self.radius_m = float(self.declare_parameter("radius_m", 10.0).value)
        self.resolution_m = float(self.declare_parameter("resolution_m", 0.10).value)
        self.sensor_x = float(self.declare_parameter("sensor_x", 0.95).value)
        self.sensor_y = float(self.declare_parameter("sensor_y", 0.0).value)
        self.min_valid_range_m = float(
            self.declare_parameter("min_valid_range_m", 0.75).value
        )
        self.max_valid_range_m = float(
            self.declare_parameter("max_valid_range_m", self.radius_m).value
        )
        self.inflation_radius_m = float(
            self.declare_parameter("inflation_radius_m", 0.38).value
        )
        self.publish_period_s = float(
            self.declare_parameter("publish_period_s", 0.20).value
        )
        self.last_publish_time = None

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(OccupancyGrid, self.costmap_topic, qos)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, scan_qos)

    def on_scan(self, scan):
        now = self.get_clock().now()
        if self.last_publish_time is not None:
            elapsed = (now - self.last_publish_time).nanoseconds * 1.0e-9
            if elapsed < self.publish_period_s:
                return
        self.last_publish_time = now

        width = int(math.ceil((2.0 * self.radius_m) / self.resolution_m))
        height = width
        grid = [0] * (width * height)
        inflation_cells = max(1, int(math.ceil(self.inflation_radius_m / self.resolution_m)))

        angle = scan.angle_min
        max_range = min(float(scan.range_max), self.max_valid_range_m)
        min_range = max(float(scan.range_min), self.min_valid_range_m)
        for distance in scan.ranges:
            if math.isfinite(distance) and min_range <= distance <= max_range:
                x = float(distance) * math.cos(angle) + self.sensor_x
                y = float(distance) * math.sin(angle) + self.sensor_y
                if math.hypot(x, y) <= self.radius_m:
                    self.mark_obstacle(grid, width, height, x, y, inflation_cells)
            angle += scan.angle_increment

        msg = OccupancyGrid()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.info = self.map_info(width, height)
        msg.data = grid
        self.publisher.publish(msg)

    def mark_obstacle(self, grid, width, height, x, y, inflation_cells):
        cx = int((x + self.radius_m) / self.resolution_m)
        cy = int((y + self.radius_m) / self.resolution_m)
        if not (0 <= cx < width and 0 <= cy < height):
            return

        for dx in range(-inflation_cells, inflation_cells + 1):
            for dy in range(-inflation_cells, inflation_cells + 1):
                if dx * dx + dy * dy > inflation_cells * inflation_cells:
                    continue
                ix = cx + dx
                iy = cy + dy
                if not (0 <= ix < width and 0 <= iy < height):
                    continue
                value = 100 if dx == 0 and dy == 0 else 72
                index = iy * width + ix
                grid[index] = max(grid[index], value)

    def map_info(self, width, height):
        info = MapMetaData()
        info.resolution = self.resolution_m
        info.width = width
        info.height = height
        origin = Pose()
        origin.position.x = -self.radius_m
        origin.position.y = -self.radius_m
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        info.origin = origin
        return info


def main(args=None):
    rclpy.init(args=args)
    node = LidarLocalCostmap()
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
