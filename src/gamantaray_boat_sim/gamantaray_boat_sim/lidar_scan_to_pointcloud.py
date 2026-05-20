import rclpy
from laser_geometry import LaserProjection
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2


class LidarScanToPointCloud(Node):
    def __init__(self):
        super().__init__("lidar_scan_to_pointcloud")
        self.scan_topic = self.declare_parameter(
            "scan_topic", "/asv/lidar/scan"
        ).value
        self.cloud_topic = self.declare_parameter(
            "cloud_topic", "/asv/lidar/points_filtered"
        ).value
        self.range_cutoff = float(
            self.declare_parameter("range_cutoff", 10.0).value
        )

        self.projector = LaserProjection()
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(PointCloud2, self.cloud_topic, sensor_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, sensor_qos)

    def on_scan(self, scan):
        cloud = self.projector.projectLaser(scan, range_cutoff=self.range_cutoff)
        cloud.header = scan.header
        self.publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = LidarScanToPointCloud()
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
