import math

import rclpy
from mavros_msgs.msg import ObstacleDistance3D
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ArduPilotLidarBridge(Node):
    MAV_DISTANCE_SENSOR_LASER = 0
    MAV_FRAME_BODY_FRD = 12

    def __init__(self):
        super().__init__("ardupilot_lidar_bridge")
        self.scan_topic = self.declare_parameter(
            "scan_topic", "/asv/lidar/scan"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/mavros/obstacle/send"
        ).value
        self.max_obstacles = int(self.declare_parameter("max_obstacles", 12).value)
        self.min_separation_rad = float(
            self.declare_parameter("min_separation_rad", 0.20).value
        )
        self.max_range = float(self.declare_parameter("max_range", 18.0).value)

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(ObstacleDistance3D, self.output_topic, 20)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, scan_qos)

    def on_scan(self, scan):
        candidates = []
        angle = scan.angle_min
        for distance in scan.ranges:
            if math.isfinite(distance):
                in_range = scan.range_min <= distance <= min(scan.range_max, self.max_range)
                if in_range:
                    candidates.append((distance, angle))
            angle += scan.angle_increment

        selected = []
        for distance, angle in sorted(candidates, key=lambda item: item[0]):
            if all(abs(self.angle_delta(angle, other)) >= self.min_separation_rad for _, other in selected):
                selected.append((distance, angle))
            if len(selected) >= self.max_obstacles:
                break

        for obstacle_id, (distance, angle) in enumerate(selected):
            msg = ObstacleDistance3D()
            msg.header.stamp = scan.header.stamp
            msg.header.frame_id = "base_link"
            msg.sensor_type = self.MAV_DISTANCE_SENSOR_LASER
            msg.frame = self.MAV_FRAME_BODY_FRD
            msg.obstacle_id = obstacle_id
            msg.position.x = distance * math.cos(angle)
            msg.position.y = distance * math.sin(angle)
            msg.position.z = 0.0
            msg.min_distance = scan.range_min
            msg.max_distance = min(scan.range_max, self.max_range)
            self.publisher.publish(msg)

    @staticmethod
    def angle_delta(first, second):
        return math.atan2(math.sin(first - second), math.cos(first - second))


def main(args=None):
    rclpy.init(args=args)
    node = ArduPilotLidarBridge()
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
