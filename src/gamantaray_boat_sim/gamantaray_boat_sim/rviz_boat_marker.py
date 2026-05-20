import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return {
        "w": cr * cp * cy + sr * sp * sy,
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
    }


class RvizBoatMarker(Node):
    def __init__(self):
        super().__init__("rviz_boat_marker")
        self.use_heavy_mesh = self.declare_parameter("use_heavy_mesh", False).value
        self.odom_topic = self.declare_parameter("odom_topic", "/asv/odom").value
        self.fixed_frame = self.declare_parameter("fixed_frame", "odom").value
        self.publish_in_fixed_frame = self.declare_parameter(
            "publish_in_fixed_frame", True
        ).value
        self.latest_odom_pose = None
        self.latest_odom_stamp = None
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            MarkerArray, "/asv/visualization/boat_model", qos
        )
        odom_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, odom_qos)
        self.timer = self.create_timer(0.5, self.publish_boat_model)

    def on_odom(self, msg):
        self.latest_odom_pose = msg.pose.pose
        self.latest_odom_stamp = msg.header.stamp

    def publish_boat_model(self):
        if self.publish_in_fixed_frame and self.latest_odom_pose is None:
            return
        stamp = self.latest_odom_stamp if self.publish_in_fixed_frame else self.get_clock().now().to_msg()
        markers = []
        if self.use_heavy_mesh:
            markers.append(self.mesh_marker(0, stamp))
        else:
            markers.extend(self.proxy_hull_markers(stamp))

        markers.extend([
            self.cylinder_marker(
                10, stamp, "left_thruster", -0.85, 0.45, -0.2, 0.16, 0.16, 0.25,
                0.02, 0.02, 0.02, 1.0, 0.0, 1.5708, 0.0,
            ),
            self.cylinder_marker(
                11, stamp, "right_thruster", -0.85, -0.45, -0.2, 0.16, 0.16, 0.25,
                0.02, 0.02, 0.02, 1.0, 0.0, 1.5708, 0.0,
            ),
            self.cylinder_marker(
                12, stamp, "left_propeller", -0.95, 0.45, -0.05, 0.15, 0.15, 0.025,
                0.05, 0.05, 0.05, 1.0, 0.0, 1.570796, 0.0,
            ),
            self.cylinder_marker(
                13, stamp, "right_propeller", -0.95, -0.45, -0.05, 0.15, 0.15, 0.025,
                0.05, 0.05, 0.05, 1.0, 0.0, 1.570796, 0.0,
            ),
            self.cylinder_marker(
                14, stamp, "lidar_mast", 0.52, 0.0, 0.78, 0.05, 0.05, 1.42,
                0.65, 0.65, 0.65, 1.0,
            ),
            self.box_marker(
                15, stamp, "sensor_crossbar", 0.52, 0.0, 0.98, 0.16, 0.40, 0.05,
                0.72, 0.72, 0.72, 1.0,
            ),
            self.cylinder_marker(
                16, stamp, "lidar_sensor", 0.55, 0.0, 0.95, 0.16, 0.16, 0.05,
                0.05, 0.05, 0.05, 1.0,
            ),
            self.box_marker(
                17, stamp, "front_camera", 0.85, 0.0, 1.25, 0.14, 0.18, 0.10,
                0.03, 0.03, 0.03, 1.0,
            ),
        ])
        self.publisher.publish(MarkerArray(markers=markers))

    def base_marker(self, marker_id, stamp, marker_type, ns):
        marker = Marker()
        marker.header.frame_id = (
            self.fixed_frame if self.publish_in_fixed_frame else "base_link"
        )
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        return marker

    def mesh_marker(self, marker_id, stamp):
        marker = self.base_marker(marker_id, stamp, Marker.MESH_RESOURCE, "asv_mesh")
        marker.mesh_resource = (
            "package://gamantaray_boat_sim/models/gamantaray_boat/"
            "meshes/assembly_2_0.obj"
        )
        marker.mesh_use_embedded_materials = True
        self.set_pose(marker, -1.144, 0.0, -0.638, 0.0, 0.0, 0.0)
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        return marker

    def proxy_hull_markers(self, stamp):
        return [
            self.box_marker(
                0, stamp, "hull", 0.0, 0.0, -0.02, 2.35, 1.20, 0.34,
                0.90, 0.94, 0.96, 1.0,
            ),
            self.box_marker(
                1, stamp, "hull_deck", 0.15, 0.0, 0.22, 1.35, 0.82, 0.16,
                0.78, 0.82, 0.84, 1.0,
            ),
            self.box_marker(
                2, stamp, "bow_direction", 0.95, 0.0, 0.30, 0.35, 0.18, 0.08,
                0.10, 0.45, 1.00, 1.0,
            ),
        ]

    def cylinder_marker(
        self,
        marker_id,
        stamp,
        ns,
        x,
        y,
        z,
        sx,
        sy,
        sz,
        red,
        green,
        blue,
        alpha,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    ):
        marker = self.base_marker(marker_id, stamp, Marker.CYLINDER, ns)
        self.set_pose(marker, x, y, z, roll, pitch, yaw)
        self.set_scale(marker, sx, sy, sz)
        self.set_color(marker, red, green, blue, alpha)
        return marker

    def box_marker(
        self,
        marker_id,
        stamp,
        ns,
        x,
        y,
        z,
        sx,
        sy,
        sz,
        red,
        green,
        blue,
        alpha,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
    ):
        marker = self.base_marker(marker_id, stamp, Marker.CUBE, ns)
        self.set_pose(marker, x, y, z, roll, pitch, yaw)
        self.set_scale(marker, sx, sy, sz)
        self.set_color(marker, red, green, blue, alpha)
        return marker

    def set_pose(self, marker, x, y, z, roll, pitch, yaw):
        if self.publish_in_fixed_frame and self.latest_odom_pose is not None:
            boat_pose = self.latest_odom_pose
            boat_yaw = self.yaw_from_quaternion(boat_pose.orientation)
            cos_yaw = math.cos(boat_yaw)
            sin_yaw = math.sin(boat_yaw)
            marker.pose.position.x = (
                boat_pose.position.x + cos_yaw * x - sin_yaw * y
            )
            marker.pose.position.y = (
                boat_pose.position.y + sin_yaw * x + cos_yaw * y
            )
            marker.pose.position.z = boat_pose.position.z + z
            quat = quaternion_from_rpy(roll, pitch, boat_yaw + yaw)
        else:
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            quat = quaternion_from_rpy(roll, pitch, yaw)
        marker.pose.orientation.x = quat["x"]
        marker.pose.orientation.y = quat["y"]
        marker.pose.orientation.z = quat["z"]
        marker.pose.orientation.w = quat["w"]

    @staticmethod
    def yaw_from_quaternion(orientation):
        return math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

    def set_scale(self, marker, x, y, z):
        marker.scale.x = x
        marker.scale.y = y
        marker.scale.z = z

    def set_color(self, marker, red, green, blue, alpha):
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha


def main(args=None):
    rclpy.init(args=args)
    node = RvizBoatMarker()
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
