import math

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


class AntiSinkGuard(Node):
    def __init__(self):
        super().__init__("anti_sink_guard")
        self.odom_topic = self.declare_parameter("odom_topic", "/asv/odom").value
        self.service_name = self.declare_parameter(
            "set_pose_service", "/world/tecnofest_asv_course/set_pose"
        ).value
        self.entity_name = self.declare_parameter(
            "entity_name", "gamantaray_boat"
        ).value
        self.min_z = float(self.declare_parameter("min_z", -0.20).value)
        self.restore_z = float(self.declare_parameter("restore_z", 0.45).value)
        self.max_abs_x = float(self.declare_parameter("max_abs_x", 80.0).value)
        self.max_abs_y = float(self.declare_parameter("max_abs_y", 30.0).value)
        self.safe_x = float(self.declare_parameter("safe_x", -49.0).value)
        self.safe_y = float(self.declare_parameter("safe_y", -8.0).value)
        self.safe_yaw = float(self.declare_parameter("safe_yaw", 0.42).value)
        self.max_abs_roll_pitch = float(
            self.declare_parameter("max_abs_roll_pitch_rad", 1.05).value
        )
        self.min_call_period_s = float(
            self.declare_parameter("min_call_period_s", 0.50).value
        )
        self.last_call_time = None

        self.client = self.create_client(SetEntityPose, self.service_name)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)

    def on_odom(self, msg):
        pose = msg.pose.pose
        roll, pitch, yaw = self.rpy_from_quaternion(pose.orientation)
        bad_position = (
            not math.isfinite(pose.position.x)
            or not math.isfinite(pose.position.y)
            or not math.isfinite(pose.position.z)
            or abs(pose.position.x) > self.max_abs_x
            or abs(pose.position.y) > self.max_abs_y
        )
        needs_restore = bad_position or pose.position.z < self.min_z
        needs_attitude_reset = (
            abs(roll) > self.max_abs_roll_pitch or abs(pitch) > self.max_abs_roll_pitch
        )
        if not needs_restore and not needs_attitude_reset:
            return
        if not self.can_call_now():
            return
        if not self.client.service_is_ready():
            self.client.wait_for_service(timeout_sec=0.01)
            return

        req = SetEntityPose.Request()
        req.entity.name = self.entity_name
        req.entity.type = Entity.MODEL
        if bad_position:
            req.pose.position.x = self.safe_x
            req.pose.position.y = self.safe_y
            req.pose.position.z = self.restore_z
            req.pose.orientation = self.quaternion_from_rpy(0.0, 0.0, self.safe_yaw)
        else:
            req.pose.position.x = pose.position.x
            req.pose.position.y = pose.position.y
            req.pose.position.z = self.restore_z if pose.position.z < self.min_z else pose.position.z
            if needs_attitude_reset:
                req.pose.orientation = self.quaternion_from_rpy(0.0, 0.0, yaw)
            else:
                req.pose.orientation = pose.orientation

        self.last_call_time = self.get_clock().now()
        self.client.call_async(req)
        self.get_logger().warn(
            f"Restoring {self.entity_name}: x={pose.position.x:.2f}, "
            f"y={pose.position.y:.2f}, z={pose.position.z:.2f}, "
            f"roll={roll:.2f}, pitch={pitch:.2f}, bad_position={bad_position}"
        )

    def can_call_now(self):
        if self.last_call_time is None:
            return True
        elapsed = (self.get_clock().now() - self.last_call_time).nanoseconds * 1e-9
        return elapsed >= self.min_call_period_s

    @staticmethod
    def rpy_from_quaternion(q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def quaternion_from_rpy(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


def main(args=None):
    rclpy.init(args=args)
    node = AntiSinkGuard()
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
