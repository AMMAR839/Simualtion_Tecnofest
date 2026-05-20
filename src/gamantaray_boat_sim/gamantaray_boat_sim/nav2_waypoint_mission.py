import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def yaw_to_quaternion(yaw):
    pose_orientation = {
        "z": math.sin(yaw * 0.5),
        "w": math.cos(yaw * 0.5),
    }
    return pose_orientation


class Nav2WaypointMission(Node):
    def __init__(self):
        super().__init__("nav2_waypoint_mission")
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("start_delay_s", 8.0)
        self.declare_parameter("use_through_poses", False)
        self.declare_parameter("prereq_timeout_s", 90.0)
        self.declare_parameter("waypoint_acceptance_radius_m", 1.80)
        self.declare_parameter("waypoint_passed_margin_m", 2.00)
        self.declare_parameter("waypoint_passed_cross_track_m", 6.50)
        self.declare_parameter("waypoint_check_period_s", 0.20)
        self.declare_parameter("waypoint_advance_pause_s", 0.15)
        self.declare_parameter("status_period_s", 1.0)
        self.declare_parameter("max_goal_retries", 3)
        self.status_pub = self.create_publisher(String, "/asv/navigation/status", 10)
        self.clock_seen = False
        self.odom_seen = False
        self.scan_seen = False
        self.last_odom_xy = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.prereq_subscriptions = [
            self.create_subscription(Clock, "/clock", self.on_clock, sensor_qos),
            self.create_subscription(Odometry, "/asv/odom", self.on_odom, sensor_qos),
            self.create_subscription(
                LaserScan, "/asv/lidar/scan", self.on_scan, sensor_qos
            ),
        ]

    def publish_status(self, status):
        if not rclpy.ok():
            return
        try:
            self.get_logger().info(status)
            self.status_pub.publish(String(data=status))
        except RCLError:
            return
        time.sleep(0.05)

    def on_clock(self, _msg):
        self.clock_seen = True

    def on_odom(self, msg):
        self.odom_seen = True
        self.last_odom_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def on_scan(self, _msg):
        self.scan_seen = True

    def start(self):
        delay = float(self.get_parameter("start_delay_s").value)
        if delay > 0.0:
            self.publish_status(f"waiting_for_nav2:{delay:.1f}s")
            time.sleep(delay)
        self.run_mission()

    def run_mission(self):
        waypoint_file = Path(str(self.get_parameter("waypoint_file").value))
        if not waypoint_file.exists():
            self.publish_status(f"missing_waypoint_file:{waypoint_file}")
            return
        with waypoint_file.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)

        frame_id = str(data.get("frame_id", self.get_parameter("frame_id").value))
        start_pose = self.pose_from_item(data.get("startpoint", {}), frame_id)
        waypoints = []
        for item in data.get("waypoints", []):
            pose = self.pose_from_item(item, frame_id)
            waypoints.append((str(item.get("name", f"wp{len(waypoints) + 1}")), pose))

        self.apply_segment_orientations(waypoints)

        if not waypoints:
            self.publish_status("no_waypoints")
            return
        if not self.wait_for_runtime_ready(frame_id):
            return

        navigator = BasicNavigator(node_name="asv_basic_navigator")
        try:
            use_sim_time = bool(self.get_parameter("use_sim_time").value)
            navigator.set_parameters(
                [Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)]
            )
            navigator.waitUntilNav2Active(localizer="robot_localization")

            self.publish_status(f"nav2_waypoints_started:{len(waypoints)}")
            if bool(self.get_parameter("use_through_poses").value):
                self.run_through_poses(navigator, waypoints)
                return

            max_retries = max(0, int(self.get_parameter("max_goal_retries").value))
            check_period_s = max(
                0.05, float(self.get_parameter("waypoint_check_period_s").value)
            )
            advance_pause_s = max(
                0.0, float(self.get_parameter("waypoint_advance_pause_s").value)
            )
            status_period_s = max(
                check_period_s, float(self.get_parameter("status_period_s").value)
            )
            for index, (name, pose) in enumerate(waypoints, start=1):
                previous_pose = start_pose if index == 1 else waypoints[index - 2][1]
                waypoint_reached = False
                for attempt in range(1, max_retries + 2):
                    pose.header.stamp = navigator.get_clock().now().to_msg()
                    suffix = "" if attempt == 1 else f":retry{attempt - 1}"
                    self.publish_status(f"nav2_waypoint_goal:{index}:{name}{suffix}")
                    navigator.goToPose(pose)
                    reached_by_radius = False
                    reached_by_passed = False
                    last_running_status = 0.0
                    while rclpy.ok():
                        rclpy.spin_once(self, timeout_sec=0.0)
                        if self.distance_to_pose(pose) <= float(
                            self.get_parameter("waypoint_acceptance_radius_m").value
                        ):
                            reached_by_radius = True
                            self.cancel_current_task(navigator)
                            break
                        if self.has_passed_waypoint(previous_pose, pose):
                            reached_by_passed = True
                            self.cancel_current_task(navigator)
                            break
                        if navigator.isTaskComplete():
                            break
                        feedback = navigator.getFeedback()
                        now = time.monotonic()
                        if feedback and now - last_running_status >= status_period_s:
                            self.publish_status(f"nav2_running:{index}:{name}")
                            last_running_status = now
                        time.sleep(check_period_s)

                    if not rclpy.ok():
                        return

                    if reached_by_radius or reached_by_passed:
                        if reached_by_passed:
                            self.publish_status(f"nav2_waypoint_passed:{index}:{name}")
                        self.publish_status(f"nav2_waypoint_reached:{index}:{name}")
                        if advance_pause_s > 0.0:
                            time.sleep(advance_pause_s)
                        waypoint_reached = True
                        break

                    result = navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self.publish_status(f"nav2_waypoint_reached:{index}:{name}")
                        waypoint_reached = True
                        break
                    if self.distance_to_pose(pose) <= float(
                        self.get_parameter("waypoint_acceptance_radius_m").value
                    ):
                        self.publish_status(f"nav2_waypoint_reached:{index}:{name}")
                        waypoint_reached = True
                        break
                    if result == TaskResult.CANCELED and attempt > max_retries:
                        self.publish_status(f"nav2_waypoint_canceled:{index}:{name}")
                        self.publish_status("nav2_waypoints_canceled")
                        return
                    if attempt <= max_retries:
                        self.publish_status(
                            f"nav2_waypoint_retry:{index}:{name}:attempt{attempt}"
                        )
                        time.sleep(0.8)
                        continue
                    self.publish_status(f"nav2_waypoint_failed:{index}:{name}")
                    self.publish_status("nav2_waypoints_failed")
                    return

                if not waypoint_reached:
                    return

            self.publish_status("nav2_waypoints_succeeded")
        except RCLError:
            return
        finally:
            try:
                navigator.destroy_node()
            except RCLError:
                pass

    def wait_for_runtime_ready(self, frame_id):
        base_frame = str(self.get_parameter("base_frame").value)
        timeout_s = float(self.get_parameter("prereq_timeout_s").value)
        start_time = time.monotonic()
        last_status = 0.0
        self.publish_status("waiting_for_runtime:/clock,/asv/odom,/asv/lidar/scan,tf")

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            missing = []
            if not self.clock_seen:
                missing.append("clock")
            if not self.odom_seen:
                missing.append("odom")
            if not self.scan_seen:
                missing.append("lidar")
            if not self.tf_buffer.can_transform(
                frame_id, base_frame, Time(), timeout=Duration(seconds=0.05)
            ):
                missing.append(f"tf:{frame_id}->{base_frame}")

            if not missing:
                self.publish_status("runtime_ready")
                return True

            now = time.monotonic()
            if now - last_status >= 2.0:
                self.publish_status(f"waiting_for_runtime:{','.join(missing)}")
                last_status = now

            if timeout_s > 0.0 and now - start_time >= timeout_s:
                self.publish_status(f"runtime_prereq_timeout:{','.join(missing)}")
                return False

        return False

    def distance_to_pose(self, pose):
        if self.last_odom_xy is None:
            return float("inf")
        dx = float(pose.pose.position.x) - self.last_odom_xy[0]
        dy = float(pose.pose.position.y) - self.last_odom_xy[1]
        return math.hypot(dx, dy)

    def has_passed_waypoint(self, previous_pose, target_pose):
        if self.last_odom_xy is None or previous_pose is None:
            return False
        margin_m = float(self.get_parameter("waypoint_passed_margin_m").value)
        max_cross_track_m = float(
            self.get_parameter("waypoint_passed_cross_track_m").value
        )
        return self.segment_passed(
            (
                float(previous_pose.pose.position.x),
                float(previous_pose.pose.position.y),
            ),
            (
                float(target_pose.pose.position.x),
                float(target_pose.pose.position.y),
            ),
            self.last_odom_xy,
            margin_m,
            max_cross_track_m,
        )

    @staticmethod
    def cancel_current_task(navigator):
        navigator.cancelTask()
        start = time.monotonic()
        while rclpy.ok() and not navigator.isTaskComplete():
            if time.monotonic() - start > 2.0:
                break
            time.sleep(0.1)

    @staticmethod
    def pose_from_item(item, frame_id):
        if "x" not in item or "y" not in item:
            return None
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(item["x"])
        pose.pose.position.y = float(item["y"])
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    @staticmethod
    def segment_passed(start_xy, target_xy, current_xy, margin_m, max_cross_track_m):
        sx, sy = start_xy
        tx, ty = target_xy
        cx, cy = current_xy
        vx = tx - sx
        vy = ty - sy
        wx = cx - sx
        wy = cy - sy
        seg_len = math.hypot(vx, vy)
        if seg_len < 1.0e-6:
            return False
        along = (wx * vx + wy * vy) / seg_len
        cross = abs(wx * vy - wy * vx) / seg_len
        return along >= seg_len + margin_m and cross <= max_cross_track_m

    def apply_segment_orientations(self, waypoints):
        for index, (_, pose) in enumerate(waypoints):
            if len(waypoints) == 1:
                yaw = 0.0
            elif index < len(waypoints) - 1:
                next_pose = waypoints[index + 1][1]
                yaw = math.atan2(
                    next_pose.pose.position.y - pose.pose.position.y,
                    next_pose.pose.position.x - pose.pose.position.x,
                )
            else:
                prev_pose = waypoints[index - 1][1]
                yaw = math.atan2(
                    pose.pose.position.y - prev_pose.pose.position.y,
                    pose.pose.position.x - prev_pose.pose.position.x,
                )
            quat = yaw_to_quaternion(yaw)
            pose.pose.orientation.z = quat["z"]
            pose.pose.orientation.w = quat["w"]

    def run_through_poses(self, navigator, waypoints):
        poses = []
        for _, pose in waypoints:
            pose.header.stamp = navigator.get_clock().now().to_msg()
            poses.append(pose)

        self.publish_status(f"nav2_through_poses_goal:{len(poses)}")
        accepted = navigator.goThroughPoses(poses)
        if not accepted:
            self.publish_status("nav2_through_poses_rejected")
            return

        last_remaining = None
        while rclpy.ok():
            if navigator.isTaskComplete():
                break
            feedback = navigator.getFeedback()
            if feedback:
                remaining = int(feedback.number_of_poses_remaining)
                if remaining != last_remaining:
                    completed = len(waypoints) - remaining
                    active_index = max(1, min(len(waypoints), completed + 1))
                    active_name = waypoints[active_index - 1][0]
                    self.publish_status(
                        f"nav2_through_poses_active:{active_index}:{active_name}:remaining:{remaining}"
                    )
                    last_remaining = remaining
            time.sleep(1.0)

        if not rclpy.ok():
            return

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            for index, (name, _) in enumerate(waypoints, start=1):
                self.publish_status(f"nav2_waypoint_reached:{index}:{name}")
            self.publish_status("nav2_waypoints_succeeded")
            return
        if result == TaskResult.CANCELED:
            self.publish_status("nav2_waypoints_canceled")
            return
        self.publish_status("nav2_waypoints_failed")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2WaypointMission()
    try:
        node.start()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except RCLError:
            pass
        if rclpy.ok():
            rclpy.shutdown()
