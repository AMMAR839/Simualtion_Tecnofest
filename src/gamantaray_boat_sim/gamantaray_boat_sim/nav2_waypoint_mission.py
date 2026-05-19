import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


class Nav2WaypointMission(Node):
    def __init__(self):
        super().__init__("nav2_waypoint_mission")
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("start_delay_s", 8.0)
        self.status_pub = self.create_publisher(String, "/asv/navigation/status", 10)

    def publish_status(self, status):
        if not rclpy.ok():
            return
        try:
            self.get_logger().info(status)
            self.status_pub.publish(String(data=status))
        except RCLError:
            return
        time.sleep(0.05)

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
        waypoints = []
        for item in data.get("waypoints", []):
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(item["x"])
            pose.pose.position.y = float(item["y"])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            waypoints.append((str(item.get("name", f"wp{len(waypoints) + 1}")), pose))

        if not waypoints:
            self.publish_status("no_waypoints")
            return

        navigator = BasicNavigator(node_name="asv_basic_navigator")
        try:
            use_sim_time = bool(self.get_parameter("use_sim_time").value)
            navigator.set_parameters(
                [Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)]
            )
            navigator.waitUntilNav2Active(localizer="robot_localization")

            self.publish_status(f"nav2_waypoints_started:{len(waypoints)}")
            for index, (name, pose) in enumerate(waypoints, start=1):
                pose.header.stamp = navigator.get_clock().now().to_msg()
                self.publish_status(f"nav2_waypoint_goal:{index}:{name}")
                navigator.goToPose(pose)
                while rclpy.ok():
                    if navigator.isTaskComplete():
                        break
                    feedback = navigator.getFeedback()
                    if feedback:
                        self.publish_status(f"nav2_running:{index}:{name}")
                    time.sleep(1.0)

                if not rclpy.ok():
                    return

                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.publish_status(f"nav2_waypoint_reached:{index}:{name}")
                    continue
                if result == TaskResult.CANCELED:
                    self.publish_status(f"nav2_waypoint_canceled:{index}:{name}")
                    self.publish_status("nav2_waypoints_canceled")
                    return
                self.publish_status(f"nav2_waypoint_failed:{index}:{name}")
                self.publish_status("nav2_waypoints_failed")
                return

            self.publish_status("nav2_waypoints_succeeded")
        except RCLError:
            return
        finally:
            try:
                navigator.destroy_node()
            except RCLError:
                pass


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
