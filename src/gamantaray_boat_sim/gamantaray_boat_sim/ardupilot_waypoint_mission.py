import math
import time
from pathlib import Path

import rclpy
import yaml
from mavros_msgs.msg import CommandCode, State, Waypoint, WaypointReached
from mavros_msgs.srv import (
    CommandBool,
    CommandLong,
    SetMode,
    WaypointClear,
    WaypointPush,
    WaypointSetCurrent,
)
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String

try:
    from pymavlink import mavutil
except ImportError:  # pragma: no cover - handled at runtime on target system.
    mavutil = None


EARTH_RADIUS_M = 6378137.0


class ArduPilotWaypointMission(Node):
    """Upload the existing TEKNOFEST waypoint YAML as an ArduPilot mission.

    The Nav2 waypoint flow remains separate. This node only runs when launch
    selects navigation_mode:=ardupilot.
    """

    def __init__(self):
        super().__init__("ardupilot_waypoint_mission")
        self.declare_parameter("waypoint_file", "")
        self.declare_parameter("origin_lat_deg", -6.2)
        self.declare_parameter("origin_lon_deg", 106.816666)
        self.declare_parameter("origin_alt_m", 0.0)
        self.declare_parameter("mission_alt_m", 0.0)
        self.declare_parameter("control_mode", "guided")
        self.declare_parameter("waypoint_radius_m", 6.50)
        self.declare_parameter("virtual_waypoint_radius_m", 4.80)
        self.declare_parameter("course2_entry_waypoint_radius_m", 7.50)
        self.declare_parameter("final_waypoint_radius_m", 7.00)
        self.declare_parameter("waypoint_passed_margin_m", 2.00)
        self.declare_parameter("waypoint_passed_cross_track_m", 18.00)
        self.declare_parameter("waypoint_recede_skip_enabled", True)
        self.declare_parameter("waypoint_recede_skip_radius_m", 12.00)
        self.declare_parameter("waypoint_recede_skip_margin_m", 3.00)
        self.declare_parameter("final_waypoint_progress_fraction", 0.85)
        self.declare_parameter("enable_passed_waypoint_skip", True)
        self.declare_parameter("auto_upload", True)
        self.declare_parameter("clear_existing_mission", True)
        self.declare_parameter("set_current_index", 0)
        self.declare_parameter("mode_before_arm", "GUIDED")
        self.declare_parameter("mode_after_upload", "AUTO")
        self.declare_parameter("guided_mode", "GUIDED")
        self.declare_parameter(
            "guided_mavlink_urls",
            ["tcp:127.0.0.1:5762", "tcp:127.0.0.1:5763", "tcp:127.0.0.1:5760"],
        )
        self.declare_parameter("guided_mavlink_timeout_s", 8.0)
        self.declare_parameter("guided_require_mavros", False)
        self.declare_parameter("guided_optional_mavros_setup", False)
        self.declare_parameter("guided_setpoint_rate_hz", 4.0)
        self.declare_parameter("guided_use_lookahead_target", True)
        self.declare_parameter("guided_lookahead_m", 9.0)
        self.declare_parameter("guided_course2_route_enabled", True)
        self.declare_parameter(
            "guided_course2_route_points",
            [
                "-12.8,4.2",
                "-5.8,3.6",
                "0.5,2.8",
                "7.8,2.0",
                "14.0,1.3",
                "21.0,0.3",
                "28.5,-0.5",
                "34.0,-0.9",
            ],
        )
        self.declare_parameter("guided_direct_thrust", True)
        self.declare_parameter("direct_left_topic", "/asv/ardupilot/direct_left_thrust")
        self.declare_parameter("direct_right_topic", "/asv/ardupilot/direct_right_thrust")
        self.declare_parameter("direct_thrust_rate_hz", 20.0)
        self.declare_parameter("direct_cruise_thrust_n", 145.0)
        self.declare_parameter("direct_min_forward_thrust_n", 75.0)
        self.declare_parameter("direct_max_forward_thrust_n", 160.0)
        self.declare_parameter("direct_max_reverse_thrust_n", 12.0)
        self.declare_parameter("direct_yaw_thrust_n_per_rad", 48.0)
        self.declare_parameter("direct_max_turn_thrust_n", 70.0)
        self.declare_parameter("direct_slowdown_radius_m", 9.0)
        self.declare_parameter("direct_pivot_yaw_error_rad", 1.15)
        self.declare_parameter("direct_large_yaw_error_rad", 0.65)
        self.declare_parameter("direct_pivot_throttle_fraction", 0.16)
        self.declare_parameter("direct_large_yaw_throttle_fraction", 0.48)
        self.declare_parameter("direct_yaw_sign", 1.0)
        self.declare_parameter("direct_avoidance_enabled", True)
        self.declare_parameter("direct_avoidance_scan_topic", "/asv/lidar/scan")
        self.declare_parameter("direct_avoidance_distance_m", 3.20)
        self.declare_parameter("direct_avoidance_half_angle_deg", 34.0)
        self.declare_parameter("direct_avoidance_min_range_m", 0.75)
        self.declare_parameter("direct_avoidance_min_points", 3)
        self.declare_parameter("direct_avoidance_turn_bias_rad", 0.95)
        self.declare_parameter("direct_avoidance_throttle_fraction", 0.50)
        self.declare_parameter("direct_avoidance_memory_s", 1.20)
        self.declare_parameter("direct_backup_distance_m", 1.35)
        self.declare_parameter("direct_backup_thrust_n", 24.0)
        self.declare_parameter("direct_backup_turn_thrust_n", 96.0)
        self.declare_parameter("direct_stuck_recovery_enabled", True)
        self.declare_parameter("direct_stuck_timeout_s", 4.0)
        self.declare_parameter("direct_stuck_min_progress_m", 0.35)
        self.declare_parameter("direct_stuck_backup_duration_s", 1.25)
        self.declare_parameter("direct_stuck_turn_duration_s", 1.75)
        self.declare_parameter("direct_stuck_backup_thrust_n", 38.0)
        self.declare_parameter("direct_stuck_turn_thrust_n", 125.0)
        self.declare_parameter("direct_stuck_skip_virtual_waypoints", True)
        self.declare_parameter("direct_stuck_virtual_skip_max_distance_m", 60.0)
        self.declare_parameter("arm_vehicle", True)
        self.declare_parameter("force_arm_if_needed", True)
        self.declare_parameter("rtl_on_finish", False)
        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("mission_service_namespace", "")
        self.declare_parameter("command_service_namespace", "")
        self.declare_parameter("mode_service_namespace", "")
        self.declare_parameter("connection_timeout_s", 120.0)
        self.declare_parameter("service_timeout_s", 60.0)
        self.declare_parameter("status_period_s", 1.0)

        mavros_ns = str(self.get_parameter("mavros_namespace").value).rstrip("/")
        mission_service_ns = str(
            self.get_parameter("mission_service_namespace").value
        ).strip().rstrip("/")
        if not mission_service_ns:
            # MAVROS ROS 2 exposes the waypoint services under this namespace
            # even though the state topic remains /mavros/state.
            mission_service_ns = f"{mavros_ns}/mavros"
        command_service_ns = str(
            self.get_parameter("command_service_namespace").value
        ).strip().rstrip("/")
        mode_service_ns = str(
            self.get_parameter("mode_service_namespace").value
        ).strip().rstrip("/")
        self.status_pub = self.create_publisher(
            String, "/asv/ardupilot/navigation/status", 10
        )
        self.direct_left_pub = self.create_publisher(
            Float64, str(self.get_parameter("direct_left_topic").value), 20
        )
        self.direct_right_pub = self.create_publisher(
            Float64, str(self.get_parameter("direct_right_topic").value), 20
        )
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(State, f"{mavros_ns}/state", self.on_state, 10)
        self.create_subscription(Odometry, "/asv/odom", self.on_odom, qos)
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("direct_avoidance_scan_topic").value),
            self.on_scan,
            qos,
        )
        self.create_subscription(
            WaypointReached,
            f"{mavros_ns}/mission/reached",
            self.on_waypoint_reached,
            10,
        )

        self.clear_client = self.create_client(
            WaypointClear, f"{mission_service_ns}/clear"
        )
        self.push_client = self.create_client(
            WaypointPush, f"{mission_service_ns}/push"
        )
        self.set_current_client = self.create_client(
            WaypointSetCurrent, f"{mission_service_ns}/set_current"
        )
        self.mode_clients = self.create_service_candidates(
            SetMode,
            [
                f"{mode_service_ns}/set_mode" if mode_service_ns else "",
                f"{mavros_ns}/set_mode",
                f"{mavros_ns}/mavros/set_mode",
            ],
        )
        self.arm_clients = self.create_service_candidates(
            CommandBool,
            [
                f"{command_service_ns}/arming" if command_service_ns else "",
                f"{command_service_ns}/cmd/arming" if command_service_ns else "",
                f"{mavros_ns}/mavros/arming",
                f"{mavros_ns}/cmd/arming",
            ],
        )
        self.command_clients = self.create_service_candidates(
            CommandLong,
            [
                f"{command_service_ns}/command" if command_service_ns else "",
                f"{mavros_ns}/mavros/command",
                f"{mavros_ns}/cmd/command",
            ],
        )
        self.mode_client = None
        self.arm_client = None
        self.command_client = None

        self.state = None
        self.odom_xy = None
        self.odom_yaw = 0.0
        self.latest_scan = None
        self.last_scan_time = 0.0
        self.last_avoidance_time = 0.0
        self.last_avoidance_turn_sign = 1.0
        self.last_avoidance_min_range = float("inf")
        self.last_avoidance_points = 0
        self.direct_progress_index = -1
        self.direct_best_distance = float("inf")
        self.direct_last_progress_time = 0.0
        self.direct_recovery_until = 0.0
        self.direct_recovery_backup_until = 0.0
        self.direct_recovery_turn_sign = 1.0
        self.start_xy = None
        self.local_waypoints = []
        self.active_index = 0
        self.closest_distance_by_index = {}
        self.mission_uploaded = False
        self.finish_reported = False
        self.last_status_time = 0.0
        self.control_mode = (
            str(self.get_parameter("control_mode").value).strip().lower()
        )
        self.guided_mavlink = None
        self.guided_start_time = time.monotonic()

        self.monitor_timer = self.create_timer(0.25, self.monitor_progress)
        guided_period = 1.0 / max(
            1.0, float(self.get_parameter("guided_setpoint_rate_hz").value)
        )
        self.guided_timer = self.create_timer(guided_period, self.publish_guided_target)
        direct_period = 1.0 / max(
            1.0, float(self.get_parameter("direct_thrust_rate_hz").value)
        )
        self.direct_thrust_timer = self.create_timer(
            direct_period, self.publish_direct_thrust
        )

    def create_service_candidates(self, srv_type, service_names):
        clients = []
        seen = set()
        for service_name in service_names:
            service_name = service_name.strip().replace("//", "/")
            if not service_name or service_name in seen:
                continue
            seen.add(service_name)
            clients.append((service_name, self.create_client(srv_type, service_name)))
        return clients

    def publish_status(self, status):
        self.get_logger().info(status)
        self.status_pub.publish(String(data=status))

    def on_state(self, msg):
        self.state = msg

    def on_odom(self, msg):
        self.odom_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        self.odom_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def on_scan(self, msg):
        self.latest_scan = msg
        self.last_scan_time = time.monotonic()

    def on_waypoint_reached(self, msg):
        reached = int(msg.wp_seq)
        self.active_index = max(self.active_index, reached + 1)
        self.closest_distance_by_index.pop(reached, None)
        self.publish_status(f"ardupilot_waypoint_reached:{reached}")

    def start(self):
        if not bool(self.get_parameter("auto_upload").value):
            self.publish_status("ardupilot_auto_upload_disabled")
            return

        if not self.load_waypoints():
            return

        if self.control_mode == "guided":
            self.start_guided_mode()
            return

        if self.control_mode not in ("auto_mission", "auto"):
            self.publish_status(f"ardupilot_unknown_control_mode:{self.control_mode}")
            return

        if not self.wait_for_mavros():
            return

        if not self.prepare_services(include_mission=True):
            return

        if bool(self.get_parameter("clear_existing_mission").value):
            if not self.call_clear_mission():
                return

        mission = self.build_mavros_mission()
        if not mission:
            self.publish_status("ardupilot_no_waypoints")
            return

        if not self.call_push_mission(mission):
            return

        current_index = max(0, int(self.get_parameter("set_current_index").value))
        current_index = min(current_index, len(mission) - 1)
        self.call_set_current(current_index)
        self.active_index = current_index
        self.closest_distance_by_index.clear()
        self.mission_uploaded = True

        mode_before_arm = str(self.get_parameter("mode_before_arm").value).strip()
        if mode_before_arm:
            self.call_set_mode(mode_before_arm)

        if bool(self.get_parameter("arm_vehicle").value):
            self.call_arm(True)

        mode_after_upload = str(self.get_parameter("mode_after_upload").value).strip()
        if mode_after_upload:
            self.call_set_mode(mode_after_upload)

        self.publish_status(
            f"ardupilot_mission_started:{len(mission)}:mode:{mode_after_upload or 'unchanged'}"
        )

    def start_guided_mode(self):
        direct_enabled = bool(self.get_parameter("guided_direct_thrust").value)
        require_mavros = bool(self.get_parameter("guided_require_mavros").value)

        if direct_enabled and not require_mavros:
            self.active_index = 0
            self.closest_distance_by_index.clear()
            self.mission_uploaded = True
            self.finish_reported = False
            self.guided_start_time = time.monotonic()
            self.publish_status(
                f"ardupilot_guided_direct_started:{len(self.local_waypoints)}"
            )
            if mavutil is None:
                self.publish_status("ardupilot_guided_mavlink_skipped:pymavlink_missing")
                return
            if self.state is None or not self.state.connected:
                self.publish_status("ardupilot_guided_mavros_optional:not_connected")
                return
            if not bool(self.get_parameter("guided_optional_mavros_setup").value):
                self.publish_status("ardupilot_guided_mavros_optional:setup_skipped")
                return
            self.try_prepare_guided_mavros_best_effort()
            return

        if mavutil is None:
            self.publish_status("ardupilot_guided_missing_pymavlink")
            return
        if not self.wait_for_mavros():
            return
        if not self.prepare_services(include_mission=False):
            return
        self.guided_mavlink = self.connect_guided_mavlink()
        if self.guided_mavlink is None:
            self.publish_status("ardupilot_guided_mavlink_unavailable")
            return
        self.active_index = 0
        self.closest_distance_by_index.clear()
        self.mission_uploaded = True
        self.finish_reported = False
        self.guided_start_time = time.monotonic()

        mode_before_arm = str(self.get_parameter("mode_before_arm").value).strip()
        if mode_before_arm:
            self.call_set_mode(mode_before_arm)

        if bool(self.get_parameter("arm_vehicle").value):
            self.call_arm(True)

        guided_mode = str(self.get_parameter("guided_mode").value).strip() or "GUIDED"
        self.call_set_mode(guided_mode)
        self.publish_status(
            f"ardupilot_guided_started:{len(self.local_waypoints)}:mode:{guided_mode}"
        )

    def try_prepare_guided_mavros_best_effort(self):
        if not self.prepare_services(include_mission=False):
            self.publish_status("ardupilot_guided_mavros_optional:services_unavailable")
            return
        self.guided_mavlink = self.connect_guided_mavlink()
        if self.guided_mavlink is None:
            self.publish_status("ardupilot_guided_mavros_optional:mavlink_unavailable")
            return

        mode_before_arm = str(self.get_parameter("mode_before_arm").value).strip()
        if mode_before_arm:
            self.call_set_mode(mode_before_arm, wait=False)

        if bool(self.get_parameter("arm_vehicle").value):
            self.call_arm(True)

        guided_mode = str(self.get_parameter("guided_mode").value).strip() or "GUIDED"
        self.call_set_mode(guided_mode, wait=False)
        self.publish_status(f"ardupilot_guided_mavros_optional:started:{guided_mode}")

    def load_waypoints(self):
        waypoint_file = Path(str(self.get_parameter("waypoint_file").value))
        if not waypoint_file.exists():
            self.publish_status(f"ardupilot_missing_waypoint_file:{waypoint_file}")
            return False

        with waypoint_file.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        startpoint = data.get("startpoint") or {}
        if "x" in startpoint and "y" in startpoint:
            self.start_xy = (float(startpoint["x"]), float(startpoint["y"]))

        self.local_waypoints = []
        for item in data.get("waypoints", []):
            x = float(item["x"])
            y = float(item["y"])
            lat, lon = self.enu_to_wgs84(x, y)
            self.local_waypoints.append(
                {
                    "name": str(item.get("name", f"wp{len(self.local_waypoints) + 1}")),
                    "x": x,
                    "y": y,
                    "lat": lat,
                    "lon": lon,
                }
            )

        if self.control_mode == "guided" and bool(
            self.get_parameter("guided_course2_route_enabled").value
        ):
            self.local_waypoints = self.expand_course2_route(self.local_waypoints)

        self.publish_status(f"ardupilot_waypoints_loaded:{len(self.local_waypoints)}")
        return bool(self.local_waypoints)

    def expand_course2_route(self, waypoints):
        if len(waypoints) < 2:
            return waypoints

        route_points = self.parse_route_points(
            self.get_parameter("guided_course2_route_points").value
        )
        if not route_points:
            return waypoints

        expanded = []
        inserted = False
        for index, waypoint in enumerate(waypoints):
            expanded.append(waypoint)
            if index >= len(waypoints) - 1:
                continue

            current_name = str(waypoint.get("name", "")).lower()
            next_name = str(waypoints[index + 1].get("name", "")).lower()
            if "gn4" not in current_name or "gn5" not in next_name:
                continue

            for route_index, (x, y) in enumerate(route_points, start=1):
                lat, lon = self.enu_to_wgs84(x, y)
                expanded.append(
                    {
                        "name": f"p2_safe_{route_index:02d}",
                        "x": x,
                        "y": y,
                        "lat": lat,
                        "lon": lon,
                        "virtual": True,
                    }
                )
            inserted = True

        if inserted:
            self.publish_status(f"ardupilot_course2_route_inserted:{len(route_points)}")
        return expanded

    @staticmethod
    def parse_route_points(value):
        points = []
        for item in value:
            if isinstance(item, str):
                pieces = [piece.strip() for piece in item.split(",")]
            else:
                pieces = list(item)
            if len(pieces) < 2:
                continue
            try:
                points.append((float(pieces[0]), float(pieces[1])))
            except (TypeError, ValueError):
                continue
        return points

    def wait_for_mavros(self):
        timeout_s = float(self.get_parameter("connection_timeout_s").value)
        start = time.monotonic()
        self.publish_status("ardupilot_waiting_for_mavros")
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state is not None and self.state.connected:
                self.publish_status("ardupilot_mavros_connected")
                return True
            if time.monotonic() - start > timeout_s:
                self.publish_status("ardupilot_mavros_connection_timeout")
                return False
        return False

    def prepare_services(self, include_mission=True):
        timeout_s = float(self.get_parameter("service_timeout_s").value)
        if include_mission:
            clients = [
                ("mission_clear", self.clear_client),
                ("mission_push", self.push_client),
                ("mission_set_current", self.set_current_client),
            ]
            for name, client in clients:
                if not client.wait_for_service(timeout_sec=timeout_s):
                    self.publish_status(f"ardupilot_service_unavailable:{name}")
                    return False
        self.mode_client = self.select_first_available_service(
            "set_mode", self.mode_clients, timeout_s
        )
        if self.mode_client is None:
            return False
        self.arm_client = self.select_first_available_service(
            "arming", self.arm_clients, timeout_s
        )
        if self.arm_client is None:
            return False
        if bool(self.get_parameter("force_arm_if_needed").value):
            self.command_client = self.select_first_available_service(
                "command_long", self.command_clients, timeout_s
            )
            if self.command_client is None:
                return False
        return True

    def select_first_available_service(self, label, clients, timeout_s):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            for service_name, client in clients:
                if client.service_is_ready():
                    self.publish_status(f"ardupilot_service_ready:{label}:{service_name}")
                    return client
            for _, client in clients:
                client.wait_for_service(timeout_sec=0.15)
            rclpy.spin_once(self, timeout_sec=0.05)
        names = ",".join(name for name, _ in clients)
        self.publish_status(f"ardupilot_service_unavailable:{label}:{names}")
        return None

    def build_mavros_mission(self):
        radius_m = float(self.get_parameter("waypoint_radius_m").value)
        pass_margin_m = float(self.get_parameter("waypoint_passed_margin_m").value)
        altitude_m = float(self.get_parameter("mission_alt_m").value)

        mission = []
        for index, item in enumerate(self.local_waypoints):
            lat, lon = self.enu_to_wgs84(float(item["x"]), float(item["y"]))
            waypoint = Waypoint()
            waypoint.frame = Waypoint.FRAME_GLOBAL_RELATIVE_ALT_INT
            waypoint.command = CommandCode.NAV_WAYPOINT
            waypoint.is_current = index == 0
            waypoint.autocontinue = True
            waypoint.param1 = 0.0
            waypoint.param2 = radius_m
            waypoint.param3 = pass_margin_m
            waypoint.param4 = float("nan")
            waypoint.x_lat = lat
            waypoint.y_long = lon
            waypoint.z_alt = altitude_m
            mission.append(waypoint)
        return mission

    def enu_to_wgs84(self, east_m, north_m):
        lat0 = math.radians(float(self.get_parameter("origin_lat_deg").value))
        lon0 = math.radians(float(self.get_parameter("origin_lon_deg").value))
        lat = lat0 + north_m / EARTH_RADIUS_M
        lon = lon0 + east_m / (EARTH_RADIUS_M * math.cos(lat0))
        return math.degrees(lat), math.degrees(lon)

    def call_clear_mission(self):
        request = WaypointClear.Request()
        future = self.clear_client.call_async(request)
        if not self.wait_future(future):
            self.publish_status("ardupilot_clear_mission_timeout")
            return False
        if not future.result().success:
            self.publish_status("ardupilot_clear_mission_failed")
            return False
        self.publish_status("ardupilot_clear_mission_ok")
        return True

    def call_push_mission(self, waypoints):
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = waypoints
        future = self.push_client.call_async(request)
        if not self.wait_future(future):
            self.publish_status("ardupilot_push_mission_timeout")
            return False
        result = future.result()
        if not result.success:
            self.publish_status("ardupilot_push_mission_failed")
            return False
        self.publish_status(f"ardupilot_push_mission_ok:{result.wp_transfered}")
        return True

    def call_set_current(self, index, wait=True):
        request = WaypointSetCurrent.Request()
        request.wp_seq = int(index)
        future = self.set_current_client.call_async(request)
        if not wait:
            future.add_done_callback(
                lambda done_future: self.on_set_current_done(done_future, index)
            )
            return True
        if not self.wait_future(future):
            self.publish_status(f"ardupilot_set_current_timeout:{index}")
            return False
        if not future.result().success:
            self.publish_status(f"ardupilot_set_current_failed:{index}")
            return False
        self.publish_status(f"ardupilot_set_current_ok:{index}")
        return True

    def on_set_current_done(self, future, index):
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - status must survive MAVROS errors.
            self.publish_status(f"ardupilot_set_current_error:{index}:{exc}")
            return
        if result.success:
            self.publish_status(f"ardupilot_set_current_ok:{index}")
        else:
            self.publish_status(f"ardupilot_set_current_failed:{index}")

    def call_set_mode(self, mode, wait=True):
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = str(mode)
        future = self.mode_client.call_async(request)
        if not wait:
            future.add_done_callback(
                lambda done_future: self.on_set_mode_done(done_future, mode)
            )
            return True
        if not self.wait_future(future):
            self.publish_status(f"ardupilot_set_mode_timeout:{mode}")
            return False
        result = future.result()
        status = "ok" if result.mode_sent else "failed"
        self.publish_status(f"ardupilot_set_mode_{status}:{mode}")
        return bool(result.mode_sent)

    def on_set_mode_done(self, future, mode):
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - status must survive MAVROS errors.
            self.publish_status(f"ardupilot_set_mode_error:{mode}:{exc}")
            return
        status = "ok" if result.mode_sent else "failed"
        self.publish_status(f"ardupilot_set_mode_{status}:{mode}")

    def call_arm(self, value):
        request = CommandBool.Request()
        request.value = bool(value)
        future = self.arm_client.call_async(request)
        if not self.wait_future(future):
            self.publish_status("ardupilot_arm_timeout")
            return False
        result = future.result()
        if result.success:
            self.publish_status("ardupilot_arm_ok")
            return True
        self.publish_status(f"ardupilot_arm_failed:{result.result}")
        if bool(self.get_parameter("force_arm_if_needed").value):
            return self.call_force_arm(value)
        return False

    def call_force_arm(self, value):
        if self.command_client is None:
            self.publish_status("ardupilot_force_arm_unavailable")
            return False
        request = CommandLong.Request()
        request.broadcast = False
        request.command = CommandCode.COMPONENT_ARM_DISARM
        request.confirmation = 0
        request.param1 = 1.0 if value else 0.0
        # ArduPilot force-arm magic value. This is only enabled for SITL so
        # prearm sensor checks do not block simulation bringup.
        request.param2 = 21196.0 if value else 0.0
        future = self.command_client.call_async(request)
        if not self.wait_future(future):
            self.publish_status("ardupilot_force_arm_timeout")
            return False
        result = future.result()
        status = "ok" if result.success else f"failed:{result.result}"
        self.publish_status(f"ardupilot_force_arm_{status}")
        return bool(result.success)

    def wait_future(self, future):
        timeout_s = float(self.get_parameter("service_timeout_s").value)
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - start > timeout_s:
                return False
        return future.done()

    def connect_guided_mavlink(self):
        urls = list(self.get_parameter("guided_mavlink_urls").value)
        timeout_s = float(self.get_parameter("guided_mavlink_timeout_s").value)
        for url in urls:
            try:
                connection = mavutil.mavlink_connection(
                    url,
                    source_system=255,
                    source_component=191,
                    autoreconnect=False,
                )
                connection.wait_heartbeat(timeout=timeout_s)
                self.publish_status(f"ardupilot_guided_mavlink_connected:{url}")
                return connection
            except Exception as exc:  # noqa: BLE001 - try the next SITL port.
                self.publish_status(f"ardupilot_guided_mavlink_failed:{url}:{exc}")
        return None

    def publish_guided_target(self):
        if self.control_mode != "guided":
            return
        if not self.mission_uploaded or self.active_index >= len(self.local_waypoints):
            return
        if self.guided_mavlink is None:
            return

        target_x, target_y = self.guided_target_xy()
        target_lat, target_lon = self.enu_to_wgs84(target_x, target_y)
        ignore_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        time_boot_ms = int((time.monotonic() - self.guided_start_time) * 1000.0) & 0xFFFFFFFF
        self.guided_mavlink.mav.set_position_target_global_int_send(
            time_boot_ms,
            self.guided_mavlink.target_system or 1,
            self.guided_mavlink.target_component or 1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            ignore_mask,
            int(target_lat * 1.0e7),
            int(target_lon * 1.0e7),
            float(self.get_parameter("mission_alt_m").value),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def guided_target_xy(self):
        target = self.local_waypoints[self.active_index]
        target_xy = (float(target["x"]), float(target["y"]))
        if (
            bool(target.get("virtual", False))
            or not bool(self.get_parameter("guided_use_lookahead_target").value)
            or self.active_index >= len(self.local_waypoints) - 1
        ):
            return target_xy

        next_target = self.local_waypoints[self.active_index + 1]
        vx = float(next_target["x"]) - target_xy[0]
        vy = float(next_target["y"]) - target_xy[1]
        seg_len = math.hypot(vx, vy)
        if seg_len < 1.0e-6:
            return target_xy

        lookahead_m = max(0.0, float(self.get_parameter("guided_lookahead_m").value))
        return (
            target_xy[0] + lookahead_m * vx / seg_len,
            target_xy[1] + lookahead_m * vy / seg_len,
        )

    def publish_direct_thrust(self):
        if self.control_mode != "guided":
            return
        if not bool(self.get_parameter("guided_direct_thrust").value):
            return
        if not self.mission_uploaded or self.active_index >= len(self.local_waypoints):
            self.publish_direct_pair(0.0, 0.0)
            return
        if self.odom_xy is None:
            return

        target_x, target_y = self.guided_target_xy()
        dx = target_x - self.odom_xy[0]
        dy = target_y - self.odom_xy[1]
        distance = math.hypot(dx, dy)
        if distance < 1.0e-6:
            self.publish_direct_pair(0.0, 0.0)
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.angle_delta(desired_yaw, self.odom_yaw)
        avoid_danger, avoid_backup, avoid_turn_sign, _points, _min_range = (
            self.direct_avoidance_state()
        )
        stuck_active, stuck_backup, stuck_turn_sign = self.direct_stuck_state(
            distance, yaw_error
        )
        cruise = float(self.get_parameter("direct_cruise_thrust_n").value)
        min_forward = float(self.get_parameter("direct_min_forward_thrust_n").value)
        slowdown_radius = max(
            0.1, float(self.get_parameter("direct_slowdown_radius_m").value)
        )
        throttle = cruise * min(1.0, max(0.35, distance / slowdown_radius))

        yaw_sign = float(self.get_parameter("direct_yaw_sign").value)
        max_forward = abs(float(self.get_parameter("direct_max_forward_thrust_n").value))
        max_reverse = abs(float(self.get_parameter("direct_max_reverse_thrust_n").value))
        if stuck_active:
            backup = abs(float(self.get_parameter("direct_stuck_backup_thrust_n").value))
            turn_power = abs(
                float(self.get_parameter("direct_stuck_turn_thrust_n").value)
            )
            turn = yaw_sign * stuck_turn_sign * turn_power
            if stuck_backup:
                base = -backup
            else:
                base = min_forward * 0.35
            left = self.clamp(base - turn, -max_reverse, max_forward)
            right = self.clamp(base + turn, -max_reverse, max_forward)
            self.publish_direct_pair(left, right)
            return

        if avoid_backup:
            backup = abs(float(self.get_parameter("direct_backup_thrust_n").value))
            backup_turn = abs(
                float(self.get_parameter("direct_backup_turn_thrust_n").value)
            )
            turn = yaw_sign * avoid_turn_sign * backup_turn
            left = self.clamp(-backup - turn, -max_reverse, max_forward)
            right = self.clamp(-backup + turn, -max_reverse, max_forward)
            self.publish_direct_pair(left, right)
            return

        if avoid_danger:
            turn_bias = abs(
                float(self.get_parameter("direct_avoidance_turn_bias_rad").value)
            )
            yaw_error = self.angle_delta(
                yaw_error + avoid_turn_sign * turn_bias,
                0.0,
            )

        abs_error = abs(yaw_error)
        pivot_error = abs(float(self.get_parameter("direct_pivot_yaw_error_rad").value))
        large_error = abs(float(self.get_parameter("direct_large_yaw_error_rad").value))
        pivot_fraction = self.clamp(
            float(self.get_parameter("direct_pivot_throttle_fraction").value),
            0.0,
            1.0,
        )
        large_fraction = self.clamp(
            float(self.get_parameter("direct_large_yaw_throttle_fraction").value),
            0.0,
            1.0,
        )
        if abs_error > pivot_error:
            throttle = min(throttle, cruise * pivot_fraction)
        elif abs_error > large_error:
            throttle = min(throttle, max(min_forward, cruise * large_fraction))
        else:
            throttle = max(min_forward, throttle)

        if avoid_danger:
            throttle *= self.clamp(
                float(self.get_parameter("direct_avoidance_throttle_fraction").value),
                0.05,
                1.0,
            )

        yaw_gain = float(self.get_parameter("direct_yaw_thrust_n_per_rad").value)
        max_turn = abs(float(self.get_parameter("direct_max_turn_thrust_n").value))
        turn = yaw_sign * self.clamp(yaw_gain * yaw_error, -max_turn, max_turn)
        left = self.clamp(throttle - turn, -max_reverse, max_forward)
        right = self.clamp(throttle + turn, -max_reverse, max_forward)
        self.publish_direct_pair(left, right)

    def direct_avoidance_state(self):
        if not bool(self.get_parameter("direct_avoidance_enabled").value):
            return False, False, 1.0, 0, float("inf")
        if self.latest_scan is None:
            return False, False, 1.0, 0, float("inf")
        now = time.monotonic()
        if now - self.last_scan_time > 1.0:
            return False, False, 1.0, 0, float("inf")

        danger_distance = float(
            self.get_parameter("direct_avoidance_distance_m").value
        )
        half_angle = math.radians(
            float(self.get_parameter("direct_avoidance_half_angle_deg").value)
        )
        min_range = max(
            float(self.latest_scan.range_min),
            float(self.get_parameter("direct_avoidance_min_range_m").value),
        )
        min_points = int(self.get_parameter("direct_avoidance_min_points").value)

        count = 0
        nearest = float("inf")
        angle = self.latest_scan.angle_min
        for distance in self.latest_scan.ranges:
            if (
                math.isfinite(distance)
                and min_range <= distance <= danger_distance
                and abs(angle) <= half_angle
            ):
                count += 1
                nearest = min(nearest, float(distance))
            angle += self.latest_scan.angle_increment

        if count < min_points:
            memory_s = float(self.get_parameter("direct_avoidance_memory_s").value)
            if now - self.last_avoidance_time <= memory_s:
                backup = self.last_avoidance_min_range <= float(
                    self.get_parameter("direct_backup_distance_m").value
                )
                return (
                    True,
                    backup,
                    self.last_avoidance_turn_sign,
                    self.last_avoidance_points,
                    self.last_avoidance_min_range,
                )
            return False, False, 1.0, count, nearest

        left_clear = self.direct_side_clearance(0.45, 1.75)
        right_clear = self.direct_side_clearance(-1.75, -0.45)
        turn_sign = 1.0 if left_clear >= right_clear else -1.0
        self.last_avoidance_time = now
        self.last_avoidance_turn_sign = turn_sign
        self.last_avoidance_min_range = nearest
        self.last_avoidance_points = count
        backup = nearest <= float(self.get_parameter("direct_backup_distance_m").value)
        return True, backup, turn_sign, count, nearest

    def direct_side_clearance(self, angle_min, angle_max):
        if self.latest_scan is None:
            return 0.0
        values = []
        angle = self.latest_scan.angle_min
        capped_max = min(float(self.latest_scan.range_max), 10.0)
        min_range = max(
            float(self.latest_scan.range_min),
            float(self.get_parameter("direct_avoidance_min_range_m").value),
        )
        for distance in self.latest_scan.ranges:
            if angle_min <= angle <= angle_max and math.isfinite(distance):
                values.append(max(min_range, min(float(distance), capped_max)))
            angle += self.latest_scan.angle_increment
        if not values:
            return capped_max
        values.sort(reverse=True)
        top = values[: max(1, len(values) // 3)]
        return sum(top) / len(top)

    def publish_direct_pair(self, left, right):
        self.direct_left_pub.publish(Float64(data=float(left)))
        self.direct_right_pub.publish(Float64(data=float(right)))

    def direct_stuck_state(self, distance, yaw_error):
        if not bool(self.get_parameter("direct_stuck_recovery_enabled").value):
            return False, False, 1.0
        now = time.monotonic()
        if self.active_index != self.direct_progress_index:
            self.direct_progress_index = self.active_index
            self.direct_best_distance = distance
            self.direct_last_progress_time = now
            self.direct_recovery_until = 0.0
            self.direct_recovery_backup_until = 0.0
            return False, False, 1.0

        if now < self.direct_recovery_until:
            return (
                True,
                now < self.direct_recovery_backup_until,
                self.direct_recovery_turn_sign,
            )

        min_progress = abs(
            float(self.get_parameter("direct_stuck_min_progress_m").value)
        )
        if distance < self.direct_best_distance - min_progress:
            self.direct_best_distance = distance
            self.direct_last_progress_time = now
            return False, False, 1.0

        timeout_s = abs(float(self.get_parameter("direct_stuck_timeout_s").value))
        active_radius = self.active_waypoint_radius()
        if distance <= active_radius * 1.05 or now - self.direct_last_progress_time < timeout_s:
            return False, False, 1.0

        target = self.local_waypoints[self.active_index]
        if bool(target.get("virtual", False)) and bool(
            self.get_parameter("direct_stuck_skip_virtual_waypoints").value
        ):
            max_skip_distance = abs(
                float(
                    self.get_parameter(
                        "direct_stuck_virtual_skip_max_distance_m"
                    ).value
                )
            )
            if distance <= max_skip_distance:
                skipped = self.active_index
                name = str(target.get("name", f"wp{skipped + 1}"))
                self.active_index += 1
                self.closest_distance_by_index.pop(skipped, None)
                self.direct_progress_index = -1
                self.direct_best_distance = float("inf")
                self.direct_last_progress_time = now
                self.direct_recovery_until = 0.0
                self.direct_recovery_backup_until = 0.0
                self.publish_status(
                    "ardupilot_waypoint_skip:"
                    f"{skipped}:virtual_stuck:{name}:dist={distance:.1f}m"
                )
                return False, False, 1.0

        left_clear = self.direct_side_clearance(0.45, 1.75)
        right_clear = self.direct_side_clearance(-1.75, -0.45)
        if left_clear == right_clear:
            turn_sign = 1.0 if yaw_error >= 0.0 else -1.0
        else:
            turn_sign = 1.0 if left_clear > right_clear else -1.0
        self.direct_recovery_turn_sign = turn_sign
        backup_duration = abs(
            float(self.get_parameter("direct_stuck_backup_duration_s").value)
        )
        turn_duration = abs(
            float(self.get_parameter("direct_stuck_turn_duration_s").value)
        )
        self.direct_recovery_backup_until = now + backup_duration
        self.direct_recovery_until = now + backup_duration + turn_duration
        self.direct_last_progress_time = now
        self.direct_best_distance = distance
        self.publish_status(
            "ardupilot_direct_recovery:stuck:"
            f"wp={self.active_index + 1}:dist={distance:.1f}m:"
            f"turn={'left' if turn_sign > 0.0 else 'right'}"
        )
        return True, True, turn_sign

    def monitor_progress(self):
        if not self.mission_uploaded or not self.local_waypoints:
            return
        if self.odom_xy is None:
            return

        if self.active_index >= len(self.local_waypoints):
            if not self.finish_reported:
                self.finish_reported = True
                self.publish_status("ardupilot_waypoints_succeeded")
                if bool(self.get_parameter("rtl_on_finish").value):
                    self.call_set_mode("RTL", wait=False)
            return

        now = time.monotonic()
        if now - self.last_status_time >= float(
            self.get_parameter("status_period_s").value
        ):
            target = self.local_waypoints[self.active_index]
            name = target["name"]
            distance = math.hypot(
                self.odom_xy[0] - target["x"],
                self.odom_xy[1] - target["y"],
            )
            previous_xy = self.start_xy or self.odom_xy
            if self.active_index > 0:
                previous = self.local_waypoints[self.active_index - 1]
                previous_xy = (previous["x"], previous["y"])
            progress = self.segment_progress(
                previous_xy, (target["x"], target["y"]), self.odom_xy
            )
            signed_progress = self.signed_segment_progress(
                previous_xy, (target["x"], target["y"]), self.odom_xy
            )
            progress_text = ""
            if progress is not None and signed_progress is not None:
                along, cross, seg_len = progress
                _, signed_cross, _ = signed_progress
                progress_text = (
                    f":along={along:.1f}/{seg_len:.1f}m"
                    f":cross={cross:.1f}m:signed_cross={signed_cross:.1f}m"
                )
            self.publish_status(
                f"ardupilot_running:{self.active_index + 1}:{name}:dist={distance:.1f}m"
                f"{progress_text}"
            )
            self.last_status_time = now

        if not bool(self.get_parameter("enable_passed_waypoint_skip").value):
            return
        skip_reason = self.current_waypoint_skip_reason()
        if skip_reason:
            skipped = self.active_index
            self.active_index += 1
            self.closest_distance_by_index.pop(skipped, None)
            if self.active_index < len(self.local_waypoints) and self.control_mode != "guided":
                self.call_set_current(self.active_index, wait=False)
            self.publish_status(f"ardupilot_waypoint_skip:{skipped}:{skip_reason}")

    def current_waypoint_skip_reason(self):
        if self.active_index >= len(self.local_waypoints):
            return ""
        target = self.local_waypoints[self.active_index]
        target_xy = (target["x"], target["y"])
        distance = math.hypot(
            self.odom_xy[0] - target_xy[0],
            self.odom_xy[1] - target_xy[1],
        )
        closest = self.closest_distance_by_index.get(self.active_index, float("inf"))
        if distance < closest:
            self.closest_distance_by_index[self.active_index] = distance
            closest = distance

        radius_m = self.active_waypoint_radius()
        if distance <= radius_m:
            return f"radius:{distance:.2f}m"

        if bool(self.get_parameter("waypoint_recede_skip_enabled").value):
            recede_radius_m = float(
                self.get_parameter("waypoint_recede_skip_radius_m").value
            )
            recede_margin_m = float(
                self.get_parameter("waypoint_recede_skip_margin_m").value
            )
            if closest <= recede_radius_m and distance >= closest + recede_margin_m:
                return f"receding:closest={closest:.2f}m:now={distance:.2f}m"

        if self.active_index == 0:
            previous_xy = self.start_xy or self.odom_xy
        else:
            previous = self.local_waypoints[self.active_index - 1]
            previous_xy = (previous["x"], previous["y"])

        progress = self.segment_progress(previous_xy, target_xy, self.odom_xy)
        if progress is not None and self.active_index == len(self.local_waypoints) - 1:
            along, cross, seg_len = progress
            final_fraction = float(
                self.get_parameter("final_waypoint_progress_fraction").value
            )
            max_cross = float(self.get_parameter("waypoint_passed_cross_track_m").value)
            if along >= seg_len * final_fraction and cross <= max_cross:
                return f"final_progress:{along / seg_len:.2f}:cross={cross:.2f}m"

        if self.segment_passed(
            previous_xy,
            target_xy,
            self.odom_xy,
            float(self.get_parameter("waypoint_passed_margin_m").value),
            float(self.get_parameter("waypoint_passed_cross_track_m").value),
        ):
            return "segment_passed"

        return ""

    def active_waypoint_radius(self):
        if self.active_index >= len(self.local_waypoints):
            return float(self.get_parameter("waypoint_radius_m").value)
        waypoint = self.local_waypoints[self.active_index]
        name = str(waypoint.get("name", "")).lower()
        if "gn4" in name:
            return float(self.get_parameter("course2_entry_waypoint_radius_m").value)
        if bool(waypoint.get("virtual", False)):
            return float(self.get_parameter("virtual_waypoint_radius_m").value)
        if self.active_index == len(self.local_waypoints) - 1:
            return float(self.get_parameter("final_waypoint_radius_m").value)
        return float(self.get_parameter("waypoint_radius_m").value)

    @staticmethod
    def segment_passed(start_xy, target_xy, current_xy, margin_m, max_cross_track_m):
        progress = ArduPilotWaypointMission.segment_progress(
            start_xy, target_xy, current_xy
        )
        if progress is None:
            return False
        along, cross, seg_len = progress
        return along >= seg_len + margin_m and cross <= max_cross_track_m

    @staticmethod
    def segment_progress(start_xy, target_xy, current_xy):
        sx, sy = start_xy
        tx, ty = target_xy
        cx, cy = current_xy
        vx = tx - sx
        vy = ty - sy
        wx = cx - sx
        wy = cy - sy
        seg_len = math.hypot(vx, vy)
        if seg_len < 1.0e-6:
            return None
        along = (wx * vx + wy * vy) / seg_len
        cross = abs(wx * vy - wy * vx) / seg_len
        return along, cross, seg_len

    @staticmethod
    def signed_segment_progress(start_xy, target_xy, current_xy):
        sx, sy = start_xy
        tx, ty = target_xy
        cx, cy = current_xy
        vx = tx - sx
        vy = ty - sy
        wx = cx - sx
        wy = cy - sy
        seg_len = math.hypot(vx, vy)
        if seg_len < 1.0e-6:
            return None
        along = (wx * vx + wy * vy) / seg_len
        signed_cross = (wx * vy - wy * vx) / seg_len
        return along, signed_cross, seg_len

    @staticmethod
    def quaternion_to_yaw(orientation):
        x = float(orientation.x)
        y = float(orientation.y)
        z = float(orientation.z)
        w = float(orientation.w)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def angle_delta(target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))


def main(args=None):
    rclpy.init(args=args)
    node = ArduPilotWaypointMission()
    try:
        node.start()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.guided_mavlink is not None:
            node.guided_mavlink.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
