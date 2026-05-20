from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


BRIDGE_ARGUMENTS = [
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    "/asv/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
    "/asv/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
    "/asv/lidar/scan_raw@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    "/asv/camera/front/image@sensor_msgs/msg/Image[gz.msgs.Image",
    "/asv/camera/front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    "/asv/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    "/world/tecnofest_asv_course/set_pose@ros_gz_interfaces/srv/SetEntityPose",
    "/model/gamantaray_boat/joint/left_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double",
    "/model/gamantaray_boat/joint/right_propeller_joint/cmd_thrust@std_msgs/msg/Float64]gz.msgs.Double",
]


def joined_path(parts):
    value = []
    for index, part in enumerate(parts):
        if index > 0:
            value.append(":")
        value.append(part)
    return value


def static_tf_args(x, y, z, roll, pitch, yaw, parent, child):
    return [
        "--x", x,
        "--y", y,
        "--z", z,
        "--roll", roll,
        "--pitch", pitch,
        "--yaw", yaw,
        "--frame-id", parent,
        "--child-frame-id", child,
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_rviz = LaunchConfiguration("use_rviz")
    navigation_mode = LaunchConfiguration("navigation_mode")
    world = LaunchConfiguration("world")
    waypoint_file = LaunchConfiguration("waypoint_file")
    target_color = LaunchConfiguration("target_color")

    pkg_share = FindPackageShare("gamantaray_boat_sim")

    default_world = PathJoinSubstitution(
        [pkg_share, "worlds", "tecnofest_asv_course.sdf"]
    )
    default_waypoints = PathJoinSubstitution(
        [pkg_share, "config", "tecnofest_waypoints.yaml"]
    )
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"])
    ardupilot_params = PathJoinSubstitution([pkg_share, "config", "ardupilot_asv.parm"])
    rviz_config = PathJoinSubstitution([pkg_share, "config", "tecnofest_nav2.rviz"])
    gazebo_gui_config = PathJoinSubstitution(
        [pkg_share, "config", "tecnofest_gazebo_gui.config"]
    )

    resource_paths = [
        PathJoinSubstitution([pkg_share]),
        PathJoinSubstitution([pkg_share, "models"]),
        EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
    ]
    plugin_paths = [
        PathJoinSubstitution([pkg_share, "plugins"]),
        "/home/ammar/ardupilot_gazebo/build",
        "/usr/lib/x86_64-linux-gnu",
        "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins",
        EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
    ]
    nav2_condition = IfCondition(
        PythonExpression(["'", navigation_mode, "' == 'nav2' and '", use_nav2, "' == 'true'"])
    )
    ardupilot_condition = IfCondition(
        PythonExpression(["'", navigation_mode, "' == 'ardupilot'"])
    )
    non_ardupilot_condition = IfCondition(
        PythonExpression(["'", navigation_mode, "' != 'ardupilot'"])
    )

    gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            "-v",
            "3",
            "--gui-config",
            gazebo_gui_config,
            world,
        ],
        output="screen",
        name="gazebo",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_nav2", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("navigation_mode", default_value="nav2"),
            DeclareLaunchArgument("target_color", default_value="green"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("waypoint_file", default_value=default_waypoints),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=joined_path(resource_paths),
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_SYSTEM_PLUGIN_PATH",
                value=joined_path(plugin_paths),
            ),
            SetEnvironmentVariable(
                name="GZ_RENDERING_PLUGIN_PATH",
                value=joined_path(plugin_paths),
            ),
            SetEnvironmentVariable(
                name="LD_LIBRARY_PATH",
                value=joined_path(
                    plugin_paths + [EnvironmentVariable("LD_LIBRARY_PATH", default_value="")]
                ),
            ),
            gazebo,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gazebo,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="Gazebo exited; shutting down ROS/Nav2 stack"
                            )
                        )
                    ],
                )
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="asv_gz_bridge",
                output="screen",
                arguments=BRIDGE_ARGUMENTS,
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static_tf",
                arguments=static_tf_args("0", "0", "0", "0", "0", "0", "map", "odom"),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_lidar_static_tf",
                arguments=static_tf_args(
                    "0.55", "0", "0.95", "0", "0", "0", "base_link", "lidar_link"
                ),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_gazebo_lidar_static_tf",
                arguments=static_tf_args(
                    "0.55",
                    "0",
                    "0.95",
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "gamantaray_boat/base_link/lidar_sensor",
                ),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_front_camera_static_tf",
                arguments=static_tf_args(
                    "0.85",
                    "0",
                    "1.25",
                    "0",
                    "0.10",
                    "0",
                    "base_link",
                    "front_camera_link",
                ),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_gazebo_front_camera_static_tf",
                arguments=static_tf_args(
                    "0.85",
                    "0",
                    "1.25",
                    "0",
                    "0.10",
                    "0",
                    "base_link",
                    "gamantaray_boat/base_link/front_camera_sensor",
                ),
            ),
            Node(
                package="gamantaray_boat_sim",
                executable="odom_tf_broadcaster",
                name="odom_tf_broadcaster",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "odom_topic": "/asv/odom",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                    }
                ],
            ),
            Node(
                package="gamantaray_boat_sim",
                executable="cmd_vel_to_thrusters",
                name="cmd_vel_to_thrusters",
                output="screen",
                condition=non_ardupilot_condition,
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "max_forward_thrust_n": 24.0,
                        "max_reverse_thrust_n": 12.0,
                        "max_speed_cmd_mps": 0.42,
                        "yaw_to_thrust_n_per_radps": 58.0,
                        "yaw_sign": 1.0,
                        "yaw_rate_feedback_gain": 0.25,
                        "max_yaw_rate_cmd_radps": 1.15,
                        "cmd_timeout_s": 0.8,
                        "thrust_slew_rate_nps": 28.0,
                        "turn_throttle_reduction": 0.55,
                        "min_turn_throttle_fraction": 0.25,
                        "speed_feedback_gain_n_per_mps": 0.0,
                        "speed_feedforward_fraction": 0.85,
                        "odom_timeout_s": 0.7,
                    }
                ],
            ),
            Node(
                package="gamantaray_boat_sim",
                executable="lidar_scan_filter",
                name="lidar_scan_filter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "raw_topic": "/asv/lidar/scan_raw",
                        "filtered_topic": "/asv/lidar/scan",
                        "min_valid_range": 0.75,
                        "fallback_frame_id": "lidar_link",
                        "force_frame_id": True,
                        "restamp_scan": True,
                        "stamp_future_offset_s": 0.20,
                        "sensor_x": 0.55,
                        "sensor_y": 0.0,
                        "keep_angle_min": -2.05,
                        "keep_angle_max": 2.05,
                        "cluster_gap_m": 0.30,
                        "min_cluster_points": 4,
                        "min_cluster_width_m": 0.20,
                        "max_cluster_width_m": 0.95,
                        "max_cluster_points": 55,
                    }
                ],
            ),
            Node(
                package="gamantaray_boat_sim",
                executable="anti_sink_guard",
                name="anti_sink_guard",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "odom_topic": "/asv/odom",
                        "set_pose_service": "/world/tecnofest_asv_course/set_pose",
                        "entity_name": "gamantaray_boat",
                        "min_z": -0.35,
                        "restore_z": 0.30,
                        "max_abs_x": 70.0,
                        "max_abs_y": 30.0,
                        "safe_x": -49.0,
                        "safe_y": -8.0,
                        "safe_yaw": 0.42,
                        "max_abs_roll_pitch_rad": 1.20,
                    }
                ],
            ),
            Node(
                package="gamantaray_boat_sim",
                executable="target_buoy_detector",
                name="target_buoy_detector",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "target_color": target_color,
                    }
                ],
            ),
            Node(
                package="gamantaray_boat_sim",
                executable="rviz_boat_marker",
                name="rviz_boat_marker",
                output="screen",
                condition=IfCondition(use_rviz),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "use_heavy_mesh": False,
                        "odom_topic": "/asv/odom",
                        "fixed_frame": "odom",
                        "publish_in_fixed_frame": False,
                    }
                ],
            ),
            Node(
                package="gamantaray_boat_sim",
                executable="lidar_ray_marker",
                name="lidar_ray_marker",
                output="screen",
                condition=IfCondition(use_rviz),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "scan_topic": "/asv/lidar/scan",
                        "marker_topic": "/asv/visualization/lidar_rays",
                        "marker_frame": "lidar_link",
                        "max_ray_range": 8.0,
                        "sample_step": 32,
                        "publish_period_s": 0.35,
                    }
                ],
            ),
            Node(
                package="gamantaray_boat_sim",
                executable="lidar_obstacle_marker",
                name="lidar_obstacle_marker",
                output="screen",
                condition=IfCondition(use_rviz),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "scan_topic": "/asv/lidar/scan",
                        "marker_topic": "/asv/perception/lidar_obstacles",
                        "marker_frame": "lidar_link",
                        "max_range": 12.0,
                        "cluster_gap_m": 0.40,
                        "min_cluster_points": 4,
                        "publish_period_s": 0.35,
                    }
                ],
            ),
            TimerAction(
                period=12.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [pkg_share, "launch", "nav2_asv.launch.py"]
                            )
                        ),
                        condition=nav2_condition,
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "params_file": nav2_params,
                            "autostart": "true",
                        }.items(),
                    )
                ],
            ),
            TimerAction(
                period=14.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="tecnofest_nav2_rviz",
                        output="screen",
                        condition=IfCondition(use_rviz),
                        arguments=["-d", rviz_config],
                        parameters=[{"use_sim_time": use_sim_time}],
                    )
                ],
            ),
            TimerAction(
                period=24.0,
                actions=[
                    Node(
                        package="gamantaray_boat_sim",
                        executable="nav2_waypoint_mission",
                        output="screen",
                        condition=nav2_condition,
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "waypoint_file": waypoint_file,
                                "target_color": target_color,
                                "start_delay_s": 2.0,
                                "use_through_poses": False,
                                "prereq_timeout_s": 90.0,
                                "waypoint_acceptance_radius_m": 3.40,
                                "waypoint_check_period_s": 0.20,
                                "status_period_s": 1.0,
                                "max_goal_retries": 3,
                            }
                        ],
                    )
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "run",
                            "gamantaray_boat_sim",
                            "ardupilot_sitl_terminal",
                            "--param-file",
                            ardupilot_params,
                        ],
                        output="screen",
                        name="ardurover_sitl_terminal",
                        condition=ardupilot_condition,
                    )
                ],
            ),
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="mavros",
                        executable="mavros_node",
                        name="mavros",
                        output="screen",
                        condition=ardupilot_condition,
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "fcu_url": "udp://:14550@127.0.0.1:14555",
                                "gcs_url": "",
                                "target_system_id": 1,
                                "target_component_id": 1,
                            }
                        ],
                    ),
                    Node(
                        package="gamantaray_boat_sim",
                        executable="ardupilot_lidar_bridge",
                        name="ardupilot_lidar_bridge",
                        output="screen",
                        condition=ardupilot_condition,
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "scan_topic": "/asv/lidar/scan",
                                "output_topic": "/mavros/obstacle/send",
                                "max_range": 18.0,
                            }
                        ],
                    ),
                ],
            ),
        ]
    )
