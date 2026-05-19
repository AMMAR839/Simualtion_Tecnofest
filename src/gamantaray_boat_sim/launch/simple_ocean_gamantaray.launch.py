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
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


BRIDGE_ARGUMENTS = [
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    "/asv/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
    "/asv/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
    "/asv/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    "/asv/camera/front/image@sensor_msgs/msg/Image[gz.msgs.Image",
    "/asv/camera/front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    "/asv/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
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
    world = LaunchConfiguration("world")
    waypoint_file = LaunchConfiguration("waypoint_file")
    target_color = LaunchConfiguration("target_color")

    pkg_share = FindPackageShare("gamantaray_boat_sim")
    nav2_bringup_share = FindPackageShare("nav2_bringup")

    default_world = PathJoinSubstitution(
        [pkg_share, "worlds", "tecnofest_asv_course.sdf"]
    )
    default_waypoints = PathJoinSubstitution(
        [pkg_share, "config", "tecnofest_waypoints.yaml"]
    )
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"])
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
        "/usr/lib/x86_64-linux-gnu",
        "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins",
        EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
    ]

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
                    "0.35", "0", "1.42", "0", "0", "0", "base_link", "lidar_link"
                ),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_gazebo_lidar_static_tf",
                arguments=static_tf_args(
                    "0.35",
                    "0",
                    "1.42",
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
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "max_forward_thrust_n": 18.0,
                        "max_reverse_thrust_n": 8.0,
                        "max_speed_cmd_mps": 0.60,
                        "yaw_to_thrust_n_per_radps": 12.0,
                        "yaw_sign": 1.0,
                        "max_yaw_rate_cmd_radps": 0.70,
                        "cmd_timeout_s": 0.8,
                        "thrust_slew_rate_nps": 28.0,
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
                        "publish_in_fixed_frame": True,
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
                        "max_range": 18.0,
                    }
                ],
            ),
            TimerAction(
                period=12.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [nav2_bringup_share, "launch", "navigation_launch.py"]
                            )
                        ),
                        condition=IfCondition(use_nav2),
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
                        condition=IfCondition(use_nav2),
                        parameters=[
                            {
                                "use_sim_time": use_sim_time,
                                "waypoint_file": waypoint_file,
                                "target_color": target_color,
                                "start_delay_s": 2.0,
                                "use_through_poses": False,
                                "prereq_timeout_s": 90.0,
                                "waypoint_acceptance_radius_m": 1.35,
                            }
                        ],
                    )
                ],
            ),
        ]
    )
