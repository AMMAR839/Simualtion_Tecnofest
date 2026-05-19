from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
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
    world = LaunchConfiguration("world")
    waypoint_file = LaunchConfiguration("waypoint_file")
    target_color = LaunchConfiguration("target_color")

    pkg_share = FindPackageShare("gamantaray_boat_sim")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")
    nav2_bringup_share = FindPackageShare("nav2_bringup")

    default_world = PathJoinSubstitution(
        [pkg_share, "worlds", "tecnofest_asv_course.sdf"]
    )
    default_waypoints = PathJoinSubstitution(
        [pkg_share, "config", "tecnofest_waypoints.yaml"]
    )
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2_params.yaml"])

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

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_nav2", default_value="true"),
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [ros_gz_sim_share, "launch", "gz_sim.launch.py"]
                    )
                ),
                launch_arguments={"gz_args": ["-r -v 3 ", world]}.items(),
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
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="gamantaray_boat_sim",
                executable="cmd_vel_to_thrusters",
                name="cmd_vel_to_thrusters",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
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
                            }
                        ],
                    )
                ],
            ),
        ]
    )
