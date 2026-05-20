from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def nav2_node(
    package,
    executable,
    name,
    params_file,
    log_level,
    remappings=None,
    extra_parameters=None,
):
    node_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    if remappings:
        node_remappings.extend(remappings)
    parameters = [params_file]
    if extra_parameters:
        parameters.extend(extra_parameters)
    return Node(
        package=package,
        executable=executable,
        name=name,
        output="screen",
        parameters=parameters,
        arguments=["--ros-args", "--log-level", log_level],
        remappings=node_remappings,
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    asv_bt_xml = PathJoinSubstitution(
        [
            FindPackageShare("gamantaray_nav2_bt_plugins"),
            "behavior_trees",
            "asv_navigate_to_pose.xml",
        ]
    )

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
        "waypoint_follower",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("params_file"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            SetParameter("use_sim_time", use_sim_time),
            nav2_node(
                "nav2_controller",
                "controller_server",
                "controller_server",
                params_file,
                log_level,
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            nav2_node(
                "nav2_smoother",
                "smoother_server",
                "smoother_server",
                params_file,
                log_level,
            ),
            nav2_node(
                "nav2_planner",
                "planner_server",
                "planner_server",
                params_file,
                log_level,
            ),
            nav2_node(
                "nav2_behaviors",
                "behavior_server",
                "behavior_server",
                params_file,
                log_level,
                remappings=[("cmd_vel", "cmd_vel_nav")],
            ),
            nav2_node(
                "nav2_velocity_smoother",
                "velocity_smoother",
                "velocity_smoother",
                params_file,
                log_level,
                remappings=[
                    ("cmd_vel", "cmd_vel_nav"),
                    ("cmd_vel_smoothed", "cmd_vel_nav2_smoothed"),
                ],
            ),
            nav2_node(
                "nav2_collision_monitor",
                "collision_monitor",
                "collision_monitor",
                params_file,
                log_level,
            ),
            nav2_node(
                "nav2_bt_navigator",
                "bt_navigator",
                "bt_navigator",
                params_file,
                log_level,
                extra_parameters=[
                    {
                        "default_nav_to_pose_bt_xml": asv_bt_xml,
                    }
                ],
            ),
            nav2_node(
                "nav2_waypoint_follower",
                "waypoint_follower",
                "waypoint_follower",
                params_file,
                log_level,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ]
    )
