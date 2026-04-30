from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('gamantaray_boat_sim')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    world_path = os.path.join(pkg_share, 'worlds', 'simple_ocean.sdf')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            pkg_share,
            os.path.join(pkg_share, 'models'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ]),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/gamantaray_boat/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/world/simple_ocean/model/gamantaray_boat/link/base_link/sensor/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/simple_ocean/model/gamantaray_boat/link/base_link/sensor/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/world/simple_ocean/model/gamantaray_boat/link/base_link/sensor/front_lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    cmd_vel_to_thrusters = Node(
        package='gamantaray_boat_sim',
        executable='cmd_vel_to_thrusters',
        parameters=[{
            'max_linear_speed': 1.2,
            'max_yaw_rate': 0.8,
            'command_timeout': 0.5,
        }],
        output='screen',
    )

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        bridge,
        cmd_vel_to_thrusters,
    ])
