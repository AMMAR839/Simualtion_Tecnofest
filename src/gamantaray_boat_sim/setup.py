import os
import gzip
import shutil
from glob import glob

from setuptools import find_packages, setup

package_name = 'gamantaray_boat_sim'


def ensure_compressed_meshes():
    mesh_pairs = [
        (
            os.path.join('models', 'gamantaray_boat', 'meshes', 'assembly_2_0.obj.gz'),
            os.path.join('models', 'gamantaray_boat', 'meshes', 'assembly_2_0.obj'),
        ),
    ]

    for compressed_path, output_path in mesh_pairs:
        if os.path.exists(output_path) or not os.path.exists(compressed_path):
            continue

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with gzip.open(compressed_path, 'rb') as source:
            with open(output_path, 'wb') as target:
                shutil.copyfileobj(source, target)


def package_files(directory):
    if not os.path.isdir(directory):
        return []
    files = []
    for path, _, filenames in os.walk(directory):
        for filename in filenames:
            if filename.endswith(('.pyc', '.pyo')):
                continue
            files.append(os.path.join(path, filename))
    return files


ensure_compressed_meshes()

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), package_files('config')),
        *[
            (os.path.join('share', package_name, os.path.dirname(path)), [path])
            for path in package_files('models')
        ],
        *[
            (os.path.join('share', package_name, os.path.dirname(path)), [path])
            for path in package_files('plugins')
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ammar',
    maintainer_email='ammar@todo.todo',
    description='Simple Gazebo ocean simulation for the Gamantaray OBJ boat.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_thrusters = gamantaray_boat_sim.cmd_vel_to_thrusters:main',
            'odom_tf_broadcaster = gamantaray_boat_sim.odom_tf_broadcaster:main',
            'target_buoy_detector = gamantaray_boat_sim.target_buoy_detector:main',
            'nav2_waypoint_mission = gamantaray_boat_sim.nav2_waypoint_mission:main',
            'rviz_boat_marker = gamantaray_boat_sim.rviz_boat_marker:main',
            'lidar_obstacle_marker = gamantaray_boat_sim.lidar_obstacle_marker:main',
            'lidar_scan_filter = gamantaray_boat_sim.lidar_scan_filter:main',
            'lidar_scan_to_pointcloud = gamantaray_boat_sim.lidar_scan_to_pointcloud:main',
            'lidar_ray_marker = gamantaray_boat_sim.lidar_ray_marker:main',
            'anti_sink_guard = gamantaray_boat_sim.anti_sink_guard:main',
            'ardupilot_lidar_bridge = gamantaray_boat_sim.ardupilot_lidar_bridge:main',
            'ardupilot_sitl_terminal = gamantaray_boat_sim.ardupilot_sitl_terminal:main',
        ],
    },
)
