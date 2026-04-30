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
    files = []
    for path, _, filenames in os.walk(directory):
        for filename in filenames:
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
        *[
            (os.path.join('share', package_name, os.path.dirname(path)), [path])
            for path in package_files('models')
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
        ],
    },
)
