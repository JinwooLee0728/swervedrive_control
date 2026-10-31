import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'swerve_cooptrans_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jinwoo Lee',
    maintainer_email='jinwoolee0728@snu.ac.kr',
    description='Adapter between the cooptrans controller and swerve hardware.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'module_bridge = swerve_cooptrans_bridge.module_bridge:main',
            'lidar_pose_adapter = swerve_cooptrans_bridge.lidar_pose_adapter:main',
            'hardware_check = swerve_cooptrans_bridge.hardware_check:main',
            'sim_planner = swerve_cooptrans_bridge.sim_planner:main',
            'lidar_sim = swerve_cooptrans_bridge.lidar_sim:main',
            'swerve_viz = swerve_cooptrans_bridge.swerve_viz:main',
        ],
    },
)
