"""NUC 2 -- robot 2's onboard computer. Hardware only.

The coordination stack (localization, reference, controller) runs on NUC1; this
machine only drives robot 2's modules and reports their state back. It consumes
/module3,4/{cmd_vel, desired_heading} and /payload/pose over the network, all
in the root namespace, and publishes /module3,4/pose.

Both machines MUST share ROS_DOMAIN_ID:

    export ROS_DOMAIN_ID=17
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

If NUC1 or the network dies, the bridge's own watchdog (cmd_timeout) stops the
wheels and holds steering -- it does not depend on hearing anything from NUC1
to fail safe.

Usage:
    ros2 launch swerve_cooptrans_bridge nuc2.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

PKG = 'swerve_cooptrans_bridge'


def generate_launch_description():
    cfg = LaunchConfiguration('config')
    share = get_package_share_directory(PKG)
    default_cfg = os.path.join(share, 'config', 'hardware.yaml')

    args = [
        DeclareLaunchArgument(
            'config', default_value=default_cfg,
            description='The single hardware configuration file. Must be the '
                        'SAME content as the one on NUC1 -- the geometry it '
                        'declares has to agree or the controller is solving '
                        'for a robot that does not exist.'),
    ]

    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(PKG), 'launch', 'robot.launch.py'])),
        launch_arguments={
            'robot_ns': 'robot2',
            'config': cfg,
        }.items(),
    )

    return LaunchDescription(args + [robot2])
