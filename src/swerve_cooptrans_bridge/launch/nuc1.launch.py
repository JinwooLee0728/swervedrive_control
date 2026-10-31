"""NUC 1 -- the coordinating machine. Run this on robot 1's onboard computer.

There is no central controlling computer. ROS 2 is peer-to-peer with no master,
so "the machine that runs the controller" is just a deployment choice: NUC1
runs the shared coordination stack in the ROOT namespace plus robot 1's own
hardware under /robot1. NUC2 runs only robot 2's hardware under /robot2 and
subscribes to the commands over the network.

    NUC1                                   NUC2
      lidar_pose_adapter -> /payload/pose    (subscribes to it)
      trajectory_generator -> ref_*
      controller -> /module1..4/{cmd_vel, desired_heading}
      /robot1/* hardware                     /robot2/* hardware

Both machines MUST share ROS_DOMAIN_ID and must not be discovery-restricted to
localhost:

    export ROS_DOMAIN_ID=17
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

No clock sync is required for safety: every watchdog compares against a receipt
time stamped on its own machine. If NUC1 dies, robot 2's bridge times out after
cmd_timeout and stops its wheels while holding steering.

Usage:
    ros2 launch swerve_cooptrans_bridge nuc1.launch.py
    ros2 launch swerve_cooptrans_bridge nuc1.launch.py use_progression_regulator:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PKG = 'swerve_cooptrans_bridge'


def generate_launch_description():
    cfg = LaunchConfiguration('config')
    use_reg = LaunchConfiguration('use_progression_regulator')
    skip_check = LaunchConfiguration('skip_hardware_check')

    share = get_package_share_directory(PKG)
    default_cfg = os.path.join(share, 'config', 'hardware.yaml')

    args = [
        DeclareLaunchArgument(
            'config', default_value=default_cfg,
            description='The single hardware configuration file. One file for '
                        'the whole system; both NUCs load the same one.'),
        DeclareLaunchArgument(
            'use_progression_regulator', default_value='true',
            description="true  -> trajectory_generator ingests the external "
                        "planner's nav_msgs/Path horizon and applies the "
                        "paper's progression-rate regulator (eq. 13). "
                        "false -> the controller subscribes to externally "
                        "published ref_pose/twist/accel directly; eq. 13 is "
                        "then unavailable (re-timing needs a parameterized "
                        "path, not a setpoint)."),
        DeclareLaunchArgument(
            'skip_hardware_check', default_value='false',
            description='Bypass the config validator. Do not use on hardware.'),
    ]

    # Runs FIRST and is not an rclpy node: it re-derives every cross-block
    # constraint in hardware.yaml (module geometry vs mount+offset, w_max vs
    # Dynamixel profile_velocity, gear-ratio agreement, sim mirror) and exits
    # non-zero on any mismatch, which aborts the launch.
    hardware_check = ExecuteProcess(
        cmd=['ros2', 'run', PKG, 'hardware_check', cfg],
        output='screen',
        condition=UnlessCondition(skip_check),
    )

    # External LiDAR localization -> the <payload>/pose the controller expects.
    lidar_adapter = Node(
        package=PKG, executable='lidar_pose_adapter',
        name='lidar_pose_adapter', parameters=[cfg], output='screen',
    )

    # External receding-horizon planner -> ref_pose/twist/accel, with eq. 13
    # re-timing. Skipped when the planner publishes setpoints itself.
    trajectory_generator = Node(
        package='trajectory_generation', executable='trajectory_generator',
        name='trajectory_generator', parameters=[cfg], output='screen',
        condition=IfCondition(use_reg),
    )

    controller = Node(
        package='controller', executable='controller_node',
        name='controller', parameters=[cfg], output='screen',
    )

    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(PKG), 'launch', 'robot.launch.py'])),
        launch_arguments={
            'robot_ns': 'robot1',
            'config': cfg,
        }.items(),
    )

    return LaunchDescription(args + [
        hardware_check,
        lidar_adapter,
        trajectory_generator,
        controller,
        robot1,
    ])
