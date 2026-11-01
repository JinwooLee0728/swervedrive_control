"""Keyboard teleop of the simulated payload, in one command.

    ros2 launch keyboard_to_swerve sim_teleop.launch.py

Starts the same simulation as swerve_cooptrans_bridge/sim.launch.py MINUS the
planner, trajectory generator and controller (they publish the same
/moduleK/{cmd_vel,desired_heading} topics as the teleop), and adds
payload_keyboard:

    payload_keyboard  --/moduleK/cmd_vel, desired_heading-->  kinematic_plant
          |                         |                          (servo mode)
          |               swerve_cooptrans_bridge x2                |
          |               (/robot1, /robot2: what HW gets)          |
          +<--------------- /moduleK/pose ---------------------------+
                                                                    |
                          lidar_sim -> lidar_pose_adapter -> /payload/pose
                                                                    |
                                                         swerve_viz (GUI)

The teleop reads the keyboard, and `ros2 launch` does not forward stdin to
its children, so payload_keyboard opens in its OWN terminal window: click it
before typing. Closing that window (or CTRL-C in it) shuts the whole
simulation down. Use `terminal:='xterm -e'` if gnome-terminal is unavailable.

Arguments:
    config    sim config (default: swerve_cooptrans_bridge/config/sim.yaml)
    gui       start swerve_viz              (default: true)
    terminal  command prefix for the teleop window
              (default: 'gnome-terminal --wait --'; --wait keeps the launch
              attached to the window so closing it ends the launch)
    skip_hardware_check                     (default: false)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            GroupAction, RegisterEventHandler)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

SIM = 'swerve_cooptrans_bridge'


def generate_launch_description():
    cfg = LaunchConfiguration('config')
    gui = LaunchConfiguration('gui')
    terminal = LaunchConfiguration('terminal')
    skip_check = LaunchConfiguration('skip_hardware_check')

    default_cfg = os.path.join(get_package_share_directory(SIM),
                               'config', 'sim.yaml')

    args = [
        DeclareLaunchArgument('config', default_value=default_cfg),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'terminal', default_value='gnome-terminal --wait --',
            description='Prefix that opens the interactive teleop window.'),
        DeclareLaunchArgument('skip_hardware_check', default_value='false'),
    ]

    validator = ExecuteProcess(
        cmd=['ros2', 'run', SIM, 'hardware_check', cfg],
        output='screen', condition=UnlessCondition(skip_check))

    # The plant publishes the TRUE pose on a separate topic so the simulated
    # LiDAR can degrade it; <payload>/pose is owned by the adapter, as on HW.
    plant = Node(
        package='cooptrans_sim', executable='kinematic_plant',
        name='kinematic_plant', parameters=[cfg], output='screen',
        remappings=[('/payload/pose', '/payload/truth_pose'),
                    ('/payload/twist', '/payload/truth_twist')])

    lidar_sim = Node(
        package=SIM, executable='lidar_sim', name='lidar_sim',
        parameters=[cfg], output='screen')

    lidar_adapter = Node(
        package=SIM, executable='lidar_pose_adapter',
        name='lidar_pose_adapter', parameters=[cfg], output='screen')

    bridges = [
        GroupAction([
            PushRosNamespace(ns),
            Node(package=SIM, executable='module_bridge',
                 name='swerve_cooptrans_bridge', parameters=[cfg],
                 output='screen'),
        ]) for ns in ('robot1', 'robot2')
    ]

    viz = Node(
        package=SIM, executable='swerve_viz', name='swerve_viz',
        parameters=[cfg], output='screen', condition=IfCondition(gui))

    # use_external_pose: lidar_pose_adapter already owns /payload/pose.
    teleop = Node(
        package='keyboard_to_swerve', executable='payload_keyboard',
        name='payload_keyboard', prefix=terminal, output='screen',
        parameters=[{'config': cfg, 'use_external_pose': True}])

    shutdown_with_teleop = RegisterEventHandler(OnProcessExit(
        target_action=teleop,
        on_exit=[EmitEvent(event=Shutdown(reason='teleop window closed'))]))

    return LaunchDescription(args + [
        validator,
        plant,
        lidar_sim,
        lidar_adapter,
        *bridges,
        viz,
        teleop,
        shutdown_with_teleop,
    ])
