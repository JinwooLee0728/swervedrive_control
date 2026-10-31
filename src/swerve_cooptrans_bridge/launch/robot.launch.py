"""One swerve robot's hardware stack, pushed into a namespace.

Included by nuc1.launch.py (robot_ns:=robot1) and nuc2.launch.py
(robot_ns:=robot2). Can also be run directly for single-robot bench work.

Everything below is pushed into /<robot_ns>, which is the ONLY thing separating
robots on a shared DDS domain. That works because every topic name inside
swerve_test, swerve_odrive_bridge and odrive_can is relative. The deliberate
exception is the bridge's controller-facing topics (/<agent>/cmd_vel,
/<agent>/desired_heading, /<agent>/pose), which are absolute because the
controller runs in the root namespace -- possibly on the other machine.

All parameters come from the single hardware.yaml, whose blocks are keyed by
fully-qualified node name (/robot1/swerve_steering, ...), so one file
configures both robots and the node only picks up its own section.

Bench teleop without the controller:
    ros2 launch swerve_cooptrans_bridge robot.launch.py \
        robot_ns:=robot1 use_bridge:=false
    ros2 run keyboard_to_swerve swerve_keyboard --ros-args -r __ns:=/robot1 \
        --params-file <share>/keyboard_to_swerve/config/keyboard_to_swerve.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

PKG = 'swerve_cooptrans_bridge'


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_ns')
    cfg = LaunchConfiguration('config')
    use_bridge = LaunchConfiguration('use_bridge')

    share = get_package_share_directory(PKG)
    default_cfg = os.path.join(share, 'config', 'hardware.yaml')

    args = [
        DeclareLaunchArgument(
            'robot_ns', default_value='robot1',
            description='Namespace for this robot. Must be unique per robot, '
                        'and must match the block names in the config file.'),
        DeclareLaunchArgument(
            'config', default_value=default_cfg,
            description='The single hardware configuration file.'),
        DeclareLaunchArgument(
            'use_bridge', default_value='true',
            description='Start swerve_cooptrans_bridge. Set false for '
                        'keyboard-only bench testing -- the keyboard node and '
                        'the bridge both publish <module>/cmd_vel, so running '
                        'both would have two publishers fighting.'),
    ]

    # Two ODrive CAN nodes, one per drive motor. Their namespaces nest under
    # the robot namespace -> /robot1/odrive_axis0/..., which is what the
    # bridge's relative odrive1_namespace/odrive2_namespace resolve to.
    # node_id and interface come from the config file like everything else.
    odrive_axis0 = Node(
        package='odrive_can', executable='odrive_can_node',
        name='odrive_can_node', namespace='odrive_axis0',
        parameters=[cfg], output='screen',
    )
    odrive_axis1 = Node(
        package='odrive_can', executable='odrive_can_node',
        name='odrive_can_node', namespace='odrive_axis1',
        parameters=[cfg], output='screen',
    )

    # cmd_vel.linear.x (m/s) -> ODrive input_vel, plus steering-drive coupling
    # compensation and the auto-arm sequence.
    odrive_bridge = Node(
        package='swerve_odrive_bridge', executable='swerve_odrive_bridge',
        name='swerve_odrive_bridge', parameters=[cfg], output='screen',
    )

    # cmd_vel.angular.y (rad, chassis frame) -> Dynamixel goal position.
    steering = Node(
        package='swerve_test', executable='swerve_steering_node',
        name='swerve_steering', parameters=[cfg], output='screen',
    )

    # Controller adapter: theta_d -> chassis-frame steer angle, and module
    # state -> the per-agent pose the controller subscribes to.
    cooptrans_bridge = Node(
        package=PKG, executable='module_bridge',
        name='swerve_cooptrans_bridge', parameters=[cfg],
        condition=IfCondition(use_bridge), output='screen',
    )

    return LaunchDescription(args + [
        GroupAction([
            PushRosNamespace(robot_ns),
            odrive_axis0,
            odrive_axis1,
            odrive_bridge,
            steering,
            cooptrans_bridge,
        ])
    ])
