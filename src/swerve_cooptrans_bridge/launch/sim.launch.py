"""Full desktop simulation of the swerve cooperative-transport system.

Runs the SAME nodes the hardware uses -- controller, trajectory_generator,
lidar_pose_adapter, swerve_cooptrans_bridge -- against simulated planner,
sensing and plant. Only the Dynamixel/ODrive drivers are absent.

    sim_planner --<payload>/planned_path--> trajectory_generator (eq. 13)
                                                  |
                                       ref_pose / ref_twist / ref_accel
                                                  v
                                             controller
                                                  |
                     /moduleK/cmd_vel  +  /moduleK/desired_heading
                        |                               |
         swerve_cooptrans_bridge (x2, /robot1 /robot2)   |
           phi = wrap(theta_d - theta_payload - alpha)   |
           -> /robotN/swerveM/cmd_vel  (what the HW gets)|
                        ^                               v
          /robotN/swerveM/heading_angle          kinematic_plant (servo mode)
          (fake encoders from the plant)           - rate-limited steering
                        |                          - Procrustes payload fit
                        +--------------------------+
                                                   |
                                      <payload>/truth_pose
                                                   v
                                  lidar_sim (20 Hz, noise, delay)
                                                   |
                                        /lidar/payload_odom
                                                   v
                                  lidar_pose_adapter -> <payload>/pose
                                                   v
                                             controller + swerve_viz

Topic ownership in sim: the PLANT owns <agent>/pose (it has the true module
state), so the bridges run with publish_module_pose:=false. They are still in
the loop -- they do the theta_d -> steer-angle conversion, and the GUI shows
their output -- but they do not fight the plant for the pose topic.

Usage:
    ros2 launch swerve_cooptrans_bridge sim.launch.py
    ros2 launch swerve_cooptrans_bridge sim.launch.py planner_mode:=direct \
        use_progression_regulator:=false     # test the bypass path
    ros2 launch swerve_cooptrans_bridge sim.launch.py gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

PKG = 'swerve_cooptrans_bridge'


def generate_launch_description():
    cfg = LaunchConfiguration('config')
    gui = LaunchConfiguration('gui')
    use_reg = LaunchConfiguration('use_progression_regulator')
    planner_mode = LaunchConfiguration('planner_mode')
    skip_check = LaunchConfiguration('skip_hardware_check')

    share = get_package_share_directory(PKG)
    default_cfg = os.path.join(share, 'config', 'sim.yaml')

    args = [
        DeclareLaunchArgument('config', default_value=default_cfg),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'planner_mode', default_value='path',
            description="'path' -> sim_planner publishes a receding-horizon "
                        "nav_msgs/Path consumed by trajectory_generator. "
                        "'direct' -> it publishes ref_pose/twist/accel itself "
                        "(use with use_progression_regulator:=false)."),
        DeclareLaunchArgument(
            'use_progression_regulator', default_value='true',
            description='Run trajectory_generator (eq. 13 re-timing). Set '
                        'false together with planner_mode:=direct.'),
        DeclareLaunchArgument('skip_hardware_check', default_value='false'),
    ]

    validator = ExecuteProcess(
        cmd=['ros2', 'run', PKG, 'hardware_check', cfg],
        output='screen', condition=UnlessCondition(skip_check))

    sim_planner = Node(
        package=PKG, executable='sim_planner', name='sim_planner',
        parameters=[cfg, {'mode': planner_mode}], output='screen')

    trajectory_generator = Node(
        package='trajectory_generation', executable='trajectory_generator',
        name='trajectory_generator', parameters=[cfg], output='screen',
        condition=IfCondition(use_reg))

    controller = Node(
        package='controller', executable='controller_node',
        name='controller', parameters=[cfg], output='screen')

    # The plant publishes the TRUE payload pose on a separate topic so the
    # simulated LiDAR can degrade it; <payload>/pose is owned by the adapter,
    # exactly as on hardware.
    plant = Node(
        package='cooptrans_sim', executable='kinematic_plant',
        name='kinematic_plant', parameters=[cfg], output='screen',
        remappings=[('/payload/pose', '/payload/truth_pose'),
                    ('/payload/twist', '/payload/truth_twist')])

    lidar_sim = Node(
        package=PKG, executable='lidar_sim', name='lidar_sim',
        parameters=[cfg], output='screen')

    lidar_adapter = Node(
        package=PKG, executable='lidar_pose_adapter',
        name='lidar_pose_adapter', parameters=[cfg], output='screen')

    bridges = [
        GroupAction([
            PushRosNamespace(ns),
            Node(package=PKG, executable='module_bridge',
                 name='swerve_cooptrans_bridge', parameters=[cfg],
                 output='screen'),
        ]) for ns in ('robot1', 'robot2')
    ]

    viz = Node(
        package=PKG, executable='swerve_viz', name='swerve_viz',
        parameters=[cfg], output='screen', condition=IfCondition(gui))

    return LaunchDescription(args + [
        validator,
        sim_planner,
        trajectory_generator,
        controller,
        plant,
        lidar_sim,
        lidar_adapter,
        *bridges,
        viz,
    ])
