from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bridge_cfg = PathJoinSubstitution([
        FindPackageShare('swerve_odrive_bridge'),
        'config',
        'swerve_odrive_bridge.yaml',
    ])

    return LaunchDescription([
        Node(
            package='swerve_odrive_bridge',
            executable='swerve_odrive_bridge',
            name='swerve_odrive_bridge',
            parameters=[bridge_cfg],
            output='screen',
        )
    ])
