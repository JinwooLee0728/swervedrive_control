from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    steering_cfg = PathJoinSubstitution([
        FindPackageShare('swerve_test'),
        'config',
        'swerve_steering.yaml',
    ])
    bridge_cfg = PathJoinSubstitution([
        FindPackageShare('swerve_odrive_bridge'),
        'config',
        'swerve_odrive_bridge.yaml',
    ])

    odrive_axis0 = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name='odrive_can_node',
        namespace='odrive_axis0',
        parameters=[{'node_id': 0, 'interface': 'can0'}],
        output='screen',
    )

    odrive_axis1 = Node(
        package='odrive_can',
        executable='odrive_can_node',
        name='odrive_can_node',
        namespace='odrive_axis1',
        parameters=[{'node_id': 1, 'interface': 'can0'}],
        output='screen',
    )

    swerve_odrive_bridge = Node(
        package='swerve_odrive_bridge',
        executable='swerve_odrive_bridge',
        name='swerve_odrive_bridge',
        parameters=[bridge_cfg],
        output='screen',
    )

    swerve_steering = Node(
        package='swerve_test',
        executable='swerve_steering_node',
        name='swerve_steering',
        parameters=[steering_cfg],
        output='screen',
    )

    return LaunchDescription([
        odrive_axis0,
        odrive_axis1,
        swerve_odrive_bridge,
        swerve_steering,
    ])
