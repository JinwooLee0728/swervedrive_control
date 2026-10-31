# swerve_odrive_bridge

Small ROS2 node that bridges swerve module TwistStamped commands to odrive_can ControlMessage publications.

By default it subscribes to `/swerve1/cmd_vel` and `/swerve2/cmd_vel` and publishes to `/odrive_axis0/control_message` and `/odrive_axis1/control_message`.

Parameters:
- `swerve1_topic`, `swerve2_topic`
- `odrive1_topic`, `odrive2_topic`
- `odrive1_velocity_scale`, `odrive2_velocity_scale`

Build with colcon in your workspace root (where `ros_odrive` is located):

```bash
cd /home/yyj/test_motor
colcon build --packages-select odrive_can swerve_odrive_bridge
source install/setup.bash
```

Then run:

```bash
ros2 launch swerve_odrive_bridge swerve_odrive_bridge.launch.py
```

Note: This bridge publishes ODrive `ControlMessage.input_vel` in rev/s (revolutions per second) by default.
The default scaling assumes a wheel radius of 1/6 m so rev/s = linear_x * 6 / (2π) and is set via the parameter `odriveN_velocity_scale`.
Adjust `odrive1_velocity_scale` and `odrive2_velocity_scale` to match your hardware if needed.
