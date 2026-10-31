# ros_odrive — vendored upstream driver

Copied from `odriverobotics/ros_odrive` @ `8ed510b`. Only `odrive_node`
(package name **`odrive_can`**) is used; it provides `odrive_can_node`, which
the swerve stack talks to over `<ns>/control_message`, `<ns>/clear_errors` and
`<ns>/request_axis_state`.

Two upstream packages are marked `COLCON_IGNORE` because nothing here uses
them, and building them only adds compiler warnings to every build:

- `odrive_ros2_control` — a ros2_control `SystemInterface` plugin, an
  alternative integration path we do not take.
- `odrive_botwheel_explorer` — upstream's diffbot demo.

Delete the `COLCON_IGNORE` file in either directory if you ever want them.

Local modification to the vendored tree: `odrive_node/launch/example_launch.yaml`
gained an axis1 node. The swerve stack does not use that launch file (see
`swerve_cooptrans_bridge/launch/robot.launch.py`), but it is kept for bench
testing a bare ODrive.
