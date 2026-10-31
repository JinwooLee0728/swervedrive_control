# swdr_ws — swerve cooperative transport, hardware + coordination

Two swerve robots, two NUCs, no central computer. NUC1 runs the coordination
stack (localization adapter, reference, controller) plus robot 1's hardware;
NUC2 runs robot 2's hardware and subscribes over the network.

## The idea: a module is an agent

The cooptrans controller models each agent as a nonholonomic unicycle rigidly
attached to the payload — it commands `(v, ω)` and reads the agent's heading.

A steered swerve module **is** exactly that: its contact point rolls only along
the wheel direction, and the steer angle is directly actuated. So each
**module** is registered as an agent, not each robot. 2 robots × 2 modules = 4.

| controller concept | swerve module |
|---|---|
| agent `k` | one steering module |
| `r_x[k], r_y[k]` | module position in the payload frame |
| `theta_arr[k]` | `θ_payload + α_k + φ_k` |
| `out.v[k]` | wheel speed [m/s] → ODrive `input_vel` |
| `out.th_des[k]` | **desired steer angle** (paper eq. 8) → Dynamixel |
| `v_max`, `w_max` | the module's real motor limits |

> **Valid only because each chassis is RIGIDLY BOLTED to the payload.** A
> chassis that can yaw freely makes `r^[k]` time-varying and breaks everything.

## Direct steering

The paper's low-level block exists only because a differential-drive robot
cannot set its heading directly: eq. (22) is a sliding-mode regulator driving
e_θ → 0, and eq. (23) attenuates speed (`v_d·exp(−γ_v·e_θ)`) while it turns.
A Dynamixel-steered module is commanded straight to θ_d, so both are skipped
(`proposed.direct_steering: true`).

This matters because **Proposition 2 guarantees strict kinematic feasibility
only at e_θ = 0**, which direct steering achieves by construction rather than
asymptotically. Measured in simulation (40 s runs, 4 modules):

| configuration | tracking error | e_θ mean | e_θ max | slip |
|---|---|---|---|---|
| published cascade (eq. 22–23) | 0.030 m | 0.0052 rad | 0.0271 rad | 0.00000 |
| **direct steering (θ_d)** | **0.020 m** | **0.0021 rad** | **0.0144 rad** | 0.00000 |

The system-wide scaling (eq. 24) is **kept** — κ = |v_d|/v̄ + |ω_d|/ω̄ still
bounds the slew demand against the servo's real limit, preserving Prop. 1.

This is a deliberate deviation from the published cascade. Hardware results are
"high-level + rigid-body mappings with direct steering", not the paper's
controller as published. Set `direct_steering: false` to reproduce the paper.

## Architecture

```
NUC1  (robot 1 + coordination)                NUC2  (robot 2 only)
  hardware_check          <- runs first, aborts launch on config mismatch
  lidar_pose_adapter  --> /payload/pose ------------> (subscribes)
  trajectory_generator -> /payload/ref_{pose,twist,accel}
  controller ---> /module1..4/cmd_vel  (wheel m/s)
                  /module1..4/desired_heading  (theta_d, world frame)
       |                                              |
  /robot1/* hardware                            /robot2/* hardware
    swerve_cooptrans_bridge                       swerve_cooptrans_bridge
      phi = wrap(theta_d - theta_payload - alpha)
      -> /robotN/swerveM/cmd_vel (TwistStamped: linear.x=m/s, angular.y=rad)
      -> /moduleK/pose  (yaw = theta_payload + alpha + phi_measured)
         |                    |
    swerve_steering      swerve_odrive_bridge -> odrive_axis0/1 -> ODrive
      (Dynamixel)
```

Robots are separated **by namespace alone** — every topic inside `swerve_test`,
`swerve_odrive_bridge` and `odrive_can` is relative. The deliberate exception
is the bridge's controller-facing topics (`/moduleK/*`), which are absolute
because the controller lives in the root namespace, possibly on the other NUC.

## Network

Both NUCs, before launching:

```bash
export ROS_DOMAIN_ID=17                     # same on both
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

No clock sync is required: every watchdog compares against a receipt time
stamped on its own machine. If NUC1 or the network dies, robot 2's bridge times
out after `cmd_timeout` (0.2 s), stops its wheels and **holds** steering.

## One config file

`config/hardware.yaml` is the only file to edit when hardware changes. It is
passed to every node on both NUCs; blocks are keyed by fully-qualified node
name (`/robot1/swerve_steering`, …) so one file configures both robots.

Values several nodes need are necessarily repeated — ROS 2 parameter files
cannot alias across blocks. That duplication is a **checked invariant**, not a
convention: `hardware_check` runs first and aborts the launch on any mismatch.

```
$ ros2 run swerve_cooptrans_bridge hardware_check config/hardware.yaml
  FATAL geometry mismatch for 'module1': controller has r=(+0.4000, +0.2300)
        but /robot1 mount+offset gives (+0.3500, +0.2300), off by 0.0500 m.
  FATAL /robot1: controller w_max=1.2000 rad/s but profile_velocity=100.0 with
        gear_ratio=5.0 delivers only 0.4796 rad/s. Set profile_velocity=250.2
        or lower w_max to 0.4796.
```

It checks: module geometry vs mount+offset, `w_max` vs the Dynamixel's
`profile_velocity`, steering gear-ratio agreement between the steering node and
the ODrive bridge, bridge clamps vs controller limits, module-name pairing,
agent ownership (every module owned by exactly one robot), the sim mirror, and
warns on an implausible `wheel_radius`.

## Build

This workspace is **self-contained** — it needs no other checkout, so a robot
NUC only has to clone this one.

```bash
sudo apt install -y ros-$ROS_DISTRO-dynamixel-sdk can-utils \
                    python3-numpy python3-pyqtgraph python3-yaml
cd ~/swdr_ws
colcon build --symlink-install
source install/setup.bash
```

| package | role |
|---|---|
| `swerve_cooptrans_bridge` | controller adapter, LiDAR adapter, config validator, simulation, GUI, all launch files |
| `swerve_test` | Dynamixel steering node |
| `swerve_odrive_bridge` | wheel speed → ODrive, coupling compensation, auto-arm |
| `ros_odrive` | vendored ODrive CAN driver (upstream `odriverobotics/ros_odrive`) |
| `keyboard_to_swerve` | single-robot bench teleop |
| `controller` | the control law — **vendored** from `cooptrans_ws` |
| `trajectory_generation` | reference + eq. 13 progression regulator — **vendored** |
| `cooptrans_sim` | kinematic plant — **vendored** |

The last three originate in `cooptrans_ws` and carry swerve-specific changes
(`direct_steering`, the servo steering model, fake encoders). These copies are
what the swerve launches use; edits here do not propagate to `cooptrans_ws`,
or the other way round.

> **Source only this workspace for swerve work.** `cooptrans_ws` provides
> packages by the same three names, so sourcing both means whichever was
> sourced *last* silently wins. `hardware_check` runs first in every launch and
> prints where each package actually resolved, marking any that came from
> elsewhere with `!!` — so if you ever wonder which controller is running, read
> the top of the launch output:
>
> ```
> package origins:
>    controller              .../swdr_ws/install/controller
> !! cooptrans_sim           .../cooptrans_ws/install/cooptrans_sim
> ```

## Calibration (once, after bolting)

`cooptrans_bringup`'s `init.launch.py` is **obsolete here** — it spins each
robot to calibrate a *pinned* attachment. Do not run it. Instead:

1. Record the four module positions in the payload frame → `controller.r_x/r_y`.
2. Record each chassis's mount pose → `mount_x`/`mount_y`/`mount_angle`.
   `hardware_check` verifies (1) equals mount + R(α)·offset.
3. Per boot: hand-align both wheels along chassis +x before launching
   (`assume_forward_on_startup` snapshots that as module zero).


## Run

```bash
# NUC1
ros2 launch swerve_cooptrans_bridge nuc1.launch.py
# NUC2
ros2 launch swerve_cooptrans_bridge nuc2.launch.py
```

Single-robot bench teleop (no controller):

```bash
ros2 launch swerve_cooptrans_bridge robot.launch.py robot_ns:=robot1 use_bridge:=false
ros2 run keyboard_to_swerve swerve_keyboard --ros-args -r __ns:=/robot1 \
  --params-file $(ros2 pkg prefix keyboard_to_swerve)/share/keyboard_to_swerve/config/keyboard_to_swerve.yaml
```

`use_bridge:=false` matters — the keyboard node and the bridge both publish
`<module>/cmd_vel`, so running both gives two publishers fighting.

## Simulation

Run the whole system on a desktop before touching hardware. The **same nodes**
the robots run — controller, trajectory_generator, lidar_pose_adapter,
swerve_cooptrans_bridge — execute against a simulated planner, sensor and
plant. Only the Dynamixel/ODrive drivers are absent.

```bash
ros2 launch swerve_cooptrans_bridge sim.launch.py
```

| piece | node | what it fakes |
|---|---|---|
| planner | `sim_planner` | receding-horizon `nav_msgs/Path`, replanned every 1 s from the payload's actual progress, over a closed lissajous loop |
| sensor | `lidar_sim` | 20 Hz pose with 4 mm / 4 mrad Gaussian noise, optional delay and dropouts |
| plant | `kinematic_plant` | payload physics + the Dynamixel servo (rate-limited slew to θ_d) + fake steering encoders |
| GUI | `swerve_viz` | top-down state and commands |

Topic ownership in sim: the plant owns `<agent>/pose` because it has the true
module state, so the bridges run with `publish_module_pose:=false`. They are
still in the loop doing the θ_d → steer-angle conversion — the GUI shows their
actual output on `/robotN/swerveM/cmd_vel`, i.e. exactly what the hardware
would receive.

Test the bypass path (planner emits setpoints instead of a horizon, so eq. 13
is not in the loop):

```bash
ros2 launch swerve_cooptrans_bridge sim.launch.py \
    planner_mode:=direct use_progression_regulator:=false
```

Config is `config/sim.yaml` — a separate file from `hardware.yaml` so you can
perturb the simulation freely, but validated by the same `hardware_check`, so
a sim that quietly diverges from the rig geometry is caught rather than
trusted.

### The GUI

`swerve_viz` shows state and control input together:

- **Top-down** — payload at the *estimated* pose with a dotted outline at the
  *true* pose, so localization error is visible rather than assumed; each
  module as a wheel at its **measured heading**, a solid arrow for
  **commanded wheel speed**, and a dashed spoke for **commanded heading**;
  the reference path and a ghost payload at the setpoint, joined to the
  measured pose by a line whose length is the error driving eq. 13.
- **Common ICR**, fit in least squares from the module headings, with the
  wheel-normal rays drawn to it. Each module constrains the ICR to lie
  perpendicular to its own heading, so all rays meeting at one point *is* the
  statement that the commands are ICR-consistent. The printed residual is
  effectively the paper's ρ_ICR — it sits around 0.002 m in a healthy run.
- **Module heading panel** — grey bar = commanded θ_d, black tick = measured,
  red segment = the error between them.
- **Commanded wheel speed** against the ±v_max envelope, turning red on
  violation.
- **Rolling traces** of payload position error and max module heading error.

Capture a figure without a screen recorder:

```bash
ros2 run swerve_cooptrans_bridge swerve_viz --ros-args \
  --params-file <share>/swerve_cooptrans_bridge/config/sim.yaml \
  -p screenshot_path:=/tmp/run.png -p screenshot_after:=30.0
```


## External interfaces

**Localization.** The controller reads `geometry_msgs/PoseStamped` on
`<payload>/pose`; it never had an OptiTrack dependency. `lidar_pose_adapter`
absorbs the message type — set `input_type` to `PoseStamped`, `Odometry`, or
`PoseWithCovarianceStamped`. Measured twist is optional (viz only; the
controller uses `ref_twist`).

**Reference.** Launch with `use_progression_regulator:=true` (default) and have
the planner publish its horizon as `nav_msgs/Path` on `<payload>/planned_path`.
`trajectory_generator`'s planner mode was already built for a receding-horizon
replanner: it re-fits the spline across replans and applies the paper's
progression-rate regulator, eq. 13 (`τ̇ = 1/(1 + eᵀKₛe)`), which slows the
reference as tracking error grows.

If your planner publishes instantaneous pose/twist/accel instead, launch with
`use_progression_regulator:=false` — the controller then subscribes directly.
**Eq. 13 is then unavailable**: re-timing evaluates q_ref(τ) and needs a
parameterized path, not a setpoint. There is no way to recover it downstream.

## Steering rate is the binding constraint

```
w_max [rad/s] = profile_velocity * 0.229 * 2*pi / (60 * gear_ratio)
```

Measured with direct steering and a rate-limited servo plant:

| `profile_velocity` | `w_max` | tracking error | e_θ max | slip |
|---|---|---|---|---|
| 100 | 0.48 rad/s | 0.060 m | 0.550 rad | 0.00004 |
| **250** | **1.2 rad/s** | **0.020 m** | **0.014 rad** | 0.00000 |
| 625 | 3.0 rad/s | 0.010 m | 0.011 rad | 0.00000 |

At 0.48 rad/s the module lags θ_d by up to 0.55 rad and slip becomes non-zero —
the servo cannot meet the ICR-consistent slew demand. That is exactly what the
bridge's local `attenuate_on_heading_error` guards against: with direct steering
the controller assumes e_θ ≈ 0, so the bridge bleeds off wheel speed when the
measured heading lags, and stops outright past `heading_error_stop`. Verify 250
is within your servo's Velocity Limit (EEPROM 44; ~265 on a stock XM430-W350).

## Verify before trusting hardware

- **`wheel_radius: 0.35`** is a 0.7 m diameter wheel, larger than the 0.46 m
  module track. If the real wheel is 70 mm this must be `0.035` and every wheel
  command is 10× wrong. `hardware_check` warns; it still needs a measurement.
- **`steering_drive_coupling`** must be measured: hold the drive at zero, turn
  the steering by hand, tune until the wheel stops rolling.
- **ODrive `node_id`** must be unique per physical CAN bus (6-bit, 0..63).
  Separate buses per robot means 0/1 can repeat.
- **NUC1 loop jitter** — it carries the controller, adapter, trajectory
  generator and its own hardware. The paper ran 100 Hz for 4 robots on an
  i7-1165G7, so headroom is likely fine, but measure it.

## Only `proposed` runs on hardware

`kanayama`, `li`, `peng` and `muhamed` still build and run in simulation, but
they emit unicycle `(v, ω)` and compute no desired heading, so they cannot
drive the swerve modules. `li` and `muhamed` additionally need `ref_path` /
`progress`, which the hardware launch no longer provides.
