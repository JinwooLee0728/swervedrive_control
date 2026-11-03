# swdr_ws — swerve cooperative transport

Two swerve robots, each with two steered modules, are rigidly bolted to one
payload and carry it together. There is no central computer:

- **NUC1** runs robot 1's hardware and the coordination stack (localization
  adapter, reference generator, controller).
- **NUC2** runs robot 2's hardware and talks to NUC1 over the network.

You can drive the payload two ways:

| mode | who commands the modules | use it for |
|---|---|---|
| **cooperative transport** | `controller` (the paper's control law) tracking a planned path | experiments |
| **payload teleop** | `payload_keyboard`, from the keyboard | bring-up, manual moves, demos |

Both modes run in simulation as well as on the real robots.

---

## Contents

1. [Quick start](#quick-start)
2. [Build](#build)
3. [Simulation](#simulation)
4. [Real hardware](#real-hardware)
5. [Keyboard teleop](#keyboard-teleop)
6. [The GUI](#the-gui)
7. [Configuration](#configuration)
8. [Before the first powered run](#before-the-first-powered-run)
9. [Architecture](#architecture)
10. [Design notes](#design-notes)

---

## Quick start

```bash
# build (once)
source /opt/ros/humble/setup.bash
colcon build --symlink-install && source install/setup.bash

# drive the simulated payload from the keyboard, with the GUI
ros2 launch keyboard_to_swerve sim_teleop.launch.py

# watch the controller track a path in simulation
ros2 launch swerve_cooptrans_bridge sim.launch.py
```

---

## Build

The workspace is self-contained, so a robot NUC only needs to clone this
repository.

```bash
sudo apt install -y ros-$ROS_DISTRO-dynamixel-sdk can-utils \
                    python3-numpy python3-pyqtgraph python3-yaml
cd ~/swdr_ws
colcon build --symlink-install
source install/setup.bash
```

Every new terminal needs `source install/setup.bash`.

| package | role |
|---|---|
| `swerve_cooptrans_bridge` | controller ↔ hardware adapter, LiDAR adapter, config validator, simulation nodes, GUI, launch files |
| `keyboard_to_swerve` | keyboard teleop: whole payload (`payload_keyboard`) or one robot (`swerve_keyboard`) |
| `swerve_test` | Dynamixel steering node |
| `swerve_odrive_bridge` | wheel speed → ODrive, steering–drive coupling compensation, auto-arm |
| `ros_odrive` | ODrive CAN driver, vendored from upstream `odriverobotics/ros_odrive` |
| `controller` | the control law, **vendored** from `cooptrans_ws` |
| `trajectory_generation` | reference generator + eq. 13 progression regulator, **vendored** |
| `cooptrans_sim` | kinematic plant, **vendored** |

The three vendored packages come from `cooptrans_ws` and carry swerve-specific
changes (`direct_steering`, the servo steering model, fake encoders). Edits do
not propagate in either direction.

> **Source only this workspace.** `cooptrans_ws` provides packages with the
> same three names, and whichever workspace is sourced *last* silently wins.
> `hardware_check` runs first in every launch and prints where each package
> resolved, marking foreign ones with `!!`:
>
> ```
> package origins:
>    controller              .../swdr_ws/install/controller
> !! cooptrans_sim           .../cooptrans_ws/install/cooptrans_sim
> ```

---

## Simulation

The simulation runs the **same nodes** as the robots: controller, trajectory
generator, LiDAR adapter and both bridges. Only the Dynamixel and ODrive
drivers are replaced:

| piece | node | what it fakes |
|---|---|---|
| planner | `sim_planner` | receding-horizon `nav_msgs/Path` over a closed lissajous loop, replanned every 1 s |
| sensor | `lidar_sim` | 20 Hz pose with 4 mm / 4 mrad Gaussian noise, optional delay and dropouts |
| plant | `kinematic_plant` | payload motion, the Dynamixel servo (rate-limited slew), fake steering encoders |
| GUI | `swerve_viz` | top-down state and commands |

### Keyboard teleop in simulation

```bash
ros2 launch keyboard_to_swerve sim_teleop.launch.py
```

This starts the plant, simulated LiDAR, LiDAR adapter, both bridges, the GUI
and `payload_keyboard`. The planner and controller are left out because they
publish the same topics as the teleop.

`ros2 launch` cannot pass keystrokes to a node, so **the teleop opens in its
own terminal window. Click that window before typing.** Closing it, or Ctrl-C
inside it, shuts the whole simulation down; Ctrl-C on the launch closes the
window too.

| argument | default | |
|---|---|---|
| `gui` | `true` | start `swerve_viz` |
| `terminal` | `gnome-terminal --wait --` | how to open the teleop window, e.g. `terminal:='xterm -e'` |
| `config` | `swerve_cooptrans_bridge/config/sim.yaml` | sim config |
| `skip_hardware_check` | `false` | skip the config validator |

### Controller in simulation

```bash
ros2 launch swerve_cooptrans_bridge sim.launch.py
```

To test the bypass path, where the planner emits setpoints directly and
eq. 13 is not in the loop:

```bash
ros2 launch swerve_cooptrans_bridge sim.launch.py \
    planner_mode:=direct use_progression_regulator:=false
```

### Notes

- The sim config is `config/sim.yaml`, separate from `hardware.yaml` so you
  can perturb it freely. `hardware_check` validates it the same way, so a sim
  that drifts from the rig geometry is caught.
- In sim the plant owns `/moduleK/pose` (it knows the true module state), so
  the bridges run with `publish_module_pose: false`. They still do the
  heading → steer-angle conversion, and the GUI shows their real output on
  `/robotN/swerveM/cmd_vel`, which is exactly what the hardware would get.

---

## Real hardware

### Network (both NUCs, every terminal)

```bash
export ROS_DOMAIN_ID=17                     # same on both
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

No clock sync is needed: every watchdog compares against a receipt time from
its own machine.

### Before every boot

Hand-align all four wheels along their chassis +x. With
`assume_forward_on_startup` the steering node takes that position as module
zero.

### Cooperative transport (controller)

```bash
# NUC1
ros2 launch swerve_cooptrans_bridge nuc1.launch.py
# NUC2
ros2 launch swerve_cooptrans_bridge nuc2.launch.py
```

### Payload teleop (both robots, from the keyboard)

```bash
# NUC2
ros2 launch swerve_cooptrans_bridge nuc2.launch.py
# NUC1: robot 1's hardware only (NOT nuc1.launch.py, which starts the controller)
ros2 launch swerve_cooptrans_bridge robot.launch.py robot_ns:=robot1
# NUC1, a second interactive terminal
ros2 run keyboard_to_swerve payload_keyboard
```

Without LiDAR the teleop publishes a fixed identity `/payload/pose`, so the
world frame equals the payload frame. If `lidar_pose_adapter` is running, use
`ros2 run keyboard_to_swerve payload_keyboard --ros-args -p use_external_pose:=true`
instead. The teleop refuses to drive if it finds a second publisher on
`/payload/pose`.

### Single-robot bench test (one robot, not on the payload)

```bash
ros2 launch swerve_cooptrans_bridge robot.launch.py robot_ns:=robot1 use_bridge:=false
ros2 run keyboard_to_swerve swerve_keyboard --ros-args -r __ns:=/robot1 \
  --params-file $(ros2 pkg prefix keyboard_to_swerve)/share/keyboard_to_swerve/config/keyboard_to_swerve.yaml
```

`use_bridge:=false` is required because the keyboard node and the bridge both
publish `<module>/cmd_vel`.

> ⚠ `swerve_odrive_bridge` has **no command timeout**: it keeps sending the
> last wheel speed until told otherwise. In the controller and payload-teleop
> setups the bridge's own watchdog covers this. In the bench setup above,
> nothing does, so keep a hand on the E-stop.

### Failure behaviour

If NUC1, the network or the teleop dies, each robot's bridge times out after
`cmd_timeout` (0.2 s), stops its wheels and **holds** steering. This runs
locally on each NUC and does not depend on hearing from the other machine.

### Running from a laptop

No monitor on the NUCs is needed. Put the laptop on the same subnet and use
the same network variables as above.

**Teleop over SSH.** `payload_keyboard` is a terminal program, so it runs over
SSH. Use `-t`, because it refuses to start without a TTY:

```bash
ssh -t -o ServerAliveInterval=1 -o ServerAliveCountMax=2 user@nuc1
ros2 run keyboard_to_swerve payload_keyboard
```

> ⚠ The bridge watchdogs do not cover the laptop–NUC1 link. If Wi-Fi freezes,
> a hung SSH session leaves the teleop running on NUC1, still publishing your
> last command. The `ServerAlive` options close a dead session, and the teleop
> with it, within about 2 s. Do **not** run the teleop inside `tmux` or
> `screen`, which keep it alive after SSH drops.

**Teleop on the laptop.** Alternatively, build and source this workspace on
the laptop and run `payload_keyboard` there. A Wi-Fi loss then stops the
wheels within `cmd_timeout`. The cost: commands and module feedback both go
over Wi-Fi, so a poor link can trip the watchdog and make the wheels stutter.

**GUI.** Run `swerve_viz` on the laptop, not over `ssh -X` (pyqtgraph is slow
over X11). It only subscribes, so it is always safe:

```bash
ros2 run swerve_cooptrans_bridge swerve_viz
```

---

## Keyboard teleop

`payload_keyboard` drives the payload as one rigid body. Commands are in the
**payload frame**.

| key | action |
|---|---|
| `w` / `x` | forward / back (±0.01 m/s per press) |
| `a` / `d` | left / right (±0.01 m/s per press) |
| `q` / `e` | rotate CCW / CW (±0.05 rad/s per press) |
| `s` or space | stop; steering holds its angle |
| Ctrl-C | quit; wheels stop |

The status line shows the command, the largest module steering error, and the
current wheel gain. A gain of 0.00 means the wheels are held while the modules
finish steering.

**How it works.** For each module at payload-frame position `r_k`, the module
velocity is `(vx − ω·r_y[k], vy + ω·r_x[k])`, which makes all four modules
share one centre of rotation. The module positions and `v_max` are read from
the `controller` block of `hardware.yaml`, so there is no second copy of the
geometry. The teleop publishes `/moduleK/cmd_vel` and
`/moduleK/desired_heading`, the same interface as the controller, so both
bridges stay in the loop. That gives you the per-NUC watchdog, the mount-angle
conversion and the speed clamp.

**Safety.**

- The whole twist is scaled down when any module would exceed `v_max`, which
  keeps the centre of rotation.
- Paper eq. 8–9, same as the controller: of the two equivalent wheel headings
  (h forward / h + 180° backward) each module takes the one nearer its
  measured heading, and the wheel speed is signed. So `x` after `w` (or `d`
  after `a`, `e` after `q`) just spins the wheels the other way; a 90° change
  steers 90°. Wheels may end up pointing backward and running in reverse,
  which is kinematically identical.
- The sent twist ramps at `max_accel`, so a new direction sweeps each wheel's
  velocity across instead of reversing it (`s` still stops at once).
- Wheels wait until every module's commanded and measured heading are within `align_window` of its
  target, so a wheel never drives along its old axis while turning.
- Each module's commanded heading turns at most `max_heading_rate`.
- All wheels slow together while any module is still short of its target
  heading (measured vs. target, not vs. the slewed command), and stop if its
  error exceeds `heading_error_stop` (0.5 rad).
- Wheels stop if feedback from any module is missing.

| parameter | default | |
|---|---|---|
| `config` | `swerve_cooptrans_bridge/config/hardware.yaml` | where the geometry is read from |
| `use_external_pose` | `false` | read `/payload/pose` instead of publishing an identity pose |
| `lin_step` / `ang_step` | 0.01 m/s / 0.01 rad/s | change per key press |
| `heading_error_gain` | 7.0 | wheel gain `exp(−gain·e_max)` while steering |
| `heading_error_stop` | 0.5 rad | above this steering error, all wheels stop |
| `max_heading_rate` | 0.8 rad/s | max slew of each module's commanded heading; keep below the REAL steering rate (robot1 measured 0.90–0.95 rad/s under load, although `profile_velocity` 250 predicts ~1.2). 0 disables |
| `max_accel` | 0.15 m/s² | ramp of the sent twist (max change of any module velocity vector); `s` and quit bypass it. 0 disables |
| `branch_hysteresis` | 0.2 rad | eq. 8 switch margin: change to the other heading only if it is this much nearer the measured heading (same as `proposed.branch_hysteresis`; 0 = eq. 8 exactly) |
| `align_window` | 0.5 rad | wheels held at 0 until every moving module's commanded AND measured heading are within this of its target, full speed at 0. 0 disables |
| `agents` | all | subset of agents to drive, e.g. `"[module1, module2]"` for robot 1 alone |
| `rate_hz` | 50 | publish rate (bridge timeout is 0.2 s) |

Never run it together with the controller: both publish `/moduleK/*`.

---

## The GUI

`swerve_viz` only subscribes, so it is safe to run anywhere on the same
`ROS_DOMAIN_ID`, including a laptop.

- **Top-down view.** The payload is drawn at its *estimated* pose, with a
  dotted outline at the *true* pose in sim. Each module is drawn as a wheel at
  its measured heading, with a solid arrow for commanded wheel speed and a
  dashed spoke for commanded heading. With a controller running, the reference
  path is shown, plus a ghost payload at the setpoint joined to the measured
  pose.
- **Common centre of rotation.** A least-squares fit from the module headings,
  with the wheel-normal rays drawn to it. All rays meeting at one point means
  the commands are consistent. The printed residual is effectively the paper's
  ρ_ICR, around 0.002 m in a healthy controller run.
- **Module heading panel.** Grey bar = commanded, black tick = measured, red =
  the error.
- **Wheel speed panel.** Commanded speed against ±`v_max`, red on violation.
- **Tracking traces.** Payload position error and max module heading error.
  These need a reference trajectory, so **they stay empty during teleop**.

On hardware, reuse the sim config with ground truth off:

```bash
S=$(ros2 pkg prefix swerve_cooptrans_bridge)/share/swerve_cooptrans_bridge/config/sim.yaml
ros2 run swerve_cooptrans_bridge swerve_viz --ros-args --params-file $S -p truth_topic:=''
```

Keep its `swerve_viz` geometry in sync with `hardware.yaml`. During teleop
without LiDAR, the payload stays drawn at the origin and only the modules
move.

To capture a figure without a screen recorder:

```bash
ros2 run swerve_cooptrans_bridge swerve_viz --ros-args --params-file $S \
  -p screenshot_path:=/tmp/run.png -p screenshot_after:=30.0
```

---

## Configuration

`swerve_cooptrans_bridge/config/hardware.yaml` is the **only file to edit when
the hardware changes**. Every node on both NUCs gets it; blocks are keyed by
fully-qualified node name (`/robot1/swerve_steering`, …), so one file
configures both robots.

Some values are needed by several nodes and must be repeated, because ROS 2
parameter files cannot alias across blocks. That duplication is a **checked
invariant**: `hardware_check` runs first and aborts the launch on any mismatch.

```
$ ros2 run swerve_cooptrans_bridge hardware_check config/hardware.yaml
  FATAL geometry mismatch for 'module1': controller has r=(+0.4000, +0.2300)
        but /robot1 mount+offset gives (+0.3500, +0.2300), off by 0.0500 m.
  FATAL /robot1: controller w_max=1.2000 rad/s but profile_velocity=100.0 with
        gear_ratio=5.0 delivers only 0.4796 rad/s. Set profile_velocity=250.2
        or lower w_max to 0.4796.
```

It checks:

- module geometry against mount + offset,
- `w_max` against the Dynamixel's `profile_velocity`,
- that the steering node and the ODrive bridge use the same steering gear
  ratio,
- bridge clamps against controller limits,
- module-name pairing, and that every module is owned by exactly one robot,
- the sim mirror.

It also warns on an implausible `wheel_radius`.

### Calibration (once, after bolting)

Do **not** run `cooptrans_bringup`'s `init.launch.py`. It spins each robot to
calibrate a *pinned* attachment and is obsolete here.

1. Measure the four module positions in the payload frame → `controller.r_x/r_y`.
2. Measure each chassis's mount pose → `mount_x` / `mount_y` / `mount_angle`.
   `hardware_check` verifies that (1) equals mount + R(α)·offset.
3. Per boot, hand-align the wheels (see [Before every boot](#before-every-boot)).

**ODrive motor calibration.** If `swerve_odrive_bridge` dies with
`procedure_result=14 (NOT_CALIBRATED)`, recalibrate that axis over CAN. Stop
the robot launch and lift the wheel first: the motor spins on its own.

```bash
tools/odrive_calibrate.sh --status 0   # read-only: state of node_id 0
tools/odrive_calibrate.sh 0            # calibrate, save, verify closed loop
```

---

## Before the first powered run

- **Geometry.** Every number in `hardware.yaml` is a placeholder until
  measured on the rig.
- **`wheel_radius: 0.35`** is a 0.7 m wheel, larger than the 0.46 m module
  track. If the real wheel is 70 mm this must be `0.035`, otherwise every
  wheel command is 10× wrong. `hardware_check` warns, but it still needs a
  measurement.
- **`steering_drive_coupling`** must be measured: hold the drive at zero, turn
  the steering by hand, and tune until the wheel stops rolling.
- **ODrive `node_id`** must be unique per physical CAN bus (0..63). Each robot
  has its own bus, so 0/1 can repeat across robots.
- **Servo velocity limit.** `profile_velocity: 250` must be within the servo's
  Velocity Limit (EEPROM 44; ~265 on a stock XM430-W350). See
  [Steering rate](#steering-rate-is-the-binding-constraint).
- **NUC1 loop jitter.** It carries the controller, adapter, trajectory
  generator and its own hardware. The paper ran 100 Hz for 4 robots on an
  i7-1165G7, so there is likely headroom, but measure it.

---

## Architecture

```
NUC1  (robot 1 + coordination)                NUC2  (robot 2 only)
  hardware_check          <- runs first, aborts launch on config mismatch
  lidar_pose_adapter  --> /payload/pose ------------> (subscribes)
  trajectory_generator -> /payload/ref_{pose,twist,accel}
  controller ---> /module1..4/cmd_vel  (wheel m/s)          [or payload_keyboard]
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

Robots are separated **by namespace alone**: every topic inside `swerve_test`,
`swerve_odrive_bridge` and `odrive_can` is relative. The one deliberate
exception is the bridge's controller-facing topics (`/moduleK/*`), which are
absolute because the controller runs in the root namespace, possibly on the
other NUC.

### External interfaces

**Localization.** The controller reads `geometry_msgs/PoseStamped` on
`<payload>/pose` and has no OptiTrack dependency. `lidar_pose_adapter`
converts your LiDAR output: set `input_type` to `PoseStamped`, `Odometry`, or
`PoseWithCovarianceStamped`. Measured twist is optional (GUI only; the
controller uses `ref_twist`).

**Reference.** With `use_progression_regulator:=true` (default), the planner
publishes its horizon as `nav_msgs/Path` on `<payload>/planned_path`.
`trajectory_generator` re-fits the spline across replans and applies the
paper's progression-rate regulator, eq. 13 (`τ̇ = 1/(1 + eᵀKₛe)`), which slows
the reference as tracking error grows.

If your planner publishes instantaneous pose/twist/accel instead, launch with
`use_progression_regulator:=false` and the controller subscribes directly.
**Eq. 13 is then unavailable**, because re-timing needs a parameterized path,
not a setpoint.

---

## Design notes

### A module is an agent

The cooptrans controller models each agent as a nonholonomic unicycle rigidly
attached to the payload: it commands `(v, ω)` and reads the agent's heading. A
steered swerve module **is** exactly that, since its contact point rolls only
along the wheel direction and the steer angle is directly actuated. So each
**module** is registered as an agent, not each robot: 2 robots × 2 modules = 4.

| controller concept | swerve module |
|---|---|
| agent `k` | one steering module |
| `r_x[k], r_y[k]` | module position in the payload frame |
| `theta_arr[k]` | `θ_payload + α_k + φ_k` |
| `out.v[k]` | wheel speed [m/s] → ODrive `input_vel` |
| `out.th_des[k]` | **desired steer angle** (paper eq. 8) → Dynamixel |
| `v_max`, `w_max` | the module's real motor limits |

> **This is valid only because each chassis is RIGIDLY BOLTED to the
> payload.** A chassis that can yaw freely makes `r^[k]` time-varying and
> breaks everything.

### Direct steering

The paper's low-level block exists only because a differential-drive robot
cannot set its heading directly. Eq. (22) is a sliding-mode regulator driving
e_θ → 0, and eq. (23) attenuates speed (`v_d·exp(−γ_v·e_θ)`) while it turns. A
Dynamixel-steered module is commanded straight to θ_d, so both are skipped
(`proposed.direct_steering: true`).

This matters because **Proposition 2 guarantees strict kinematic feasibility
only at e_θ = 0**, which direct steering achieves by construction rather than
asymptotically. Measured in simulation (40 s runs, 4 modules):

| configuration | tracking error | e_θ mean | e_θ max | slip |
|---|---|---|---|---|
| published cascade (eq. 22–23) | 0.030 m | 0.0052 rad | 0.0271 rad | 0.00000 |
| **direct steering (θ_d)** | **0.020 m** | **0.0021 rad** | **0.0144 rad** | 0.00000 |

The system-wide scaling (eq. 24) is **kept**: κ = |v_d|/v̄ + |ω_d|/ω̄ still
bounds the slew demand against the servo's real limit, preserving Prop. 1.

This is a deliberate deviation from the published cascade. Hardware results
are "high-level + rigid-body mappings with direct steering", not the paper's
controller as published. Set `direct_steering: false` to reproduce the paper.

### Steering rate is the binding constraint

```
w_max [rad/s] = profile_velocity * 0.229 * 2*pi / (60 * gear_ratio)
```

Measured with direct steering and a rate-limited servo plant:

| `profile_velocity` | `w_max` | tracking error | e_θ max | slip |
|---|---|---|---|---|
| 100 | 0.48 rad/s | 0.060 m | 0.550 rad | 0.00004 |
| **250** | **1.2 rad/s** | **0.020 m** | **0.014 rad** | 0.00000 |
| 625 | 3.0 rad/s | 0.010 m | 0.011 rad | 0.00000 |

At 0.48 rad/s the module lags θ_d by up to 0.55 rad and slip becomes non-zero,
because the servo cannot meet the slew demand. The bridge's local
`attenuate_on_heading_error` guards against this: with direct steering the
controller assumes e_θ ≈ 0, so the bridge bleeds off wheel speed when the
measured heading lags, and stops outright past `heading_error_stop`.

### Heading branch and wheel direction (eq. 8–9)

`rigidBody()` follows the paper: of the two ICR-perpendicular headings
θ + φ ± π/2, it takes the one nearer the measured module heading (eq. 8), and
the wheel speed v_d = ṗ_dᵀ t(θ_d) is signed (eq. 9). A module whose travel
reverses rolls backward on the same axis instead of steering 180°, so θ_d
stays continuous. The only addition is `proposed.branch_hysteresis` (0.2 rad;
0 gives eq. 8 exactly), which keeps a module about 90° from both candidates
from switching on encoder noise. `payload_keyboard` uses the same rule.

An earlier "forward-only" version hid a hardware fault: robot1's ODrive axes
were cross-wired between the left and right wheels, which only shows once the
two wheels of a robot need opposite spins. Bench-test each robot's wiring
before trusting signed speeds: with wheels lifted and `robot.launch.py
use_bridge:=false`, publish to one `/robotN/swerveM/cmd_vel` and check that
the SAME wheel both steers and spins.

### Only `proposed` runs on hardware

`kanayama`, `li`, `peng` and `muhamed` still build and run in simulation, but
they emit unicycle `(v, ω)` and compute no desired heading, so they cannot
drive the swerve modules. `li` and `muhamed` also need `ref_path` /
`progress`, which the hardware launch no longer provides.
