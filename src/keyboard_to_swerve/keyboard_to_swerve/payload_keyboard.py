"""Keyboard teleop for the PAYLOAD: both robots, all four modules, one twist.

WHY NOT TWO COPIES OF swerve_keyboard
-------------------------------------
Both chassis are rigidly bolted to the payload, so the four modules form ONE
rigid swerve body. Every module must agree on a single instantaneous centre of
rotation (ICR) expressed in the payload frame. Two independent single-robot
teleops each rotate about their own chassis centre -> the wheels fight.

WHAT THIS NODE DOES
-------------------
You command the payload body twist (vx, vy, omega) in the PAYLOAD frame. For
each module k at payload-frame position r_k = (r_x[k], r_y[k]):

    v_k = (vx - omega * r_y[k],  vy + omega * r_x[k])
    speed_k = |v_k|,  heading_k = atan2(v_k)   (payload frame)

That is ICR-consistent by construction. The module positions, agent names and
the wheel-speed envelope `v_max` are read from the `controller` block of the
SAME hardware.yaml the rest of the stack uses (and that hardware_check
validates against each robot's mount + offsets), so there is no second copy of
the geometry to drift.

It publishes exactly what the cooptrans controller publishes:

    /<agent>/cmd_vel          geometry_msgs/Twist   linear.x = wheel speed [m/s]
    /<agent>/desired_heading  std_msgs/Float64      theta_d, WORLD frame

and each robot's own swerve_cooptrans_bridge does the rest: subtracts payload
yaw and mount_angle to get the chassis-frame steer angle, clamps wheel speed,
and -- importantly -- runs a LOCAL watchdog on each NUC. swerve_odrive_bridge
has no command timeout of its own, so driving /robotN/swerveM/cmd_vel directly
from one NUC would leave the other robot's wheels spinning on network loss.
Through the bridge, robot 2 stops within cmd_timeout (0.2 s).

PAYLOAD POSE
------------
The bridge needs <payload>/pose to convert world-frame theta_d. With
`use_external_pose: false` (default, no LiDAR running) this node publishes an
identity pose, i.e. world frame == payload frame. With `use_external_pose:
true` it instead reads the pose published by lidar_pose_adapter and adds its
yaw to theta_d (the bridge subtracts it again). In both cases the commands are
in the payload BODY frame: "w" always means "payload +x".

SAFETY BEHAVIOUR
----------------
- Twist is scaled uniformly (preserving the ICR) so no module exceeds v_max.
- No pi-flip: every module points along its direction of travel and its wheel
  speed is never negative. A direction change of up to 180 deg is done by
  steering (wheels held still meanwhile, see `align_window`), never by
  reversing the wheel.
- Each module's commanded heading slews toward its target at no more than
  `max_heading_rate`, so a key press cannot jump the command 90 deg ahead of a
  servo that turns at ~1.2 rad/s.
- The twist actually sent ramps toward the keyboard twist at `max_accel`
  (per-module velocity vector, m/s^2), so a new direction is reached by
  sweeping each wheel's velocity across, not by reversing its spin. The stop
  key and quitting bypass the ramp.
- While a commanded heading slews, all wheels are held back until every moving
  module is within `align_window` of its target heading (smoothly to full speed
  at 0), so a turning wheel never drives along its old axis.
- All wheel speeds are scaled by ONE factor exp(-gain * e_max), where e_max is
  the largest error between a module's measured heading and its TARGET
  heading (not the slewed command, which the wheel tracks closely; the
  bridge's own attenuation only sees that small lag); above `heading_error_stop` all
  wheels stop. Scaling all modules together keeps the motion rigid while the
  steering slews, instead of letting one module push while another is still
  turning.
- No module feedback, or stale external pose -> zero wheel speed.
- On exit wheels are commanded to zero, then publishing stops; the bridges
  time out and hold steering.

Do NOT run this together with the controller (nuc1.launch.py) -- both publish
/<agent>/cmd_vel.
"""

import math
import os
import select
import sys
import termios
import threading
import tty

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def default_hardware_config():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('swerve_cooptrans_bridge'),
                        'config', 'hardware.yaml')


def load_controller_block(path):
    """Agent names, payload-frame module positions and v_max from hardware.yaml."""
    with open(path) as f:
        cfg = yaml.safe_load(f)
    try:
        params = cfg['controller']['ros__parameters']
    except (KeyError, TypeError):
        raise ValueError(f'{path}: no controller.ros__parameters block')
    names = list(params['robot_names'])
    r_x = [float(v) for v in params['r_x']]
    r_y = [float(v) for v in params['r_y']]
    if not (len(names) == len(r_x) == len(r_y)) or not names:
        raise ValueError(
            f'{path}: controller robot_names/r_x/r_y lengths differ '
            f'({len(names)}, {len(r_x)}, {len(r_y)})')
    payload = str(params.get('payload_name', 'payload'))
    return names, r_x, r_y, float(params['v_max']), payload


HELP = """
Payload keyboard teleop  ({n} modules, payload BODY frame)
------------------------------------------------------------
        q    w    e            w / x : +/- vx     (payload forward)
        a    s    d            a / d : +/- vy     (payload left)
             x                 q / e : +/- omega  (CCW / CW)
                               s, space : stop (steering held)
                               CTRL-C   : quit
step: {lin:.3f} m/s, {ang:.3f} rad/s   limits: |v|<={vmax:.2f} m/s,
|omega|<={wmax:.2f} rad/s   (every module wheel speed <= v_max {vmod:.2f} m/s)
"""


class PayloadTeleop:

    def __init__(self):
        self.node = rclpy.create_node('payload_keyboard')
        n = self.node
        p = n.declare_parameter

        cfg_path = str(p('config', '').value) or default_hardware_config()
        (self.agents, self.r_x, self.r_y, self.v_mod_max,
         payload) = load_controller_block(cfg_path)
        # Optional subset of agents to drive, e.g. agents:="[module1, module2]"
        # to teleop robot 1 alone while robot 2 is offline. Unlisted modules
        # get no commands (their bridge's cmd_timeout stops them if they come
        # up) and are left out of the feedback check and the steering-error
        # gate. Default (unset) = every agent in hardware.yaml.
        only = p('agents', Parameter.Type.STRING_ARRAY).value
        if only:
            unknown = [a for a in only if a not in self.agents]
            if unknown:
                raise ValueError(f'agents {unknown} not in hardware.yaml '
                                 f'robot_names {self.agents}')
            keep = [i for i, a in enumerate(self.agents) if a in only]
            self.agents = [self.agents[i] for i in keep]
            self.r_x = [self.r_x[i] for i in keep]
            self.r_y = [self.r_y[i] for i in keep]
        self.n = len(self.agents)

        self.payload_topic = str(p('payload_topic', f'/{payload}/pose').value)
        self.use_external_pose = bool(p('use_external_pose', False).value)
        self.rate_hz = float(p('rate_hz', 50.0).value)
        self.pose_timeout = float(p('pose_timeout', 0.5).value)
        self.lin_step = float(p('lin_step', 0.01).value)
        self.ang_step = float(p('ang_step', 0.01).value)
        # Same meaning as the bridge's attenuate_on_heading_error /
        # heading_error_stop, but applied to all modules at once.
        self.gain = float(p('heading_error_gain', 7.0).value)
        self.err_stop = float(p('heading_error_stop', 0.5).value)
        # Max slew rate of each module's COMMANDED heading [rad/s]. One key
        # press can swing a module's target by 10+ deg at low speed, i.e.
        # several rad/s under key repeat -- far faster than the servos. Keep
        # this below the REAL steering rate: profile_velocity 250 predicts ~1.2
        # rad/s at the module, but robot1 measured 0.90-0.95 rad/s under load
        # (2026-11-03), so 0.8 keeps a margin and the wheel stays with the
        # command. 0 disables.
        self.max_heading_rate = float(p('max_heading_rate', 0.8).value)
        # Max rate of change of any module's velocity VECTOR [m/s^2]. The key
        # presses set a target twist; the twist actually sent ramps toward it,
        # so a new direction is reached gradually rather than by reversing a
        # wheel. The `s`/space stop and quitting bypass the ramp. 0 disables.
        self.max_accel = float(p('max_accel', 0.15).value)
        # While headings slew, wheel speed is 0 when any module's commanded
        # heading is more than this [rad] from its target, rising smoothly to
        # full speed at 0 (see _step). 0 disables (drive full speed along the
        # slewing heading).
        self.align_window = float(p('align_window', 0.5).value)

        r_max = max(math.hypot(x, y) for x, y in zip(self.r_x, self.r_y))
        self.v_max = self.v_mod_max
        self.w_max = self.v_mod_max / r_max if r_max > 1e-6 else 1.0

        # --- state (guarded by lock: keyboard thread vs. timer) -----------
        self.lock = threading.Lock()
        self.vx = self.vy = self.wz = 0.0    # keyboard target twist
        self.ramp_vx = self.ramp_vy = self.ramp_wz = 0.0   # twist actually sent
        self.head_cmd = [None] * self.n      # payload-frame commanded heading (slewed)
        self.head_tgt = [None] * self.n      # payload-frame target heading (= direction of travel)
        self.last_step_time = None
        self.head_meas = [None] * self.n     # WORLD-frame measured heading
        self.payload_yaw = 0.0
        self.last_pose_time = None
        self.scale_info = (0.0, 1.0)         # (e_max, attenuation) for display
        self.warned = None

        qos = qos_profile_sensor_data
        self.pub_cmd = [n.create_publisher(Twist, f'/{a}/cmd_vel', qos)
                        for a in self.agents]
        self.pub_head = [n.create_publisher(Float64, f'/{a}/desired_heading', qos)
                         for a in self.agents]
        for i, a in enumerate(self.agents):
            n.create_subscription(PoseStamped, f'/{a}/pose',
                                  self._make_meas_cb(i), qos)

        if self.use_external_pose:
            n.create_subscription(PoseStamped, self.payload_topic,
                                  self._on_payload, qos)
            self.pub_pose = None
        else:
            self.pub_pose = n.create_publisher(PoseStamped, self.payload_topic, qos)
        # Re-checked once a second (discovery is not complete at startup).
        self.pose_conflict = False
        self.last_conflict_check = None

        self.timer = n.create_timer(1.0 / self.rate_hz, self._step)

        n.get_logger().info(
            f'payload_keyboard: agents {self.agents}, r_x={self.r_x}, '
            f'r_y={self.r_y}, v_max={self.v_mod_max} (from {cfg_path}); '
            'payload pose: ' + (f'external {self.payload_topic}'
                                if self.use_external_pose
                                else 'identity (published here)'))

    # --- callbacks -------------------------------------------------------

    def _make_meas_cb(self, i):
        def cb(msg):
            with self.lock:
                self.head_meas[i] = yaw_from_quat(msg.pose.orientation)
        return cb

    def _on_payload(self, msg):
        with self.lock:
            self.payload_yaw = yaw_from_quat(msg.pose.orientation)
            self.last_pose_time = self.node.get_clock().now()

    def _warn_once(self, key, text):
        if self.warned != key:
            self.node.get_logger().warn(text)
            self.warned = key

    # --- kinematics ------------------------------------------------------

    def _module_targets(self, vx, vy, wz):
        """Per-module (speed, payload-frame heading); heading None = hold."""
        out = []
        for rx, ry in zip(self.r_x, self.r_y):
            mx, my = vx - wz * ry, vy + wz * rx
            s = math.hypot(mx, my)
            out.append((s, math.atan2(my, mx) if s > 1e-4 else None))
        return out

    def _step(self):
        now = self.node.get_clock().now()
        with self.lock:
            vx, vy, wz = self.vx, self.vy, self.wz
            yaw = self.payload_yaw
            meas = list(self.head_meas)
            pose_age = (float('inf') if self.last_pose_time is None
                        else (now - self.last_pose_time).nanoseconds * 1e-9)

        if self.pub_pose is not None:
            if (self.last_conflict_check is None or
                    (now - self.last_conflict_check).nanoseconds > 1e9):
                self.last_conflict_check = now
                self.pose_conflict = (
                    self.node.count_publishers(self.payload_topic) > 1)
            yaw = 0.0
            m = PoseStamped()
            m.header.stamp = now.to_msg()
            m.header.frame_id = 'world'
            m.pose.orientation.w = 1.0
            self.pub_pose.publish(m)

        # Time step for this tick (timer jitter / stalls capped).
        if self.last_step_time is None:
            dt = 1.0 / self.rate_hz
        else:
            dt = min((now - self.last_step_time).nanoseconds * 1e-9,
                     5.0 / self.rate_hz)
        self.last_step_time = now

        # Ramp the commanded twist toward the keyboard twist. The step is one
        # straight line in twist space, scaled so no module's velocity vector
        # changes faster than max_accel. Module velocities are linear in the
        # twist, so every wheel's velocity vector also moves along a straight
        # line: a change of travel direction sweeps the direction across
        # instead of flipping the wheel's spin, and every module stays on the
        # same ICR throughout.
        # Under the lock: the stop key resets the ramp from the keyboard thread.
        with self.lock:
            vx, vy, wz = self.vx, self.vy, self.wz
            dvx, dvy, dwz = (vx - self.ramp_vx, vy - self.ramp_vy,
                             wz - self.ramp_wz)
            dv_mod = max(math.hypot(dvx - dwz * ry, dvy + dwz * rx)
                         for rx, ry in zip(self.r_x, self.r_y))
            k = 1.0
            if self.max_accel > 0.0 and dv_mod > self.max_accel * dt:
                k = self.max_accel * dt / dv_mod
            self.ramp_vx += k * dvx
            self.ramp_vy += k * dvy
            self.ramp_wz += k * dwz
            rvx, rvy, rwz = self.ramp_vx, self.ramp_vy, self.ramp_wz

        targets = self._module_targets(rvx, rvy, rwz)

        # Uniform saturation: keeps the ICR, unlike per-wheel clamping.
        s_max = max(s for s, _ in targets)
        if s_max > self.v_mod_max:
            k = self.v_mod_max / s_max
            targets = [(s * k, h) for s, h in targets]

        max_step = (self.max_heading_rate * dt if self.max_heading_rate > 0.0
                    else math.inf)

        speeds = []
        for i, (s, h) in enumerate(targets):
            m_pf = wrap(meas[i] - yaw) if meas[i] is not None else None
            if self.head_cmd[i] is None:
                # First tick: start from where the wheel actually points.
                self.head_cmd[i] = m_pf
            if h is None:
                # No motion for this module: hold the heading it already has.
                h = self.head_cmd[i] if self.head_cmd[i] is not None else 0.0
                s = 0.0
            # No pi-flip: the module always points along its direction of
            # travel and the wheel speed is never negative (s = |v_k| >= 0).
            self.head_tgt[i] = h

            # Slew the commanded heading toward the target, shortest way.
            cur = self.head_cmd[i] if self.head_cmd[i] is not None else h
            d = wrap(h - cur)
            self.head_cmd[i] = wrap(cur + max(-max_step, min(max_step, d)))
            speeds.append(s)

        # While a commanded heading is still slewing toward its target, hold
        # the wheels back: speed is 0 if any moving module is more than
        # align_window from its target, rising with cos(gap) to full speed at
        # gap 0. Without this a wheel turning to a new direction keeps
        # rolling along its old axis -- the wrong way -- which is
        # the "direction switch". One factor for all modules (the smallest),
        # like the steering-error gate, so the motion stays rigid. The gap is
        # the LARGER of target vs. slewed command and target vs. MEASURED
        # module heading: the servo can fall behind the command (robot1 steers
        # at ~0.9 rad/s under load), and the wheel must wait for the real
        # wheel, not just for the command.
        align = 1.0
        if self.align_window > 0.0:
            c_w = math.cos(min(self.align_window, math.pi / 2.0))
            for i, s in enumerate(speeds):
                if abs(s) > 1e-9:
                    gap = abs(wrap(self.head_tgt[i] - self.head_cmd[i]))
                    if meas[i] is not None:
                        gap = max(gap, abs(wrap(self.head_tgt[i]
                                                - wrap(meas[i] - yaw))))
                    c = math.cos(gap)
                    align = min(align, max(0.0, min(1.0, (c - c_w) / (1.0 - c_w))))

        # Gate ALL wheels on the worst steering error (rigid-consistent).
        if self.pose_conflict:
            self._warn_once('conflict', f'another node publishes '
                            f'{self.payload_topic} (LiDAR adapter / sim?) -> '
                            'wheels stopped. Stop it, or run with '
                            '-p use_external_pose:=true.')
            att, e_max = 0.0, float('nan')
        elif any(m is None for m in meas):
            self._warn_once('nofb', 'no /<agent>/pose from some module '
                            '(bridge running on both NUCs?) -> wheels stopped.')
            att, e_max = 0.0, float('nan')
        elif self.use_external_pose and pose_age > self.pose_timeout:
            self._warn_once('pose', f'{self.payload_topic} stale -> wheels stopped.')
            att, e_max = 0.0, float('nan')
        else:
            if self.warned in ('nofb', 'pose', 'conflict'):
                self.node.get_logger().info('feedback live.')
                self.warned = None
            # Error against the TARGET heading, not the slewed command: the
            # wheel tracks the slewed command closely, so measuring against it
            # (or relying on the bridge, which only sees the slewed command)
            # would hide how far the wheel still is from where it must point.
            e_max = max(abs(wrap(meas[i] - yaw - self.head_tgt[i]))
                        for i in range(self.n))
            att = 0.0 if e_max >= self.err_stop else math.exp(-self.gain * e_max)
        self.scale_info = (e_max, att * align)   # gain shown = both factors

        for i in range(self.n):
            t = Twist()
            t.linear.x = float(speeds[i] * att * align)
            self.pub_cmd[i].publish(t)
            self.pub_head[i].publish(Float64(data=float(wrap(self.head_cmd[i] + yaw))))

    def publish_stop(self):
        """Zero wheel speed at the last headings (called on exit)."""
        with self.lock:
            self.vx = self.vy = self.wz = 0.0
            self.ramp_vx = self.ramp_vy = self.ramp_wz = 0.0   # no ramp on quit
        for _ in range(5):
            self._step()

    # --- keyboard --------------------------------------------------------

    def _bump(self, attr, step, lim):
        setattr(self, attr, max(-lim, min(lim, getattr(self, attr) + step)))

    def process_key(self, key):
        with self.lock:
            if key == 'w':
                self._bump('vx', self.lin_step, self.v_max)
            elif key == 'x':
                self._bump('vx', -self.lin_step, self.v_max)
            elif key == 'a':
                self._bump('vy', self.lin_step, self.v_max)
            elif key == 'd':
                self._bump('vy', -self.lin_step, self.v_max)
            elif key == 'q':
                self._bump('wz', self.ang_step, self.w_max)
            elif key == 'e':
                self._bump('wz', -self.ang_step, self.w_max)
            elif key in (' ', 's'):
                # Stop is immediate: the ramp is for direction changes, not
                # for the stop key.
                self.vx = self.vy = self.wz = 0.0
                self.ramp_vx = self.ramp_vy = self.ramp_wz = 0.0
            else:
                return False
        return True

    def status_line(self):
        with self.lock:
            vx, vy, wz = self.vx, self.vy, self.wz
        e_max, att = self.scale_info
        return (f'\rvx {vx:+.3f}  vy {vy:+.3f} m/s  omega {wz:+.3f} rad/s  '
                f'| steer err {e_max:.2f} rad  wheel gain {att:.2f}   ')

    def help(self):
        return HELP.format(n=self.n, lin=self.lin_step, ang=self.ang_step,
                           vmax=self.v_max, wmax=self.w_max, vmod=self.v_mod_max)


def main():
    if not sys.stdin.isatty():
        print('payload_keyboard needs an interactive terminal: use `ros2 run`, '
              'not a launch file.', file=sys.stderr)
        return 1

    rclpy.init()
    teleop = PayloadTeleop()
    executor = SingleThreadedExecutor()
    executor.add_node(teleop.node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    settings = termios.tcgetattr(sys.stdin)
    try:
        print(teleop.help())
        tty.setcbreak(sys.stdin.fileno())   # keys unbuffered; CTRL-C -> SIGINT
        last_print = 0.0
        while rclpy.ok():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            changed = False
            if r:
                changed = teleop.process_key(sys.stdin.read(1))
            t = teleop.node.get_clock().now().nanoseconds * 1e-9
            if changed or t - last_print > 0.25:
                sys.stdout.write(teleop.status_line())
                sys.stdout.flush()
                last_print = t
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()
        executor.remove_node(teleop.node)   # stop the timer racing publish_stop
        teleop.publish_stop()
        executor.shutdown()
        teleop.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
