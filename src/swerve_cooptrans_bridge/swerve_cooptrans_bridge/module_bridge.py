"""Adapter between the cooptrans controller and one robot's swerve hardware.

WHY THIS NODE EXISTS
--------------------
The cooperative-transport controller models each agent as a nonholonomic
unicycle rigidly attached to the payload. A steered swerve module IS exactly
such a unicycle -- its contact point rolls only along the wheel direction and
the steer angle is directly actuated -- so we register each MODULE, not each
robot, as an agent.

That identification only holds because the chassis is RIGIDLY BOLTED to the
payload. If the chassis could yaw freely, a module's payload-frame offset
r^[k] would be time-varying and the controller's constant `r_x`/`r_y` would be
wrong. Do not run this against a pinned/free-yaw mount.

This node runs once per robot and owns that robot's two modules.

    DOWN (command):  <agent>/desired_heading  std_msgs/Float64
                       theta_d in the WORLD frame (paper eq. 8)
                     <agent>/cmd_vel  geometry_msgs/Twist
                       linear.x = wheel speed [m/s]  (already scaled by mu)
                     ->  <module>/cmd_vel  geometry_msgs/TwistStamped
                       linear.x  = wheel speed [m/s]
                       angular.y = steer ANGLE [rad], CHASSIS frame

    UP (state):      <module>/heading_angle  std_msgs/Float64 (chassis frame)
                     + <payload>/pose
                     ->  <agent>/pose  geometry_msgs/PoseStamped
                       yaw = theta_payload + alpha_k + phi_k
                     which is the per-agent pose topic the controller already
                     subscribes to.

DIRECT STEERING
---------------
The controller publishes theta_d (eq. 8) directly, so this node does NOT
integrate the yaw-rate command. The steering angle is a pure frame conversion:

    phi[k] = wrap(theta_d[k] - theta_payload - alpha_k)

The paper's low-level block -- heading regulation (eq. 22) and velocity
attenuation (eq. 23) -- exists only because a differential-drive robot cannot
set its heading directly. A Dynamixel can, and Proposition 2 guarantees strict
kinematic feasibility at e_theta == 0, which this achieves by construction.

One piece of eq. (23) IS kept, locally: if the measured module heading lags
theta_d (a servo that cannot keep up, e.g. profile_velocity set too low, or a
stalled module), the wheel speed is attenuated. Without this, a lagging module
drives in the wrong direction and injects exactly the slip the controller is
designed to avoid. See `attenuate_on_heading_error`.

NOTE ON THE PI-FLIP: `rigidBody()` in proposed.cpp already picks the nearer of
the two ICR-perpendicular headings. Do NOT re-apply the keyboard node's flip
here -- a second flip fights the controller.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


def wrap(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class ModuleBridge(Node):

    def __init__(self):
        super().__init__('swerve_cooptrans_bridge')

        p = self.declare_parameter

        # Agent names as the CONTROLLER knows them. Must match entries in
        # `robot_names` in the controller's params, in the same order as this
        # robot's physical modules. Deliberately resolved as GLOBAL names
        # because the controller runs in the root namespace -- see
        # _agent_topic().
        self.agent_names = list(p('agent_names', ['module1', 'module2']).value)
        # Module base names as the HARDWARE knows them, relative to this
        # robot's namespace. Must match `module_names` in swerve_steering.yaml.
        self.module_names = list(p('module_names', ['swerve1', 'swerve2']).value)

        # Module positions in the CHASSIS frame [m].
        self.off_x = list(p('module_offsets_x', [0.0, 0.0]).value)
        self.off_y = list(p('module_offsets_y', [0.23, -0.23]).value)
        # alpha: this chassis's orientation in the PAYLOAD frame [rad].
        self.alpha = float(p('mount_angle', 0.0).value)
        # This chassis's origin in the PAYLOAD frame [m]. Used only to place
        # the module positions we publish for visualization; the controller
        # reads yaw only, so these do not affect control.
        self.mount_x = float(p('mount_x', 0.0).value)
        self.mount_y = float(p('mount_y', 0.0).value)

        self.payload_topic = str(p('payload_topic', '/payload/pose').value)
        self.rate_hz = float(p('rate_hz', 100.0).value)
        # Independent of the controller's own watchdog: this one also covers
        # NUC<->NUC network loss, which the controller cannot observe.
        self.cmd_timeout = float(p('cmd_timeout', 0.2).value)
        self.pose_timeout = float(p('pose_timeout', 0.5).value)
        # Safety clamps applied to whatever the controller sends.
        self.wheel_speed_max = float(p('wheel_speed_max', 0.5).value)

        # Local remnant of eq. (23): attenuate wheel speed while the measured
        # module heading lags theta_d. 0.0 disables. The controller no longer
        # does this (direct steering assumes e_theta ~= 0), so this is the only
        # thing standing between a too-slow servo and commanded slip.
        self.gamma_v = float(p('attenuate_on_heading_error', 7.0).value)
        # Above this heading error the wheel is stopped outright [rad].
        self.heading_error_stop = float(p('heading_error_stop', 1.2).value)

        # On hardware this node is the only source of <agent>/pose, so it must
        # publish it. In SIMULATION the kinematic plant owns the true module
        # state and publishes that topic itself; two publishers would fight.
        # Setting this false lets the bridge still run in the loop (doing the
        # theta_d -> steer-angle conversion that the GUI and the simulated
        # hardware consume) without claiming pose ownership.
        self.publish_module_pose = bool(p('publish_module_pose', True).value)

        n = len(self.agent_names)
        if not (len(self.module_names) == len(self.off_x)
                == len(self.off_y) == n):
            raise ValueError(
                'agent_names, module_names, module_offsets_x and '
                'module_offsets_y must all be the same length '
                f'(got {n}, {len(self.module_names)}, {len(self.off_x)}, '
                f'{len(self.off_y)})')
        self.n = n
        self.dt = 1.0 / self.rate_hz

        # --- state -------------------------------------------------------
        self.v_cmd = [0.0] * n
        self.th_des = [None] * n           # world-frame desired heading
        self.phi_meas = [None] * n         # chassis-frame measured steer angle
        self.phi_cmd = [None] * n          # last commanded steer angle (hold on timeout)
        self.payload_xy = (0.0, 0.0)
        self.payload_yaw = None
        self.last_cmd_time = [None] * n
        self.last_thdes_time = [None] * n
        self.last_pose_time = None
        self.stale_warned = False
        self.lag_warned = False

        qos = qos_profile_sensor_data

        self.create_subscription(
            PoseStamped, self.payload_topic, self._on_payload, qos)

        self.pub_steer, self.pub_pose = [], []
        for i in range(n):
            mod, agent = self.module_names[i], self.agent_names[i]

            # from the controller (global: it runs in the root namespace)
            self.create_subscription(
                Twist, self._agent_topic(agent, 'cmd_vel'),
                self._make_cmd_cb(i), qos)
            self.create_subscription(
                Float64, self._agent_topic(agent, 'desired_heading'),
                self._make_thdes_cb(i), qos)
            # back to the controller (unless the sim plant owns this topic)
            self.pub_pose.append(
                self.create_publisher(
                    PoseStamped, self._agent_topic(agent, 'pose'), qos)
                if self.publish_module_pose else None)

            # to/from this robot's hardware (relative: lives in our namespace)
            self.pub_steer.append(self.create_publisher(
                TwistStamped, f'{mod}/cmd_vel', 10))
            self.create_subscription(
                Float64, f'{mod}/heading_angle',
                self._make_heading_cb(i), 10)

        self.timer = self.create_timer(self.dt, self.step)

        self.get_logger().info(
            f'swerve_cooptrans_bridge: {n} modules '
            f'{list(zip(self.agent_names, self.module_names))}, '
            f'alpha={self.alpha:.4f} rad, payload={self.payload_topic}, '
            f'{self.rate_hz:.0f} Hz, direct steering from <agent>/desired_heading. '
            f'Waiting for payload pose and steering feedback.')

    @staticmethod
    def _agent_topic(agent, leaf):
        """Controller-side topic name.

        The controller publishes /<agent>/... in the ROOT namespace, while this
        node runs under /robotN. A relative name would resolve to
        /robotN/<agent>/... and never connect, so these are absolute.
        """
        return f'/{agent}/{leaf}'

    # --- callbacks -------------------------------------------------------

    def _on_payload(self, msg):
        self.payload_xy = (msg.pose.position.x, msg.pose.position.y)
        self.payload_yaw = yaw_from_quat(msg.pose.orientation)
        self.last_pose_time = self.get_clock().now()

    def _make_cmd_cb(self, i):
        def cb(msg):
            self.v_cmd[i] = msg.linear.x
            self.last_cmd_time[i] = self.get_clock().now()
        return cb

    def _make_thdes_cb(self, i):
        def cb(msg):
            self.th_des[i] = wrap(msg.data)
            self.last_thdes_time[i] = self.get_clock().now()
        return cb

    def _make_heading_cb(self, i):
        def cb(msg):
            self.phi_meas[i] = wrap(msg.data)
        return cb

    # --- helpers ---------------------------------------------------------

    def _age(self, t):
        if t is None:
            return float('inf')
        return (self.get_clock().now() - t).nanoseconds * 1e-9

    def _ready(self):
        return (self.payload_yaw is not None
                and self._age(self.last_pose_time) <= self.pose_timeout
                and all(v is not None for v in self.phi_meas))

    @staticmethod
    def _clamp(x, lim):
        return max(-lim, min(lim, x))

    # --- main loop -------------------------------------------------------

    def step(self):
        if not self._ready():
            if not self.stale_warned:
                self.get_logger().warn(
                    'payload pose or steering feedback missing/stale -> '
                    'holding steering, wheels stopped.')
                self.stale_warned = True
            self._publish_hold()
            return

        if self.stale_warned:
            self.get_logger().info('inputs live; bridging.')
            self.stale_warned = False

        lagging = 0.0
        for i in range(self.n):
            thdes_stale = self._age(self.last_thdes_time[i]) > self.cmd_timeout
            cmd_stale = self._age(self.last_cmd_time[i]) > self.cmd_timeout

            # The controller stops publishing desired_heading when it commands
            # a stop (see publishStop in controller_node.cpp), so a stale
            # heading means "hold where you are", never "slew to zero".
            if thdes_stale or self.th_des[i] is None:
                phi = self.phi_cmd[i]
                if phi is None:
                    phi = self.phi_meas[i]
                v = 0.0
            else:
                phi = wrap(self.th_des[i] - self.payload_yaw - self.alpha)
                self.phi_cmd[i] = phi
                v = 0.0 if cmd_stale else self._clamp(
                    self.v_cmd[i], self.wheel_speed_max)

                # Local eq. (23): the module is supposed to already be at
                # theta_d. If it is not, it is pointing the wrong way and any
                # wheel speed becomes commanded slip -- so bleed it off.
                e_th = abs(wrap(self.phi_meas[i] - phi))
                lagging = max(lagging, e_th)
                if e_th >= self.heading_error_stop:
                    v = 0.0
                elif self.gamma_v > 0.0:
                    v *= math.exp(-self.gamma_v * e_th)

            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.module_names[i]
            m.twist.linear.x = float(v)      # wheel speed [m/s]
            m.twist.angular.y = float(phi)   # steer angle [rad], chassis frame
            self.pub_steer[i].publish(m)

        # A persistent lag means the servo cannot meet the ICR-consistent slew
        # demand -- almost always profile_velocity set below the controller's
        # w_max. Worth saying out loud; it silently degrades tracking.
        if lagging >= self.heading_error_stop:
            if not self.lag_warned:
                self.get_logger().warn(
                    f'module heading lags desired by {lagging:.3f} rad '
                    f'(>= {self.heading_error_stop:.2f}) -> wheels stopped. '
                    'Steering servo cannot keep up: check profile_velocity '
                    'against the controller w_max.')
                self.lag_warned = True
        else:
            self.lag_warned = False

        self._publish_module_poses()

    def _publish_hold(self):
        """Wheels to zero, steering held wherever it was last commanded."""
        stamp = self.get_clock().now().to_msg()
        for i in range(self.n):
            phi = self.phi_cmd[i]
            if phi is None:
                phi = self.phi_meas[i]
            if phi is None:
                continue
            m = TwistStamped()
            m.header.stamp = stamp
            m.header.frame_id = self.module_names[i]
            m.twist.linear.x = 0.0
            m.twist.angular.y = float(phi)
            self.pub_steer[i].publish(m)
        self._publish_module_poses()

    def _publish_module_poses(self):
        """Publish each module as an agent pose for the controller.

        Only the yaw is read by the controller; the position is filled in from
        the rigid mount geometry so cooptrans_viz draws the modules in the
        right place.
        """
        if self.payload_yaw is None or not self.publish_module_pose:
            return
        stamp = self.get_clock().now().to_msg()
        c, s = math.cos(self.payload_yaw), math.sin(self.payload_yaw)
        ca, sa = math.cos(self.alpha), math.sin(self.alpha)
        px, py = self.payload_xy

        for i in range(self.n):
            if self.phi_meas[i] is None:
                continue

            # module position: chassis frame -> payload frame -> world
            mx = self.mount_x + ca * self.off_x[i] - sa * self.off_y[i]
            my = self.mount_y + sa * self.off_x[i] + ca * self.off_y[i]
            wx = px + c * mx - s * my
            wy = py + s * mx + c * my

            yaw = wrap(self.payload_yaw + self.alpha + self.phi_meas[i])

            msg = PoseStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = 'world'
            msg.pose.position.x = float(wx)
            msg.pose.position.y = float(wy)
            qx, qy, qz, qw = quat_from_yaw(yaw)
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
            self.pub_pose[i].publish(msg)


def main():
    rclpy.init()
    node = ModuleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
