#!/usr/bin/env python3
"""
kinematic_plant : quasi-static cooperative-transport plant for software-in-the-loop
testing. Drop-in replacement for the OptiTrack bridge in offline runs -- it closes
the loop the bridge's offline self-test cannot (the bridge flies preset circles and
ignores commands; this integrates the commands).

Faithful port of main_simul.m's SystemDynamicsDiscrete, two stages per tick:

  1. Propagate each robot's unicycle one dt (RK4) from its CURRENT rigid position
     p_r^[k] = p + R(theta) r^[k], driven by the latest /robotk/cmd_vel (v, w):
         xdot = v cos(theta_r),  ydot = v sin(theta_r),  theta_r_dot = w
  2. Reconstruct the payload pose as the rigid body best fitting the propagated
     robot positions:  min_{x,y,theta} sum_k || p_r^[k] - (p + R(theta) r^[k]) ||^2.
     This is the 2-D Procrustes problem -> closed form (centroid align + atan2), no
     solver. The fit residual is the slip (rigid-body-consistency violation).

State carried: payload pose (x,y,theta) + robot headings. Robot POSITIONS are not
state; each tick they are re-derived from the payload pose (exactly as in MATLAB),
so the published formation is always rigid in position, while headings are the
independently propagated unicycle headings. Internal states (eta,psi,V) are NOT
here -- the controller node owns those.

Publishes on the bridge's own topics so downstream nodes are unaware:
  <prefix><payload>/pose,twist     geometry_msgs/PoseStamped, TwistStamped
  <prefix><robotk>/pose,twist      "
  <prefix>plant/slip               std_msgs/Float64  (RMS rigid-fit residual [m])
Subscribes:
  <prefix><robotk>/cmd_vel         geometry_msgs/Twist  (linear.x = v, angular.z = w)
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_msgs.msg import Float64


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rot2d(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s], [s, c]])


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))   # (x,y,z,w)


def rk4_unicycle(s, v, w, dt):
    """RK4 of [xdot,ydot,thetadot] = [v cos th, v sin th, w] over dt."""
    def f(st):
        return np.array([v * math.cos(st[2]), v * math.sin(st[2]), w])
    k1 = f(s)
    k2 = f(s + 0.5 * dt * k1)
    k3 = f(s + 0.5 * dt * k2)
    k4 = f(s + dt * k3)
    return s + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def procrustes_fit(P, Rm):
    """
    Closed-form 2-D rigid fit: find (p, theta) minimizing sum_k ||P_k - (p + R(theta) Rm_k)||^2.
    P, Rm : (n,2) arrays of robot positions and body-frame offsets.
    Returns x, y, theta, rms_residual.
    """
    pbar = P.mean(axis=0)
    rbar = Rm.mean(axis=0)
    A = P - pbar          # centered robot positions
    B = Rm - rbar         # centered offsets
    C = float(np.sum(A[:, 0] * B[:, 0] + A[:, 1] * B[:, 1]))
    S = float(np.sum(A[:, 1] * B[:, 0] - A[:, 0] * B[:, 1]))
    theta = math.atan2(S, C)
    Rth = rot2d(theta)
    p = pbar - Rth @ rbar
    pred = p + (Rm @ Rth.T)              # predicted robot positions
    resid = P - pred
    rms = math.sqrt(float(np.mean(np.sum(resid ** 2, axis=1))))
    return p[0], p[1], theta, rms


class KinematicPlant(Node):
    def __init__(self):
        super().__init__('kinematic_plant')

        p = self.declare_parameter
        self.prefix       = p('topic_prefix', '').value
        self.payload_name = p('payload_name', 'payload').value
        self.world_frame  = p('world_frame', 'world').value
        self.robot_names  = p('robot_names', ['robot1', 'robot2', 'robot3', 'robot4']).value
        rx = p('r_x', [0.10, 0.10, -0.05, -0.15]).value
        ry = p('r_y', [-0.20, 0.10, 0.15, -0.05]).value
        self.plant_rate   = p('plant_rate', 200.0).value

        # Steering model.
        #   'unicycle' : heading integrates the commanded yaw rate w. Models a
        #                differential-drive robot (the original TurtleBot rig).
        #   'servo'    : heading slews toward the commanded desired heading
        #                <robot>/desired_heading at up to steer_rate_max. Models
        #                a Dynamixel-steered swerve module, including the fact
        #                that the servo has a finite slew rate -- which is what
        #                makes this plant able to expose a profile_velocity set
        #                too low for the ICR-consistent slew demand.
        self.steering_mode = p('steering_mode', 'unicycle').value
        if self.steering_mode not in ('unicycle', 'servo'):
            raise ValueError("steering_mode must be 'unicycle' or 'servo', "
                             f"got '{self.steering_mode}'")
        # Servo slew limit [rad/s]; must equal the controller's w_max.
        self.steer_rate_max = float(p('steer_rate_max', 1.2).value)

        # Fake Dynamixel encoders. When on, the plant also publishes what each
        # module's steering encoder would read, on the same topic the real
        # swerve_steering_node uses:
        #     /<robot_ns>/<hw_name>/heading_angle   (chassis frame)
        # That lets swerve_cooptrans_bridge run IN THE LOOP during simulation
        # -- it closes its heading feedback against these instead of against
        # real hardware, so the bridge under test is the same code that will
        # run on the robots.
        # NOTE: the defaults are non-empty on purpose. rclpy infers a
        # parameter's type from its default, and an empty [] is inferred as
        # BYTE_ARRAY -- which then rejects the string list from the yaml.
        # A one-element list of the right element type pins it correctly; the
        # sentinel is filtered out below.
        self.pub_encoders = bool(p('publish_module_encoders', False).value)
        self.enc_ns = [s for s in p('module_robot_ns', ['']).value if s]
        self.enc_hw = [s for s in p('module_hw_names', ['']).value if s]
        self.enc_alpha = list(p('module_mount_angles', [0.0]).value)
        if not self.enc_ns:
            self.enc_alpha = []

        init_x   = p('init_x', 0.0).value
        init_y   = p('init_y', 0.0).value
        init_th  = p('init_theta', 0.0).value
        init_hd  = p('init_headings', [0.0, 0.0, 0.0, 0.0]).value

        self.n = len(self.robot_names)
        if len(rx) != self.n or len(ry) != self.n:
            raise ValueError('r_x/r_y length must match robot_names')
        self.r = np.column_stack([rx, ry])                      # (n,2) offsets
        self.dt = 1.0 / self.plant_rate

        # state
        self.x_obj, self.y_obj, self.theta_obj = init_x, init_y, init_th
        if len(init_hd) != self.n:
            init_hd = [0.0] * self.n
        self.theta_robots = np.array(init_hd, dtype=float)

        # latest commands (ZOH), default zero -> static until the controller speaks
        self.v_cmd = np.zeros(self.n)
        self.w_cmd = np.zeros(self.n)
        # actually-applied yaw rate (== w_cmd in unicycle mode; the rate-limited
        # servo rate in servo mode). Published as the robot twist so the
        # reported motion is what the plant integrated, not what was asked for.
        self.w_eff = np.zeros(self.n)

        qos = qos_profile_sensor_data
        self.pub_payload_pose  = self.create_publisher(PoseStamped,  f'{self.prefix}{self.payload_name}/pose', qos)
        self.pub_payload_twist = self.create_publisher(TwistStamped, f'{self.prefix}{self.payload_name}/twist', qos)
        self.pub_rpose, self.pub_rtwist = [], []
        for name in self.robot_names:
            self.pub_rpose.append(self.create_publisher(PoseStamped,  f'{self.prefix}{name}/pose', qos))
            self.pub_rtwist.append(self.create_publisher(TwistStamped, f'{self.prefix}{name}/twist', qos))
        self.pub_slip = self.create_publisher(Float64, f'{self.prefix}plant/slip', qos)

        for i, name in enumerate(self.robot_names):
            self.create_subscription(Twist, f'{self.prefix}{name}/cmd_vel', self._cmd_cb(i), qos)

        # fake steering encoders, one per module (see publish_module_encoders)
        self.pub_enc = []
        if self.pub_encoders:
            if not (len(self.enc_ns) == len(self.enc_hw)
                    == len(self.enc_alpha) == self.n):
                raise ValueError(
                    'publish_module_encoders needs module_robot_ns, '
                    'module_hw_names and module_mount_angles all of length '
                    f'{self.n} (got {len(self.enc_ns)}, {len(self.enc_hw)}, '
                    f'{len(self.enc_alpha)})')
            for ns, hw in zip(self.enc_ns, self.enc_hw):
                self.pub_enc.append(self.create_publisher(
                    Float64, f'/{ns}/{hw}/heading_angle', 10))

        # servo mode: desired module heading (world frame) from the controller
        self.th_des = [None] * self.n
        if self.steering_mode == 'servo':
            for i, name in enumerate(self.robot_names):
                self.create_subscription(
                    Float64, f'{self.prefix}{name}/desired_heading',
                    self._thdes_cb(i), qos)

        self.timer = self.create_timer(self.dt, self.step)
        self.get_logger().info(
            f"kinematic_plant: {self.n} robots at {self.plant_rate:.0f} Hz, "
            f"start payload=({init_x:.2f},{init_y:.2f},{init_th:.2f}). Publishing bridge topics.")

    def _cmd_cb(self, i):
        def cb(msg):
            self.v_cmd[i] = msg.linear.x
            self.w_cmd[i] = msg.angular.z
        return cb

    def _thdes_cb(self, i):
        def cb(msg):
            self.th_des[i] = wrap(msg.data)
        return cb

    def step(self):
        dt = self.dt
        # 1. propagate each robot's unicycle from its current rigid position
        P_next = np.zeros((self.n, 2))
        th_next = np.zeros(self.n)
        Rth = rot2d(self.theta_obj)
        for i in range(self.n):
            rp = np.array([self.x_obj, self.y_obj]) + Rth @ self.r[i]

            if self.steering_mode == 'servo':
                # Dynamixel model: slew toward the desired heading, rate-limited.
                # No desired heading yet (or controller stopped) -> hold.
                if self.th_des[i] is None:
                    w_eff = 0.0
                else:
                    err = wrap(self.th_des[i] - self.theta_robots[i])
                    w_eff = max(-self.steer_rate_max,
                                min(self.steer_rate_max, err / dt))
            else:
                w_eff = float(self.w_cmd[i])

            self.w_eff[i] = w_eff
            s0 = np.array([rp[0], rp[1], self.theta_robots[i]])
            s1 = rk4_unicycle(s0, float(self.v_cmd[i]), w_eff, dt)
            P_next[i] = s1[:2]
            th_next[i] = wrap(s1[2])

        # 2. reconstruct payload pose (closed-form Procrustes) + slip residual
        x_new, y_new, th_new, slip = procrustes_fit(P_next, self.r)

        # payload twist via finite difference of the fitted pose
        vx = (x_new - self.x_obj) / dt
        vy = (y_new - self.y_obj) / dt
        wz = wrap(th_new - self.theta_obj) / dt

        # commit state
        self.x_obj, self.y_obj, self.theta_obj = x_new, y_new, th_new
        self.theta_robots = th_next

        self._publish(vx, vy, wz, slip)

    def _publish(self, vx, vy, wz, slip):
        stamp = self.get_clock().now().to_msg()

        def pose_msg(x, y, yaw):
            m = PoseStamped(); m.header.stamp = stamp; m.header.frame_id = self.world_frame
            m.pose.position.x, m.pose.position.y = float(x), float(y)
            qx, qy, qz, qw = quat_from_yaw(yaw)
            m.pose.orientation.x, m.pose.orientation.y = qx, qy
            m.pose.orientation.z, m.pose.orientation.w = qz, qw
            return m

        def twist_msg(lx, ly, az):
            m = TwistStamped(); m.header.stamp = stamp; m.header.frame_id = self.world_frame
            m.twist.linear.x, m.twist.linear.y = float(lx), float(ly)
            m.twist.angular.z = float(az)
            return m

        self.pub_payload_pose.publish(pose_msg(self.x_obj, self.y_obj, self.theta_obj))
        self.pub_payload_twist.publish(twist_msg(vx, vy, wz))

        Rth = rot2d(self.theta_obj)
        for i in range(self.n):
            rp = np.array([self.x_obj, self.y_obj]) + Rth @ self.r[i]     # rigid position from new pose
            self.pub_rpose[i].publish(pose_msg(rp[0], rp[1], self.theta_robots[i]))
            # robot world-frame twist from its commanded (v, w)
            vth = self.theta_robots[i]
            self.pub_rtwist[i].publish(
                twist_msg(self.v_cmd[i] * math.cos(vth), self.v_cmd[i] * math.sin(vth), self.w_eff[i]))

        s = Float64(); s.data = float(slip); self.pub_slip.publish(s)

        # Fake steering encoders: what each module's Dynamixel would report.
        # The encoder reads in the CHASSIS frame, and the chassis is bolted to
        # the payload, so phi = theta_module_world - theta_payload - alpha.
        for i, pub in enumerate(self.pub_enc):
            m = Float64()
            m.data = float(wrap(self.theta_robots[i] - self.theta_obj
                                - self.enc_alpha[i]))
            pub.publish(m)


def main():
    rclpy.init()
    node = KinematicPlant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()