"""Stand-in for the external receding-horizon planner, for desktop simulation.

The real system has a separate node that replans from the payload's current
state and publishes a reference. This fakes it so the whole pipeline can be
exercised without that node existing yet, and so both integration paths can be
tested:

  mode: "path"    (default, matches `use_progression_regulator:=true`)
      Publishes a finite HORIZON of the geometric path as nav_msgs/Path on
      <payload>/planned_path, replanned every `replan_period` seconds starting
      from the payload's current progress. trajectory_generator ingests it,
      re-fits the spline, and applies the paper's progression-rate regulator
      (eq. 13). This is the preferred real integration.

  mode: "direct"  (matches `use_progression_regulator:=false`)
      Publishes <payload>/ref_pose, ref_twist and ref_accel itself at
      `rate_hz`, bypassing trajectory_generator entirely. Use this to test the
      bypass path. NOTE eq. 13 re-timing does not exist in this mode -- the
      reference advances on wall-clock regardless of tracking error, which is
      exactly the limitation of publishing setpoints instead of a path.

The underlying geometric path is a closed Lissajous-style loop with a smoothly
varying heading, chosen because it exercises both curvature reversal (the ICR
crossing from one side of the payload to the other, which is the hard case for
the eta/psi parameterization) and sustained rotation.
"""

import math

import rclpy
from geometry_msgs.msg import (AccelStamped, PoseStamped, TwistStamped)
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, qos_profile_sensor_data)


def quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimPlanner(Node):

    def __init__(self):
        super().__init__('sim_planner')
        p = self.declare_parameter

        self.mode = str(p('mode', 'path').value)
        if self.mode not in ('path', 'direct'):
            raise ValueError("mode must be 'path' or 'direct', "
                             f"got '{self.mode}'")

        prefix = str(p('topic_prefix', '').value)
        payload = str(p('payload_name', 'payload').value)
        self.world_frame = str(p('world_frame', 'world').value)

        # --- path shape -----------------------------------------------------
        self.ax = float(p('amp_x', 1.6).value)       # [m]
        self.ay = float(p('amp_y', 0.9).value)       # [m]
        self.cx = float(p('center_x', 0.0).value)
        self.cy = float(p('center_y', 0.0).value)
        self.amp_theta = float(p('amp_theta', 0.5).value)   # [rad]
        self.loop_time = float(p('loop_time', 60.0).value)  # [s] one full lap

        # --- horizon / replanning (path mode) -------------------------------
        self.horizon_time = float(p('horizon_time', 12.0).value)  # [s] ahead
        self.horizon_points = int(p('horizon_points', 60).value)
        self.replan_period = float(p('replan_period', 1.0).value)

        self.rate_hz = float(p('rate_hz', 100.0).value)
        # Seconds of lead-in during which the reference holds still, so the
        # controller can converge before the payload is asked to move.
        self.settle_time = float(p('settle_time', 2.0).value)

        qos = qos_profile_sensor_data
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        base = f'{prefix}{payload}'
        self.t = 0.0
        self.have_pose = False

        if self.mode == 'path':
            self.pub_path = self.create_publisher(
                Path, f'{base}/planned_path', latched)
            # Replanning is "receding horizon" in the honest sense: each replan
            # starts from where the payload actually is, not from where the
            # previous plan said it would be.
            self.create_subscription(
                PoseStamped, f'{base}/pose', self._on_pose, qos)
            self.create_timer(self.replan_period, self._replan)
            self.s_now = 0.0
        else:
            self.pub_pose = self.create_publisher(
                PoseStamped, f'{base}/ref_pose', qos)
            self.pub_twist = self.create_publisher(
                TwistStamped, f'{base}/ref_twist', qos)
            self.pub_accel = self.create_publisher(
                AccelStamped, f'{base}/ref_accel', qos)
            self.create_timer(1.0 / self.rate_hz, self._tick_direct)

        self.get_logger().info(
            f"sim_planner [{self.mode}]: lissajous "
            f"({self.ax:.2f} x {self.ay:.2f} m, {self.loop_time:.0f} s/lap, "
            f"heading +/-{self.amp_theta:.2f} rad)"
            + (f", horizon {self.horizon_time:.0f} s / {self.horizon_points} pts, "
               f"replan every {self.replan_period:.1f} s"
               if self.mode == 'path' else
               f", publishing ref_* directly at {self.rate_hz:.0f} Hz "
               f"(eq. 13 re-timing NOT present in this mode)"))

    # --- the reference curve, parameterized by time ----------------------

    def _q(self, t):
        """Reference pose [x, y, theta] at planner-time t."""
        w = 2.0 * math.pi / self.loop_time
        x = self.cx + self.ax * math.sin(w * t)
        y = self.cy + self.ay * math.sin(2.0 * w * t)
        th = self.amp_theta * math.sin(w * t)
        return x, y, th

    def _dq(self, t):
        w = 2.0 * math.pi / self.loop_time
        return (self.ax * w * math.cos(w * t),
                self.ay * 2.0 * w * math.cos(2.0 * w * t),
                self.amp_theta * w * math.cos(w * t))

    def _ddq(self, t):
        w = 2.0 * math.pi / self.loop_time
        return (-self.ax * w * w * math.sin(w * t),
                -self.ay * 4.0 * w * w * math.sin(2.0 * w * t),
                -self.amp_theta * w * w * math.sin(w * t))

    # --- path mode -------------------------------------------------------

    def _on_pose(self, msg):
        """Track the payload so each replan starts from where it really is."""
        self.have_pose = True
        px, py = msg.pose.position.x, msg.pose.position.y
        # Walk a coarse sampling of the loop and keep the closest point. The
        # loop is short and this runs at replan_period, so brute force is fine
        # and avoids the local-minimum trap of gradient descent on a lissajous.
        best, best_d = self.s_now, float('inf')
        n = 400
        for i in range(n):
            t = self.loop_time * i / n
            x, y, _ = self._q(t)
            d = (x - px) ** 2 + (y - py) ** 2
            if d < best_d:
                best_d, best = d, t
        self.s_now = best

    def _replan(self):
        if not self.have_pose:
            return
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        n = max(2, self.horizon_points)
        for i in range(n):
            t = self.s_now + self.horizon_time * i / (n - 1)
            x, y, th = self._q(t)
            ps = PoseStamped()
            ps.header.frame_id = self.world_frame
            ps.pose.position.x, ps.pose.position.y = x, y
            qx, qy, qz, qw = quat_from_yaw(th)
            ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
            ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    # --- direct mode -----------------------------------------------------

    def _tick_direct(self):
        dt = 1.0 / self.rate_hz
        if self.t < self.settle_time:
            self.t += dt
            x, y, th = self._q(0.0)
            self._publish_direct((x, y, th), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            return
        tt = self.t - self.settle_time
        self._publish_direct(self._q(tt), self._dq(tt), self._ddq(tt))
        self.t += dt

    def _publish_direct(self, q, dq, ddq):
        stamp = self.get_clock().now().to_msg()

        m = PoseStamped()
        m.header.stamp = stamp
        m.header.frame_id = self.world_frame
        m.pose.position.x, m.pose.position.y = float(q[0]), float(q[1])
        qx, qy, qz, qw = quat_from_yaw(q[2])
        m.pose.orientation.x, m.pose.orientation.y = qx, qy
        m.pose.orientation.z, m.pose.orientation.w = qz, qw
        self.pub_pose.publish(m)

        t = TwistStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.world_frame
        t.twist.linear.x, t.twist.linear.y = float(dq[0]), float(dq[1])
        t.twist.angular.z = float(dq[2])
        self.pub_twist.publish(t)

        a = AccelStamped()
        a.header.stamp = stamp
        a.header.frame_id = self.world_frame
        a.accel.linear.x, a.accel.linear.y = float(ddq[0]), float(ddq[1])
        a.accel.angular.z = float(ddq[2])
        self.pub_accel.publish(a)


def main():
    rclpy.init()
    node = SimPlanner()
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
