"""Stand-in for the LiDAR localization node, for desktop simulation.

Takes the plant's ground-truth payload pose and degrades it the way a real
localization system does, so the control loop is exercised against something
imperfect rather than against perfect state:

  * runs at a LiDAR-ish rate (default 20 Hz) rather than the plant's 200 Hz,
    so the controller sees a held, stale pose between updates;
  * adds zero-mean Gaussian noise to position and yaw;
  * optionally adds a fixed transport delay;
  * optionally drops messages, to prove the downstream watchdogs behave.

It publishes the type your real node will publish (`output_type`), so the
`lidar_pose_adapter` under test is the same code path that will run on
hardware. Ground truth stays available on its own topic for the GUI to draw
the true-vs-estimated discrepancy.

    plant --<payload>/truth_pose--> lidar_sim --/lidar/payload_odom-->
        lidar_pose_adapter --<payload>/pose--> controller

Noise is drawn from a seeded RNG so a run is reproducible; change `seed` to
get a different realization.
"""

import math
import random
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

OUTPUT_TYPES = {
    'PoseStamped': PoseStamped,
    'Odometry': Odometry,
    'PoseWithCovarianceStamped': PoseWithCovarianceStamped,
}


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class LidarSim(Node):

    def __init__(self):
        super().__init__('lidar_sim')
        p = self.declare_parameter

        self.truth_topic = str(p('truth_topic', '/payload/truth_pose').value)
        self.output_topic = str(p('output_topic', '/lidar/payload_odom').value)
        self.output_type = str(p('output_type', 'Odometry').value)
        if self.output_type not in OUTPUT_TYPES:
            raise ValueError(f'output_type must be one of '
                             f'{sorted(OUTPUT_TYPES)}, got {self.output_type}')

        self.rate_hz = float(p('rate_hz', 20.0).value)
        self.pos_noise_std = float(p('pos_noise_std', 0.004).value)   # [m]
        self.yaw_noise_std = float(p('yaw_noise_std', 0.004).value)   # [rad]
        self.delay_sec = float(p('delay_sec', 0.0).value)
        self.dropout_prob = float(p('dropout_prob', 0.0).value)
        self.frame_id = str(p('frame_id', 'world').value)
        self.rng = random.Random(int(p('seed', 12345).value))

        self.truth = None          # (x, y, yaw)
        self.prev = None           # for finite-difference twist
        self.prev_t = None
        self.queue = deque()       # delayed samples: (release_time, x, y, yaw)
        self.n_pub = 0
        self.n_drop = 0

        self.create_subscription(PoseStamped, self.truth_topic,
                                 self._on_truth, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            OUTPUT_TYPES[self.output_type], self.output_topic,
            qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f"lidar_sim: '{self.truth_topic}' -> '{self.output_topic}' "
            f"({self.output_type}) at {self.rate_hz:.0f} Hz, "
            f"noise pos={self.pos_noise_std * 1000:.1f} mm / "
            f"yaw={self.yaw_noise_std * 1000:.1f} mrad, "
            f"delay={self.delay_sec * 1000:.0f} ms, "
            f"dropout={self.dropout_prob * 100:.0f}%")

    def _on_truth(self, msg):
        self.truth = (msg.pose.position.x, msg.pose.position.y,
                      yaw_from_quat(msg.pose.orientation))

    def _tick(self):
        if self.truth is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        if self.dropout_prob > 0.0 and self.rng.random() < self.dropout_prob:
            self.n_drop += 1
            return

        x, y, yaw = self.truth
        x += self.rng.gauss(0.0, self.pos_noise_std)
        y += self.rng.gauss(0.0, self.pos_noise_std)
        yaw += self.rng.gauss(0.0, self.yaw_noise_std)

        self.queue.append((now + self.delay_sec, x, y, yaw))
        while self.queue and self.queue[0][0] <= now:
            _, qx, qy, qyaw = self.queue.popleft()
            self._publish(qx, qy, qyaw, now)

    def _publish(self, x, y, yaw, now):
        # Twist by finite difference of the NOISY samples, which is what a real
        # odometry source would report -- deliberately not the clean truth.
        vx = vy = wz = 0.0
        if self.prev is not None and self.prev_t is not None:
            dt = now - self.prev_t
            if dt > 1e-6:
                vx = (x - self.prev[0]) / dt
                vy = (y - self.prev[1]) / dt
                dyaw = (yaw - self.prev[2] + math.pi) % (2 * math.pi) - math.pi
                wz = dyaw / dt
        self.prev, self.prev_t = (x, y, yaw), now

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quat_from_yaw(yaw)

        if self.output_type == 'PoseStamped':
            m = PoseStamped()
            pose = m.pose
        elif self.output_type == 'Odometry':
            m = Odometry()
            pose = m.pose.pose
            m.twist.twist.linear.x = vx
            m.twist.twist.linear.y = vy
            m.twist.twist.angular.z = wz
        else:
            m = PoseWithCovarianceStamped()
            pose = m.pose.pose
            cov = [0.0] * 36
            cov[0] = cov[7] = self.pos_noise_std ** 2
            cov[35] = self.yaw_noise_std ** 2
            m.pose.covariance = cov

        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        pose.position.x, pose.position.y = float(x), float(y)
        pose.orientation.x, pose.orientation.y = qx, qy
        pose.orientation.z, pose.orientation.w = qz, qw
        self.pub.publish(m)
        self.n_pub += 1


def main():
    rclpy.init()
    node = LidarSim()
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
