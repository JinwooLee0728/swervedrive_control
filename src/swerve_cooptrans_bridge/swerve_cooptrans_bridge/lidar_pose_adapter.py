"""Normalise an external localization node's output into the topics the
cooperative-transport stack expects.

The controller subscribes to `<payload>/pose` as geometry_msgs/PoseStamped.
It never had an OptiTrack dependency -- `motive_publisher` was merely one
producer of that topic and `kinematic_plant` another. Swapping in LiDAR-based
localization therefore needs no controller change, only a producer.

What this node absorbs is the MESSAGE TYPE, which varies by localization
package. Set `input_type` to whichever your LiDAR node publishes:

    PoseStamped                 -> pass-through (no twist available)
    Odometry                    -> pose.pose + twist.twist  (SLAM / LiDAR odom)
    PoseWithCovarianceStamped   -> pose.pose  (AMCL-style; no twist)

Outputs:
    <payload>/pose    geometry_msgs/PoseStamped    <- the controller reads this
    <payload>/twist   geometry_msgs/TwistStamped   <- only if the input carries
                                                      twist; viz-only, the
                                                      controller never reads
                                                      measured twist (it uses
                                                      ref_twist)

Covariance is dropped on purpose: nothing downstream consumes it. If you later
want to gate control on localization quality, do it here -- stop republishing
when the covariance trace exceeds a threshold and every watchdog downstream
will stop the robots for you.
"""

import rclpy
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                               TwistStamped)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

INPUT_TYPES = {
    'PoseStamped': PoseStamped,
    'Odometry': Odometry,
    'PoseWithCovarianceStamped': PoseWithCovarianceStamped,
}


class LidarPoseAdapter(Node):

    def __init__(self):
        super().__init__('lidar_pose_adapter')

        p = self.declare_parameter
        self.input_topic = str(p('input_topic', '/lidar/payload_pose').value)
        self.input_type = str(p('input_type', 'PoseStamped').value)
        prefix = str(p('topic_prefix', '').value)
        payload = str(p('payload_name', 'payload').value)
        # Frame stamped on the output. The controller treats <payload>/pose as
        # being in the world frame; if your LiDAR node publishes in a different
        # frame, transform it upstream -- this node does not do TF.
        self.world_frame = str(p('world_frame', 'world').value)
        # Re-stamp outputs with receive time instead of forwarding the input
        # stamp. Useful when the localization node stamps with a clock that is
        # not synced to this machine, since downstream watchdogs compare
        # against local now().
        self.restamp = bool(p('restamp', False).value)

        if self.input_type not in INPUT_TYPES:
            raise ValueError(
                f"input_type must be one of {sorted(INPUT_TYPES)}, "
                f"got '{self.input_type}'")

        qos = qos_profile_sensor_data
        self.pub_pose = self.create_publisher(
            PoseStamped, f'{prefix}{payload}/pose', qos)
        self.pub_twist = self.create_publisher(
            TwistStamped, f'{prefix}{payload}/twist', qos)

        self.create_subscription(
            INPUT_TYPES[self.input_type], self.input_topic, self._on_msg, qos)

        self.count = 0
        self.warned_identity_quat = False
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f"lidar_pose_adapter: {self.input_type} on '{self.input_topic}' "
            f"-> '{prefix}{payload}/pose' (frame '{self.world_frame}'). "
            f"Waiting for the first message.")

    def _report(self):
        if self.count == 0:
            self.get_logger().warn(
                f"no messages yet on '{self.input_topic}' "
                f"(type {self.input_type}) -- the controller will not start "
                f"until <payload>/pose is live. Check the topic name and that "
                f"the publisher's type matches input_type.")

    def _on_msg(self, msg):
        self.count += 1

        if self.input_type == 'PoseStamped':
            pose, header, twist = msg.pose, msg.header, None
        elif self.input_type == 'Odometry':
            pose, header, twist = msg.pose.pose, msg.header, msg.twist.twist
        else:  # PoseWithCovarianceStamped
            pose, header, twist = msg.pose.pose, msg.header, None

        q = pose.orientation
        if (self.count == 1 and not self.warned_identity_quat
                and abs(q.x) + abs(q.y) + abs(q.z) < 1e-12
                and abs(q.w) < 1e-12):
            # All-zero quaternion is not identity -- it yields yaw 0 silently
            # and the payload heading would be wrong everywhere downstream.
            self.get_logger().error(
                'input orientation is an all-zero quaternion, which is not a '
                'valid rotation. Payload yaw will be garbage. Fix the '
                'localization node.')
            self.warned_identity_quat = True

        stamp = (self.get_clock().now().to_msg() if self.restamp
                 else header.stamp)

        out = PoseStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.world_frame
        out.pose = pose
        self.pub_pose.publish(out)

        if twist is not None:
            t = TwistStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.world_frame
            t.twist = twist
            self.pub_twist.publish(t)


def main():
    rclpy.init()
    node = LidarPoseAdapter()
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
