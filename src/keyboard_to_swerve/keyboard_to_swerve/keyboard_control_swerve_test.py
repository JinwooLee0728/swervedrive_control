import os
import select
import sys
import rclpy
import numpy as np

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.02
MAX_LIN_VEL = 2.0
MAX_ANG_VEL = 2.0

# Fallback module geometry, used only when the node is run without parameters.
# The real values come from config/keyboard_to_swerve.yaml (module_names,
# module_offsets_x/y, module_angle_offsets) so one robot's geometry is not
# baked into source.
DEFAULT_MODULE_NAMES = ['swerve1', 'swerve2']
DEFAULT_OFFSETS_X = [0.0, 0.0]
DEFAULT_OFFSETS_Y = [0.23, -0.23]
DEFAULT_ANGLE_OFFSETS = [0.0, 0.0]


def constrain(value, low, high):
    return max(low, min(value, high))


def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class SwerveDrive:
    def __init__(self, node, name, x, y, angle_offset):
        self.name = name
        self.node = node
        self.qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.x = x
        self.y = y
        self.angle_offset = angle_offset
        self.theta = 0.0

        self.target_linear_velocity = 0.0
        self.desired_theta = 0.0

        # Relative names: they resolve under the node's namespace (/robot1, ...).
        # Pairing is by module name, not by slicing the last character off it.
        self.cmd_vel_pub = node.create_publisher(
            TwistStamped, f'{name}/cmd_vel', self.qos
        )

        node.create_subscription(
            Float64,
            f'{name}/heading_angle',
            self._angle_callback,
            self.qos,
        )

    def _angle_callback(self, msg):
        self.theta = wrap_to_pi(msg.data + self.angle_offset)

    def set_target(self, speed, heading):
        # 현재 조향각 기준으로 heading 최적화
        angle_error = wrap_to_pi(heading - self.theta)

        if np.abs(angle_error) > np.pi / 2.0:
            heading = wrap_to_pi(heading + np.pi)
            speed = -speed

        self.target_linear_velocity = speed
        self.desired_theta = heading

    def publish_velocity(self):
        twist = TwistStamped()
        twist.header.stamp = self.node.get_clock().now().to_msg()

        # module command:
        # linear.x  -> wheel speed
        # angular.y -> steering heading
        twist.twist.linear.x = self.target_linear_velocity
        twist.twist.angular.y = self.desired_theta
        twist.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)


class RobotController:
    def __init__(self):
        self.node = rclpy.create_node(
            'keyboard_control_swerve',
            automatically_declare_parameters_from_overrides=True,
        )
        self.qos = QoSProfile(depth=10)

        def param(name, default):
            if self.node.has_parameter(name):
                return self.node.get_parameter(name).value
            return self.node.declare_parameter(name, default).value

        names = list(param('module_names', DEFAULT_MODULE_NAMES))
        ox = list(param('module_offsets_x', DEFAULT_OFFSETS_X))
        oy = list(param('module_offsets_y', DEFAULT_OFFSETS_Y))
        ao = list(param('module_angle_offsets', DEFAULT_ANGLE_OFFSETS))
        if not (len(names) == len(ox) == len(oy) == len(ao)):
            raise ValueError(
                'module_names, module_offsets_x/y and module_angle_offsets '
                f'must be the same length (got {len(names)}, {len(ox)}, '
                f'{len(oy)}, {len(ao)})')

        self.swerve_drives = [
            SwerveDrive(self.node, n, x, y, a)
            for n, x, y, a in zip(names, ox, oy, ao)
        ]

        self.desired_pub = self.node.create_publisher(
            TwistStamped, 'robot/desired/cmd_vel', self.qos
        )

        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        self.theta_steer = 0.0

    def run(self):
        settings = None
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

        try:
            print(self._help_message())
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.01)
                key = self._get_key(settings)
                self._process_key(key)
                self._update_robots()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)
        finally:
            self._stop_all()

    def _get_key(self, settings):
        if os.name == 'nt':
            return msvcrt.getch().decode('utf-8')

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def _process_key(self, key):
        if key == 'w':
            self.control_linear_velocity = constrain(
                self.control_linear_velocity + LIN_VEL_STEP_SIZE,
                -MAX_LIN_VEL, MAX_LIN_VEL,
            )
            self._print_vels()

        elif key == 'x':
            self.control_linear_velocity = constrain(
                self.control_linear_velocity - LIN_VEL_STEP_SIZE,
                -MAX_LIN_VEL, MAX_LIN_VEL,
            )
            self._print_vels()

        elif key == 'a':
            self.control_angular_velocity = constrain(
                self.control_angular_velocity + ANG_VEL_STEP_SIZE,
                -MAX_ANG_VEL, MAX_ANG_VEL,
            )
            self._print_vels()

        elif key == 'd':
            self.control_angular_velocity = constrain(
                self.control_angular_velocity - ANG_VEL_STEP_SIZE,
                -MAX_ANG_VEL, MAX_ANG_VEL,
            )
            self._print_vels()

        elif key == 'q':
            self.theta_steer += 0.01 * np.pi
            self.theta_steer = wrap_to_pi(self.theta_steer)
            self._print_vels()

        elif key == 'e':
            self.theta_steer -= 0.01 * np.pi
            self.theta_steer = wrap_to_pi(self.theta_steer)
            self._print_vels()

        elif key in (' ', 's'):
            self._stop_all()

        elif key == '\x03':
            raise KeyboardInterrupt

    def _update_robots(self):
        v = self.control_linear_velocity
        omega = self.control_angular_velocity
        heading = self.theta_steer

        self._calculate_target_velocities(v, omega, heading)

        for swerve in self.swerve_drives:
            swerve.publish_velocity()

        self._publish_desired(v, omega, heading)

    def _calculate_target_velocities(self, v, omega, heading):
        # pure translation
        if np.abs(omega) < 1e-6:
            for swerve in self.swerve_drives:
                swerve.set_target(v, heading)
            return

        # ICR
        x_icr = -v * np.sin(heading) / omega
        y_icr =  v * np.cos(heading) / omega

        for swerve in self.swerve_drives:
            rx, ry = swerve.x, swerve.y

            vx = -omega * (ry - y_icr)
            vy =  omega * (rx - x_icr)

            speed = np.hypot(vx, vy)
            target_heading = np.arctan2(vy, vx)

            swerve.set_target(speed, target_heading)

    def _publish_desired(self, v, omega, heading):
        desired = TwistStamped()
        desired.header.stamp = self.node.get_clock().now().to_msg()
        desired.twist.linear.x = v
        desired.twist.angular.y = heading
        desired.twist.angular.z = omega
        self.desired_pub.publish(desired)

    def _stop_all(self):
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0

        for swerve in self.swerve_drives:
            swerve.set_target(0.0, swerve.theta)
            swerve.publish_velocity()

        self._print_vels()

    def _print_vels(self):
        print(
            f'\rlin_vel: {self.control_linear_velocity:+.3f}  '
            f'ang_vel: {self.control_angular_velocity:+.3f}  '
            f'heading: {self.theta_steer / np.pi:+.2f}π',
            end='',
        )
        sys.stdout.flush()

    @staticmethod
    def _help_message():
        return """
Control Your 2-Swerve Modules
---------------------------
   q    w    e
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
q/e : increase/decrease heading
space, s : stop
CTRL-C : quit
"""


def main():
    rclpy.init()
    controller = RobotController()
    controller.run()


if __name__ == '__main__':
    main()