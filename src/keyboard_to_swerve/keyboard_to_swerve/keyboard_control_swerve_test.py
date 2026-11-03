import os
import select
import sys
import time
import rclpy
import numpy as np

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy

from keyboard_to_swerve.stepper_serial import StepperSerial

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


def find_stepper_port():
    """The stepper's Arduino Nano (CH340, USB vendor 1a86), by stable by-id name."""
    import glob
    hits = sorted(glob.glob('/dev/serial/by-id/*1a86*'))
    return hits[0] if hits else ''


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

        param = self._param

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
        # Relative, like every other topic here: /dynamixel_lock when run
        # without a namespace, /robotN/dynamixel_lock under one.
        self.lock_pub = self.node.create_publisher(Bool, 'dynamixel_lock', 10)
        self.stepper = self._start_stepper()

        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        self.theta_steer = 0.0
        # Heading actually sent to the modules. It follows theta_steer at no
        # more than max_heading_rate, so the command never runs far ahead of
        # the steering servos. If it did, the modules would end up >90 deg
        # behind and the drive direction would flip (robot reverses).
        self.heading_cmd = 0.0
        # rad/s at the module. Keep below the steering rate set by
        # profile_velocity in swerve_steering.yaml (250 -> ~1.2 rad/s).
        self.max_heading_rate = float(param('max_heading_rate', 1.0))
        # How far theta_steer may run ahead of heading_cmd. Must stay < pi:
        # if the target got more than half a turn ahead, the shortest-path
        # slew would turn the modules backwards while q/e is still held.
        self.max_heading_lead = min(
            float(param('max_heading_lead', 0.5)), np.pi - 0.1)
        self._last_update = time.monotonic()

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
            self._shutdown_stepper()

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
            self._nudge_heading(+0.01 * np.pi)
            self._print_vels()

        elif key == 'e':
            self._nudge_heading(-0.01 * np.pi)
            self._print_vels()

        elif key == 'u':
            if self.stepper is not None:
                self.stepper.jog(-1)

        elif key == 'n':
            if self.stepper is not None:
                self.stepper.jog(+1)

        elif key == 'l':
            self._publish_lock(True)

        elif key == 'o':
            self._publish_lock(False)

        elif key in (' ', 's'):
            self._stop_all()

        elif key == '\x03':
            raise KeyboardInterrupt

    def _update_robots(self):
        v = self.control_linear_velocity
        omega = self.control_angular_velocity
        heading = self._rate_limited_heading()

        self._calculate_target_velocities(v, omega, heading)

        for swerve in self.swerve_drives:
            swerve.publish_velocity()

        self._publish_desired(v, omega, heading)

    def _nudge_heading(self, step):
        """Move the q/e target, but never more than max_heading_lead ahead of heading_cmd."""
        lead = wrap_to_pi(self.theta_steer + step - self.heading_cmd)
        if self.max_heading_rate > 0.0:
            lead = float(np.clip(lead, -self.max_heading_lead, self.max_heading_lead))
        self.theta_steer = wrap_to_pi(self.heading_cmd + lead)

    def _rate_limited_heading(self):
        """Slew heading_cmd toward theta_steer (shortest way) at max_heading_rate."""
        now = time.monotonic()
        # This loop runs at up to ~0.11 s per pass when no key is pressed
        # (spin 0.01 s + key wait 0.1 s); cap above that so a real stall
        # can't produce a jump.
        dt = min(now - self._last_update, 0.25)
        self._last_update = now
        if self.max_heading_rate <= 0.0:
            self.heading_cmd = self.theta_steer
            return self.heading_cmd
        max_step = self.max_heading_rate * dt
        error = wrap_to_pi(self.theta_steer - self.heading_cmd)
        self.heading_cmd = wrap_to_pi(
            self.heading_cmd + float(np.clip(error, -max_step, max_step))
        )
        return self.heading_cmd

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

        if self.stepper is not None:
            self.stepper.stop()

        self._print_vels()

    def _publish_lock(self, locked):
        msg = Bool()
        msg.data = locked
        self.lock_pub.publish(msg)
        print(f"\nlock: {'locked' if locked else 'unlocked'}", flush=True)

    def _param(self, name, default):
        # Overrides passed via --ros-args are auto-declared, so only declare
        # parameters that don't exist yet.
        if self.node.has_parameter(name):
            return self.node.get_parameter(name).value
        return self.node.declare_parameter(name, default).value

    def _start_stepper(self):
        port = self._param('stepper_port', '')
        if port == 'auto':
            port = find_stepper_port()
            if not port:
                print('stepper: auto-detect found no CH340 (1a86) adapter in /dev/serial/by-id')
                return None
        if not port:
            print('stepper disabled. Pass --ros-args -p stepper_port:=/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0')
            return None
        baud = self._param('stepper_baud', 115200)
        steps = self._param('stepper_steps', 400)
        speed = self._param('stepper_speed', 400)
        link = StepperSerial(port, baud, steps, speed)
        link.start()
        print(f'stepper: {port} @ {baud}, jog={steps} steps at {speed} steps/s')
        return link

    def _shutdown_stepper(self):
        if self.stepper is None:
            return
        self.stepper.shutdown()
        self.stepper.join(timeout=5.0)
        self.stepper = None

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
u/n : stepper jog, negative / positive steps
l/o : dynamixel lock / unlock
space, s : stop swerve and stepper
CTRL-C : quit (stepper disabled on exit)
"""


def main():
    rclpy.init()
    controller = RobotController()
    controller.run()


if __name__ == '__main__':
    main()