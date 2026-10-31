"""Validate hardware.yaml before anything is allowed to move.

hardware.yaml is the single file an operator edits, but ROS 2 parameter files
are keyed by node name and cannot alias values across blocks. So a handful of
physical facts necessarily appear more than once: the module positions exist
both as the controller's r^[k] and as each bridge's mount+offset description;
the steering gear ratio appears in both the steering node and the ODrive
bridge; the controller's w_max is a function of the Dynamixel's
profile_velocity.

A silent mismatch in any of those corrupts control with no error message --
the controller would compute an ICR for a geometry the robots do not have.
This node re-derives every one of them and EXITS NON-ZERO on disagreement, so
the launch dies loudly instead of driving a payload into a wall.

It reads hardware.yaml directly rather than querying live nodes, so it can run
first, before anything else starts, and cannot drift from what the nodes load.

    ros2 run swerve_cooptrans_bridge hardware_check --ros-args \
        -p config_file:=/path/to/hardware.yaml
"""

import math
import os
import sys

import yaml

DYN_VEL_UNIT_RPM = 0.229   # Dynamixel X-series Profile Velocity unit


def _params(doc, key):
    """Return the ros__parameters mapping for a node block, or {}."""
    blk = doc.get(key)
    if not isinstance(blk, dict):
        return {}
    return blk.get('ros__parameters', {}) or {}


def steering_rate_from_dynamixel(profile_velocity, gear_ratio):
    """Max module steering rate [rad/s] implied by the servo profile."""
    motor_rev_per_s = profile_velocity * DYN_VEL_UNIT_RPM / 60.0
    return motor_rev_per_s * 2.0 * math.pi / gear_ratio


class Checker:
    def __init__(self, doc, geom_tol, rate_tol, wheel_warn):
        self.doc = doc
        self.geom_tol = geom_tol
        self.rate_tol = rate_tol
        self.wheel_warn = wheel_warn
        self.errors = []
        self.warnings = []

    def err(self, msg):
        self.errors.append(msg)

    def warn(self, msg):
        self.warnings.append(msg)

    # -- individual checks ------------------------------------------------

    def check_controller_sizes(self, ctrl):
        names = ctrl.get('robot_names', [])
        rx, ry = ctrl.get('r_x', []), ctrl.get('r_y', [])
        if not (len(names) == len(rx) == len(ry)):
            self.err(f'controller: robot_names/r_x/r_y lengths differ '
                     f'({len(names)}/{len(rx)}/{len(ry)}). The controller '
                     f'fatals on this, but fail here first.')
            return False
        if not names:
            self.err('controller: robot_names is empty.')
            return False
        return True

    def check_geometry(self, ctrl, robot_blocks):
        """r^[k] == mount + R(alpha) * module_offset, for every agent."""
        names = list(ctrl.get('robot_names', []))
        rx, ry = ctrl.get('r_x', []), ctrl.get('r_y', [])
        seen = {}

        for ns in robot_blocks:
            br = _params(self.doc, f'{ns}/swerve_cooptrans_bridge')
            if not br:
                self.err(f'{ns}: no swerve_cooptrans_bridge block in config.')
                continue
            agents = list(br.get('agent_names', []))
            ox = br.get('module_offsets_x', [])
            oy = br.get('module_offsets_y', [])
            mx = float(br.get('mount_x', 0.0))
            my = float(br.get('mount_y', 0.0))
            a = float(br.get('mount_angle', 0.0))
            ca, sa = math.cos(a), math.sin(a)

            if not (len(agents) == len(ox) == len(oy)):
                self.err(f'{ns}: agent_names/module_offsets_x/y lengths differ '
                         f'({len(agents)}/{len(ox)}/{len(oy)}).')
                continue

            for j, agent in enumerate(agents):
                if agent in seen:
                    self.err(f"agent '{agent}' is claimed by both {seen[agent]} "
                             f'and {ns}. Each module must have exactly one owner.')
                    continue
                seen[agent] = ns

                if agent not in names:
                    self.err(f"{ns}: agent '{agent}' is not in the controller's "
                             f'robot_names {names}.')
                    continue
                k = names.index(agent)

                exp_x = mx + ca * ox[j] - sa * oy[j]
                exp_y = my + sa * ox[j] + ca * oy[j]
                dx, dy = exp_x - rx[k], exp_y - ry[k]
                if math.hypot(dx, dy) > self.geom_tol:
                    self.err(
                        f"geometry mismatch for '{agent}': controller has "
                        f'r=({rx[k]:+.4f}, {ry[k]:+.4f}) but {ns} mount+offset '
                        f'gives ({exp_x:+.4f}, {exp_y:+.4f}), off by '
                        f'{math.hypot(dx, dy):.4f} m. Fix r_x/r_y or the '
                        f'mount_*/module_offsets_* in hardware.yaml.')

        for agent in names:
            if agent not in seen:
                self.err(f"agent '{agent}' is in the controller's robot_names "
                         f'but no robot block owns it. It would never receive '
                         f'commands and the controller would never see its pose.')

    def check_steering_rate(self, ctrl, robot_blocks):
        """controller w_max == the rate the Dynamixel profile actually allows."""
        w_max = ctrl.get('w_max')
        if w_max is None:
            self.err('controller: w_max is not set.')
            return
        for ns in robot_blocks:
            st = _params(self.doc, f'{ns}/swerve_steering')
            if not st:
                self.err(f'{ns}: no swerve_steering block in config.')
                continue
            pv = st.get('profile_velocity')
            gr = st.get('gear_ratio')
            if pv is None or gr is None:
                self.err(f'{ns}/swerve_steering: profile_velocity or '
                         f'gear_ratio missing.')
                continue
            if gr <= 0:
                self.err(f'{ns}/swerve_steering: gear_ratio must be > 0.')
                continue
            derived = steering_rate_from_dynamixel(pv, gr)
            if abs(derived - w_max) > self.rate_tol:
                need = w_max * 60.0 * gr / (DYN_VEL_UNIT_RPM * 2.0 * math.pi)
                self.err(
                    f'{ns}: controller w_max={w_max:.4f} rad/s but '
                    f'profile_velocity={pv} with gear_ratio={gr} delivers only '
                    f'{derived:.4f} rad/s. The actuation envelope would not '
                    f'bound the real actuator. Set profile_velocity={need:.1f} '
                    f'or lower w_max to {derived:.4f}.')

    def check_gear_ratio_agreement(self, robot_blocks):
        for ns in robot_blocks:
            st = _params(self.doc, f'{ns}/swerve_steering')
            od = _params(self.doc, f'{ns}/swerve_odrive_bridge')
            a, b = st.get('gear_ratio'), od.get('steering_gear_ratio')
            if a is None or b is None:
                continue
            if abs(float(a) - float(b)) > 1e-9:
                self.err(
                    f'{ns}: swerve_steering.gear_ratio={a} != '
                    f'swerve_odrive_bridge.steering_gear_ratio={b}. The '
                    f'steering-drive coupling compensation would be scaled '
                    f'wrong and the wheels would creep while steering.')

    def check_bridge_limits(self, ctrl, robot_blocks):
        v_max = ctrl.get('v_max')
        for ns in robot_blocks:
            br = _params(self.doc, f'{ns}/swerve_cooptrans_bridge')
            if not br:
                continue
            ws = br.get('wheel_speed_max')
            if v_max is not None and ws is not None and ws < v_max - 1e-9:
                self.err(
                    f'{ns}: bridge wheel_speed_max={ws} is below the '
                    f'controller v_max={v_max}, so the bridge would clip '
                    f'commands the controller believes are feasible and break '
                    f'ICR-consistency.')

    def check_module_name_pairing(self, robot_blocks):
        for ns in robot_blocks:
            br = _params(self.doc, f'{ns}/swerve_cooptrans_bridge')
            st = _params(self.doc, f'{ns}/swerve_steering')
            if not br or not st:
                continue
            a, b = list(br.get('module_names', [])), list(st.get('module_names', []))
            if a != b:
                self.err(
                    f'{ns}: bridge module_names={a} != steering '
                    f'module_names={b}. The bridge would publish steering '
                    f'commands on topics nothing subscribes to.')

    def check_sim_mirror(self, ctrl):
        """kinematic_plant must mirror the controller or sim results are a lie."""
        pl = _params(self.doc, 'kinematic_plant')
        if not pl:
            return
        for key in ('robot_names', 'r_x', 'r_y'):
            if key in pl and key in ctrl and list(pl[key]) != list(ctrl[key]):
                self.err(f'kinematic_plant.{key} does not match '
                         f'controller.{key}; simulation would not represent '
                         f'the configured hardware.')
        if 'steer_rate_max' in pl and 'w_max' in ctrl:
            if abs(float(pl['steer_rate_max']) - float(ctrl['w_max'])) > 1e-9:
                self.err(f"kinematic_plant.steer_rate_max="
                         f"{pl['steer_rate_max']} != controller.w_max="
                         f"{ctrl['w_max']}.")
        if pl.get('steering_mode') == 'unicycle' and \
                ctrl.get('proposed', {}).get('direct_steering', True):
            self.warn("kinematic_plant.steering_mode is 'unicycle' while "
                      "proposed.direct_steering is true; the plant would "
                      "integrate a yaw rate the controller is not servoing. "
                      "Use 'servo' for swerve.")

    def check_physical_sanity(self, robot_blocks):
        for ns in robot_blocks:
            od = _params(self.doc, f'{ns}/swerve_odrive_bridge')
            wr = od.get('wheel_radius')
            if wr is None:
                continue
            if wr <= 0:
                self.err(f'{ns}: wheel_radius must be > 0 (got {wr}).')
            elif wr > self.wheel_radius_warn_value():
                self.warn(
                    f'{ns}: wheel_radius={wr} m is a {2 * wr:.2f} m diameter '
                    f'wheel. Verify this on the rig -- it scales every wheel '
                    f'command linearly, and {wr / 10:.3f} is the likely value.')

    def wheel_radius_warn_value(self):
        return self.wheel_warn

    # -- driver -----------------------------------------------------------

    def run(self, robot_blocks):
        ctrl = _params(self.doc, 'controller')
        if not ctrl:
            self.err("no 'controller' block found in the config file.")
            return
        if not self.check_controller_sizes(ctrl):
            return
        self.check_geometry(ctrl, robot_blocks)
        self.check_steering_rate(ctrl, robot_blocks)
        self.check_gear_ratio_agreement(robot_blocks)
        self.check_bridge_limits(ctrl, robot_blocks)
        self.check_module_name_pairing(robot_blocks)
        self.check_sim_mirror(ctrl)
        self.check_physical_sanity(robot_blocks)


# Packages that swdr_ws vendors from cooptrans_ws. If both workspaces are
# sourced, whichever came LAST shadows the other -- silently, and you would be
# running a different controller than you think. This runs first in every
# launch, so it is the right place to make that visible.
VENDORED = ('controller', 'trajectory_generation', 'cooptrans_sim',
            'swerve_cooptrans_bridge')


def report_package_origins():
    """Print where each vendored package actually resolves to.

    Returns a list of warning strings (packages resolving outside the same
    workspace as swerve_cooptrans_bridge, i.e. a shadowed overlay).
    """
    try:
        from ament_index_python.packages import get_package_prefix
    except ImportError:
        return []

    prefixes, warns = {}, []
    for pkg in VENDORED:
        try:
            prefixes[pkg] = get_package_prefix(pkg)
        except Exception:                                   # noqa: BLE001
            prefixes[pkg] = None

    anchor = prefixes.get('swerve_cooptrans_bridge')
    # install/<pkg> -> the workspace install/ root
    ws = os.path.dirname(anchor) if anchor else None

    for pkg, pre in prefixes.items():
        if pre is None:
            warns.append(f"package '{pkg}' not found on AMENT_PREFIX_PATH.")
            continue
        same = ws is not None and os.path.dirname(pre) == ws
        print(f"  {'   ' if same else '!! '}{pkg:<26} {pre}")
        if not same:
            warns.append(
                f"'{pkg}' resolves to {pre}, which is NOT the workspace "
                f"providing swerve_cooptrans_bridge ({ws}). Two sourced "
                f"workspaces both provide it and the other one is winning. "
                f"Source only swdr_ws for swerve work.")
    return warns


def main():
    # Deliberately NOT an rclpy node: it must be able to fail before the ROS
    # graph exists, and launch treats a non-zero exit as a hard failure.
    import argparse
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('config_file', nargs='?')
    ap.add_argument('--ros-args', action='store_true', help=argparse.SUPPRESS)
    args, extra = ap.parse_known_args()

    path = args.config_file
    for tok in extra:                       # tolerate "-p config_file:=..."
        if tok.startswith('config_file:='):
            path = tok.split(':=', 1)[1]
    if path is None:
        print('hardware_check: no config_file given', file=sys.stderr)
        return 2

    try:
        doc = yaml.safe_load(open(path)) or {}
    except Exception as e:                  # noqa: BLE001
        print(f'hardware_check: cannot read {path}: {e}', file=sys.stderr)
        return 2

    hc = _params(doc, 'hardware_check')
    robot_blocks = list(hc.get('robot_blocks', ['/robot1', '/robot2']))

    c = Checker(doc,
                float(hc.get('geometry_tol', 1e-4)),
                float(hc.get('rate_tol', 1e-3)),
                float(hc.get('wheel_radius_warn', 0.1)))
    c.run(robot_blocks)

    print(f'hardware_check: {path}')
    print('  package origins:')
    for w in report_package_origins():
        c.warn(w)
    for w in c.warnings:
        print(f'  WARN  {w}')
    if c.errors:
        for e in c.errors:
            print(f'  FATAL {e}', file=sys.stderr)
        print(f'hardware_check: {len(c.errors)} inconsistency(ies) found -- '
              f'refusing to start.', file=sys.stderr)
        return 1
    print(f'  OK    geometry, steering rate, gear ratios, limits, sim mirror'
          f'{" (" + str(len(c.warnings)) + " warning(s))" if c.warnings else ""}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
