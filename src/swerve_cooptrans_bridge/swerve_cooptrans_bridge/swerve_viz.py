#!/usr/bin/env python3
"""swerve_viz : live view of the swerve cooperative-transport system.

Answers two questions at a glance: where is everything, and what is each
module being told to do.

  CENTER  Top-down world view.
            - payload rectangle at the ESTIMATED pose (what the controller
              sees, i.e. after the LiDAR noise), plus a faint outline at the
              TRUE pose when ground truth is available, so the localization
              error is visible rather than assumed;
            - the reference path and a hollow ghost payload at the current
              setpoint, with a line to the measured pose whose length IS the
              tracking error driving eq. 13's slowdown;
            - each module drawn as a wheel oriented at its MEASURED heading,
              with a solid arrow for commanded wheel speed and a dashed spoke
              for the COMMANDED heading. When the two diverge the module is
              not yet where the controller wants it, which is exactly the
              condition direct steering assumes away;
            - the common ICR, estimated from the module headings. All spokes
              meeting at one point is the visual statement of ICR-consistency;
              a scattered intersection means the commands are inconsistent.

  LEFT    Commanded vs measured module heading, per module, as paired bars on
          a +/-pi axis, with the heading error drawn as a bar from one to the
          other.

  RIGHT   Commanded wheel speed per module, against the v_max envelope.

  BOTTOM  Rolling traces of payload position error and max module heading
          error, the two numbers that say whether the run is healthy.

Threading: ROS callbacks only cache the latest message behind a lock; the
rclpy executor spins in a background thread; a Qt timer snapshots the cache
and repaints. Same model as cooptrans_viz.
"""

import math
import sys
import threading
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from std_msgs.msg import Float64

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')
pg.setConfigOptions(antialias=True)

MODULE_COLORS = ['#d62728', '#2ca02c', '#1f77b4', '#ff7f0e',
                 '#9467bd', '#8c564b', '#e377c2', '#7f7f7f']
PAYLOAD_FILL = (80, 120, 200, 60)
PAYLOAD_EDGE = '#1f3f7f'
TRUTH_EDGE = '#999999'
GHOST_EDGE = '#7f7f7f'
ERR_COLOR = '#d62728'
PATH_COLOR = '#9ecae1'
ICR_COLOR = '#ff7f0e'
CMD_COLOR = '#000000'


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rot(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s], [s, c]])


class VizNode(Node):
    """Caches the latest of everything; does no drawing."""

    def __init__(self):
        super().__init__('swerve_viz')
        p = self.declare_parameter

        self.prefix = str(p('topic_prefix', '').value)
        self.payload_name = str(p('payload_name', 'payload').value)
        self.agents = list(p('robot_names',
                             ['module1', 'module2', 'module3', 'module4']).value)
        self.r_x = list(p('r_x', [0.35, 0.35, -0.35, -0.35]).value)
        self.r_y = list(p('r_y', [0.23, -0.23, 0.23, -0.23]).value)
        self.payload_size = list(p('payload_size', [1.0, 0.4]).value)
        self.wheel_len = float(p('wheel_len', 0.14).value)
        self.wheel_wid = float(p('wheel_wid', 0.06).value)
        self.v_max = float(p('v_max', 0.20).value)
        self.arrow_scale = float(p('arrow_scale', 1.2).value)
        self.bounds = list(p('view_bounds', [-2.6, 2.6, -2.0, 2.0]).value)
        self.update_hz = float(p('update_hz', 30.0).value)
        self.trace_sec = float(p('trace_seconds', 30.0).value)
        self.truth_topic = str(p('truth_topic', '').value)   # '' = disabled
        self.show_icr = bool(p('show_icr', True).value)

        # Save a PNG of the window after N seconds and (optionally) quit.
        # Handy for capturing a figure from a headless/remote session, and for
        # automated checks that the GUI actually renders.
        self.shot_path = str(p('screenshot_path', '').value)
        self.shot_after = float(p('screenshot_after', 0.0).value)
        self.shot_quit = bool(p('screenshot_quit', False).value)

        self.n = len(self.agents)
        self.lock = threading.Lock()

        # cached state
        self.pose = None          # (x, y, yaw) estimated
        self.truth = None         # (x, y, yaw) ground truth, optional
        self.ref = None           # (x, y, yaw) setpoint
        self.path = None          # (N,2)
        self.th_meas = [None] * self.n    # measured module heading, world
        self.th_cmd = [None] * self.n     # commanded module heading, world
        self.v_cmd = [0.0] * self.n       # commanded wheel speed
        self.t0 = None
        self.tr_t = deque(maxlen=4000)
        self.tr_pos = deque(maxlen=4000)
        self.tr_eth = deque(maxlen=4000)

        qos = qos_profile_sensor_data
        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        base = f'{self.prefix}{self.payload_name}'
        self.create_subscription(PoseStamped, f'{base}/pose', self._on_pose, qos)
        self.create_subscription(PoseStamped, f'{base}/ref_pose', self._on_ref, qos)
        self.create_subscription(Path, f'{base}/ref_path', self._on_path, latched)
        if self.truth_topic:
            self.create_subscription(PoseStamped, self.truth_topic,
                                     self._on_truth, qos)

        for i, a in enumerate(self.agents):
            self.create_subscription(PoseStamped, f'{self.prefix}{a}/pose',
                                     self._mk_meas(i), qos)
            self.create_subscription(Float64, f'{self.prefix}{a}/desired_heading',
                                     self._mk_cmd(i), qos)
            self.create_subscription(Twist, f'{self.prefix}{a}/cmd_vel',
                                     self._mk_v(i), qos)

        self.get_logger().info(
            f'swerve_viz: {self.n} modules {self.agents}, payload '
            f'{self.payload_size[0]:.2f} x {self.payload_size[1]:.2f} m'
            + (f', ground truth from {self.truth_topic}' if self.truth_topic
               else ', no ground truth'))

    # --- callbacks (cache only) -----------------------------------------

    def _on_pose(self, m):
        with self.lock:
            self.pose = (m.pose.position.x, m.pose.position.y,
                         yaw_from_quat(m.pose.orientation))
            self._sample()

    def _on_truth(self, m):
        with self.lock:
            self.truth = (m.pose.position.x, m.pose.position.y,
                          yaw_from_quat(m.pose.orientation))

    def _on_ref(self, m):
        with self.lock:
            self.ref = (m.pose.position.x, m.pose.position.y,
                        yaw_from_quat(m.pose.orientation))

    def _on_path(self, m):
        with self.lock:
            self.path = np.array([[p.pose.position.x, p.pose.position.y]
                                  for p in m.poses]) if m.poses else None

    def _mk_meas(self, i):
        def cb(m):
            with self.lock:
                self.th_meas[i] = yaw_from_quat(m.pose.orientation)
        return cb

    def _mk_cmd(self, i):
        def cb(m):
            with self.lock:
                self.th_cmd[i] = wrap(m.data)
        return cb

    def _mk_v(self, i):
        def cb(m):
            with self.lock:
                self.v_cmd[i] = m.linear.x
        return cb

    def _sample(self):
        """Append one trace sample. Caller holds the lock."""
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        if self.pose is None or self.ref is None:
            return
        self.tr_t.append(t - self.t0)
        self.tr_pos.append(math.hypot(self.pose[0] - self.ref[0],
                                      self.pose[1] - self.ref[1]))
        errs = [abs(wrap(self.th_meas[i] - self.th_cmd[i]))
                for i in range(self.n)
                if self.th_meas[i] is not None and self.th_cmd[i] is not None]
        self.tr_eth.append(max(errs) if errs else 0.0)

    def snapshot(self):
        with self.lock:
            return dict(
                pose=self.pose, truth=self.truth, ref=self.ref,
                path=None if self.path is None else self.path.copy(),
                th_meas=list(self.th_meas), th_cmd=list(self.th_cmd),
                v_cmd=list(self.v_cmd),
                tr=(list(self.tr_t), list(self.tr_pos), list(self.tr_eth)))


def _poly(pts):
    return QtGui.QPolygonF([QtCore.QPointF(float(x), float(y)) for x, y in pts])


class Window(QtWidgets.QWidget):

    def __init__(self, node):
        super().__init__()
        self.node = node
        n = node.n
        self.setWindowTitle('swerve_viz — cooperative transport')
        self.resize(1500, 900)

        grid = QtWidgets.QGridLayout(self)

        # ---------- CENTER: top-down ----------
        self.center = pg.PlotWidget(title='Top-down (world frame) — '
                                          'solid arrow = commanded wheel speed, '
                                          'dashed spoke = commanded heading')
        self.center.setAspectLocked(True)
        self.center.showGrid(x=True, y=True, alpha=0.3)
        self.center.setLabel('bottom', 'x', units='m')
        self.center.setLabel('left', 'y', units='m')
        xmin, xmax, ymin, ymax = node.bounds
        self.center.setXRange(xmin, xmax)
        self.center.setYRange(ymin, ymax)
        self.vb = self.center.getViewBox()
        grid.addWidget(self.center, 0, 1, 2, 1)

        self.path_curve = pg.PlotCurveItem(pen=pg.mkPen(PATH_COLOR, width=2))
        self.center.addItem(self.path_curve)
        self.err_line = pg.PlotCurveItem(pen=pg.mkPen(ERR_COLOR, width=2))
        self.center.addItem(self.err_line)

        self.ghost = QtWidgets.QGraphicsPolygonItem()
        self.ghost.setBrush(pg.mkBrush(None))
        self.ghost.setPen(pg.mkPen(GHOST_EDGE, width=2, dash=[6, 4]))
        self.vb.addItem(self.ghost)

        self.truth_poly = QtWidgets.QGraphicsPolygonItem()
        self.truth_poly.setBrush(pg.mkBrush(None))
        self.truth_poly.setPen(pg.mkPen(TRUTH_EDGE, width=1, dash=[2, 3]))
        self.vb.addItem(self.truth_poly)

        self.body = QtWidgets.QGraphicsPolygonItem()
        self.body.setBrush(pg.mkBrush(PAYLOAD_FILL))
        self.body.setPen(pg.mkPen(PAYLOAD_EDGE, width=2))
        self.vb.addItem(self.body)

        self.wheels, self.cmd_spokes, self.vel_arrows = [], [], []
        for i in range(n):
            col = MODULE_COLORS[i % len(MODULE_COLORS)]
            w = QtWidgets.QGraphicsPolygonItem()
            w.setBrush(pg.mkBrush(col))
            w.setPen(pg.mkPen('k', width=1))
            self.vb.addItem(w)
            self.wheels.append(w)
            sp = pg.PlotCurveItem(pen=pg.mkPen(CMD_COLOR, width=1.5, dash=[5, 4]))
            self.center.addItem(sp)
            self.cmd_spokes.append(sp)
            ar = pg.PlotCurveItem(pen=pg.mkPen(col, width=3))
            self.center.addItem(ar)
            self.vel_arrows.append(ar)

        self.icr_dot = pg.ScatterPlotItem(size=14, symbol='x',
                                          pen=pg.mkPen(ICR_COLOR, width=3))
        self.center.addItem(self.icr_dot)
        self.icr_rays = pg.PlotCurveItem(
            pen=pg.mkPen(ICR_COLOR, width=1, dash=[2, 4]))
        self.center.addItem(self.icr_rays)
        self.icr_label = pg.TextItem('', color=ICR_COLOR, anchor=(0, 1))
        self.center.addItem(self.icr_label)

        # ---------- LEFT: heading commanded vs measured ----------
        self.left = pg.PlotWidget(title='Module heading [rad]')
        self.left.setMouseEnabled(x=False, y=False)
        self.left.setYRange(-math.pi - 0.3, math.pi + 0.3)
        self.left.setXRange(-0.6, n - 0.4)
        self.left.getAxis('bottom').setTicks([list(enumerate(node.agents))])
        self.left.addItem(pg.InfiniteLine(pos=0, angle=0,
                                          pen=pg.mkPen((150, 150, 150))))
        self.left.addLegend(offset=(10, 10))
        # grey bar = commanded (theta_d); black tick = measured; red segment
        # between them is the heading error the module has yet to take up.
        self.head_bar = pg.BarGraphItem(x=list(range(n)), height=[0] * n,
                                        width=0.55, brush='#c8c8c8')
        self.left.addItem(self.head_bar)
        self.err_segs = pg.PlotCurveItem(
            pen=pg.mkPen(ERR_COLOR, width=3), connect='finite')
        self.left.addItem(self.err_segs)
        # Drawn as explicit line segments rather than a scatter symbol: there
        # is no horizontal-tick symbol in pyqtgraph, and a wrong symbol name
        # raises inside the repaint and silently blanks every later panel.
        self.meas_ticks = pg.PlotCurveItem(
            pen=pg.mkPen('k', width=3), connect='finite')
        self.left.addItem(self.meas_ticks)
        self.left.plot([], [], pen=pg.mkPen('#c8c8c8', width=8),
                       name='commanded  theta_d')
        self.left.plot([], [], pen=pg.mkPen('k', width=3), name='measured')
        self.left.plot([], [], pen=pg.mkPen(ERR_COLOR, width=3),
                       name='error')
        grid.addWidget(self.left, 0, 0)

        # ---------- RIGHT-of-left: commanded wheel speed ----------
        self.right = pg.PlotWidget(title='Commanded wheel speed [m/s]')
        self.right.setMouseEnabled(x=False, y=False)
        lim = node.v_max * 1.35
        self.right.setYRange(-lim, lim)
        self.right.getAxis('bottom').setTicks([list(enumerate(node.agents))])
        self.vbar = pg.BarGraphItem(x=list(range(n)), height=[0] * n,
                                    width=0.55, brush='#2ca02c')
        self.right.addItem(self.vbar)
        for s in (+node.v_max, -node.v_max):
            self.right.addItem(pg.InfiniteLine(
                pos=s, angle=0,
                pen=pg.mkPen('#d62728', width=1, dash=[6, 4])))
        self.right.addItem(pg.InfiniteLine(pos=0, angle=0,
                                           pen=pg.mkPen((150, 150, 150))))
        grid.addWidget(self.right, 1, 0)

        # ---------- BOTTOM: traces ----------
        self.trace = pg.PlotWidget(title='Tracking')
        self.trace.addLegend(offset=(10, 10))
        self.trace.showGrid(x=True, y=True, alpha=0.3)
        self.trace.setLabel('bottom', 't', units='s')
        self.c_pos = self.trace.plot(pen=pg.mkPen('#1f77b4', width=2),
                                     name='payload position error [m]')
        self.c_eth = self.trace.plot(pen=pg.mkPen('#d62728', width=2),
                                     name='max module heading error [rad]')
        grid.addWidget(self.trace, 2, 0, 1, 2)

        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 3)
        grid.setRowStretch(0, 2)
        grid.setRowStretch(1, 2)
        grid.setRowStretch(2, 1)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(int(1000.0 / node.update_hz))

        if node.shot_path and node.shot_after > 0.0:
            QtCore.QTimer.singleShot(int(node.shot_after * 1000.0),
                                     self._screenshot)

    def _screenshot(self):
        ok = self.grab().save(self.node.shot_path)
        self.node.get_logger().info(
            f"screenshot {'saved to ' + self.node.shot_path if ok else 'FAILED'}")
        if self.node.shot_quit:
            QtWidgets.QApplication.instance().quit()

    # ------------------------------------------------------------------

    def _rect(self, x, y, yaw, lx, ly):
        R = rot(yaw)
        half = np.array([[lx / 2, ly / 2], [-lx / 2, ly / 2],
                         [-lx / 2, -ly / 2], [lx / 2, -ly / 2]])
        return (half @ R.T) + np.array([x, y])

    def _icr(self, mods):
        """Least-squares common ICR from module positions + headings.

        A module rolling about the ICR has its velocity perpendicular to the
        line joining it to the ICR, and its velocity direction IS its heading
        t_i = [cos th, sin th]. So (p_icr - p_i) must be perpendicular to the
        HEADING:

            t_i . (p_icr - p_i) = 0

        (Using the wheel normal here instead would assert the opposite and
        produce a spurious non-zero residual even for perfectly ICR-consistent
        commands.) Stack over modules and solve in least squares. The residual
        is the RMS perpendicular distance from the fitted point to each
        module's constraint line -- effectively the paper's common-ICR
        residual, and it should sit near zero whenever the commands are
        kinematically consistent.

        Returns (point, residual), or None when the system is near rank-1
        (all headings parallel => pure translation, ICR at infinity, nothing
        meaningful to draw).
        """
        if len(mods) < 2:
            return None
        A, b = [], []
        for (px, py, th) in mods:
            tx, ty = math.cos(th), math.sin(th)
            A.append([tx, ty])
            b.append(tx * px + ty * py)
        A, b = np.array(A), np.array(b)
        try:
            s = np.linalg.svd(A, compute_uv=False)
            if s[-1] < 1e-3:                 # near-parallel -> ICR at infinity
                return None
            sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError:
            return None
        resid = float(np.sqrt(np.mean((A @ sol - b) ** 2)))
        return sol, resid

    def refresh(self):
        # A raise partway through a repaint leaves every panel AFTER the
        # failure point frozen with stale data and no visible error -- the
        # window looks alive but is lying. Report it loudly, once, instead.
        try:
            self._refresh()
        except Exception:
            if not getattr(self, '_draw_failed', False):
                self._draw_failed = True
                import traceback
                self.node.get_logger().error(
                    'swerve_viz repaint failed; panels after the failure '
                    'point are NOT updating:\n' + traceback.format_exc())

    def _refresh(self):
        node = self.node
        d = node.snapshot()
        n = node.n

        if d['pose'] is None:
            return
        px, py, pth = d['pose']
        lx, ly = node.payload_size

        self.body.setPolygon(_poly(self._rect(px, py, pth, lx, ly)))

        if d['truth'] is not None:
            tx, ty, tth = d['truth']
            self.truth_poly.setPolygon(_poly(self._rect(tx, ty, tth, lx, ly)))
            self.truth_poly.setVisible(True)
        else:
            self.truth_poly.setVisible(False)

        if d['path'] is not None and len(d['path']) > 1:
            self.path_curve.setData(d['path'][:, 0], d['path'][:, 1])

        if d['ref'] is not None:
            rx, ry, rth = d['ref']
            self.ghost.setPolygon(_poly(self._rect(rx, ry, rth, lx, ly)))
            self.ghost.setVisible(True)
            self.err_line.setData([rx, px], [ry, py])
        else:
            self.ghost.setVisible(False)

        # modules: position is rigid in the payload frame
        R = rot(pth)
        mods = []
        heads, vels = [], []
        tick_x, tick_y, err_x, err_y = [], [], [], []
        for i in range(n):
            mp = np.array([px, py]) + R @ np.array([node.r_x[i], node.r_y[i]])
            thm = d['th_meas'][i]
            thc = d['th_cmd'][i]
            v = d['v_cmd'][i]

            if thm is None:
                self.wheels[i].setVisible(False)
            else:
                self.wheels[i].setVisible(True)
                self.wheels[i].setPolygon(_poly(self._rect(
                    mp[0], mp[1], thm, node.wheel_len, node.wheel_wid)))
                mods.append((mp[0], mp[1], thm))
                tick_x += [i - 0.33, i + 0.33, np.nan]
                tick_y += [wrap(thm), wrap(thm), np.nan]
                # commanded wheel speed arrow, along the MEASURED heading
                # (that is the direction the wheel will actually push)
                L = v * node.arrow_scale
                self.vel_arrows[i].setData(
                    [mp[0], mp[0] + L * math.cos(thm)],
                    [mp[1], mp[1] + L * math.sin(thm)])

            if thc is None:
                self.cmd_spokes[i].setData([], [])
                heads.append(0.0)
            else:
                heads.append(wrap(thc))
                s = node.wheel_len * 1.5
                self.cmd_spokes[i].setData(
                    [mp[0] - s * math.cos(thc), mp[0] + s * math.cos(thc)],
                    [mp[1] - s * math.sin(thc), mp[1] + s * math.sin(thc)])
                if thm is not None:
                    # Plot the error at the commanded bar's top so the visible
                    # gap is the error itself, not the raw wrap difference
                    # (which would jump by 2*pi across the +/-pi seam).
                    err_x += [i, i, np.nan]
                    err_y += [wrap(thc), wrap(thc) + wrap(thm - thc), np.nan]
            vels.append(v)

        self.head_bar.setOpts(height=heads)
        self.meas_ticks.setData(np.array(tick_x), np.array(tick_y))
        self.err_segs.setData(np.array(err_x), np.array(err_y))
        self.vbar.setOpts(height=vels,
                          brushes=['#d62728' if abs(v) > node.v_max + 1e-9
                                   else '#2ca02c' for v in vels])

        # common ICR + the wheel-normal rays that should meet there
        if node.show_icr and len(mods) >= 2:
            got = self._icr(mods)
            if got is None:
                self.icr_dot.setData([])
                self.icr_rays.setData([], [])
                self.icr_label.setText('ICR: at infinity (pure translation)')
                self.icr_label.setPos(node.bounds[0], node.bounds[2])
            else:
                c, resid = got
                self.icr_dot.setData([c[0]], [c[1]])
                xs, ys = [], []
                for (mx, my, _) in mods:
                    xs += [mx, c[0], np.nan]
                    ys += [my, c[1], np.nan]
                self.icr_rays.setData(np.array(xs), np.array(ys))
                self.icr_label.setText(
                    f'ICR ({c[0]:+.2f}, {c[1]:+.2f})  resid {resid:.4f} m')
                self.icr_label.setPos(node.bounds[0], node.bounds[2])

        t, ep, eth = d['tr']
        if t:
            t0 = t[-1] - node.trace_sec
            k = [j for j, tv in enumerate(t) if tv >= t0]
            if k:
                a = k[0]
                self.c_pos.setData(t[a:], ep[a:])
                self.c_eth.setData(t[a:], eth[a:])
                self.trace.setXRange(t0, max(t[-1], t0 + 1e-3))


def main():
    rclpy.init()
    node = VizNode()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    app = QtWidgets.QApplication(sys.argv)
    win = Window(node)
    win.show()
    try:
        app.exec() if hasattr(app, 'exec') else app.exec_()
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
