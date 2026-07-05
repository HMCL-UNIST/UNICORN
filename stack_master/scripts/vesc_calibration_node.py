#!/usr/bin/env python3
"""
vesc_calibration_node — fit the VESC speed/steering mapping from EXTERNAL
odometry (cartographer/kiss/livox), not the VESC's own eRPM.

It publishes ordinary Ackermann commands to the mux's high-level input
(/vesc/high_level/ackermann_cmd, the autodrive source). simple_mux arbitrates:
press the joystick autodrive button to let the calibrator drive, the humandrive
deadman to take over for an emergency — the mux handles that override for us, so
this node has NO joystick/yield logic of its own.

Because the command flows through ackermann_to_vesc, what physically reaches the
VESC is computed with the CURRENT gains. We read those current gains from
`source_node`, so for each commanded point we know the eRPM/servo actually sent
and can fit the TRUE gain against the measured response:

  SPEED   — command speeds v_cmd straight; eRPM_sent = cur_gain*v_cmd + cur_off.
            Measure ground speed v_meas, fit  eRPM_sent = g*v_meas + c
            -> speed_to_erpm_gain = g (offset c).

  STEERING— hold a constant speed, sweep steering delta_cmd across zero;
            servo_sent = cur_gain*delta_cmd + cur_off. Recover the actual wheel
            angle delta_meas = atan(yaw_rate*wheelbase/v) and fit
            servo_sent = gain*delta_meas + offset
            -> steering_angle_to_servo_gain = slope,  _offset = intercept (neutral).

All commanded/measured values stream to pitwall (/pitwall/*) for one-file MCAP
capture in Foxglove, and the "current -> suggested" result is also emitted on
/pitwall/events so it shows up in the pitwall RViz panel's Telemetry box.

One point at a time: set cmd_speed / cmd_steer live from rqt_reconfigure, then
press the joystick autodrive button to drive that single point. Each press yields
ONE standalone result (command / expected / measured / which config / RAISE-LOWER)
— no accumulation. manual_enable streams the setpoint continuously (free-drive).

SAFETY: nothing moves until the joystick AUTODRIVE button arms the mux AND starts
a run; the mux must have autodrive selected for any command to reach the car.
`~/stop`, Ctrl-C, or the joystick humandrive deadman all stop motion.

Path viz: during each autodrive session the node publishes ~/expected_path (from
/vesc/odom, vesc dead-reckoning) and ~/estimated_path (from /car_state/odom,
localization), both relative to their session start so they overlay. They clear
at the start of each new autodrive session — their divergence is the calibration
error made visible. Add two Path displays in RViz (topics
/vesc_calibration/expected_path and /vesc_calibration/estimated_path) to view them.
"""

import math
import os
import re
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu, Joy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

# pitwall is optional — if the overlay isn't built the node still runs, telemetry
# just degrades to a no-op shim.
try:
    import pitwall
except Exception:  # pragma: no cover - pitwall missing is non-fatal
    class _PitwallShim:
        def init(self, *_):
            pass

        def log(self, *_):
            pass

        def event(self, *_):
            pass

    pitwall = _PitwallShim()


class VescCalibrationNode(Node):

    def __init__(self):
        super().__init__('vesc_calibration')

        gp = self.declare_parameter

        # --- topology / fit inputs --------------------------------------
        gp('cmd_topic', '/vesc/high_level/ackermann_cmd')  # mux autodrive input
        gp('odom_topic', '/car_state/odom')                # ground-truth (ESTIMATED) speed/yaw
        gp('expected_odom_topic', '/vesc/odom')            # vesc dead-reckoning (EXPECTED path)
        gp('imu_topic', '/vesc/sensors/imu/raw')
        gp('wheelbase', 0.33)
        gp('source_node', '/vesc/ackermann_to_vesc_node')  # read current gains here
        # BOTH vesc nodes share the same gains (vehicle_config.yaml `vesc/**`
        # wildcard): ackermann_to_vesc uses them to COMMAND the car, vesc_to_odom
        # to ESTIMATE odometry. apply/save push to both so they stay consistent.
        gp('sync_nodes', ['/vesc/ackermann_to_vesc_node', '/vesc/vesc_to_odom_node'])
        # yaml written by ~/save (or the save_config toggle). Empty -> resolve to
        # the installed stack_master config/CAR/vehicle_config.yaml.
        gp('vehicle_config_path', '')

        # --- path viz: expected (vesc odom) vs estimated (localization) --------
        # Both paths are recorded RELATIVE to their own first pose of the current
        # autodrive session (so they share a common origin and can overlay), and
        # are CLEARED at the start of each new autodrive run. The divergence
        # between them is the calibration error made visible.
        gp('path_frame', 'odom')     # frame_id stamped on both published paths
        gp('path_max_len', 3000)     # cap poses per path (drop oldest beyond this)

        # current vesc gains — defaults mirror config/CAR/vehicle_config.yaml;
        # overridden live from source_node at startup when reachable.
        gp('cur_speed_to_erpm_gain', 3117.0)
        gp('cur_speed_to_erpm_offset', 0.0)
        gp('cur_steering_angle_to_servo_gain', 0.7579)
        gp('cur_steering_angle_to_servo_offset', 0.4264)

        # --- command point (ONE at a time, rqt-settable live) -----------
        # Set cmd_speed/cmd_steer, then press autodrive to drive that single point.
        # Each press gives one STANDALONE result (no accumulation). No mode: the
        # point is classified by its steering value:
        #   steering ~= 0  -> straight: speed_to_erpm_gain dir + servo neutral dir
        #   steering != 0  -> turn:     steering_angle_to_servo_gain dir
        # ~/reset clears the last result + the viz paths.
        gp('cmd_speed', 1.5)               # m/s
        gp('cmd_steer', 0.0)               # rad
        gp('steer_zero_tol', 0.02)         # |steering| below this = straight

        # --- timing / safety --------------------------------------------
        gp('settle_time', 1.0)        # min wait after ramp before checking steady
        gp('settle_timeout', 4.0)     # max extra wait for the speed to plateau
        gp('steady_window', 0.6)      # s trailing window used for the plateau test
        gp('steady_band', 0.10)       # m/s max spread in that window to call it steady
        gp('measure_time', 1.5)
        gp('pause_time', 1.0)
        gp('ramp_time', 0.5)
        gp('cmd_rate', 50.0)
        gp('min_speed', 0.05)
        gp('max_abs_speed', 3.0)                           # m/s hard cap
        gp('max_abs_steer', 0.4)                           # rad hard cap

        # --- manual (rqt) drive -----------------------------------------
        # Stream the cmd_speed/cmd_steer setpoint continuously (free-drive) without
        # recording a sample — handy to check the setpoint before pressing autodrive.
        gp('manual_enable', False)

        # --- save from rqt ----------------------------------------------
        # Toggle True in rqt_reconfigure to save the CURRENT live vesc gains into
        # vehicle_config.yaml (and sync both vesc nodes). It auto-resets to False
        # so it can be re-armed. (~/save does the same from the CLI.)
        gp('save_config', False)

        # --- joy control (no start service: the joystick IS the start) ----
        # Selecting AUTODRIVE arms the mux to forward our commands; we use the
        # same button press as the calibration start. Humandrive aborts (and the
        # mux hands the car to the operator). Indices match simple_mux._handle_joy.
        gp('joy_topic', '/joy')
        gp('joy_autostart_enable', True)
        gp('joy_autostart_button', 5)   # autodrive (arm mux + start sequence)
        gp('joy_abort_button', 4)       # humandrive deadman (abort + mux override)

        v = lambda n: self.get_parameter(n).value
        self.wheelbase = float(v('wheelbase'))
        self.settle_time = float(v('settle_time'))
        self.settle_timeout = float(v('settle_timeout'))
        self.steady_window = float(v('steady_window'))
        self.steady_band = float(v('steady_band'))
        self.measure_time = float(v('measure_time'))
        self.pause_time = float(v('pause_time'))
        self.ramp_time = float(v('ramp_time'))
        self.min_speed = float(v('min_speed'))
        self.max_abs_speed = float(v('max_abs_speed'))
        self.max_abs_steer = float(v('max_abs_steer'))
        self.sp_speed = float(v('cmd_speed'))   # current setpoint (rqt-settable)
        self.sp_steer = float(v('cmd_steer'))
        self.steer_zero_tol = float(v('steer_zero_tol'))
        cmd_rate = float(v('cmd_rate'))

        self.cur_speed_gain = float(v('cur_speed_to_erpm_gain'))
        self.cur_speed_offset = float(v('cur_speed_to_erpm_offset'))
        self.cur_steer_gain = float(v('cur_steering_angle_to_servo_gain'))
        self.cur_steer_offset = float(v('cur_steering_angle_to_servo_offset'))

        self.manual_enable = bool(v('manual_enable'))

        self.path_frame = str(v('path_frame'))
        self.path_max_len = int(v('path_max_len'))

        self.joy_autostart_enable = bool(v('joy_autostart_enable'))
        self.joy_autostart_button = int(v('joy_autostart_button'))
        self.joy_abort_button = int(v('joy_abort_button'))

        # --- state -------------------------------------------------------
        self._lock = threading.Lock()
        self.cmd_speed = 0.0        # m/s currently commanded by the sequence
        self.cmd_steer = 0.0        # rad currently commanded by the sequence
        self.last_v = 0.0
        self.last_w = 0.0
        self.collecting = False
        self._v_buf = []
        self._w_buf = []            # twist yaw-rate (fallback only)
        self._t_buf = []            # sample wall time, for pose-based yaw rate
        self._yaw_buf = []          # pose heading, for pose-based yaw rate
        self._x_buf = []            # pose x, for pose-based speed fallback
        self._y_buf = []            # pose y, for pose-based speed fallback
        self._last_results = None   # latest SINGLE-RUN estimate, for ~/apply
        self._running = False
        self._abort = threading.Event()
        self._prev_auto = False     # for autodrive button rising-edge detection

        # path viz state: relative-to-start origins + accumulated Paths
        self._exp_p0 = None         # (x0,y0,yaw0) of vesc odom at session start
        self._est_p0 = None         # (x0,y0,yaw0) of localization odom at session start
        self._exp_path = Path()
        self._est_path = Path()

        # --- ROS I/O -----------------------------------------------------
        cb = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, v('cmd_topic'), 10)
        self.exp_path_pub = self.create_publisher(Path, '~/expected_path', 1)
        self.est_path_pub = self.create_publisher(Path, '~/estimated_path', 1)
        self.create_subscription(Odometry, v('odom_topic'),
                                 self._odom_cb, 20, callback_group=cb)
        self.create_subscription(Odometry, v('expected_odom_topic'),
                                 self._vesc_odom_cb, 20, callback_group=cb)
        self.create_subscription(Imu, v('imu_topic'),
                                 self._imu_cb, 50, callback_group=cb)
        self.create_subscription(Joy, v('joy_topic'),
                                 self._joy_cb, 10, callback_group=cb)
        # republish at cmd_rate so the mux always sees a FRESH autodrive command
        # (it drops stale ones); a single missed message never stalls a step.
        self.create_timer(1.0 / cmd_rate, self._cmd_tick, callback_group=cb)

        # No start service: selecting autodrive on the joystick starts the run.
        # ~/stop stays as an optional software abort (Ctrl-C / humandrive also stop).
        self.create_service(Trigger, '~/stop', self._on_stop, callback_group=cb)
        self.create_service(Trigger, '~/reset', self._on_reset, callback_group=cb)
        self.create_service(Trigger, '~/apply', self._on_apply, callback_group=cb)
        self.create_service(Trigger, '~/save', self._on_save, callback_group=cb)
        self.add_on_set_parameters_callback(self._on_set_params)

        pitwall.init(self)
        threading.Thread(target=self._read_current_gains, daemon=True).start()

        self.get_logger().info(
            "vesc_calibration ready. Set cmd_speed/cmd_steer (rqt), then press "
            "AUTODRIVE on the joystick to drive that ONE point; each press gives a "
            "standalone RAISE/LOWER result. humandrive deadman aborts. Tune gains "
            "live in rqt on the vesc nodes, then toggle save_config (or call ~/save) "
            "to persist them to vehicle_config.yaml + sync both vesc nodes.")

    GAIN_NAMES = ('speed_to_erpm_gain', 'speed_to_erpm_offset',
                  'steering_angle_to_servo_gain', 'steering_angle_to_servo_offset')

    # ===== live parameters ==============================================
    def _on_set_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'manual_enable':
                self.manual_enable = bool(p.value)
            elif p.name == 'cmd_speed':
                self.sp_speed = float(p.value)
            elif p.name == 'cmd_steer':
                self.sp_steer = float(p.value)
            elif p.name == 'save_config' and bool(p.value):
                # do the save off-thread so the param-set returns immediately, then
                # auto-reset the toggle so it can be pressed again.
                threading.Thread(target=self._save_and_rearm, daemon=True).start()
        return SetParametersResult(successful=True)

    def _fetch_gains(self, node, timeout=5.0):
        """Read the 4 vesc gains from `node`'s parameters. Returns a dict keyed by
        GAIN_NAMES, or None on failure."""
        try:
            from rclpy.parameter_client import AsyncParameterClient
            apc = AsyncParameterClient(self, node)
            if not apc.wait_for_services(timeout_sec=timeout):
                return None
            fut = apc.get_parameters(list(self.GAIN_NAMES))
            deadline = time.time() + timeout
            while not fut.done() and time.time() < deadline and rclpy.ok():
                time.sleep(0.05)
            if not fut.done():
                return None
            vals = fut.result().values
            return {n: vals[i].double_value for i, n in enumerate(self.GAIN_NAMES)}
        except Exception as e:
            self.get_logger().warn("fetch gains from %s failed: %s" % (node, e))
            return None

    def _read_current_gains(self):
        """Best-effort: seed cur_* from `source_node`'s live parameters at startup."""
        src = self.get_parameter('source_node').value
        g = self._fetch_gains(src)
        if not g:
            self.get_logger().warn(
                "source_node %s gains unreadable; using config-default current gains" % src)
            return
        self.cur_speed_gain = g['speed_to_erpm_gain'] or self.cur_speed_gain
        self.cur_speed_offset = g['speed_to_erpm_offset']
        self.cur_steer_gain = g['steering_angle_to_servo_gain'] or self.cur_steer_gain
        self.cur_steer_offset = g['steering_angle_to_servo_offset'] or self.cur_steer_offset
        self.get_logger().info(
            "read current gains from %s: speed_gain=%.1f steer_gain=%.4f steer_offset=%.4f"
            % (src, self.cur_speed_gain, self.cur_steer_gain, self.cur_steer_offset))

    def _push_params(self, params, nodes):
        """Set `params` (list of rclpy Parameter) on every node in `nodes` LIVE.
        Returns True only if all nodes accepted them."""
        from rclpy.parameter_client import AsyncParameterClient
        all_ok = True
        for nd in nodes:
            try:
                apc = AsyncParameterClient(self, nd)
                if not apc.wait_for_services(timeout_sec=3.0):
                    self.get_logger().warn("%s not reachable" % nd)
                    all_ok = False
                    continue
                fut = apc.set_parameters(params)
                deadline = time.time() + 3.0
                while not fut.done() and time.time() < deadline and rclpy.ok():
                    time.sleep(0.05)
                if not fut.done() or not all(r.successful for r in fut.result().results):
                    self.get_logger().warn("%s rejected some params" % nd)
                    all_ok = False
            except Exception as e:
                self.get_logger().warn("push to %s failed: %s" % (nd, e))
                all_ok = False
        return all_ok

    # ===== subscriptions ================================================
    def _odom_cb(self, msg: Odometry):
        # ground-truth (ESTIMATED) odom drives both the fit measurement AND the
        # estimated path viz.
        vx = msg.twist.twist.linear.x
        v = math.copysign(math.hypot(vx, msg.twist.twist.linear.y), vx)
        w = msg.twist.twist.angular.z            # may be 0 on some localizers (kiss)
        yaw = self._yaw_from_quat(msg.pose.pose.orientation)
        px, py = msg.pose.pose.position.x, msg.pose.pose.position.y
        t = time.time()
        with self._lock:
            self.last_v = v
            self.last_w = w
            if self.collecting:
                self._v_buf.append(v)
                self._w_buf.append(w)
                self._t_buf.append(t)
                self._yaw_buf.append(yaw)
                self._x_buf.append(px)
                self._y_buf.append(py)
        pitwall.log('calib/v_meas', v)
        pitwall.log('calib/yaw_rate', w)
        if self._running:
            self._est_p0 = self._append_path(self._est_path, self._est_p0, msg)
            self.est_path_pub.publish(self._est_path)

    def _vesc_odom_cb(self, msg: Odometry):
        # vesc dead-reckoning: the EXPECTED path (where the current gains THINK
        # the car went). Only accumulated during an autodrive session.
        if self._running:
            self._exp_p0 = self._append_path(self._exp_path, self._exp_p0, msg)
            self.exp_path_pub.publish(self._exp_path)

    # ===== path viz helpers =============================================
    @staticmethod
    def _yaw_from_quat(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _append_path(self, path, p0, msg):
        """Append this odom pose to `path`, expressed RELATIVE to the session's
        first pose `p0` (so the path starts at the origin heading +x). Returns p0
        (setting it from this msg if it was None). Caps length at path_max_len."""
        p = msg.pose.pose
        yaw = self._yaw_from_quat(p.orientation)
        if p0 is None:
            p0 = (p.position.x, p.position.y, yaw)
        x0, y0, yaw0 = p0
        dx, dy = p.position.x - x0, p.position.y - y0
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        rx, ry = c * dx - s * dy, s * dx + c * dy
        ryaw = yaw - yaw0

        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = self.path_frame
        ps.pose.position.x = rx
        ps.pose.position.y = ry
        ps.pose.orientation.z = math.sin(ryaw / 2.0)
        ps.pose.orientation.w = math.cos(ryaw / 2.0)

        path.header.stamp = msg.header.stamp
        path.header.frame_id = self.path_frame
        path.poses.append(ps)
        if len(path.poses) > self.path_max_len:
            path.poses = path.poses[-self.path_max_len:]
        return p0

    def _reset_paths(self):
        self._exp_p0 = None
        self._est_p0 = None
        self._exp_path = Path()
        self._est_path = Path()
        self.exp_path_pub.publish(self._exp_path)
        self.est_path_pub.publish(self._est_path)

    def _imu_cb(self, msg: Imu):
        pitwall.log('calib/imu_yaw_rate', msg.angular_velocity.z)

    def _joy_cb(self, msg: Joy):
        # The joystick is the start/stop: a rising edge on the autodrive button
        # (which also arms the mux to forward our commands) starts the run; the
        # humandrive deadman aborts it (the mux simultaneously hands control back).
        if not self.joy_autostart_enable:
            return
        b = msg.buttons
        auto = bool(b[self.joy_autostart_button]) if len(b) > self.joy_autostart_button else False
        human = bool(b[self.joy_abort_button]) if len(b) > self.joy_abort_button else False
        if human and self._running:
            self._abort.set()
            pitwall.event("humandrive deadman - calibration aborted")
        if auto and not self._prev_auto:
            self._start_sequence()
        self._prev_auto = auto

    # ===== command republisher ==========================================
    def _cmd_tick(self):
        # running sequence owns the command; otherwise manual rqt drive if enabled;
        # otherwise idle (speed 0). The mux only forwards any of this when autodrive
        # is selected, and the joystick humandrive deadman overrides it.
        if self._running:
            with self._lock:
                speed, steer = self.cmd_speed, self.cmd_steer
        elif self.manual_enable:
            speed, steer = self.sp_speed, self.sp_steer
        else:
            speed, steer = 0.0, 0.0
        speed = max(-self.max_abs_speed, min(self.max_abs_speed, speed))
        steer = max(-self.max_abs_steer, min(self.max_abs_steer, steer))

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        self.cmd_pub.publish(msg)
        pitwall.log('calib/speed_cmd', speed)
        pitwall.log('calib/steer_cmd', steer)

    # ===== command helpers ==============================================
    def _set(self, speed=None, steer=None):
        with self._lock:
            if speed is not None:
                self.cmd_speed = speed
            if steer is not None:
                self.cmd_steer = steer

    def _ramp_speed(self, target):
        with self._lock:
            start = self.cmd_speed
        steps = max(1, int(self.ramp_time * 50))
        for i in range(1, steps + 1):
            if self._abort.is_set():
                return
            self._set(speed=start + (target - start) * i / steps)
            time.sleep(self.ramp_time / steps)

    def _stop(self):
        self._set(speed=0.0, steer=0.0)

    def _wait_steady(self):
        """Hold the command until the car has finished accelerating and the
        measured speed has plateaued. Open-loop ramp/settle alone can't know the
        real dynamics, so we watch odom: once the speed spread over a trailing
        `steady_window` stays within `steady_band`, it is steady. Bounded below by
        `settle_time` (always wait at least this) and above by `settle_timeout`."""
        time.sleep(self.settle_time)
        t_start = time.time()
        while not self._abort.is_set() and (time.time() - t_start) < self.settle_timeout:
            win = []
            t0 = time.time()
            while time.time() - t0 < self.steady_window:
                with self._lock:
                    win.append(self.last_v)
                time.sleep(0.05)
            if win and (max(win) - min(win)) <= self.steady_band:
                pitwall.log('calib/settle_extra_s', time.time() - t_start)
                return
        if not self._abort.is_set():
            self.get_logger().warn("speed did not plateau within %.1fs; measuring anyway"
                                   % self.settle_timeout)
            pitwall.event("settle timeout - measuring anyway")

    def _measure(self):
        """Wait for steady state, then measure ground speed & yaw rate over the
        window, POSE-first so it works uniformly across localizers regardless of
        which twist fields they populate:
          - yaw rate from the pose heading slope d(yaw)/dt (NOT twist.angular.z,
            which kiss_icp leaves at 0 -> drift would always read zero);
          - speed from twist.linear when present, else from pose arc-length/dt
            (some localizers publish pose only and leave twist.linear at 0).
        Falls back to the last twist sample if too few pose samples arrived."""
        self._wait_steady()
        with self._lock:
            self.collecting = True
            self._v_buf = []
            self._w_buf = []
            self._t_buf = []
            self._yaw_buf = []
            self._x_buf = []
            self._y_buf = []
        time.sleep(self.measure_time)
        with self._lock:
            self.collecting = False
            v_twist = float(np.mean(self._v_buf)) if self._v_buf else self.last_v
            t = np.array(self._t_buf)
            ya = np.array(self._yaw_buf)
            xb = np.array(self._x_buf)
            yb = np.array(self._y_buf)
            w_twist = float(np.mean(self._w_buf)) if self._w_buf else self.last_w

        # yaw rate: pose heading slope (robust); twist fallback if too few samples
        if len(ya) >= 3:
            w = float(np.polyfit(t - t[0], np.unwrap(ya), 1)[0])
        else:
            self.get_logger().warn("only %d pose samples in window; using twist yaw rate"
                                   % len(ya))
            w = w_twist

        # speed: prefer twist.linear; if it reads ~0 but the pose actually moved,
        # fall back to pose arc-length / elapsed time (uniform across localizers).
        v = v_twist
        if len(xb) >= 2 and (t[-1] - t[0]) > 1e-3:
            arc = float(np.sum(np.hypot(np.diff(xb), np.diff(yb))))
            v_pose = arc / (t[-1] - t[0])
            pitwall.log('calib/v_pose', v_pose)
            if abs(v_twist) < self.min_speed and v_pose >= self.min_speed:
                # sign from twist if it has any, else assume forward (commanded +)
                v = math.copysign(v_pose, v_twist if abs(v_twist) > 1e-9 else 1.0)
                self.get_logger().warn(
                    "twist.linear ~0 (%.3f); using pose-derived speed %.3f m/s"
                    % (v_twist, v))
        pitwall.log('calib/yaw_rate_pose', w)
        return v, w

    # ===== start / stop =================================================
    def _start_sequence(self):
        """Kick off the calibration run (idempotent while one is in progress)."""
        if self._running:
            return
        self._abort.clear()
        self._reset_paths()   # new autodrive session -> fresh expected/estimated paths
        self.get_logger().info("autodrive engaged -> starting calibration")
        pitwall.event("autodrive engaged - calibration started")
        threading.Thread(target=self._run_sequence, daemon=True).start()

    def _on_stop(self, req, resp):
        self._abort.set()
        self._stop()
        resp.success, resp.message = True, "abort requested; motion stopped"
        return resp

    def _on_reset(self, req, resp):
        self._last_results = None
        self._reset_paths()
        self.get_logger().info("reset: last result + paths cleared")
        pitwall.event("reset")
        resp.success, resp.message = True, "reset"
        return resp

    def _on_apply(self, req, resp):
        """Push the LATEST SINGLE-RUN estimate to BOTH vesc nodes (sync_nodes) LIVE,
        so the command path (ackermann_to_vesc) and the odometry path (vesc_to_odom)
        stay consistent. Needs the vesc nodes' set-parameters callbacks (added in
        this repo), so the new mapping takes effect immediately without a restart.
        A straight run pushes speed_to_erpm_gain + steering neutral offset; a turn
        run pushes steering_angle_to_servo_gain."""
        res = self._last_results
        if not res or not (res.get('speed') or res.get('steering')
                           or res.get('servo_offset')):
            resp.success, resp.message = False, "no result yet to apply"
            return resp
        from rclpy.parameter import Parameter
        D = Parameter.Type.DOUBLE
        nodes = list(self.get_parameter('sync_nodes').value)
        params, applied = [], {}
        sp = res.get('speed')
        if sp:
            params.append(Parameter('speed_to_erpm_gain', D, sp['speed_to_erpm_gain']))
            applied['speed_gain'] = sp['speed_to_erpm_gain']
        so = res.get('servo_offset')
        if so:
            params.append(Parameter('steering_angle_to_servo_offset', D,
                                    so['steering_angle_to_servo_offset']))
            applied['servo_offset'] = so['steering_angle_to_servo_offset']
        st = res.get('steering')
        if st:
            params.append(Parameter('steering_angle_to_servo_gain', D,
                                    st['steering_angle_to_servo_gain']))
            applied['steer_gain'] = st['steering_angle_to_servo_gain']
        ok = self._push_params(params, nodes)
        if ok:
            # our "current" gains are now these; the next run compares to them
            if 'speed_gain' in applied:
                self.cur_speed_gain = applied['speed_gain']
            if 'servo_offset' in applied:
                self.cur_steer_offset = applied['servo_offset']
            if 'steer_gain' in applied:
                self.cur_steer_gain = applied['steer_gain']
        resp.success = ok
        resp.message = ("applied %d params to %s" % (len(params), nodes)) if ok \
            else "some nodes rejected params (see log)"
        self.get_logger().info(resp.message)
        pitwall.event("applied calibration to %d vesc node(s)" % len(nodes))
        return resp

    # ===== save to vehicle_config.yaml ==================================
    def _on_save(self, req, resp):
        ok, msg = self._save_config()
        resp.success, resp.message = ok, msg
        return resp

    def _save_and_rearm(self):
        """Run the save (from the rqt save_config toggle) then reset the toggle."""
        ok, msg = self._save_config()
        self.get_logger().info("save_config toggle: %s" % msg)
        time.sleep(0.2)
        try:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter('save_config', Parameter.Type.BOOL, False)])
        except Exception:
            pass

    def _resolve_config_path(self):
        p = self.get_parameter('vehicle_config_path').value
        if p:
            return p
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('stack_master'),
                            'config', 'CAR', 'vehicle_config.yaml')

    def _save_config(self):
        """Read the CURRENT live gains from source_node, write them into
        vehicle_config.yaml's `vesc/**` block (preserving all other lines), and
        sync both vesc nodes so the live system matches the saved file. Returns
        (ok, message)."""
        src = self.get_parameter('source_node').value
        gains = self._fetch_gains(src, timeout=3.0)
        if gains is None:
            # fall back to our last-known cur_* values
            gains = {'speed_to_erpm_gain': self.cur_speed_gain,
                     'speed_to_erpm_offset': self.cur_speed_offset,
                     'steering_angle_to_servo_gain': self.cur_steer_gain,
                     'steering_angle_to_servo_offset': self.cur_steer_offset}
            self.get_logger().warn(
                "could not read live gains from %s; saving last-known cur_* values" % src)
        path = self._resolve_config_path()
        ok = self._write_yaml_gains(path, gains)
        if not ok:
            return False, "failed to write %s" % path
        # keep the live system consistent with what we just saved
        from rclpy.parameter import Parameter
        D = Parameter.Type.DOUBLE
        self._push_params([Parameter(n, D, gains[n]) for n in self.GAIN_NAMES],
                          list(self.get_parameter('sync_nodes').value))
        self.cur_speed_gain = gains['speed_to_erpm_gain']
        self.cur_speed_offset = gains['speed_to_erpm_offset']
        self.cur_steer_gain = gains['steering_angle_to_servo_gain']
        self.cur_steer_offset = gains['steering_angle_to_servo_offset']
        msg = ("saved vesc gains to %s (speed_gain=%.1f steer_gain=%.4f "
               "steer_offset=%.4f)" % (path, gains['speed_to_erpm_gain'],
                                       gains['steering_angle_to_servo_gain'],
                                       gains['steering_angle_to_servo_offset']))
        self.get_logger().info(msg)
        pitwall.event("saved vesc config -> vehicle_config.yaml")
        return True, msg

    def _write_yaml_gains(self, path, gains):
        """Rewrite the 4 gain lines inside the `vesc/**` block of `path` in place,
        leaving every other line (comments, other params) untouched."""
        try:
            with open(path) as f:
                lines = f.readlines()
            in_vesc = False
            for i, ln in enumerate(lines):
                # a non-indented "something:" line starts/ends a top-level section
                if ln and not ln[0].isspace() and ln.rstrip().endswith(':'):
                    in_vesc = ln.startswith('vesc/')
                    continue
                if not in_vesc:
                    continue
                m = re.match(r'^(\s*)([A-Za-z_]\w*):\s*.*$', ln)
                if m and m.group(2) in gains:
                    lines[i] = "%s%s: %s\n" % (m.group(1), m.group(2),
                                               repr(float(gains[m.group(2)])))
            with open(path, 'w') as f:
                f.writelines(lines)
            return True
        except Exception as e:
            self.get_logger().error("write %s failed: %s" % (path, e))
            return False

    # ===== routine ======================================================
    def _run_sequence(self):
        """Drive the ONE current setpoint (cmd_speed, cmd_steer) and report a
        SINGLE standalone result for that run (no accumulation). Change the
        setpoint and press autodrive again for a fresh, independent result.
          steering ~= 0 -> straight: speed_to_erpm_gain dir + servo neutral dir;
          steering != 0 -> turn:     steering_angle_to_servo_gain dir."""
        self._running = True
        try:
            v_cmd = max(-self.max_abs_speed, min(self.max_abs_speed, self.sp_speed))
            d_cmd = max(-self.max_abs_steer, min(self.max_abs_steer, self.sp_steer))
            straight = abs(d_cmd) <= self.steer_zero_tol
            self.get_logger().info("point: v=%.2f delta=%.3f (%s)"
                                   % (v_cmd, d_cmd, "straight" if straight else "turn"))
            pitwall.event("point v=%.2f delta=%.3f" % (v_cmd, d_cmd))

            self._set(steer=d_cmd)
            self._ramp_speed(v_cmd)
            v_meas, w = self._measure()
            self._stop()
            if self._abort.is_set():
                return
            if abs(v_meas) < self.min_speed:
                self.get_logger().warn("skipped (v=%.3f < min_speed); no sample recorded"
                                       % v_meas)
                return
            delta_meas = math.atan(w * self.wheelbase / v_meas)

            # ONE run -> ONE standalone result (no accumulation). pitwall shows
            # command / expected / measured / which config / RAISE-LOWER only.
            results = {}
            if straight:
                erpm_sent = self.cur_speed_gain * v_cmd + self.cur_speed_offset
                # the eRPM actually sent produced v_meas, so the true gain is
                # eRPM/v_meas (offset ~ 0): slower than commanded -> RAISE gain.
                g_est = erpm_sent / v_meas if abs(v_meas) > 1e-6 else self.cur_speed_gain
                # car curves at steering=0 -> servo neutral is off; shift the
                # offset by -gain*drift to zero the curvature.
                off_est = self.cur_steer_offset - self.cur_steer_gain * delta_meas
                # OK band for the neutral: drift under steer_zero_tol (rad) is fine.
                off_tol = abs(self.cur_steer_gain) * self.steer_zero_tol
                results['speed'] = {'speed_to_erpm_gain': float(g_est)}
                results['servo_offset'] = {
                    'steering_angle_to_servo_offset': float(off_est)}
                pitwall.log('calib/v_meas_pt', v_meas)
                pitwall.log('calib/drift_pt', delta_meas)
                self.get_logger().info(
                    "  straight: v_cmd=%.2f v_meas=%.3f drift=%.4f rad | "
                    "speed_to_erpm_gain %.1f->%.1f, servo_offset %.4f->%.4f"
                    % (v_cmd, v_meas, delta_meas, self.cur_speed_gain, g_est,
                       self.cur_steer_offset, off_est))
                pitwall.event("CMD speed=%.2f | expect=%.2f meas=%.2f m/s" %
                              (v_cmd, v_cmd, v_meas))
                pitwall.event("  -> speed_to_erpm_gain: %s" %
                              self._dir(self.cur_speed_gain, g_est))
                pitwall.event("  -> drift %.3frad -> steering_servo_offset: %s" %
                              (delta_meas, self._dir(self.cur_steer_offset, off_est, off_tol)))
            else:
                servo_sent = self.cur_steer_gain * d_cmd + self.cur_steer_offset
                # servo actually sent produced delta_meas -> true gain given the
                # current neutral; turns less than commanded -> RAISE |gain|.
                g_est = ((servo_sent - self.cur_steer_offset) / delta_meas
                         if abs(delta_meas) > 1e-6 else self.cur_steer_gain)
                results['steering'] = {'steering_angle_to_servo_gain': float(g_est)}
                pitwall.log('calib/delta_meas_pt', delta_meas)
                self.get_logger().info(
                    "  turn: d_cmd=%.3f delta_meas=%.4f | steering_angle_to_servo_gain "
                    "%.4f->%.4f" % (d_cmd, delta_meas, self.cur_steer_gain, g_est))
                pitwall.event("CMD steer=%.3f | expect=%.3f meas=%.3f rad" %
                              (d_cmd, d_cmd, delta_meas))
                pitwall.event("  -> steering_angle_to_servo_gain: %s" %
                              self._dir(self.cur_steer_gain, g_est))

            self._last_results = results   # remembered for ~/apply
        except Exception as e:
            self.get_logger().error("calibration failed: %s" % e)
        finally:
            self._stop()
            self._running = False

    @staticmethod
    def _dir(cur, est, tol=1e-6):
        """Direction a config value must move: RAISE / LOWER / OK (no magnitude)."""
        if abs(est - cur) <= tol:
            return "OK (no change)"
        return "RAISE" if est > cur else "LOWER"


def main(args=None):
    rclpy.init(args=args)
    node = VescCalibrationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._abort.set()
        node._stop()
        for _ in range(5):
            node._cmd_tick()
            time.sleep(0.01)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
