#!/usr/bin/env python3
"""
online_tuner_node — PASSIVE, online VESC gain identification WHILE A HUMAN DRIVES.

Unlike stack_master/scripts/vesc_calibration_node.py (which ACTIVELY commands the
car one autodrive point at a time), this node NEVER commands the car. It only
listens while you drive by hand and, every ~1.5 s, fits the three VESC-mapping
gains over a trailing window and streams them to pitwall:

  speed_to_erpm_gain            eRPM  = gain * v              (through the origin)
  steering_angle_to_servo_gain  servo = gain * delta + offset
  steering_angle_to_servo_offset

The model and the estimator are SHARED with the offline tool (odom_calib): the
IRLS Huber fit `robust_linfit` from odom_calib.calibrate does the work, so the
live numbers are computed exactly the same way as the rosbag analysis. That is
the "fusion" — one estimator, used offline on a bag and online on a live graph.

Inputs (all observed, nothing commanded):
  /car_state/odom                        external odom = ground-truth v, yaw-rate
  /vesc/sensors/core                     eRPM (VescState.speed)
  /vesc/sensors/servo_position_command   servo command actually sent (Float64)

Fit (same inversion as vesc_to_odom.cpp):
  SPEED  — regress eRPM on GT speed v (|v|>v_min) through the origin
           -> speed_to_erpm_gain = eRPM / v.
  STEER  — recover the ACTUAL wheel angle from GT motion,
           delta_gt = atan(yaw_rate * wheelbase / v), then regress the servo
           command on delta_gt -> gain = slope, offset = intercept (neutral).
           This needs real turning: if the window carries too little steering
           excitation (spread of delta_gt), the steering estimate is HELD and
           reported as "waiting for turns" — straight-line driving carries no
           steering information.

Every stream is time-stamped at RECEIPT on the node clock (one consistent clock,
like the bag's log-time) because the servo Float64 has no header. Estimates are
EWMA-smoothed across windows for a steady readout and published as
/pitwall/online_calib/* plus a one-line /pitwall/events summary each tick.

Display-only: it does NOT push or save gains. Read the numbers, then use
rqt_reconfigure on the vesc nodes (or vesc_calibration_node's ~/apply / ~/save)
to apply them.
"""

import math
import threading
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped

# Shared estimator with the offline odom_calib analysis -> identical math.
from odom_calib.calibrate import robust_linfit
from odom_calib.util import interp

# pitwall is optional — if the overlay isn't built the node still runs, telemetry
# just degrades to a no-op shim (same pattern as vesc_calibration_node).
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


class OnlineVescTuner(Node):

    def __init__(self):
        super().__init__('online_vesc_tuner')
        gp = self.declare_parameter

        # --- topology ----------------------------------------------------
        gp('odom_topic', '/car_state/odom')                    # ground-truth v, yaw-rate
        gp('core_topic', '/vesc/sensors/core')                 # eRPM (VescState.speed)
        gp('servo_topic', '/vesc/sensors/servo_position_command')
        gp('wheelbase', 0.33)

        # --- windowing / cadence ----------------------------------------
        gp('window_s', 6.0)          # trailing window (s) each fit runs over
        gp('publish_period', 1.5)    # s between estimate updates (1-2 s)
        gp('max_stale', 1.0)         # s; skip a fit if its newest sample is older

        # --- fit gates ---------------------------------------------------
        gp('v_min', 0.5)             # m/s min |speed| for the speed fit
        gp('v_min_steer', 0.8)       # m/s min |speed| for the steering fit
        gp('min_samples', 40)        # min points in a window to attempt a fit
        gp('steer_excite_min', 0.05) # rad; min spread of delta_gt to trust steering
        gp('ewma_alpha', 0.4)        # smoothing of published estimates (0..1)

        v = lambda n: self.get_parameter(n).value
        self.wheelbase = float(v('wheelbase'))
        self.window_s = float(v('window_s'))
        self.max_stale = float(v('max_stale'))
        self.v_min = float(v('v_min'))
        self.v_min_steer = float(v('v_min_steer'))
        self.min_samples = int(v('min_samples'))
        self.steer_excite_min = float(v('steer_excite_min'))
        self.alpha = float(v('ewma_alpha'))
        period = float(v('publish_period'))

        # --- trailing buffers, timestamped at receipt on the node clock --
        self._lock = threading.Lock()
        self._odom = deque()   # (t, v, wz, yaw)
        self._core = deque()   # (t, erpm)
        self._servo = deque()  # (t, servo)

        # --- EWMA-smoothed published estimates (None until first valid fit)
        self._sg = None        # speed_to_erpm_gain
        self._kg = None        # steering_angle_to_servo_gain
        self._ko = None        # steering_angle_to_servo_offset

        self.create_subscription(Odometry, v('odom_topic'), self._odom_cb, 30)
        self.create_subscription(VescStateStamped, v('core_topic'), self._core_cb, 50)
        self.create_subscription(Float64, v('servo_topic'), self._servo_cb, 50)
        self.create_timer(period, self._on_tick)

        pitwall.init(self)
        self.get_logger().info(
            "online_vesc_tuner ready (PASSIVE — never commands the car). Drive by "
            "hand; every %.1fs I fit speed_to_erpm_gain and steering_angle_to_servo_"
            "gain/offset over the last %.1fs and stream them to /pitwall/online_calib/"
            "*. TURN the car to get a steering estimate." % (period, self.window_s))

    # ===== clock / buffering ===========================================
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _trim(self, buf, t_new):
        cut = t_new - self.window_s
        while buf and buf[0][0] < cut:
            buf.popleft()

    @staticmethod
    def _yaw_from_quat(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ===== subscriptions ================================================
    def _odom_cb(self, msg: Odometry):
        t = self._now()
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = math.copysign(math.hypot(vx, vy), vx)
        wz = msg.twist.twist.angular.z
        yaw = self._yaw_from_quat(msg.pose.pose.orientation)
        with self._lock:
            self._odom.append((t, v, wz, yaw))
            self._trim(self._odom, t)

    def _core_cb(self, msg: VescStateStamped):
        t = self._now()
        with self._lock:
            self._core.append((t, float(msg.state.speed)))
            self._trim(self._core, t)

    def _servo_cb(self, msg: Float64):
        t = self._now()
        with self._lock:
            self._servo.append((t, float(msg.data)))
            self._trim(self._servo, t)

    # ===== fit + publish ================================================
    def _ewma(self, prev, new):
        return new if prev is None else (1.0 - self.alpha) * prev + self.alpha * new

    def _fresh(self, buf, t_ref):
        """A buffer is usable only if it has enough points AND its newest sample
        is recent (a stopped publisher must not feed stale data into interp)."""
        return len(buf) >= self.min_samples and (t_ref - buf[-1][0]) <= self.max_stale

    def _on_tick(self):
        t_ref = self._now()
        with self._lock:
            odom = list(self._odom)
            core = list(self._core)
            servo = list(self._servo)

        if not self._fresh(odom, t_ref) or not self._fresh(core, t_ref):
            self.get_logger().info(
                "warming up / no fresh motion (odom=%d core=%d servo=%d)"
                % (len(odom), len(core), len(servo)))
            return

        ot = np.array([r[0] for r in odom])
        ov = np.array([r[1] for r in odom])
        ow = np.array([r[2] for r in odom])
        oyaw = np.array([r[3] for r in odom])
        # Some localizers (e.g. kiss_icp) leave twist.angular.z at 0; if so,
        # derive yaw-rate from the pose-heading slope instead.
        if np.max(np.abs(ow)) < 1e-6 and len(ot) >= 3:
            ow = np.gradient(np.unwrap(oyaw), ot)

        sg_now, sg_r2, sg_n = self._fit_speed(core, ot, ov)
        kg_now, ko_now, excite, sk_n, steer_ok = self._fit_steer(servo, t_ref, ot, ov, ow)
        self._publish(sg_r2, sg_n, excite, sk_n, steer_ok)

    def _fit_speed(self, core, ot, ov):
        """eRPM = speed_to_erpm_gain * v, fitted through the origin (offset 0)."""
        ct = np.array([r[0] for r in core])
        erpm = np.array([r[1] for r in core])
        v_at = interp(ct, ot, ov)
        m = np.abs(v_at) > self.v_min
        n = int(m.sum())
        if n < self.min_samples:
            return None, 0.0, n
        # v = a * eRPM through origin -> gain = 1/a  (matches calibrate_speed)
        fit = robust_linfit(erpm[m], v_at[m], intercept=False)
        if abs(fit['slope']) <= 1e-12:
            return None, 0.0, n
        sg = 1.0 / fit['slope']
        self._sg = self._ewma(self._sg, sg)
        return sg, float(fit['r2_in']), n

    def _fit_steer(self, servo, t_ref, ot, ov, ow):
        """servo = gain * delta_gt + offset, where delta_gt is the GT-implied
        wheel angle atan(yaw_rate * wheelbase / v). Needs real turning."""
        if not self._fresh(servo, t_ref):
            return None, None, 0.0, 0, False
        st = np.array([r[0] for r in servo])
        sv = np.array([r[1] for r in servo])
        v_at = interp(st, ot, ov)
        w_at = interp(st, ot, ow)
        m = np.abs(v_at) > self.v_min_steer
        n = int(m.sum())
        if n < self.min_samples:
            return None, None, 0.0, n, False
        delta = np.arctan(w_at[m] * self.wheelbase / v_at[m])
        # robust spread (5-95 pct) as the excitation measure
        excite = float(np.percentile(delta, 95) - np.percentile(delta, 5))
        if excite < self.steer_excite_min:
            return None, None, excite, n, False   # too straight -> hold last estimate
        fit = robust_linfit(delta, sv[m], intercept=True)
        self._kg = self._ewma(self._kg, fit['slope'])
        self._ko = self._ewma(self._ko, fit['intercept'])
        return fit['slope'], fit['intercept'], excite, n, True

    def _publish(self, sg_r2, sg_n, excite, sk_n, steer_ok):
        if self._sg is not None:
            pitwall.log('online_calib/speed_to_erpm_gain', self._sg)
            pitwall.log('online_calib/speed_fit_r2', sg_r2)
            pitwall.log('online_calib/speed_n', float(sg_n))
        if self._kg is not None:
            pitwall.log('online_calib/steering_angle_to_servo_gain', self._kg)
            pitwall.log('online_calib/steering_angle_to_servo_offset', self._ko)
            pitwall.log('online_calib/steer_excite_rad', excite)
            pitwall.log('online_calib/steer_n', float(sk_n))

        sg = "%.1f" % self._sg if self._sg is not None else "--"
        if self._kg is not None:
            steer = "gain=%.4f off=%.4f" % (self._kg, self._ko)
            if not steer_ok:
                steer += " (held; drive turns)"
        else:
            steer = "waiting for turns"
        line = "ONLINE erpm_gain=%s | servo %s" % (sg, steer)
        self.get_logger().info(line)
        pitwall.event(line)


def main(args=None):
    rclpy.init(args=args)
    node = OnlineVescTuner()
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
