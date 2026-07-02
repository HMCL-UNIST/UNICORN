"""Offline dead-reckoning / sensor-fusion experiments.

Each estimator integrates a 2-D trajectory from a *speed* source and a *heading*
source, all resampled onto the 50 Hz VESC timeline.  Comparing their drift
against GT answers the practical question: does using the AHRS orientation (or
the gyro) for heading beat the steering-model heading that ``/vesc/odom`` uses?

All integrators are seeded at the GT pose at the start time, so the only thing
that differs between them is the heading (and speed-gain) source -- the
accumulated error is then a fair measure of that source's quality.  This mirrors
what ``robot_localization`` does when it fuses the VESC twist with the IMU.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .util import interp, interp_angle, unwrap, wrap, align_yaw_offset


@dataclass
class Traj:
    name: str
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray


def _integrate(t, v, yaw, x0, y0):
    """Euler-integrate position from speed v and heading yaw."""
    x = np.empty_like(t)
    y = np.empty_like(t)
    x[0], y[0] = x0, y0
    dt = np.diff(t)
    for k in range(len(t) - 1):
        x[k + 1] = x[k] + v[k] * np.cos(yaw[k]) * dt[k]
        y[k + 1] = y[k] + v[k] * np.sin(yaw[k]) * dt[k]
    return x, y


def _integrate_yaw(t, yaw_rate, yaw0):
    yaw = np.empty_like(t)
    yaw[0] = yaw0
    dt = np.diff(t)
    yaw[1:] = yaw0 + np.cumsum(yaw_rate[:-1] * dt)
    return yaw


def build_estimators(data, gtm, speed_cal, steer_cal, wheelbase=0.33,
                     old_gain=4614.0, old_servo_gain=-1.2135,
                     old_servo_off=0.5304, init_align_s=2.0):
    """Return a list of :class:`Traj`, one per heading/speed strategy.

    All share the 50 Hz core timeline and start at the GT pose at t[0].
    """
    t = data.core.t
    # seed pose from GT at start
    x0 = float(interp(t[0], gtm.t, gtm.x))
    y0 = float(interp(t[0], gtm.t, gtm.y))
    yaw0 = float(interp_angle(t[0], gtm.t, gtm.yaw))

    erpm = data.core.erpm
    servo = interp(t, data.servo.t, data.servo.servo)
    gyro = interp(t, data.imu.t, data.imu.gyro_z)
    ahrs = interp_angle(t, data.imu.t, data.imu.yaw)

    v_old = (erpm - 0.0) / old_gain
    v_new = (erpm - speed_cal.offset) / speed_cal.gain

    # steering-model yaw rate (current vs calibrated gains)
    delta_old = (servo - old_servo_off) / old_servo_gain
    delta_new = (servo - steer_cal.offset) / steer_cal.gain
    wz_old = v_old * np.tan(delta_old) / wheelbase
    wz_new = v_new * np.tan(delta_new) / wheelbase

    # AHRS absolute heading: sign from full-record correlation, offset from the
    # initial window only (realistic: you align the AHRS to the map at startup).
    init = t <= (t[0] + init_align_s)
    gt_init_yaw = interp_angle(t[init], gtm.t, gtm.yaw)
    _, sign = align_yaw_offset(ahrs, interp_angle(t, gtm.t, gtm.yaw))
    diff = unwrap(gt_init_yaw) - sign * unwrap(ahrs[init])
    off = np.arctan2(np.sin(diff).mean(), np.cos(diff).mean())
    ahrs_heading = wrap(sign * unwrap(ahrs) + off)

    # complementary filter: gyro (fast) corrected toward AHRS (slow, drift-free)
    yaw_comp = _complementary(t, gyro, ahrs_heading, yaw0, tau=1.0)

    out = []

    def add(name, v, yaw):
        x, y = _integrate(t, v, yaw, x0, y0)
        out.append(Traj(name, t, x, y, yaw))

    # 1. steering model, current gains (== /vesc/odom)
    add("kin_old", v_old, _integrate_yaw(t, wz_old, yaw0))
    # 2. steering model, calibrated gains
    add("kin_calib", v_new, _integrate_yaw(t, wz_new, yaw0))
    # 3. gyro heading + calibrated speed
    add("gyro", v_new, _integrate_yaw(t, gyro, yaw0))
    # 4. AHRS absolute heading + calibrated speed  (the hypothesis)
    add("ahrs", v_new, ahrs_heading)
    # 5. complementary (gyro + AHRS) + calibrated speed
    add("ahrs_gyro", v_new, yaw_comp)
    return out


def _complementary(t, gyro, ahrs_heading, yaw0, tau=1.0):
    """Complementary filter: integrate gyro, bleed toward the AHRS heading."""
    yaw = np.empty_like(t)
    yaw[0] = yaw0
    dt = np.diff(t)
    for k in range(len(t) - 1):
        pred = yaw[k] + gyro[k] * dt[k]
        alpha = tau / (tau + dt[k])
        err = wrap(ahrs_heading[k + 1] - pred)
        yaw[k + 1] = pred + (1 - alpha) * err
    return yaw
