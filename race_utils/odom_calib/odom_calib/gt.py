"""Derive ground-truth motion (body velocity, yaw rate) from the cartographer
``/tracked_pose`` trajectory.

The pose is differentiated with a Savitzky-Golay filter, which fits a local
polynomial and returns its analytic derivative -- far less noisy than a raw
finite difference on 100 Hz pose data.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.signal import savgol_filter

from .util import unwrap


@dataclass
class GtMotion:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray          # wrapped
    yaw_unwrapped: np.ndarray
    vx_world: np.ndarray
    vy_world: np.ndarray
    v_body: np.ndarray       # longitudinal speed along heading (signed)
    v_lat: np.ndarray        # lateral (side-slip) speed
    speed: np.ndarray        # |world velocity|
    yaw_rate: np.ndarray     # d(yaw)/dt


def _savgol_deriv(x, dt, window, poly):
    """Smoothed value and first derivative of x sampled at step dt."""
    window = min(window, len(x) if len(x) % 2 else len(x) - 1)
    if window < poly + 2:
        window = poly + 2 + ((poly + 2) % 2 == 0)  # make odd
        window = min(window, len(x) - 1 if len(x) % 2 == 0 else len(x))
    if window % 2 == 0:
        window -= 1
    window = max(window, poly + 2 | 1)
    smooth = savgol_filter(x, window, poly)
    deriv = savgol_filter(x, window, poly, deriv=1, delta=dt)
    return smooth, deriv


def compute_gt_motion(gt, smooth_window=0.15, poly=2) -> GtMotion:
    """Compute GT velocity / yaw-rate from a tracked-pose :class:`Series`.

    ``smooth_window`` is the Savitzky-Golay window length in *seconds*.
    """
    t = np.asarray(gt.t, dtype=float)
    dt = float(np.median(np.diff(t)))
    win = max(int(round(smooth_window / dt)), poly + 2)
    if win % 2 == 0:
        win += 1

    xs, vx = _savgol_deriv(gt.x, dt, win, poly)
    ys, vy = _savgol_deriv(gt.y, dt, win, poly)
    yaw_u = unwrap(gt.yaw)
    yaw_s, yaw_rate = _savgol_deriv(yaw_u, dt, win, poly)

    cos_y = np.cos(yaw_s)
    sin_y = np.sin(yaw_s)
    v_body = vx * cos_y + vy * sin_y         # longitudinal
    v_lat = -vx * sin_y + vy * cos_y         # lateral
    speed = np.hypot(vx, vy)

    return GtMotion(
        t=t, x=xs, y=ys,
        yaw=(yaw_s + np.pi) % (2 * np.pi) - np.pi,
        yaw_unwrapped=yaw_s,
        vx_world=vx, vy_world=vy,
        v_body=v_body, v_lat=v_lat, speed=speed,
        yaw_rate=yaw_rate,
    )
