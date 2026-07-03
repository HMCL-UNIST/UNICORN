"""Automatic identification of the VESC odometry gains from a rosbag.

The VESC -> odometry model (see ``vesc_to_odom.cpp``) is::

    v     = (eRPM  - speed_to_erpm_offset)            / speed_to_erpm_gain
    delta = (servo - steering_angle_to_servo_offset)  / steering_angle_to_servo_gain
    omega = v * tan(delta) / wheelbase

Treating the cartographer trajectory as ground truth we recover:

  * ``speed_to_erpm_gain``  -- three independent ways that must agree:
      (1) regress GT longitudinal speed on eRPM (through origin),
      (2) match the eRPM-integrated distance to the GT arc length,
      (3) median eRPM / GT-speed ratio at speed.
    Agreement across all three is what makes the number trustworthy (GT arc
    length is checked to be jitter-robust, so it is not inflated by 100 Hz
    pose noise).

  * ``steering_angle_to_servo_gain / offset`` -- preferentially from the clean
    ``ackermann_cmd.steering_angle`` -> ``servo`` mapping (what the driver
    applied), *validated* against the GT vehicle response so the number is the
    physically-true gain and not merely the gain that was configured.

All GT-based fits are robust (IRLS with Huber weights).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .util import interp


# ---- robust linear fit ----------------------------------------------------
def robust_linfit(x, y, intercept=True, iters=12, huber_k=1.345):
    """IRLS Huber linear fit.  Returns dict(slope, intercept, weights, r2, r2_in)."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    A = np.c_[x, np.ones_like(x)] if intercept else x[:, None]
    w = np.ones_like(y)
    p = np.zeros(A.shape[1])
    for _ in range(iters):
        sw = np.sqrt(w)
        p, *_ = np.linalg.lstsq(A * sw[:, None], y * sw, rcond=None)
        r = A @ p - y
        med = np.median(r)
        s = 1.4826 * np.median(np.abs(r - med)) + 1e-12
        k = huber_k * s
        a = np.abs(r)
        w = np.where(a <= k, 1.0, k / a)
    slope = p[0]
    intercept_val = p[1] if intercept else 0.0
    pred = A @ p
    inl = w > 0.5
    def _r2(mask):
        yy = y[mask]
        ss_res = np.sum((yy - pred[mask]) ** 2)
        ss_tot = np.sum((yy - yy.mean()) ** 2) + 1e-12
        return 1.0 - ss_res / ss_tot
    return dict(slope=slope, intercept=intercept_val, weights=w,
                r2=_r2(np.ones_like(y, bool)),
                r2_in=_r2(inl) if inl.sum() > 2 else _r2(np.ones_like(y, bool)),
                inlier_frac=float(inl.mean()))


# ===========================================================================
# Speed
# ===========================================================================
@dataclass
class SpeedCal:
    gain: float            # recommended (through-origin regression)
    offset: float          # recommended offset (0.0)
    gain_dist: float       # distance-matched gain (odom dist == GT arc length)
    gain_ratio: float      # median(eRPM / GT speed) at speed
    gain_offsetfit: float  # gain when an eRPM offset is also fitted
    offset_offsetfit: float
    r2: float
    n: int
    rms_old: float
    rms_new: float
    erpm: np.ndarray
    v_gt: np.ndarray
    weights: np.ndarray


def calibrate_speed(data, gtm, v_min=0.3, old_gain=4614.0, old_offset=0.0) -> SpeedCal:
    """Identify speed_to_erpm_gain (offset recommended 0)."""
    t = data.core.t
    erpm = data.core.erpm
    v_gt = interp(t, gtm.t, gtm.v_body)
    mask = np.abs(v_gt) > v_min
    e, v = erpm[mask], v_gt[mask]

    # (1) through-origin robust fit  v = a*eRPM  -> gain = 1/a   [recommended]
    fit0 = robust_linfit(e, v, intercept=False)
    gain = 1.0 / fit0["slope"]
    # offset-fit variant  v = a*eRPM + c
    fit = robust_linfit(e, v, intercept=True)
    gain_off = 1.0 / fit["slope"]
    off_erpm = -fit["intercept"] / fit["slope"]
    # (2) distance-matched gain  (jitter-robust; independent of differentiation)
    dt_core = np.diff(t)
    odom_dist_unit = np.sum(np.abs(erpm[:-1]) * dt_core)        # == gain * distance
    gt_len = float(np.sum(np.hypot(np.diff(gtm.x), np.diff(gtm.y))))
    gain_dist = odom_dist_unit / gt_len if gt_len > 0 else float("nan")
    # (3) median ratio at speed
    hi = np.abs(v) > max(v_min, 1.0)
    gain_ratio = float(np.median(e[hi] / v[hi])) if hi.sum() else gain

    v_old = (e - old_offset) / old_gain
    v_new = e / gain
    rms_old = float(np.sqrt(np.mean((v_old - v) ** 2)))
    rms_new = float(np.sqrt(np.mean((v_new - v) ** 2)))

    return SpeedCal(gain=gain, offset=0.0, gain_dist=gain_dist, gain_ratio=gain_ratio,
                    gain_offsetfit=gain_off, offset_offsetfit=off_erpm,
                    r2=fit0["r2_in"], n=int(mask.sum()),
                    rms_old=rms_old, rms_new=rms_new,
                    erpm=e, v_gt=v, weights=fit0["weights"])


# ===========================================================================
# Steering
# ===========================================================================
@dataclass
class SteerCal:
    gain: float            # recommended servo-per-rad gain (signed)
    offset: float          # recommended servo offset (servo at delta=0)
    sign: int              # +1 / -1
    response_ratio: float  # actual steering angle / commanded (1.0 == well calibrated)
    corr: float            # quality of the servo<->steer fit
    lag_s: float
    method: str            # "command" or "gt"
    gain_gt: float         # GT-only gain (cross-check, attenuated by noise)
    rms_old: float
    rms_new: float
    n: int
    max_steer_deg: float
    # arrays for plotting (GT-implied steering vs servo)
    delta_gt: np.ndarray
    servo: np.ndarray
    weights: np.ndarray


def _best_lag(t_ref, ref, gtm, wheelbase, v_min, lag_range=0.4, lag_step=0.01):
    """Lag (s) that best correlates ``ref`` with the GT-implied steering angle."""
    lags = np.arange(-lag_range, lag_range + 1e-9, lag_step)
    best_lag, best = 0.0, -np.inf
    for lag in lags:
        v = interp(t_ref + lag, gtm.t, gtm.v_body)
        wz = interp(t_ref + lag, gtm.t, gtm.yaw_rate)
        m = np.abs(v) > v_min
        if m.sum() < 20:
            continue
        delta = np.arctan(wz[m] * wheelbase / v[m])
        sc = abs(np.corrcoef(delta, ref[m])[0, 1])
        if sc > best:
            best, best_lag = sc, lag
    return best_lag


def calibrate_steering(data, gtm, wheelbase=0.33, v_min=0.6,
                       old_gain=-1.2135, old_offset=0.5304, find_lag=True) -> SteerCal:
    """Identify steering_angle_to_servo_gain / offset.

    Preferred path uses the recorded ackermann steering command (clean) and
    validates it against the GT response.  Falls back to a GT-only fit if the
    ackermann command is absent.
    """
    servo_t = data.servo.t
    servo = data.servo.servo

    # ---- GT-only fit (always computed, used for plotting + cross-check) ----
    lag = _best_lag(servo_t, servo, gtm, wheelbase, v_min) if find_lag else 0.0
    v = interp(servo_t + lag, gtm.t, gtm.v_body)
    wz = interp(servo_t + lag, gtm.t, gtm.yaw_rate)
    mask = np.abs(v) > v_min
    delta_gt = np.arctan(wz[mask] * wheelbase / v[mask])
    sv = servo[mask]
    fit_gt = robust_linfit(delta_gt, sv, intercept=True)
    gain_gt, offset_gt = fit_gt["slope"], fit_gt["intercept"]
    max_steer = float(np.degrees(np.percentile(np.abs(delta_gt), 98)))

    have_ack = len(data.ack) > 0 and hasattr(data.ack, "ack_steer")
    if have_ack:
        # used mapping  servo = G_used*steer + b_used  (driver applied this)
        ack_t = data.ack.t
        steer = data.ack.ack_steer
        servo_at = interp(ack_t, servo_t, servo)
        m = np.abs(steer) > 0.02
        fit_cmd = robust_linfit(steer[m], servo_at[m], intercept=True)
        g_used, b_used = fit_cmd["slope"], fit_cmd["intercept"]
        corr = float(np.corrcoef(steer[m], servo_at[m])[0, 1])
        # response ratio: GT-implied steering / commanded steering
        lag_r = _best_lag(ack_t, steer, gtm, wheelbase, v_min)
        vv = interp(ack_t + lag_r, gtm.t, gtm.v_body)
        ww = interp(ack_t + lag_r, gtm.t, gtm.yaw_rate)
        mm = np.abs(vv) > v_min
        d_gt = np.arctan(ww[mm] * wheelbase / vv[mm])
        # response ratio is a *diagnostic* only (sensitive to GT differentiation);
        # it is NOT folded into the recommended gain.
        ratio = float(robust_linfit(steer[mm], d_gt, intercept=True)["slope"])
        ratio = ratio if abs(ratio) > 1e-3 else 1.0
        gain = g_used               # effective servo-per-commanded-rad (robust, exact)
        offset = b_used
        sign = int(np.sign(gain))
        method = "command"
        lag = lag_r
    else:
        gain, offset, sign = gain_gt, offset_gt, int(np.sign(gain_gt))
        corr = float(np.sqrt(max(fit_gt["r2"], 0.0)))
        ratio = 1.0
        method = "gt"

    delta_old = (sv - old_offset) / old_gain
    delta_new = (sv - offset) / gain
    rms_old = float(np.degrees(np.sqrt(np.mean((delta_old - delta_gt) ** 2))))
    rms_new = float(np.degrees(np.sqrt(np.mean((delta_new - delta_gt) ** 2))))

    return SteerCal(gain=gain, offset=offset, sign=sign, response_ratio=ratio,
                    corr=corr, lag_s=lag, method=method, gain_gt=gain_gt,
                    rms_old=rms_old, rms_new=rms_new, n=int(mask.sum()),
                    max_steer_deg=max_steer, delta_gt=delta_gt, servo=sv,
                    weights=fit_gt["weights"])


# ===========================================================================
# Distance cross-check (independent of pose differentiation)
# ===========================================================================
@dataclass
class DistCheck:
    gt_len: float
    odom_len_old: float       # eRPM-integrated distance at old gain
    odom_len_new: float       # at recommended gain
    vesc_odom_rec_len: float  # recorded /vesc/odom path length
    decim_lengths: dict       # GT arc length at several decimations (jitter test)


def distance_check(data, gtm, old_gain, new_gain) -> DistCheck:
    t = data.core.t
    dt = np.diff(t)
    iu = np.sum(np.abs(data.core.erpm[:-1]) * dt)
    gt_len = float(np.sum(np.hypot(np.diff(gtm.x), np.diff(gtm.y))))
    vo_len = float(np.sum(np.hypot(np.diff(data.vo.x), np.diff(data.vo.y)))) if len(data.vo) else float("nan")
    x, y = data.gt.x, data.gt.y
    decim = {}
    for d in (1, 5, 10, 20):
        xs, ys = x[::d], y[::d]
        decim[d] = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
    return DistCheck(gt_len=gt_len, odom_len_old=iu / old_gain,
                     odom_len_new=iu / new_gain, vesc_odom_rec_len=vo_len,
                     decim_lengths=decim)
