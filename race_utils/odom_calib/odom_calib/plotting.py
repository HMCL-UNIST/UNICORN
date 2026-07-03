"""Figure generation for the calibration / evaluation report."""
from __future__ import annotations

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .util import interp, interp_angle, se2_align, wrap


def _save(fig, outdir, name):
    path = os.path.join(outdir, name)
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return path


def plot_speed_cal(sc, outdir, old_gain=4614.0):
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    w = sc.weights
    ax[0].scatter(sc.erpm, sc.v_gt, s=6, c=w, cmap="viridis", alpha=0.6)
    xs = np.linspace(sc.erpm.min(), sc.erpm.max(), 100)
    ax[0].plot(xs, (xs - sc.offset) / sc.gain, "r-", lw=2,
               label=f"calib gain={sc.gain:.1f}")
    ax[0].plot(xs, xs / old_gain, "k--", lw=1.5, label=f"current gain={old_gain:.1f}")
    ax[0].set_xlabel("eRPM (core.speed)")
    ax[0].set_ylabel("GT longitudinal speed [m/s]")
    ax[0].set_title(f"Speed gain fit  (R²={sc.r2:.3f}, n={sc.n})")
    ax[0].legend(); ax[0].grid(alpha=0.3)

    res_new = (sc.erpm - sc.offset) / sc.gain - sc.v_gt
    res_old = sc.erpm / old_gain - sc.v_gt
    ax[1].scatter(sc.v_gt, res_old, s=6, c="k", alpha=0.4, label=f"current (RMS={sc.rms_old:.3f})")
    ax[1].scatter(sc.v_gt, res_new, s=6, c="r", alpha=0.4, label=f"calib (RMS={sc.rms_new:.3f})")
    ax[1].axhline(0, color="gray", lw=0.8)
    ax[1].set_xlabel("GT speed [m/s]"); ax[1].set_ylabel("speed error [m/s]")
    ax[1].set_title("Velocity residual"); ax[1].legend(); ax[1].grid(alpha=0.3)
    return _save(fig, outdir, "01_speed_calibration.png")


def plot_steer_cal(stc, outdir, old_gain=-1.2135, old_off=0.5304):
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    w = stc.weights
    d_deg = np.degrees(stc.delta_gt)
    ax[0].scatter(d_deg, stc.servo, s=6, c=w, cmap="viridis", alpha=0.6)
    xs = np.linspace(d_deg.min(), d_deg.max(), 100)
    xr = np.radians(xs)
    ax[0].plot(xs, stc.gain * xr + stc.offset, "r-", lw=2,
               label=f"calib G={stc.gain:.4f} b={stc.offset:.4f}")
    ax[0].plot(xs, old_gain * xr + old_off, "k--", lw=1.5,
               label=f"current G={old_gain:.4f} b={old_off:.4f}")
    ax[0].set_xlabel("GT-implied steering angle [deg]")
    ax[0].set_ylabel("servo command")
    ax[0].set_title(f"Steering fit (corr={stc.corr:.3f}, lag={stc.lag_s*1000:.0f} ms, n={stc.n})")
    ax[0].legend(); ax[0].grid(alpha=0.3)

    res_new = np.degrees((stc.servo - stc.offset) / stc.gain - stc.delta_gt)
    res_old = np.degrees((stc.servo - old_off) / old_gain - stc.delta_gt)
    ax[1].scatter(d_deg, res_old, s=6, c="k", alpha=0.4, label=f"current (RMS={stc.rms_old:.2f}°)")
    ax[1].scatter(d_deg, res_new, s=6, c="r", alpha=0.4, label=f"calib (RMS={stc.rms_new:.2f}°)")
    ax[1].axhline(0, color="gray", lw=0.8)
    ax[1].set_xlabel("steering angle [deg]"); ax[1].set_ylabel("steering error [deg]")
    ax[1].set_title("Steering residual"); ax[1].legend(); ax[1].grid(alpha=0.3)
    return _save(fig, outdir, "02_steering_calibration.png")


def plot_gt_motion(data, gtm, sc, outdir, old_gain=4614.0):
    fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    v_meas_new = (data.core.erpm - sc.offset) / sc.gain
    v_meas_old = data.core.erpm / old_gain
    ax[0].plot(gtm.t, gtm.v_body, "b-", lw=1.2, label="GT v_body")
    ax[0].plot(data.core.t, v_meas_old, "k-", lw=0.8, alpha=0.6, label="eRPM/current")
    ax[0].plot(data.core.t, v_meas_new, "r-", lw=0.8, alpha=0.8, label="eRPM/calib")
    ax[0].set_ylabel("speed [m/s]"); ax[0].legend(ncol=3); ax[0].grid(alpha=0.3)
    ax[0].set_title("GT motion vs VESC measurement")

    ax[1].plot(gtm.t, np.degrees(gtm.yaw_rate), "b-", lw=1.2, label="GT yaw-rate")
    ax[1].plot(data.imu.t, np.degrees(data.imu.gyro_z), "g-", lw=0.7, alpha=0.6, label="IMU gyro_z")
    ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("yaw-rate [deg/s]")
    ax[1].legend(ncol=2); ax[1].grid(alpha=0.3)
    return _save(fig, outdir, "03_gt_motion.png")


def plot_heading(data, gtm, trajs, outdir):
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(gtm.t, np.degrees(np.unwrap(gtm.yaw)), "b-", lw=1.6, label="GT yaw")
    colors = {"kin_old": "k", "kin_calib": "orange", "gyro": "g", "ahrs": "r", "ahrs_gyro": "m"}
    for tr in trajs:
        ax.plot(tr.t, np.degrees(np.unwrap(tr.yaw)), color=colors.get(tr.name, "gray"),
                lw=1.0, alpha=0.8, label=tr.name)
    ax.set_xlabel("t [s]"); ax.set_ylabel("heading [deg]")
    ax.set_title("Heading source comparison"); ax.legend(ncol=3); ax.grid(alpha=0.3)
    return _save(fig, outdir, "04_heading.png")


def plot_trajectories(gtm, vo, trajs, outdir):
    fig, ax = plt.subplots(figsize=(8.5, 8))
    ax.plot(gtm.x, gtm.y, "b-", lw=2.5, label="GT (cartographer)", zorder=10)
    # recorded vesc/odom, umeyama-aligned for shape comparison
    if len(vo):
        _, _, _, al = se2_align(np.c_[vo.x, vo.y], np.c_[
            interp(vo.t, gtm.t, gtm.x), interp(vo.t, gtm.t, gtm.y)])
        ax.plot(al[:, 0], al[:, 1], "c--", lw=1.2, alpha=0.7, label="vesc/odom (aligned)")
    colors = {"kin_old": "k", "kin_calib": "orange", "gyro": "g", "ahrs": "r", "ahrs_gyro": "m"}
    for tr in trajs:
        ax.plot(tr.x, tr.y, color=colors.get(tr.name, "gray"), lw=1.2, alpha=0.85, label=tr.name)
    ax.plot(gtm.x[0], gtm.y[0], "ko", ms=8, label="start")
    ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Trajectories (dead-reckoning, seeded at GT start)")
    ax.legend(ncol=2); ax.grid(alpha=0.3)
    return _save(fig, outdir, "05_trajectories.png")


def plot_drift_over_time(gtm, trajs, outdir):
    """Position error vs time for each dead-reckoning estimator (initial pose fixed)."""
    from .util import interp, interp_angle, wrap
    fig, ax = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
    colors = {"kin_old": "k", "kin_calib": "orange", "gyro": "g", "ahrs": "r", "ahrs_gyro": "m"}
    for tr in trajs:
        m = (tr.t >= gtm.t[0]) & (tr.t <= gtm.t[-1])
        t = tr.t[m]
        gx = interp(t, gtm.t, gtm.x); gy = interp(t, gtm.t, gtm.y)
        err = np.hypot(tr.x[m] - gx, tr.y[m] - gy)
        gyaw = interp_angle(t, gtm.t, gtm.yaw)
        yerr = np.degrees(np.abs(wrap(tr.yaw[m] - gyaw)))
        c = colors.get(tr.name, "gray")
        ax[0].plot(t, err, color=c, lw=1.3, label=f"{tr.name} (end {err[-1]:.2f} m)")
        ax[1].plot(t, yerr, color=c, lw=1.3, label=tr.name)
    ax[0].set_ylabel("position error [m]")
    ax[0].set_title("Drift growth from the fixed initial pose")
    ax[0].legend(ncol=2, fontsize=8); ax[0].grid(alpha=0.3)
    ax[1].set_ylabel("heading error [deg]"); ax[1].set_xlabel("t [s]")
    ax[1].legend(ncol=2, fontsize=8); ax[1].grid(alpha=0.3)
    return _save(fig, outdir, "07_drift_over_time.png")


def plot_ape_bars(metrics_list, outdir):
    fig, ax = plt.subplots(figsize=(9, 4.2))
    names = [m.name for m in metrics_list]
    rmse = [m.ape_rmse for m in metrics_list]
    final = [m.final_drift for m in metrics_list]
    x = np.arange(len(names))
    ax.bar(x - 0.2, rmse, 0.4, label="APE RMSE [m]", color="steelblue")
    ax.bar(x + 0.2, final, 0.4, label="final drift [m]", color="salmon")
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=20)
    ax.set_ylabel("error [m]"); ax.set_title("Dead-reckoning error vs GT")
    ax.legend(); ax.grid(alpha=0.3, axis="y")
    for i, (r, f) in enumerate(zip(rmse, final)):
        ax.text(i - 0.2, r, f"{r:.2f}", ha="center", va="bottom", fontsize=8)
        ax.text(i + 0.2, f, f"{f:.2f}", ha="center", va="bottom", fontsize=8)
    return _save(fig, outdir, "06_ape_bars.png")
