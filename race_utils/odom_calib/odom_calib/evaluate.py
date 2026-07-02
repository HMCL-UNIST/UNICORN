"""Trajectory accuracy metrics against the GT (cartographer) trajectory.

Two flavours of error:

* ``anchored`` -- no realignment.  Valid for dead-reckoning trajectories that
  were already seeded at the GT start pose; the error *is* the accumulated
  drift, which is what we care about.
* ``umeyama`` -- rigid SE2 alignment first.  Needed for a recorded trajectory
  that lives in its own frame (e.g. ``/vesc/odom`` in the free-running ``odom``
  frame), so we compare shape independent of frame.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .util import interp, interp_angle, se2_align, wrap


@dataclass
class TrajMetrics:
    name: str
    ape_rmse: float
    ape_mean: float
    ape_median: float
    ape_max: float
    final_drift: float
    path_len: float
    drift_pct: float        # final_drift / path_len * 100
    yaw_rmse_deg: float
    rpe_rmse: float         # relative pose error (1 m segments), translation
    n: int


def _resample_gt(t, gtm):
    gx = interp(t, gtm.t, gtm.x)
    gy = interp(t, gtm.t, gtm.y)
    gyaw = interp_angle(t, gtm.t, gtm.yaw)
    return gx, gy, gyaw


def _rpe(ex, ey, gx, gy, seg=1.0):
    """Translation RPE over ~``seg`` metre GT segments."""
    gd = np.r_[0, np.cumsum(np.hypot(np.diff(gx), np.diff(gy)))]
    errs = []
    j = 0
    for i in range(len(gd)):
        while j < len(gd) and gd[j] - gd[i] < seg:
            j += 1
        if j >= len(gd):
            break
        dg = np.hypot(gx[j] - gx[i], gy[j] - gy[i])
        de = np.hypot(ex[j] - ex[i], ey[j] - ey[i])
        errs.append(abs(de - dg))
    return float(np.sqrt(np.mean(np.square(errs)))) if errs else float("nan")


def evaluate(traj, gtm, align="anchor"):
    """Return (:class:`TrajMetrics`, aligned_xy) for ``traj`` vs GT."""
    t = traj.t
    # clip to GT time overlap
    m = (t >= gtm.t[0]) & (t <= gtm.t[-1])
    t = t[m]
    ex, ey, eyaw = traj.x[m], traj.y[m], traj.yaw[m]
    gx, gy, gyaw = _resample_gt(t, gtm)

    if align == "umeyama":
        _, _, _, aligned = se2_align(np.c_[ex, ey], np.c_[gx, gy])
        ex_a, ey_a = aligned[:, 0], aligned[:, 1]
    else:
        ex_a, ey_a = ex, ey

    err = np.hypot(ex_a - gx, ey_a - gy)
    path_len = float(np.sum(np.hypot(np.diff(gx), np.diff(gy))))
    final_drift = float(err[-1])
    yaw_err = np.degrees(np.abs(wrap(eyaw - gyaw)))

    metrics = TrajMetrics(
        name=traj.name,
        ape_rmse=float(np.sqrt(np.mean(err ** 2))),
        ape_mean=float(np.mean(err)),
        ape_median=float(np.median(err)),
        ape_max=float(np.max(err)),
        final_drift=final_drift,
        path_len=path_len,
        drift_pct=float(final_drift / path_len * 100) if path_len > 0 else float("nan"),
        yaw_rmse_deg=float(np.sqrt(np.mean(yaw_err ** 2))),
        rpe_rmse=_rpe(ex_a, ey_a, gx, gy),
        n=int(m.sum()),
    )
    return metrics, (ex_a, ey_a)


def write_tum(path, t, x, y, yaw):
    """Dump a trajectory in TUM format (t tx ty tz qx qy qz qw) for evo."""
    qz = np.sin(yaw / 2.0)
    qw = np.cos(yaw / 2.0)
    zeros = np.zeros_like(t)
    arr = np.c_[t, x, y, zeros, zeros, zeros, qz, qw]
    np.savetxt(path, arr, fmt="%.6f")
