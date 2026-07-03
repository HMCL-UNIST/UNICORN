"""Small shared helpers: angle handling, interpolation, SE2 alignment."""
from __future__ import annotations

import numpy as np


def wrap(a):
    """Wrap angle(s) to (-pi, pi]."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def unwrap(a):
    return np.unwrap(np.asarray(a, dtype=float))


def interp(t_query, t, x):
    """Linear interpolation of x(t) at t_query, clamped to the data range."""
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    return np.interp(t_query, t, x)


def interp_angle(t_query, t, ang):
    """Interpolate an angle series (unwrap -> interp -> wrap)."""
    return wrap(np.interp(t_query, np.asarray(t), unwrap(ang)))


def se2_align(src_xy, dst_xy, with_scale=False):
    """Least-squares rigid (optionally similarity) SE2 alignment src -> dst.

    Returns (R, t, s) and the aligned source points.  Uses the Umeyama method.
    """
    src = np.asarray(src_xy, dtype=float)
    dst = np.asarray(dst_xy, dtype=float)
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    src_c = src - mu_s
    dst_c = dst - mu_d
    cov = dst_c.T @ src_c / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[1, 1] = -1
    R = U @ S @ Vt
    if with_scale:
        var_s = (src_c ** 2).sum() / len(src)
        s = (D * np.diag(S)).sum() / var_s
    else:
        s = 1.0
    t = mu_d - s * R @ mu_s
    aligned = (s * (R @ src.T)).T + t
    return R, t, s, aligned


def align_yaw_offset(src_yaw, dst_yaw):
    """Constant yaw offset (and sign) that best maps src_yaw onto dst_yaw.

    Returns (offset, sign).  The AHRS heading has an arbitrary absolute
    reference and a possibly flipped Z axis, so we detect both.
    """
    s = unwrap(src_yaw)
    d = unwrap(dst_yaw)
    # sign: do the two headings rotate the same way?
    ds = np.diff(s)
    dd = np.diff(d)
    sign = 1.0 if np.dot(ds, dd) >= 0 else -1.0
    # circular-mean offset
    diff = d - sign * s
    offset = np.arctan2(np.sin(diff).mean(), np.cos(diff).mean())
    return offset, sign
