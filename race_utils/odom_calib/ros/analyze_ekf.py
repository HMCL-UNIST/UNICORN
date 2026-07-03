"""Compare a recorded EKF replay (/odometry/filtered) + raw /vesc/odom against GT
(/tracked_pose), the HONEST dead-reckoning way: anchor BOTH position and heading
at the start, then watch the drift grow.

Why start-anchored, not Umeyama: a global Umeyama fit rotates/translates the whole
trajectory to minimise total error, which can *hide* drift by splitting it front-to-
back. Anchoring the pose (x, y, AND yaw) only at t0 -- exactly how you initialise an
estimator on the real car -- makes the error at every later time the true accumulated
drift. We still print the Umeyama APE for reference.

Usage: python3 analyze_ekf.py <recorded.mcap> <out_prefix> [label]
"""
import os, sys, math, numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # tool root (ros/ -> ..)
from odom_calib.util import se2_align, interp, interp_angle, wrap
from odom_calib.bag_io import yaw_from_quat
from mcap_ros2.reader import read_ros2_messages
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt

REC = sys.argv[1]
OUTP = sys.argv[2]                       # output prefix (without extension)
LABEL = sys.argv[3] if len(sys.argv) > 3 else ""
SETTLE = 0.3                             # s, let the filter initialise before anchoring


def read(topic):
    t = []; x = []; y = []; yaw = []
    for m in read_ros2_messages(REC, topics=[topic]):
        msg = m.ros_msg
        st = msg.header.stamp
        p = msg.pose if topic == "/tracked_pose" else msg.pose.pose
        q = p.orientation
        t.append(st.sec + st.nanosec * 1e-9)
        x.append(p.position.x); y.append(p.position.y)
        yaw.append(yaw_from_quat(q.x, q.y, q.z, q.w))
    return [np.array(a) for a in (t, x, y, yaw)]


gt = read("/tracked_pose"); ekf = read("/odometry/filtered"); vo = read("/vesc/odom")
if len(ekf[0]) == 0 or len(gt[0]) == 0:
    print("! /odometry/filtered or /tracked_pose empty in recording — EKF stage produced no usable data.")
    sys.exit(2)
t0 = gt[0][0]
for s in (gt, ekf, vo):
    s[0] = s[0] - t0


def gt_at(t):
    return interp(t, gt[0], gt[1]), interp(t, gt[0], gt[2]), interp_angle(t, gt[0], gt[3])


def anchor(est):
    """Rigidly map est so its pose (x,y,yaw) coincides with GT at the anchor time."""
    t, x, y, yaw = est
    ta = max(gt[0][0], t[0]) + SETTLE
    gx0, gy0, gyaw0 = gt_at(ta)
    ex0 = interp(ta, t, x); ey0 = interp(ta, t, y); eyaw0 = interp_angle(ta, t, yaw)
    dyaw = wrap(gyaw0 - eyaw0)
    c, s = math.cos(dyaw), math.sin(dyaw)
    ax = c * (x - ex0) - s * (y - ey0) + gx0
    ay = s * (x - ex0) + c * (y - ey0) + gy0
    ayaw = wrap(yaw + dyaw)
    m = (t >= ta) & (t <= gt[0][-1])
    gx, gy, gyaw = gt_at(t[m])
    perr = np.hypot(ax[m] - gx, ay[m] - gy)
    herr = np.degrees(np.abs(wrap(ayaw[m] - gyaw)))
    return dict(t=t[m], ax=ax[m], ay=ay[m], perr=perr, herr=herr, ax_full=ax, ay_full=ay)


def umeyama_ape(est):
    t, x, y, yaw = est
    m = (t >= gt[0][0]) & (t <= gt[0][-1])
    gx, gy, _ = gt_at(t[m])
    _, _, _, al = se2_align(np.c_[x[m], y[m]], np.c_[gx, gy])
    return float(np.sqrt(np.mean((al[:, 0] - gx) ** 2 + (al[:, 1] - gy) ** 2)))


A_ekf, A_vo = anchor(ekf), anchor(vo)
print(f"--- {LABEL} ---")
for name, A, est in [("EKF (odom+IMU AHRS)", A_ekf, ekf), ("raw /vesc/odom", A_vo, vo)]:
    print(f"{name:22s}  [start-anchored] drift_rmse={np.sqrt(np.mean(A['perr']**2)):.3f}  "
          f"final={A['perr'][-1]:.3f}  max={A['perr'].max():.3f}  "
          f"yaw_rmse={np.sqrt(np.mean(A['herr']**2)):.2f}deg   "
          f"[umeyama] APE={umeyama_ape(est):.3f}")

# ---- figure: anchored trajectory + drift growth (pos & heading) ----
fig, ax = plt.subplots(1, 3, figsize=(16, 5.2))
ax[0].plot(gt[1], gt[2], "b-", lw=2.5, label="GT (cartographer)", zorder=10)
ax[0].plot(A_ekf["ax_full"], A_ekf["ay_full"], "r-", lw=1.4, label="EKF (odom+IMU), anchored")
ax[0].plot(A_vo["ax_full"], A_vo["ay_full"], "k--", lw=1.0, alpha=0.6, label="raw vesc/odom, anchored")
gx0, gy0, _ = gt_at(max(gt[0][0], ekf[0][0]) + SETTLE)
ax[0].plot(gx0, gy0, "ko", ms=8, label="anchor (pose+heading)")
ax[0].set_aspect("equal"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
ax[0].set_title(f"{LABEL}  start-anchored trajectory"); ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")

ax[1].plot(A_ekf["t"], A_ekf["perr"], "r-", lw=1.4, label=f"EKF (end {A_ekf['perr'][-1]:.2f} m)")
ax[1].plot(A_vo["t"], A_vo["perr"], "k--", lw=1.2, label=f"raw odom (end {A_vo['perr'][-1]:.2f} m)")
ax[1].set_title("position drift from anchor"); ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("position error [m]")
ax[1].legend(); ax[1].grid(alpha=0.3)

ax[2].plot(A_ekf["t"], A_ekf["herr"], "r-", lw=1.4, label="EKF")
ax[2].plot(A_vo["t"], A_vo["herr"], "k--", lw=1.2, label="raw odom")
ax[2].set_title("heading drift from anchor"); ax[2].set_xlabel("t [s]"); ax[2].set_ylabel("heading error [deg]")
ax[2].legend(); ax[2].grid(alpha=0.3)

fig.tight_layout()
fig.savefig(OUTP + ".png", dpi=120, bbox_inches="tight")
print("saved", OUTP + ".png")
