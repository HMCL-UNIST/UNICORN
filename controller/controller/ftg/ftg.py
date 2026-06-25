#!/usr/bin/env python3
"""Follow-The-Gap (disparity-extender). Beam/FOV independent: all thresholds in
metres/radians, so one tuning works on the 2160-beam sim and the ~1080-beam car.
Per scan: sanitize NaN->max, window the front +-FRONT_FOV, smooth, mask returns
within track_width/2, bubble each >=DISP_THRESH disparity, take the largest free
gap, aim at its centre, EMA-smooth the steer."""
import math
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray


class FTG:
    # speed-scheduling thresholds on |steer| [rad]
    STRAIGHTS_STEERING_ANGLE = np.pi / 18   # 10 deg
    MILD_CURVE_ANGLE = np.pi / 6            # 30 deg
    ULTRASTRAIGHTS_ANGLE = np.pi / 60       # 3 deg

    # disparity-extender tunables: DEFAULTS, overridden from controller.yaml (ftg_*)
    # and live-tunable via rqt.
    FRONT_FOV = math.radians(45.0)    # HALF-angle [rad] = ftg_front_fov_deg/2 (total 90 deg)
    SMOOTH_RAD = math.radians(1.0)    # scan smoothing window [rad]     (ftg_smooth_deg)
    DISP_THRESH = 0.5                 # range jump = discontinuity [m]  (ftg_disp_thresh)
    BUBBLE_M = 0.30                   # safety bubble per disparity [m] (ftg_bubble_m)
    STEER_EMA = 0.0                   # steer low-pass a: s=a*prev+(1-a)*new (ftg_steer_ema)
    MAX_STEER = 0.4                   # steering clip [rad]             (ftg_max_steer)
    SPEED_SCALE = 1.0                 # overall speed multiplier

    def __init__(self, node=None, mapping=False, debug=False,
                 safety_radius=None, max_lidar_dist=None, max_speed=1.5,
                 range_offset=None, track_width=2.0,
                 front_fov_deg=None, smooth_deg=None, disp_thresh=None,
                 bubble_m=None, steer_ema=None, max_steer=None) -> None:
        self.node = node
        self.mapping = mapping

        self.DEBUG = debug
        self.SAFETY_RADIUS = safety_radius          # accepted for compat (unused)
        self.range_offset = range_offset            # accepted for compat (unused)
        self.MAX_LIDAR_DIST = max_lidar_dist if max_lidar_dist else 10.0
        self.MAX_SPEED = max_speed
        self.track_width = track_width

        # override class-default tunables from yaml when provided
        if front_fov_deg is not None:
            self.FRONT_FOV = math.radians(float(front_fov_deg) / 2.0)  # param = TOTAL front FOV
        if smooth_deg is not None:
            self.SMOOTH_RAD = math.radians(float(smooth_deg))
        if disp_thresh is not None:
            self.DISP_THRESH = float(disp_thresh)
        if bubble_m is not None:
            self.BUBBLE_M = float(bubble_m)
        if steer_ema is not None:
            self.STEER_EMA = float(steer_ema)
        if max_steer is not None:
            self.MAX_STEER = float(max_steer)

        self.recompute_speeds()

        self.velocity = 0.0
        self._steer_prev = None
        self.angle_min = -0.75 * np.pi
        self.angle_inc = None
        self.radians_per_elem = None

        self.best_pnt = self.scan_pub = self.best_gap = None
        if self.node is not None:
            self.best_pnt = self.node.create_publisher(Marker, '/best_points/marker', 10)
            self.scan_pub = self.node.create_publisher(MarkerArray, '/scan_proc/markers', 10)
            self.best_gap = self.node.create_publisher(MarkerArray, '/best_gap/markers', 10)

    def recompute_speeds(self) -> None:
        s = self.SPEED_SCALE
        self.CORNERS_SPEED = 0.3 * self.MAX_SPEED * s
        self.MILD_CORNERS_SPEED = 0.45 * self.MAX_SPEED * s
        self.STRAIGHTS_SPEED = 0.8 * self.MAX_SPEED * s
        self.ULTRASTRAIGHTS_SPEED = self.MAX_SPEED * s

    def set_vel(self, velocity) -> None:
        self.velocity = velocity

    def _now(self):
        return self.node.get_clock().now().to_msg() if self.node is not None else None

    def _bubble_beams(self, near_range) -> int:
        near = max(float(near_range), 0.05)
        return int(math.ceil(math.atan2(self.BUBBLE_M, near) / self.angle_inc))

    def process_lidar(self, ranges, angle_min=None, angle_increment=None) -> tuple:
        """Returns (speed, steering_angle). angle_min/angle_increment from the
        LaserScan make it beam/FOV independent; if omitted a 270-deg FOV is assumed."""
        n = len(ranges)
        if angle_increment is not None and angle_increment > 0.0:
            self.angle_inc = float(angle_increment)
            self.angle_min = float(angle_min) if angle_min is not None else -(n - 1) * self.angle_inc / 2.0
        else:
            self.angle_inc = (1.5 * np.pi) / n
            self.angle_min = -(n - 1) * self.angle_inc / 2.0
        self.radians_per_elem = self.angle_inc

        # NaN/inf == no return == open -> max range, then clip
        r = np.asarray(ranges, dtype=float)
        r = np.where(np.isfinite(r), r, self.MAX_LIDAR_DIST)
        r = np.clip(r, 0.0, self.MAX_LIDAR_DIST)

        # front FOV window (angle-based)
        i_lo = max(0, int(math.ceil((-self.FRONT_FOV - self.angle_min) / self.angle_inc)))
        i_hi = min(n, int(math.floor((self.FRONT_FOV - self.angle_min) / self.angle_inc)) + 1)
        if i_hi - i_lo < 3:
            return self._speed_for(0.0), 0.0
        proc = r[i_lo:i_hi].copy()
        base_angle = self.angle_min + i_lo * self.angle_inc

        # smoothing window sized in radians
        w = max(1, int(round(self.SMOOTH_RAD / self.angle_inc)))
        if w > 1:
            proc = np.convolve(proc, np.ones(w) / w, 'same')

        # mask returns within half the track width (too close = wall/self)
        free = proc >= (self.track_width / 2.0)

        # disparity extender: bubble the near side of each >DISP_THRESH jump
        d = np.diff(proc)
        L = len(proc)
        for i in np.where(np.abs(d) > self.DISP_THRESH)[0]:
            if d[i] > 0:
                b = self._bubble_beams(proc[i])
                free[i + 1: min(L, i + 1 + b)] = False
            else:
                b = self._bubble_beams(proc[i + 1])
                free[max(0, i + 1 - b): i + 1] = False

        # largest contiguous free run; fall back to farthest point
        gl, gr = self._largest_run(free)
        mid = (gl + gr) // 2 if gr > gl else int(np.argmax(proc))

        # steer toward the gap centre (0 = forward, + = left), then EMA-smooth
        raw_steer = float(np.clip(base_angle + mid * self.angle_inc,
                                  -self.MAX_STEER, self.MAX_STEER))
        if self._steer_prev is None:
            steer = raw_steer
        else:
            a = self.STEER_EMA
            steer = a * self._steer_prev + (1.0 - a) * raw_steer
        self._steer_prev = steer

        if self.DEBUG:
            self._publish_debug(proc, base_angle, gl, gr, mid)
        return self._speed_for(steer), steer

    @staticmethod
    def _largest_run(mask) -> tuple:
        if not mask.any():
            return 0, 0
        edges = np.where(np.diff(np.concatenate(([0], mask.view(np.int8), [0]))))[0]
        starts, ends = edges[0::2], edges[1::2]
        k = int(np.argmax(ends - starts))
        return int(starts[k]), int(ends[k]) - 1

    def _speed_for(self, steering_angle) -> float:
        if self.mapping:
            return 1.5
        a = abs(steering_angle)
        if a > self.MILD_CURVE_ANGLE:
            return self.CORNERS_SPEED
        if a > self.STRAIGHTS_STEERING_ANGLE:
            return self.MILD_CORNERS_SPEED
        if a > self.ULTRASTRAIGHTS_ANGLE:
            return self.STRAIGHTS_SPEED
        return self.ULTRASTRAIGHTS_SPEED

    def _publish_debug(self, proc, base_angle, gl, gr, mid) -> None:
        clr = MarkerArray()
        m = Marker(); m.header.frame_id = 'laser'; m.header.stamp = self._now()
        m.action = Marker.DELETEALL
        clr.markers.append(m)
        self.best_gap.publish(clr)

        gap_markers = MarkerArray()
        for i in range(gl, max(gl + 1, gr)):
            ang = base_angle + i * self.angle_inc
            rng = float(proc[i]) if i < len(proc) else 1.0
            mrk = Marker()
            mrk.header.frame_id = 'laser'; mrk.header.stamp = self._now()
            mrk.type = mrk.SPHERE
            mrk.scale.x = mrk.scale.y = mrk.scale.z = 0.05
            mrk.color.a = 1.0; mrk.color.r = 1.0; mrk.color.g = 1.0
            mrk.id = i - gl
            mrk.pose.position.x = math.cos(ang) * rng
            mrk.pose.position.y = math.sin(ang) * rng
            mrk.pose.orientation.w = 1.0
            gap_markers.markers.append(mrk)
        self.best_gap.publish(gap_markers)

        ang = base_angle + mid * self.angle_inc
        rng = float(proc[mid]) if mid < len(proc) else 1.0
        bm = Marker()
        bm.header.frame_id = 'laser'; bm.header.stamp = self._now()
        bm.type = bm.SPHERE
        bm.scale.x = bm.scale.y = bm.scale.z = 0.2
        bm.color.a = 1.0; bm.color.b = 1.0; bm.color.g = 1.0
        bm.id = 0
        bm.pose.position.x = math.cos(ang) * rng
        bm.pose.position.y = math.sin(ang) * rng
        bm.pose.orientation.w = 1.0
        self.best_pnt.publish(bm)

        sm = MarkerArray()
        step = max(1, len(proc) // 360)
        for i in range(0, len(proc), step):
            ang = base_angle + i * self.angle_inc
            mrk = Marker()
            mrk.header.frame_id = 'laser'; mrk.header.stamp = self._now()
            mrk.type = mrk.SPHERE
            mrk.scale.x = mrk.scale.y = mrk.scale.z = 0.05
            mrk.color.a = 1.0; mrk.color.r = 1.0; mrk.color.b = 1.0
            mrk.id = i
            mrk.pose.position.x = math.cos(ang) * float(proc[i])
            mrk.pose.position.y = math.sin(ang) * float(proc[i])
            mrk.pose.orientation.w = 1.0
            sm.markers.append(mrk)
        self.scan_pub.publish(sm)
