#!/usr/bin/env python3
"""Run the full VESC odometry calibration + localization evaluation on a rosbag.

    python run.py /path/to/rosbag2_xxx.mcap --out results [--t-start 1.0] [--t-end 20]

Outputs (in --out): figures *.png, report.md, and TUM trajectories for evo.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from odom_calib import bag_io, gt, calibrate, fusion, evaluate, plotting, report  # noqa: E402

# defaults from stack_master/config/CAR/vehicle_config.yaml
OLD = dict(speed_gain=4614.0, speed_offset=0.0,
           servo_gain=-1.2135, servo_offset=0.5304)
WHEELBASE = 0.33


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="path to the .mcap file (or its directory)")
    ap.add_argument("--out", default="results", help="output directory")
    ap.add_argument("--t-start", type=float, default=None, help="crop start [s]")
    ap.add_argument("--t-end", type=float, default=None, help="crop end [s]")
    ap.add_argument("--wheelbase", type=float, default=WHEELBASE)
    ap.add_argument("--no-lag", action="store_true", help="skip steering lag search")
    # reference config the RECORDING car used (defaults = repo CAR/vehicle_config.yaml).
    # Pass the bag vehicle's actual values for a clean "is it already right?" comparison.
    ap.add_argument("--speed-gain", type=float, default=OLD["speed_gain"])
    ap.add_argument("--servo-gain", type=float, default=OLD["servo_gain"])
    ap.add_argument("--servo-offset", type=float, default=OLD["servo_offset"])
    args = ap.parse_args()

    OLD["speed_gain"] = args.speed_gain
    OLD["servo_gain"] = args.servo_gain
    OLD["servo_offset"] = args.servo_offset

    bag = args.bag
    if os.path.isdir(bag):
        mcaps = [f for f in os.listdir(bag) if f.endswith(".mcap")]
        if not mcaps:
            ap.error(f"no .mcap in {bag}")
        bag = os.path.join(bag, sorted(mcaps)[0])

    os.makedirs(args.out, exist_ok=True)

    print(f"[1/6] reading {bag} ...")
    data = bag_io.read_bag(bag, args.t_start, args.t_end)
    print(f"      GT poses: {len(data.gt)}, core: {len(data.core)}, imu: {len(data.imu)}, "
          f"vesc/odom: {len(data.vo)}")

    print("[2/6] computing GT motion ...")
    gtm = gt.compute_gt_motion(data.gt)

    print("[3/6] calibrating speed gain ...")
    sc = calibrate.calibrate_speed(data, gtm, old_gain=OLD["speed_gain"],
                                   old_offset=OLD["speed_offset"])
    dist = calibrate.distance_check(data, gtm, OLD["speed_gain"], sc.gain)
    print(f"      speed_to_erpm_gain: {OLD['speed_gain']:.0f} -> {sc.gain:.0f}  "
          f"(dist-match {sc.gain_dist:.0f}, ratio {sc.gain_ratio:.0f}, R²={sc.r2:.3f})")
    print(f"      GT dist {dist.gt_len:.1f} m vs odom@old {dist.odom_len_old:.1f} m "
          f"-> odom@new {dist.odom_len_new:.1f} m")

    print("[4/6] calibrating steering ...")
    stc = calibrate.calibrate_steering(data, gtm, wheelbase=args.wheelbase,
                                       old_gain=OLD["servo_gain"],
                                       old_offset=OLD["servo_offset"],
                                       find_lag=not args.no_lag)
    print(f"      servo_gain: {OLD['servo_gain']:+.4f} -> {stc.gain:+.4f} ({stc.method}, "
          f"corr {stc.corr:.3f}), offset {OLD['servo_offset']:.4f} -> {stc.offset:.4f}, "
          f"resp_ratio {stc.response_ratio:.2f}, lag {stc.lag_s*1000:.0f} ms")

    print("[5/6] building dead-reckoning estimators + evaluating ...")
    trajs = fusion.build_estimators(data, gtm, sc, stc, wheelbase=args.wheelbase,
                                    old_gain=OLD["speed_gain"],
                                    old_servo_gain=OLD["servo_gain"],
                                    old_servo_off=OLD["servo_offset"])
    metrics = []
    for tr in trajs:
        m, _ = evaluate.evaluate(tr, gtm, align="anchor")
        metrics.append(m)
        evaluate.write_tum(os.path.join(args.out, f"traj_{tr.name}.tum"),
                           tr.t, tr.x, tr.y, tr.yaw)
    # recorded vesc/odom (own frame -> umeyama) as a cross-check
    if len(data.vo):
        vo_traj = fusion.Traj("vesc_odom_rec", data.vo.t, data.vo.x, data.vo.y, data.vo.yaw)
        m_vo, _ = evaluate.evaluate(vo_traj, gtm, align="umeyama")
        m_vo.name = "vesc_odom_rec*"
        metrics.append(m_vo)
    # GT TUM for evo
    evaluate.write_tum(os.path.join(args.out, "traj_gt.tum"),
                       gtm.t, gtm.x, gtm.y, gtm.yaw)

    print("[6/6] plotting + report ...")
    plotting.plot_speed_cal(sc, args.out, old_gain=OLD["speed_gain"])
    plotting.plot_steer_cal(stc, args.out, old_gain=OLD["servo_gain"], old_off=OLD["servo_offset"])
    plotting.plot_gt_motion(data, gtm, sc, args.out, old_gain=OLD["speed_gain"])
    plotting.plot_heading(data, gtm, trajs, args.out)
    plotting.plot_trajectories(gtm, data.vo, trajs, args.out)
    plotting.plot_drift_over_time(gtm, trajs, args.out)
    plotting.plot_ape_bars(metrics, args.out)

    rep = report.build_report(bag, data.duration, sc, stc, dist, metrics, OLD, args.wheelbase)
    with open(os.path.join(args.out, "report.md"), "w") as f:
        f.write(rep)

    print("\n" + "=" * 70)
    print(rep)
    print("=" * 70)
    print(f"\nwrote results to {os.path.abspath(args.out)}/")


if __name__ == "__main__":
    main()
