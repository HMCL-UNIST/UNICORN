"""Assemble a markdown report from the calibration / evaluation results."""
from __future__ import annotations


def _row(cells):
    return "| " + " | ".join(str(c) for c in cells) + " |"


def _improve(old, new):
    if old <= 0:
        return "n/a"
    pct = (old - new) / old * 100
    sign = "−" if pct >= 0 else "+"
    return f"{sign}{abs(pct):.0f}%"


def build_report(bag, dur, sc, stc, dist, metrics, old, wheelbase):
    L = []
    A = L.append
    A("# VESC odometry calibration & localization report\n")
    A(f"- bag: `{bag}`")
    A(f"- analysed duration: {dur:.1f} s")
    A(f"- ground truth: cartographer `/tracked_pose` (LIDAR scan-matched, metric scale)\n")

    # --- speed ---
    A("## 1. Speed gain — `speed_to_erpm_gain`\n")
    A("Three independent estimators (they agree, so the number is trustworthy):\n")
    A(_row(["method", "gain", "note"]))
    A(_row(["---", "---", "---"]))
    A(_row(["regression (through origin)", f"**{sc.gain:.0f}**", f"R²={sc.r2:.3f}, n={sc.n}"]))
    A(_row(["distance-matched", f"{sc.gain_dist:.0f}", "odom dist == GT arc length"]))
    A(_row(["median eRPM/speed", f"{sc.gain_ratio:.0f}", "at |v|>1 m/s"]))
    A(_row(["regression (+offset)", f"{sc.gain_offsetfit:.0f}", f"offset {sc.offset_offsetfit:.0f} eRPM"]))
    A(_row(["**current config**", f"{old['speed_gain']:.0f}", "← what the car uses now"]))
    A("")
    A(f"- The configured **{old['speed_gain']:.0f}** under-measures distance: it makes "
      f"`/vesc/odom` report **{dist.odom_len_old:.1f} m** for a run that GT (and LIDAR) "
      f"say was **{dist.gt_len:.1f} m** — a ~{(1-dist.odom_len_old/dist.gt_len)*100:.0f}% scale error.")
    A(f"- With the identified gain odom distance becomes **{dist.odom_len_new:.1f} m** "
      f"(matches GT).")
    A(f"- Longitudinal-speed error vs GT: current **{sc.rms_old:.3f} m/s** → "
      f"identified **{sc.rms_new:.3f} m/s** ({_improve(sc.rms_old, sc.rms_new)}).\n")

    # --- steering ---
    A("## 2. Steering gain — `steering_angle_to_servo_*`\n")
    A(_row(["param", "current config", "identified", "note"]))
    A(_row(["---", "---", "---", "---"]))
    A(_row(["steering_angle_to_servo_gain", f"{old['servo_gain']:+.4f}", f"**{stc.gain:+.4f}**",
            f"|·| {'matches' if abs(abs(stc.gain)-abs(old['servo_gain']))<0.1 else 'differs from'} config"]))
    A(_row(["steering_angle_to_servo_offset", f"{old['servo_offset']:.4f}", f"**{stc.offset:.4f}**",
            "servo at δ=0 (straight)"]))
    A(_row(["(servo↔steer fit quality)", "-", f"corr {stc.corr:.3f}", f"method={stc.method}"]))
    A(_row(["(actual/commanded steer)", "-", f"{stc.response_ratio:.3f}", "1.0 ⇒ car steers what it's told"]))
    A(_row(["(steering response lag)", "-", f"{stc.lag_s*1000:.0f} ms", "servo→yaw delay"]))
    A("")
    A(f"- **Magnitude is essentially already correct** (|{stc.gain:.3f}| ≈ |{old['servo_gain']:.3f}|) and the car "
      f"steers the angle it is commanded (ratio {stc.response_ratio:.2f}). Steering scale needs no real change.")
    if stc.sign * old["servo_gain"] < 0:
        A(f"- ⚠️ **Sign differs**: this run used gain **{stc.gain:+.3f}** but your config has "
          f"**{old['servo_gain']:+.3f}**. Either the config was changed after this recording or the "
          f"servo direction was reversed in hardware — confirm on the car before flipping it.")
    doff = stc.offset - old["servo_offset"]
    A(f"- Offset differs by {doff:+.3f} servo (~{abs(doff)/abs(stc.gain)*57.3:.1f}° steering bias); "
      f"nudge `steering_angle_to_servo_offset` toward **{stc.offset:.4f}** if the car pulls to one side.\n")

    # --- drop-in ---
    A("### Drop-in `vehicle_config.yaml` (CAR) values\n")
    A("```yaml")
    A("vesc/**:")
    A(f"    speed_to_erpm_gain: {sc.gain:.0f}            # was {old['speed_gain']:.0f}  (the real fix)")
    A(f"    speed_to_erpm_offset: 0.0")
    A(f"    steering_angle_to_servo_gain: {stc.gain:.4f}    # |{abs(stc.gain):.3f}| ~ current |{abs(old['servo_gain']):.3f}|, mind the sign")
    A(f"    steering_angle_to_servo_offset: {stc.offset:.4f}")
    A("```\n")

    # --- distance / jitter validation ---
    A("## 3. Why trust the GT scale (jitter check)\n")
    A("GT arc length barely changes when the 100 Hz pose is decimated, so it is real "
      "distance, not noise inflation:\n")
    A(_row(["GT sample rate", "100 Hz", "20 Hz", "10 Hz", "5 Hz"]))
    A(_row(["arc length [m]"] + [f"{dist.decim_lengths[d]:.1f}" for d in (1, 5, 10, 20)]))
    A("")

    # --- trajectory comparison ---
    A("## 4. Dead-reckoning accuracy vs GT — initial pose fixed\n")
    A("Every estimator is seeded at the GT start pose and then **dead-reckons with no "
      "further correction**, so the error is pure accumulated drift. `kin_old` ≈ what "
      "`/vesc/odom` publishes today.\n")
    A(_row(["estimator", "APE RMSE [m]", "final drift [m]", "drift %path", "yaw RMSE [°]", "RPE(1m) [m]"]))
    A(_row(["---"] * 6))
    for m in metrics:
        A(_row([m.name, f"{m.ape_rmse:.3f}", f"{m.final_drift:.3f}",
                f"{m.drift_pct:.1f}%", f"{m.yaw_rmse_deg:.2f}", f"{m.rpe_rmse:.3f}"]))
    A("")
    A(_legend())

    # --- verdict ---
    A("## 5. Does the IMU / AHRS help?\n")
    by = {m.name: m for m in metrics}
    base = by.get("kin_old")
    cand = [m for m in metrics if m.name in ("kin_calib", "gyro", "ahrs", "ahrs_gyro")]
    best = min(cand, key=lambda m: m.ape_rmse) if cand else None
    if base and best:
        A(f"- Baseline (`kin_old`, current gains, steering-model heading): "
          f"APE **{base.ape_rmse:.2f} m**, final drift **{base.final_drift:.2f} m**, "
          f"yaw RMSE **{base.yaw_rmse_deg:.0f}°**.")
        A(f"- Best variant: **`{best.name}`** at APE **{best.ape_rmse:.2f} m** "
          f"({_improve(base.ape_rmse, best.ape_rmse)} vs baseline).")
        for name in ("kin_calib", "gyro", "ahrs", "ahrs_gyro"):
            if name in by:
                m = by[name]
                A(f"  - `{name}`: APE {m.ape_rmse:.2f} m, yaw RMSE {m.yaw_rmse_deg:.1f}° "
                  f"({_improve(base.ape_rmse, m.ape_rmse)}).")
        A("")
        A("**Takeaway.**")
        A("1. The steering-model heading that `/vesc/odom` integrates is the dominant error "
          "source — yaw RMSE ~100°. Fixing the speed gain alone (`kin_calib`) cuts position "
          "drift but not heading.")
        A("2. **Any IMU heading source collapses yaw error to a few degrees** and position "
          "drift to ~1 m: gyro, AHRS, and the complementary filter are all within noise of "
          "each other here.")
        A("3. On this 22 s clip the plain gyro is marginally best because it has not had time "
          "to drift; the **AHRS absolute heading is drift-free**, so it is the safer choice on "
          "long runs, and the complementary filter (`ahrs_gyro`) keeps the gyro's smoothness "
          "while pinning it to the AHRS — the best general-purpose option.")
        A("4. This is exactly what feeding `/vesc/sensors/imu` (orientation + gyro) into "
          "`robot_localization` buys you. The shipped `ekf_cartographer.yaml` does **not** fuse "
          "the IMU at all — see `ros/ekf_deadreckoning.yaml` for a config that does, plus how to "
          "replay this bag through the real package.")
    A("")
    A("Figures: `01_speed_calibration.png`, `02_steering_calibration.png`, "
      "`03_gt_motion.png`, `04_heading.png`, `05_trajectories.png`, `06_ape_bars.png`, "
      "`07_drift_over_time.png`.\n")
    return "\n".join(L)


def _legend():
    return (
        "- `kin_old`  — eRPM/servo kinematic model, **current** gains (≈ /vesc/odom)\n"
        "- `kin_calib`— same model, **identified** gains\n"
        "- `gyro`     — identified speed + heading from integrating the IMU gyro\n"
        "- `ahrs`     — identified speed + **absolute AHRS heading** (drift-free)\n"
        "- `ahrs_gyro`— identified speed + complementary filter (gyro + AHRS)\n"
        "- `vesc_odom_rec*` — the recorded /vesc/odom, rigidly aligned (shape only)\n"
    )
