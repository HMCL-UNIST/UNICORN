# odom_calib — VESC odometry gain ID & localization evaluation from a rosbag

Offline tool that takes a rosbag2 (`.mcap`) where cartographer was running and:

1. treats cartographer `/tracked_pose` as **ground truth (GT)**;
2. **auto-identifies the VESC odometry gains** (`speed_to_erpm_gain`,
   `steering_angle_to_servo_gain/offset`) from `eRPM` (`/vesc/sensors/core`) and
   the servo command, by regression against GT;
3. measures how far **`/vesc/odom`** drifts from GT (APE / RPE), with current vs
   identified gains;
4. runs **dead-reckoning fusion experiments** — steering-model heading vs IMU
   **gyro** vs IMU **AHRS absolute heading** vs a complementary filter — to answer
   *"does going through the IMU / `robot_localization` help, especially the AHRS
   orientation?"*

No ROS install is needed for the analysis: `mcap-ros2-support` decodes every
message (including the custom `vesc_msgs`) straight from the schema embedded in
the bag.

## One command, one bag → everything

```bash
pip install -r requirements.txt          # once (no ROS needed for the offline part)
./analyze_bag.sh /path/to/rosbag2_dir_or.mcap \
    --speed-gain 4614 --servo-gain 1.21 --servo-offset 0.48   # the recording car's current config
```

`analyze_bag.sh` runs the whole thing end to end:
1. **offline** VESC gain ID + dead-reckoning evaluation → figures `01..07` + `report.md`
2. **robot_localization replay**: replays the bag through the *real* `ekf_node`
   (sourcing the unicorn ROS env automatically) → `ekf_replay/ekf_vs_gt.png` + metrics
3. bundles it all into one self-contained **`index.html`** dashboard

Output lands in `results/<bagname>/`. Options:

| flag | meaning |
|---|---|
| `--out DIR` | output dir (default `results/<bagname>`) |
| `--speed-gain / --servo-gain / --servo-offset` | the config the recording car used (for the "is it already right?" comparison) |
| `--t-start S` / `--t-end S` | crop the bag |
| `--imu-yaw RAD` | `base_link→vesc_imu` yaw for the EKF TF (default `+pi/2`) |
| `--no-ekf` | skip the robot_localization replay (offline only; no ROS needed) |

The replay stage needs the ROS env; set `UNICORN_SETUP=/path/to/unicorn.sh` if it
isn't at the default. Without ROS, pass `--no-ekf` — the offline `ahrs_gyro`
estimator already emulates the EKF.

### Just the offline part
```bash
python run.py /path/to/bag --out results        # figures 01..07 + report.md + traj_*.tum
```

## What the model is

`vesc_to_odom.cpp` integrates:

```
v     = (eRPM  - speed_to_erpm_offset)           / speed_to_erpm_gain
delta = (servo - steering_angle_to_servo_offset) / steering_angle_to_servo_gain
omega = v * tan(delta) / wheelbase
x += v cos(yaw) dt ;  y += v sin(yaw) dt ;  yaw += omega dt
```

The tool inverts each relationship against GT motion (GT velocity / yaw-rate from
a Savitzky-Golay differentiation of `/tracked_pose`).

## Findings on `rosbag2_2026_06_30-22_27_29` (see `results/report.md`)

| topic | result |
|---|---|
| **speed_to_erpm_gain** | configured **4614** under-measures distance by ~31% (GT 36.3 m vs odom 24.9 m). Three independent estimators agree on **≈ 3160–3190**. GT scale is jitter-robust (LIDAR-metric), so this is a real miscalibration. |
| **steering gain** | magnitude **≈ 1.21** already matches the config (`1.2135`); only the **sign** (recording used `+`, config has `−`) and a small **offset** (0.48 vs 0.53, ~2.8° bias) differ — verify the steering convention on the car. |
| **IMU / AHRS** | steering-model heading drifts to **~100° yaw RMSE**; feeding **gyro or AHRS** heading drops it to **~3°** and position drift from ~8.7 m → ~1.6 m. AHRS is drift-free (best for long runs); gyro is smooth short-term; the complementary filter combines both. |

### Caveats
- GT is cartographer, which itself ingests `/vesc/odom` + IMU as a motion prior,
  but its scale is set by LIDAR scan-matching (metric) — the 36 m vs 25 m gap
  proves LIDAR overrode the wheel scale, so GT scale is trustworthy.
- The dead-reckoning estimators are all seeded at the GT start pose (initial pose
  fixed) and never corrected, so their error is pure accumulated drift.
- Gains are identified from one ~22 s run; re-run on a longer, more varied bag for
  a production value.

## Validate through the real `robot_localization` (done — it agrees)
The shipped `stack_master/config/ekf_cartographer.yaml` fuses `/vesc/odom` twist +
cartographer pose, and **does not use the IMU**. `ros/ekf_deadreckoning.yaml` fuses
`/vesc/odom` (vx) + `/vesc/sensors/imu` (AHRS yaw + gyro) with **no** map pose,
so its `/odometry/filtered` is an honest dead-reckoning estimate.

Replayed both bags through the actual `ekf_node` (`ros/replay_verify.sh` +
`ros/analyze_ekf.py`, results in `results/ekf_replay/`):

| bag | raw `/vesc/odom` APE | **EKF (odom+IMU)** APE | yaw: raw → EKF |
|---|---|---|---|
| 22_27_29 | 2.98 m | **0.67 m** | 102° → **6.6°** |
| 23_28_49 | 2.61 m | **0.47 m** | 111° → **4.1°** |

The raw-odom numbers match the offline `vesc_odom_rec` exactly, so the offline
emulation and the real package agree: the IMU cuts position error ~4–5× and
heading ~20×. Reproduce:

```bash
bash ros/replay_verify.sh /path/to/bag /tmp/ekf_out /tmp/ekf_logs
python3 ros/analyze_ekf.py /tmp/ekf_out/ekf_out_0.mcap out.png
```

## Layout
```
analyze_bag.sh        ONE-COMMAND entry: offline + EKF replay + html
run.py                offline-only CLI
odom_calib/
  bag_io.py     mcap -> time series (GT, vesc/odom, core eRPM, servo, AHRS, ackermann)
  gt.py         GT velocity / yaw-rate (Savitzky-Golay)
  calibrate.py  speed + steering gain ID (robust IRLS) + distance cross-check
  fusion.py     dead-reckoning estimators (kinematic / gyro / AHRS / complementary)
  evaluate.py   APE / RPE vs GT (+ TUM export)
  plotting.py   figures 01..07
  report.py     markdown report
ros/
  ekf_deadreckoning.yaml   dead-reckoning EKF (vesc/odom vx + IMU AHRS yaw + gyro)
  replay_verify.sh         replay a bag through the real ekf_node, record output
  analyze_ekf.py           start-anchored EKF-vs-GT drift analysis + figure
  replay_ekf.launch.xml    (alternative launch-file replay)
tools/
  make_report_html.py      bundle a results dir into one self-contained index.html
results/<bagname>/         per-bag output (figures, report.md, ekf_replay/, index.html)
```
