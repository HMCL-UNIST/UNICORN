#!/usr/bin/env bash
# analyze_bag.sh — one bag in, full analysis out.
#
#   ./analyze_bag.sh /path/to/rosbag2_dir_or.mcap [options]
#
# Runs, end to end:
#   1. offline VESC gain ID + dead-reckoning evaluation  -> figures 01..07 + report.md
#   2. replay through the REAL robot_localization EKF     -> ekf_replay/*.png + metrics
#   3. bundles everything into a single self-contained    -> index.html
#
# Stage 2 sources the unicorn ROS env automatically; if ROS isn't available it is
# skipped (the offline 'ahrs_gyro' estimator already emulates it) and you still get
# the figures + html.
#
# Options:
#   --out DIR            output dir            (default: results/<bagname>)
#   --speed-gain G       reference config the recording car used (for the report)
#   --servo-gain G       "
#   --servo-offset O     "
#   --t-start S / --t-end S   crop the bag
#   --imu-yaw RAD        base_link->vesc_imu yaw for the EKF TF (default +pi/2)
#   --no-ekf             skip the robot_localization replay stage

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYBASE="$(command -v python3)"          # base env (has mcap/numpy/scipy/matplotlib)

BAG=""; OUT=""; DO_EKF=1; IMU_YAW="1.5707963"; EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --out)          OUT="$2"; shift 2;;
    --speed-gain)   EXTRA+=(--speed-gain "$2"); shift 2;;
    --servo-gain)   EXTRA+=(--servo-gain "$2"); shift 2;;
    --servo-offset) EXTRA+=(--servo-offset "$2"); shift 2;;
    --t-start)      EXTRA+=(--t-start "$2"); shift 2;;
    --t-end)        EXTRA+=(--t-end "$2"); shift 2;;
    --imu-yaw)      IMU_YAW="$2"; shift 2;;
    --no-ekf)       DO_EKF=0; shift;;
    -h|--help)      sed -n '2,30p' "${BASH_SOURCE[0]}"; exit 0;;
    *)              BAG="$1"; shift;;
  esac
done

[ -z "$BAG" ] && { echo "ERROR: give a bag (dir or .mcap)"; exit 1; }
[ -e "$BAG" ] || { echo "ERROR: no such bag: $BAG"; exit 1; }
NAME="$(basename "${BAG%/}")"; NAME="${NAME%.mcap}"
[ -z "$OUT" ] && OUT="$HERE/results/$NAME"
mkdir -p "$OUT"
echo "bag : $BAG"
echo "out : $OUT"

echo "==> [1/3] offline calibration + dead-reckoning figures (1-7)"
"$PYBASE" "$HERE/run.py" "$BAG" --out "$OUT" "${EXTRA[@]}" || { echo "offline stage failed"; exit 1; }

if [ "$DO_EKF" = 1 ]; then
  echo "==> [2/3] robot_localization replay (real ekf_node; sources unicorn ROS env)"
  mkdir -p "$OUT/ekf_replay"
  REC="$OUT/ekf_replay/rec"
  bash -l "$HERE/ros/replay_verify.sh" "$BAG" "$REC" "$OUT/ekf_replay/logs" "$IMU_YAW" \
       > "$OUT/ekf_replay/replay.log" 2>&1
  MCAP="$(ls "$REC"/*.mcap 2>/dev/null | head -1)"
  if [ -n "$MCAP" ]; then
    "$PYBASE" "$HERE/ros/analyze_ekf.py" "$MCAP" "$OUT/ekf_replay/ekf_vs_gt" "$NAME" \
         2>&1 | tee "$OUT/ekf_replay/metrics.txt" \
      || echo "  ! EKF analysis skipped (see ekf_replay/replay.log)"
  else
    echo "  ! no recorded mcap — ROS env unavailable? skipping (see ekf_replay/replay.log)"
  fi
else
  echo "==> [2/3] robot_localization replay skipped (--no-ekf)"
fi

echo "==> [3/3] building HTML dashboard"
"$PYBASE" "$HERE/tools/make_report_html.py" "$OUT" "$NAME"

echo
echo "DONE."
echo "  report : $OUT/report.md"
echo "  html   : $OUT/index.html"
