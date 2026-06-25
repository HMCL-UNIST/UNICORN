#!/usr/bin/env bash
#
# setup_conda_onLaptop.sh — sim-only / laptop setup.
#
# Same as setup_conda_onCar.sh, but first marks the HARDWARE-only packages with
# COLCON_IGNORE so colcon skips them, while keeping the bits a laptop still needs:
#   - skip: urg_node (lidar driver), vesc_driver/vesc_ackermann/vesc (motor HW),
#           particle_filter (on-car localization).
#   - keep: vesc_msgs (so you can `ros2 topic echo` VESC messages), and the full
#           cartographer stack incl. cartographer_rviz (to view submaps in RViz —
#           cartographer_rviz depends on cartographer + cartographer_ros{,_msgs}).
#
#   conda activate base      # (or have conda/mamba on PATH)
#   ./setup_conda_onLaptop.sh
#
# To go back to a full (car) build on this checkout, just run
# ./setup_conda_onCar.sh — it clears these COLCON_IGNORE files on a direct run.

set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

# Hardware-only packages to skip on a laptop. KEEP THIS LIST IN SYNC with the
# CAR_ONLY list in setup_conda_onCar.sh. Note: vesc_msgs and all cartographer
# packages are intentionally NOT here (echo + submap RViz need them).
CAR_ONLY=(sensor/urg_node \
          sensor/vesc/vesc_driver sensor/vesc/vesc_ackermann sensor/vesc/vesc \
          state_estimation)

echo "==> marking car-only packages COLCON_IGNORE (sim/laptop build)…"
for p in "${CAR_ONLY[@]}"; do
  if [ -d "$REPO/$p" ]; then
    touch "$REPO/$p/COLCON_IGNORE"
    echo "   ignore: $p"
  else
    echo "   (missing, skipping) $p"
  fi
done

# Tell the car script to KEEP these ignores (so it doesn't clear them on entry),
# then hand off to it for env + deps + build.
export URS_KEEP_COLCON_IGNORE=1
exec "$REPO/setup_conda_onCar.sh"
