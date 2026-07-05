#!/usr/bin/env bash
# Replay a bag through the REAL robot_localization ekf_node and record the result.
#   replay_verify.sh <bag> <record_out_dir> <log_dir> [imu_tf_yaw]
# Env overrides:
#   UNICORN_SETUP   path to the ROS env setup to source (default: the unicorn.sh in unicorn_ws)
# Handles sourcing the ROS env, the EKF config, the +90deg IMU TF, sim-time clock,
# and a robust cleanup (ros2 run orphans its child node, so kill by executable name).

BAG="$1"                       # bag dir
OUT="$2"                       # output recorded bag dir
LOG="$3"                       # log dir
IMU_YAW="${4:-1.5707963}"      # base_link->vesc_imu yaw (default +90deg; pass 0 to test TF sensitivity)

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG="$HERE/ekf_deadreckoning.yaml"
UNICORN_SETUP="${UNICORN_SETUP:-/home/js/unicorn_ws/src/unicorn-racing-stack/unicorn.sh}"
mkdir -p "$LOG"
rm -rf "$OUT"

source "$UNICORN_SETUP" >/dev/null 2>&1
if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ROS env not available (sourced '$UNICORN_SETUP'). Set UNICORN_SETUP." >&2
    exit 3
fi
echo "ROS_DISTRO=$ROS_DISTRO domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION  IMU_TF_yaw=$IMU_YAW"

# `ros2 run` spawns the node as a CHILD, so killing the ros2-run PID orphans the
# node -> a leftover EKF would double-publish /odometry/filtered and corrupt the
# next run. Kill the real executables by name, before AND after.
cleanup() {
    pkill -9 -f "robot_localization/ekf_node" 2>/dev/null
    pkill -9 -f "tf2_ros/static_transform_publisher" 2>/dev/null
}
cleanup; sleep 1

# 1) static TF base_link -> vesc_imu, sim time
ros2 run tf2_ros static_transform_publisher \
   --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw "$IMU_YAW" \
   --frame-id base_link --child-frame-id vesc_imu \
   --ros-args -p use_sim_time:=true >"$LOG/tf.log" 2>&1 &

# 2) EKF (dead reckoning: vesc/odom vx + imu AHRS yaw + gyro)
ros2 run robot_localization ekf_node --ros-args \
   -r __node:=ekf_deadreckoning \
   --params-file "$CFG" \
   -p use_sim_time:=true >"$LOG/ekf.log" 2>&1 &

# 3) recorder: EKF output + GT + raw odom
ros2 bag record -o "$OUT" --use-sim-time \
   /odometry/filtered /tracked_pose /vesc/odom >"$LOG/rec.log" 2>&1 &
REC_PID=$!

sleep 4   # let nodes + recorder come up

# 4) play only the standard-typed topics the EKF/compare need; publish /clock
echo "playing bag ..."
ros2 bag play "$BAG" --clock 200 \
   --topics /vesc/odom /vesc/sensors/imu /tracked_pose >"$LOG/play.log" 2>&1
echo "play done"

sleep 2
# recorder MUST shut down cleanly (SIGINT) so the mcap footer/index is written;
# kill -9 truncates the file and makes it unreadable.
kill -INT "$REC_PID" 2>/dev/null; sleep 4; kill -TERM "$REC_PID" 2>/dev/null
cleanup
wait 2>/dev/null
echo "=== ekf.log tail ==="; tail -4 "$LOG/ekf.log"
echo "=== recorded ==="; ls "$OUT" 2>/dev/null
