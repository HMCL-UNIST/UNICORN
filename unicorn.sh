#!/usr/bin/env bash
# unicorn.sh — enter the unicorn-racing-stack dev environment in one step.
#
# SOURCE it (do not execute). Works in bash AND zsh. setup_conda_onCar.sh adds the alias
# to your ~/.bashrc and ~/.zshrc:
#     alias unicorn='source /path/to/unicorn-racing-stack/unicorn.sh'
# then just run:  unicorn
#
# It (1) activates the RoboStack conda env, (2) selects CycloneDDS + ROS domain,
# (3) sources the colcon workspace, and (4) defines ros2kill / cbuild helpers.

# --- locate this repo and the colcon workspace root (<ws>/src/<repo>) ---
# Resolve this script's own path under whichever shell sourced it (bash sets
# BASH_SOURCE; zsh uses ${(%):-%x}).
if [ -n "${BASH_SOURCE:-}" ]; then
    _URS_SRC="${BASH_SOURCE[0]}"
elif [ -n "${ZSH_VERSION:-}" ]; then
    _URS_SRC="${(%):-%x}"
else
    _URS_SRC="$0"
fi
_URS_REPO="$(cd "$(dirname "$_URS_SRC")" && pwd)"
_URS_WS="$(cd "$_URS_REPO/../.." && pwd)"

# --- 1) conda env: RoboStack ROS 2 Jazzy ('unicorn') ---
source "$(conda info --base)/etc/profile.d/conda.sh"

# Start from a CLEAN ROS environment. These vars are 100% ROS-owned, so reset
# them BEFORE activating: whatever the host shell leaked — a system ROS or another
# workspace `source`d in ~/.bashrc, ANY distro, ANY path — is discarded, and
# conda's activation + this workspace rebuild them. You can't pattern-match every
# user's ~/.bashrc, so don't try; reset to a known-good baseline instead.
unset AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX CMAKE_PREFIX_PATH COLCON_PREFIX_PATH \
      ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH 2>/dev/null

conda activate unicorn

# Never let ~/.local user-site packages shadow the conda env (stale numba, etc.).
export PYTHONNOUSERSITE=1

# The mixed path vars (PYTHONPATH/LD_LIBRARY_PATH/PATH also carry non-ROS entries
# like CUDA) can't just be unset, so drop only the ROS leakage: /opt/ros/* and
# apt-style ROS python dirs (.../lib/pythonX/dist-packages — conda/colcon use
# site-packages, so dist-packages is exclusively system ROS). This is what
# shadowed rosidl_generator_c on the Orin ("generate_c() takes 1 arg but 2 given").
# Portable (bash + zsh): filter one colon-list, echo the survivors.
_urs_filter_path() {
    local old="$1" new="" p rest="$1"
    while [ -n "$rest" ]; do
        case "$rest" in
            *:*) p="${rest%%:*}"; rest="${rest#*:}" ;;
            *)   p="$rest";       rest="" ;;
        esac
        case "$p" in
            /opt/ros/*) continue ;;
            */lib/python*/dist-packages) continue ;;
        esac
        new="${new:+$new:}$p"
    done
    printf '%s' "$new"
}
[ -n "${PYTHONPATH:-}" ]     && export PYTHONPATH="$(_urs_filter_path "$PYTHONPATH")"
[ -n "${LD_LIBRARY_PATH:-}" ] && export LD_LIBRARY_PATH="$(_urs_filter_path "$LD_LIBRARY_PATH")"
export PATH="$(_urs_filter_path "$PATH")"

# --- 2) middleware + ROS domain ---
# CycloneDDS is far lighter than the default FastDDS on this many-node single-host
# graph: FastDDS busy-spins a whole core (~22 Hz sim), CycloneDDS idles at ~21%
# CPU and hits the full 80 Hz. IMPORTANT: `conda activate` clears
# RMW_IMPLEMENTATION, so it must be (re)set AFTER activation — that is the whole
# reason this lives in a sourced script instead of ~/.bashrc.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$_URS_REPO/cyclonedds.xml"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"

# --- 3) colcon workspace overlay + gym raycaster dir ---
# colcon generates setup.{bash,zsh,sh}; source the one matching the live shell.
if [ -n "${ZSH_VERSION:-}" ]; then _urs_setup=setup.zsh; else _urs_setup=setup.bash; fi
[ -f "$_URS_WS/install/$_urs_setup" ] && source "$_URS_WS/install/$_urs_setup"
export RAYCASTER_DIR="$_URS_REPO/race_utils/raycaster"

# --- macOS portability (no-op on Linux) ---
# (1) Linux-only CLIs (taskset, ...) some launch files use via launch-prefix:
#     put repo-tracked shims on PATH so they don't crash on macOS.
# (2) ros2 launch spawns nodes with DYLD_* stripped by SIP, so they can't find the
#     workspace rosidl typesupport dylibs in install/<pkg>/lib -> "type_support is
#     null" deaths. Mirror them into $CONDA_PREFIX/lib (on the python rpath).
if [ "$(uname)" = "Darwin" ]; then
    export PATH="$_URS_REPO/.install_utils/macos-shims:$PATH"
    URS_QUIET_LINK=1 bash "$_URS_REPO/.install_utils/macos_link_rosidl_typesupports.sh" "$_URS_WS" 2>/dev/null || true
fi

# --- 4) helpers ---
# Kill every ROS 2 process (nodes, launchers, daemon), any package/language.
ros2kill() {
    ros2 daemon stop 2>/dev/null                     # graceful CLI daemon shutdown
    pkill -9 -f '_ros2_daemon'      2>/dev/null       # the daemon process
    pkill -9 -f -- '--ros-args'     2>/dev/null       # any node started via ros2 run/launch
    pkill -9 -f 'ros2 (run|launch)' 2>/dev/null       # the launcher itself
    pkill -9 -f '/opt/ros/'         2>/dev/null       # rviz2 etc. from a ROS install path
    echo "[ros2kill] killed all ROS 2 nodes"
}

# Open the pitwall RViz (Sim Control + telemetry panel). More tools may join
# this launch later. Pass-through args go to ros2 launch.
alias pitwall='ros2 launch stack_master pitwall.launch.xml'

# Live cartographer matching-health monitor (frozen-map constraints, match
# scores, matcher/backend queues). Needs cartographer_node -collect_metrics.
alias cartometrics='python3 "$_URS_REPO/stack_master/scripts/carto_metrics.py" --watch 2'

# colcon build (Release) + re-source. No args = whole workspace; args = packages.
cbuild() {
    local sel=()
    [ $# -gt 0 ] && sel=(--packages-select "$@")
    ( cd "$_URS_WS" && colcon build "${sel[@]}" --symlink-install \
          --cmake-args -DCMAKE_BUILD_TYPE=Release ) \
        && source "$_URS_WS/install/$_urs_setup" \
        && bash "$_URS_REPO/.install_utils/macos_link_rosidl_typesupports.sh" "$_URS_WS"
}

echo "[unicorn] env ready  |  RMW=$RMW_IMPLEMENTATION  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  |  helpers: cbuild, ros2kill, cartometrics"
