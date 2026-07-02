#!/usr/bin/env bash
#
# setup_conda_onCar.sh — set up unicorn-racing-stack in ONE shot (FULL build,
# including the sensor drivers + SLAM). Run after you've cloned it.
#
#   conda activate base      # (or have conda/mamba on PATH — see README "Get started")
#   ./setup_conda_onCar.sh   # creates the 'unicorn' env, installs deps, builds everything
#
# For a laptop / sim-only build (skips urg_node, vesc, cartographer{,_ros},
# particle_filter) use setup_conda_onLaptop.sh, which adds COLCON_IGNORE files
# and then runs THIS script.
#
# Cross-platform: works from bash OR zsh (it always re-execs under bash), and it
# registers the 'unicorn' alias into BOTH ~/.bashrc and ~/.zshrc.
#
# What it does, in order (see the README "Get started" section for the why):
#   1) conda env create -f environment.yml          -> env 'unicorn' (ROS 2 Jazzy)
#   2) register `alias unicorn='source .../unicorn.sh'` in ~/.bashrc + ~/.zshrc
#   3) pip layer under the env (requirements, gym core, range_libc)
#   4) swap the broken quadprog PyPI wheel for the conda-forge build
#   5) raise OS socket buffers for CycloneDDS (idempotent, sudo, non-fatal)
#   6) colcon build (Release)

# No `-u` (nounset): conda re-runs the env's activate.d hooks after every
# `conda activate`/`conda install`, and RoboStack's hooks reference unbound vars
# (e.g. CONDA_BUILD), which would abort the script. We still get -e and pipefail.
set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"   # .../src/unicorn-racing-stack
WS="$(cd "$REPO/../.." && pwd)"                            # colcon workspace root
ENV_NAME="unicorn"

# Hardware-only packages the laptop build excludes via COLCON_IGNORE. On a DIRECT
# car run (no marker from setup_conda_onLaptop.sh), clear any leftover ignores so
# this really builds everything — otherwise a prior laptop run would silently skip
# them. KEEP THIS LIST IN SYNC with the CAR_ONLY list in setup_conda_onLaptop.sh.
CAR_ONLY=(sensor/urg_node \
          sensor/vesc/vesc_driver sensor/vesc/vesc_ackermann sensor/vesc/vesc \
          state_estimation/particle_filter)
if [ -z "${URS_KEEP_COLCON_IGNORE:-}" ]; then
  for p in "${CAR_ONLY[@]}"; do rm -f "$REPO/$p/COLCON_IGNORE"; done
fi

say() { printf '\n\033[1;36m==>\033[0m %s\n' "$*"; }

# --- 0. sanity ---------------------------------------------------------------
command -v conda >/dev/null 2>&1 || {
  echo "ERROR: conda/mamba not found on PATH." >&2
  echo "       Install Miniforge first — see the README 'Get started' section." >&2
  exit 1
}
source "$(conda info --base)/etc/profile.d/conda.sh"

# --- 1. conda env (ROS 2 Jazzy + toolchain + pinned libs) --------------------
if conda env list | awk '{print $1}' | grep -qxF "$ENV_NAME"; then
  say "conda env '$ENV_NAME' already exists — skipping create (run 'conda env update -f environment.yml' to refresh)"
else
  say "creating conda env '$ENV_NAME' from environment.yml (ROS 2 Jazzy + deps)…"
  conda env create -f "$REPO/environment.yml"
fi

# --- 2. register the 'unicorn' alias in bash AND zsh -------------------------
ALIAS_LINE="alias unicorn='source $REPO/unicorn.sh'"
for rc in "$HOME/.bashrc" "$HOME/.zshrc"; do
  # Only touch a zshrc if zsh is actually installed; always do bash.
  if [ "$rc" = "$HOME/.zshrc" ] && ! command -v zsh >/dev/null 2>&1; then
    continue
  fi
  touch "$rc"
  if grep -qF "source $REPO/unicorn.sh" "$rc"; then
    say "alias already in $(basename "$rc") — leaving it"
  else
    printf '\n# unicorn-racing-stack: enter the dev env with `unicorn`\n%s\n' "$ALIAS_LINE" >> "$rc"
    say "added 'unicorn' alias to $(basename "$rc")"
  fi
done

# --- 3. enter the env (with PYTHONNOUSERSITE so ~/.local can't shadow it) -----
conda activate "$ENV_NAME"
export PYTHONNOUSERSITE=1

# --- 4. pip layer ------------------------------------------------------------
say "pip: requirements + editable gym core + range_libc…"
cd "$REPO"
pip install -r requirements.txt
pip install -e ./race_utils/unicorn_gym/f1tenth_gym                             # gym core -> f110_gym
pip install --no-build-isolation -e ./race_utils/raycaster/range_libc/pywrapper # range_libc
pip install "git+https://github.com/ForzaETH/CCMA.git"                          # CCMA smoothing (dynamic-avoidance planners)

# --- 4b. numpy — macOS only: swap the PyPI wheel for the conda-forge build ----
# The pip layer above (scipy/numba deps) pulls the PyPI numpy wheel, which on
# macOS links Apple's Accelerate BLAS (NEWLAPACK/ILP64) and then crashes at
# import here: "Symbol not found: _cblas_caxpy$NEWLAPACK$ILP64 ... Accelerate".
# Re-assert the conda-forge (OpenBLAS) build that RoboStack ships. No-op on Linux,
# where the PyPI numpy wheel bundles its own OpenBLAS and works fine. Same swap
# pattern as quadprog below, and MUST run before the quadprog import test (quadprog
# imports numpy).
if [ "$(uname)" = "Darwin" ]; then
  say "macOS: restoring conda-forge numpy (pip pulled the Accelerate-linked PyPI wheel)…"
  pip uninstall -y numpy 2>/dev/null || true
  conda install -y --force-reinstall -c conda-forge "numpy=2.4"
  python -c "import numpy" || { echo "ERROR: numpy still broken after conda-forge reinstall" >&2; exit 1; }
fi

# --- 5. quadprog — swap the broken PyPI wheel (MUST be after step 4) ---------
# trajectory_planning_helpers re-pulls quadprog==0.1.7, whose wheel crashes the
# state machine at import; the conda-forge build is correct + API-compatible.
#
# --force-reinstall is REQUIRED: a plain `conda install quadprog=0.1.13` often
# reports "All requested packages already installed" while the files are actually
# missing (a "ghost" install) and then does nothing -> import fails. We then
# VERIFY the import and, if still broken, do a clean remove+reinstall and finally
# hard-fail. Otherwise a broken quadprog silently survives the build and only
# surfaces at runtime as `ModuleNotFoundError: No module named 'quadprog'` from
# gb_optimizer (global_planner dies -> no global waypoints -> sector yamls never
# get n_sectors -> sector_tuner/ot_interpolator KeyError).
say "swapping quadprog wheel for the conda-forge build…"
pip uninstall -y quadprog 2>/dev/null || true
conda install -y --force-reinstall -c conda-forge quadprog=0.1.13
if ! python -c "import quadprog" 2>/dev/null; then
  say "quadprog still not importable after install — clean reinstall…"
  conda remove -y --force quadprog 2>/dev/null || true
  pip uninstall -y quadprog 2>/dev/null || true
  conda install -y --force-reinstall -c conda-forge quadprog=0.1.13
  python -c "import quadprog" || { echo "ERROR: quadprog import still failing after reinstall" >&2; exit 1; }
fi
say "quadprog OK -> $(python -c 'import quadprog; print(quadprog.__file__)')"

# --- 6. OS socket buffers (so CycloneDDS can take the 10 MB cyclonedds.xml asks
#        for; the kernel default ~212 KB caps it otherwise). Idempotent: skips
#        when the live value is already >= target. Needs sudo; never fatal. -----
SOCKBUF_TARGET=26214400   # 25 MiB
case "$(uname)" in
  Linux)
    cur=$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)
    if [ "${cur:-0}" -ge "$SOCKBUF_TARGET" ]; then
      say "socket buffers already >= ${SOCKBUF_TARGET} (rmem_max=${cur}) — skipping"
    else
      say "raising net.core.r/wmem_max to ${SOCKBUF_TARGET} via /etc/sysctl.conf (sudo)…"
      { printf 'net.core.rmem_max=%s\nnet.core.wmem_max=%s\n' "$SOCKBUF_TARGET" "$SOCKBUF_TARGET" \
          | sudo tee -a /etc/sysctl.conf >/dev/null && sudo sysctl -p >/dev/null; } \
        || echo "  (skipped: no sudo / sysctl failed — DDS still works, just smaller buffers)"
    fi
    ;;
  Darwin)
    cur=$(sysctl -n kern.ipc.maxsockbuf 2>/dev/null || echo 0)
    if [ "${cur:-0}" -ge "$SOCKBUF_TARGET" ]; then
      say "macOS kern.ipc.maxsockbuf already >= ${SOCKBUF_TARGET} (${cur}) — skipping"
    else
      say "raising kern.ipc.maxsockbuf to ${SOCKBUF_TARGET} (sudo, this session only)…"
      sudo sysctl -w kern.ipc.maxsockbuf=$SOCKBUF_TARGET >/dev/null \
        || echo "  (skipped: no sudo — DDS still works, just smaller buffers)"
      echo "  note: macOS resets this on reboot; add a launchd plist to persist."
    fi
    ;;
esac

# --- 6b. WiFi power-save OFF (Linux/NetworkManager only) ---------------------
# NetworkManager parks the WiFi radio between packets to save power; on the car
# that shows up as periodic latency spikes / brief dropouts on the DDS traffic.
# Drop a conf.d snippet (wifi.powersave=2 == disabled) and reload. Idempotent:
# skips when the file already has the setting. Needs sudo; never fatal.
if [ "$(uname)" = "Linux" ] && command -v nmcli >/dev/null 2>&1; then
  PSAVE_CONF=/etc/NetworkManager/conf.d/wifi-powersave-off.conf
  if [ -f "$PSAVE_CONF" ] && grep -q 'wifi.powersave *= *2' "$PSAVE_CONF" 2>/dev/null; then
    say "WiFi power-save already disabled ($PSAVE_CONF) — skipping"
  else
    say "disabling WiFi power-save via NetworkManager (sudo)…"
    { printf '[connection]\nwifi.powersave = 2\n' | sudo tee "$PSAVE_CONF" >/dev/null \
        && sudo systemctl restart NetworkManager; } \
      || echo "  (skipped: no sudo / NetworkManager reload failed — network still works)"
  fi
fi

# --- 7. build ----------------------------------------------------------------
# Build in a CLEAN, isolated environment so a system ROS the caller's ~/.bashrc
# sourced (e.g. `source /opt/ros/noetic/setup.bash`) cannot leak into the build.
# If it leaks, colcon detects /opt/ros/<distro> as a parent prefix and BAKES it
# into install/setup.bash; then every `unicorn` entry re-sources ROS1 and nodes die
# with `_TYPE_SUPPORT`. `conda activate` only PREPENDS the env, it never removes the
# leak (it even re-appends the pre-activate CMAKE_PREFIX_PATH), so we don't try to
# scrub it -- we start from `env -i` (empty env) and let conda build the env fresh.
# This needs no allow/deny path list and is immune to any distro/rc the user has.
say "colcon build (Release, in isolated env)…"
CONDA_BASE="$(conda info --base)"
env -i HOME="$HOME" USER="${USER:-$(id -un)}" TERM="${TERM:-xterm}" \
       PATH="/usr/bin:/bin" \
  bash -c '
    set -eo pipefail
    source "'"$CONDA_BASE"'/etc/profile.d/conda.sh"
    conda activate "'"$ENV_NAME"'"
    export PYTHONNOUSERSITE=1
    cd "'"$WS"'" && colcon build --symlink-install \
        --base-paths "src/'"$(basename "$REPO")"'" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
  '

say "done. Open a NEW shell (or 'source ~/.bashrc') then run:  unicorn"
echo "    ros2 launch stack_master race.launch.xml sim:=true map:=f"
