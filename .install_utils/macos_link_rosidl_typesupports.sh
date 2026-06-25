#!/usr/bin/env bash
# macOS-only: make the workspace's rosidl typesupport dylibs discoverable at runtime.
#
# WHY: `ros2 launch` spawns each node as a child process, and macOS SIP / hardened
# runtime STRIPS the DYLD_* environment variables from those children. The ament
# `library_path` hook puts install/<pkg>/lib on DYLD_LIBRARY_PATH, so a node works
# when run directly from the shell — but once launch-spawned, DYLD_LIBRARY_PATH is
# gone, and CycloneDDS (rmw) loads the *introspection* typesupport by bare name via
# dlopen. dlopen then only searches the paths baked into the python binary's rpath
# (notably $CONDA_PREFIX/lib), NOT install/<pkg>/lib -> "type_support is null" and
# the node dies (tracking_node, opp_prediction, gaussian_process_opp_traj, ...).
#
# FIX: symlink the workspace's lib*__rosidl_*.dylib into $CONDA_PREFIX/lib, which IS
# on that rpath and therefore survives the DYLD_* stripping. Idempotent. No-op off
# macOS (Linux keeps DYLD/LD_LIBRARY_PATH across spawn, so nothing to do).
#
# Runs automatically from unicorn.sh (every env entry) and cbuild (after a rebuild).
set -eu

[ "$(uname)" = "Darwin" ] || exit 0
: "${CONDA_PREFIX:?[link-rosidl] activate the conda env first}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
# arg 1 = workspace root; default = three levels up (ws/src/<repo>/.install_utils)
WS="${1:-$(cd "$HERE/../../.." && pwd)}"
DEST="$CONDA_PREFIX/lib"

[ -d "$WS/install" ] || { echo "[link-rosidl] no $WS/install yet — build first"; exit 0; }

n=0
for f in "$WS"/install/*/lib/lib*__rosidl_*.dylib; do
  [ -e "$f" ] || continue
  ln -sf "$f" "$DEST/$(basename "$f")" && n=$((n + 1))
done
[ "${URS_QUIET_LINK:-0}" = "1" ] || echo "[link-rosidl] linked $n rosidl typesupport dylibs into \$CONDA_PREFIX/lib"
