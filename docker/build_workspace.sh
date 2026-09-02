#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${SIMENV_DEVEL_DIR:-$ROOT/devel}"
JOBS="${SIMENV_BUILD_JOBS:-$(nproc)}"
source /opt/ros/noetic/setup.bash
cd "$ROOT"
if [[ -z "${LIBTORCH_ROOT:-}" || ! -f "$LIBTORCH_ROOT/share/cmake/Torch/TorchConfig.cmake" ]]; then
  echo "LIBTORCH_ROOT must point to a LibTorch distribution containing share/cmake/Torch/TorchConfig.cmake." >&2
  echo "Mount it into the container and export LIBTORCH_ROOT before running this script." >&2
  exit 2
fi
echo "Building SimEnv in $BUILD_DIR with $JOBS job(s)"
catkin_make -j"$JOBS" -DCMAKE_BUILD_TYPE=Release -DLIBTORCH_ROOT="$LIBTORCH_ROOT"
echo "Built successfully. Source $BUILD_DIR/setup.bash before running."
