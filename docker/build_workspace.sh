#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CANONICAL_BUILD_DIR="$ROOT/.simenv_build/build"
CANONICAL_DEVEL_DIR="$ROOT/.simenv_build/devel"
JOBS="${SIMENV_BUILD_JOBS:-$(nproc)}"
source /opt/ros/noetic/setup.bash
cd "$ROOT"
if [[ -z "${LIBTORCH_ROOT:-}" || ! -f "$LIBTORCH_ROOT/share/cmake/Torch/TorchConfig.cmake" ]]; then
  echo "LIBTORCH_ROOT must point to a LibTorch distribution containing share/cmake/Torch/TorchConfig.cmake." >&2
  echo "Mount it into the container and export LIBTORCH_ROOT before running this script." >&2
  exit 2
fi
echo "Building SimEnv in $CANONICAL_BUILD_DIR with $JOBS job(s)"
cmake -S "$ROOT/src" -B "$CANONICAL_BUILD_DIR" \
  -DCATKIN_DEVEL_PREFIX="$CANONICAL_DEVEL_DIR" \
  -DCMAKE_INSTALL_PREFIX="$ROOT/install" \
  -DCMAKE_BUILD_TYPE=Release \
  -DLIBTORCH_ROOT="$LIBTORCH_ROOT" \
  -G "Unix Makefiles"
cmake --build "$CANONICAL_BUILD_DIR" --parallel "$JOBS"
echo "Built successfully. Source $CANONICAL_DEVEL_DIR/setup.bash before running."
