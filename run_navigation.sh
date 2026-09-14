#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CANONICAL_DEVEL_DIR="$WORKSPACE_DIR/.simenv_build/devel"
DEVEL_DIR="${SIMENV_DEVEL_DIR:-$CANONICAL_DEVEL_DIR}"
if [ "$(readlink -f "$DEVEL_DIR")" != "$(readlink -f "$CANONICAL_DEVEL_DIR")" ]; then
  echo "Refusing non-canonical build: SIMENV_DEVEL_DIR must be $CANONICAL_DEVEL_DIR" >&2
  exit 3
fi

if [ ! -f "$DEVEL_DIR/setup.bash" ]; then
  echo "Missing $DEVEL_DIR/setup.bash. Build the workspace first." >&2
  exit 1
fi

source "$DEVEL_DIR/setup.bash"
export ROS_PACKAGE_PATH="$WORKSPACE_DIR/src:${ROS_PACKAGE_PATH:-}"

exec roslaunch simnav simnav.launch
