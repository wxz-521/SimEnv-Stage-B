#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/noetic/setup.bash
if [[ -f /workspace/SimEnv/.simenv_build/devel/setup.bash ]]; then
  source /workspace/SimEnv/.simenv_build/devel/setup.bash
fi
export ROS_PACKAGE_PATH="/workspace/SimEnv/src:${ROS_PACKAGE_PATH:-}"
exec "$@"
