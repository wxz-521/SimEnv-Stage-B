#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_DIR="${1:-$WORKSPACE_DIR/logs/rl_speed_sweep_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$RUN_DIR"

AUTO_PID=""
CHILD_PIDS=()

collect_tree() {
  local parent="$1"
  local child
  for child in $(pgrep -P "$parent" 2>/dev/null || true); do
    CHILD_PIDS+=("$child")
    collect_tree "$child"
  done
}

cleanup() {
  set +e
  if [ -n "$AUTO_PID" ]; then
    CHILD_PIDS=()
    collect_tree "$AUTO_PID"
    kill -TERM "$AUTO_PID" "${CHILD_PIDS[@]}" 2>/dev/null || true
    kill -TERM -- "-$AUTO_PID" 2>/dev/null || true
    sleep 2
    kill -KILL "$AUTO_PID" "${CHILD_PIDS[@]}" 2>/dev/null || true
    kill -KILL -- "-$AUTO_PID" 2>/dev/null || true
  fi
  AUTO_PID=""
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

cd "$WORKSPACE_DIR"
export SIMENV_DEVEL_DIR="${SIMENV_DEVEL_DIR:-$WORKSPACE_DIR/.simenv_build/devel}"
source /opt/ros/noetic/setup.bash
source "$SIMENV_DEVEL_DIR/setup.bash"
export ROS_PACKAGE_PATH="$WORKSPACE_DIR/src:${ROS_PACKAGE_PATH:-}"
export GUI=false
export PAUSED=false
export AUTO_UNPAUSE=1
export START_CONTROLLER=1
export CONTROLLER_FOREGROUND=0
export CONTROLLER_USE_PTY=1
export START_VIRTUAL_JOY=0
export START_BUILDING_CONTROL=0
export ENABLE_SENSOR_DATA=0
export ENABLE_LIVOX=0
export ENABLE_LIVOX_IMU=0
export ENABLE_REALSENSE=false
export ENABLE_FRONT_CAMERA=false
export ENABLE_GROUND_TRUTH=1
export ENABLE_REFEREE_ODOM=0
export ENABLE_POINTCLOUD_CONVERTER=0
export WRITE_GENERATED_TRUTH_COPY=0

setsid ./auto.sh >"$RUN_DIR/auto.log" 2>&1 &
AUTO_PID=$!
echo "$AUTO_PID" >"$RUN_DIR/runner.pid"

for _ in $(seq 1 180); do
  if rosservice list 2>/dev/null | grep -q '^/gazebo/get_model_state$'; then
    break
  fi
  sleep 1
done
if ! rosservice list 2>/dev/null | grep -q '^/gazebo/get_model_state$'; then
  echo "Timed out waiting for Gazebo services" >&2
  exit 1
fi

python3 team_scripts/activate_stage_b_controller.py --mode sequence --stand-hold 2.0 \
  >"$RUN_DIR/activate.log" 2>&1
python3 team_scripts/measure_rl_speed.py \
  --speeds "${SPEEDS:-0.30,0.45,0.60,0.75,0.90,1.05}" \
  --phase "${PHASE:-4.0}" \
  --settle "${SETTLE:-1.0}" \
  --output "$RUN_DIR/result.json" \
  | tee "$RUN_DIR/result.stdout.json"
