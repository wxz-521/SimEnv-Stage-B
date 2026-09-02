#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -n "${SIMENV_DEVEL_DIR:-}" ]; then
  DEVEL_DIR="$SIMENV_DEVEL_DIR"
elif [ -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
  DEVEL_DIR="$WORKSPACE_DIR/devel"
else
  DEVEL_DIR="$WORKSPACE_DIR/.simenv_build/devel"
fi
TEST_PORT="${SIMNAV_TEST_PORT:-11321}"
RESULT_FILE="${SIMNAV_TEST_RESULT:-/tmp/simnav_stage_b_result.json}"
MASTER_PID=""
LAUNCH_PID=""

terminate_process_group() {
  local pid="$1"
  [ -n "$pid" ] || return 0
  local child
  while read -r child; do
    terminate_process_group "$child"
  done < <(pgrep -P "$pid" 2>/dev/null || true)
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  set +e
  terminate_process_group "$LAUNCH_PID"
  terminate_process_group "$MASTER_PID"
}
trap cleanup EXIT INT TERM

if command -v ss >/dev/null && ss -ltn | rg -q ":$TEST_PORT\\b"; then
  echo "Test ROS master port $TEST_PORT is already in use; refusing to disturb an unknown process." >&2
  exit 1
fi
if [ ! -f "$DEVEL_DIR/setup.bash" ]; then
  echo "Missing $DEVEL_DIR/setup.bash. Build the workspace first." >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
source "$DEVEL_DIR/setup.bash"
export ROS_MASTER_URI="http://127.0.0.1:$TEST_PORT"
export ROS_HOSTNAME=127.0.0.1
export ROS_HOME="/tmp/simnav_stage_b_ros_home_$TEST_PORT"
export PYTHONPATH="$WORKSPACE_DIR/src/simnav/scripts:${PYTHONPATH:-}"

python3 -m unittest discover -s "$WORKSPACE_DIR/src/simnav/test" -p 'test_*_core.py' -v

rm -f "$RESULT_FILE" /tmp/simnav_detected_danger.json /tmp/simnav_detected_danger_debug.json
setsid roscore -p "$TEST_PORT" > /tmp/simnav_stage_b_roscore.log 2>&1 &
MASTER_PID=$!
for _ in $(seq 1 100); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  if ! kill -0 "$MASTER_PID" 2>/dev/null; then
    tail -n 80 /tmp/simnav_stage_b_roscore.log >&2
    exit 1
  fi
  sleep 0.05
done

setsid roslaunch simnav offline_stage_b_logic.launch result_file:="$RESULT_FILE" \
  expected_rooms_per_floor:="${STAGE_B_EXPECTED_ROOM_COUNT:-4}" \
  > /tmp/simnav_stage_b_logic.log 2>&1 &
LAUNCH_PID=$!
wait "$LAUNCH_PID"
LAUNCH_PID=""

test -f "$RESULT_FILE"
rg -q '"passed": true' "$RESULT_FILE"
cat "$RESULT_FILE"
