#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SEED_VALUE="${1:?usage: run_stage_b_seed.sh SEED [SIM_TIMEOUT] [OUTPUT_ROOT]}"
SIM_TIMEOUT="${2:-600}"
OUTPUT_ROOT="${3:-$WORKSPACE_DIR/logs/stage_b_matrix}"
RUN_MODE="${4:-coverage}"
START_RVIZ="${STAGE_B_START_RVIZ:-1}"
RVIZ_CONFIG="${STAGE_B_RVIZ_CONFIG:-$WORKSPACE_DIR/src/simnav/rviz/stage_b.rviz}"
GAZEBO_GUI="${STAGE_B_GUI:-true}"
RUN_DIR="$OUTPUT_ROOT/seed_$SEED_VALUE"
RUNTIME_ROOT="${STAGE_B_RUNTIME_ROOT:-$WORKSPACE_DIR}"
SCENE_DIR="$RUNTIME_ROOT/generated_building"
RESULTS_DIR="$RUNTIME_ROOT/results"
RUNTIME_LOG_DIR="$RUNTIME_ROOT/logs"
DEVEL_DIR="${SIMENV_DEVEL_DIR:-$WORKSPACE_DIR/.simenv_build/devel}"
STAGE_B_POLICY_PATH="${UNITREE_POLICY_PATH:-$WORKSPACE_DIR/src/unitree_guide/logs/policy_act_inference_stair.pt}"
STAGE_B_PLANE_POLICY_PATH="${UNITREE_PLANE_POLICY_PATH:-$WORKSPACE_DIR/src/unitree_guide/logs/policy_act_inference_plane.pt}"
LOCK_FILE="$RUNTIME_ROOT/.stage_b_runner.lock"
ROS_PORT="${SIMNAV_STAGE_B_PORT:-11320}"
GAZEBO_PORT="${SIMNAV_STAGE_B_GAZEBO_PORT:-11345}"
CPU_LIST="${SIMNAV_CPU_LIST:-}"
RUN_PREFIX=()
CORE_PID=""
AUTO_PID=""
NAV_PID=""
BEHAVIOR_PID=""
MONITOR_PID=""
RVIZ_PID=""

mkdir -p "$RUNTIME_ROOT" "$SCENE_DIR" "$RESULTS_DIR" "$RUNTIME_LOG_DIR"
if [ ! -f "$STAGE_B_POLICY_PATH" ]; then
  echo "Stage B locomotion policy not found: $STAGE_B_POLICY_PATH" >&2
  exit 3
fi
if [ ! -f "$STAGE_B_PLANE_POLICY_PATH" ]; then
  echo "Stage B plane locomotion policy not found: $STAGE_B_PLANE_POLICY_PATH" >&2
  exit 3
fi
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "another Stage B runner is already active: $LOCK_FILE" >&2
  exit 2
fi

if command -v ss >/dev/null 2>&1 && ss -ltn 2>/dev/null | rg -q ":$ROS_PORT\b"; then
  echo "Stage B ROS port $ROS_PORT is already in use; refusing to disturb an unknown process." >&2
  exit 3
fi
if command -v ss >/dev/null 2>&1 && ss -ltn 2>/dev/null | rg -q ":$GAZEBO_PORT\b"; then
  echo "Stage B Gazebo port $GAZEBO_PORT is already in use; refusing to disturb an unknown process." >&2
  exit 3
fi
if [ "${STAGE_B_ALLOW_CONCURRENT:-0}" != "1" ]; then
  for process_pattern in \
    'gzserver' 'gzclient' 'fastlio_mapping' 'junior_ctrl' \
    'coverage_explorer_node.py' 'danger_detector_node.py' \
    'stage_b_localization.launch' 'stage_b_behavior.launch' 'rviz -d' \
    'monitor_stage_b_coverage.py'; do
    if pgrep -af "$process_pattern" 2>/dev/null | awk -v self="$$" \
      '$1 != self { found = 1 } END { exit !found }'; then
      echo "an existing process matching $process_pattern is running in the SimEnv container; refusing to share Gazebo/ROS resources." >&2
      exit 3
    fi
  done
fi
if [ -n "$CPU_LIST" ]; then
  if ! command -v taskset >/dev/null 2>&1; then
    echo "SIMNAV_CPU_LIST=$CPU_LIST was requested but taskset is unavailable." >&2
    exit 3
  fi
  if ! taskset --cpu-list "$CPU_LIST" true >/dev/null 2>&1; then
    echo "SIMNAV_CPU_LIST=$CPU_LIST is not a valid CPU set for this container." >&2
    exit 3
  fi
  RUN_PREFIX=(taskset --cpu-list "$CPU_LIST")
fi

terminate_process_group() {
  local pid="$1"
  [ -n "$pid" ] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    local child
    while read -r child; do
      terminate_process_group "$child"
    done < <(pgrep -P "$pid" 2>/dev/null || true)
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  set +e
  terminate_process_group "$MONITOR_PID"
  terminate_process_group "$RVIZ_PID"
  terminate_process_group "$BEHAVIOR_PID"
  terminate_process_group "$NAV_PID"
  terminate_process_group "$AUTO_PID"
  terminate_process_group "$CORE_PID"
}
trap cleanup EXIT INT TERM

mkdir -p "$RUN_DIR"
source /opt/ros/noetic/setup.bash
source "$DEVEL_DIR/setup.bash"
export ROS_PACKAGE_PATH="$WORKSPACE_DIR/src:${ROS_PACKAGE_PATH:-}"
export PYTHONPATH="$WORKSPACE_DIR/src/simnav/scripts:${PYTHONPATH:-}"
export ROS_MASTER_URI="http://127.0.0.1:$ROS_PORT"
export GAZEBO_MASTER_URI="http://127.0.0.1:$GAZEBO_PORT"
echo "Stage B resource guard: ROS_PORT=$ROS_PORT GAZEBO_PORT=$GAZEBO_PORT CPU_LIST=${CPU_LIST:-inherited}" > "$RUN_DIR/resource_guard.log"
echo "Stage B locomotion policy: $STAGE_B_POLICY_PATH" >> "$RUN_DIR/resource_guard.log"
echo "Stage B plane policy: $STAGE_B_PLANE_POLICY_PATH" >> "$RUN_DIR/resource_guard.log"

"${RUN_PREFIX[@]}" setsid roscore -p "$ROS_PORT" > "$RUN_DIR/roscore.log" 2>&1 &
CORE_PID=$!
for _ in $(seq 1 100); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  if ! kill -0 "$CORE_PID" 2>/dev/null; then
    echo "roscore exited during startup on port $ROS_PORT" >&2
    tail -n 80 "$RUN_DIR/roscore.log" >&2
    exit 1
  fi
  sleep 0.1
done
if ! rosparam list >/dev/null 2>&1; then
  echo "Timed out waiting for roscore on port $ROS_PORT" >&2
  exit 1
fi

wait_for_topic() {
  local topic="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if timeout 2s rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
      return 0
    fi
    if [ -n "$AUTO_PID" ] && ! kill -0 "$AUTO_PID" 2>/dev/null; then
      echo "auto.sh exited while waiting for $topic" >&2
      tail -n 100 "$RUN_DIR/auto.log" >&2
      return 1
    fi
    sleep 0.2
  done
  echo "Timed out waiting for $topic" >&2
  return 1
}

copy_artifacts() {
  cp -f "$SCENE_DIR/scene_manifest.json" "$RUN_DIR/" 2>/dev/null || true
  cp -f "$SCENE_DIR/layout_metadata.json" "$RUN_DIR/" 2>/dev/null || true
  cp -f "$RESULTS_DIR/danger_truth.json" "$RUN_DIR/" 2>/dev/null || true
  cp -f "$RESULTS_DIR/detected_danger.json" "$RUN_DIR/" 2>/dev/null || true
  cp -f "$RESULTS_DIR/detected_danger_debug.json" "$RUN_DIR/" 2>/dev/null || true
  cp -f "$RUNTIME_LOG_DIR/competition_gazebo.log" "$RUN_DIR/" 2>/dev/null || true
  cp -f "$RUNTIME_LOG_DIR/junior_ctrl.log" "$RUN_DIR/" 2>/dev/null || true
}

save_visual_artifacts() {
  if rostopic list 2>/dev/null | rg -qx '/exploration_map'; then
    timeout 20s rosrun map_server map_saver \
      -f "$RUN_DIR/exploration_map" map:=/exploration_map \
      > "$RUN_DIR/map_saver.log" 2>&1 || true
  fi
  if [ "$START_RVIZ" = "1" ] && [ -n "${DISPLAY:-}" ] && command -v import >/dev/null 2>&1; then
    import -window root "$RUN_DIR/rviz_final.png" \
      > "$RUN_DIR/rviz_screenshot.log" 2>&1 || true
  fi
}

rm -f "$RESULTS_DIR/detected_danger.json" \
  "$RESULTS_DIR/detected_danger_debug.json" "$RUN_DIR/result.json"

cd "$WORKSPACE_DIR"
GUI="$GAZEBO_GUI" PAUSED=true AUTO_UNPAUSE=1 AUTO_UNPAUSE_DELAY=6 \
  UNITREE_POLICY_PATH="$STAGE_B_POLICY_PATH" \
  UNITREE_PLANE_POLICY_PATH="$STAGE_B_PLANE_POLICY_PATH" \
  SIMENV_DEVEL_DIR="$DEVEL_DIR" \
  SIMENV_SCENE_OUTPUT_DIR="$SCENE_DIR" SIMENV_RESULTS_DIR="$RESULTS_DIR" \
  SIMENV_RUNTIME_LOG_DIR="$RUNTIME_LOG_DIR" \
  START_CONTROLLER=1 CONTROLLER_FOREGROUND=0 START_VIRTUAL_JOY=0 \
  ENABLE_GROUND_TRUTH="${STAGE_B_ENABLE_GROUND_TRUTH:-0}" ENABLE_REFEREE_ODOM=0 \
  POINTCLOUD_USE_GROUND_TRUTH_ODOM=0 SEED="$SEED_VALUE" \
  "${RUN_PREFIX[@]}" setsid ./auto.sh > "$RUN_DIR/auto.log" 2>&1 &
AUTO_PID=$!

wait_for_topic /clock 180
wait_for_topic /trunk_imu 180
wait_for_topic /a1_gazebo/FR_hip_controller/state 180

python3 team_scripts/activate_stage_b_controller.py --mode stand \
  > "$RUN_DIR/controller_stand.log" 2>&1

"${RUN_PREFIX[@]}" setsid roslaunch simnav stage_b_localization.launch \
  team_scene_info:="$SCENE_DIR/team_scene_info.json" \
  > "$RUN_DIR/navigation.log" 2>&1 &
NAV_PID=$!
wait_for_topic /simnav/odom 120
wait_for_topic /exploration_map 120

if [ "$START_RVIZ" = "1" ]; then
  if [ -n "${DISPLAY:-}" ] && command -v rviz >/dev/null 2>&1; then
    "${RUN_PREFIX[@]}" setsid rviz -d "$RVIZ_CONFIG" > "$RUN_DIR/rviz.log" 2>&1 &
    RVIZ_PID=$!
    echo "RViz started with config $RVIZ_CONFIG (pid=$RVIZ_PID)" >> "$RUN_DIR/resource_guard.log"
  else
    echo "RViz not started: DISPLAY is unset or rviz is unavailable" >> "$RUN_DIR/resource_guard.log"
  fi
fi

python3 team_scripts/activate_stage_b_controller.py --mode rl \
  > "$RUN_DIR/controller_rl.log" 2>&1

if [ "$RUN_MODE" != "coverage" ] && [ "$RUN_MODE" != "full" ]; then
  echo "unsupported active run mode: $RUN_MODE (door modes are archived)" >&2
  exit 2
fi

"${RUN_PREFIX[@]}" setsid roslaunch simnav stage_b_behavior.launch \
  result_dir:="$RESULTS_DIR" \
  > "$RUN_DIR/behavior.log" 2>&1 &
BEHAVIOR_PID=$!
wait_for_topic /simnav/explorer_status 60

"${RUN_PREFIX[@]}" python3 team_scripts/monitor_stage_b_coverage.py \
  --sim-timeout "$SIM_TIMEOUT" --seed "$SEED_VALUE" \
  > "$RUN_DIR/result.json" 2> "$RUN_DIR/monitor.stderr" &
MONITOR_PID=$!

set +e
wait "$MONITOR_PID"
MONITOR_EXIT=$?
MONITOR_PID=""
set -e
save_visual_artifacts
copy_artifacts

set +e
python3 team_scripts/evaluate_stage_b_danger.py \
  --truth "$RUN_DIR/danger_truth.json" \
  --detected "$RUN_DIR/detected_danger.json" \
  --output "$RUN_DIR/danger_evaluation.json" \
  --summary "$RUN_DIR/result.json" \
  > "$RUN_DIR/danger_evaluation.log" 2>&1
DANGER_EXIT=$?
set -e
if [ "$DANGER_EXIT" -ne 0 ]; then
  MONITOR_EXIT=1
fi

exit "$MONITOR_EXIT"
