#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SEED_VALUE="${1:?usage: run_stage_b_seed.sh SEED [SIM_TIMEOUT] [OUTPUT_ROOT]}"
SIM_TIMEOUT="${2:-600}"
OUTPUT_ROOT="${3:-$WORKSPACE_DIR/logs/stage_b_matrix}"
RUN_MODE="${4:-coverage}"
ROOM_COMBINED_COVERAGE_TARGET="${STAGE_B_ROOM_COMBINED_COVERAGE_TARGET:-0.84}"
MOTION_SPEED="${STAGE_B_MOTION_SPEED:-0.60}"
TRANSITION_ONLY="${STAGE_B_TRANSITION_ONLY:-0}"
GATE_OVERRIDE="${STAGE_B_GATE_OVERRIDE:-}"
START_RVIZ="${STAGE_B_START_RVIZ:-1}"
RVIZ_CONFIG="${STAGE_B_RVIZ_CONFIG:-$WORKSPACE_DIR/src/simnav/rviz/stage_b.rviz}"
GAZEBO_GUI="${STAGE_B_GUI:-true}"
RUN_DIR="$OUTPUT_ROOT/seed_$SEED_VALUE"
RUNTIME_ROOT="${STAGE_B_RUNTIME_ROOT:-$WORKSPACE_DIR}"
SCENE_DIR="$RUNTIME_ROOT/generated_building"
RESULTS_DIR="$RUNTIME_ROOT/results"
RUNTIME_LOG_DIR="$RUNTIME_ROOT/logs"
CANONICAL_DEVEL_DIR="$WORKSPACE_DIR/.simenv_build/devel"
DEVEL_DIR="${SIMENV_DEVEL_DIR:-$CANONICAL_DEVEL_DIR}"
if [ "$(readlink -f "$DEVEL_DIR")" != "$(readlink -f "$CANONICAL_DEVEL_DIR")" ]; then
  echo "Refusing non-canonical build: SIMENV_DEVEL_DIR must be $CANONICAL_DEVEL_DIR" >&2
  exit 3
fi

strip_legacy_workspace_env() {
  local variable value part cleaned
  for variable in CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH; do
    value="${!variable:-}"
    cleaned=""
    IFS=':' read -r -a parts <<< "$value"
    for part in "${parts[@]}"; do
      case "$part" in
        "$WORKSPACE_DIR/devel"|"$WORKSPACE_DIR/devel/"*) continue ;;
      esac
      cleaned="${cleaned:+$cleaned:}$part"
    done
    printf -v "$variable" '%s' "$cleaned"
    export "$variable"
  done
}
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
SUPPORT_PID=""
SUPERVISOR_PID=""
MONITOR_PID=""
TELEMETRY_PID=""
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
    'lio_localization_bridge_node.py' 'lio_occupancy_node.py' \
    'lio_health_monitor_node.py' 'map_views_node.py' \
    'coverage_explorer_node.py' 'danger_detector_node.py' \
    'elevator_transition_node.py' 'two_floor_explorer_supervisor.py' \
    'stage_b_localization.launch' 'stage_b_behavior.launch' 'rviz -d' \
    'monitor_stage_b_coverage.py' 'record_stage_b_telemetry.py'; do
    found_process=0
    while read -r process_pid process_args; do
      [ -n "$process_pid" ] || continue
      [ "$process_pid" = "$$" ] && continue
      case "$process_args" in
        *run_stage_b_seed.sh*|*"bash -lc"*|*"pgrep -af"*|rg\ *|*/rg\ *)
          continue
          ;;
      esac
      found_process=1
      break
    done < <(ps -eo pid=,args= | rg "$process_pattern" || true)
    if [ "$found_process" -eq 1 ]; then
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

cleanup_orphaned_stage_b_nodes() {
  # A forced stop can orphan roslaunch children after the parent shell exits.
  # Match their ROS_MASTER_URI before terminating them so parallel, unrelated
  # ROS instances are never disturbed.
  local pid env_master process_args
  while read -r pid process_args; do
    [ -n "$pid" ] || continue
    [ "$pid" = "$$" ] && continue
    case "$process_args" in
      *stage_b_localization.launch*|*lio_localization_bridge_node.py*|*lio_occupancy_node.py*|*lio_health_monitor_node.py*|*map_views_node.py*|*fastlio_mapping*|*coverage_explorer_node.py*|*danger_detector_node.py*|*elevator_transition_node.py*|*two_floor_explorer_supervisor.py*)
        env_master=""
        if [ -r "/proc/$pid/environ" ]; then
          env_master=$(tr '\0' '\n' < "/proc/$pid/environ" | sed -n 's/^ROS_MASTER_URI=//p' | head -n 1)
        fi
        if [ "$env_master" = "http://127.0.0.1:$ROS_PORT" ] || [ "$env_master" = "http://127.0.0.1:$ROS_PORT/" ]; then
          kill -TERM "$pid" 2>/dev/null || true
        fi
        ;;
    esac
  done < <(ps -eo pid=,args=)
  sleep 1
  while read -r pid process_args; do
    [ -n "$pid" ] || continue
    case "$process_args" in
      *stage_b_localization.launch*|*lio_localization_bridge_node.py*|*lio_occupancy_node.py*|*lio_health_monitor_node.py*|*map_views_node.py*|*fastlio_mapping*|*coverage_explorer_node.py*|*danger_detector_node.py*|*elevator_transition_node.py*|*two_floor_explorer_supervisor.py*)
        env_master=""
        if [ -r "/proc/$pid/environ" ]; then
          env_master=$(tr '\0' '\n' < "/proc/$pid/environ" | sed -n 's/^ROS_MASTER_URI=//p' | head -n 1)
        fi
        if [ "$env_master" = "http://127.0.0.1:$ROS_PORT" ] || [ "$env_master" = "http://127.0.0.1:$ROS_PORT/" ]; then
          kill -KILL "$pid" 2>/dev/null || true
        fi
        ;;
    esac
  done < <(ps -eo pid=,args=)
}

cleanup() {
  set +e
  terminate_process_group "$MONITOR_PID"
  terminate_process_group "$TELEMETRY_PID"
  terminate_process_group "$RVIZ_PID"
  terminate_process_group "$BEHAVIOR_PID"
  terminate_process_group "$SUPERVISOR_PID"
  terminate_process_group "$SUPPORT_PID"
  terminate_process_group "$NAV_PID"
  terminate_process_group "$AUTO_PID"
  terminate_process_group "$CORE_PID"
  cleanup_orphaned_stage_b_nodes
}
trap cleanup EXIT INT TERM

mkdir -p "$RUN_DIR"
strip_legacy_workspace_env
source /opt/ros/noetic/setup.bash
source "$DEVEL_DIR/setup.bash"
export ROS_PACKAGE_PATH="$WORKSPACE_DIR/src:${ROS_PACKAGE_PATH:-}"
export PYTHONPATH="$WORKSPACE_DIR/src/simnav/scripts:${PYTHONPATH:-}"
if [ "$(rospack find simnav 2>/dev/null)" != "$WORKSPACE_DIR/src/simnav" ]; then
  echo "simnav resolves outside this workspace; refusing stale ROS environment" >&2
  exit 3
fi
if ! rg -q "^CATKIN_DEVEL_PREFIX:PATH=${CANONICAL_DEVEL_DIR}$" \
  "$WORKSPACE_DIR/.simenv_build/build/CMakeCache.txt"; then
  echo "canonical build cache does not target .simenv_build/devel; rebuild required" >&2
  exit 3
fi
export ROS_MASTER_URI="http://127.0.0.1:$ROS_PORT"
export GAZEBO_MASTER_URI="http://127.0.0.1:$GAZEBO_PORT"
for required_node in lio_localization_bridge_node.py lio_occupancy_node.py map_views_node.py; do
  node_path="$WORKSPACE_DIR/src/simnav/scripts/$required_node"
  if [ ! -x "$node_path" ]; then
    echo "required ROS node is not executable: $node_path" >&2
    echo "restore it with: chmod +x $node_path" >&2
    exit 3
  fi
done
echo "Stage B resource guard: ROS_PORT=$ROS_PORT GAZEBO_PORT=$GAZEBO_PORT CPU_LIST=${CPU_LIST:-inherited}" > "$RUN_DIR/resource_guard.log"
echo "Stage B canonical devel: $CANONICAL_DEVEL_DIR" >> "$RUN_DIR/resource_guard.log"
echo "Stage B simnav package: $(rospack find simnav)" >> "$RUN_DIR/resource_guard.log"
if [ -f config/stage_b_entry_frozen.sha256 ]; then
  echo "Stage B frozen manifest: $(sha256sum config/stage_b_entry_frozen.sha256 | cut -d' ' -f1)" >> "$RUN_DIR/resource_guard.log"
fi
echo "Stage B locomotion policy: $STAGE_B_POLICY_PATH" >> "$RUN_DIR/resource_guard.log"
echo "Stage B plane policy: $STAGE_B_PLANE_POLICY_PATH" >> "$RUN_DIR/resource_guard.log"
echo "Stage B room combined coverage target: $ROOM_COMBINED_COVERAGE_TARGET" >> "$RUN_DIR/resource_guard.log"
echo "Stage B exploration motion speed: $MOTION_SPEED" >> "$RUN_DIR/resource_guard.log"

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
    # junior_ctrl may pause Gazebo once during its own initialization, after
    # auto.sh's scheduled unpause has already run.  Keep startup moving until
    # the simulated clock and sensor/controller topics are alive.
    if [ "$topic" = "/clock" ] \
      || [ "$topic" = "/trunk_imu" ] \
      || [ "$topic" = "/a1_gazebo/FR_hip_controller/state" ]; then
      if rosservice list 2>/dev/null | rg -qx '/gazebo/unpause_physics'; then
        timeout 2s rosservice call /gazebo/unpause_physics >/dev/null 2>&1 || true
      fi
    fi
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
  ROBOT_X="${STAGE_B_ROBOT_X:-0.0}" ROBOT_Y="${STAGE_B_ROBOT_Y:--3.2}" \
  ROBOT_Z="${STAGE_B_ROBOT_Z:-0.6}" ROBOT_YAW="${STAGE_B_ROBOT_YAW:-1.5708}" \
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

if [ "$RUN_MODE" != "coverage" ] && [ "$RUN_MODE" != "full" ] && [ "$RUN_MODE" != "two_floor" ] && [ "$RUN_MODE" != "three_floor" ]; then
  echo "unsupported active run mode: $RUN_MODE (door modes are archived)" >&2
  exit 2
fi

if [ "$RUN_MODE" = "two_floor" ] || [ "$RUN_MODE" = "three_floor" ]; then
  FROZEN_CHECKSUM_FILE="config/stage_b_floor0_frozen.sha256"
  if [ -f config/stage_b_entry_frozen.sha256 ]; then
    FROZEN_CHECKSUM_FILE="config/stage_b_entry_frozen.sha256"
  fi
  if ! sha256sum -c "$FROZEN_CHECKSUM_FILE" >/dev/null; then
    # The same runtime is used for single- and multi-floor runs.  A source
    # change must not silently turn the floor count into a separate test
    # species; retain the manifest as an audit signal and continue.
    echo "warning: frozen floor-0 manifest differs from the current explorer; continuing with the selected runtime floor count" >&2
    echo "Stage B frozen manifest warning: current explorer differs from $FROZEN_CHECKSUM_FILE" >> "$RUN_DIR/resource_guard.log"
  fi
fi

if [ "$RUN_MODE" = "two_floor" ] || [ "$RUN_MODE" = "three_floor" ] || [ "$TRANSITION_ONLY" = "1" ]; then
  "${RUN_PREFIX[@]}" setsid roslaunch simnav stage_b_two_floor_support.launch \
    result_dir:="$RESULTS_DIR" \
    max_floor:="$([ "$RUN_MODE" = "three_floor" ] && echo 2 || echo 1)" \
    start_immediately:="$([ "$TRANSITION_ONLY" = "1" ] && echo true || echo false)" \
    finish_at_top_floor:=false \
    gate_override:="${GATE_OVERRIDE:-[]}" \
    > "$RUN_DIR/two_floor_support.log" 2>&1 &
  SUPPORT_PID=$!
  wait_for_topic /simnav/elevator_status 60
  if [ "$TRANSITION_ONLY" != "1" ]; then
    "${RUN_PREFIX[@]}" setsid python3 team_scripts/two_floor_explorer_supervisor.py \
      > "$RUN_DIR/two_floor_supervisor.log" 2>&1 &
    SUPERVISOR_PID=$!
  fi
else
  "${RUN_PREFIX[@]}" setsid roslaunch simnav stage_b_behavior.launch \
    result_dir:="$RESULTS_DIR" \
    room_combined_coverage_target:="$ROOM_COMBINED_COVERAGE_TARGET" \
    motion_speed:="$MOTION_SPEED" \
    enable_elevator_transition:="$([ "$RUN_MODE" = "full" ] || [ "$TRANSITION_ONLY" = "1" ] && echo true || echo false)" \
    elevator_transition_only:="$([ "$TRANSITION_ONLY" = "1" ] && echo true || echo false)" \
    enable_coverage_explorer:="$([ "$TRANSITION_ONLY" = "1" ] && echo false || echo true)" \
    gate_override:="${GATE_OVERRIDE:-[]}" \
    > "$RUN_DIR/behavior.log" 2>&1 &
  BEHAVIOR_PID=$!
fi
if [ "$TRANSITION_ONLY" = "1" ]; then
  wait_for_topic /simnav/elevator_status 60
else
  wait_for_topic /simnav/explorer_status 60
fi

MONITOR_ARGS=(--sim-timeout "$SIM_TIMEOUT" --seed "$SEED_VALUE")
if [ "$RUN_MODE" = "full" ]; then
  MONITOR_ARGS+=(--wait-floor-transition)
fi
if [ "$RUN_MODE" = "two_floor" ] || [ "$RUN_MODE" = "three_floor" ]; then
  MONITOR_ARGS+=(--two-floor)
fi
if [ "$RUN_MODE" = "three_floor" ]; then
  MONITOR_ARGS+=(--three-floor)
fi
if [ "$TRANSITION_ONLY" = "1" ]; then
  MONITOR_ARGS+=(--wait-floor-transition --transition-only)
fi
"${RUN_PREFIX[@]}" python3 team_scripts/monitor_stage_b_coverage.py \
  "${MONITOR_ARGS[@]}" \
  > "$RUN_DIR/result.json" 2> "$RUN_DIR/monitor.stderr" &
MONITOR_PID=$!

# High-rate base telemetry.  The coverage monitor only reports topology at
# 1 Hz and the explorer stops at floor completion, so a fall during the
# elevator transition otherwise leaves no diagnostic trace at all.
"${RUN_PREFIX[@]}" python3 team_scripts/record_stage_b_telemetry.py \
  --out-dir "$RUN_DIR" \
  > "$RUN_DIR/telemetry.log" 2>&1 &
TELEMETRY_PID=$!

set +e
wait "$MONITOR_PID"
MONITOR_EXIT=$?
MONITOR_PID=""
set -e
save_visual_artifacts
copy_artifacts

# The scored artifact must be evaluated for every mode.  Single-floor runs keep
# the historical floor-0 rule; multi-floor runs must match all building truth.
DANGER_FLOOR_INDEX=0
if [ "$RUN_MODE" = "two_floor" ] || [ "$RUN_MODE" = "three_floor" ]; then
  DANGER_FLOOR_INDEX=-1
fi
set +e
python3 team_scripts/evaluate_stage_b_danger.py \
  --truth "$RUN_DIR/danger_truth.json" \
  --detected "$RUN_DIR/detected_danger.json" \
  --floor-index "$DANGER_FLOOR_INDEX" \
  --output "$RUN_DIR/danger_evaluation.json" \
  --summary "$RUN_DIR/result.json" \
  > "$RUN_DIR/danger_evaluation.log" 2>&1
DANGER_EXIT=$?
set -e
if [ "$DANGER_EXIT" -ne 0 ]; then
  MONITOR_EXIT=1
fi

exit "$MONITOR_EXIT"
