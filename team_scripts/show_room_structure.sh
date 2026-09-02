#!/usr/bin/env bash
set -euo pipefail

# This script is intended to run inside the simenv-noetic container.
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GENERATOR_SCRIPT="$WORKSPACE_DIR/src/building_obstacles/scripts/generate_competition_scene.py"
FILTER_SCRIPT="$WORKSPACE_DIR/team_scripts/make_structure_world.py"
UNITREE_GAZEBO_MODELS="$WORKSPACE_DIR/src/unitree_guide/unitree_ros/unitree_gazebo/models"

usage() {
  cat <<'EOF'
Usage: show_room_structure.sh --seed SEED
       show_room_structure.sh SEED

Show one deterministic SimEnv building with walls, floors, stairs, elevator,
all room furniture, and all red/green danger and distractor models, but no
robot or sensors. Floor 0 remains opaque; floors above it are 80% transparent
by default, while all obstacles remain opaque for visibility. Press Ctrl-C or
close Gazebo to stop this run.

Environment overrides:
  FLOOR_COUNT, ROOMS_PER_FLOOR, BUILDING_WIDTH, BUILDING_LENGTH
  DANGER_COUNT (default: 3:6), DISTRACTOR_COUNT (default: 4:8)
  GAZEBO_ROOM_STRUCTURE_PORT (default: 11347)
  ROOM_STRUCTURE_UPPER_TRANSPARENCY (default: 0.80)
  ROOM_STRUCTURE_CAMERA_POSE (six space-separated pose values)
  ROOM_STRUCTURE_OUTPUT_ROOT (default: /tmp/simenv_room_structure)
  ROOM_STRUCTURE_HEADLESS=1 (start only gzserver for a smoke test)
EOF
}

seed_value="${SEED:-}"
positional_seed=""
explicit_seed=0
while (($#)); do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --seed)
      if (($# < 2)); then
        echo "--seed requires an integer value" >&2
        exit 2
      fi
      if [ "$explicit_seed" -eq 1 ]; then
        echo "seed option was provided more than once" >&2
        exit 2
      fi
      seed_value="$2"
      explicit_seed=1
      shift 2
      ;;
    --seed=*)
      if [ "$explicit_seed" -eq 1 ]; then
        echo "seed option was provided more than once" >&2
        exit 2
      fi
      seed_value="${1#*=}"
      explicit_seed=1
      shift
      ;;
    --)
      shift
      while (($#)); do
        if [ -n "$positional_seed" ]; then
          echo "only one positional seed is allowed" >&2
          exit 2
        fi
        positional_seed="$1"
        shift
      done
      ;;
    -*)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [ -n "$positional_seed" ]; then
        echo "only one positional seed is allowed" >&2
        exit 2
      fi
      positional_seed="$1"
      shift
      ;;
  esac
done

if [ -n "$positional_seed" ]; then
  if [ "$explicit_seed" -eq 1 ]; then
    echo "seed was provided both positionally and with --seed" >&2
    exit 2
  fi
  if [ -n "${SEED:-}" ] && [ "$positional_seed" != "$SEED" ]; then
    echo "positional seed conflicts with SEED=$SEED" >&2
    exit 2
  fi
  seed_value="$positional_seed"
fi
if [ -z "$seed_value" ]; then
  echo "a seed is required" >&2
  usage >&2
  exit 2
fi
if ! [[ "$seed_value" =~ ^-?[0-9]+$ ]]; then
  echo "seed must be an integer: $seed_value" >&2
  exit 2
fi

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  echo "ROS Noetic is unavailable. Run this script inside simenv-noetic." >&2
  exit 3
fi
for command_name in python3 gzserver gzclient setsid flock; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "missing command: $command_name" >&2
    exit 3
  fi
done
source /opt/ros/noetic/setup.bash

GAZEBO_PORT="${GAZEBO_ROOM_STRUCTURE_PORT:-11347}"
if ! [[ "$GAZEBO_PORT" =~ ^[0-9]+$ ]] || [ "$GAZEBO_PORT" -lt 1024 ] || [ "$GAZEBO_PORT" -gt 65535 ]; then
  echo "GAZEBO_ROOM_STRUCTURE_PORT must be a TCP port in [1024, 65535]" >&2
  exit 2
fi
if command -v ss >/dev/null 2>&1 && ss -ltn 2>/dev/null | rg -q ":${GAZEBO_PORT}\\b"; then
  echo "Gazebo port $GAZEBO_PORT is already in use; refusing to share it." >&2
  exit 3
fi

LOCK_FILE="$WORKSPACE_DIR/.room_structure_visual.lock"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "another room-structure visualizer is already active" >&2
  exit 3
fi

FLOOR_COUNT="${FLOOR_COUNT:-3}"
ROOMS_PER_FLOOR="${ROOMS_PER_FLOOR:-4}"
BUILDING_WIDTH="${BUILDING_WIDTH:-20.0}"
BUILDING_LENGTH="${BUILDING_LENGTH:-36.0}"
UPPER_TRANSPARENCY="${ROOM_STRUCTURE_UPPER_TRANSPARENCY:-0.80}"
DANGER_COUNT="${DANGER_COUNT:-3:6}"
DISTRACTOR_COUNT="${DISTRACTOR_COUNT:-4:8}"
OUTPUT_ROOT="${ROOM_STRUCTURE_OUTPUT_ROOT:-/tmp/simenv_room_structure}"
RUN_DIR="$OUTPUT_ROOT/seed_${seed_value}_$(date +%Y%m%d_%H%M%S)_$$"
BUILDING_DIR="$RUN_DIR/generated_building"
RESULTS_DIR="$RUN_DIR/results"
WORLD_FILE="$RUN_DIR/room_structure.world"
mkdir -p "$BUILDING_DIR" "$RESULTS_DIR"

CAMERA_POSE="${ROOM_STRUCTURE_CAMERA_POSE:-24 30 30 0 0.65 -2.0}"
read -r -a CAMERA_VALUES <<< "$CAMERA_POSE"
if [ "${#CAMERA_VALUES[@]}" -ne 6 ]; then
  echo "ROOM_STRUCTURE_CAMERA_POSE must contain six numbers" >&2
  exit 2
fi

SERVER_PID=""
CLIENT_PID=""
terminate_group() {
  local pid="$1"
  [ -n "$pid" ] || return 0
  if ! kill -0 "$pid" 2>/dev/null; then
    wait "$pid" 2>/dev/null || true
    return 0
  fi
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 40); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  terminate_group "$CLIENT_PID"
  terminate_group "$SERVER_PID"
  echo "room-structure visualizer stopped; no global Gazebo/ROS cleanup was performed."
  echo "artifacts: $RUN_DIR"
  exit "$status"
}
trap cleanup EXIT INT TERM

export PYTHONPATH="$WORKSPACE_DIR/src/building_generator_classic:$WORKSPACE_DIR/src/building_generator_core:${PYTHONPATH:-}"
export GAZEBO_MASTER_URI="http://127.0.0.1:$GAZEBO_PORT"
export GAZEBO_MODEL_PATH="$BUILDING_DIR:$UNITREE_GAZEBO_MODELS:${GAZEBO_MODEL_PATH:-}"

echo "Generating structure for seed $seed_value..."
python3 "$GENERATOR_SCRIPT" \
  --output-dir "$BUILDING_DIR" \
  --results-dir "$RESULTS_DIR" \
  --team-info-dir "$BUILDING_DIR" \
  --seed "$seed_value" \
  --floor-count "$FLOOR_COUNT" \
  --rooms-per-floor "$ROOMS_PER_FLOOR" \
  --width "$BUILDING_WIDTH" \
  --length "$BUILDING_LENGTH" \
  --danger-count "$DANGER_COUNT" \
  --distractor-count "$DISTRACTOR_COUNT" \
  > "$RUN_DIR/generator_manifest.stdout.json"

python3 "$FILTER_SCRIPT" \
  --input "$BUILDING_DIR/competition_scene.world" \
  --output "$WORLD_FILE" \
  --upper-transparency "$UPPER_TRANSPARENCY" \
  --camera-pose "${CAMERA_VALUES[@]}" \
  > "$RUN_DIR/structure_filter.json"

echo "World: $WORLD_FILE"
echo "Gazebo master: $GAZEBO_MASTER_URI"
echo "Floor 0 (including furniture): opaque; floors 1+: transparency=$UPPER_TRANSPARENCY"
echo "Starting Gazebo structure view. Use the GUI orbit controls to inspect doors, corners, stairs, and occlusions."
setsid gzserver --verbose "$WORLD_FILE" > "$RUN_DIR/gzserver.log" 2>&1 &
SERVER_PID=$!

port_ready=0
for _ in $(seq 1 150); do
  if ! kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "gzserver exited during startup; see $RUN_DIR/gzserver.log" >&2
    exit 1
  fi
  if ! command -v ss >/dev/null 2>&1; then
    port_ready=1
    break
  fi
  if ss -ltn 2>/dev/null | rg -q ":${GAZEBO_PORT}\\b"; then
    port_ready=1
    break
  else
    sleep 0.2
  fi
done
if ! kill -0 "$SERVER_PID" 2>/dev/null; then
  echo "gzserver exited during startup; see $RUN_DIR/gzserver.log" >&2
  exit 1
fi
if [ "$port_ready" -ne 1 ]; then
  echo "timed out waiting for Gazebo master on $GAZEBO_MASTER_URI; see $RUN_DIR/gzserver.log" >&2
  exit 1
fi

if [ "${ROOM_STRUCTURE_HEADLESS:-0}" = "1" ]; then
  echo "ROOM_STRUCTURE_HEADLESS=1: Gazebo server is running for inspection; press Ctrl-C to stop."
  wait "$SERVER_PID"
else
  if [ -z "${DISPLAY:-}" ]; then
    echo "DISPLAY is unset; set DISPLAY (usually :0) or use ROOM_STRUCTURE_HEADLESS=1." >&2
    exit 3
  fi
  setsid gzclient --verbose > "$RUN_DIR/gzclient.log" 2>&1 &
  CLIENT_PID=$!
  wait "$CLIENT_PID"
fi
