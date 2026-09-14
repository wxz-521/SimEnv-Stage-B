#!/bin/bash
# Isolated RTF benchmark: same world + same robot xacro as Stage B, but on private
# ROS/Gazebo ports so a live run is never touched. Used to attribute the real-time
# factor cost to individual sensors before changing any runtime configuration.
#
#   bench_rtf_isolated.sh <tag> <livox:0|1> <realsense:0|1> <spawn_controllers:0|1>
#
# Outputs, per run, inside BENCH_DIR:
#   <tag>.stats.txt   raw `gz stats` sample
#   <tag>.result.txt  one-line summary: tag livox realsense controllers rtf sim real
#   <tag>.threads.txt per-thread CPU share of gzserver during the sample window
#   <tag>.gazebo.log  launch log (tail kept)
set -u

TAG="${1:?tag}"
LIVOX="${2:?livox 0|1}"
REALSENSE="${3:?realsense 0|1}"
SPAWN_CTRL="${4:-0}"

WORKSPACE_DIR=/workspace/SimEnv
CANONICAL_DEVEL_DIR="$WORKSPACE_DIR/.simenv_build/devel"
BENCH_DIR="${BENCH_DIR:-/tmp/simenv_rtf_bench}"
ROS_PORT="${BENCH_ROS_PORT:-11360}"
GAZEBO_PORT="${BENCH_GAZEBO_PORT:-11365}"
WARMUP="${BENCH_WARMUP:-40}"
SAMPLE="${BENCH_SAMPLE:-8}"
mkdir -p "$BENCH_DIR"

as_bool() { [ "$1" = "1" ] && echo true || echo false; }

export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export GAZEBO_MASTER_URI="http://127.0.0.1:${GAZEBO_PORT}"
export ROS_IP=127.0.0.1
export GAZEBO_IP=127.0.0.1
export BUILDING_WORLD_FILE="$WORKSPACE_DIR/generated_building/competition_scene.world"
export DISPLAY="${DISPLAY:-:0}"

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
source "$CANONICAL_DEVEL_DIR/setup.bash"
# BENCH_OVERLAY lets a variant a1_description (edited sensor block) win the
# $(find a1_description) lookup without touching the package the live run uses.
export ROS_PACKAGE_PATH="${BENCH_OVERLAY:+$BENCH_OVERLAY:}$WORKSPACE_DIR/src:$WORKSPACE_DIR/src/unitree_guide/unitree_ros/robots:${ROS_PACKAGE_PATH:-}"
cd "$WORKSPACE_DIR" || exit 3

setsid roscore -p "$ROS_PORT" >"$BENCH_DIR/$TAG.roscore.log" 2>&1 &
RC_PID=$!
sleep 3

setsid roslaunch unitree_guide multi_floor_gazeboSim.launch \
  gui:=false paused:=false headless:=true debug:=false \
  start_controller_spawner:="$(as_bool "$SPAWN_CTRL")" \
  user_debug:=False rname:=a1 \
  enable_sensor_data:="$(as_bool "$REALSENSE")" \
  enable_livox:="$(as_bool "$LIVOX")" \
  enable_livox_imu:="$(as_bool "$LIVOX")" \
  enable_realsense:="$(as_bool "$REALSENSE")" \
  enable_front_camera:=false \
  enable_ground_truth:=true \
  enable_referee_odom:=false \
  enable_foot_contact_sensor:=false \
  enable_joy_node:=false \
  enable_pointcloud_converter:="$(as_bool "$LIVOX")" \
  >"$BENCH_DIR/$TAG.gazebo.log" 2>&1 &
LP_PID=$!

cleanup() {
  kill -TERM -"$LP_PID" 2>/dev/null
  kill -TERM -"$RC_PID" 2>/dev/null
  sleep 4
  kill -KILL -"$LP_PID" 2>/dev/null
  kill -KILL -"$RC_PID" 2>/dev/null
}
trap cleanup EXIT

# Fixed wall-clock warmup, then a single sampling call. `gz stats` reports the
# instantaneous factor, so one call after the startup transient is enough and
# avoids fragile polling loops.
sleep "$WARMUP"
printf 'warmup_s=%s\n' "$WARMUP" >"$BENCH_DIR/$TAG.warmup.txt"

gz stats -d "$SAMPLE" >"$BENCH_DIR/$TAG.stats.txt" 2>&1

GZ_PID=$(pgrep -g "$LP_PID" -f "gzserver -u -e ode" 2>/dev/null | head -1)
{
  echo "tid cpu% name"
  if [ -n "${GZ_PID:-}" ]; then
    declare -A first
    for t in /proc/"$GZ_PID"/task/*; do
      tid=${t##*/}
      [ -r "$t/stat" ] || continue
      read -r _ _ _ _ _ _ _ _ _ _ _ _ _ u s _ <"$t/stat"
      first["$tid"]=$((u + s))
    done
    sleep 5
    for t in /proc/"$GZ_PID"/task/*; do
      tid=${t##*/}
      [ -r "$t/stat" ] || continue
      read -r _ _ _ _ _ _ _ _ _ _ _ _ _ u s _ <"$t/stat"
      d=$((u + s - ${first[$tid]:-0}))
      [ "$d" -gt 5 ] || continue
      printf '%s %d.%d %s\n' "$tid" $((d / 5)) $(((d % 5) * 2)) "$(cat "$t/comm" 2>/dev/null)"
    done | sort -k2 -rn
  else
    echo "gzserver pid not found in bench process group $LP_PID"
  fi
} >"$BENCH_DIR/$TAG.threads.txt" 2>&1

python3 - "$BENCH_DIR/$TAG.stats.txt" "$TAG" "$LIVOX" "$REALSENSE" "$SPAWN_CTRL" >"$BENCH_DIR/$TAG.result.txt" <<'PY'
import re, sys
path, tag, livox, rs, ctrl = sys.argv[1:6]
facs, sims, reals = [], [], []
with open(path, errors="replace") as fh:
    for line in fh:
        m = re.match(r"Factor\[([-\d.]+)\] SimTime\[([-\d.]+)\] RealTime\[([-\d.]+)\]", line.strip())
        if not m:
            continue
        facs.append(float(m.group(1)))
        sims.append(float(m.group(2)))
        reals.append(float(m.group(3)))
if facs:
    mid = sorted(facs)[len(facs) // 2]
    print("tag=%s livox=%s realsense=%s controllers=%s samples=%d rtf_median=%.3f rtf_max=%.3f sim=%.2f real=%.2f"
          % (tag, livox, rs, ctrl, len(facs), mid, max(facs), sims[-1], reals[-1]))
else:
    print("tag=%s livox=%s realsense=%s controllers=%s NO_DATA" % (tag, livox, rs, ctrl))
PY
cat "$BENCH_DIR/$TAG.result.txt"
