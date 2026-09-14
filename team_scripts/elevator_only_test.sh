#!/usr/bin/env bash
# Standalone "elevator only" test -- exploration is NOT started at all.
#
#   team_scripts/elevator_only_test.sh [SEED] [RUN_TAG]
#
# This is a TEST TOOL and it touches no mainline file.  It boots only:
#
#       roscore + auto.sh (Gazebo, robot, junior_ctrl) + stage_b_localization
#
# It deliberately does NOT launch stage_b_behavior.launch or
# stage_b_two_floor_support.launch, so neither the coverage explorer nor the
# elevator_transition node exists during the run; the only thing driving the
# robot is team_scripts/elevator_only_driver.py.
#
# The driver is told where the lift is through parameters (the scene truth by
# default): doorway (1.65, 2.60), facing yaw 0.0 (into the car), width 1.40 m.
# Override with ELEVATOR_DOOR="x,y,yaw,width".
#
# Default spawn (the mission the user asked for): OUTSIDE the main gate, 3.2 m
# in front of it at (0.0, -3.2), facing the building (+y, yaw +1.5708), so the
# driver starts in ENTER_BUILDING and walks in through the main entrance.
# It then drives up the corridor to the mouth, U-turns there (that is when the
# lobby + lift are on the localisation map), comes back to the lift doorway,
# boards, and rides 1F->2F->3F->1F with a corridor-mouth U-turn after each of
# the first two exits.  The last leg leaves through the main entrance and stops
# back at (0.0, -3.2) facing the building.
# Override the pose with STAGE_B_ROBOT_X / STAGE_B_ROBOT_Y / STAGE_B_ROBOT_YAW.
# SKIP_ENTRANCE=1 keeps the old in-lobby debugging spawn (0, 2.6) and disables
# the entry corridor leg; it is not the mission.
set -uo pipefail

SEED="${1:-20260902}"
TAG="${2:-elevtest_$(date +%H%M)}"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEVEL_DIR="$WORKSPACE_DIR/.simenv_build/devel"
SCENE_DIR="$WORKSPACE_DIR/generated_building"
RESULTS_DIR="$WORKSPACE_DIR/results"
RUNTIME_LOG_DIR="$WORKSPACE_DIR/logs"
RUN_DIR="$WORKSPACE_DIR/logs/$TAG/seed_$SEED"
ROS_PORT="${SIMNAV_STAGE_B_PORT:-11320}"
GAZEBO_PORT="${SIMNAV_STAGE_B_GAZEBO_PORT:-11345}"

ELEVATOR_DOOR="${ELEVATOR_DOOR:-1.65,2.60,0.0,1.40}"
MAIN_ENTRANCE="${MAIN_ENTRANCE:-0.0,0.0,1.5708,2.0}"
SKIP_ENTRANCE="${SKIP_ENTRANCE:-0}"
# Gait policy for the doorway crossing: stair (junior_ctrl default, trained for
# steps), plane (the flat-ground policy the mainline elevator node selected) or
# keep (change nothing).
GAIT_POLICY="${GAIT_POLICY:-stair}"
# Speed of the committed run through the doorway (the mainline launch uses 0.28).
CROSSING_SPEED="${CROSSING_SPEED:-0.45}"
# How long the committed run may take before it is called a failure.  Set
# this high to separate "cannot climb the 6 cm threshold" from "climbs it
# too slowly for the budget".
# Entry budget.  It used to be 35 s to separate "cannot climb the threshold"
# from "climbs it too slowly"; the entry now also keeps pushing through contact
# with a retreat/retry watchdog, so the budget has to cover a few retries.
ENTRY_TIMEOUT="${ENTRY_TIMEOUT:-75}"
# Ride schedule in ARRIVAL-floor order (0 = ground floor / "1F").  The default
# is the user's full cycle: 1F -> 2F, 2F -> 3F, 3F -> 1F.
TARGET_FLOORS="${TARGET_FLOORS:-1,2,0}"
# Per-ride corridor excursion, in metres past the corridor gate (-0.25, 7.30):
# after the 1F->2F ride and after the 2F->3F ride the robot drives to the
# corridor mouth, U-turns and comes back to the doorway.  0 = no excursion.
# 1.6 m puts the stop at world y ~8.5-8.9, i.e. ~0.7-1.0 m inside the corridor
# (its floor starts at y = 7.85) and clear of the mouth wall for the U-turn.
CORRIDOR_DEPTHS="${CORRIDOR_DEPTHS:-1.6,1.6,0.0}"
# Same for the very first leg, after walking in through the main entrance:
# drive up the corridor to the mouth and U-turn before approaching the lift.
# 0 disables the leg.
ENTRY_CORRIDOR_DEPTH="${ENTRY_CORRIDOR_DEPTH:-}"
# World-frame corridor gate: [x, y, yaw].  The corridor runs along +yaw from it.
CORRIDOR_GATE="${CORRIDOR_GATE:--0.25,7.30,1.5708}"
# Driving speed for the corridor legs (flat floor).
CORRIDOR_SPEED="${CORRIDOR_SPEED:-0.60}"
# How close to the corridor axis the robot must be before it may U-turn.  The
# corridor is 2.2 m wide and the body needs ~0.4 m of swing radius, so this is
# a safety gate, not a precision score (probe 00 jammed at 0.75 m off axis).
CORRIDOR_AXIS_TOLERANCE="${CORRIDOR_AXIS_TOLERANCE:-0.30}"
# Leaving the car climbs the same 6 cm threshold and used to jam on the lip
# (probe 01: 1.11 m out, centre x = 1.514, stalled 28 s).  The driver watches
# ground-truth progress, retreats briefly and retries with an alternating
# +/- biased heading, like the mainline elevator node.  The tested crossing
# speed itself is NOT changed.
EXIT_RETRY_LIMIT="${EXIT_RETRY_LIMIT:-3}"
EXIT_RETRY_YAW_BIAS="${EXIT_RETRY_YAW_BIAS:-0.16}"
# Every boarding must cross the doorway centred on its centre line (user
# requirement "每次进电梯前，确保自己能够对准中心进门").
BOARD_LATERAL_TOLERANCE="${BOARD_LATERAL_TOLERANCE:-0.20}"
# Driver wall-clock backstop in seconds.  The harness runs well below real time
# (a lift cycle is several minutes of wall time), so this has to be generous.
WALL_TIMEOUT="${WALL_TIMEOUT:-3600}"

if [ "$SKIP_ENTRANCE" = "1" ]; then
  # Debugging shortcut: start outdoors too, but just outside the door in the
  # lobby, facing the lift, with no entry corridor leg.
  ROBOT_X="${STAGE_B_ROBOT_X:--2.0}"
  ROBOT_Y="${STAGE_B_ROBOT_Y:-2.60}"
  ROBOT_YAW="${STAGE_B_ROBOT_YAW:-0.0}"
  ENTER_BUILDING=false
  ENTRY_CORRIDOR_DEPTH="${ENTRY_CORRIDOR_DEPTH:-0.0}"
else
  # Mission spawn: outdoors in front of the main gate, facing the building.
  # The production default (0.0, -3.2, +pi/2) walks in through the entrance.
  ROBOT_X="${STAGE_B_ROBOT_X:-0.0}"
  ROBOT_Y="${STAGE_B_ROBOT_Y:--3.2}"
  ROBOT_YAW="${STAGE_B_ROBOT_YAW:-1.5708}"
  ENTER_BUILDING=true
  ENTRY_CORRIDOR_DEPTH="${ENTRY_CORRIDOR_DEPTH:-1.6}"
fi

say() { echo "[elevator-only] $*"; }

source /opt/ros/noetic/setup.bash
if [ ! -f "$DEVEL_DIR/setup.bash" ]; then
  echo "Missing $DEVEL_DIR/setup.bash; build the workspace first." >&2
  exit 3
fi
source "$DEVEL_DIR/setup.bash"
export ROS_MASTER_URI="http://127.0.0.1:$ROS_PORT"
export ROS_HOSTNAME=127.0.0.1
export ROS_IP=127.0.0.1

mkdir -p "$RUN_DIR"

say "stopping anything left over from an earlier run"
bash "$WORKSPACE_DIR/team_scripts/kill_sim_processes.sh" || true

say "starting roscore on port $ROS_PORT"
setsid roscore -p "$ROS_PORT" > "$RUN_DIR/roscore.log" 2>&1 &
CORE_PID=$!
for _ in $(seq 1 200); do
  rosparam list >/dev/null 2>&1 && break
  kill -0 "$CORE_PID" 2>/dev/null || { echo "roscore died" >&2; exit 1; }
  sleep 0.1
done

wait_for_topic() {
  local topic="$1" timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))
  while [ "$SECONDS" -lt "$deadline" ]; do
    # junior_ctrl may pause Gazebo once during its own initialisation.
    if rosservice list 2>/dev/null | rg -qx '/gazebo/unpause_physics'; then
      timeout 2s rosservice call /gazebo/unpause_physics >/dev/null 2>&1 || true
    fi
    timeout 2s rostopic echo -n 1 "$topic" >/dev/null 2>&1 && return 0
    sleep 0.2
  done
  echo "Timed out waiting for $topic" >&2
  return 1
}

say "starting Gazebo + robot at ($ROBOT_X, $ROBOT_Y) yaw $ROBOT_YAW"
cd "$WORKSPACE_DIR" || exit 1
GUI=false PAUSED=true AUTO_UNPAUSE=1 AUTO_UNPAUSE_DELAY=6 \
  ROBOT_X="$ROBOT_X" ROBOT_Y="$ROBOT_Y" ROBOT_Z="${STAGE_B_ROBOT_Z:-0.6}" \
  ROBOT_YAW="$ROBOT_YAW" \
  UNITREE_POLICY_PATH="${UNITREE_POLICY_PATH:-$WORKSPACE_DIR/src/unitree_guide/logs/policy_act_inference_stair.pt}" \
  UNITREE_PLANE_POLICY_PATH="${UNITREE_PLANE_POLICY_PATH:-$WORKSPACE_DIR/src/unitree_guide/logs/policy_act_inference_plane.pt}" \
  SIMENV_DEVEL_DIR="$DEVEL_DIR" \
  SIMENV_SCENE_OUTPUT_DIR="$SCENE_DIR" SIMENV_RESULTS_DIR="$RESULTS_DIR" \
  SIMENV_RUNTIME_LOG_DIR="$RUNTIME_LOG_DIR" \
  START_CONTROLLER=1 CONTROLLER_FOREGROUND=0 START_VIRTUAL_JOY=0 \
  ENABLE_GROUND_TRUTH=0 ENABLE_REFEREE_ODOM=0 POINTCLOUD_USE_GROUND_TRUTH_ODOM=0 \
  SEED="$SEED" \
  setsid ./auto.sh > "$RUN_DIR/auto.log" 2>&1 &
AUTO_PID=$!

wait_for_topic /clock 240 || exit 1
wait_for_topic /trunk_imu 240 || exit 1
wait_for_topic /a1_gazebo/FR_hip_controller/state 240 || exit 1

say "switching the controller to fixed stand, then RL /cmd_vel"
python3 team_scripts/activate_stage_b_controller.py --mode sequence \
  > "$RUN_DIR/controller_activation.log" 2>&1

say "starting localisation + mapping (no explorer, no elevator node)"
setsid roslaunch simnav stage_b_localization.launch \
  team_scene_info:="$SCENE_DIR/team_scene_info.json" \
  > "$RUN_DIR/navigation.log" 2>&1 &
NAV_PID=$!

wait_for_topic /simnav/odom 180 || exit 1
wait_for_topic /simnav/world_pose_metric 180 || exit 1
wait_for_topic /exploration_map 180 || exit 1

say "handing the lift coordinate to the robot: doorway = ($ELEVATOR_DOOR)"
say "gait policy for the crossing: $GAIT_POLICY"
say "ride schedule: $TARGET_FLOORS   corridor depths: $CORRIDOR_DEPTHS (entry $ENTRY_CORRIDOR_DEPTH)"
say "exit retries: $EXIT_RETRY_LIMIT x ${EXIT_RETRY_YAW_BIAS} rad bias; boarding centre tolerance: $BOARD_LATERAL_TOLERANCE m"
say "spawn: ($ROBOT_X, $ROBOT_Y) yaw $ROBOT_YAW  (enter_building=$ENTER_BUILDING)"
python3 "$WORKSPACE_DIR/team_scripts/elevator_only_driver.py" \
  --output-csv "$RUN_DIR/elevator_only_timeline.csv" \
  --output-json "$RUN_DIR/elevator_only_summary.json" \
  _elevator_door:="[$ELEVATOR_DOOR]" \
  _main_entrance:="[$MAIN_ENTRANCE]" \
  _enter_building:="$ENTER_BUILDING" \
  _gait_policy:="$GAIT_POLICY" \
  _crossing_speed:="$CROSSING_SPEED" \
  _entry_timeout:="$ENTRY_TIMEOUT" \
  _entry_corridor_depth:="$ENTRY_CORRIDOR_DEPTH" \
  _entry_retry_limit:="${ENTRY_RETRY_LIMIT:-3}" \
  _entry_retry_yaw_bias:="${ENTRY_RETRY_YAW_BIAS:-0.16}" \
  _wall_timeout:="$WALL_TIMEOUT" \
  _ride_floors:="[$TARGET_FLOORS]" \
  _corridor_depths:="[$CORRIDOR_DEPTHS]" \
  _corridor_gate:="[$CORRIDOR_GATE]" \
  _corridor_axis_tolerance:="$CORRIDOR_AXIS_TOLERANCE" \
  _corridor_speed:="$CORRIDOR_SPEED" \
  _exit_retry_limit:="$EXIT_RETRY_LIMIT" \
  _exit_retry_yaw_bias:="$EXIT_RETRY_YAW_BIAS" \
  _board_lateral_tolerance:="$BOARD_LATERAL_TOLERANCE" \
  _return_to_spawn:=true \
  _spawn_world:="[$ROBOT_X,$ROBOT_Y,$ROBOT_YAW]" \
  _current_floor:=0 \
  _seed:="$SEED" \
  _tag:="$TAG" \
  2>&1 | tee "$RUN_DIR/elevator_only_driver.log"
STATUS=${PIPESTATUS[0]}

save_artifacts() {
  for image_topic in /exploration_map; do
    timeout 20s rosrun map_server map_saver \
      -f "$RUN_DIR/exploration_map" map:=$image_topic \
      > "$RUN_DIR/map_saver.log" 2>&1 || true
  done
  cp -f "$SCENE_DIR/layout_metadata.json" "$RUN_DIR/" 2>/dev/null || true
  cp -f "$RUNTIME_LOG_DIR/junior_ctrl.log" "$RUN_DIR/" 2>/dev/null || true
}
save_artifacts

say "stopping the test stack"
bash "$WORKSPACE_DIR/team_scripts/kill_sim_processes.sh" || true

say "done: status=$STATUS  logs in $RUN_DIR"
exit "$STATUS"
