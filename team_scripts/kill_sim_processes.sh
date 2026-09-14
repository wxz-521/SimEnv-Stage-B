#!/usr/bin/env bash
# Kill leftover Stage-B / Gazebo / ROS processes inside simenv-noetic.
#
# Run by path so the pattern list never appears on a command line:
#
#   docker exec simenv-noetic bash /workspace/SimEnv/team_scripts/kill_sim_processes.sh
#
# Why this file exists: the container shares the host PID namespace, so
# `pkill -f <pattern>` from inside also matches the caller's own command line
# whenever that command line happens to contain the pattern text -- which killed
# the invoking shell twice (exit 137).  Patterns live in this file instead, and
# the script protects itself and its parents.
set -uo pipefail

patterns=(
  "gzserver" "gzclient" "fastlio_mapping" "junior_ctrl"
  "lio_localization_bridge_node.py" "lio_occupancy_node.py"
  "lio_health_monitor_node.py" "map_views_node.py"
  "coverage_explorer_node.py" "danger_detector_node.py"
  "elevator_transition_node.py" "two_floor_explorer_supervisor.py"
  "two_floor_explorer_supervisor" "monitor_stage_b_coverage.py"
  "record_stage_b_telemetry.py" "sample_run_status.py"
  "verify_three_floor_run.py" "run_stage_b_seed.sh"
  "stage_b_localization.launch" "stage_b_behavior.launch"
  # auto.sh backgrounds the door/elevator control service and only records its
  # PID in logs/building_control.pid, which nothing reaps.  Without this pattern
  # every run leaks one sleeping copy: 86 of them had accumulated in the
  # container, all of which reconnect and republish the building config to the
  # next rosmaster they see.
  "building_generator_classic_control"
  "activate_stage_b_controller.py" "record_elevator_phase.py"
  "elevator_only_driver.py" "elevator_only_matrix.sh" "elevator_only_test.sh"
  "stage_b_two_floor_support.launch" "pointcloud2livox"
  "pointcloud_to_laserscan" "robot_state_publisher" "rosmaster" "roslaunch"
  "rosout" "elevator_cycle_check.sh" "rviz" "unitree_gazebo"
  "rosgraph_msgs.msg import Clock"
)

protect=" $$ ${PPID:-0} "

for pattern in "${patterns[@]}"; do
  for pid in $(pgrep -f -- "${pattern}" 2>/dev/null); do
    case "${protect}" in
      *" ${pid} "*) continue ;;
    esac
    kill -9 "${pid}" 2>/dev/null
  done
done

sleep 3
rm -f /workspace/SimEnv/.stage_b_runner.lock
remaining=$(ps -eo pid=,args= 2>/dev/null | grep -cE "gzserver|fastlio_mapping|junior_ctrl|run_stage_b_seed|coverage_explorer_node|elevator_transition_node|monitor_stage_b_coverage" || true)
echo "kill_sim_processes: remaining=$((remaining > 0 ? remaining - 1 : 0))"
exit 0
