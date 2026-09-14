#!/usr/bin/env bash
# TEST-ONLY guarded launcher.  Delete this file for the final version.
#
#   team_scripts/testonly_launch_guarded.sh SEED [SIM_TIMEOUT] [OUTPUT_ROOT] \
#       [RUN_MODE] [MAX_ATTEMPTS]
#
# Nothing in the mainline calls it: `run_stage_b_seed.sh` is untouched, so the
# final version simply stops using this wrapper (and the two `testonly_*` files
# can be removed).  It only adds waiting and checking around the existing
# launcher:
#
#   1. clean up, then WAIT until the process table is really empty,
#   2. settle, clear stale Gazebo shared state, re-verify the ports are free,
#   3. launch the run,
#   4. judge the opening with testonly_startup_health.py and retry a bad start.
#
# Why: run94 was launched 3 s after `kill -9` of the previous run.  kill -9 is
# asynchronous, Gazebo came up during the previous teardown, the robot was
# ejected (world z 0.32 -> 7.43 -> -11.1 m) and "fell" at sim 32; the whole run
# was wasted because nothing checked the opening.
set -uo pipefail

SEED="${1:?usage: testonly_launch_guarded.sh SEED [SIM_TIMEOUT] [OUTPUT_ROOT] [RUN_MODE] [ATTEMPTS]}"
SIM_TIMEOUT="${2:-2400}"
OUTPUT_ROOT="${3:-logs/stage_b_matrix/seed_${SEED}}"
RUN_MODE="${4:-three_floor}"
ATTEMPTS="${5:-3}"
SETTLE_SECONDS="${SETTLE_SECONDS:-15}"
HEALTH_WINDOW="${HEALTH_WINDOW:-15}"

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAUNCH_LOG="${WORKSPACE_DIR}/${OUTPUT_ROOT%/seed_*}.launch.log"

source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/.simenv_build/devel/setup.bash"
export ROS_MASTER_URI="http://127.0.0.1:${SIMNAV_STAGE_B_PORT:-11320}"
export ROS_HOSTNAME=127.0.0.1
export ROS_IP=127.0.0.1

say() { echo "[guard] $*"; }

wait_for_quiet() {
  local deadline=$((SECONDS + 120))
  while [ "$SECONDS" -lt "$deadline" ]; do
    # pgrep -c already prints 0 when nothing matches (and exits 1), so a
    # `|| echo 0` fallback produced "0\n0" and the comparison never matched --
    # every attempt then burned the full 120 s wait for nothing.
    local alive
    alive=$(pgrep -fc 'gzserver|gzclient|fastlio_mapping|junior_ctrl|rosmaster|roslaunch|coverage_explorer_node|elevator_transition_node|run_stage_b_seed' 2>/dev/null)
    alive=${alive:-0}
    if [ "$alive" = "0" ]; then
      return 0
    fi
    sleep 1
  done
  return 1
}

for attempt in $(seq 1 "$ATTEMPTS"); do
  say "attempt ${attempt}/${ATTEMPTS}"
  bash "$WORKSPACE_DIR/team_scripts/kill_sim_processes.sh" || true

  if ! wait_for_quiet; then
    say "WARNING: processes still alive after 120 s; continuing anyway"
  fi

  # kill -9 returns before Gazebo has released its shared memory, ports and
  # /tmp state.  Starting a new world inside that window is what ejected the
  # robot in run94, so the settle is not cosmetic.
  say "settling ${SETTLE_SECONDS}s so Gazebo can release shared state"
  sleep "$SETTLE_SECONDS"
  rm -rf /tmp/.gazebo 2>/dev/null || true
  for port in "${SIMNAV_STAGE_B_PORT:-11320}" "${SIMNAV_STAGE_B_GAZEBO_PORT:-11345}"; do
    if command -v ss >/dev/null 2>&1 && ss -ltn 2>/dev/null | grep -q ":${port}\b"; then
      say "WARNING: port ${port} still listening before launch"
    fi
  done

  rm -rf "${WORKSPACE_DIR:?}/${OUTPUT_ROOT}"
  cd "$WORKSPACE_DIR" || exit 1
  setsid nohup ./team_scripts/run_stage_b_seed.sh \
    "$SEED" "$SIM_TIMEOUT" "$OUTPUT_ROOT" "$RUN_MODE" \
    > "$LAUNCH_LOG" 2>&1 < /dev/null &

  say "waiting for the simulation to come up"
  up=0
  for _ in $(seq 1 120); do
    if timeout 3 rostopic echo -n 1 /clock >/dev/null 2>&1 \
       && timeout 3 rostopic echo -n 1 /simnav/world_pose_metric >/dev/null 2>&1; then
      up=1
      break
    fi
    sleep 2
  done
  if [ "$up" != "1" ]; then
    say "simulation did not advertise /clock + world pose; retrying"
    continue
  fi

  say "opening health gate (${HEALTH_WINDOW} simulated seconds)"
  if python3 "$WORKSPACE_DIR/team_scripts/testonly_startup_health.py" \
      --window "$HEALTH_WINDOW"; then
    # The status sampler is part of the guarded flow on purpose: it was
    # forgotten by hand twice (run93/run96), losing the timing record.
    say "starting the status sampler"
    setsid nohup env ROS_MASTER_URI="$ROS_MASTER_URI" ROS_HOSTNAME=127.0.0.1 ROS_IP=127.0.0.1 \
      bash -c 'for _ in $(seq 1 400); do
        timeout 40 python3 /workspace/SimEnv/team_scripts/sample_run_status.py --run-dir "$0" >/dev/null 2>&1
        sleep 120
      done' "$OUTPUT_ROOT" \
      > "${WORKSPACE_DIR}/${OUTPUT_ROOT%/seed_*}.sampler.log" 2>&1 < /dev/null &

    say "healthy start on attempt ${attempt}; the run is live"
    exit 0
  fi

  say "BAD START on attempt ${attempt}; cleaning up and retrying"
  bash "$WORKSPACE_DIR/team_scripts/kill_sim_processes.sh" || true
done

say "gave up after ${ATTEMPTS} attempts"
exit 1
