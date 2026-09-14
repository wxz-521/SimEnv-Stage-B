#!/usr/bin/env bash
# Run the standalone elevator-only mission until N CONSECUTIVE runs pass.
#
#   team_scripts/elevator_only_matrix.sh [SEED] [PREFIX]
#
# A run passes when team_scripts/elevator_only_driver.py reaches its own DONE
# state with no fault AND every verdict in its report is PASS (all rides on the
# right floor by ground truth, every boarding at the doorway, every corridor
# excursion out to the mouth and back, and the final pose back at the spawn).
# That is the driver's exit status, so the harness exit status is the verdict.
#
# Runs are sequential: only one Gazebo/ROS stack can exist at a time.  Before
# every run the container is cleaned room-style (kill, wait, verify, drop the
# Gazebo cache), because a leftover gzserver or master silently corrupts the
# next run.
#
# Env knobs:
#   STREAK_TARGET  consecutive passes required (default 10)
#   MAX_RUNS       hard cap on how many runs are attempted (default 24)
#   CLEAN_WAIT     seconds to wait after the kill before verifying (default 25)
#   plus every knob team_scripts/elevator_only_test.sh understands
#   (TARGET_FLOORS, CORRIDOR_DEPTHS, CORRIDOR_SPEED, GAIT_POLICY, ...).
#
# Outputs:
#   logs/<PREFIX>_summary.txt   one line per run, in order
#   logs/<PREFIX>_legs.tsv      one line per mission leg with its sim times
#   logs/<PREFIX>_runs.jsonl    the raw per-run summary JSON, one per line
#   logs/<TAG>/seed_<SEED>/     full evidence for each run
set -uo pipefail

# kill_sim_processes.sh (run by every test run, and by clean_room below) kills
# any process whose command line matches "elevator_only_matrix.sh" or
# "elevator_only_test.sh".  It protects itself and its parent, which covers the
# test script but NOT this grandparent runner -- so re-exec a copy under a
# neutral name (/tmp/elevator_runner_*.sh) and keep running from there.
if [ "${ELEVATOR_ONLY_MATRIX_REEXEC:-0}" != "1" ]; then
  matrix_source="${BASH_SOURCE[0]}"
  neutral="/tmp/elevator_runner_$(date +%s)_$$.sh"
  cp "$matrix_source" "$neutral" || exit 4
  export ELEVATOR_ONLY_MATRIX_REEXEC=1
  export ELEVATOR_ONLY_MATRIX_HOME="$(cd "$(dirname "$matrix_source")" && pwd)"
  echo "[matrix] re-exec under a neutral name: $neutral"
  exec bash "$neutral" "$@"
fi

SEED="${1:-20260902}"
PREFIX="${2:-elevstreak}"
WORKSPACE_DIR="$(cd "${ELEVATOR_ONLY_MATRIX_HOME:-$(dirname "${BASH_SOURCE[0]}")}/.." && pwd)"
LOGS="$WORKSPACE_DIR/logs"
STREAK_TARGET="${STREAK_TARGET:-10}"
MAX_RUNS="${MAX_RUNS:-24}"
CLEAN_WAIT="${CLEAN_WAIT:-25}"
SUMMARY="$LOGS/${PREFIX}_summary.txt"
LEGS="$LOGS/${PREFIX}_legs.tsv"
RUNS_JSONL="$LOGS/${PREFIX}_runs.jsonl"

mkdir -p "$LOGS"
: > "$SUMMARY"
: > "$LEGS"
: > "$RUNS_JSONL"
printf 'run\ttag\tseed\tstatus\twall_s\tstate\tpass\trides\tfault\n' >> "$SUMMARY"
printf 'run\ttag\tseed\tleg\tfloor\tboard_sim\tarrived_sim\texit_sim\tcorridor_sim\tturn_sim\tback_sim\tcorridor_truth_max_along\n' >> "$LEGS"

clean_room() {
  bash "$WORKSPACE_DIR/team_scripts/kill_sim_processes.sh" || true
  sleep "$CLEAN_WAIT"
  # Bracket the patterns so the probe itself can never match.
  local residual
  residual="$(ps -eo pid=,comm= | awk '{print $2}' \
    | grep -E '^(gzserver|gzclient|rosmaster|roslaunch|rosout)$' || true)"
  local helpers
  helpers="$(pgrep -f 'fastlio_mapping|junior_ctrl|lio_localization_bridge|building_generator_classic_control' 2>/dev/null | tr '\n' ' ' || true)"
  echo "[matrix] clean-room residual: gz/ros='${residual:-none}' helpers='${helpers:-none}'"
  rm -rf /tmp/.gazebo
}

streak=0
attempt=0
while [ "$attempt" -lt "$MAX_RUNS" ] && [ "$streak" -lt "$STREAK_TARGET" ]; do
  attempt=$((attempt + 1))
  tag="$(printf '%s_%02d' "$PREFIX" "$attempt")"
  run_dir="$LOGS/$tag/seed_$SEED"
  echo "[matrix] ==== run $attempt (tag=$tag) streak=$streak/$STREAK_TARGET ===="
  clean_room

  start_wall="$(date +%s)"
  bash "$WORKSPACE_DIR/team_scripts/elevator_only_test.sh" "$SEED" "$tag" \
    > "$LOGS/${tag}.outer.log" 2>&1
  status=$?
  wall=$(( $(date +%s) - start_wall ))

  mkdir -p "$run_dir"
  {
    echo "tag=$tag"
    echo "seed=$SEED"
    echo "exit_status=$status"
    echo "wall_seconds=$wall"
  } > "$run_dir/elevator_only_run.env"

  python3 - "$RUNS_JSONL" "$SUMMARY" "$LEGS" "$run_dir" "$attempt" "$tag" \
    "$SEED" "$status" "$wall" <<'PY'
import json
import os
import sys

runs_jsonl, summary, legs_tsv, run_dir, attempt, tag, seed, status, wall = sys.argv[1:10]
summary_json = os.path.join(run_dir, "elevator_only_summary.json")
data = None
if os.path.exists(summary_json):
    try:
        with open(summary_json) as handle:
            data = json.load(handle)
    except Exception as error:  # noqa: BLE001
        print("[matrix] could not read %s: %s" % (summary_json, error))
if data is None:
    data = {"passed": False, "final_state": "NO_SUMMARY", "fault": "NO_SUMMARY_JSON",
            "legs": [], "ride_floors": [], "corridor_depths": [], "sim_now": None}
data["exit_status"] = int(status)
data["wall_seconds"] = int(wall)
data["run"] = int(attempt)
data["tag"] = tag
with open(runs_jsonl, "a") as handle:
    handle.write(json.dumps(data, sort_keys=True) + "\n")

with open(summary, "a") as handle:
    handle.write("%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
        attempt, tag, seed, status, wall, data.get("final_state"),
        "PASS" if data.get("passed") else "FAIL", len(data.get("legs") or []),
        (data.get("fault") or "none")))

with open(legs_tsv, "a") as handle:
    for leg in data.get("legs") or []:
        handle.write("%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
            attempt, tag, seed, leg.get("leg"), leg.get("floor"),
            leg.get("board_sim"), leg.get("arrived_sim"), leg.get("exit_sim"),
            leg.get("corridor_sim"), leg.get("turn_sim"), leg.get("back_sim"),
            leg.get("corridor_truth_max_along")))
print("[matrix] %s -> %s state=%s rides=%s wall=%ss fault=%s" % (
    tag, "PASS" if data.get("passed") else "FAIL", data.get("final_state"),
    len(data.get("legs") or []), wall, data.get("fault") or "none"))
PY

  passed="$(python3 -c "import json,sys; print('1' if json.load(open('$run_dir/elevator_only_summary.json')).get('passed') else '0')" 2>/dev/null || echo 0)"
  if [ "$passed" = "1" ] && [ "$status" = "0" ]; then
    streak=$((streak + 1))
  else
    streak=0
  fi
  echo "[matrix] streak now $streak/$STREAK_TARGET"
done

echo
echo "[matrix] ================= run table (seed $SEED) ================="
cat "$SUMMARY"
echo "[matrix] longest target: $STREAK_TARGET consecutive passes; achieved $streak"
echo "[matrix] per-leg sim times: $LEGS"
echo "[matrix] raw per-run json : $RUNS_JSONL"
[ "$streak" -ge "$STREAK_TARGET" ]
