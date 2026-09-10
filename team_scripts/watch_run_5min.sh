#!/usr/bin/env bash
# Poll a Stage B run every INTERVAL seconds and append a compact report.
# Reads files only: no ROS queries, so a report can never block on a topic.
# Usage: watch_run_5min.sh <run_dir> [interval_seconds]
set -u

RUN_DIR="${1:?usage: watch_run_5min.sh RUN_DIR [INTERVAL]}"
INTERVAL="${2:-300}"
REPORT="$RUN_DIR/periodic_report.log"

report() {
  local stamp
  stamp="$(date '+%H:%M:%S')"
  {
    echo "==================== $stamp ===================="
    if docker exec simenv-noetic bash -lc 'pgrep -f "bash ./team_scripts/run_stage_b_seed.sh" >/dev/null' 2>/dev/null; then
      echo "run: RUNNING"
    else
      echo "run: STOPPED"
    fi
    echo "fall_snapshots: $(ls "$RUN_DIR"/fall_*.csv 2>/dev/null | wc -l)"
    if [ -f "$RUN_DIR/telemetry.csv" ]; then
      echo "telemetry_rows: $(wc -l < "$RUN_DIR/telemetry.csv")"
      tail -1 "$RUN_DIR/telemetry.csv" \
        | awk -F, '{printf "telemetry_last: sim=%s wz=%s roll=%s pitch=%s cmd_vx=%s cmd_wz=%s state=%s\n",$2,$8,$9,$10,$17,$18,$19}'
    else
      echo "telemetry_rows: (missing)"
    fi
    if [ -f "$RUN_DIR/watch_status.log" ]; then
      echo "--- explorer events ---"
      grep -E "ROOM COMPLETE|CHG topology_lock|CHG completed_topologies" \
        "$RUN_DIR/watch_status.log" | tail -6
      echo "--- explorer last ---"
      tail -1 "$RUN_DIR/watch_status.log" | cut -c1-150
    fi
    if [ -s "$RUN_DIR/result.json" ]; then
      echo "--- result.json ---"
      timeout 15 python3 -c "
import json
try:
    d=json.load(open('$RUN_DIR/result.json'))
except Exception as exc:
    print('  unreadable:', exc); raise SystemExit
for k in ('completed_room_count','completed_rooms_ok','floor_complete','floor_transition_complete','two_floor_mission_complete','mission_fault','passed','elapsed_sim_time'):
    print('  %-26s = %s' % (k, d.get(k)))
" 2>/dev/null
    fi
    echo
  } >> "$REPORT" 2>&1
}

while true; do
  report
  sleep "$INTERVAL"
done
