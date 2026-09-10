#!/usr/bin/env python3
"""Append a compact Stage B run report every few minutes.

Reads only files under the run directory, so it can never block on ROS or a
container call.  Reports are appended to <run_dir>/periodic_report.log.
"""

import argparse
import csv
import json
import os
import subprocess
import time
from datetime import datetime


def read_last_line(path):
    try:
        with open(path, "rb") as handle:
            handle.seek(0, os.SEEK_END)
            size = handle.tell()
            block = min(4096, size)
            handle.seek(size - block)
            lines = handle.read().decode("utf-8", "replace").splitlines()
        return lines[-1] if lines else ""
    except OSError:
        return ""


def count_lines(path):
    try:
        with open(path, "rb") as handle:
            return sum(1 for _ in handle)
    except OSError:
        return 0


def run_alive():
    try:
        result = subprocess.run(
            ["docker", "exec", "simenv-noetic", "bash", "-lc",
             'pgrep -f "bash ./team_scripts/run_stage_b_seed.sh" >/dev/null'],
            timeout=15, capture_output=True,
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, OSError):
        return None


def report(run_dir):
    out = []
    stamp = datetime.now().strftime("%H:%M:%S")
    out.append("=" * 22 + " " + stamp + " " + "=" * 22)

    alive = run_alive()
    out.append("run: " + ("RUNNING" if alive else "STOPPED" if alive is False else "UNKNOWN"))

    falls = sorted(
        name for name in (os.listdir(run_dir) if os.path.isdir(run_dir) else [])
        if name.startswith("fall_") and name.endswith(".csv")
    )
    out.append("fall_snapshots: {} {}".format(len(falls), falls[-3:] if falls else ""))

    telemetry = os.path.join(run_dir, "telemetry.csv")
    if os.path.exists(telemetry):
        out.append("telemetry_rows: {}".format(count_lines(telemetry)))
        last = read_last_line(telemetry)
        try:
            row = next(csv.reader([last]))
            out.append(
                "telemetry_last: sim={} wz={} roll={} pitch={} cmd_vx={} cmd_wz={} state={}".format(
                    row[1], row[7], row[8], row[9], row[16], row[17], row[18]
                )
            )
        except (IndexError, csv.Error):
            pass
    else:
        out.append("telemetry_rows: (missing)")

    status = os.path.join(run_dir, "watch_status.log")
    if os.path.exists(status):
        events = []
        with open(status, "r", errors="replace") as handle:
            for line in handle:
                if ("ROOM COMPLETE" in line or "CHG topology_lock" in line
                        or "CHG completed_topologies" in line):
                    events.append(line.rstrip())
        out.append("--- explorer events (last 6) ---")
        out.extend(events[-6:])
        out.append("--- explorer last ---")
        out.append(read_last_line(status)[:150])

    result = os.path.join(run_dir, "result.json")
    if os.path.exists(result) and os.path.getsize(result) > 0:
        try:
            with open(result) as handle:
                payload = json.load(handle)
            out.append("--- result.json ---")
            for key in ("completed_room_count", "completed_rooms_ok", "floor_complete",
                        "floor_transition_complete", "two_floor_mission_complete",
                        "mission_fault", "passed", "elapsed_sim_time"):
                out.append("  {:<26} = {}".format(key, payload.get(key)))
        except (OSError, ValueError) as exc:
            out.append("  result unreadable: {}".format(exc))

    out.append("")
    return "\n".join(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--interval", type=float, default=300.0)
    args = parser.parse_args()
    report_path = os.path.join(args.run_dir, "periodic_report.log")
    os.makedirs(args.run_dir, exist_ok=True)
    while True:
        text = report(args.run_dir)
        with open(report_path, "a") as handle:
            handle.write(text + "\n")
        print(text, flush=True)
        time.sleep(max(20.0, args.interval))


if __name__ == "__main__":
    main()
