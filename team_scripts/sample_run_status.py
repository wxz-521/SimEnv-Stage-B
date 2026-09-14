#!/usr/bin/env python3
"""Append one compact status line per call - a timeline for the final verdict.

Read-only.  Intended to be run periodically by a background sampler:

    docker exec simenv-noetic python3 team_scripts/sample_run_status.py \
        --run-dir logs/run79_modules_20260913/seed_20260902

Writes ``<run-dir>/verify_timeline.log`` with one CSV row per call so the
"complete exploration" checklist can be reconstructed after the run instead of
being inferred from prose.
"""

import argparse
import datetime
import json
import os

import rospy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rospy.init_node("sample_run_status", anonymous=True, disable_signals=True)
    stamp = datetime.datetime.utcnow().strftime("%H:%M:%S")
    try:
        sim = rospy.wait_for_message("/clock", Clock, timeout=args.timeout).clock.to_sec()
    except Exception:
        sim = float("nan")
    try:
        payload = json.loads(
            rospy.wait_for_message("/simnav/explorer_status", String, timeout=args.timeout).data
        )
    except Exception:
        payload = {}

    coverages = payload.get("room_coverages") or {}
    cov = ";".join(
        "%s=%.3f" % (key.replace("ROOM_", ""), float((value or {}).get("combined", 0.0)))
        for key, value in sorted(coverages.items())
    )
    # T-E: the ordered doorway-crossing history and the A*-start offset.  Written
    # as compact JSON in one CSV field so a single row still answers "how many
    # times did it enter this room, and when", without a 120 s sampler gap hiding
    # an enter/leave/re-enter.
    entry_sequence = payload.get("room_entry_sequence") or []
    entry_seq = ";".join(
        "%s@%s:%s" % (item[2], item[1], item[3])
        for item in entry_sequence
        if isinstance(item, (list, tuple)) and len(item) >= 4
    )
    path_offset = payload.get("path_start_offset")
    fields = [
        datetime.datetime.utcnow().strftime("%Y-%m-%d"),
        stamp,
        "%.1f" % sim,
        str(payload.get("floor_index")),
        str(payload.get("state")),
        str(payload.get("topology_region")),
        str(payload.get("topology_lock")),
        json.dumps(payload.get("room_entry_counts") or {}, sort_keys=True),
        json.dumps(payload.get("retired_topologies") or [], sort_keys=True),
        str(payload.get("active_target_kind")),
        str(payload.get("active_target_topology")),
        str(payload.get("navigation_blocks")),
        str(payload.get("stuck_target_drops")),
        cov,
        entry_seq,
        "%.3f" % float(path_offset) if path_offset is not None else "",
    ]
    line = ",".join(fields)
    path = os.path.join(args.run_dir, "verify_timeline.log")
    header = (
        "date,utc,sim,floor,state,region,lock,entries,retired,target_kind,"
        "target_topology,nav_blocks,stuck_drops,coverages,entry_seq,path_offset\n"
    )
    new = not os.path.exists(path)
    with open(path, "a") as handle:
        if new:
            handle.write(header)
        handle.write(line + "\n")
    print(line)


if __name__ == "__main__":
    main()
