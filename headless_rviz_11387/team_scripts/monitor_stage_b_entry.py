#!/usr/bin/env python3
"""Stop after the first room entry and report the entry state timings."""

import argparse
import json
from pathlib import Path
import time

import rospy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-timeout", type=float, default=120.0)
    parser.add_argument("--seed", default="unknown")
    parser.add_argument("--output", required=True)
    parser.add_argument("--minimum-depth", type=float, default=0.65)
    parser.add_argument("--maximum-lateral-error", type=float, default=0.30)
    parser.add_argument("--minimum-travel", type=float, default=0.75)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("stage_b_entry_monitor", anonymous=True)

    first_candidate = None
    state_sequence = []
    durations = {}
    active_key = None
    active_elapsed = 0.0
    entered_room = False
    latest_status = {}
    crossing_metrics = {}

    def finish_active():
        if active_key is not None:
            durations[active_key] = durations.get(active_key, 0.0) + active_elapsed

    def status_callback(message):
        nonlocal first_candidate, active_key, active_elapsed
        nonlocal entered_room, latest_status, crossing_metrics
        payload = json.loads(message.data)
        latest_status = payload
        if payload.get("door_crossing_depth") is not None:
            crossing_metrics = {
                "depth": float(payload["door_crossing_depth"]),
                "lateral_error": float(payload["door_crossing_lateral_error"]),
                "travel": float(payload["door_crossing_travel"]),
            }
        state = payload.get("state", "")
        candidate = payload.get("active_candidate")
        if first_candidate is None and candidate and state == "GO_TO_PRE_DOOR":
            first_candidate = candidate
        phase = payload.get("room_scan_phase") if state == "ROOM_SCAN" else None
        key = phase if phase else state
        if candidate and first_candidate is None:
            first_candidate = candidate
        # Entry timing is intentionally global until the first ROOM_SCAN: a
        # latched status may omit the earlier candidate id, but the state
        # transition itself remains the correct boundary for this test.
        try:
            elapsed = max(0.0, float(payload.get("state_elapsed", 0.0)))
        except (TypeError, ValueError):
            elapsed = 0.0
        if active_key is not None and key != active_key:
            durations[active_key] = durations.get(active_key, 0.0) + active_elapsed
        active_key = key
        active_elapsed = elapsed
        if not state_sequence or state_sequence[-1] != state:
            state_sequence.append(state)
        if state == "ROOM_SCAN":
            entered_room = True

    rospy.Subscriber("/simnav/explorer_status", String, status_callback, queue_size=20)
    while not rospy.is_shutdown() and rospy.Time.now() == rospy.Time(0):
        time.sleep(0.02)
    start = rospy.Time.now()
    while not rospy.is_shutdown() and not entered_room:
        if (rospy.Time.now() - start).to_sec() >= args.sim_timeout:
            break
        time.sleep(0.05)
    finish_active()
    elapsed = max(0.0, (rospy.Time.now() - start).to_sec())
    geometry_passed = bool(
        crossing_metrics
        and crossing_metrics["depth"] >= args.minimum_depth
        and abs(crossing_metrics["lateral_error"]) <= args.maximum_lateral_error
        and crossing_metrics["travel"] >= args.minimum_travel
    )
    payload = {
        "mode": "entry_only",
        "seed": args.seed,
        "passed": bool(entered_room and geometry_passed),
        "entered_room": bool(entered_room),
        "crossing_geometry_passed": geometry_passed,
        "crossing_metrics": crossing_metrics,
        "crossing_thresholds": {
            "minimum_depth": args.minimum_depth,
            "maximum_lateral_error": args.maximum_lateral_error,
            "minimum_travel": args.minimum_travel,
        },
        "first_candidate": first_candidate,
        "elapsed_sim_time": round(elapsed, 3),
        "state_sequence": state_sequence,
        "state_durations": {
            key: round(value, 3) for key, value in sorted(durations.items())
        },
        "latest_status": latest_status,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    rospy.signal_shutdown("first room entry measured")
    return 0 if entered_room and geometry_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
