#!/usr/bin/env python3
"""Stop after the first room entry and report the entry state timings."""

import argparse
import json
from pathlib import Path
import time

from gazebo_msgs.msg import ModelStates
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
    parser.add_argument("--target-entries", type=int, default=1)
    parser.add_argument("--model-name", default="a1_gazebo")
    parser.add_argument("--ground-truth-entry-depth", type=float, default=0.45)
    parser.add_argument("--ground-truth-lateral-margin", type=float, default=0.10)
    parser.add_argument("--same-station-tolerance", type=float, default=1.5)
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
    model_pose = None
    ground_truth_entries = {}
    entry_order = []

    def model_callback(message):
        nonlocal model_pose
        try:
            index = message.name.index(args.model_name)
        except ValueError:
            return
        pose = message.pose[index]
        model_pose = (float(pose.position.x), float(pose.position.y))

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
            geometry = payload.get("active_candidate_geometry") or {}
            center = geometry.get("center")
            width = geometry.get("width")
            normal_yaw = geometry.get("normal_yaw")
            if (
                candidate
                and model_pose is not None
                and isinstance(center, list)
                and len(center) >= 2
                and width is not None
                and normal_yaw is not None
            ):
                dx = model_pose[0] - float(center[0])
                dy = model_pose[1] - float(center[1])
                cosine = math.cos(float(normal_yaw))
                sine = math.sin(float(normal_yaw))
                depth = dx * cosine + dy * sine
                lateral = -dx * sine + dy * cosine
                physical_entry = {
                    "candidate": candidate,
                    "model_pose": list(model_pose),
                    "door_center": [float(center[0]), float(center[1])],
                    "door_width": float(width),
                    "normal_yaw": float(normal_yaw),
                    "depth": depth,
                    "lateral_error": lateral,
                    "passed": bool(
                        depth >= args.ground_truth_entry_depth
                        and abs(lateral)
                        <= 0.5 * float(width) + args.ground_truth_lateral_margin
                    ),
                }
                if physical_entry["passed"] and candidate not in ground_truth_entries:
                    ground_truth_entries[candidate] = physical_entry
                    entry_order.append(candidate)
            entered_room = len(ground_truth_entries) >= args.target_entries

    rospy.Subscriber("/simnav/explorer_status", String, status_callback, queue_size=20)
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_callback, queue_size=5)
    while not rospy.is_shutdown() and rospy.Time.now() == rospy.Time(0):
        time.sleep(0.02)
    start = rospy.Time.now()
    while not rospy.is_shutdown() and not entered_room:
        if (rospy.Time.now() - start).to_sec() >= args.sim_timeout:
            break
        time.sleep(0.05)
    finish_active()
    elapsed = max(0.0, (rospy.Time.now() - start).to_sec())
    reported_geometry_passed = bool(
        crossing_metrics
        and crossing_metrics["depth"] >= args.minimum_depth
        and abs(crossing_metrics["lateral_error"]) <= args.maximum_lateral_error
        and crossing_metrics["travel"] >= args.minimum_travel
    )
    same_station_passed = True
    if args.target_entries >= 2 and len(entry_order) >= 2:
        first = ground_truth_entries[entry_order[0]]
        second = ground_truth_entries[entry_order[1]]
        tangent = (
            -math.sin(first["normal_yaw"]),
            math.cos(first["normal_yaw"]),
        )
        center_delta = (
            second["door_center"][0] - first["door_center"][0],
            second["door_center"][1] - first["door_center"][1],
        )
        station_delta = abs(
            center_delta[0] * tangent[0] + center_delta[1] * tangent[1]
        )
        normal_delta = abs(
            math.atan2(
                math.sin(second["normal_yaw"] - first["normal_yaw"]),
                math.cos(second["normal_yaw"] - first["normal_yaw"]),
            )
        )
        same_station_passed = bool(
            station_delta <= args.same_station_tolerance
            and abs(normal_delta - math.pi) <= 0.40
        )
    elif args.target_entries >= 2:
        same_station_passed = False
    geometry_passed = bool(
        len(ground_truth_entries) >= args.target_entries and same_station_passed
    )
    payload = {
        "mode": "entry_only",
        "seed": args.seed,
        "passed": bool(entered_room and geometry_passed),
        "entered_room": bool(entered_room),
        "target_entries": args.target_entries,
        "ground_truth_entry_count": len(ground_truth_entries),
        "ground_truth_entries": [
            ground_truth_entries[candidate] for candidate in entry_order
        ],
        "same_station_passed": same_station_passed,
        "crossing_geometry_passed": geometry_passed,
        "reported_crossing_geometry_passed": reported_geometry_passed,
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
