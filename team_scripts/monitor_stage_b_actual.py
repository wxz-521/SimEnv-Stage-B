#!/usr/bin/env python3
"""Monitor real competition-world Stage B behavior using public algorithm topics."""

import argparse
import json
import math
from pathlib import Path
import subprocess
import time

from gazebo_msgs.msg import PerformanceMetrics
from nav_msgs.msg import OccupancyGrid, Odometry
import rospy
from std_msgs.msg import Bool, String

from stage_b_monitor_core import confirmed_hypotheses, doorway_station_structure


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-timeout", type=float, default=240.0)
    parser.add_argument("--seed", default="unknown")
    parser.add_argument(
        "--expected-rooms-per-floor", "--expected-room-count",
        dest="expected_rooms_per_floor", type=int, default=4,
    )
    parser.add_argument("--commit", default="")
    parser.add_argument(
        "--result-file", default="/workspace/SimEnv/results/detected_danger.json"
    )
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("stage_b_actual_monitor", anonymous=True)
    state_sequence = []
    latest_counts = {}
    latest_candidate_statuses = {}
    danger_count = 0
    floor_complete = False
    last_pose = None
    max_pose_step = 0.0
    pose_messages = 0
    known_cells = 0
    occupied_cells = 0
    rtf_samples = []
    latest_status = {}
    active_status_key = None
    active_status_elapsed = 0.0
    state_durations = {}
    door_hypotheses = []
    pending_door_hypothesis_count = 0
    localization_state_sequence = []
    localization_snapshot = {}
    loop_closure_snapshot = {}
    loop_closure_events = []
    mission_fault = None
    latest_detection_lifecycle = {}
    corridor_diagnostics = []

    def status_callback(message):
        nonlocal latest_counts, latest_status, latest_candidate_statuses
        nonlocal active_status_key, active_status_elapsed
        payload = json.loads(message.data)
        latest_status = payload
        state = payload.get("state", "")
        latest_counts = payload.get("counts", {})
        candidate_statuses = payload.get("candidate_statuses", {})
        if isinstance(candidate_statuses, dict):
            latest_candidate_statuses = {
                str(candidate_id): str(status)
                for candidate_id, status in candidate_statuses.items()
            }
        phase = payload.get("room_scan_phase") if state == "ROOM_SCAN" else None
        status_key = (
            state,
            payload.get("active_candidate"),
            phase,
        )
        try:
            state_elapsed = max(0.0, float(payload.get("state_elapsed", 0.0)))
        except (TypeError, ValueError):
            state_elapsed = 0.0
        if active_status_key is not None and status_key != active_status_key:
            duration_key = _status_duration_key(active_status_key)
            state_durations[duration_key] = (
                state_durations.get(duration_key, 0.0) + active_status_elapsed
            )
        active_status_key = status_key
        active_status_elapsed = state_elapsed
        if state and (not state_sequence or state_sequence[-1] != state):
            state_sequence.append(state)

    def danger_callback(message):
        nonlocal danger_count
        danger_count = len(json.loads(message.data).get("dangers", []))

    def complete_callback(message):
        nonlocal floor_complete
        floor_complete = message.data

    def odom_callback(message):
        nonlocal last_pose, max_pose_step, pose_messages
        pose = (message.pose.pose.position.x, message.pose.pose.position.y)
        if last_pose is not None:
            max_pose_step = max(max_pose_step, math.hypot(pose[0] - last_pose[0], pose[1] - last_pose[1]))
        last_pose = pose
        pose_messages += 1

    def map_callback(message):
        nonlocal known_cells, occupied_cells
        known_cells = sum(1 for value in message.data if value >= 0)
        occupied_cells = sum(1 for value in message.data if value >= 50)

    def performance_callback(message):
        if message.real_time_factor > 0.0:
            rtf_samples.append(message.real_time_factor)

    def hypothesis_callback(message):
        nonlocal door_hypotheses, pending_door_hypothesis_count
        raw_hypotheses = json.loads(message.data)
        door_hypotheses = confirmed_hypotheses(raw_hypotheses)
        pending_door_hypothesis_count = len(raw_hypotheses) - len(door_hypotheses)

    def health_callback(message):
        nonlocal localization_snapshot
        localization_snapshot = json.loads(message.data)
        state = localization_snapshot.get("state")
        if state and (
            not localization_state_sequence
            or localization_state_sequence[-1] != state
        ):
            localization_state_sequence.append(state)

    def loop_closure_callback(message):
        nonlocal last_pose, loop_closure_snapshot
        payload = json.loads(message.data)
        loop_closure_snapshot = payload
        loop_closure_events.append(payload)
        corrected = payload.get("corrected_pose")
        if payload.get("accepted") and corrected:
            last_pose = (float(corrected[0]), float(corrected[1]))

    def fault_callback(message):
        nonlocal mission_fault
        try:
            mission_fault = json.loads(message.data)
        except (TypeError, ValueError):
            mission_fault = {"state": "MISSION_FAULT", "reason": message.data}

    def lifecycle_callback(message):
        nonlocal latest_detection_lifecycle
        try:
            latest_detection_lifecycle = json.loads(message.data)
        except (TypeError, ValueError):
            latest_detection_lifecycle = {}

    def corridor_diagnostic_callback(message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        corridor_diagnostics.append(payload)
        # Keep the formal result bounded while retaining enough history to
        # classify a corridor stall after the run.
        if len(corridor_diagnostics) > 2000:
            del corridor_diagnostics[:-2000]

    rospy.Subscriber("/simnav/explorer_status", String, status_callback, queue_size=10)
    rospy.Subscriber("/simnav/danger_tracks", String, danger_callback, queue_size=5)
    rospy.Subscriber("/simnav/floor_complete", Bool, complete_callback, queue_size=2)
    rospy.Subscriber("/simnav/odom", Odometry, odom_callback, queue_size=20)
    rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size=1)
    rospy.Subscriber("/gazebo/performance_metrics", PerformanceMetrics, performance_callback, queue_size=5)
    rospy.Subscriber(
        "/simnav/door_hypotheses", String, hypothesis_callback, queue_size=5
    )
    rospy.Subscriber(
        "/simnav/localization_health", String, health_callback, queue_size=10
    )
    rospy.Subscriber(
        "/simnav/local_loop_closure_applied",
        String,
        loop_closure_callback,
        queue_size=10,
    )
    rospy.Subscriber("/simnav/mission_fault", String, fault_callback, queue_size=2)
    rospy.Subscriber(
        "/simnav/danger_detection_lifecycle", String,
        lifecycle_callback, queue_size=10
    )
    rospy.Subscriber(
        "/simnav/corridor_diagnostic", String,
        corridor_diagnostic_callback, queue_size=20
    )
    while rospy.Time.now() == rospy.Time(0):
        time.sleep(0.02)
    start = rospy.Time.now()
    while not rospy.is_shutdown() and not floor_complete and mission_fault is None:
        if (rospy.Time.now() - start).to_sec() >= args.sim_timeout:
            break
        time.sleep(0.05)
    elapsed = (rospy.Time.now() - start).to_sec()
    end = rospy.Time.now()
    if active_status_key is not None:
        duration_key = _status_duration_key(active_status_key)
        state_durations[duration_key] = (
            state_durations.get(duration_key, 0.0) + active_status_elapsed
        )
    commit = args.commit or _current_commit()
    result_json = None
    result_path = Path(args.result_file)
    if result_path.is_file():
        try:
            result_json = json.loads(result_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            result_json = None
    visited_door_hypotheses = [
        item
        for item in door_hypotheses
        if latest_candidate_statuses.get(str(item.get("id"))) == "VISITED"
    ]
    # Historical/unvisited hypotheses remain useful diagnostics, but the
    # four-room structure is defined by rooms actually completed.
    structure_hypotheses = (
        visited_door_hypotheses if latest_candidate_statuses else door_hypotheses
    )
    door_structure = doorway_station_structure(structure_hypotheses)
    behavior_passed = (
        mission_fault is None
        and floor_complete
        and int(latest_counts.get("VISITED", 0)) == args.expected_rooms_per_floor
        and int(latest_counts.get("UNREACHABLE", 0)) == 0
        and len(structure_hypotheses) == args.expected_rooms_per_floor
        and door_structure["valid"]
        and max_pose_step < 1.0
    )
    payload = {
        "commit": commit,
        "seed": args.seed,
        "expected_room_count": args.expected_rooms_per_floor,
        "expected_rooms_per_floor": args.expected_rooms_per_floor,
        "passed": behavior_passed,
        "mission_fault": mission_fault,
        "latest_detection_lifecycle": latest_detection_lifecycle,
        "corridor_diagnostic_count": len(corridor_diagnostics),
        "corridor_diagnostics": corridor_diagnostics,
        "floor_complete": floor_complete,
        "elapsed_sim_time": round(elapsed, 3),
        "status_counts": latest_counts,
        "state_sequence": state_sequence,
        "state_durations": {
            key: round(value, 3) for key, value in sorted(state_durations.items())
        },
        "danger_count": danger_count,
        "pose_messages": pose_messages,
        "max_pose_step": round(max_pose_step, 4),
        "known_cells": known_cells,
        "occupied_cells": occupied_cells,
        "mean_real_time_factor": round(sum(rtf_samples) / len(rtf_samples), 4) if rtf_samples else None,
        "sim_start": round(start.to_sec(), 3),
        "sim_end": round(end.to_sec(), 3),
        "door_hypothesis_count": len(door_hypotheses),
        "door_hypotheses": door_hypotheses,
        "visited_door_hypotheses": visited_door_hypotheses,
        "candidate_statuses": latest_candidate_statuses,
        "pending_door_hypothesis_count": pending_door_hypothesis_count,
        "door_structure": door_structure,
        "room_entries": state_sequence.count("DOOR_CROSSING"),
        "room_exits": state_sequence.count("EXIT_ROOM"),
        "reverse_sweeps": int(latest_status.get("reverse_sweeps", 0)),
        "localization_state_sequence": localization_state_sequence,
        "localization_snapshot": localization_snapshot,
        "local_loop_closure": loop_closure_snapshot,
        "local_loop_closure_events": loop_closure_events,
        "result_json": result_json,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["passed"] else 1


def _status_duration_key(status_key):
    """Create a stable, readable key for one state/candidate/scan phase."""
    state, candidate, phase = status_key
    if state == "ROOM_SCAN" and phase:
        return "ROOM_SCAN/{}".format(phase)
    if candidate:
        return "{}/{}".format(state, candidate)
    return state


def _current_commit():
    repository = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=str(repository),
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


if __name__ == "__main__":
    raise SystemExit(main())
