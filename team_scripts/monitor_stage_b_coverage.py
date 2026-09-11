#!/usr/bin/env python3
"""Monitor the coverage-based Stage B explorer through public ROS topics."""

import argparse
import copy
import json
import math
import time

from gazebo_msgs.msg import PerformanceMetrics
from nav_msgs.msg import Odometry
import rospy
from std_msgs.msg import Bool, String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-timeout", type=float, default=600.0)
    parser.add_argument("--seed", default="unknown")
    parser.add_argument("--wait-floor-transition", action="store_true")
    parser.add_argument("--transition-only", action="store_true")
    parser.add_argument("--two-floor", action="store_true")
    parser.add_argument("--three-floor", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("stage_b_coverage_monitor", anonymous=True)

    latest = {}
    floor_complete = False
    floor_transition_complete = False
    two_floor_mission_complete = False
    mission_fault = None
    danger_count = 0
    state_sequence = []
    localization_states = []
    elevator_states = []
    latest_elevator = {}
    active_floor_index = 0
    floor_results = {}
    last_pose = None
    max_pose_step = 0.0
    rtf_samples = []

    def status_callback(message):
        nonlocal latest
        try:
            latest = json.loads(message.data)
        except (TypeError, ValueError):
            return
        state = latest.get("state")
        if state == "FLOOR_COMPLETE":
            floor_results[active_floor_index] = copy.deepcopy(latest)
        if state and (not state_sequence or state_sequence[-1] != state):
            state_sequence.append(state)

    def complete_callback(message):
        nonlocal floor_complete
        floor_complete = bool(message.data)
        if message.data:
            floor_results[active_floor_index] = copy.deepcopy(latest)

    def transition_callback(message):
        nonlocal floor_transition_complete
        floor_transition_complete = bool(message.data)

    def elevator_status_callback(message):
        nonlocal latest_elevator, active_floor_index
        try:
            latest_elevator = json.loads(message.data)
        except (TypeError, ValueError):
            return
        state = latest_elevator.get("state")
        if state == "FLOOR_1_READY":
            try:
                active_floor_index = int(latest_elevator.get("floor_index", 0))
            except (TypeError, ValueError):
                pass
        if state and (not elevator_states or elevator_states[-1] != state):
            elevator_states.append(state)

    def two_floor_complete_callback(message):
        nonlocal two_floor_mission_complete
        two_floor_mission_complete = bool(message.data)

    def fault_callback(message):
        nonlocal mission_fault
        try:
            mission_fault = json.loads(message.data)
        except (TypeError, ValueError):
            mission_fault = {"reason": message.data}

    def danger_callback(message):
        nonlocal danger_count
        try:
            danger_count = len(json.loads(message.data).get("dangers", []))
        except (TypeError, ValueError):
            pass

    def health_callback(message):
        try:
            state = json.loads(message.data).get("state")
        except (TypeError, ValueError):
            return
        if state and (not localization_states or localization_states[-1] != state):
            localization_states.append(state)

    def odom_callback(message):
        nonlocal last_pose, max_pose_step
        pose = (message.pose.pose.position.x, message.pose.pose.position.y)
        if last_pose is not None:
            max_pose_step = max(
                max_pose_step,
                math.hypot(pose[0] - last_pose[0], pose[1] - last_pose[1]),
            )
        last_pose = pose

    def performance_callback(message):
        if message.real_time_factor > 0.0:
            rtf_samples.append(float(message.real_time_factor))

    rospy.Subscriber("/simnav/coverage_status", String, status_callback, queue_size=10)
    rospy.Subscriber("/simnav/floor_complete", Bool, complete_callback, queue_size=2)
    rospy.Subscriber(
        "/simnav/floor_transition_complete", Bool, transition_callback, queue_size=2
    )
    rospy.Subscriber(
        "/simnav/elevator_status", String, elevator_status_callback, queue_size=5
    )
    rospy.Subscriber(
        "/simnav/two_floor_mission_complete", Bool,
        two_floor_complete_callback, queue_size=2,
    )
    rospy.Subscriber("/simnav/mission_fault", String, fault_callback, queue_size=2)
    rospy.Subscriber("/simnav/danger_tracks", String, danger_callback, queue_size=5)
    rospy.Subscriber("/simnav/localization_health", String, health_callback, queue_size=5)
    rospy.Subscriber("/simnav/odom", Odometry, odom_callback, queue_size=20)
    rospy.Subscriber(
        "/gazebo/performance_metrics", PerformanceMetrics, performance_callback, queue_size=5
    )

    while not rospy.is_shutdown() and rospy.Time.now() == rospy.Time(0):
        time.sleep(0.02)
    start = rospy.Time.now()
    def task_complete():
        if args.two_floor or args.three_floor:
            return two_floor_mission_complete
        return floor_transition_complete if args.wait_floor_transition else floor_complete

    while not rospy.is_shutdown() and not task_complete() and mission_fault is None:
        if (rospy.Time.now() - start).to_sec() >= args.sim_timeout:
            break
        time.sleep(0.05)
    # /simnav/two_floor_mission_complete is published in the same control
    # iteration as the final /simnav/elevator_status; give the status message a
    # moment to arrive so returned_to_spawn/main_entrance_opened are not read
    # from the previous payload.
    completed_at = rospy.Time.now()
    if task_complete() and not rospy.is_shutdown():
        rospy.sleep(rospy.Duration(1.0))
    elapsed = max(0.0, (completed_at - start).to_sec())
    laser = float(latest.get("laser_coverage", 0.0))
    camera = float(latest.get("camera_coverage", 0.0))
    combined = float(latest.get("combined_coverage", 0.0))
    expected_rooms = int(latest.get("expected_rooms_per_floor", 4))
    completed_topologies = set(latest.get("completed_topologies", []))
    topology_states = latest.get("topology_states", {})
    completed_rooms_ok = bool(
        len(completed_topologies) >= expected_rooms
        and all(
            topology_states.get(topology_id, {}).get("state") == "COMPLETE"
            for topology_id in completed_topologies
        )
    )
    def floor_rooms_ok(floor_index):
        status = floor_results.get(floor_index, {})
        expected = int(status.get("expected_rooms_per_floor", 4))
        completed = set(status.get("completed_topologies", []))
        states = status.get("topology_states", {})
        return bool(
            len(completed) >= expected
            and all(states.get(item, {}).get("state") == "COMPLETE" for item in completed)
        )

    def completed_floor_indices(payload):
        try:
            return {int(item) for item in payload.get("completed_floor_indices", [])}
        except (TypeError, ValueError):
            return set()

    if args.three_floor:
        # The mission now ends by riding back down, opening the main entrance
        # and returning to spawn, so `floor_index` is 0 at completion.  The
        # top-floor requirement has to be checked against the recorded set of
        # completed floors, not against the live floor index.
        passed = bool(
            mission_fault is None
            and two_floor_mission_complete
            and latest_elevator.get("returned_to_spawn", False)
            and latest_elevator.get("main_entrance_opened", False)
            and 2 in completed_floor_indices(latest_elevator)
            and all(floor_rooms_ok(index) for index in (0, 1, 2))
            and max_pose_step < 1.0
        )
    elif args.two_floor:
        passed = bool(
            mission_fault is None
            and two_floor_mission_complete
            and latest_elevator.get("returned_to_spawn", False)
            and latest_elevator.get("main_entrance_opened", False)
            and 1 in completed_floor_indices(latest_elevator)
            and completed_rooms_ok
            and latest_elevator.get("floor1_complete", False)
            and max_pose_step < 1.0
        )
    elif args.transition_only:
        passed = bool(
            mission_fault is None
            and floor_transition_complete
            and latest_elevator.get("floor1_topology_isolated", False)
            and max_pose_step < 1.0
        )
    else:
        # Plain single-floor run.  The elevator isolation flag only exists when
        # the transition was actually enabled (RUN_MODE=full, i.e.
        # --wait-floor-transition); a coverage-only run has no elevator_status
        # at all and used to be unable to pass even with a complete floor.
        passed = bool(
            mission_fault is None
            and floor_complete
            and (floor_transition_complete or not args.wait_floor_transition)
            and (
                latest_elevator.get("floor1_topology_isolated", False)
                if args.wait_floor_transition
                else True
            )
            and completed_rooms_ok
            and not latest.get("unreviewed_sphere_hypotheses", [])
            and max_pose_step < 1.0
        )
    payload = {
        "mode": "joint_coverage",
        "seed": args.seed,
        "passed": passed,
        "floor_complete": floor_complete,
        "floor_transition_complete": floor_transition_complete,
        "two_floor_mission_complete": two_floor_mission_complete,
        "returned_to_spawn": bool(latest_elevator.get("returned_to_spawn", False)),
        "main_entrance_opened": bool(latest_elevator.get("main_entrance_opened", False)),
        "elapsed_sim_time": round(elapsed, 3),
        "mission_fault": mission_fault,
        "state_sequence": state_sequence,
        "localization_state_sequence": localization_states,
        "elevator_state_sequence": elevator_states,
        "max_pose_step": round(max_pose_step, 4),
        "mean_real_time_factor": (
            round(sum(rtf_samples) / len(rtf_samples), 4) if rtf_samples else None
        ),
        "danger_count": danger_count,
        "completed_rooms_ok": completed_rooms_ok,
        "completed_room_count": len(completed_topologies),
        "expected_rooms_per_floor": expected_rooms,
        "laser_coverage": laser,
        "camera_coverage": camera,
        "combined_coverage": combined,
        "task_extent_confident": bool(latest.get("task_extent_confident", False)),
        "task_forward_limit": latest.get("task_forward_limit"),
        "sphere_hypotheses": int(latest.get("sphere_hypotheses", 0)),
        "unreviewed_sphere_hypotheses": latest.get("unreviewed_sphere_hypotheses", []),
        "targets_reached": int(latest.get("targets_reached", 0)),
        "latest_status": latest,
        "latest_elevator_status": latest_elevator,
        "floor_results": {str(key): value for key, value in floor_results.items()},
        "transition_only": args.transition_only,
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
