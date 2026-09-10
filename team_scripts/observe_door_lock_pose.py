#!/usr/bin/env python3
"""Record corridor-arrival pose and room-door lock timing without controlling ROS."""

import argparse
import json
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def normalize_angle(value):
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_pose(pose):
    q = pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--stop-on-search-limit", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("door_lock_pose_observer", anonymous=True)

    pose = None
    status = None
    previous_initial_forward = None
    sim_time = None
    started = None
    arrival = None
    samples = {}
    first_evidence = None
    locked = None

    def clock_callback(message):
        nonlocal sim_time
        sim_time = message.clock.to_sec()

    def pose_callback(message):
        nonlocal pose
        pose = message.pose

    def snapshot(label, payload):
        if pose is None:
            return None
        gate = payload.get("virtual_isolation_door")
        pose_yaw = yaw_from_pose(pose)
        result = {
            "label": label,
            "sim_time": round(float(sim_time or 0.0), 4),
            "pose": [
                round(float(pose.position.x), 5),
                round(float(pose.position.y), 5),
                round(float(pose_yaw), 6),
            ],
            "door_search_travel": float(payload.get("door_search_travel", 0.0)),
            "region": payload.get("topology_region"),
            "evidence": dict(payload.get("portal_evidence", {})),
        }
        if isinstance(gate, list) and len(gate) >= 3:
            dx = float(pose.position.x) - float(gate[0])
            dy = float(pose.position.y) - float(gate[1])
            gate_yaw = float(gate[2])
            result["gate"] = [float(value) for value in gate[:3]]
            result["along_from_gate"] = round(
                dx * math.cos(gate_yaw) + dy * math.sin(gate_yaw), 5
            )
            result["lateral_from_gate"] = round(
                -dx * math.sin(gate_yaw) + dy * math.cos(gate_yaw), 5
            )
            result["yaw_error_deg"] = round(
                math.degrees(normalize_angle(pose_yaw - gate_yaw)), 4
            )
        return result

    def status_callback(message):
        nonlocal status, previous_initial_forward, arrival, first_evidence, locked
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        status = payload
        initial_forward = bool(payload.get("initial_forward_active", False))
        if (
            arrival is None
            and previous_initial_forward is True
            and not initial_forward
        ):
            arrival = snapshot("corridor_arrival", payload)
        previous_initial_forward = initial_forward

        travel = float(payload.get("door_search_travel", 0.0))
        for milestone in (1, 2, 3):
            key = "search_{}m".format(milestone)
            if key not in samples and travel >= float(milestone):
                samples[key] = snapshot(key, payload)

        evidence = payload.get("portal_evidence", {})
        maximum = max((int(value) for value in evidence.values()), default=0)
        if first_evidence is None and maximum > 0:
            first_evidence = snapshot("first_evidence", payload)
        confirm_cycles = int(payload.get("portal_confirm_cycles", 3))
        if locked is None and (
            maximum >= confirm_cycles or payload.get("active_target_topology")
        ):
            locked = snapshot("locked", payload)

    rospy.Subscriber(
        "/simnav/world_pose_metric", PoseStamped, pose_callback, queue_size=20
    )
    rospy.Subscriber("/clock", Clock, clock_callback, queue_size=20)
    rospy.Subscriber(
        "/simnav/explorer_status", String, status_callback, queue_size=20
    )

    while not rospy.is_shutdown() and sim_time is None:
        time.sleep(0.02)
    started = sim_time
    wall_started = time.monotonic()
    while not rospy.is_shutdown():
        if sim_time is not None and sim_time < started:
            started = sim_time
        elapsed = float(sim_time or started) - started
        search_limit_reached = bool(
            args.stop_on_search_limit
            and status
            and float(status.get("door_search_travel", 0.0))
            >= float(status.get("door_search_limit", 3.0)) - 1e-6
        )
        if locked is not None or search_limit_reached or elapsed >= args.timeout:
            break
        if time.monotonic() - wall_started >= max(180.0, 8.0 * args.timeout):
            break
        time.sleep(0.02)

    result = {
        "arrival": arrival,
        "samples": samples,
        "first_evidence": first_evidence,
        "locked": locked,
        "locked_successfully": locked is not None,
        "final_region": status.get("topology_region") if status else None,
        "final_door_search_travel": (
            status.get("door_search_travel") if status else None
        ),
    }
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
