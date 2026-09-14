#!/usr/bin/env python3
"""Sweep RL /cmd_vel forward commands and measure stable body speed."""

import argparse
import json
import math
import time
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def roll_pitch_from_quaternion(q):
    sin_roll = 2.0 * (q.w * q.x + q.y * q.z)
    cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    return roll, pitch


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))
    return ordered[index]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--speeds",
        default="0.30,0.45,0.60,0.75,0.90,1.05",
        help="comma-separated forward command speeds in m/s",
    )
    parser.add_argument("--phase", type=float, default=4.0)
    parser.add_argument("--settle", type=float, default=1.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    speeds = [float(value) for value in args.speeds.split(",") if value.strip()]

    rospy.init_node("measure_rl_speed", anonymous=True)
    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    state = {"truth": None, "previous": None, "samples": []}

    def truth_callback(message):
        stamp = message.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.Time.now().to_sec()
        pose = message.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        roll, pitch = roll_pitch_from_quaternion(pose.orientation)
        current = {
            "stamp": stamp,
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "yaw": yaw,
            "roll": roll,
            "pitch": pitch,
            "twist_forward_speed": math.cos(yaw) * message.twist.twist.linear.x + math.sin(yaw) * message.twist.twist.linear.y,
            "twist_lateral_speed": -math.sin(yaw) * message.twist.twist.linear.x + math.cos(yaw) * message.twist.twist.linear.y,
            "twist_yaw_rate": message.twist.twist.angular.z,
        }
        previous = state["truth"]
        if previous is not None:
            dt = stamp - previous["stamp"]
            if 0.0005 <= dt <= 0.25:
                dx = current["x"] - previous["x"]
                dy = current["y"] - previous["y"]
                heading = 0.5 * (current["yaw"] + previous["yaw"])
                current["forward_speed"] = (math.cos(heading) * dx + math.sin(heading) * dy) / dt
                current["lateral_speed"] = (-math.sin(heading) * dx + math.cos(heading) * dy) / dt
                current["yaw_rate"] = math.atan2(
                    math.sin(current["yaw"] - previous["yaw"]),
                    math.cos(current["yaw"] - previous["yaw"]),
                ) / dt
                state["samples"].append(current)
        state["truth"] = current

    rospy.Subscriber("/ground_truth/base_w", Odometry, truth_callback, queue_size=100)
    deadline = time.monotonic() + 60.0
    while not rospy.is_shutdown() and state["truth"] is None:
        if time.monotonic() >= deadline:
            raise RuntimeError("/ground_truth/base_w did not become available")
        time.sleep(0.05)

    rate = rospy.Rate(30.0)
    results = []
    for speed in speeds:
        state["samples"] = []
        phase_start = rospy.Time.now().to_sec()
        stable_start = phase_start + min(1.0, args.phase * 0.25)
        stop_reason = "completed"
        command = Twist()
        command.linear.x = speed
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now - phase_start >= args.phase:
                break
            current = state["truth"]
            if current is not None and (current["z"] < 0.02 or abs(current["roll"]) > 0.80 or abs(current["pitch"]) > 0.80):
                stop_reason = "unstable_pose"
                break
            publisher.publish(command)
            rate.sleep()
        publisher.publish(Twist())
        samples = [item for item in state["samples"] if item["stamp"] >= stable_start]
        forward = [item.get("twist_forward_speed", item.get("forward_speed", 0.0)) for item in samples]
        lateral = [abs(item.get("twist_lateral_speed", item.get("lateral_speed", 0.0))) for item in samples]
        yaw_rates = [abs(item.get("twist_yaw_rate", item.get("yaw_rate", 0.0))) for item in samples]
        result = {
            "command_speed": speed,
            "stop_reason": stop_reason,
            "sample_count": len(samples),
            "forward_speed_median": percentile(forward, 0.50),
            "forward_speed_p90": percentile(forward, 0.90),
            "lateral_speed_abs_p90": percentile(lateral, 0.90),
            "yaw_rate_abs_p90": percentile(yaw_rates, 0.90),
            "max_abs_roll": max((abs(item["roll"]) for item in samples), default=None),
            "max_abs_pitch": max((abs(item["pitch"]) for item in samples), default=None),
            "min_z": min((item["z"] for item in samples), default=None),
        }
        results.append(result)
        if stop_reason != "completed":
            break
        settle_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() - settle_start < args.settle:
            publisher.publish(Twist())
            rate.sleep()

    publisher.publish(Twist())
    payload = {"speeds": results, "stable_command_speed": None}
    for result in results:
        if result["stop_reason"] != "completed":
            break
        median = result["forward_speed_median"] or 0.0
        if median >= 0.20 and (result["max_abs_roll"] or 0.0) < 0.60 and (result["max_abs_pitch"] or 0.0) < 0.60:
            payload["stable_command_speed"] = result["command_speed"]
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
