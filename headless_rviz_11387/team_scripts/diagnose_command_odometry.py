#!/usr/bin/env python3
"""Offline-only diagnostic comparing command integration, LIO, and Gazebo truth."""

import argparse
import json
import math
from pathlib import Path
import threading

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rospy


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class CommandOdometryDiagnostic:
    def __init__(self, output_path):
        self.output_path = Path(output_path)
        self.lock = threading.Lock()
        self.command = Twist()
        self.previous_truth = None
        self.command_distance = 0.0
        self.truth_forward_distance = 0.0
        self.truth_path_distance = 0.0
        self.samples = 0
        self.latest_truth = None
        self.latest_lio = None
        rospy.Subscriber("/cmd_vel", Twist, self._command_callback, queue_size=10)
        rospy.Subscriber(
            "/ground_truth/base_w", Odometry, self._truth_callback, queue_size=50
        )
        rospy.Subscriber(
            "/simnav/world_pose", PoseStamped, self._lio_callback, queue_size=50
        )
        rospy.Timer(rospy.Duration(0.5), self._write)

    def _command_callback(self, message):
        with self.lock:
            self.command = message

    def _truth_callback(self, message):
        stamp = message.header.stamp.to_sec()
        position = (message.pose.pose.position.x, message.pose.pose.position.y)
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        with self.lock:
            if self.previous_truth is not None:
                previous_stamp, previous_position, previous_yaw = self.previous_truth
                dt = stamp - previous_stamp
                if 0.0 < dt <= 0.2:
                    dx = position[0] - previous_position[0]
                    dy = position[1] - previous_position[1]
                    heading = 0.5 * (yaw + previous_yaw)
                    self.truth_forward_distance += (
                        math.cos(heading) * dx + math.sin(heading) * dy
                    )
                    self.truth_path_distance += math.hypot(dx, dy)
                    self.command_distance += self.command.linear.x * dt
                    if abs(self.command.linear.x) >= 0.05:
                        self.samples += 1
            self.previous_truth = (stamp, position, yaw)
            self.latest_truth = position

    def _lio_callback(self, message):
        with self.lock:
            self.latest_lio = (message.pose.position.x, message.pose.position.y)

    def _write(self, _event):
        with self.lock:
            ratio = (
                self.truth_forward_distance / self.command_distance
                if abs(self.command_distance) > 0.1
                else None
            )
            lio_error = None
            if self.latest_truth is not None and self.latest_lio is not None:
                lio_error = math.hypot(
                    self.latest_truth[0] - self.latest_lio[0],
                    self.latest_truth[1] - self.latest_lio[1],
                )
            payload = {
                "command_distance": self.command_distance,
                "truth_forward_distance": self.truth_forward_distance,
                "truth_path_distance": self.truth_path_distance,
                "truth_to_command_ratio": ratio,
                "lio_position_error": lio_error,
                "moving_samples": self.samples,
                "truth_position": self.latest_truth,
                "lio_position": self.latest_lio,
            }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        temporary.replace(self.output_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("command_odometry_diagnostic")
    CommandOdometryDiagnostic(args.output)
    rospy.spin()
