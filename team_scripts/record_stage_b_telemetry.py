#!/usr/bin/env python3
"""Record high-rate base telemetry so a fall can be diagnosed after the fact.

A quadruped that tips over leaves no trace in the coverage logs: the explorer
only reports topology at 1 Hz and stops entirely once the floor is complete,
and FAST-LIO's own position log is disabled for these runs.  This node keeps a
short rolling window of metric pose, body attitude and the commanded velocity,
writes it to CSV, and dumps a focused snapshot the moment a fall is detected.

Outputs:
  <out_dir>/telemetry.csv   full run, one row per sample
  <out_dir>/fall_<sim>.csv  the rolling window around a detected fall
"""

import argparse
import csv
import math
import os
import threading
import time
from collections import deque

import rospy
import tf.transformations as transformations
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String


COLUMNS = (
    "wall",
    "sim",
    "src_x",
    "src_y",
    "src_yaw",
    "wx",
    "wy",
    "wz",
    "wroll",
    "wpitch",
    "wyaw",
    "imu_roll",
    "imu_pitch",
    "imu_gx",
    "imu_gy",
    "imu_gz",
    "cmd_vx",
    "cmd_wz",
    "elevator_state",
)


def euler(orientation):
    return transformations.euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w]
    )


class TelemetryRecorder:
    def __init__(self, out_dir, window_seconds, roll_limit_deg, pitch_limit_deg,
                 base_height):
        self.lock = threading.Lock()
        self.start = time.time()
        self.out_dir = out_dir
        self.fall_roll_limit = math.radians(float(roll_limit_deg))
        self.fall_pitch_limit = math.radians(float(pitch_limit_deg))
        self.fall_base_height = float(base_height)
        self.source = {}
        self.metric = {}
        self.imu = {}
        self.command = (0.0, 0.0)
        self.elevator_state = None
        self.fall_reported = False

        os.makedirs(self.out_dir, exist_ok=True)
        self.telemetry_path = os.path.join(self.out_dir, "telemetry.csv")
        self.handle = open(self.telemetry_path, "w", newline="")
        self.writer = csv.writer(self.handle)
        self.writer.writerow(COLUMNS)
        self.handle.flush()

        # Bounded by time, not by count: the sample rate differs per topic.
        self.buffer = deque(maxlen=4000)
        self.window_seconds = max(2.0, float(window_seconds))

        rospy.Subscriber("/simnav/odom", Odometry, self._source_cb, queue_size=50)
        rospy.Subscriber(
            "/simnav/world_pose_metric", PoseStamped, self._metric_cb, queue_size=50
        )
        rospy.Subscriber("/trunk_imu", Imu, self._imu_cb, queue_size=100)
        rospy.Subscriber("/cmd_vel", Twist, self._command_cb, queue_size=50)
        rospy.Subscriber(
            "/simnav/elevator_status", String, self._elevator_cb, queue_size=10
        )
        rospy.Timer(rospy.Duration(0.05), self._tick)

    # --- inputs -----------------------------------------------------------
    def _source_cb(self, message):
        with self.lock:
            pose = message.pose.pose
            self.source = {
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": euler(pose.orientation)[2],
            }

    def _metric_cb(self, message):
        with self.lock:
            roll, pitch, yaw = euler(message.pose.orientation)
            self.metric = {
                "x": float(message.pose.position.x),
                "y": float(message.pose.position.y),
                "z": float(message.pose.position.z),
                "roll": float(roll),
                "pitch": float(pitch),
                "yaw": float(yaw),
            }

    def _imu_cb(self, message):
        with self.lock:
            roll, pitch, _yaw = euler(message.orientation)
            self.imu = {
                "roll": float(roll),
                "pitch": float(pitch),
                "gx": float(message.angular_velocity.x),
                "gy": float(message.angular_velocity.y),
                "gz": float(message.angular_velocity.z),
            }

    def _command_cb(self, message):
        with self.lock:
            self.command = (float(message.linear.x), float(message.angular.z))

    def _elevator_cb(self, message):
        try:
            import json

            self.elevator_state = json.loads(message.data).get("state")
        except (TypeError, ValueError):
            return

    # --- recording --------------------------------------------------------
    def _sample(self):
        now = rospy.Time.now().to_sec()
        with self.lock:
            source = dict(self.source)
            metric = dict(self.metric)
            imu = dict(self.imu)
            command = self.command
            state = self.elevator_state
        return (
            round(time.time() - self.start, 3),
            round(now, 3),
            round(source.get("x", float("nan")), 4),
            round(source.get("y", float("nan")), 4),
            round(source.get("yaw", float("nan")), 4),
            round(metric.get("x", float("nan")), 4),
            round(metric.get("y", float("nan")), 4),
            round(metric.get("z", float("nan")), 4),
            round(metric.get("roll", float("nan")), 4),
            round(metric.get("pitch", float("nan")), 4),
            round(metric.get("yaw", float("nan")), 4),
            round(imu.get("roll", float("nan")), 4),
            round(imu.get("pitch", float("nan")), 4),
            round(imu.get("gx", float("nan")), 4),
            round(imu.get("gy", float("nan")), 4),
            round(imu.get("gz", float("nan")), 4),
            round(command[0], 4),
            round(command[1], 4),
            state,
        )

    def _fall_reason(self, row):
        _wall, sim, _sx, _sy, _syaw, _wx, _wy, wz, wroll, wpitch, _wyaw, \
            iroll, ipitch, _gx, _gy, _gz, _vx, _wz, _state = row
        if abs(wroll) > self.fall_roll_limit or abs(wpitch) > self.fall_pitch_limit:
            return "WORLD_ROLLED"
        if abs(iroll) > self.fall_roll_limit or abs(ipitch) > self.fall_pitch_limit:
            return "IMU_ROLLED"
        if wz == wz and wz < self.fall_base_height:  # NaN-safe comparison
            return "BASE_ON_GROUND"
        return None

    def _tick(self, _event):
        row = self._sample()
        self.writer.writerow(row)
        self.handle.flush()
        self.buffer.append(row)
        reason = self._fall_reason(row)
        if reason is not None and not self.fall_reported:
            self.fall_reported = True
            self._dump_window(row, reason)

    def _dump_window(self, row, reason):
        sim = row[1]
        cutoff = row[0] - self.window_seconds
        path = os.path.join(self.out_dir, "fall_{:.1f}.csv".format(sim))
        with open(path, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(("fall_reason", reason))
            writer.writerow(COLUMNS)
            for entry in self.buffer:
                if entry[0] >= cutoff:
                    writer.writerow(entry)
        rospy.logerr(
            "FALL DETECTED (%s) at sim %.1f; window written to %s",
            reason,
            sim,
            path,
        )

    def shutdown(self):
        try:
            self.handle.close()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--window-seconds", type=float, default=20.0)
    parser.add_argument("--roll-limit-deg", type=float, default=60.0)
    parser.add_argument("--pitch-limit-deg", type=float, default=60.0)
    parser.add_argument("--base-height", type=float, default=0.15)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("stage_b_telemetry")
    recorder = TelemetryRecorder(
        args.out_dir,
        args.window_seconds,
        args.roll_limit_deg,
        args.pitch_limit_deg,
        args.base_height,
    )
    rospy.on_shutdown(recorder.shutdown)
    rospy.loginfo("Telemetry recording to %s", recorder.telemetry_path)
    rospy.spin()


if __name__ == "__main__":
    main()
