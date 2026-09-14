#!/usr/bin/env python3
"""Stop a Stage B run early when the robot stops making physical progress.

The failure this guards against is not "no plan" but "no motion": run26 floor 0
sat with ``cmd_vel`` exactly 0 for 731 consecutive telemetry samples while the
planner kept reporting NO_FRONTIER, and target churn can hide the same thing
from any planner-side counter.  The one signal that cannot be faked is the
robot's own pose: if neither its horizontal position nor its heading changes
for ``--stall-seconds``, the run is not going to finish and the remaining
budget is better spent on the next attempt.

Deliberately conservative, because stopping a healthy run is worse than
waiting:

* motion is measured from ``/simnav/odom`` (source frame, immune to the metric
  z drift that made a height test useless);
* the check is skipped while the elevator transition owns the robot
  (``elevator_state != WAIT_FLOOR_COMPLETE``), because portal alignment and the
  ride itself are legitimately stationary;
* it is skipped once ``floor_complete`` is set, because waiting for a
  map-confirmed elevator portal is a real state, not a stall;
* it is skipped for the first ``--grace-seconds`` of wall time after the
  explorer starts publishing, which covers startup and the lobby transit.

On verdict it writes ``position_stall_verdict.json`` and, unless
``--no-kill`` is given, terminates the runner's process group.
"""

import argparse
import json
import math
import os
import subprocess
import time

import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class PositionStallWatchdog:
    def __init__(self, out_dir, stall_seconds, move_threshold, yaw_threshold,
                 grace_seconds, kill):
        self.out_dir = out_dir
        self.stall_seconds = max(30.0, float(stall_seconds))
        self.move_threshold = max(0.05, float(move_threshold))
        self.yaw_threshold = max(0.05, float(yaw_threshold))
        self.grace_seconds = max(0.0, float(grace_seconds))
        self.kill = bool(kill)
        os.makedirs(self.out_dir, exist_ok=True)
        self.log_path = os.path.join(self.out_dir, "position_stall.log")
        self.verdict_path = os.path.join(self.out_dir, "position_stall_verdict.json")
        self.log = open(self.log_path, "a", buffering=1)

        self.pose = None
        self.last_pose_stamp = None
        self.status = {}
        self.reference = None          # (x, y, yaw, wall_time)
        self.furthest_move_wall = None
        self.started_wall = None
        self.verdict = None
        # Judge stalls on the mission clock.  The sim runs at ~0.09x real time,
        # so a wall-second budget killed run57 while it was inside the normal
        # floor-completion window (12 sim seconds of standing still).
        self.sim_now = rospy.Time(0)
        self.sim_stall_seconds = 0.0

    # ---------------------------------------------------------------- inputs
    def clock_callback(self, message):
        self.sim_now = message.clock

    def pose_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        self.pose = (float(position.x), float(position.y), float(yaw))
        self.last_pose_stamp = time.time()

    def status_callback(self, message):
        try:
            self.status = json.loads(message.data)
        except (TypeError, ValueError):
            self.status = {}

    # ------------------------------------------------------------ evaluation
    def _transition_owns_robot(self):
        state = str(self.status.get("state") or "")
        return bool(state) and state != "WAIT_FLOOR_COMPLETE"

    def _waiting_is_legitimate(self):
        return bool(self.status.get("floor_complete"))

    def tick(self):
        now = time.time()
        if self.pose is None or self.last_pose_stamp is None:
            return
        if now - self.last_pose_stamp > 2.0:
            # No odometry at all: that is a broken bridge, not a stall.
            self.log.write(
                "  odometry silent for {:.1f}s (not counted as a stall)\n".format(
                    now - self.last_pose_stamp
                )
            )
            self.reference = None
            return
        if self.started_wall is None:
            self.started_wall = now
        if now - self.started_wall < self.grace_seconds:
            return
        if self._transition_owns_robot() or self._waiting_is_legitimate():
            # Reset the reference so the stationary transition does not count
            # towards a stall once exploration resumes.
            self.reference = (self.pose[0], self.pose[1], self.pose[2], now, self.sim_now)
            return

        x, y, yaw = self.pose
        if self.reference is None:
            self.reference = (x, y, yaw, now, self.sim_now)
            return
        ref_x, ref_y, ref_yaw, ref_wall, ref_sim = self.reference
        moved = math.hypot(x - ref_x, y - ref_y)
        turned = abs(math.atan2(math.sin(yaw - ref_yaw), math.cos(yaw - ref_yaw)))
        # Translation only.  Turning is logged but must not count as progress:
        # run30 spent 297 sim seconds spinning in place at the ROOM_L_43 doorway
        # (position pinned inside a 0.5 x 1.5 m box, cmd_vx == 0 in 87% of
        # samples, heading swinging across the whole [-pi, pi]) and this
        # watchdog never fired, because the yaw test kept resetting the
        # reference.  A legitimate in-place alignment lasts a few seconds; a
        # spin that outlasts the stall window is a failure.
        if moved >= self.move_threshold:
            self.reference = (x, y, yaw, now, self.sim_now)
            return

        # Judge the stall on the mission clock, not the wall clock.  The sim
        # runs at ~0.09x real time, so a wall budget of 420 s is only ~38 sim
        # seconds - inside the normal floor-completion window, which is exactly
        # how run57 was killed while it was healthy.
        stalled_wall = now - ref_wall
        stalled = (self.sim_now - ref_sim).to_sec() if ref_sim is not None else stalled_wall
        self.log.write(
            "still: moved={:.3f}m turned={:.2f}rad for {:.0f} sim s "
            "({:.0f} wall s) state={} region={}\n".format(
                moved, turned, stalled, stalled_wall,
                self.status.get("state"), self.status.get("topology_region"),
            )
        )
        if stalled >= self.stall_seconds:
            self._report(x, y, yaw, moved, turned, stalled, ref_x, ref_y)

    def _report(self, x, y, yaw, moved, turned, stalled, ref_x, ref_y):
        self.verdict = {
            "verdict": "POSITION_STALL",
            "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "stalled_seconds": round(stalled, 1),
            "displacement_m": round(moved, 3),
            "heading_change_rad": round(turned, 3),
            "pose": {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)},
            "reference_pose": {"x": round(ref_x, 3), "y": round(ref_y, 3)},
            "explorer_status": self.status,
        }
        with open(self.verdict_path, "w") as handle:
            json.dump(self.verdict, handle, indent=2, sort_keys=True, default=str)
        self.log.write(
            "POSITION STALL: no motion for {:.0f}s (moved {:.3f}m, turned "
            "{:.2f}rad) -> {}\n".format(stalled, moved, turned, self.verdict_path)
        )
        if self.kill:
            self._terminate_run()
        rospy.signal_shutdown("position stall")

    def _terminate_run(self):
        try:
            listing = subprocess.run(
                ["ps", "-eo", "pid=,args="], capture_output=True, text=True,
                timeout=10,
            ).stdout
        except (OSError, subprocess.SubprocessError) as error:
            self.log.write("  could not list processes: {}\n".format(error))
            return
        victims = []
        for line in listing.splitlines():
            fields = line.strip().split(None, 1)
            if len(fields) != 2:
                continue
            pid, args = fields
            if "watch_position_stall" in args or "ps -eo" in args:
                continue
            if any(
                token in args
                for token in (
                    "run_stage_b_seed.sh", "roslaunch", "rosmaster", "gzserver",
                    "junior_ctrl", "coverage_explorer_node.py",
                    "two_floor_explorer_supervisor.py", "elevator_transition_node.py",
                    "danger_detector_node.py", "lio_", "map_views_node.py",
                    "monitor_stage_b_coverage.py", "record_stage_b_telemetry.py",
                )
            ):
                victims.append(pid)
        for pid in victims:
            subprocess.run(["kill", "-9", pid], capture_output=True, timeout=5)
        self.log.write("  terminated {} run processes\n".format(len(victims)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    # Sim seconds: a genuinely stuck robot still trips this, while the
    # normal completion/elevator waits no longer do.
    parser.add_argument("--stall-seconds", type=float, default=180.0)
    parser.add_argument("--move-threshold", type=float, default=0.30)
    parser.add_argument("--yaw-threshold", type=float, default=0.40)
    parser.add_argument("--grace-seconds", type=float, default=120.0)
    parser.add_argument("--no-kill", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("stage_b_position_stall_watch", anonymous=True)
    watchdog = PositionStallWatchdog(
        args.out_dir, args.stall_seconds, args.move_threshold,
        args.yaw_threshold, args.grace_seconds, not args.no_kill,
    )
    rospy.Subscriber("/simnav/odom", Odometry, watchdog.pose_callback, queue_size=20)
    rospy.Subscriber(
        "/simnav/elevator_status", String, watchdog.status_callback, queue_size=5
    )
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            watchdog.tick()
        except Exception as error:  # a watchdog must never crash the run itself
            watchdog.log.write("watchdog error: {}\n".format(error))
        rate.sleep()


if __name__ == "__main__":
    main()
