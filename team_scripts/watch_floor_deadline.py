#!/usr/bin/env python3
"""Stop a Stage B run when a single floor overruns its time budget.

The position watchdog catches a robot that stops moving.  It cannot catch a
robot that keeps moving but makes no meaningful progress, which is the other
way a floor burns the whole run budget (run30 spent 297 sim seconds spinning at
one doorway).

The budget is per floor and excludes the elevator transit: the clock starts when
that floor's exploration starts and stops when the floor reports complete.  On
overrun the verdict is written and, unless --no-kill is given, the run is
terminated so the remaining time goes to the next attempt instead of the
timeout.
"""

import argparse
import json
import os
import subprocess
import time

import rospy
from std_msgs.msg import String


class FloorDeadlineWatchdog:
    def __init__(self, out_dir, floor_limit, grace, kill):
        self.out_dir = out_dir
        self.floor_limit = max(30.0, float(floor_limit))
        self.grace = max(0.0, float(grace))
        self.kill = bool(kill)
        os.makedirs(self.out_dir, exist_ok=True)
        self.log_path = os.path.join(self.out_dir, "floor_deadline.log")
        self.verdict_path = os.path.join(self.out_dir, "floor_deadline_verdict.json")
        self.log = open(self.log_path, "a", buffering=1)
        self.elevator = {}
        self.explorer = {}
        self.floor_start = {}
        self.floor_end = {}
        self.floor_flag_seen_false = {}
        self.verdict = None
        self.last_report = 0.0

    def elevator_callback(self, message):
        try:
            self.elevator = json.loads(message.data)
        except (TypeError, ValueError):
            pass

    def explorer_callback(self, message):
        try:
            self.explorer = json.loads(message.data)
        except (TypeError, ValueError):
            pass

    def _sim_now(self):
        try:
            return rospy.Time.now().to_sec()
        except Exception:
            return 0.0

    def tick(self):
        now = self._sim_now()
        if now <= 0.0:
            return
        floor = self.elevator.get("floor_index")
        if floor is None:
            return
        floor = int(floor)
        # ``floor_complete`` still carries the previous floor's value for a
        # moment after the index changes, which made the watchdog record the new
        # floor as "complete after 0.0 s" and silently disabled its own timeout
        # for every floor above the ground one.  A floor only counts as complete
        # once its own flag has been seen False and then True.
        raw_complete = bool(self.elevator.get("floor_complete"))
        if raw_complete and not self.floor_flag_seen_false.get(floor, False):
            raw_complete = False
        if not self.elevator.get("floor_complete"):
            self.floor_flag_seen_false[floor] = True
        complete = raw_complete
        region = str(self.explorer.get("topology_region") or "")

        # Exploration of a floor starts once the robot is past the lobby; the
        # elevator transit itself is deliberately not charged to the floor.
        exploring = region not in ("", "LOBBY_TRANSIT")
        if exploring and floor not in self.floor_start:
            self.floor_start[floor] = now
            self.log.write(
                "floor {} exploration started at sim {:.1f}\n".format(floor, now)
            )
        if complete and floor not in self.floor_end:
            self.floor_end[floor] = now
            started = self.floor_start.get(floor)
            elapsed = (now - started) if started else float("nan")
            self.log.write(
                "floor {} complete at sim {:.1f} after {:.1f} sim s\n".format(
                    floor, now, elapsed
                )
            )
            return

        started = self.floor_start.get(floor)
        if started is None or complete:
            return
        elapsed = now - started
        if now - self.last_report >= 60.0:
            self.last_report = now
            self.log.write(
                "floor {} running: {:.0f}/{:.0f} sim s, region={}\n".format(
                    floor, elapsed, self.floor_limit, region
                )
            )
        if elapsed < self.grace:
            return
        if elapsed >= self.floor_limit:
            self._report(floor, elapsed, now, region)

    def _report(self, floor, elapsed, now, region):
        if self.verdict is not None:
            return
        self.verdict = {
            "verdict": "FLOOR_DEADLINE_EXCEEDED",
            "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "floor_index": floor,
            "floor_elapsed_sim_seconds": round(elapsed, 1),
            "floor_limit_sim_seconds": self.floor_limit,
            "completed_floors": sorted(self.floor_end),
            "floor_elapsed": {str(k): round(v - self.floor_start[k], 1)
                              for k, v in self.floor_end.items()
                              if k in self.floor_start},
            "topology_region": region,
            "explorer_status": self.explorer,
            "elevator_status": self.elevator,
        }
        with open(self.verdict_path, "w") as handle:
            json.dump(self.verdict, handle, indent=2, sort_keys=True, default=str)
        self.log.write(
            "FLOOR {} DEADLINE: {:.0f} sim s >= {:.0f} -> {}\n".format(
                floor, elapsed, self.floor_limit, self.verdict_path
            )
        )
        if self.kill:
            self._terminate_run()
        rospy.signal_shutdown("floor deadline")

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
            if "watch_floor_deadline" in args or "ps -eo" in args:
                continue
            if any(
                token in args
                for token in (
                    "run_stage_b_seed.sh", "roslaunch", "rosmaster", "gzserver",
                    "junior_ctrl", "coverage_explorer_node.py",
                    "two_floor_explorer_supervisor.py", "elevator_transition_node.py",
                    "danger_detector_node.py", "lio_", "map_views_node.py",
                    "monitor_stage_b_coverage.py", "record_stage_b_telemetry.py",
                    "rviz -d",
                )
            ):
                victims.append(pid)
        for pid in victims:
            subprocess.run(["kill", "-9", pid], capture_output=True, timeout=5)
        self.log.write("  terminated {} run processes\n".format(len(victims)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--floor-limit-seconds", type=float, default=600.0)
    parser.add_argument("--grace-seconds", type=float, default=120.0)
    parser.add_argument("--no-kill", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("stage_b_floor_deadline", anonymous=True)
    watchdog = FloorDeadlineWatchdog(
        args.out_dir, args.floor_limit_seconds, args.grace_seconds,
        not args.no_kill,
    )
    rospy.Subscriber(
        "/simnav/elevator_status", String, watchdog.elevator_callback, queue_size=5
    )
    rospy.Subscriber(
        "/simnav/explorer_status", String, watchdog.explorer_callback, queue_size=5
    )
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            watchdog.tick()
        except Exception as error:
            watchdog.log.write("watchdog error: {}\n".format(error))
        rate.sleep()


if __name__ == "__main__":
    main()
