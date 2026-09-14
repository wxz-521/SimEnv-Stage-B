#!/usr/bin/env python3
"""Elevator transition progress watchdog.

run52 floor 2 spent more than five minutes in ``ESTABLISH_FLOOR_1_TOPOLOGY``
(654 failed A* attempts, ``route_target`` null, a 0.45 m/s crawl that covered
2 m in 25 sim seconds) and nothing reported it as abnormal: the position
watchdog only fires when the robot is completely still, and a slow arc into a
wall is not still.

This watchdog judges every transition state on two things:

* how long the state lasts (per-state budget, sim seconds), and
* whether the robot is actually performing a sensible action - displacement,
  heading change, and the share of non-zero velocity commands.

Verdicts are appended to ``elevator_progress.log`` and the last one is written
to ``elevator_progress_verdict.json``.  Unless ``--no-kill`` is given, a state
that exceeds its hard limit terminates the run so a broken transition does not
consume the whole session.
"""

import argparse
import json
import math
import os
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

# Sim-second budgets measured against a healthy run (run52 floors 0-1): the
# slowest legitimate phase was the post-exit topology drive at 107 sim s.
DEFAULT_BUDGETS = {
    "WAIT_FLOOR_COMPLETE": 3000.0,   # exploration owns this window
    "RETURN_TO_ELEVATOR": 300.0,
    "ALIGN_ELEVATOR": 45.0,
    "ENTER_ELEVATOR": 90.0,
    "RIDE_TO_FLOOR_1": 60.0,
    "RIDE_TO_NEXT_FLOOR": 60.0,
    "RIDE_TO_GROUND_FLOOR": 60.0,
    "ALIGN_FLOOR_1_EXIT": 60.0,
    "EXIT_ELEVATOR": 120.0,
    "ESTABLISH_FLOOR_1_TOPOLOGY": 170.0,
    "FLOOR_1_READY": 3000.0,         # exploration owns this window
    "RETURN_TO_FLOOR_1_GATE": 240.0,
    "ENTER_FLOOR_1_LOBBY": 150.0,
    "SEARCH_FLOOR_1_ELEVATOR": 120.0,
    "ALIGN_FLOOR_1_ELEVATOR_RETURN": 45.0,
    "ENTER_FLOOR_1_ELEVATOR_RETURN": 90.0,
    "ALIGN_GROUND_FLOOR_EXIT": 60.0,
    "EXIT_GROUND_FLOOR": 120.0,
    "OPEN_MAIN_ENTRANCE": 60.0,
    "RETURN_TO_SPAWN": 300.0,
}

# States where the robot legitimately stands still (the lift does the moving).
PASSIVE_STATES = {
    "WAIT_FLOOR_COMPLETE",
    "FLOOR_1_READY",
    "RIDE_TO_FLOOR_1",
    "RIDE_TO_NEXT_FLOOR",
    "RIDE_TO_GROUND_FLOOR",
}

NO_ACTION_SECONDS = 60.0
NO_ACTION_MOVE = 0.30
NO_ACTION_YAW = 0.30


class ElevatorProgressWatch:
    def __init__(self, out_dir, hard_factor, kill, no_action_seconds,
                 wall_limit=1800.0):
        self.wall_limit = float(wall_limit)
        self.out_dir = out_dir
        self.hard_factor = float(hard_factor)
        self.kill = bool(kill)
        self.no_action_seconds = float(no_action_seconds)
        self.log_path = os.path.join(out_dir, "elevator_progress.log")
        self.verdict_path = os.path.join(out_dir, "elevator_progress_verdict.json")
        self.log = open(self.log_path, "a", buffering=1)

        # Judge budgets on the mission's own clock.  The sim runs at ~0.09x
        # real time, so a wall-second budget reported a healthy 26-sim-second
        # return-to-elevator drive as "no action for 281s" and killed the run.
        self.sim_now = rospy.Time(0)
        self.wall_since = None
        self.state = None
        self.state_since = None
        self.reference_pose = None
        self.commands = 0
        self.moving_commands = 0
        self.worst = None

        rospy.Subscriber("/clock", Clock, self.clock_callback)
        rospy.Subscriber("/simnav/elevator_status", String, self.status_callback)
        rospy.Subscriber("/simnav/odom", Odometry, self.pose_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.command_callback)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.tick)

    # ------------------------------------------------------------------ inputs
    def clock_callback(self, message):
        self.sim_now = message.clock

    def status_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        state = str(payload.get("state") or "")
        if not state or state == self.state:
            return
        self.finish_state()
        self.state = state
        self.state_since = self.sim_now
        self.wall_since = time.time()
        self.reference_pose = self.pose()
        self.commands = 0
        self.moving_commands = 0
        self.log.write(
            "[{}] state -> {} (budget {:.0f}s)\n".format(
                time.strftime("%H:%M:%S"),
                state,
                DEFAULT_BUDGETS.get(state, 180.0),
            )
        )

    def pose_callback(self, message):
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        self._pose = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(yaw),
        )

    def command_callback(self, message):
        self.commands += 1
        if abs(message.linear.x) > 1e-3 or abs(message.angular.z) > 1e-3:
            self.moving_commands += 1

    def pose(self):
        return getattr(self, "_pose", None)

    # ------------------------------------------------------------------ judging
    def _motion(self):
        current = self.pose()
        if current is None:
            return 0.0, 0.0
        if self.reference_pose is None:
            # The first state begins before odometry arrives; adopt the first
            # pose as the reference instead of reporting zero motion for ever.
            self.reference_pose = current
            return 0.0, 0.0
        moved = (
            (current[0] - self.reference_pose[0]) ** 2
            + (current[1] - self.reference_pose[1]) ** 2
        ) ** 0.5
        return moved, abs(current[2] - self.reference_pose[2])

    def _dwell(self):
        if self.state_since is None:
            return 0.0
        return (self.sim_now - self.state_since).to_sec()

    def _wall_dwell(self):
        if self.wall_since is None:
            return 0.0
        return time.time() - self.wall_since

    def tick(self, _event=None):
        if self.state is None:
            return
        dwell = self._dwell()
        budget = DEFAULT_BUDGETS.get(self.state, 180.0)
        moved, turned = self._motion()
        share = (self.moving_commands / self.commands) if self.commands else 0.0
        if self.state in PASSIVE_STATES:
            return
        # Session guard: a transition that burns real time without advancing the
        # mission clock is a broken run regardless of sim budgets.
        if self._wall_dwell() > self.wall_limit:
            self._report(
                "WALL_TIME_EXCEEDED", dwell, budget, moved, turned, share
            )
            return
        if dwell >= budget:
            self._report(
                "STATE_OVER_BUDGET", dwell, budget, moved, turned, share
            )
            return
        if (
            dwell >= self.no_action_seconds
            and moved < NO_ACTION_MOVE
            and turned < NO_ACTION_YAW
        ):
            self._report("NO_ACTION", dwell, budget, moved, turned, share)

    def finish_state(self):
        if self.state is None:
            return
        moved, turned = self._motion()
        share = (self.moving_commands / self.commands) if self.commands else 0.0
        dwell = self._dwell()
        self.log.write(
            "    {} ended after {:.1f} sim s ({:.0f} wall s): moved {:.2f}m "
            "turned {:.2f}rad, non-zero commands {:.0%}\n".format(
                self.state, dwell, self._wall_dwell(), moved, turned, share
            )
        )
        self.reference_pose = None
        self.commands = 0
        self.moving_commands = 0

    def _report(self, verdict, dwell, budget, moved, turned, share):
        self.worst = {
            "verdict": verdict,
            "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "state": self.state,
            "dwell_seconds": round(dwell, 1),
            "dwell_wall_seconds": round(self._wall_dwell(), 1),
            "budget_seconds": budget,
            "displacement_m": round(moved, 3),
            "heading_change_rad": round(turned, 3),
            "moving_command_share": round(share, 3),
            "pose": self.pose(),
        }
        with open(self.verdict_path, "w") as handle:
            json.dump(self.worst, handle, indent=2, sort_keys=True, default=str)
        self.log.write(
            "ELEVATOR PROGRESS ABNORMAL: {} in {} after {:.0f}s "
            "(budget {:.0f}s, moved {:.2f}m, turned {:.2f}rad, "
            "non-zero commands {:.0%}) -> {}\n".format(
                verdict, self.state, dwell, budget, moved, turned, share,
                self.verdict_path,
            )
        )
        hard = budget * self.hard_factor
        if verdict == "WALL_TIME_EXCEEDED":
            self.log.write(
                "  wall limit {:.0f}s exceeded; {}\n".format(
                    self.wall_limit, "terminating run" if self.kill else "not killing"
                )
            )
            if self.kill:
                self._terminate_run()
                rospy.signal_shutdown("elevator progress")
            return
        if verdict == "STATE_OVER_BUDGET" and dwell >= hard:
            self.log.write(
                "  hard limit {:.0f}s exceeded; {}\n".format(
                    hard, "terminating run" if self.kill else "not killing"
                )
            )
            if self.kill:
                self._terminate_run()
                rospy.signal_shutdown("elevator progress")
        elif verdict == "NO_ACTION" and dwell >= self.no_action_seconds * 3.0:
            self.log.write("  no action for {:.0f}s; {}\n".format(
                dwell, "terminating run" if self.kill else "not killing"))
            if self.kill:
                self._terminate_run()
                rospy.signal_shutdown("elevator progress")

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
            if "elevator_progress" in args or "ps -eo" in args:
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
    parser.add_argument("--hard-factor", type=float, default=2.0)
    parser.add_argument(
        "--no-action-seconds", type=float, default=NO_ACTION_SECONDS
    )
    parser.add_argument("--wall-limit", type=float, default=1800.0)
    parser.add_argument("--no-kill", action="store_true")
    args = parser.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    rospy.init_node("watch_elevator_progress", anonymous=True)
    ElevatorProgressWatch(
        args.out_dir,
        args.hard_factor,
        not args.no_kill,
        args.no_action_seconds,
        args.wall_limit,
    )
    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())
