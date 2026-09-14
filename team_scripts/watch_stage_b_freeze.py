#!/usr/bin/env python3
"""Watch a Stage B explorer and capture full diagnostics the moment it stalls.

The compact 1 Hz watcher line only carries the plan *reason* (``NO_FRONTIER``),
which is not enough to tell an empty candidate pool from a pool that was
emptied by the detour filter.  This monitor fingerprints the failure instead:

* it appends the same compact line, and
* when ``combined_coverage`` has not moved for ``--stall-seconds`` of sim time
  while the planner is emitting no target, it writes the complete status JSON
  once, so ``last_reject_reason``, ``planner_diagnostics``,
  ``candidate_topologies`` and the per-room coverage table are all preserved.

Read-only: it subscribes to /simnav/explorer_status and touches no service.

Usage:
  watch_stage_b_freeze.py --out-dir DIR [--stall-seconds 20] [--interval 1.0]
"""

import argparse
import json
import os
import sys
import time

import rospy
from std_msgs.msg import String


class FreezeWatcher:
    def __init__(self, out_dir, stall_seconds, interval, heartbeat=30.0):
        self.out_dir = out_dir
        self.stall_seconds = max(5.0, float(stall_seconds))
        self.interval = max(0.2, float(interval))
        self.heartbeat = max(5.0, float(heartbeat))
        self.last_heartbeat_sim = 0.0
        self.status = None
        self.sim_time = 0.0
        self.last_key = None
        self.last_change_sim = None
        self.dumped = False
        self.episodes = 0
        os.makedirs(self.out_dir, exist_ok=True)
        self.stream_path = os.path.join(self.out_dir, "freeze_watch.log")
        self.dump_path = os.path.join(self.out_dir, "freeze_diagnostics.json")
        self.stream = open(self.stream_path, "a", buffering=1)

    def _sim_now(self):
        try:
            return rospy.Time.now().to_sec()
        except Exception:
            return time.time()

    def status_callback(self, message):
        try:
            self.status = json.loads(message.data)
        except (TypeError, ValueError):
            return

    def _key(self, data):
        # Progress is measured by coverage only.  Including the active target
        # let target churn reset the stall timer, which is exactly how the
        # run26 deadlock went unrecorded: the explorer alternated between a
        # corridor lidar frontier and no target while combined coverage stayed
        # pinned at 0.169, so every tick looked like a change.
        return (
            round(float(data.get("laser_coverage") or 0.0), 3),
            round(float(data.get("camera_coverage") or 0.0), 3),
            round(float(data.get("combined_coverage") or 0.0), 3),
            len(data.get("completed_topologies") or ()),
        )

    def tick(self):
        sim = self._sim_now()
        data = self.status
        if data is None:
            return
        key = self._key(data)
        reason = data.get("last_plan_reason")
        target = data.get("active_target")
        changed = key != self.last_key
        if changed:
            self.last_key = key
            self.last_change_sim = sim
            self.dumped = False
            self.last_heartbeat_sim = sim
        stalled = sim - float(self.last_change_sim or sim)
        # A planner that reports NO_FRONTIER with no dispatched target is the
        # measured deadlock signature even before coverage visibly stalls, so
        # treat it as a stall in its own right.
        deadlocked = target is None and str(reason) == "NO_FRONTIER"
        # Startup is not a stall.  Floor generation, robot spawn and the fixed
        # lobby transit all legitimately emit no target for tens of sim seconds
        # with zero coverage, which produced a spurious forensic dump at sim 22
        # of run28 and would make the dump file meaningless as a signal.
        exploring = bool(
            str(data.get("topology_region")) not in ("", "LOBBY_TRANSIT", "None")
            or float(data.get("combined_coverage") or 0.0) > 0.0
        ) and str(reason) not in ("NOT_READY", "NO_SAFE_SEED")
        heartbeat_due = sim - float(getattr(self, "last_heartbeat_sim", 0.0) or 0.0) >= self.heartbeat
        if changed or heartbeat_due or stalled >= self.stall_seconds:
            self.last_heartbeat_sim = sim
            line = (
                "sim={:8.2f} region={:<16} lock={:<12} target={:<10}/{} "
                "cov={}/{}/{} reason={} stalled={:6.1f}s deadlock={}"
            ).format(
                sim,
                str(data.get("topology_region")),
                str(data.get("topology_lock")),
                str(data.get("active_target_kind")),
                str(data.get("active_target_topology")),
                round(float(data.get("laser_coverage") or 0.0), 3),
                round(float(data.get("camera_coverage") or 0.0), 3),
                round(float(data.get("combined_coverage") or 0.0), 3),
                str(reason),
                stalled,
                "YES" if deadlocked else "no",
            )
            self.stream.write(line + "\n")
        # Coverage can legitimately stand still for many seconds while a target
        # is already dispatched -- returning to the corridor, aligning, or
        # holding a review stance.  Only a planner that has *no* target is the
        # deadlock this dump exists for; without this gate the file appeared at
        # sim 96 of run29 during a normal ROOM_RETURNING manoeuvre and stopped
        # being a usable signal.
        no_target = target is None
        if not self.dumped and exploring and no_target and (
            stalled >= self.stall_seconds
            or (deadlocked and stalled >= max(5.0, self.stall_seconds / 2.0))
        ):
            self.dumped = True
            self.episodes += 1
            self._dump(data, sim, stalled, reason, target)

    def _dump(self, data, sim, stalled, reason, target):
        payload = {
            "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "sim_time": sim,
            "stalled_sim_seconds": stalled,
            "plan_reason": reason,
            "active_target": target,
            "explorer_status": data,
            "planner_diagnostics": data.get("planner_diagnostics"),
        }
        try:
            with open(self.dump_path, "w") as handle:
                json.dump(payload, handle, indent=2, sort_keys=True, default=str)
            self.stream.write(
                "  !! STALL captured (episode {}): reason={} -> {}\n".format(
                    self.episodes, reason, self.dump_path
                )
            )
        except OSError as error:
            self.stream.write("  !! dump failed: {}\n".format(error))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--stall-seconds", type=float, default=20.0)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--heartbeat-seconds", type=float, default=30.0)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("stage_b_freeze_watch", anonymous=True)
    watcher = FreezeWatcher(
        args.out_dir, args.stall_seconds, args.interval, args.heartbeat_seconds
    )
    rospy.Subscriber("/simnav/explorer_status", String, watcher.status_callback, queue_size=2)
    rate = rospy.Rate(1.0 / watcher.interval)
    while not rospy.is_shutdown():
        try:
            watcher.tick()
        except Exception as error:  # never let monitoring kill the watcher
            sys.stderr.write("freeze watcher error: {}\n".format(error))
        rate.sleep()


if __name__ == "__main__":
    main()
