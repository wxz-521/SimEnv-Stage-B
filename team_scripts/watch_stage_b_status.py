#!/usr/bin/env python3
"""Live one-line view of the Stage B explorer state for interactive testing.

Subscribes to public topics only and prints a compact line per planning cycle
plus an explicit line whenever a state-relevant field changes.  Per-room
coverage is tracked for every topology the planner reports, so room completion
can be audited against the configured target instead of inferred from the
floor-wide coverage alone.
"""

import argparse
import json
import time

import rospy
from std_msgs.msg import String


WATCH_FIELDS = (
    "topology_region",
    "topology_lock",
    "active_target_kind",
    "active_target_topology",
    "last_plan_reason",
    "completed_topologies",
    "candidate_topologies",
    "actionable_portals",
    "completed_front_sides",
    "front_station_topologies",
)


def num(value):
    try:
        return "{:.3f}".format(float(value))
    except (TypeError, ValueError):
        return str(value)


class Watcher:
    def __init__(self, interval):
        self.interval = max(0.2, float(interval))
        self.last_print = 0.0
        self.last_watch = {}
        self.rooms_seen = []
        self.elevator = {}
        self.last_room_line = {}
        self.coverage_history = {}

        rospy.Subscriber("/simnav/explorer_status", String, self.status_cb, queue_size=1)
        rospy.Subscriber("/simnav/elevator_status", String, self.elevator_cb, queue_size=5)

    def elevator_cb(self, message):
        try:
            self.elevator = json.loads(message.data)
        except (TypeError, ValueError):
            return

    def _report_room_coverage(self, data, stamp):
        """Print one line per room whenever the reported coverage moves."""
        coverages = data.get("room_coverages")
        if not isinstance(coverages, dict):
            return
        target = data.get("room_combined_coverage_target")
        for topology_id in sorted(coverages):
            local = coverages.get(topology_id) or {}
            combined = float(local.get("combined", 0.0))
            previous = self.last_room_line.get(topology_id)
            if previous is not None and abs(combined - previous) < 0.005:
                continue
            self.last_room_line[topology_id] = combined
            self.coverage_history.setdefault(topology_id, []).append(
                (round(stamp, 2), combined)
            )
            reached = ""
            try:
                if target is not None and combined >= float(target):
                    reached = "  >= target"
            except (TypeError, ValueError):
                pass
            print(
                "{}  ROOM COVERAGE {:<14} laser={:<6} camera={:<6} combined={:<6} "
                "target={} cells={}{}".format(
                    "sim={:8.2f}".format(stamp),
                    topology_id,
                    num(local.get("laser")),
                    num(local.get("camera")),
                    num(combined),
                    num(target) if target is not None else "-",
                    int(local.get("task_cells", 0)),
                    reached,
                ),
                flush=True,
            )

    def status_cb(self, message):
        try:
            data = json.loads(message.data)
        except (TypeError, ValueError):
            return
        now = rospy.Time.now().to_sec()
        if now - self.last_print < self.interval:
            return
        self.last_print = now

        stamp = "sim={:8.2f}".format(now)
        rooms = data.get("completed_topologies") or []
        done = len(rooms)

        # Explicit change lines so a state transition is never missed.
        for field in WATCH_FIELDS:
            value = data.get(field)
            if isinstance(value, list):
                value = tuple(value)
            if self.last_watch.get(field) != value:
                self.last_watch[field] = value
                print(
                    "{}  CHG {:<26} -> {}".format(stamp, field, value),
                    flush=True,
                )
        for room in rooms:
            if room not in self.rooms_seen:
                self.rooms_seen.append(room)
                print(
                    "{}  ROOM COMPLETE  {}   ({} total)".format(stamp, room, len(self.rooms_seen)),
                    flush=True,
                )

        self._report_room_coverage(data, now)

        line = (
            "{}  reg={:<24} lock={:<12} tgt={:<14}/{:<12} "
            "cov={}/{}/{} comb={} rooms={} act={} cand={} portals={}"
        ).format(
            stamp,
            str(data.get("topology_region")),
            str(data.get("topology_lock")),
            str(data.get("active_target_kind")),
            str(data.get("active_target_topology")),
            num(data.get("laser_coverage")),
            num(data.get("camera_coverage")),
            num(data.get("combined_coverage")),
            num(data.get("combined_coverage_target")),
            done,
            len(data.get("actionable_portals") or ()),
            len(data.get("candidate_topologies") or ()),
            len(data.get("observed_portals") or ()),
        )
        print(line, flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("stage_b_status_watcher", anonymous=True)
    watcher = Watcher(args.interval)
    print("watching /simnav/explorer_status (interval={}s)".format(watcher.interval), flush=True)
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    main()
