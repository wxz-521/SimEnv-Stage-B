#!/usr/bin/env python3
"""Print selected fields of /simnav/explorer_status as parsed JSON.

``rostopic echo`` renders the payload as an escaped YAML string, which is
awkward to read while debugging a stall.  This subscribes once, waits for a
message, and prints ``key = value`` lines for the fields that matter.
"""

import argparse
import json
import sys

import rospy
from std_msgs.msg import String

FIELDS = (
    "floor_index",
    "topology_region",
    "topology_lock",
    "active_target_kind",
    "active_target_topology",
    "active_target",
    "last_plan_reason",
    "last_plan_error",
    "plan_cycles",
    "plan_failures",
    "navigation_blocks",
    "target_replacements",
    "targets_reached",
    "completed_topologies",
    "candidate_topologies",
    "actionable_portals",
    "retired_topologies",
    "room_entry_counts",
    "stuck_target_drops",
    "control_faults",
    "progress_idle_seconds",
    "cmd_vel",
    "combined_coverage",
    "laser_coverage",
    "camera_coverage",
    "combined_coverage_target",
    "front_rooms_complete",
    "room_exhaust_grace_seconds",
    "end_pose_clearance",
    "floor_complete",
    "path_length",
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/simnav/explorer_status")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--fields", default="")
    args = parser.parse_args()

    rospy.init_node("probe_explorer_status", anonymous=True)
    holder = {}

    def callback(message):
        holder["raw"] = message.data

    rospy.Subscriber(args.topic, String, callback)
    deadline = rospy.Time.now() + rospy.Duration(args.timeout)
    while not rospy.is_shutdown() and "raw" not in holder:
        if rospy.Time.now() > deadline:
            print("TIMEOUT: no message on %s within %.1fs" % (args.topic, args.timeout))
            return 2
        rospy.sleep(0.2)

    payload = json.loads(holder["raw"])
    wanted = [f.strip() for f in args.fields.split(",") if f.strip()] or FIELDS
    for key in wanted:
        if key in payload:
            print("%s = %s" % (key, payload[key]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
