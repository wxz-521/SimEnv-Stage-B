#!/usr/bin/env python3
"""Capture the live occupancy grid plus the planner's portal geometry.

Used to replay a stalled planning cycle offline: guessing at why a doorway
band is empty is far slower than re-running the band computation on the exact
map the planner saw.
"""

import argparse
import json
import pickle

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--grid-topic", default="/navigation_map")
    parser.add_argument("--status-topic", default="/simnav/explorer_status")
    parser.add_argument("--out", required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    rospy.init_node("capture_stall_scene", anonymous=True)
    holder = {}

    def grid_cb(message):
        holder["grid"] = {
            "resolution": message.info.resolution,
            "origin_x": message.info.origin.position.x,
            "origin_y": message.info.origin.position.y,
            "width": message.info.width,
            "height": message.info.height,
            "data": list(message.data),
        }

    def status_cb(message):
        holder["status"] = json.loads(message.data)

    rospy.Subscriber(args.grid_topic, OccupancyGrid, grid_cb)
    rospy.Subscriber(args.status_topic, String, status_cb)
    deadline = rospy.Time.now() + rospy.Duration(args.timeout)
    while not rospy.is_shutdown() and (
        "grid" not in holder or "status" not in holder
    ):
        if rospy.Time.now() > deadline:
            break
        rospy.sleep(0.2)

    with open(args.out, "wb") as handle:
        pickle.dump(holder, handle)
    print(
        "captured grid=%s status=%s -> %s"
        % ("grid" in holder, "status" in holder, args.out)
    )


if __name__ == "__main__":
    main()
