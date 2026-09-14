#!/usr/bin/env python3
"""Record the elevator hand-off at high rate, read-only.

The 120 s status sampler is enough to see *that* the exploration finished, but
the elevator leg is over in tens of simulated seconds and its failure modes
(wrong heading, off-centre doorway, boarding guard refusing) are only visible
in the raw state stream.  This logger writes one row every 0.5 simulated
seconds to ``<run-dir>/elevator_phase.csv`` and never publishes or calls
anything, so it cannot influence the run.

    python3 team_scripts/record_elevator_phase.py --run-dir logs/<run>/seed_<seed>
"""

import argparse
import json
import math
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

COLUMNS = [
    "sim", "state", "floor_index", "fault",
    "src_x", "src_y", "src_yaw", "src_z",
    "world_x", "world_y", "world_z",
    "portal_src_x", "portal_src_y", "portal_src_yaw",
    "portal_world_x", "portal_world_y",
    "elevator_heading", "route_target", "front_clearance",
]


def yaw_of(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )


class PhaseRecorder(object):
    def __init__(self, path, period):
        self.path = path
        self.period = period
        self.sim_now = 0.0
        self.source = None
        self.world = None
        self.status = {}
        self.last_write = -1.0
        self.written = 0
        self.lock_holder = None
        rospy.Subscriber("/clock", Clock, self._clock, queue_size=10)
        rospy.Subscriber("/simnav/odom", Odometry, self._source, queue_size=10)
        rospy.Subscriber(
            "/simnav/world_pose_metric", PoseStamped, self._world, queue_size=10
        )
        rospy.Subscriber(
            "/simnav/elevator_status", String, self._status, queue_size=10
        )
        if not os.path.exists(path):
            with open(path, "w") as handle:
                handle.write(",".join(COLUMNS) + "\n")
        rospy.Timer(rospy.Duration(period / 4.0), self._tick)

    def _clock(self, message):
        self.sim_now = message.clock.to_sec()

    def _source(self, message):
        pose = message.pose.pose
        self.source = (pose.position.x, pose.position.y,
                       yaw_of(pose.orientation), pose.position.z)

    def _world(self, message):
        pose = message.pose
        self.world = (pose.position.x, pose.position.y,
                      yaw_of(pose.orientation), pose.position.z)

    def _status(self, message):
        try:
            self.status = json.loads(message.data)
        except (TypeError, ValueError):
            pass

    @staticmethod
    def _pick(values, index):
        try:
            return float(values[index])
        except (TypeError, ValueError, IndexError):
            return None

    def _tick(self, _event):
        if self.sim_now - self.last_write < self.period:
            return
        self.last_write = self.sim_now
        status = self.status
        portal = status.get("elevator_portal")
        portal_world = status.get("elevator_portal_world")
        heading = status.get("elevator_heading")
        route = status.get("route_target")
        source = self.source or (None,) * 4
        world = self.world or (None,) * 4
        row = [
            round(self.sim_now, 2),
            status.get("state"),
            status.get("floor_index"),
            status.get("fault", {}).get("reason") if isinstance(status.get("fault"), dict)
            else status.get("fault"),
            self._round(source[0]), self._round(source[1]),
            self._round(source[2]), self._round(source[3]),
            self._round(world[0]), self._round(world[1]), self._round(world[2]),
            self._round(self._pick(portal, 0)), self._round(self._pick(portal, 1)),
            self._round(self._pick(portal, 2)),
            self._round(self._pick(portal_world, 0)), self._round(self._pick(portal_world, 1)),
            self._round(heading),
            "{}".format([round(float(v), 2) for v in route]) if route else "",
            self._round(status.get("front_clearance")),
        ]
        try:
            with open(self.path, "a") as handle:
                handle.write(",".join("" if v is None else str(v) for v in row) + "\n")
            self.written += 1
        except OSError as error:
            rospy.logwarn_throttle(10.0, "elevator phase recorder: %s", error)

    @staticmethod
    def _round(value):
        return None if value is None else round(float(value), 3)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--period", type=float, default=0.5)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("elevator_phase_recorder", anonymous=True, disable_signals=True)
    path = os.path.join(args.run_dir, "elevator_phase.csv")
    recorder = PhaseRecorder(path, max(0.1, args.period))
    rospy.loginfo("elevator phase recorder -> %s", path)
    rospy.spin()
    sys.exit(0)


if __name__ == "__main__":
    main()
