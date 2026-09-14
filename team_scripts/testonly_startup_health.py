#!/usr/bin/env python3
"""TEST-ONLY opening health gate.

Delete this file for the final version: nothing in the mainline calls it.

Why it exists
-------------
run94 was started 3 s after a ``kill -9`` of the previous run.  ``kill -9`` is
asynchronous, so Gazebo came up while the previous world was still tearing down
and the robot was ejected -- world z went 0.32 -> 7.43 -> -11.1 m, the robot
"fell" (BASE_ON_GROUND) at sim 32, LIO diverged to src_x = -51 m, the map turned
into a fan of garbage rays, and the whole run was wasted.  That failure is
visible within ~15 simulated seconds, so this gate aborts a bad start instead of
letting it burn ten minutes before the fall.

Exit status 0 = healthy start, 1 = bad start (the caller retries).
"""

import argparse
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


class HealthGate(object):
    def __init__(self, args):
        self.args = args
        self.sim_now = None
        self.wz = None
        self.src = None
        self.roll = None
        self.samples = 0
        self.violations = []
        rospy.Subscriber("/clock", Clock, self._clock, queue_size=10)
        rospy.Subscriber(
            "/simnav/world_pose_metric", PoseStamped, self._world, queue_size=10
        )
        rospy.Subscriber("/simnav/odom", Odometry, self._source, queue_size=10)
        rospy.Subscriber("/trunk_imu", Imu, self._imu, queue_size=50)

    def _clock(self, message):
        self.sim_now = message.clock.to_sec()

    def _world(self, message):
        self.wz = float(message.pose.position.z)

    def _source(self, message):
        position = message.pose.pose.position
        self.src = (float(position.x), float(position.y))

    def _imu(self, message):
        q = message.orientation
        self.roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x ** 2 + q.y ** 2)
        )

    def check(self):
        """Record one verdict string, or None when this sample is fine."""
        if self.sim_now is None or self.wz is None:
            return None
        self.samples += 1
        if not (self.args.min_z <= self.wz <= self.args.max_z):
            return "world z %.2f m outside [%.2f, %.2f]" % (
                self.wz, self.args.min_z, self.args.max_z
            )
        if self.roll is not None and abs(self.roll) > self.args.max_roll:
            return "body roll %.2f rad exceeds %.2f" % (self.roll, self.args.max_roll)
        if self.src is not None and math.hypot(*self.src) > self.args.max_src:
            return "source pose %.1f m from origin exceeds %.0f" % (
                math.hypot(*self.src), self.args.max_src
            )
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--window", type=float, default=15.0,
                        help="simulated seconds to watch before judging")
    parser.add_argument("--startup-timeout", type=float, default=300.0,
                        help="wall seconds allowed for the topics to appear")
    parser.add_argument("--min-z", type=float, default=0.15)
    parser.add_argument("--max-z", type=float, default=0.55)
    parser.add_argument("--max-roll", type=float, default=0.35)
    parser.add_argument("--max-src", type=float, default=100.0)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("testonly_startup_health", anonymous=True, disable_signals=True)
    gate = HealthGate(args)

    deadline = rospy.Time.now().to_sec() + args.startup_timeout
    while gate.sim_now is None and not rospy.is_shutdown():
        if rospy.Time.now().to_sec() > deadline and gate.sim_now is None:
            print("STARTUP-HEALTH: FAIL - no /clock within %.0f s" % args.startup_timeout)
            return 1
        rospy.sleep(0.2)

    start = gate.sim_now
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        verdict = gate.check()
        if verdict is not None:
            gate.violations.append((gate.sim_now, verdict))
            print("STARTUP-HEALTH: FAIL at sim %.1f - %s" % (gate.sim_now, verdict))
            return 1
        if gate.sim_now - start >= args.window:
            break
        rate.sleep()

    print(
        "STARTUP-HEALTH: PASS - %d samples over %.1f simulated seconds, "
        "world z stayed in [%.2f, %.2f]" % (gate.samples, args.window, args.min_z, args.max_z)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
