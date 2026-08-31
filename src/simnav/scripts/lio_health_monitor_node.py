#!/usr/bin/env python3
"""Publish bounded FAST-LIO effective-point health for Stage B fault handling."""

import json

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class LioHealthMonitor:
    def __init__(self):
        self.minimum_points = int(rospy.get_param("~minimum_effective_points", 5))
        self.max_count_points = int(rospy.get_param("~max_count_points", 20000))
        self.last_message = rospy.Time(0)
        self.pub = rospy.Publisher("/simnav/lio_health", String, queue_size=5, latch=True)
        self.sub = rospy.Subscriber(
            rospy.get_param("~cloud_topic", "/cloud_registered"),
            PointCloud2,
            self._callback,
            queue_size=2,
        )
        self.timer = rospy.Timer(rospy.Duration(0.5), self._stale_callback)

    def _callback(self, message):
        # PointCloud2 carries a bounded frame size.  Counting the advertised
        # samples avoids a Python per-point scan at every FAST-LIO frame.
        count = min(
            int(message.width) * max(1, int(message.height)),
            self.max_count_points,
        )
        stamp = message.header.stamp if message.header.stamp != rospy.Time(0) else rospy.Time.now()
        self.last_message = rospy.Time.now()
        self._publish("GOOD" if count >= self.minimum_points else "NO_EFFECTIVE_POINTS", count, stamp)

    def _stale_callback(self, _event):
        if self.last_message == rospy.Time(0) or (rospy.Time.now() - self.last_message).to_sec() > 1.5:
            self._publish("STALE", 0, rospy.Time.now())

    def _publish(self, state, count, stamp):
        self.pub.publish(
            String(
                data=json.dumps(
                    {
                        "state": state,
                        "effective_points": int(count),
                        "timestamp": stamp.to_sec(),
                    },
                    sort_keys=True,
                )
            )
        )


if __name__ == "__main__":
    rospy.init_node("lio_health_monitor")
    LioHealthMonitor()
    rospy.spin()
