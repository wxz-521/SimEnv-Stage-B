#!/usr/bin/env python3
"""Drive one map-axis centerline segment for a supervised Gazebo test."""

import argparse
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
from sensor_msgs.msg import LaserScan


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("target_x", type=float)
    parser.add_argument("--center-y", type=float, default=0.0)
    parser.add_argument("--speed", type=float, default=0.45)
    parser.add_argument("--sim-timeout", type=float, default=30.0)
    parser.add_argument("--stop-distance", type=float, default=0.8)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("drive_centerline", anonymous=True)
    state = {"pose": None, "front": float("inf")}
    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def odom_callback(message):
        q = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        state["pose"] = (message.pose.pose.position.x, message.pose.pose.position.y, yaw)

    def scan_callback(message):
        ranges = []
        for index, distance in enumerate(message.ranges):
            angle = normalize_angle(message.angle_min + index * message.angle_increment)
            if abs(angle) <= math.radians(18.0) and math.isfinite(distance):
                if message.range_min <= distance <= message.range_max:
                    ranges.append(distance)
        state["front"] = min(ranges) if ranges else float("inf")

    rospy.Subscriber("/simnav/odom", Odometry, odom_callback, queue_size=1)
    rospy.Subscriber("/scan_2d", LaserScan, scan_callback, queue_size=1)
    while state["pose"] is None or rospy.Time.now() == rospy.Time(0):
        time.sleep(0.02)
    start_time = rospy.Time.now()
    rate = rospy.Rate(20)
    reason = "shutdown"
    try:
        while not rospy.is_shutdown():
            x, y, yaw = state["pose"]
            if x >= args.target_x:
                reason = "target"
                break
            if state["front"] < args.stop_distance:
                reason = "obstacle"
                break
            if (rospy.Time.now() - start_time).to_sec() >= args.sim_timeout:
                reason = "timeout"
                break
            desired_yaw = max(-0.16, min(0.16, -0.30 * (y - args.center_y)))
            yaw_error = normalize_angle(desired_yaw - yaw)
            command = Twist()
            command.linear.x = args.speed
            command.angular.z = max(-0.15, min(0.15, 0.8 * yaw_error))
            publisher.publish(command)
            rate.sleep()
    finally:
        publisher.publish(Twist())
    x, y, yaw = state["pose"]
    print(
        "reason={} elapsed_sim={:.3f} pose=({:.3f}, {:.3f}, {:.3f}) front={:.3f}".format(
            reason, (rospy.Time.now() - start_time).to_sec(), x, y, yaw, state["front"]
        )
    )
    return 0 if reason == "target" else 1


if __name__ == "__main__":
    raise SystemExit(main())
