#!/usr/bin/env python3
"""Send one move_base goal with a ROS simulated-time timeout."""

import argparse
import json
import math
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import rospy


TERMINAL_STATES = {
    GoalStatus.PREEMPTED,
    GoalStatus.SUCCEEDED,
    GoalStatus.ABORTED,
    GoalStatus.REJECTED,
    GoalStatus.RECALLED,
    GoalStatus.LOST,
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("yaw", type=float)
    parser.add_argument("--sim-timeout", type=float, default=60.0)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("send_simnav_goal", anonymous=True)
    latest_pose = [None]

    def pose_callback(message):
        q = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        latest_pose[0] = [message.pose.pose.position.x, message.pose.pose.position.y, yaw]

    rospy.Subscriber("/simnav/odom", Odometry, pose_callback, queue_size=1)
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    wall_deadline = time.monotonic() + 20.0
    while not client.wait_for_server(rospy.Duration(0.2)):
        if time.monotonic() >= wall_deadline:
            raise SystemExit("move_base action server did not become ready")
    while rospy.Time.now() == rospy.Time(0):
        time.sleep(0.02)
    goal = MoveBaseGoal()
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = "simnav_map"
    goal.target_pose.pose.position.x = args.x
    goal.target_pose.pose.position.y = args.y
    goal.target_pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(args.yaw / 2.0)
    start_time = rospy.Time.now()
    start_pose = latest_pose[0]
    client.send_goal(goal)
    timed_out = False
    while not rospy.is_shutdown() and client.get_state() not in TERMINAL_STATES:
        if (rospy.Time.now() - start_time).to_sec() >= args.sim_timeout:
            timed_out = True
            client.cancel_goal()
            break
        time.sleep(0.03)
    state = client.get_state()
    payload = {
        "state": state,
        "state_text": client.get_goal_status_text(),
        "timed_out": timed_out,
        "elapsed_sim_time": round((rospy.Time.now() - start_time).to_sec(), 3),
        "start_pose": start_pose,
        "final_pose": latest_pose[0],
        "goal": [args.x, args.y, args.yaw],
    }
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if state == GoalStatus.SUCCEEDED and not timed_out else 1


if __name__ == "__main__":
    raise SystemExit(main())
