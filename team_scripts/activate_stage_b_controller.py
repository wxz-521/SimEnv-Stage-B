#!/usr/bin/env python3
"""Switch junior_ctrl from passive to stand, then RL /cmd_vel mode."""

import argparse
import time

import rospy
from sensor_msgs.msg import Joy


BUTTON_STAND = 1
BUTTON_RL = 3


def publish_button(publisher, button, duration):
    start = rospy.Time.now()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < duration:
        message = Joy()
        message.header.stamp = rospy.Time.now()
        message.axes = [0.0] * 6
        message.buttons = [0] * 11
        message.buttons[button] = 1
        publisher.publish(message)
        rate.sleep()


def wait_sim_duration(duration, wall_timeout):
    start = rospy.Time.now()
    deadline = time.monotonic() + wall_timeout
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < duration:
        if time.monotonic() >= deadline:
            raise RuntimeError("simulation clock did not advance during controller activation")
        rate.sleep()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("sequence", "stand", "rl"), default="sequence")
    parser.add_argument("--stand-hold", type=float, default=2.0)
    parser.add_argument("--command-duration", type=float, default=0.5)
    parser.add_argument("--wall-timeout", type=float, default=60.0)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("stage_b_controller_activation", anonymous=True)
    publisher = rospy.Publisher("/joy", Joy, queue_size=1)

    deadline = time.monotonic() + args.wall_timeout
    while publisher.get_num_connections() == 0 and not rospy.is_shutdown():
        if time.monotonic() >= deadline:
            raise RuntimeError("junior_ctrl did not subscribe to /joy")
        time.sleep(0.05)

    if args.mode in ("sequence", "stand"):
        publish_button(publisher, BUTTON_STAND, args.command_duration)
        wait_sim_duration(args.stand_hold, args.wall_timeout)
        rospy.loginfo("junior_ctrl activated: fixed stand")
    if args.mode in ("sequence", "rl"):
        publish_button(publisher, BUTTON_RL, args.command_duration)
        rospy.loginfo("junior_ctrl activated: RL /cmd_vel")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
