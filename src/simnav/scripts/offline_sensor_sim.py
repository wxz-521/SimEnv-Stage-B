#!/usr/bin/env python3
"""Deterministic no-Gazebo scan/IMU source for Stage B node tests."""

import math
import time

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def cross(ax, ay, bx, by):
    return ax * by - ay * bx


class OfflineSensorSimulator:
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.scan_rate_hz = float(rospy.get_param("~scan_rate", 10.0))
        self.speed = float(rospy.get_param("~speed", 0.35))
        self.max_range = float(rospy.get_param("~max_range", 12.0))
        self.motion_mode = rospy.get_param("~motion_mode", "scripted")
        self.sim_time = 0.0
        self.last_scan_time = -1.0
        self.command = Twist()
        self.command_stamp = 0.0
        self.command_pose = [0.0, -1.0, math.pi / 2.0]
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)
        self.scan_pub = rospy.Publisher("/scan_2d", LaserScan, queue_size=2)
        self.imu_pub = rospy.Publisher("/trunk_imu", Imu, queue_size=10)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        rospy.Subscriber("/cmd_vel", Twist, self._command_callback, queue_size=1)
        self.segments = self._build_world()
        self._publish_static_transform()

    @staticmethod
    def _build_world():
        segments = []

        def wall(x1, y1, x2, y2):
            segments.append((float(x1), float(y1), float(x2), float(y2)))

        wall(-6, -2, 6, -2)
        wall(-6, 22, 6, 22)
        wall(-6, -2, -6, 22)
        wall(6, -2, 6, 22)
        # Corridor walls with four 1.2 m openings.
        for x in (-1.2, 1.2):
            wall(x, 0, x, 4.4)
            wall(x, 5.6, x, 10.4)
            wall(x, 11.6, x, 16.4)
            wall(x, 17.6, x, 22)
        # Room dividers and furniture-like obstacles.
        for y in (8.0, 14.0):
            wall(-6, y, -1.2, y)
            wall(1.2, y, 6, y)
        for x, y, sx, sy in ((-4.2, 3.0, 1.2, 0.7), (4.0, 9.5, 0.8, 1.2), (-3.8, 17.5, 1.0, 1.0)):
            wall(x - sx / 2, y - sy / 2, x + sx / 2, y - sy / 2)
            wall(x + sx / 2, y - sy / 2, x + sx / 2, y + sy / 2)
            wall(x + sx / 2, y + sy / 2, x - sx / 2, y + sy / 2)
            wall(x - sx / 2, y + sy / 2, x - sx / 2, y - sy / 2)
        return segments

    def _publish_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "base"
        transform.child_frame_id = "laser"
        transform.transform.translation.x = 0.20
        transform.transform.translation.z = 0.08
        transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(transform)

    def _pose(self):
        if self.motion_mode == "cmd_vel":
            return self.command_pose[0], self.command_pose[1], self.command_pose[2], self.command.angular.z
        # Long straight corridor motion with short stationary turns at each end.
        leg_time = 52.0
        turn_time = 4.0
        cycle = 2.0 * (leg_time + turn_time)
        phase = self.sim_time % cycle
        if phase < leg_time:
            return 0.0, -1.0 + self.speed * phase, math.pi / 2.0, 0.0
        if phase < leg_time + turn_time:
            ratio = (phase - leg_time) / turn_time
            return 0.0, -1.0 + self.speed * leg_time, math.pi / 2.0 + math.pi * ratio, math.pi / turn_time
        phase -= leg_time + turn_time
        if phase < leg_time:
            return 0.0, -1.0 + self.speed * (leg_time - phase), -math.pi / 2.0, 0.0
        ratio = (phase - leg_time) / turn_time
        return 0.0, -1.0, -math.pi / 2.0 + math.pi * ratio, math.pi / turn_time

    def _command_callback(self, message):
        self.command = message
        self.command_stamp = self.sim_time

    def _integrate_command(self, step):
        if self.motion_mode != "cmd_vel":
            return
        command = self.command if self.sim_time - self.command_stamp <= 0.5 else Twist()
        yaw = self.command_pose[2]
        self.command_pose[0] += (math.cos(yaw) * command.linear.x - math.sin(yaw) * command.linear.y) * step
        self.command_pose[1] += (math.sin(yaw) * command.linear.x + math.cos(yaw) * command.linear.y) * step
        self.command_pose[2] = math.atan2(
            math.sin(yaw + command.angular.z * step),
            math.cos(yaw + command.angular.z * step),
        )

    def _ray_distance(self, origin_x, origin_y, direction_x, direction_y):
        closest = self.max_range
        for x1, y1, x2, y2 in self.segments:
            segment_x = x2 - x1
            segment_y = y2 - y1
            denominator = cross(direction_x, direction_y, segment_x, segment_y)
            if abs(denominator) < 1e-9:
                continue
            rel_x = x1 - origin_x
            rel_y = y1 - origin_y
            ray_distance = cross(rel_x, rel_y, segment_x, segment_y) / denominator
            segment_ratio = cross(rel_x, rel_y, direction_x, direction_y) / denominator
            if ray_distance >= 0.05 and 0.0 <= segment_ratio <= 1.0:
                closest = min(closest, ray_distance)
        return closest

    def _publish_scan(self, stamp, x, y, yaw):
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "laser"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.radians(0.5)
        scan.range_min = 0.10
        scan.range_max = self.max_range
        scan.scan_time = 1.0 / self.scan_rate_hz
        count = int(round((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1
        laser_x = x + 0.20 * math.cos(yaw)
        laser_y = y + 0.20 * math.sin(yaw)
        scan.ranges = [
            self._ray_distance(laser_x, laser_y, math.cos(yaw + scan.angle_min + index * scan.angle_increment), math.sin(yaw + scan.angle_min + index * scan.angle_increment))
            for index in range(count)
        ]
        self.scan_pub.publish(scan)

    def run(self):
        step = 1.0 / self.rate_hz
        while not rospy.is_shutdown():
            self.sim_time += step
            self._integrate_command(step)
            stamp = rospy.Time.from_sec(self.sim_time)
            self.clock_pub.publish(Clock(clock=stamp))
            x, y, yaw, yaw_rate = self._pose()
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = "imu_link"
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation.w = qw
            imu.angular_velocity.z = yaw_rate
            imu.linear_acceleration.z = 9.81
            self.imu_pub.publish(imu)
            if self.sim_time - self.last_scan_time + 1e-9 >= 1.0 / self.scan_rate_hz:
                self._publish_scan(stamp, x, y, yaw)
                self.last_scan_time = self.sim_time
            # This node is the simulated clock source, so it must not sleep on
            # the ROS clock that it is responsible for advancing.
            time.sleep(step)


if __name__ == "__main__":
    rospy.init_node("offline_sensor_sim")
    OfflineSensorSimulator().run()
