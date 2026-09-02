#!/usr/bin/env python3
"""Expose Hector scan matching as a continuous, non-GT SimNav pose chain."""

import json
import math
from pathlib import Path

import rospy
import tf.transformations as transformations
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from pose_continuity import PoseContinuityFilter


def yaw_from_quaternion(quaternion):
    return transformations.euler_from_quaternion(
        [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    )[2]


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class LocalizationBridge:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "simnav_map")
        self.odom_frame = rospy.get_param("~odom_frame", "simnav_odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.floor_z = float(rospy.get_param("~current_floor_z", 0.0))
        self.base_height = float(rospy.get_param("~base_height", 0.265))
        self.latest_imu = None
        self.imu_reference_yaw = None
        self.local_reference_yaw = None
        self.previous_pose = None
        self.continuity = PoseContinuityFilter(
            minimum_jump=rospy.get_param("~continuity_minimum_jump", 0.45),
            maximum_speed=rospy.get_param("~continuity_maximum_speed", 1.5),
            jump_rotation=rospy.get_param("~continuity_jump_rotation", 0.60),
        )
        self.world_alignment = None
        self.world_start = self._load_public_start()
        self.odom_pub = rospy.Publisher("/simnav/odom", Odometry, queue_size=10)
        self.world_pose_pub = rospy.Publisher("/simnav/world_pose", PoseStamped, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        rospy.Subscriber(rospy.get_param("~imu_topic", "/trunk_imu"), Imu, self._imu_callback, queue_size=20)
        rospy.Subscriber(
            rospy.get_param("~pose_topic", "/poseupdate"),
            PoseWithCovarianceStamped,
            self._pose_callback,
            queue_size=20,
        )

    def _load_public_start(self):
        path_text = rospy.get_param("~team_scene_info", "")
        if not path_text:
            return (
                float(rospy.get_param("~world_start_x", 0.0)),
                float(rospy.get_param("~world_start_y", 0.0)),
                float(rospy.get_param("~world_start_yaw", 0.0)),
            )
        path = Path(path_text)
        payload = json.loads(path.read_text())
        start = payload["robot_start"]
        rospy.loginfo("World alignment uses allowed robot_start from %s", path)
        return float(start["x"]), float(start["y"]), float(start["yaw"])

    def _imu_callback(self, message):
        self.latest_imu = message

    def _pose_callback(self, message):
        stamp = message.header.stamp if message.header.stamp != rospy.Time() else rospy.Time.now()
        raw_x = message.pose.pose.position.x
        raw_y = message.pose.pose.position.y
        scan_yaw = yaw_from_quaternion(message.pose.pose.orientation)
        local_yaw = scan_yaw
        if self.latest_imu is not None:
            imu_yaw = yaw_from_quaternion(self.latest_imu.orientation)
            if self.imu_reference_yaw is None:
                self.imu_reference_yaw = imu_yaw
                self.local_reference_yaw = scan_yaw
            local_yaw = normalize_angle(
                self.local_reference_yaw + normalize_angle(imu_yaw - self.imu_reference_yaw)
            )
        local_x, local_y, jumped = self.continuity.update(
            stamp.to_sec(), raw_x, raw_y, scan_yaw, local_yaw
        )
        if jumped:
            rospy.logwarn_throttle(
                1.0,
                "Absorbed scan-matcher relocalization jump (count=%d)",
                self.continuity.absorbed_jumps,
            )
        if self.world_alignment is None:
            world_x, world_y, world_yaw = self.world_start
            alignment_yaw = normalize_angle(world_yaw - local_yaw)
            cosine = math.cos(alignment_yaw)
            sine = math.sin(alignment_yaw)
            alignment_x = world_x - (cosine * local_x - sine * local_y)
            alignment_y = world_y - (sine * local_x + cosine * local_y)
            self.world_alignment = alignment_x, alignment_y, alignment_yaw
            self._publish_world_to_map(stamp, raw_x, raw_y, scan_yaw)

        roll = pitch = 0.0
        if self.latest_imu is not None:
            roll, pitch, _imu_yaw = transformations.euler_from_quaternion(
                [
                    self.latest_imu.orientation.x,
                    self.latest_imu.orientation.y,
                    self.latest_imu.orientation.z,
                    self.latest_imu.orientation.w,
                ]
            )
        quaternion = transformations.quaternion_from_euler(roll, pitch, local_yaw)
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.odom_frame
        odometry.child_frame_id = self.base_frame
        odometry.pose.pose.position.x = local_x
        odometry.pose.pose.position.y = local_y
        odometry.pose.pose.position.z = self.floor_z + self.base_height
        odometry.pose.pose.orientation.x = quaternion[0]
        odometry.pose.pose.orientation.y = quaternion[1]
        odometry.pose.pose.orientation.z = quaternion[2]
        odometry.pose.pose.orientation.w = quaternion[3]
        odometry.pose.covariance = list(message.pose.covariance)
        self._populate_velocity(odometry, stamp, local_x, local_y, local_yaw)
        self.odom_pub.publish(odometry)
        self._publish_map_to_odom(
            stamp, raw_x, raw_y, scan_yaw, local_x, local_y, local_yaw
        )
        self._publish_base_transform(odometry)
        self._publish_world_pose(odometry)
        self.previous_pose = stamp, local_x, local_y, local_yaw

    def _populate_velocity(self, odometry, stamp, x, y, yaw):
        if self.previous_pose is None:
            return
        previous_stamp, previous_x, previous_y, previous_yaw = self.previous_pose
        delta_time = (stamp - previous_stamp).to_sec()
        if delta_time <= 1e-4:
            return
        world_vx = (x - previous_x) / delta_time
        world_vy = (y - previous_y) / delta_time
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        odometry.twist.twist.linear.x = cosine * world_vx + sine * world_vy
        odometry.twist.twist.linear.y = -sine * world_vx + cosine * world_vy
        odometry.twist.twist.angular.z = normalize_angle(yaw - previous_yaw) / delta_time

    def _publish_world_to_map(self, stamp, raw_x, raw_y, raw_yaw):
        world_x, world_y, world_yaw = self.world_start
        alignment_yaw = normalize_angle(world_yaw - raw_yaw)
        cosine = math.cos(alignment_yaw)
        sine = math.sin(alignment_yaw)
        alignment_x = world_x - (cosine * raw_x - sine * raw_y)
        alignment_y = world_y - (sine * raw_x + cosine * raw_y)
        world_to_map = TransformStamped()
        world_to_map.header.stamp = stamp
        world_to_map.header.frame_id = self.world_frame
        world_to_map.child_frame_id = self.map_frame
        world_to_map.transform.translation.x = alignment_x
        world_to_map.transform.translation.y = alignment_y
        quaternion = transformations.quaternion_from_euler(0.0, 0.0, alignment_yaw)
        world_to_map.transform.rotation.x = quaternion[0]
        world_to_map.transform.rotation.y = quaternion[1]
        world_to_map.transform.rotation.z = quaternion[2]
        world_to_map.transform.rotation.w = quaternion[3]
        self.static_broadcaster.sendTransform(world_to_map)

    def _publish_map_to_odom(
        self, stamp, raw_x, raw_y, raw_yaw, local_x, local_y, local_yaw
    ):
        map_to_odom_yaw = normalize_angle(raw_yaw - local_yaw)
        cosine = math.cos(map_to_odom_yaw)
        sine = math.sin(map_to_odom_yaw)
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = self.map_frame
        map_to_odom.child_frame_id = self.odom_frame
        map_to_odom.transform.translation.x = raw_x - (cosine * local_x - sine * local_y)
        map_to_odom.transform.translation.y = raw_y - (sine * local_x + cosine * local_y)
        quaternion = transformations.quaternion_from_euler(0.0, 0.0, map_to_odom_yaw)
        map_to_odom.transform.rotation.x = quaternion[0]
        map_to_odom.transform.rotation.y = quaternion[1]
        map_to_odom.transform.rotation.z = quaternion[2]
        map_to_odom.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(map_to_odom)

    def _publish_base_transform(self, odometry):
        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _publish_world_pose(self, odometry):
        alignment_x, alignment_y, alignment_yaw = self.world_alignment
        cosine = math.cos(alignment_yaw)
        sine = math.sin(alignment_yaw)
        pose = PoseStamped()
        pose.header.stamp = odometry.header.stamp
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = alignment_x + cosine * odometry.pose.pose.position.x - sine * odometry.pose.pose.position.y
        pose.pose.position.y = alignment_y + sine * odometry.pose.pose.position.x + cosine * odometry.pose.pose.position.y
        pose.pose.position.z = odometry.pose.pose.position.z
        local_yaw = yaw_from_quaternion(odometry.pose.pose.orientation)
        quaternion = transformations.quaternion_from_euler(0.0, 0.0, normalize_angle(alignment_yaw + local_yaw))
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        self.world_pose_pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("localization_bridge")
    LocalizationBridge()
    rospy.spin()
