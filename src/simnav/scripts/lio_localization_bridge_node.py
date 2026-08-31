#!/usr/bin/env python3
"""Expose FAST-LIO as the stable SimNav localization contract."""

import json
import math
from pathlib import Path
import threading
import time

import numpy as np
import rospy
import tf.transformations as transformations
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from pose_continuity import CommandPoseIntegrator


def yaw_from_matrix(matrix):
    return transformations.euler_from_matrix(matrix)[2]


class LioLocalizationBridge:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "simnav_map")
        self.odom_frame = rospy.get_param("~odom_frame", "simnav_odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.base_height = float(rospy.get_param("~base_height", 0.265))
        self.world_start = self._load_public_start()
        self.metric_pose = CommandPoseIntegrator(
            self.world_start[0],
            self.world_start[1],
            response_scale=float(rospy.get_param("~command_response_scale", 0.95)),
            command_timeout=float(rospy.get_param("~command_timeout", 0.35)),
            minimum_motion_fraction=float(
                rospy.get_param("~minimum_motion_fraction", 0.20)
            ),
        )
        self.world_alignment = None
        self.local_alignment = None
        self.latest_base_pose = None
        self.lock = threading.Lock()
        self.metric_lock = threading.Lock()
        self.previous_metric_source = None
        self.latest_metric_source = None
        self.latest_metric_output = (self.world_start[0], self.world_start[1])
        self.explorer_state = None
        self.trust_command_motion = False
        self.max_corridor_constraint = float(
            rospy.get_param("~max_corridor_constraint", 3.0)
        )
        self.max_loop_correction = float(
            rospy.get_param("~max_loop_correction", 1.0)
        )
        self.max_loop_rotation = float(rospy.get_param("~max_loop_rotation", 0.12))
        self.loop_closure_count = 0
        self.loop_closure_rejections = 0
        self.loop_closure_total_ms = 0.0
        self.loop_closure_max_translation = 0.0
        translation = rospy.get_param(
            "~base_to_imu_translation", [0.223376, -0.02329, 0.118983]
        )
        rotation_rpy = rospy.get_param(
            "~base_to_imu_rotation_rpy", [0.0, 0.785, 0.0]
        )
        self.base_to_imu = transformations.euler_matrix(*rotation_rpy)
        self.base_to_imu[:3, 3] = np.asarray(translation, dtype=float)
        self.imu_to_base = np.linalg.inv(self.base_to_imu)
        self.odom_pub = rospy.Publisher("/simnav/odom", Odometry, queue_size=20)
        self.world_pose_pub = rospy.Publisher(
            "/simnav/world_pose", PoseStamped, queue_size=20
        )
        self.metric_pose_pub = rospy.Publisher(
            "/simnav/world_pose_metric", PoseStamped, queue_size=20
        )
        self.alignment_pub = rospy.Publisher(
            "/simnav/lio_map_transform", TransformStamped, queue_size=1, latch=True
        )
        self.loop_closure_pub = rospy.Publisher(
            "/simnav/local_loop_closure_applied", String, queue_size=5, latch=True
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        rospy.Subscriber(
            rospy.get_param("~lio_odom_topic", "/Odometry"),
            Odometry,
            self._odom_callback,
            queue_size=50,
        )
        rospy.Subscriber(
            "/simnav/local_loop_closure_request",
            String,
            self._loop_closure_callback,
            queue_size=5,
        )
        rospy.Subscriber("/cmd_vel", Twist, self._command_callback, queue_size=20)
        rospy.Subscriber(
            "/simnav/explorer_status",
            String,
            self._explorer_status_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            "/simnav/metric_corridor_constraint",
            String,
            self._metric_corridor_constraint_callback,
            queue_size=5,
        )

    def _command_callback(self, message):
        with self.metric_lock:
            self.metric_pose.set_command(
                rospy.Time.now().to_sec(), message.linear.x, message.linear.y
            )

    def _explorer_status_callback(self, message):
        try:
            state = json.loads(message.data).get("state")
        except (TypeError, ValueError):
            return
        with self.metric_lock:
            self.explorer_state = state
            # Door crossing and camera projection must follow observed motion.
            # Command integration can report progress while the body is pinned
            # against a jamb, producing a false room entry and misplaced RViz
            # camera cells.  FAST-LIO remains authoritative in every phase.
            self.trust_command_motion = False

    def _metric_corridor_constraint_callback(self, message):
        try:
            payload = json.loads(message.data)
            target = tuple(float(value) for value in payload["position"][:2])
            stamp = float(payload["timestamp"])
        except (KeyError, TypeError, ValueError):
            return
        if len(target) != 2 or rospy.Time.now().to_sec() - stamp > 0.5:
            return
        with self.metric_lock:
            correction = math.hypot(
                target[0] - self.latest_metric_output[0],
                target[1] - self.latest_metric_output[1],
            )
            if correction > self.max_corridor_constraint:
                return
            self.metric_pose.x, self.metric_pose.y = target
            self.latest_metric_output = target

    def _load_public_start(self):
        path_text = rospy.get_param("~team_scene_info", "")
        if not path_text:
            return 0.0, 0.0, 0.0
        payload = json.loads(Path(path_text).read_text())
        start = payload["robot_start"]
        return float(start["x"]), float(start["y"]), float(start["yaw"])

    def _odom_callback(self, message):
        imu_pose = transformations.quaternion_matrix(
            [
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
            ]
        )
        imu_pose[:3, 3] = [
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        ]
        raw_base_pose = np.matmul(imu_pose, self.imu_to_base)
        with self.lock:
            if self.local_alignment is None:
                desired_base_pose = np.identity(4)
                desired_base_pose[2, 3] = self.base_height
                self.local_alignment = np.matmul(
                    desired_base_pose, np.linalg.inv(raw_base_pose)
                )
                alignment_message = self._matrix_transform(
                    self.local_alignment,
                    message.header.stamp,
                    self.map_frame,
                    "camera_init",
                )
                self.alignment_pub.publish(alignment_message)
            base_pose = np.matmul(self.local_alignment, raw_base_pose)
            if self.world_alignment is None:
                local_yaw = yaw_from_matrix(base_pose)
                world_x, world_y, world_yaw = self.world_start
                alignment_yaw = math.atan2(
                    math.sin(world_yaw - local_yaw), math.cos(world_yaw - local_yaw)
                )
                self.world_alignment = transformations.euler_matrix(0.0, 0.0, alignment_yaw)
                rotated = np.matmul(self.world_alignment, base_pose)
                self.world_alignment[0, 3] = world_x - rotated[0, 3]
                self.world_alignment[1, 3] = world_y - rotated[1, 3]
                self.world_alignment[2, 3] = self.base_height - rotated[2, 3]
                self._publish_static_chain(message.header.stamp)
            self.latest_base_pose = base_pose.copy()
            world_alignment = self.world_alignment.copy()

        output = Odometry()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.odom_frame
        output.child_frame_id = self.base_frame
        self._set_pose(output.pose.pose, base_pose)
        output.pose.covariance = list(message.pose.covariance)
        output.twist = message.twist
        self.odom_pub.publish(output)
        self._publish_base_tf(output)
        self._publish_world_pose(output, base_pose, world_alignment)
        self._publish_metric_pose(output, base_pose, world_alignment)

    def _loop_closure_callback(self, message):
        started = time.perf_counter()
        try:
            payload = json.loads(message.data)
            observed = payload["observed"]
            correction_values = payload["correction"]
            correction_x = float(correction_values[0])
            correction_y = float(correction_values[1])
            correction_yaw = float(correction_values[2])
            correction = math.hypot(correction_x, correction_y)
        except (KeyError, TypeError, ValueError):
            self.loop_closure_rejections += 1
            return
        with self.lock:
            latest = self.latest_base_pose
            request_is_current = latest is not None and math.hypot(
                latest[0, 3] - float(observed[0]),
                latest[1, 3] - float(observed[1]),
            ) <= 0.50
            accepted = (
                request_is_current
                and bool(payload.get("quality_accepted"))
                and correction <= self.max_loop_correction
                and abs(correction_yaw) <= self.max_loop_rotation
            )
            if accepted:
                closure = transformations.euler_matrix(0.0, 0.0, correction_yaw)
                closure[0, 3] = correction_x
                closure[1, 3] = correction_y
                self.local_alignment = np.matmul(closure, self.local_alignment)
                corrected = np.matmul(closure, latest)
                self.latest_base_pose = corrected
                self.loop_closure_count += 1
                self.loop_closure_max_translation = max(
                    self.loop_closure_max_translation, correction
                )
                alignment_message = self._matrix_transform(
                    self.local_alignment,
                    rospy.Time.now(),
                    self.map_frame,
                    "camera_init",
                )
                self.alignment_pub.publish(alignment_message)
                world_closure = np.matmul(
                    np.matmul(self.world_alignment, closure),
                    np.linalg.inv(self.world_alignment),
                )
            else:
                corrected = latest
                world_closure = np.identity(4)
                self.loop_closure_rejections += 1
            corrected_world = (
                np.matmul(self.world_alignment, corrected)
                if accepted and corrected is not None
                else None
            )
        if corrected_world is not None:
            corrected_source = (
                float(corrected_world[0, 3]),
                float(corrected_world[1, 3]),
            )
            with self.metric_lock:
                self.previous_metric_source = corrected_source
                self.latest_metric_source = corrected_source
        elapsed_ms = (
            (time.perf_counter() - started) * 1000.0
            + float(payload.get("matcher_processing_ms", 0.0))
        )
        self.loop_closure_total_ms += elapsed_ms
        event = {
            "accepted": accepted,
            "candidate_id": payload.get("candidate_id"),
            "constraint": payload.get("constraint"),
            "translation": correction,
            "rotation": abs(correction_yaw),
            "quality": payload.get("quality", {}),
            "preserve_danger_ids": payload.get("preserve_danger_ids", []),
            "processing_ms": elapsed_ms,
            "count": self.loop_closure_count,
            "rejections": self.loop_closure_rejections,
            "total_processing_ms": self.loop_closure_total_ms,
            "max_translation": self.loop_closure_max_translation,
            "local_translation": [correction_x, correction_y],
            "world_transform": [
                float(world_closure[0, 0]),
                float(world_closure[0, 1]),
                float(world_closure[0, 3]),
                float(world_closure[1, 0]),
                float(world_closure[1, 1]),
                float(world_closure[1, 3]),
                0.0,
                0.0,
                1.0,
            ],
        }
        if corrected is not None:
            event["corrected_pose"] = [
                float(corrected[0, 3]),
                float(corrected[1, 3]),
                yaw_from_matrix(corrected),
            ]
        self.loop_closure_pub.publish(
            String(data=json.dumps(event, sort_keys=True))
        )

    @staticmethod
    def _set_pose(pose, matrix):
        quaternion = transformations.quaternion_from_matrix(matrix)
        pose.position.x = matrix[0, 3]
        pose.position.y = matrix[1, 3]
        pose.position.z = matrix[2, 3]
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

    def _publish_static_chain(self, stamp):
        world_to_map = self._matrix_transform(
            self.world_alignment, stamp, self.world_frame, self.map_frame
        )
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = self.map_frame
        map_to_odom.child_frame_id = self.odom_frame
        map_to_odom.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform([world_to_map, map_to_odom])

    @staticmethod
    def _matrix_transform(matrix, stamp, parent, child):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = matrix[0, 3]
        transform.transform.translation.y = matrix[1, 3]
        transform.transform.translation.z = matrix[2, 3]
        quaternion = transformations.quaternion_from_matrix(matrix)
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        return transform

    def _publish_base_tf(self, odometry):
        transform = TransformStamped()
        transform.header = odometry.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _publish_world_pose(self, odometry, base_pose, world_alignment):
        pose = PoseStamped()
        pose.header.stamp = odometry.header.stamp
        pose.header.frame_id = self.world_frame
        self._set_pose(pose.pose, np.matmul(world_alignment, base_pose))
        self.world_pose_pub.publish(pose)

    def _publish_metric_pose(self, odometry, base_pose, world_alignment):
        world_pose = np.matmul(world_alignment, base_pose)
        world_yaw = yaw_from_matrix(world_pose)
        with self.metric_lock:
            current_xy = (float(world_pose[0, 3]), float(world_pose[1, 3]))
            observed_translation = None
            if self.previous_metric_source is not None:
                observed_translation = math.hypot(
                    current_xy[0] - self.previous_metric_source[0],
                    current_xy[1] - self.previous_metric_source[1],
                )
            self.previous_metric_source = current_xy
            if self.trust_command_motion:
                metric_x, metric_y = self.metric_pose.update(
                    odometry.header.stamp.to_sec(),
                    world_yaw,
                    observed_translation,
                    trust_command_motion=True,
                )
            else:
                # In the lobby, corridor and pre-door phases FAST-LIO has
                # abundant wall structure and is the authoritative metric
                # pose.  Pure command integration underestimates the A1 gait
                # and previously put camera coverage more than a metre behind
                # the RobotModel in RViz.
                metric_x, metric_y = self.metric_pose.synchronize(
                    current_xy[0],
                    current_xy[1],
                    odometry.header.stamp.to_sec(),
                    world_yaw,
                )
            self.latest_metric_source = current_xy
            self.latest_metric_output = (metric_x, metric_y)
        pose = PoseStamped()
        pose.header = odometry.header
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = metric_x
        pose.pose.position.y = metric_y
        pose.pose.position.z = world_pose[2, 3]
        quaternion = transformations.quaternion_from_matrix(world_pose)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        self.metric_pose_pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("localization_bridge")
    LioLocalizationBridge()
    rospy.spin()
