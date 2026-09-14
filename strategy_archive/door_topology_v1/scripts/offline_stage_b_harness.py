#!/usr/bin/env python3
"""Accelerated ROS integration harness for Stage B without Gazebo."""

import json
import math
from pathlib import Path
import time

import actionlib
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, TransformStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, String
import tf.transformations as transformations
import tf2_ros


class OfflineStageBHarness:
    def __init__(self):
        self.sim_time = 0.0
        self.pose = [0.0, 1.0, math.pi / 2.0]
        self.command = Twist()
        self.floor_complete = False
        self.danger_count = 0
        self.current_state = ""
        self.danger_target = None
        self.gate_received = False
        self.status_counts = {}
        self.expected_rooms_per_floor = max(
            1, int(rospy.get_param("~expected_rooms_per_floor", 4))
        )
        self.sim_timeout = float(rospy.get_param("~sim_timeout", 240.0))
        self.result_file = Path(rospy.get_param("~result_file", "/tmp/simnav_stage_b_result.json"))
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=2)
        self.map_pub = rospy.Publisher("/exploration_map", OccupancyGrid, queue_size=1, latch=True)
        self.odom_pub = rospy.Publisher("/simnav/odom", Odometry, queue_size=10)
        self.metric_pose_pub = rospy.Publisher(
            "/simnav/world_pose_metric", PoseStamped, queue_size=10
        )
        self.info_pub = rospy.Publisher("/offline/camera_info", CameraInfo, queue_size=1, latch=True)
        self.rgb_pub = rospy.Publisher("/offline/rgb", self._image_type(), queue_size=2)
        self.depth_pub = rospy.Publisher("/offline/depth", self._image_type(), queue_size=2)
        self.defer_pub = rospy.Publisher("/simnav/defer_zone", PolygonStamped, queue_size=1, latch=True)
        rospy.Subscriber("/cmd_vel", Twist, self._command_callback, queue_size=1)
        rospy.Subscriber("/simnav/floor_complete", Bool, self._complete_callback, queue_size=1)
        rospy.Subscriber("/simnav/danger_tracks", String, self._danger_callback, queue_size=1)
        rospy.Subscriber("/simnav/entrance_gate", PolygonStamped, self._gate_callback, queue_size=1)
        rospy.Subscriber("/simnav/explorer_status", String, self._status_callback, queue_size=10)
        self.move_server = actionlib.SimpleActionServer(
            "move_base", MoveBaseAction, execute_cb=self._move_callback, auto_start=False
        )
        self.move_server.start()
        self.bridge = CvBridge()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.map_message = self._build_map()
        self.info_message = self._build_camera_info()
        self.rgb_image, self.depth_image = self._build_rgbd()

    @staticmethod
    def _image_type():
        from sensor_msgs.msg import Image

        return Image

    @staticmethod
    def _build_map():
        resolution = 0.05
        width, height = 280, 440
        origin_x, origin_y = -7.0, -1.0
        data = np.zeros((height, width), dtype=np.int8)

        def wall_x(x, gaps):
            column = int(round((x - origin_x) / resolution))
            data[:, column - 1 : column + 2] = 100
            for center, gap_width in gaps:
                row_min = int((center - gap_width / 2.0 - origin_y) / resolution)
                row_max = int((center + gap_width / 2.0 - origin_y) / resolution)
                data[row_min:row_max, column - 1 : column + 2] = 0

        # Four always-open room doorways plus one deferred elevator doorway.
        wall_x(-1.2, [(4.0, 1.2), (12.0, 1.2)])
        wall_x(1.2, [(8.0, 1.2), (16.0, 1.2), (19.0, 1.2)])
        message = OccupancyGrid()
        message.header.frame_id = "simnav_map"
        message.info.resolution = resolution
        message.info.width = width
        message.info.height = height
        message.info.origin.position.x = origin_x
        message.info.origin.position.y = origin_y
        message.info.origin.orientation.w = 1.0
        message.data = data.reshape(-1).tolist()
        return message

    @staticmethod
    def _build_camera_info():
        message = CameraInfo()
        message.header.frame_id = "camera"
        message.width = 320
        message.height = 240
        message.K = [400.0, 0.0, 160.0, 0.0, 400.0, 120.0, 0.0, 0.0, 1.0]
        return message

    @staticmethod
    def _build_rgbd():
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        depth = np.full((240, 320), 4.0, dtype=np.float32)
        cv2.rectangle(image, (35, 75), (95, 135), (0, 0, 255), -1)
        cv2.circle(image, (255, 165), 20, (0, 255, 0), -1)
        return image, depth

    def _publish_static_transform(self):
        transforms = []
        for parent, child in (
            ("world", "simnav_map"),
            ("simnav_map", "simnav_odom"),
            ("base", "camera"),
        ):
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.static_broadcaster.sendTransform(transforms)

    def _publish_defer_zone(self):
        message = PolygonStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "simnav_map"
        for x, y in ((0.7, 18.2), (1.8, 18.2), (1.8, 19.8), (0.7, 19.8)):
            message.polygon.points.append(Point32(x=x, y=y))
        self.defer_pub.publish(message)

    def _command_callback(self, message):
        self.command = message

    def _complete_callback(self, message):
        self.floor_complete = message.data

    def _danger_callback(self, message):
        self.danger_count = len(json.loads(message.data).get("dangers", []))

    def _gate_callback(self, _message):
        self.gate_received = True

    def _status_callback(self, message):
        payload = json.loads(message.data)
        self.current_state = payload.get("state", "")
        self.status_counts = payload.get("counts", {})
        if self.danger_target is None and self.current_state == "ROOM_SCAN":
            # Put the synthetic target near the image centre.  This harness's
            # camera model maps camera-x to image horizontal position and
            # camera-y to image vertical position; use a small lateral offset
            # instead of the prior forward offset that projected out of frame.
            self.danger_target = (
                self.pose[0] + 0.2 * math.cos(self.pose[2]),
                self.pose[1] + 0.2 * math.sin(self.pose[2]),
                0.8,
            )
        elif self.current_state == "FLOOR_COMPLETE" and self.danger_target is None:
            self.danger_target = (self.pose[0], self.pose[1], 0.8)

    def _move_callback(self, goal):
        target = goal.target_pose.pose
        self.pose[0] = target.position.x
        self.pose[1] = target.position.y
        self.pose[2] = math.atan2(
            2.0 * target.orientation.w * target.orientation.z,
            1.0 - 2.0 * target.orientation.z * target.orientation.z,
        )
        self.move_server.set_succeeded(MoveBaseResult())

    def _integrate(self, step):
        yaw = self.pose[2]
        self.pose[0] += math.cos(yaw) * self.command.linear.x * step
        self.pose[1] += math.sin(yaw) * self.command.linear.x * step
        self.pose[2] = math.atan2(
            math.sin(yaw + self.command.angular.z * step),
            math.cos(yaw + self.command.angular.z * step),
        )

    def _publish_odom(self, stamp):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "simnav_odom"
        message.child_frame_id = "base"
        message.pose.pose.position.x = self.pose[0]
        message.pose.pose.position.y = self.pose[1]
        quaternion = transformations.quaternion_from_euler(0.0, 0.0, self.pose[2])
        message.pose.pose.orientation.x = quaternion[0]
        message.pose.pose.orientation.y = quaternion[1]
        message.pose.pose.orientation.z = quaternion[2]
        message.pose.pose.orientation.w = quaternion[3]
        self.odom_pub.publish(message)

        metric = PoseStamped()
        metric.header.stamp = stamp
        metric.header.frame_id = "world"
        metric.pose = message.pose.pose
        self.metric_pose_pub.publish(metric)

    def _publish_rgbd(self, stamp):
        image = self.rgb_image.copy()
        depth_values = self.depth_image.copy()
        if self.danger_target is not None:
            dx = self.danger_target[0] - self.pose[0]
            dy = self.danger_target[1] - self.pose[1]
            yaw = self.pose[2]
            camera_x = math.cos(yaw) * dx + math.sin(yaw) * dy
            camera_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            depth_value = self.danger_target[2]
            center_x = int(round(160.0 + 400.0 * camera_x / depth_value))
            center_y = int(round(120.0 + 400.0 * camera_y / depth_value))
            if 24 <= center_x < 296 and 24 <= center_y < 216:
                cv2.circle(image, (center_x, center_y), 24, (0, 0, 255), -1)
                cv2.circle(depth_values, (center_x, center_y), 24, depth_value, -1)
        rgb = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        depth = self.bridge.cv2_to_imgmsg(depth_values, encoding="32FC1")
        rgb.header.stamp = depth.header.stamp = stamp
        # The offline camera has an identity extrinsic; publish it in base
        # coordinates so the harness does not depend on a live TF buffer.
        rgb.header.frame_id = depth.header.frame_id = "base"
        self.rgb_pub.publish(rgb)
        self.depth_pub.publish(depth)

    def run(self):
        step = 0.05
        last_sensor_time = -1.0
        while not rospy.is_shutdown() and self.sim_time < self.sim_timeout:
            self.sim_time += step
            self._integrate(step)
            stamp = rospy.Time.from_sec(self.sim_time)
            self.clock_pub.publish(Clock(clock=stamp))
            self.map_message.header.stamp = stamp
            self.info_message.header.stamp = stamp
            self.map_pub.publish(self.map_message)
            self.info_pub.publish(self.info_message)
            self._publish_odom(stamp)
            if self.sim_time < 0.5:
                self._publish_static_transform()
                self._publish_defer_zone()
            if (
                self.current_state in ("ROOM_SCAN", "FLOOR_COMPLETE")
                and self.sim_time - last_sensor_time >= 0.20
            ):
                self._publish_rgbd(stamp)
                last_sensor_time = self.sim_time
            if self.floor_complete and self.danger_count == 1:
                break
            time.sleep(0.005)
        visited_count = int(self.status_counts.get("VISITED", 0))
        result = {
            "passed": (
                self.floor_complete
                and visited_count == self.expected_rooms_per_floor
                and self.danger_count == 1
                and self.gate_received
            ),
            "sim_time": round(self.sim_time, 3),
            "floor_complete": self.floor_complete,
            "danger_count": self.danger_count,
            "gate_received": self.gate_received,
            "visited_room_count": visited_count,
            "expected_rooms_per_floor": self.expected_rooms_per_floor,
            "status_counts": self.status_counts,
        }
        self.result_file.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        rospy.loginfo("OFFLINE_STAGE_B_RESULT %s", json.dumps(result, sort_keys=True))
        rospy.signal_shutdown("offline Stage B integration finished")
        if not result["passed"]:
            raise SystemExit("Stage B offline integration did not pass")


if __name__ == "__main__":
    rospy.init_node("offline_stage_b_harness")
    OfflineStageBHarness().run()
