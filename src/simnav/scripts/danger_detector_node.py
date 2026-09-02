#!/usr/bin/env python3
"""ROS RGB-D red-ball detector with world-frame confirmation and deduplication."""

import json
from collections import deque
from pathlib import Path
import sys
import threading
import math

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray, PoseStamped
import message_filters
from nav_msgs.msg import Odometry
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
import tf2_geometry_msgs
import tf2_ros
import tf.transformations as transformations
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(rospkg.RosPack().get_path("simnav")) / "scripts"))

from danger_detector_core import (  # noqa: E402
    CameraIntrinsics,
    DangerTracker,
    project_pose_from_anchor,
    RedBallDetector,
    ResultWriter,
    position_on_floor,
    rasterize_planar_ray,
)


class DangerDetectorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.detector = RedBallDetector(
            min_area=float(rospy.get_param("~min_area", 80.0)),
            min_circularity=float(rospy.get_param("~min_circularity", 0.82)),
        )
        self.tracker = DangerTracker(
            confirmation_frames=int(rospy.get_param("~confirmation_frames", 3)),
            cluster_radius=float(rospy.get_param("~cluster_radius", 0.75)),
        )
        self.output_file = Path(
            rospy.get_param(
                "~output_file", "/workspace/SimEnv/results/detected_danger.json"
            )
        )
        self.debug_output_file = Path(
            rospy.get_param(
                "~debug_output_file",
                "/workspace/SimEnv/results/detected_danger_debug.json",
            )
        )
        self.result_writer = ResultWriter(self.output_file, self.debug_output_file)
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.current_floor_z = float(rospy.get_param("~current_floor_z", 0.0))
        self.floor_min_offset = float(rospy.get_param("~floor_min_offset", -0.2))
        self.floor_max_offset = float(rospy.get_param("~floor_max_offset", 1.2))
        self.moving_frequency = float(rospy.get_param("~moving_frequency", 5.0))
        self.scan_frequency = float(rospy.get_param("~scan_frequency", 15.0))
        self.last_process_time = rospy.Time(0)
        self.room_scanning = False
        self.observation_phase = ""
        self.floor_complete = False
        self.mission_fault = False
        self.mission_fault_reason = None
        self.room_entry_pose = None
        self.room_entry_source_pose = None
        self.source_pose = None
        self.source_pose_stamp = rospy.Time(0)
        self.localization_health = "STALE"
        self.start_time = None
        self.pending_timeout = float(rospy.get_param("~pending_timeout", 1.0))
        self.camera_info = None
        self.metric_pose = None
        self.loop_merge_track_ids = set()
        self.loop_merge_until = rospy.Time(0)
        self.lock = threading.Lock()
        self.result_lock = threading.Lock()
        self.camera_observation_enabled = bool(
            rospy.get_param("~camera_observation_markers_enabled", True)
        )
        self.camera_observation_resolution = max(
            0.05, float(rospy.get_param("~camera_observation_resolution", 0.25))
        )
        self.camera_observation_pixel_stride = max(
            8, int(rospy.get_param("~camera_observation_pixel_stride", 32))
        )
        self.camera_observation_max_cells = max(
            100, int(rospy.get_param("~camera_observation_max_cells", 16000))
        )
        self.camera_observed_cells = set()
        self.camera_observed_order = deque()
        self.camera_observed_ray_count = 0
        # Keep two camera maps.  The active-scope map is reset when crossing a
        # real doorway and is the only map used for room completion.  The
        # global map is never reset during a floor run, so RViz continues to
        # show everything viewed from the corridor and previously visited
        # rooms.
        self.camera_global_observed_cells = set()
        self.camera_global_observed_order = deque()
        self.camera_global_observed_ray_count = 0
        self.camera_room_id = None
        self.camera_room_active = False
        self.camera_scope_type = None
        self.camera_exploration_active = False
        self.camera_observation_summary_period = max(
            0.10, float(rospy.get_param("~camera_observation_summary_period", 0.50))
        )
        self.camera_observation_last_summary = rospy.Time(0)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pose_pub = rospy.Publisher("/simnav/danger_poses", PoseArray, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/simnav/danger_markers", MarkerArray, queue_size=1, latch=True)
        self.debug_marker_pub = rospy.Publisher(
            "/simnav/danger_debug_markers", MarkerArray, queue_size=1, latch=True
        )
        self.camera_observation_pub = rospy.Publisher(
            "/simnav/camera_observed_markers", MarkerArray, queue_size=1, latch=True
        )
        self.camera_coverage_pub = rospy.Publisher(
            "/simnav/camera_coverage", String, queue_size=1, latch=True
        )
        self.track_pub = rospy.Publisher("/simnav/danger_tracks", String, queue_size=1, latch=True)
        self.confirmation_pub = rospy.Publisher(
            "/simnav/danger_confirmation_active", Bool, queue_size=1, latch=True
        )
        self.lifecycle_pub = rospy.Publisher(
            "/simnav/danger_detection_lifecycle", String, queue_size=10
        )
        self.valid_frame_pub = rospy.Publisher(
            "/simnav/danger_valid_frame", String, queue_size=10
        )
        rospy.Subscriber(
            rospy.get_param("~camera_info_topic", "/real_sense/rgb/camera_info"),
            CameraInfo,
            self._info_callback,
            queue_size=1,
        )
        rospy.Subscriber("/simnav/explorer_status", String, self._status_callback, queue_size=5)
        rospy.Subscriber(
            "/simnav/localization_health", String, self._health_callback, queue_size=5
        )
        rospy.Subscriber(
            "/simnav/world_pose_metric",
            PoseStamped,
            self._metric_pose_callback,
            queue_size=20,
        )
        rospy.Subscriber("/simnav/odom", Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber(
            "/simnav/floor_complete", Bool, self._complete_callback, queue_size=1
        )
        rospy.Subscriber(
            "/simnav/floor_exploration_context", String,
            self._floor_context_callback, queue_size=1,
        )
        rospy.Subscriber(
            "/simnav/mission_fault", String, self._fault_callback, queue_size=2
        )
        rospy.Subscriber(
            "/simnav/room_entry", String, self._room_entry_callback, queue_size=5
        )
        rospy.Subscriber(
            "/simnav/local_loop_closure_applied",
            String,
            self._loop_closure_callback,
            queue_size=5,
        )
        rgb_sub = message_filters.Subscriber(
            rospy.get_param("~rgb_topic", "/real_sense/rgb/image_raw"), Image
        )
        depth_sub = message_filters.Subscriber(
            rospy.get_param("~depth_topic", "/real_sense/depth/image_raw"), Image
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=8, slop=0.08
        )
        self.synchronizer.registerCallback(self._image_callback)
        self.result_timer = rospy.Timer(rospy.Duration(0.5), self._result_timer)

    def _info_callback(self, message):
        with self.lock:
            self.camera_info = CameraIntrinsics(
                fx=float(message.K[0]),
                fy=float(message.K[4]),
                cx=float(message.K[2]),
                cy=float(message.K[5]),
            )

    def _status_callback(self, message):
        try:
            payload = json.loads(message.data)
            state = payload.get("state")
            corridor_ready = bool(payload.get("camera_exploration_active", False))
            topology_lock = payload.get("topology_lock")
            self.room_scanning = state in (
                "DOOR_CROSSING",
                "ROOM_SCAN",
                "EXIT_ROOM",
            )
            with self.lock:
                scope_type = self.camera_scope_type
            if corridor_ready and not self.camera_exploration_active:
                self._activate_camera_scope(
                    str(topology_lock or "room_pending"), "room"
                )
            elif (
                corridor_ready
                and scope_type == "room"
                and topology_lock
                and self.camera_room_id != str(topology_lock)
            ):
                self._activate_camera_scope(str(topology_lock), "room")
            elif not corridor_ready and self.camera_exploration_active:
                # The sensor remains published by Gazebo, but image conversion,
                # red detection and coverage ray casting are disabled while the
                # robot is only using the corridor as a transport topology.
                with self.lock:
                    self.camera_exploration_active = False
                    self.camera_room_active = False
                    self.camera_room_id = None
                    self.camera_scope_type = None
            if state == "ROOM_SCAN":
                self.observation_phase = payload.get("room_scan_phase") or "ROOM_FRONTIER_EXPLORE"
            elif state == "DOOR_CROSSING":
                self.observation_phase = "INGRESS_SCAN"
            elif state == "EXIT_ROOM":
                self.observation_phase = "EGRESS_SCAN"
            else:
                self.observation_phase = ""
        except (TypeError, ValueError):
            self.room_scanning = False
            self.observation_phase = ""

    def _activate_camera_scope(self, scope_id, scope_type, reset_global=False):
        """Enable RGB-D exploration and start a fresh local topology scope."""
        with self.lock:
            unchanged = (
                self.camera_exploration_active
                and self.camera_room_id == scope_id
                and self.camera_scope_type == scope_type
            )
            self.camera_exploration_active = True
            self.camera_room_id = scope_id
            self.camera_scope_type = scope_type
            self.camera_room_active = True
            self.camera_observation_last_summary = rospy.Time(0)
        if unchanged and not reset_global:
            return
        with self.result_lock:
            self.camera_observed_cells.clear()
            self.camera_observed_order.clear()
            self.camera_observed_ray_count = 0
            if reset_global:
                self.camera_global_observed_cells.clear()
                self.camera_global_observed_order.clear()
                self.camera_global_observed_ray_count = 0
        rospy.loginfo(
            "RGB-D exploration scope active: type=%s id=%s",
            scope_type,
            scope_id,
        )
        self._publish_camera_observation_markers(force_summary=True)

    def _health_callback(self, message):
        try:
            self.localization_health = json.loads(message.data).get("state", "STALE")
        except (TypeError, ValueError):
            self.localization_health = "STALE"

    def _metric_pose_callback(self, message):
        with self.result_lock:
            self.metric_pose = message

    def _odom_callback(self, message):
        orientation = message.pose.pose.orientation
        source_yaw = transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )[2]
        source_pose = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(source_yaw),
        )
        with self.lock:
            self.source_pose = source_pose
            self.source_pose_stamp = message.header.stamp

    def _complete_callback(self, message):
        self.floor_complete = bool(message.data)
        self._write_results(rospy.Time.now())

    def _floor_context_callback(self, message):
        try:
            payload = json.loads(message.data)
            floor_index = int(payload["floor_index"])
            floor_z = float(payload["floor_z"])
        except (TypeError, ValueError, KeyError):
            return
        if floor_index <= 0:
            return
        with self.lock:
            self.current_floor_z = floor_z
            self.floor_complete = False
            self.room_scanning = False
            self.camera_room_id = None
            self.camera_room_active = False
            self.camera_scope_type = None
            self.camera_exploration_active = False
            self.camera_observed_cells.clear()
            self.camera_observed_order.clear()
            self.camera_global_observed_cells.clear()
            self.camera_global_observed_order.clear()
        rospy.loginfo("Danger detector switched to floor %d at z=%.3f", floor_index, floor_z)

    def _fault_callback(self, message):
        try:
            payload = json.loads(message.data)
            self.mission_fault_reason = payload.get("reason")
        except (TypeError, ValueError):
            self.mission_fault_reason = message.data
        self.mission_fault = True
        rospy.logwarn("Danger detector frozen by Stage B mission fault: %s", self.mission_fault_reason)

    def _room_entry_callback(self, message):
        try:
            payload = json.loads(message.data)
            pose = payload.get("pose")
            candidate_id = str(payload.get("candidate_id") or "")
            if pose and len(pose) >= 3 and candidate_id:
                with self.lock:
                    self.room_entry_pose = tuple(float(value) for value in pose[:3])
                    self.room_entry_source_pose = self.source_pose
                self._activate_camera_scope(candidate_id, "room")
        except (TypeError, ValueError):
            return

    def _loop_closure_callback(self, message):
        try:
            payload = json.loads(message.data)
            if not payload.get("accepted"):
                return
            transform = np.asarray(payload["world_transform"], dtype=np.float64).reshape((3, 3))
            preserved_track_ids = payload.get("preserve_danger_ids", [])
        except (KeyError, TypeError, ValueError):
            return
        with self.result_lock:
            self.tracker.transform_except(transform, preserved_track_ids)
            if preserved_track_ids:
                self.loop_merge_track_ids = {int(value) for value in preserved_track_ids}
                self.loop_merge_until = rospy.Time.now() + rospy.Duration(
                    float(rospy.get_param("~loop_merge_duration", 5.0))
                )
        with self.lock:
            if self.room_entry_pose is not None:
                anchor_x, anchor_y, anchor_yaw = self.room_entry_pose
                transformed_anchor = np.matmul(
                    transform,
                    [anchor_x, anchor_y, 1.0],
                )
                rotation_yaw = math.atan2(transform[1, 0], transform[0, 0])
                self.room_entry_pose = (
                    float(transformed_anchor[0]),
                    float(transformed_anchor[1]),
                    math.atan2(
                        math.sin(anchor_yaw + rotation_yaw),
                        math.cos(anchor_yaw + rotation_yaw),
                    ),
                )
        self._write_results(rospy.Time.now())

    def _tracking_pose(self, metric_pose, stamp):
        """Use short-term odometry relative to the room-entry reference frame."""
        orientation = metric_pose.pose.orientation
        metric_roll, metric_pitch, metric_yaw = transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        fallback = (
            float(metric_pose.pose.position.x),
            float(metric_pose.pose.position.y),
            float(metric_yaw),
            float(metric_roll),
            float(metric_pitch),
        )
        with self.lock:
            room_scanning = self.room_scanning
            room_entry_pose = self.room_entry_pose
            room_entry_source_pose = self.room_entry_source_pose
            source_pose = self.source_pose
            source_stamp = self.source_pose_stamp
        if (
            not room_scanning
            or room_entry_pose is None
            or room_entry_source_pose is None
            or source_pose is None
        ):
            return fallback
        if stamp != rospy.Time(0) and source_stamp != rospy.Time(0):
            if abs((stamp - source_stamp).to_sec()) > 0.5:
                return fallback
        projected = project_pose_from_anchor(
            source_pose,
            room_entry_source_pose,
            room_entry_pose,
        )
        return projected + (metric_roll, metric_pitch)

    def _image_callback(self, rgb_message, depth_message):
        if self.mission_fault:
            return
        # The Gazebo sensor itself is intentionally always-on.  Expensive
        # conversion, red-object inference and coverage ray casting begin only
        # after the virtual entrance gate establishes the corridor topology.
        with self.lock:
            camera_exploration_active = self.camera_exploration_active
        if not camera_exploration_active:
            return
        frequency = self.scan_frequency if self.room_scanning else self.moving_frequency
        stamp = rgb_message.header.stamp if rgb_message.header.stamp != rospy.Time() else rospy.Time.now()
        if self.last_process_time != rospy.Time(0) and (stamp - self.last_process_time).to_sec() < 1.0 / frequency:
            return
        with self.lock:
            intrinsics = self.camera_info
        with self.result_lock:
            metric_pose = self.metric_pose
        if intrinsics is None or metric_pose is None:
            return
        tracking_x, tracking_y, tracking_yaw, tracking_roll, tracking_pitch = self._tracking_pose(
            metric_pose, stamp
        )
        try:
            image = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
        except CvBridgeError as error:
            rospy.logwarn_throttle(5.0, "RGB-D conversion failed: %s", error)
            return
        depth = np.asarray(depth, dtype=np.float32)
        if depth_message.encoding in ("16UC1", "mono16"):
            depth *= 0.001
        metric_transform = transformations.euler_matrix(
            tracking_roll, tracking_pitch, tracking_yaw
        )
        metric_transform[:3, 3] = [
            tracking_x,
            tracking_y,
            metric_pose.pose.position.z,
        ]
        camera_frame = depth_message.header.frame_id or rgb_message.header.frame_id
        camera_to_base = None
        with self.lock:
            camera_room_active = self.camera_room_active
        if self.camera_observation_enabled and camera_room_active and camera_frame:
            try:
                camera_to_base = self.tf_buffer.lookup_transform(
                    "base", camera_frame, stamp, rospy.Duration(0.08)
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                camera_to_base = None
        if camera_to_base is not None:
            self._accumulate_camera_observation(
                depth, intrinsics, camera_to_base, metric_transform, stamp
            )
        observations = self.detector.detect(image, depth, intrinsics)
        positions = []
        tf_rejected = 0
        floor_rejected = 0
        for observation in observations:
            point = PointStamped()
            point.header.stamp = stamp
            point.header.frame_id = depth_message.header.frame_id or rgb_message.header.frame_id
            point.point.x, point.point.y, point.point.z = observation.position_camera
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base", point.header.frame_id, stamp, rospy.Duration(0.08)
                )
                base_point = tf2_geometry_msgs.do_transform_point(point, transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                tf_rejected += 1
                continue
            position = np.matmul(
                metric_transform,
                [base_point.point.x, base_point.point.y, base_point.point.z, 1.0],
            )[:3]
            if position_on_floor(
                position,
                self.current_floor_z,
                self.floor_min_offset,
                self.floor_max_offset,
            ):
                positions.append(position)
            else:
                floor_rejected += 1
        with self.result_lock:
            previous_ids = {track.track_id for track in self.tracker.tracks}
            previous_confirmed = {track.track_id for track in self.tracker.confirmed()}
            confirmed = self.tracker.update(
                positions, stamp.to_sec(), localization_health=self.localization_health
            )
            if self.loop_merge_track_ids and stamp <= self.loop_merge_until:
                self.tracker.merge_nearby_tracks(
                    self.loop_merge_track_ids,
                    max_distance=float(rospy.get_param("~loop_merge_distance", 1.0)),
                )
                confirmed = self.tracker.confirmed()
            current_ids = {track.track_id for track in self.tracker.tracks}
            current_confirmed = {track.track_id for track in confirmed}
            observation_count = {
                str(track.track_id): track.observations for track in self.tracker.tracks
            }
        stats = dict(self.detector.last_stats)
        lifecycle = {
            "timestamp": stamp.to_sec(),
            "red_mask_found": int(stats.get("red_mask_found", 0)),
            "contour_pass": int(stats.get("contour_pass", 0)),
            "depth_pass": int(stats.get("depth_pass", 0)),
            "tf_pass": len(positions) + floor_rejected,
            "tf_reject": tf_rejected,
            "floor_pass": len(positions),
            "floor_reject": floor_rejected,
            "track_created": sorted(current_ids - previous_ids),
            "track_associated": sorted(current_ids & previous_ids),
            "observation_count": observation_count,
            "confirmed": sorted(current_confirmed),
            "newly_confirmed": sorted(current_confirmed - previous_confirmed),
            "localization_health": self.localization_health,
            "mission_fault": self.mission_fault,
            "observation_phase": self.observation_phase,
        }
        lifecycle["stage_events"] = [
            "RED_MASK_FOUND" if lifecycle["red_mask_found"] else "RED_MASK_EMPTY",
            "CONTOUR_PASS" if lifecycle["contour_pass"] else "CONTOUR_REJECT",
            "DEPTH_PASS" if lifecycle["depth_pass"] else "DEPTH_REJECT",
            "TF_PASS" if lifecycle["tf_pass"] else "TF_REJECT",
            "TRACK_CREATED" if lifecycle["track_created"] else "TRACK_ASSOCIATED",
            "CONFIRMED" if lifecycle["newly_confirmed"] else "UNCONFIRMED",
            "LOCAL_LOOP_CORRECTION" if self.loop_merge_track_ids else "NO_LOOP_CORRECTION",
            "FINAL_OUTPUT",
        ]
        self.lifecycle_pub.publish(
            String(data=json.dumps(lifecycle, sort_keys=True, allow_nan=False))
        )
        self.valid_frame_pub.publish(
            String(
                data=json.dumps(
                    {
                        "timestamp": stamp.to_sec(),
                        "observation_phase": self.observation_phase,
                        "rgb_valid": True,
                        "depth_valid": bool(np.any(np.isfinite(depth) & (depth > 0.0))),
                        "tf_valid": bool(tf_rejected == 0 or positions or not observations),
                    },
                    sort_keys=True,
                )
            )
        )
        for track_id in sorted(current_ids - previous_ids):
            rospy.loginfo("Danger lifecycle TRACK_CREATED id=%s", track_id)
        for track_id in sorted(current_confirmed - previous_confirmed):
            rospy.loginfo("Danger lifecycle CONFIRMED id=%s", track_id)
        self._publish_debug_markers(stamp)
        self.last_process_time = stamp
        self._ensure_start_time(stamp)
        if positions or confirmed:
            self._publish(confirmed, stamp)
        self._write_results(stamp)

    @staticmethod
    def _transform_point(transform, point_camera):
        point = PointStamped()
        point.point.x, point.point.y, point.point.z = point_camera
        point.header.stamp = transform.header.stamp
        point.header.frame_id = transform.child_frame_id
        transformed = tf2_geometry_msgs.do_transform_point(point, transform)
        return np.asarray(
            [
                transformed.point.x,
                transformed.point.y,
                transformed.point.z,
            ],
            dtype=np.float64,
        )

    def _accumulate_camera_observation(
        self, depth, intrinsics, camera_to_base, metric_transform, stamp
    ):
        """Accumulate only RGB-D line-of-sight cells for RViz diagnostics."""
        del stamp
        stride = self.camera_observation_pixel_stride
        height, width = depth.shape[:2]
        origin_base = self._transform_point(camera_to_base, (0.0, 0.0, 0.0))
        origin_world = np.matmul(metric_transform, np.r_[origin_base, 1.0])[:3]
        observed = set()
        for pixel_y in range(stride // 2, height, stride):
            for pixel_x in range(stride // 2, width, stride):
                distance = float(depth[pixel_y, pixel_x])
                if not math.isfinite(distance) or distance <= 0.15 or distance >= 12.0:
                    continue
                camera_point = (
                    (float(pixel_x) - intrinsics.cx) * distance / intrinsics.fx,
                    (float(pixel_y) - intrinsics.cy) * distance / intrinsics.fy,
                    distance,
                )
                base_point = self._transform_point(camera_to_base, camera_point)
                endpoint_world = np.matmul(metric_transform, np.r_[base_point, 1.0])[:3]
                for cell in rasterize_planar_ray(
                    origin_world[:2], endpoint_world[:2], self.camera_observation_resolution
                ):
                    observed.add(cell)
        if not observed:
            return
        with self.result_lock:
            for cell in sorted(observed):
                if cell in self.camera_observed_cells:
                    pass
                else:
                    self.camera_observed_cells.add(cell)
                    self.camera_observed_order.append(cell)
                if cell not in self.camera_global_observed_cells:
                    self.camera_global_observed_cells.add(cell)
                    self.camera_global_observed_order.append(cell)
            while len(self.camera_observed_order) > self.camera_observation_max_cells:
                old_cell = self.camera_observed_order.popleft()
                self.camera_observed_cells.discard(old_cell)
            while len(self.camera_global_observed_order) > self.camera_observation_max_cells:
                old_cell = self.camera_global_observed_order.popleft()
                self.camera_global_observed_cells.discard(old_cell)
            self.camera_observed_ray_count += len(observed)
            self.camera_global_observed_ray_count += len(observed)
        self._publish_camera_observation_markers()

    def _publish_camera_observation_markers(self, force_summary=False):
        markers = MarkerArray()
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.world_frame
        marker.ns = "camera_observed_space"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.camera_observation_resolution
        marker.scale.y = self.camera_observation_resolution
        marker.scale.z = 0.025
        marker.color.r = 0.05
        marker.color.g = 0.85
        marker.color.b = 0.95
        marker.color.a = 0.22
        with self.result_lock:
            cells = tuple(self.camera_global_observed_cells)
        marker.points = [
            Point(
                (float(cell[0]) + 0.5) * self.camera_observation_resolution,
                (float(cell[1]) + 0.5) * self.camera_observation_resolution,
                0.035,
            )
            for cell in cells
        ]
        markers.markers.append(marker)
        self.camera_observation_pub.publish(markers)
        now = rospy.Time.now()
        with self.lock:
            room_id = self.camera_room_id
            last_summary = self.camera_observation_last_summary
        if room_id is None:
            return
        if not force_summary and last_summary != rospy.Time(0):
            if (now - last_summary).to_sec() < self.camera_observation_summary_period:
                return
        with self.result_lock:
            summary_cells = tuple(sorted(self.camera_observed_cells))
            ray_count = int(self.camera_observed_ray_count)
        self.camera_coverage_pub.publish(
            String(
                data=json.dumps(
                    {
                        "room_id": room_id,
                        "scope_type": self.camera_scope_type,
                        "resolution": self.camera_observation_resolution,
                        "cells": [[int(cell[0]), int(cell[1])] for cell in summary_cells],
                        "count": len(summary_cells),
                        "ray_count": ray_count,
                        "timestamp": now.to_sec(),
                    },
                    sort_keys=True,
                )
            )
        )
        with self.lock:
            self.camera_observation_last_summary = now

    def _publish_debug_markers(self, stamp):
        """Show short-lived red detections without changing official output."""
        markers = MarkerArray()
        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.world_frame
        clear.ns = "unconfirmed_red_targets"
        clear.id = 0
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        now = stamp.to_sec()
        with self.result_lock:
            tracks = tuple(self.tracker.tracks)
        for track in tracks:
            confirmed = track.observations >= self.tracker.confirmation_frames
            if confirmed or now - track.last_seen > self.pending_timeout:
                continue
            marker = Marker()
            marker.header = clear.header
            marker.ns = "unconfirmed_red_targets"
            marker.id = int(track.track_id)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(track.position_world[0])
            marker.pose.position.y = float(track.position_world[1])
            marker.pose.position.z = float(track.position_world[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.30 if confirmed else 0.22
            marker.color.r = 1.0
            marker.color.g = 0.05 if confirmed else 0.55
            marker.color.b = 0.02
            marker.color.a = 0.90 if confirmed else 0.55
            markers.markers.append(marker)
        self.debug_marker_pub.publish(markers)

    def _publish(self, tracks, stamp):
        poses = PoseArray()
        poses.header.stamp = stamp
        poses.header.frame_id = self.world_frame
        markers = MarkerArray()
        payload = []
        for track in tracks:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = track.position_world.tolist()
            pose.orientation.w = 1.0
            poses.poses.append(pose)
            marker = Marker()
            marker.header = poses.header
            marker.ns = "confirmed_dangers"
            marker.id = track.track_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = marker.scale.y = marker.scale.z = 0.30
            marker.color.r = 1.0
            marker.color.a = 0.9
            markers.markers.append(marker)
            item = {
                    "id": track.track_id,
                    "position_world": [round(float(value), 4) for value in track.position_world],
                    "observations": track.observations,
                    "confidence": round(track.confidence, 4),
                    "position_variance": [
                        round(float(value), 6) for value in track.position_variance
                    ],
                    "localization_health_counts": dict(
                        track.localization_health_counts
                    ),
                }
            with self.lock:
                room_entry_pose = self.room_entry_pose
            if room_entry_pose is not None:
                ax, ay, ayaw = room_entry_pose
                dx = float(track.position_world[0]) - ax
                dy = float(track.position_world[1]) - ay
                item["position_room"] = [
                    round(math.cos(ayaw) * dx + math.sin(ayaw) * dy, 4),
                    round(-math.sin(ayaw) * dx + math.cos(ayaw) * dy, 4),
                    round(float(track.position_world[2]), 4),
                ]
            payload.append(item)
        self.pose_pub.publish(poses)
        self.marker_pub.publish(markers)
        serialized = json.dumps(
            {"frame_id": self.world_frame, "dangers": payload}, indent=2, sort_keys=True
        ) + "\n"
        self.track_pub.publish(String(data=serialized))

    def _result_timer(self, event):
        del event
        self._write_results(rospy.Time.now())

    def _ensure_start_time(self, stamp):
        if self.start_time is None and stamp != rospy.Time(0):
            self.start_time = stamp

    def _write_results(self, stamp):
        self._ensure_start_time(stamp)
        exploration_time = 0.0
        if self.start_time is not None and stamp >= self.start_time:
            exploration_time = (stamp - self.start_time).to_sec()
        now = stamp.to_sec()
        with self.result_lock:
            confirmed = self.tracker.confirmed()
            self.result_writer.write(
                self.tracker.tracks,
                confirmed,
                exploration_time,
                frame_id=self.world_frame,
                floor_complete=self.floor_complete,
            )
            confirmation_active = self.room_scanning and any(
                track.observations < self.tracker.confirmation_frames
                and now - track.last_seen <= self.pending_timeout
                for track in self.tracker.tracks
            )
        self.confirmation_pub.publish(Bool(data=confirmation_active))


if __name__ == "__main__":
    rospy.init_node("danger_detector")
    DangerDetectorNode()
    rospy.spin()
