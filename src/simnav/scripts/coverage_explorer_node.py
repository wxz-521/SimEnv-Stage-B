#!/usr/bin/env python3
"""Direct corridor-and-room exploration using laser/camera joint coverage."""

import json
import math
import os
import sys
import threading
import traceback
from collections import deque

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf.transformations as transformations
from geometry_msgs.msg import Point, Point32, PolygonStamped, PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray

# Catkin's devel-space relay lives beside generated wrappers.  Put this
# source file's directory first so a stale relay cannot shadow pure modules.
SCRIPT_DIRECTORY = os.path.dirname(os.path.realpath(__file__))
if not sys.path or sys.path[0] != SCRIPT_DIRECTORY:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from coverage_explorer_core import (
    FrontierTarget,
    GridView,
    TaskCoveragePlanner,
    corrected_portal_heading,
    coverage_classification,
    detect_sphere_like_clusters,
    task_region_mask,
    infer_task_extent,
    detect_room_portals,
    detect_lobby_portals,
    normalize_angle,
    pair_room_portals,
    portal_return_along_offsets,
    topology_state_for_new_target,
    target_kind_allowed_for_topology_state,
    topology_completion_ready,
)


class CoverageExplorer:
    def __init__(self):
        self.lock = threading.RLock()
        self.grid = None
        self.navigation_grid = None
        self.pose = None
        self.world_pose = None
        self.last_pose_stamp = rospy.Time(0)
        self.front_clearance = float("inf")
        self.left_clearance = float("inf")
        self.right_clearance = float("inf")
        self.initial_forward_distance = max(
            0.0, float(rospy.get_param("~initial_forward_distance", 14.5))
        )
        self.initial_forward_speed = max(
            0.05, float(rospy.get_param("~initial_forward_speed", 0.60))
        )
        self.initial_centering_start_distance = max(
            0.0, float(rospy.get_param("~initial_centering_start_distance", 10.5))
        )
        # The robot may continue several metres into the corridor before
        # exploration starts, but the lobby/task topology gate belongs at the
        # observed corridor entrance, not at that final transit pose.
        self.virtual_gate_forward_distance = max(
            0.0,
            min(
                self.initial_forward_distance,
                float(
                    rospy.get_param(
                        "~virtual_gate_forward_distance",
                        self.initial_centering_start_distance,
                    )
                ),
            ),
        )
        self.configured_yaw = float(rospy.get_param("~corridor_yaw", 0.0))
        self.forward_anchor = None
        self.forward_yaw = None
        self.world_forward_yaw = None
        self.lobby_entry_source = None
        self.lobby_entry_world = None
        self.gate_source = None
        self.gate_world = None
        self.elevator_portal = None
        self.elevator_portal_world = None
        self.elevator_portal_evidence = 0
        self.topology_region = "LOBBY_TRANSIT"
        self.plane_policy_active = False
        self.policy_switch_stop_since = None
        self.policy_switch_service = rospy.ServiceProxy(
            "/unitree/select_plane_policy", SetBool, persistent=True
        )

        self.robot_radius = float(rospy.get_param("~robot_radius", 0.38))
        self.safety_margin = float(rospy.get_param("~safety_margin", 0.04))
        self.navigation_clearance = float(
            rospy.get_param("~navigation_clearance", 0.20)
        )
        self.preferred_clearance = float(
            rospy.get_param("~preferred_clearance", 0.32)
        )
        self.camera_weight = max(
            0.0, min(1.0, float(rospy.get_param("~camera_weight", 0.95)))
        )
        self.laser_coverage_target = max(
            0.0, min(1.0, float(rospy.get_param("~laser_coverage_target", 0.95)))
        )
        self.camera_coverage_target = max(
            0.0, min(1.0, float(rospy.get_param("~camera_coverage_target", 0.80)))
        )
        # Room locks use the same two-sensor semantics as the floor metric,
        # but over that room's map-derived local denominator.
        self.room_laser_coverage_target = max(
            0.0,
            min(1.0, float(rospy.get_param("~room_laser_coverage_target", self.laser_coverage_target))),
        )
        self.room_camera_coverage_target = max(
            0.0,
            min(1.0, float(rospy.get_param("~room_camera_coverage_target", self.camera_coverage_target))),
        )
        self.combined_coverage_target = max(
            0.0, min(1.0, float(rospy.get_param("~combined_coverage_target", 0.84)))
        )
        self.room_combined_coverage_target = max(
            0.0,
            min(1.0, float(rospy.get_param("~room_combined_coverage_target", 0.84))),
        )
        self.expected_rooms_per_floor = max(
            1, int(rospy.get_param("~expected_rooms_per_floor", 4))
        )
        self.completion_stable_duration = max(
            0.5, float(rospy.get_param("~completion_stable_duration", 3.0))
        )
        self.completion_since = None
        self.task_back_extension = float(rospy.get_param("~task_back_extension", 3.45))
        self.task_entry_buffer = max(
            0.0, float(rospy.get_param("~task_entry_buffer", 0.45))
        )

        self.planner = TaskCoveragePlanner(
            robot_radius=self.robot_radius,
            safety_margin=self.safety_margin,
            frontier_cluster_radius=float(rospy.get_param("~frontier_cluster_radius", 0.45)),
            revisit_radius=float(rospy.get_param("~frontier_revisit_radius", 0.70)),
            information_radius=float(rospy.get_param("~information_radius", 2.5)),
            camera_weight=self.camera_weight,
            back_extension=self.task_back_extension,
            forward_depth=float(rospy.get_param("~task_forward_depth", 24.6)),
            lateral_half_width=float(rospy.get_param("~task_lateral_half_width", 9.5)),
            corridor_half_width=float(rospy.get_param("~task_corridor_half_width", 1.1)),
            navigation_clearance=self.navigation_clearance,
            preferred_clearance=self.preferred_clearance,
            clearance_cost_weight=float(
                rospy.get_param("~clearance_cost_weight", 1.4)
            ),
            turn_cost_weight=float(rospy.get_param("~turn_cost_weight", 0.10)),
            far_room_first=bool(rospy.get_param("~far_room_first", False)),
            minimum_room_stations=int(
                rospy.get_param("~minimum_room_stations", 2)
            ),
            front_station_search_limit=float(
                rospy.get_param("~front_station_search_limit", 10.0)
            ),
            virtual_gate_half_width=float(
                rospy.get_param("~virtual_gate_half_width", 1.1)
            ),
            virtual_gate_depth=float(rospy.get_param("~virtual_gate_depth", 0.30)),
        )
        # The virtual entrance gate spans the full usable corridor width.  It
        # is a task/topology boundary only; the raw navigation map remains
        # unchanged, so the robot is not physically blocked by this marker.
        self.virtual_gate_half_width = self.planner.virtual_gate_half_width
        self.virtual_gate_depth = self.planner.virtual_gate_depth
        # Room-door markers are diagnostic only.  Keep the map-derived portal
        # detector and topology state machine active, but hide the coloured
        # door lines by default so they cannot be mistaken for obstacles or
        # coverage boundaries in RViz.  The entrance gate remains visible.
        self.show_room_virtual_doors = bool(
            rospy.get_param("~show_room_virtual_doors", False)
        )
        self.replan_period = max(0.25, float(rospy.get_param("~replan_period", 1.0)))
        self.target_tolerance = max(0.15, float(rospy.get_param("~target_tolerance", 0.35)))
        self.heading_tolerance = max(0.08, float(rospy.get_param("~heading_tolerance", 0.25)))
        self.motion_stop_distance = max(0.30, float(rospy.get_param("~motion_stop_distance", 0.48)))
        self.motion_speed = max(0.30, float(rospy.get_param("~motion_speed", 0.60)))
        self.turn_speed = max(0.15, float(rospy.get_param("~turn_speed", 0.65)))
        self.review_hold_duration = max(0.5, float(rospy.get_param("~sphere_review_hold", 2.0)))
        self.camera_resolution = max(0.05, float(rospy.get_param("~camera_resolution", 0.25)))

        self.camera_points_world = set()
        self.camera_points_order = deque()
        self.camera_points_memory_limit = max(
            1000, int(rospy.get_param("~camera_points_memory_limit", 120000))
        )
        self.camera_update = rospy.Time(0)
        self.current_plan = None
        self.active_target = None
        self.active_path = ()
        self.path_index = 0
        self.last_plan_time = rospy.Time(0)
        self.plan_cycles = 0
        self.plan_failures = 0
        self.last_plan_error = None
        self.last_plan_reason = "NOT_READY"
        self.visited_targets = []
        self.blocked_targets = []
        self.navigation_blocks = 0
        self.targets_reached = 0
        self.review_hold_since = None
        # Room dispatch state.  The planner derives IDs from observed doorway
        # coordinates; this node only remembers lifecycle and hysteresis.
        self.topology_lock = None
        self.completed_topologies = set()
        self.topology_states = {}
        self.topology_miss_cycles = {}
        self.returning_topology = None
        # The rear pair is deliberately inaccessible until both rooms at the
        # entrance-nearest station have completed and returned to CORRIDOR.
        self.rear_transit_distance = max(
            1.0, float(rospy.get_param("~rear_transit_distance", 12.0))
        )
        self.rear_transit_active = False
        self.rear_transit_start_along = None
        self.rear_rooms_unlocked = False
        self.front_station_along = None
        self.front_station_topologies = set()
        self.completed_front_sides = set()
        # Door search is deliberately stop-and-go.  A one-metre segment gives
        # lidar a new viewpoint, then the robot stops while temporal portal
        # evidence is rebuilt.  The short phase limit prevents this fallback
        # from driving past the front station toward the rear rooms.
        self.door_search_step_distance = max(
            0.25, float(rospy.get_param("~door_search_step_distance", 1.0))
        )
        self.door_search_front_limit = max(
            self.door_search_step_distance,
            float(rospy.get_param("~door_search_front_limit", 3.0)),
        )
        self.door_search_rear_limit = max(
            self.door_search_step_distance,
            float(rospy.get_param("~door_search_rear_limit", 3.0)),
        )
        self.door_search_wait_cycles = max(
            1, int(rospy.get_param("~door_search_wait_cycles", 3))
        )
        self.door_search_active = False
        self.door_search_step_start_along = None
        self.door_search_travel = 0.0
        self.door_search_idle_cycles = 0
        self.door_search_phase = "FRONT"
        self.room_scope_announced = None
        self.portal_evidence = {}
        self.portal_last_seen = {}
        # Preserve the exact confirmed doorway used to enter each topology.
        # A sparse scan near/inside the room must not erase the only safe exit.
        self.topology_portals = {}
        self.portal_confirm_cycles = max(
            2, int(rospy.get_param("~portal_confirm_cycles", 3))
        )
        self.active_target_started = rospy.Time(0)
        self.active_target_last_progress = None
        self.active_target_last_progress_stamp = rospy.Time(0)
        self.active_target_last_distance = None
        self.target_replacements = 0
        self.target_replace_min_age = max(
            1.0, float(rospy.get_param("~target_replace_min_age", 4.0))
        )
        self.target_replace_gain_ratio = max(
            1.05, float(rospy.get_param("~target_replace_gain_ratio", 1.30))
        )
        self.target_replace_stall = max(
            3.0, float(rospy.get_param("~target_replace_stall", 8.0))
        )
        self.room_completion_miss_cycles = max(
            2, int(rospy.get_param("~room_completion_miss_cycles", 3))
        )
        self.latest_snapshot = None
        self.floor_complete = False
        self.floor_index = 0
        self.floor_laser_isolated = False
        self.floor_prefix = "ROOM"
        self.last_command = (0.0, 0.0)

        self.cloud_alignment = None
        self.sphere_hypotheses = {}
        self.reviewed_hypotheses = set()
        self.next_sphere_id = 0
        self.sphere_min_hits = max(2, int(rospy.get_param("~sphere_min_hits", 3)))
        self.sphere_point_stride = max(1, int(rospy.get_param("~sphere_point_stride", 2)))
        self.sphere_process_period = max(
            0.20, float(rospy.get_param("~sphere_process_period", 0.50))
        )
        self.last_sphere_process = rospy.Time(0)
        self.sphere_stale_duration = max(
            2.0, float(rospy.get_param("~sphere_stale_duration", 8.0))
        )

        self.status_pub = rospy.Publisher("/simnav/explorer_status", String, queue_size=3, latch=True)
        self.coverage_pub = rospy.Publisher("/simnav/coverage_status", String, queue_size=3, latch=True)
        self.complete_pub = rospy.Publisher("/simnav/floor_complete", Bool, queue_size=1, latch=True)
        self.command_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.marker_pub = rospy.Publisher("/simnav/coverage_markers", MarkerArray, queue_size=1, latch=True)
        self.coverage_layers_pub = rospy.Publisher(
            "/simnav/coverage_layers", MarkerArray, queue_size=1, latch=True
        )
        self.path_pub = rospy.Publisher("/simnav/coverage_path", Path, queue_size=1, latch=True)
        self.gate_pub = rospy.Publisher("/simnav/entrance_gate", PolygonStamped, queue_size=1, latch=True)
        self.room_entry_pub = rospy.Publisher(
            "/simnav/room_entry", String, queue_size=1, latch=True
        )
        rospy.Subscriber(
            "/simnav/floor_exploration_context", String,
            self._floor_context_callback, queue_size=1,
        )

        rospy.Subscriber("/exploration_map", OccupancyGrid, self._map_callback, queue_size=1)
        rospy.Subscriber(
            "/navigation_map", OccupancyGrid, self._navigation_map_callback, queue_size=1
        )
        rospy.Subscriber("/simnav/odom", Odometry, self._pose_callback, queue_size=10)
        rospy.Subscriber("/simnav/world_pose_metric", PoseStamped, self._world_pose_callback, queue_size=10)
        rospy.Subscriber("/scan_2d", LaserScan, self._scan_callback, queue_size=1)
        rospy.Subscriber("/simnav/camera_coverage", String, self._camera_callback, queue_size=2)
        rospy.Subscriber("/simnav/lio_map_transform", TransformStamped, self._alignment_callback, queue_size=1)
        rospy.Subscriber("/cloud_registered", PointCloud2, self._cloud_callback, queue_size=1)
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self._control)
        # Catch planner exceptions inside the callback.  An exception escaping
        # rospy.Timer terminates that timer thread and otherwise looks exactly
        # like a robot that simply stopped choosing new frontiers.
        self.plan_timer = rospy.Timer(
            rospy.Duration(self.replan_period), self._plan_guarded
        )
        rospy.on_shutdown(self._shutdown)

    @staticmethod
    def _yaw(orientation):
        return transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )[2]

    def _map_callback(self, message):
        with self.lock:
            self.grid = GridView(
                data=np.asarray(message.data, dtype=np.int16).reshape(message.info.height, message.info.width),
                resolution=float(message.info.resolution),
                origin_x=float(message.info.origin.position.x),
                origin_y=float(message.info.origin.position.y),
                frame_id=message.header.frame_id or "simnav_map",
            )

    def _navigation_map_callback(self, message):
        with self.lock:
            self.navigation_grid = GridView(
                data=np.asarray(message.data, dtype=np.int16).reshape(
                    message.info.height, message.info.width
                ),
                resolution=float(message.info.resolution),
                origin_x=float(message.info.origin.position.x),
                origin_y=float(message.info.origin.position.y),
                frame_id=message.header.frame_id or "simnav_map",
            )

    def _pose_callback(self, message):
        with self.lock:
            self.pose = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                self._yaw(message.pose.pose.orientation),
                float(message.pose.pose.position.z),
            )
            self.last_pose_stamp = rospy.Time.now()

    def _world_pose_callback(self, message):
        with self.lock:
            self.world_pose = (
                float(message.pose.position.x),
                float(message.pose.position.y),
                self._yaw(message.pose.orientation),
                float(message.pose.position.z),
            )

    def _floor_context_callback(self, message):
        """Reset room topology after the elevator hands off a new floor."""
        try:
            payload = json.loads(message.data)
            floor_index = int(payload["floor_index"])
            gate = tuple(float(value) for value in payload["gate_source"][:3])
            gate_world = tuple(float(value) for value in payload["gate_world"][:3])
        except (TypeError, ValueError, KeyError, json.JSONDecodeError):
            return
        if floor_index <= 0:
            return
        with self.lock:
            if self.floor_index == floor_index and not self.floor_complete:
                return
            self.floor_index = floor_index
            self.floor_laser_isolated = True
            self.floor_prefix = "F{}_ROOM".format(floor_index)
            self.floor_complete = False
            self.gate_source = gate
            self.gate_world = gate_world
            self.topology_region = "TASK_REGION"
            self.completed_topologies.clear()
            self.topology_states.clear()
            self.topology_miss_cycles.clear()
            self.topology_portals.clear()
            self.portal_evidence.clear()
            self.portal_last_seen.clear()
            self.topology_lock = None
            self.returning_topology = None
            self.rear_transit_active = False
            self.rear_rooms_unlocked = False
            self.front_station_along = None
            self.front_station_topologies.clear()
            self.completed_front_sides.clear()
            self.camera_points_world.clear()
            self.camera_points_order.clear()
            self.sphere_hypotheses.clear()
            self.reviewed_hypotheses.clear()
            self.visited_targets.clear()
            self.blocked_targets.clear()
            self.current_plan = None
            self.active_target = None
            self.active_path = ()
            self.path_index = 0
            self.last_plan_time = rospy.Time(0)
            self.elevator_portal = None
            self.elevator_portal_world = None
            self.elevator_portal_evidence = 0
        self._publish_gate()
        self.complete_pub.publish(Bool(data=False))
        self._publish_status()
        rospy.loginfo("Floor context switched to floor %d; topology prefix=%s", floor_index, self.floor_prefix)

    def _scan_callback(self, message):
        front, left, right = [], [], []
        for index, distance in enumerate(message.ranges):
            if not math.isfinite(distance) or distance < message.range_min or distance > message.range_max:
                continue
            angle = normalize_angle(message.angle_min + index * message.angle_increment)
            if abs(angle) <= math.radians(18.0):
                front.append(distance)
            if math.radians(65.0) <= angle <= math.radians(110.0):
                left.append(distance)
            if math.radians(-110.0) <= angle <= math.radians(-65.0):
                right.append(distance)
        with self.lock:
            self.front_clearance = float(np.percentile(front, 20.0)) if front else float("inf")
            self.left_clearance = float(np.median(left)) if left else float("inf")
            self.right_clearance = float(np.median(right)) if right else float("inf")

    def _camera_callback(self, message):
        try:
            payload = json.loads(message.data)
            # Camera summaries are emitted for the corridor/task region and
            # for every active room scope.  Room observations must remain in
            # the floor-wide union; dropping ``scope_type=room`` makes the
            # planner believe that only the corridor was viewed and can cause
            # both rooms to be released with their lower halves still unseen.
            if payload.get("scope_type") not in ("corridor", "task_region", "room"):
                return
            resolution = max(0.05, float(payload.get("resolution", self.camera_resolution)))
            points = {
                ((int(cell[0]) + 0.5) * resolution, (int(cell[1]) + 0.5) * resolution)
                for cell in payload.get("cells", [])
                if isinstance(cell, (list, tuple)) and len(cell) >= 2
            }
        except (TypeError, ValueError, OverflowError):
            return
        # The detector publishes an accumulated scope, but its scope is reset
        # when it changes from a room back to the corridor.  Keep a bounded
        # floor-wide union here so a previous room's camera coverage is not
        # erased on the next scope message and later room completion remains
        # based on persistent observations.
        with self.lock:
            for point in points:
                if point in self.camera_points_world:
                    continue
                self.camera_points_world.add(point)
                self.camera_points_order.append(point)
            while len(self.camera_points_order) > self.camera_points_memory_limit:
                old_point = self.camera_points_order.popleft()
                self.camera_points_world.discard(old_point)
            self.camera_update = rospy.Time.now()

    def _alignment_callback(self, message):
        q = message.transform.rotation
        matrix = transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        matrix[:3, 3] = [
            message.transform.translation.x,
            message.transform.translation.y,
            message.transform.translation.z,
        ]
        with self.lock:
            self.cloud_alignment = matrix

    def _cloud_callback(self, message):
        with self.lock:
            now_ros = rospy.Time.now()
            if (
                self.last_sphere_process != rospy.Time(0)
                and (now_ros - self.last_sphere_process).to_sec() < self.sphere_process_period
            ):
                return
            alignment = None if self.cloud_alignment is None else self.cloud_alignment.copy()
            robot_z = self.pose[3] if self.pose is not None else None
            ready = self._camera_exploration_ready_locked()
            if ready:
                self.last_sphere_process = now_ros
        if not ready or alignment is None or robot_z is None:
            return
        raw = []
        for index, point in enumerate(point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)):
            if index % self.sphere_point_stride == 0:
                raw.append(point)
        if not raw:
            return
        values = np.asarray(raw, dtype=np.float64)
        homogeneous = np.ones((len(values), 4), dtype=np.float64)
        homogeneous[:, :3] = values
        transformed = np.matmul(alignment, homogeneous.T).T[:, :3]
        centers = detect_sphere_like_clusters(transformed, robot_z)
        now = rospy.Time.now().to_sec()
        with self.lock:
            for center in centers:
                if not self._point_in_task_envelope_locked(center[:2]):
                    continue
                match = None
                for hypothesis_id, item in self.sphere_hypotheses.items():
                    if math.hypot(center[0] - item["center"][0], center[1] - item["center"][1]) <= 0.45:
                        match = hypothesis_id
                        break
                if match is None:
                    match = "sphere_{:04d}".format(self.next_sphere_id)
                    self.next_sphere_id += 1
                    self.sphere_hypotheses[match] = {
                        "id": match,
                        "center": center,
                        "hits": 1,
                        "last_seen": now,
                    }
                else:
                    item = self.sphere_hypotheses[match]
                    weight = 1.0 / float(item["hits"] + 1)
                    item["center"] = tuple(
                        (1.0 - weight) * item["center"][axis] + weight * center[axis]
                        for axis in range(3)
                    )
                    item["hits"] += 1
                    item["last_seen"] = now
            stale = [
                hypothesis_id
                for hypothesis_id, item in self.sphere_hypotheses.items()
                if item["hits"] < self.sphere_min_hits
                and now - item["last_seen"] > self.sphere_stale_duration
            ]
            for hypothesis_id in stale:
                del self.sphere_hypotheses[hypothesis_id]

    def _camera_exploration_ready_locked(self):
        # The corridor is transport-only.  RGB-D processing starts when a
        # room door has been selected and stays active through room exploration
        # and the explicit return to the corridor.
        return bool(self.topology_lock is not None)

    def _point_in_task_envelope_locked(self, point):
        if self.gate_source is None:
            return True
        dx = float(point[0]) - self.gate_source[0]
        dy = float(point[1]) - self.gate_source[1]
        along = dx * math.cos(self.gate_source[2]) + dy * math.sin(self.gate_source[2])
        lateral = -dx * math.sin(self.gate_source[2]) + dy * math.cos(self.gate_source[2])
        return (
            -self.planner.back_extension <= along <= self.planner.forward_depth
            and abs(lateral) <= self.planner.lateral_half_width
        )

    def _world_to_source(self, point, source_pose, world_pose):
        frame_yaw = source_pose[2] - world_pose[2]
        dx, dy = point[0] - world_pose[0], point[1] - world_pose[1]
        cosine, sine = math.cos(frame_yaw), math.sin(frame_yaw)
        return (
            source_pose[0] + cosine * dx - sine * dy,
            source_pose[1] + sine * dx + cosine * dy,
        )

    def _camera_seen_grid(self, grid, source_pose, world_pose, points):
        seen = np.zeros(grid.data.shape, dtype=bool)
        for point in points:
            source = self._world_to_source(point, source_pose, world_pose)
            row, column = grid.world_to_cell(*source)
            radius = max(0, int(math.ceil(0.5 * self.camera_resolution / grid.resolution)))
            row_start, row_stop = max(0, row - radius), min(seen.shape[0], row + radius + 1)
            column_start, column_stop = max(0, column - radius), min(seen.shape[1], column + radius + 1)
            if row_start < row_stop and column_start < column_stop:
                seen[row_start:row_stop, column_start:column_stop] = True
        return seen

    def _stable_spheres(self):
        return [
            dict(item)
            for item in self.sphere_hypotheses.values()
            if item["hits"] >= self.sphere_min_hits
        ]

    def _update_reviewed(self, grid, camera_seen):
        radius_cells = max(1, int(math.ceil(0.40 / grid.resolution)))
        for item in self._stable_spheres():
            if item["id"] in self.reviewed_hypotheses:
                continue
            row, column = grid.world_to_cell(*item["center"][:2])
            row_start, row_stop = max(0, row - radius_cells), min(camera_seen.shape[0], row + radius_cells + 1)
            column_start, column_stop = max(0, column - radius_cells), min(camera_seen.shape[1], column + radius_cells + 1)
            if np.any(camera_seen[row_start:row_stop, column_start:column_stop]):
                self.reviewed_hypotheses.add(item["id"])
                rospy.loginfo("Camera reviewed lidar sphere hypothesis %s", item["id"])

    def _plan_guarded(self, event):
        try:
            self._plan(event)
        except Exception as error:  # keep the rospy.Timer thread alive
            with self.lock:
                self.plan_failures += 1
                self.last_plan_error = "{}: {}".format(type(error).__name__, error)
                self.last_plan_time = rospy.Time.now()
            rospy.logerr("coverage planning cycle failed:\n%s", traceback.format_exc())
            self._stop()
            self._publish_status()

    def _detect_elevator_portal(self, grid, gate_source, gate_world):
        """Cache a lobby-side wide opening without entering room scheduling."""
        if grid is None or gate_source is None:
            return
        lobby_portals = detect_lobby_portals(grid, gate_source[:2], gate_source[2])
        if not lobby_portals:
            return
        selected = max(lobby_portals, key=lambda item: item.width)
        with self.lock:
            if (
                self.elevator_portal is not None
                and selected.width < self.elevator_portal.width
            ):
                return
            self.elevator_portal = selected
            self.elevator_portal_evidence += 1
            if gate_world is not None:
                lobby_yaw = gate_world[2] + math.pi
                tangent = (math.cos(lobby_yaw), math.sin(lobby_yaw))
                normal = (-math.sin(lobby_yaw), math.cos(lobby_yaw))
                side_sign = 1.0 if selected.side == "L" else -1.0
                self.elevator_portal_world = (
                    gate_world[0]
                    + selected.along * tangent[0]
                    + selected.lateral * normal[0],
                    gate_world[1]
                    + selected.along * tangent[1]
                    + selected.lateral * normal[1],
                    normalize_angle(gate_world[2] + side_sign * math.pi / 2.0),
                )
            evidence = self.elevator_portal_evidence
        rospy.loginfo_throttle(
            5.0,
            "Passive elevator portal candidate %s along=%.2f width=%.2f evidence=%d",
            selected.topology_id,
            selected.along,
            selected.width,
            evidence,
        )

    def _plan(self, _event):
        with self.lock:
            if self.floor_complete:
                return
            grid, navigation_grid = self.grid, self.navigation_grid
            pose, world_pose = self.pose, self.world_pose
            gate_source = self.gate_source
            gate_world = self.gate_world
            points = tuple(self.camera_points_world)
            target_active = self.active_target is not None
            active_path = self.active_path
            active_path_index = self.path_index
            now = rospy.Time.now().to_sec()
            self.blocked_targets = [
                item for item in self.blocked_targets if item[2] > now
            ]
            visited = tuple(self.visited_targets) + tuple(
                item[:2] for item in self.blocked_targets
            )
            # Ask for an alternative view of the current room as well.  The
            # active target is kept by the node when its path is still safe,
            # but excluding it from this cycle lets the replacement hysteresis
            # compare against the next real frontier instead of rediscovering
            # the same cell every time the map updates.
            if self.active_target is not None and self.active_target.kind != "SPHERE_REVIEW":
                visited = visited + (tuple(self.active_target.target),)
            spheres = self._stable_spheres()
            reviewed = set(self.reviewed_hypotheses)
            topology_lock = self.topology_lock
            completed_topologies = tuple(self.completed_topologies)
            confirmed_topologies = tuple(
                topology_id
                for topology_id, count in self.portal_evidence.items()
                if int(count) >= self.portal_confirm_cycles
            )
        if grid is None or pose is None or world_pose is None:
            return
        # Passive elevator recognition is deliberately outside the room
        # planner.  It observes the lobby-side negative-along region after the
        # entrance gate is established and never creates ROOM_* targets.
        if gate_source is not None:
            self._detect_elevator_portal(grid, gate_source, gate_world)
        if gate_source is None:
            self._publish_status()
            return
        camera_seen = self._camera_seen_grid(grid, pose, world_pose, points)
        with self.lock:
            self._update_reviewed(grid, camera_seen)
            reviewed = set(self.reviewed_hypotheses)
        plan = self.planner.plan(
            grid,
            pose[:3],
            self.gate_source[:2],
            self.gate_source[2],
            camera_seen,
            visited,
            spheres,
            reviewed,
            self.camera_coverage_target,
            navigation_grid=navigation_grid,
            minimum_forward=self.task_entry_buffer,
            topology_lock=topology_lock,
            completed_topologies=completed_topologies,
            confirmed_topologies=confirmed_topologies,
            remembered_portals=tuple(self.topology_portals.values()),
            rear_rooms_unlocked=self.rear_rooms_unlocked,
            front_station_along_hint=self.front_station_along,
            completed_front_sides=tuple(self.completed_front_sides),
            portal_prefix=self.floor_prefix,
            force_laser_unknown=self.floor_laser_isolated,
        )
        with self.lock:
            seen_portal_ids = set()
            for portal in plan.observed_portals:
                seen_portal_ids.add(portal.topology_id)
                self.portal_evidence[portal.topology_id] = int(
                    self.portal_evidence.get(portal.topology_id, 0)
                ) + 1
                self.portal_last_seen[portal.topology_id] = rospy.Time.now().to_sec()
            for portal in plan.actionable_portals:
                self.topology_portals.setdefault(portal.topology_id, portal)
            # Cache only candidates admitted by the planner's current station
            # lifecycle.  Do not promote every temporally repeated raw map gap:
            # a degraded map can expose several same-side seams at once.
            for portal in plan.front_station_portals:
                if (
                    self.portal_evidence.get(portal.topology_id, 0)
                    >= self.portal_confirm_cycles
                ):
                    self.topology_portals.setdefault(portal.topology_id, portal)
            # A disappeared candidate is not immediately deleted, but stale
            # evidence must eventually expire so a localization jump cannot
            # make an old false portal permanently executable.
            evidence_now = rospy.Time.now().to_sec()
            for topology_id, stamp in list(self.portal_last_seen.items()):
                state = self.topology_states.get(topology_id, {}).get("state")
                stable_unfinished = bool(
                    topology_id in self.topology_portals
                    and state not in ("COMPLETE", "BLOCKED")
                    and topology_id not in self.completed_topologies
                )
                if (
                    topology_id not in seen_portal_ids
                    and evidence_now - float(stamp) > 5.0
                    and not stable_unfinished
                ):
                    self.portal_evidence.pop(topology_id, None)
                    self.portal_last_seen.pop(topology_id, None)
            # Candidate IDs not seen in this map update must not be forgotten
            # immediately: one scan dropout is common near a doorway.  They
            # simply stop accumulating until observed again.
            self.current_plan = plan
            if plan.front_station_along is not None:
                self.front_station_along = float(plan.front_station_along)
                self._remember_front_portals(plan.front_station_portals)
            self.latest_snapshot = plan.snapshot
            self.last_plan_time = rospy.Time.now()
            self.plan_cycles += 1
            self.last_plan_error = None
            self.last_plan_reason = plan.reason
            active_safe = bool(
                target_active
                and self.planner.path_is_safe(
                    navigation_grid if navigation_grid is not None else grid,
                    active_path[active_path_index:],
                    0.12 if self.active_target is not None
                    and self.active_target.kind == "RETURN_TO_CORRIDOR" else None,
                )
            )
            sphere_preemption = bool(
                plan.target is not None
                and plan.target.kind == "SPHERE_REVIEW"
                and (self.active_target is None or self.active_target.kind != "SPHERE_REVIEW")
                and (
                    self.active_target is None
                    or self.active_target.kind != "RETURN_TO_CORRIDOR"
                )
            )
            replacement = bool(
                active_safe
                and not sphere_preemption
                and self._should_replace_active(plan.target, now)
            )
            lifecycle_changed = self._update_topology_lifecycle(plan, target_active)
            transit_changed = self._update_rear_transit(plan)
            door_search_changed = self._update_door_search(plan, target_active)
            self._check_completion()
            if self.floor_complete:
                self._publish_status()
                return
            if lifecycle_changed or transit_changed or door_search_changed:
                self._publish_path()
                self._publish_markers()
                self._publish_coverage_layers()
                self._publish_status()
                return
            lock_state = None
            if self.topology_lock is not None:
                lock_state = self.topology_states.get(self.topology_lock, {}).get("state")
            if lock_state == "RETURNING":
                # A failed return-path rebuild must stop and retry; it must
                # never fall through to the ordinary room-frontier assignment
                # below during the same cycle.
                if not target_kind_allowed_for_topology_state(
                    lock_state, getattr(self.active_target, "kind", None)
                ):
                    self.active_target = None
                    self.active_path = ()
                    self.path_index = 0
                self._publish_path()
                self._publish_markers()
                self._publish_coverage_layers()
                self._publish_status()
                return
            if active_safe and not sphere_preemption and not replacement:
                self._publish_markers()
                self._publish_coverage_layers()
                self._publish_status()
                return
            self.active_target = None
            self.active_path = ()
            self.path_index = 0
            if plan.target is not None:
                self.active_target = plan.target
                self.active_path = plan.target.path or (plan.target.target,)
                self.path_index = min(1, len(self.active_path) - 1)
                self.review_hold_since = None
                self.active_target_started = rospy.Time.from_sec(now)
                self.active_target_last_progress = self.path_index
                self.active_target_last_progress_stamp = rospy.Time.from_sec(now)
                self.active_target_last_distance = None
                if (
                    plan.target.topology_id != "CORRIDOR"
                    and "UNASSIGNED" not in plan.target.topology_id
                ):
                    self.topology_lock = plan.target.topology_id
                    state = self.topology_states.setdefault(
                        plan.target.topology_id,
                        {"state": "APPROACHING", "targets": 0},
                    )
                    # A new frontier in the same locked room is not a new
                    # doorway approach.  Preserve geometric proof that the
                    # robot has already crossed into the room; otherwise the
                    # completion/return branch can never run once the robot
                    # has moved away from the doorway.
                    state["state"] = topology_state_for_new_target(
                        state.get("state")
                    )
                    if self.room_scope_announced != plan.target.topology_id:
                        self.room_entry_pub.publish(
                            String(
                                data=json.dumps(
                                    {
                                        "candidate_id": plan.target.topology_id,
                                        "pose": [
                                            float(self.world_pose[0]),
                                            float(self.world_pose[1]),
                                            float(self.world_pose[2]),
                                        ],
                                    },
                                    sort_keys=True,
                                )
                            )
                        )
                        self.room_scope_announced = plan.target.topology_id
                rospy.loginfo(
                    "COVERAGE target kind=%s topology=%s path=%.2f laser_gain=%.2f camera_gain=%.2f combined_gain=%.2f%s",
                    plan.target.kind,
                    plan.target.topology_id,
                    plan.target.path_length,
                    plan.target.laser_gain,
                    plan.target.camera_gain,
                    plan.target.combined_gain,
                    " (replaced)" if replacement else "",
                )
            self._publish_path()
            self._publish_markers()
            self._publish_coverage_layers()
            self._publish_status()

    def _should_replace_active(self, candidate, now):
        """Apply target-switch hysteresis while a route is still safe.

        Map updates are frequent and can create a slightly nearer version of
        the same frontier.  Such updates must not cause oscillation.  A switch
        is allowed only after a minimum age, for a genuinely different target,
        and either a substantial information-gain advantage or a stalled
        route.  Sphere review remains handled by the separate preemption rule.
        """
        if candidate is None or self.active_target is None:
            return False
        active = self.active_target
        lock_state = None
        if self.topology_lock is not None:
            lock_state = self.topology_states.get(self.topology_lock, {}).get("state")
        if not target_kind_allowed_for_topology_state(
            lock_state, getattr(candidate, "kind", None)
        ):
            return False
        if active.kind == "RETURN_TO_CORRIDOR":
            return False
        if self.topology_lock is not None:
            state = self.topology_states.get(self.topology_lock, {}).get("state")
            # Keep the originally collision-checked entry manoeuvre intact.
            # Replacing a room frontier before doorway crossing rebuilds the
            # staged path from the corridor centre and can pull a robot that
            # is already in the doorway back out again.
            if state == "APPROACHING":
                return False
        if candidate.kind == "SPHERE_REVIEW" or active.kind == "SPHERE_REVIEW":
            return False
        if candidate.topology_id != active.topology_id:
            # A room lock is intentionally a hard boundary once a room has
            # been selected.  Before a lock is established, a clearly better
            # room may still replace a corridor frontier.
            if self.topology_lock is not None:
                return False
        separation = math.hypot(
            candidate.target[0] - active.target[0],
            candidate.target[1] - active.target[1],
        )
        if separation < max(0.8, 0.8 * self.planner.revisit_radius):
            return False
        age = float(now) - self.active_target_started.to_sec()
        if age < self.target_replace_min_age:
            return False
        progress_age = float(now) - self.active_target_last_progress_stamp.to_sec()
        gain_better = candidate.combined_gain >= max(
            1e-6, active.combined_gain * self.target_replace_gain_ratio
        )
        stalled = progress_age >= self.target_replace_stall
        if not (gain_better or stalled):
            return False
        self.target_replacements += 1
        return True

    def _topology_coordinates(self):
        if self.pose is None or self.gate_source is None:
            return None, None
        dx = self.pose[0] - self.gate_source[0]
        dy = self.pose[1] - self.gate_source[1]
        along = dx * math.cos(self.gate_source[2]) + dy * math.sin(self.gate_source[2])
        lateral = -dx * math.sin(self.gate_source[2]) + dy * math.cos(self.gate_source[2])
        return along, lateral

    def _remember_front_portals(self, portals):
        """Keep one stable front-room identity per side across SLAM jitter."""
        for portal in portals:
            existing = next(
                (
                    topology_id
                    for topology_id in self.front_station_topologies
                    if topology_id.split("_")[-2] == portal.side
                ),
                None,
            )
            if existing == portal.topology_id:
                continue
            if existing is not None and (
                existing in self.completed_topologies
                or existing == self.topology_lock
            ):
                continue
            if existing is not None:
                self.front_station_topologies.discard(existing)
            self.front_station_topologies.add(portal.topology_id)

    def _start_corridor_return(self, plan, lock):
        portal = next(
            (
                item
                for item in plan.actionable_portals
                if item.topology_id == str(lock)
            ),
            None,
        )
        if portal is None:
            portal = self.topology_portals.get(str(lock))
        grid = self.navigation_grid if self.navigation_grid is not None else self.grid
        if portal is None or grid is None or self.pose is None or self.gate_source is None:
            return False
        sign = 1.0 if portal.side == "L" else -1.0
        door_centre = self.planner._portal_waypoint(
            self.gate_source[:2], self.gate_source[2], portal.along, portal.lateral
        )
        corridor_stage = self.planner._portal_waypoint(
            self.gate_source[:2], self.gate_source[2], portal.along, 0.0
        )
        path, path_length, minimum = (), 0.0, 0.0
        # The online portal centre can drift by a few map cells after the robot
        # has scanned the room.  Search only inside the cached doorway width,
        # keeping the crossing normal and clearance unchanged.  This remains a
        # return through the confirmed door, not a generic wall-gap fallback.
        for along_offset in portal_return_along_offsets(portal.width):
            shifted_along = float(portal.along) + along_offset
            shifted_corridor = self.planner._portal_waypoint(
                self.gate_source[:2], self.gate_source[2], shifted_along, 0.0
            )
            for depth in (0.80, 0.70, 0.60, 0.50, 0.40, 0.30, 0.20, 0.12, 0.08):
                room_stage = self.planner._portal_waypoint(
                    self.gate_source[:2],
                    self.gate_source[2],
                    shifted_along,
                    sign * (self.planner.corridor_half_width + depth),
                )
                path, path_length, minimum = (
                    self.planner.navigation_path_from_room_through_portal(
                        grid,
                        self.pose[:3],
                        room_stage,
                        shifted_corridor,
                        portal_clearance=0.12,
                    )
                )
                if path:
                    corridor_stage = shifted_corridor
                    break
            if path:
                break
        if not path:
            return False
        target = FrontierTarget(
            kind="RETURN_TO_CORRIDOR",
            target=corridor_stage,
            path=path,
            path_length=path_length,
            laser_gain=0.0,
            camera_gain=0.0,
            combined_gain=0.0,
            min_clearance=minimum,
            topology_id="CORRIDOR",
        )
        now = rospy.Time.now()
        self.active_target = target
        self.active_path = target.path
        self.path_index = min(1, len(self.active_path) - 1)
        self.active_target_started = now
        self.active_target_last_progress = self.path_index
        self.active_target_last_progress_stamp = now
        self.active_target_last_distance = None
        self.review_hold_since = None
        self.returning_topology = str(lock)
        return True

    def _update_topology_lifecycle(self, plan, target_active):
        """Advance room state; every completed room must return via CORRIDOR."""
        lock = self.topology_lock
        if lock is None:
            return False
        state = self.topology_states.setdefault(lock, {"state": "APPROACHING", "targets": 0})
        along, lateral = self._topology_coordinates()

        if state.get("state") == "RETURNING":
            in_corridor = bool(
                lateral is not None
                and abs(lateral) <= self.planner.corridor_half_width - 0.10
            )
            if in_corridor:
                state["state"] = "COMPLETE"
                self.completed_topologies.add(lock)
                if lock in self.front_station_topologies:
                    self.completed_front_sides.add(lock.split("_")[-2])
                self.topology_lock = None
                self.returning_topology = None
                self.active_target = None
                self.active_path = ()
                self.path_index = 0
                self.last_plan_time = rospy.Time(0)
                rospy.loginfo(
                    "Topology room %s complete and robot confirmed back in CORRIDOR",
                    lock,
                )
                return True
            # RETURNING owns the motion channel exclusively.  A stale room
            # frontier can survive the exact planner cycle that changed the
            # lifecycle, or appear after a completed return segment clears the
            # active target.  Remove it here before any normal plan can run.
            if not target_kind_allowed_for_topology_state(
                "RETURNING",
                getattr(self.active_target, "kind", None),
            ):
                self.active_target = None
                self.active_path = ()
                self.path_index = 0
            if self.active_target is None:
                if self._start_corridor_return(plan, lock):
                    rospy.loginfo("Retrying corridor return for topology room %s", lock)
                    return True
                rospy.logwarn_throttle(
                    5.0, "No executable return-to-corridor path for topology room %s", lock
                )
            return False

        # Crossing the corridor side boundary is the minimum geometric proof
        # that the robot entered a room.  Reaching a room target in the
        # corridor does not count as entry.
        entered = state.get("state") == "EXPLORING"
        portal = next(
            (item for item in plan.actionable_portals if item.topology_id == str(lock)),
            None,
        )
        if portal is None:
            portal = self.topology_portals.get(str(lock))
        if not entered and along is not None and lateral is not None and portal is not None:
            longitudinal_ok = abs(along - portal.along) <= max(
                1.0, 0.5 * float(portal.width) + 0.45
            )
            side_ok = (
                lateral > self.planner.corridor_half_width + 0.30
                if portal.side == "L"
                else lateral < -self.planner.corridor_half_width - 0.30
            )
            entered = longitudinal_ok and side_ok
        if entered:
            state["state"] = "EXPLORING"
        elif state.get("state") not in ("COMPLETE", "BLOCKED"):
            state["state"] = "APPROACHING"

        local = (plan.topology_coverages or {}).get(lock)
        local_coverage_ok = bool(
            state.get("state") == "EXPLORING"
            and local is not None
            and local.combined >= self.room_combined_coverage_target
        )
        if local_coverage_ok:
            state["state"] = "RETURNING"
            if self._start_corridor_return(plan, lock):
                rospy.loginfo(
                    "Topology room %s coverage complete; returning to CORRIDOR before next room",
                    lock,
                )
                return True
            rospy.logwarn_throttle(
                5.0,
                "Topology room %s coverage complete but return-to-corridor path is unavailable",
                lock,
            )
            return False

        # ``candidate_topologies`` describes the global pool before the room
        # lock is applied.  Use the filtered list here: a different room being
        # available must not keep an exhausted locked room alive forever.
        has_room_candidates = any(
            item.topology_id == lock for item in plan.targets
        )
        if target_active or has_room_candidates:
            self.topology_miss_cycles[lock] = 0
            return
        # No active target and no candidate owned by this room.  Require
        # repeated planner cycles because a single SLAM update can temporarily
        # hide a frontier while the map is being fused.
        misses = self.topology_miss_cycles.get(lock, 0) + 1
        self.topology_miss_cycles[lock] = misses
        if misses < self.room_completion_miss_cycles:
            return
        if state.get("state") != "EXPLORING":
            # We never crossed the doorway.  This is a failed approach, not a
            # completed room.  Release the lock so another portal can be tried
            # while the blocked target remains on a short cooldown.
            state["state"] = "BLOCKED"
            self.topology_lock = None
            rospy.logwarn("Topology room %s approach failed before doorway crossing", lock)
            return True
        # Never dispatch another topology while the robot is still inside this
        # room.  Keep the lock and let later map/camera updates expose another
        # local frontier.
        state["state"] = "EXPLORING"
        self.topology_miss_cycles[lock] = 0
        rospy.logwarn_throttle(
            5.0,
            "Topology room %s has no executable frontier but local coverage is low "
            "(laser=%.3f camera=%.3f); keeping room lock",
            lock,
            local.laser if local is not None else 0.0,
            local.camera if local is not None else 0.0,
        )
        return False

    def _update_rear_transit(self, plan):
        """Unlock rear-door search only after both front rooms are complete."""
        if self.topology_lock is not None or self.rear_rooms_unlocked:
            return False
        if self.rear_transit_active:
            return False
        self._remember_front_portals(plan.front_station_portals)
        remembered_sides = {
            topology_id.split("_")[-2]
            for topology_id in self.front_station_topologies
            if "_ROOM_" in topology_id or topology_id.startswith("ROOM_")
        }
        front_complete = bool(
            remembered_sides == {"L", "R"}
            and self.completed_front_sides == {"L", "R"}
        )
        if not front_complete:
            return False
        along, lateral = self._topology_coordinates()
        if along is None or lateral is None:
            return False
        if abs(lateral) > self.planner.corridor_half_width:
            return False
        self.rear_transit_active = True
        self.rear_transit_start_along = float(along)
        self.topology_region = "REAR_TRANSIT"
        self.active_target = None
        self.active_path = ()
        self.path_index = 0
        self.last_plan_time = rospy.Time(0)
        rospy.loginfo(
            "Both front rooms complete; starting %.2f m controlled rear transit",
            self.rear_transit_distance,
        )
        return True

    def _update_door_search(self, plan, target_active):
        """Start/cancel a bounded one-metre corridor scan segment.

        A visible but not-yet-confirmed portal always wins: the robot remains
        stopped so its evidence can reach ``portal_confirm_cycles``.  Motion is
        used only after repeated completely empty portal scans.
        """
        if (
            self.topology_lock is not None
            or self.rear_transit_active
            or target_active
            or plan.target is not None
            or plan.actionable_portals
        ):
            changed = self.door_search_active
            self.door_search_active = False
            self.door_search_step_start_along = None
            self.door_search_idle_cycles = 0
            return changed

        # Never creep away from an already established front station while
        # waiting for its opposite door.  The paired door must be found from
        # the same station, ensuring all front rooms finish before rear travel.
        phase = "REAR" if self.rear_rooms_unlocked else "FRONT"
        if phase == "FRONT" and self.front_station_along is not None:
            self.door_search_active = False
            self.door_search_step_start_along = None
            self.door_search_idle_cycles = 0
            return False

        along, lateral = self._topology_coordinates()
        if along is None or lateral is None:
            return False
        if abs(lateral) > self.planner.corridor_half_width:
            return False

        if phase != self.door_search_phase:
            self.door_search_phase = phase
            self.door_search_travel = 0.0
            self.door_search_idle_cycles = 0
            self.door_search_active = False
            self.door_search_step_start_along = None

        # Do not move while a geometrically valid portal is accumulating its
        # temporal confirmation count.
        if plan.observed_portals:
            changed = self.door_search_active
            self.door_search_active = False
            self.door_search_step_start_along = None
            self.door_search_idle_cycles = 0
            self.topology_region = "{}_DOOR_CONFIRM".format(phase)
            return changed

        limit = (
            self.door_search_rear_limit
            if phase == "REAR"
            else self.door_search_front_limit
        )
        if self.door_search_active or self.door_search_travel >= limit - 1e-6:
            return False
        self.door_search_idle_cycles += 1
        if self.door_search_idle_cycles < self.door_search_wait_cycles:
            self.topology_region = "{}_DOOR_RESCAN".format(phase)
            return False

        self.door_search_active = True
        self.door_search_step_start_along = float(along)
        self.door_search_idle_cycles = 0
        self.topology_region = "{}_DOOR_SEARCH_STEP".format(phase)
        self.active_target = None
        self.active_path = ()
        self.path_index = 0
        rospy.loginfo(
            "%s door not observed; starting bounded %.2f m recognition segment "
            "(travel %.2f/%.2f m)",
            phase,
            min(self.door_search_step_distance, limit - self.door_search_travel),
            self.door_search_travel,
            limit,
        )
        return True

    def _check_completion(self):
        unreviewed = [
            item for item in self._stable_spheres() if item["id"] not in self.reviewed_hypotheses
        ]
        meets = topology_completion_ready(
            self.completed_topologies,
            self.expected_rooms_per_floor,
            len(unreviewed),
        )
        now = rospy.Time.now()
        if not meets:
            self.completion_since = None
            return
        if self.completion_since is None:
            self.completion_since = now
            return
        if (now - self.completion_since).to_sec() >= self.completion_stable_duration:
            self.floor_complete = True
            self._stop()
            self.complete_pub.publish(Bool(data=True))
            rospy.loginfo(
                "TASK_REGION_COMPLETE rooms=%d/%d",
                len(self.completed_topologies),
                self.expected_rooms_per_floor,
            )

    def _control(self, _event):
        with self.lock:
            pose = self.pose
            world_pose = self.world_pose
            front = self.front_clearance
            complete = self.floor_complete
            gate = self.gate_source
            target = self.active_target
            path = self.active_path
            path_index = self.path_index
            topology_lock = self.topology_lock
            topology_state = self.topology_states.get(topology_lock, {}).get("state")
            topology_portal = self.topology_portals.get(str(topology_lock))
        # _check_completion() publishes one final zero command before latching
        # floor_complete.  Afterwards the elevator transition node owns
        # /cmd_vel; continuing to publish zeros here would fight that handoff.
        if complete:
            return
        if pose is None:
            self._stop()
            return
        if (rospy.Time.now() - self.last_pose_stamp).to_sec() > 1.0:
            self._stop()
            return
        if gate is None:
            self._control_initial_forward(pose, world_pose, front)
            self._publish_status()
            return
        if self.rear_transit_active:
            self._control_rear_transit(pose, front)
            self._publish_status()
            return
        if self.door_search_active:
            self._control_door_search(pose, front)
            self._publish_status()
            return
        if target is None or not path:
            self._stop()
            return
        waypoint = path[path_index]
        distance = math.hypot(waypoint[0] - pose[0], waypoint[1] - pose[1])
        with self.lock:
            if (
                self.active_target_last_distance is None
                or distance < self.active_target_last_distance - 0.08
            ):
                self.active_target_last_distance = distance
                self.active_target_last_progress_stamp = rospy.Time.now()
        if distance <= self.target_tolerance:
            self._stop()
            with self.lock:
                if self.path_index + 1 < len(self.active_path):
                    self.path_index += 1
                    self._publish_path()
                    return
                if target.kind in ("SPHERE_REVIEW", "CAMERA_FRONTIER") and target.look_at is not None:
                    look_yaw = math.atan2(target.look_at[1] - pose[1], target.look_at[0] - pose[0])
                    error = normalize_angle(look_yaw - pose[2])
                    if abs(error) > 0.12:
                        command = Twist()
                        command.angular.z = math.copysign(self.turn_speed, error)
                        self._publish_command(command)
                        self.review_hold_since = None
                        return
                    if self.review_hold_since is None:
                        self.review_hold_since = rospy.Time.now()
                        return
                    hold_duration = self.review_hold_duration if target.kind == "SPHERE_REVIEW" else 0.6
                    if (rospy.Time.now() - self.review_hold_since).to_sec() < hold_duration:
                        return
                self.visited_targets.append(tuple(target.target))
                self.targets_reached += 1
                self.active_target = None
                self.active_path = ()
                self.path_index = 0
                self.last_plan_time = rospy.Time(0)
            return
        target_yaw = math.atan2(waypoint[1] - pose[1], waypoint[0] - pose[0])
        doorway_entry = False
        if topology_state == "APPROACHING" and topology_portal is not None:
            tangent = (math.cos(gate[2]), math.sin(gate[2]))
            normal = (-math.sin(gate[2]), math.cos(gate[2]))
            along = (pose[0] - gate[0]) * tangent[0] + (pose[1] - gate[1]) * tangent[1]
            lateral = (pose[0] - gate[0]) * normal[0] + (pose[1] - gate[1]) * normal[1]
            waypoint_lateral = (
                (waypoint[0] - gate[0]) * normal[0]
                + (waypoint[1] - gate[1]) * normal[1]
            )
            side_sign = 1.0 if topology_portal.side == "L" else -1.0
            doorway_entry = bool(
                abs(along - topology_portal.along)
                <= max(0.75, 0.5 * float(topology_portal.width) + 0.30)
                and side_sign * waypoint_lateral > side_sign * lateral + 0.08
            )
            if doorway_entry:
                target_yaw = corrected_portal_heading(
                    gate[2], topology_portal.side, topology_portal.along - along
                )
        yaw_error = normalize_angle(target_yaw - pose[2])
        command = Twist()
        heading_tolerance = 0.10 if doorway_entry else self.heading_tolerance
        if abs(yaw_error) > heading_tolerance:
            command.angular.z = math.copysign(self.turn_speed, yaw_error)
        elif front < self.motion_stop_distance:
            with self.lock:
                # A local collision stop is a navigation failure, not proof
                # that this viewpoint has been explored.  Suppress it only
                # briefly so a map update can produce a new approach.
                self.blocked_targets.append(
                    (
                        float(target.target[0]),
                        float(target.target[1]),
                        rospy.Time.now().to_sec() + 6.0,
                    )
                )
                self.navigation_blocks += 1
                self.active_target = None
                self.active_path = ()
                self.path_index = 0
                self.last_plan_time = rospy.Time(0)
            self._stop()
            return
        else:
            command.linear.x = min(self.motion_speed, 0.25) if doorway_entry else self.motion_speed
            command.angular.z = max(-0.15, min(0.15, 0.8 * yaw_error))
        self._publish_command(command)

    def _control_door_search(self, pose, front):
        along, _lateral = self._topology_coordinates()
        if along is None or self.door_search_step_start_along is None:
            self._stop()
            return
        phase = self.door_search_phase
        limit = (
            self.door_search_rear_limit
            if phase == "REAR"
            else self.door_search_front_limit
        )
        segment_limit = min(
            self.door_search_step_distance,
            max(0.0, limit - self.door_search_travel),
        )
        progress = max(0.0, float(along) - float(self.door_search_step_start_along))
        if progress >= segment_limit - 1e-3:
            self._stop()
            self.door_search_travel = min(limit, self.door_search_travel + progress)
            self.door_search_active = False
            self.door_search_step_start_along = None
            self.door_search_idle_cycles = 0
            self.topology_region = "{}_DOOR_RESCAN".format(phase)
            self.last_plan_time = rospy.Time(0)
            rospy.loginfo(
                "%s door recognition segment complete; stopped for rescan "
                "(travel %.2f/%.2f m)",
                phase,
                self.door_search_travel,
                limit,
            )
            return

        left, right = self.left_clearance, self.right_clearance
        target_yaw = self.gate_source[2]
        if math.isfinite(left) and math.isfinite(right):
            target_yaw = normalize_angle(
                target_yaw + max(-0.12, min(0.12, 0.10 * (left - right)))
            )
        error = normalize_angle(target_yaw - pose[2])
        command = Twist()
        if abs(error) > 0.18:
            command.angular.z = math.copysign(min(0.35, self.turn_speed), error)
        elif front >= self.motion_stop_distance:
            command.linear.x = min(self.motion_speed, self.initial_forward_speed)
            command.angular.z = max(-0.10, min(0.10, 0.8 * error))
        self._publish_command(command)

    def _control_rear_transit(self, pose, front):
        along, lateral = self._topology_coordinates()
        if along is None or self.rear_transit_start_along is None:
            self._stop()
            return
        progress = max(0.0, float(along) - float(self.rear_transit_start_along))
        if progress >= self.rear_transit_distance:
            self._stop()
            with self.lock:
                self.rear_transit_active = False
                self.rear_rooms_unlocked = True
                self.topology_region = "REAR_DOOR_SEARCH"
                self.door_search_phase = "REAR"
                self.door_search_travel = 0.0
                self.door_search_idle_cycles = 0
                self.last_plan_time = rospy.Time(0)
            rospy.loginfo(
                "Controlled rear transit complete at %.2f m; rear doors unlocked",
                progress,
            )
            return
        left, right = self.left_clearance, self.right_clearance
        target_yaw = self.gate_source[2]
        if math.isfinite(left) and math.isfinite(right):
            target_yaw = normalize_angle(
                target_yaw + max(-0.12, min(0.12, 0.10 * (left - right)))
            )
        error = normalize_angle(target_yaw - pose[2])
        command = Twist()
        if abs(error) > 0.18:
            command.angular.z = math.copysign(min(0.35, self.turn_speed), error)
        elif front >= self.motion_stop_distance:
            command.linear.x = self.motion_speed
            command.angular.z = max(-0.10, min(0.10, 0.8 * error))
        self._publish_command(command)

    def _control_initial_forward(self, pose, world_pose, front):
        with self.lock:
            if self.forward_anchor is None:
                self.forward_anchor = pose[:2]
                self.forward_yaw = normalize_angle(self.configured_yaw)
                self.lobby_entry_source = (pose[0], pose[1], self.forward_yaw)
                if world_pose is not None:
                    frame_yaw = world_pose[2] - pose[2]
                    self.world_forward_yaw = normalize_angle(self.forward_yaw + frame_yaw)
                    self.lobby_entry_world = (world_pose[0], world_pose[1], self.world_forward_yaw)
            anchor, yaw = self.forward_anchor, self.forward_yaw
            left, right = self.left_clearance, self.right_clearance
        progress = (pose[0] - anchor[0]) * math.cos(yaw) + (pose[1] - anchor[1]) * math.sin(yaw)
        if progress >= self.initial_forward_distance:
            self._stop()
            if not self.plane_policy_active:
                now = rospy.Time.now()
                if self.policy_switch_stop_since is None:
                    self.policy_switch_stop_since = now
                    rospy.loginfo(
                        "Entrance threshold crossed; stopping before plane-policy switch"
                    )
                    return
                if (now - self.policy_switch_stop_since).to_sec() < 0.8:
                    return
                try:
                    rospy.wait_for_service(
                        "/unitree/select_plane_policy", timeout=0.20
                    )
                    response = self.policy_switch_service(True)
                except (rospy.ROSException, rospy.ServiceException) as error:
                    rospy.logwarn_throttle(
                        2.0, "Waiting for safe plane-policy switch: %s", error
                    )
                    return
                if not response.success:
                    rospy.logwarn_throttle(
                        2.0, "Plane-policy switch deferred: %s", response.message
                    )
                    return
                self.plane_policy_active = True
                rospy.loginfo("Plane locomotion policy active; continuing exploration")
            with self.lock:
                gate_distance = self.virtual_gate_forward_distance
                self.gate_source = (
                    anchor[0] + gate_distance * math.cos(yaw),
                    anchor[1] + gate_distance * math.sin(yaw),
                    yaw,
                )
                if self.lobby_entry_world is not None:
                    self.gate_world = (
                        self.lobby_entry_world[0]
                        + gate_distance * math.cos(self.world_forward_yaw),
                        self.lobby_entry_world[1]
                        + gate_distance * math.sin(self.world_forward_yaw),
                        self.world_forward_yaw,
                    )
                self.topology_region = "TASK_REGION"
                self.active_target = None
                self.last_plan_time = rospy.Time(0)
            self._publish_gate()
            self._publish_markers()
            self._detect_elevator_portal(self.grid, self.gate_source, self.gate_world)
            rospy.loginfo("Lobby transit complete; direct task-region exploration enabled")
            return
        target_yaw = yaw
        if progress >= self.initial_centering_start_distance and math.isfinite(left) and math.isfinite(right):
            target_yaw = normalize_angle(yaw + max(-0.18, min(0.18, 0.12 * (left - right))))
        error = normalize_angle(target_yaw - pose[2])
        command = Twist()
        if abs(error) > 0.10:
            command.angular.z = math.copysign(self.turn_speed, error)
        elif front >= self.motion_stop_distance:
            command.linear.x = self.initial_forward_speed
            command.angular.z = max(-0.10, min(0.10, 0.8 * error))
        self._publish_command(command)

    def _publish_gate(self):
        with self.lock:
            gate = self.gate_source
            grid = self.grid
        if gate is None:
            return
        message = PolygonStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = grid.frame_id if grid is not None else "simnav_map"
        normal = (-math.sin(gate[2]), math.cos(gate[2]))
        for sign in (-1.0, 1.0):
            message.polygon.points.append(
                Point32(
                    gate[0] + sign * self.virtual_gate_half_width * normal[0],
                    gate[1] + sign * self.virtual_gate_half_width * normal[1],
                    0.0,
                )
            )
        self.gate_pub.publish(message)

    def _publish_command(self, command):
        self.last_command = (float(command.linear.x), float(command.angular.z))
        self.command_pub.publish(command)

    def _stop(self):
        self._publish_command(Twist())

    def _publish_path(self):
        with self.lock:
            grid, path, index = self.grid, self.active_path, self.path_index
        message = Path()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = grid.frame_id if grid is not None else "simnav_map"
        for point in path[index:]:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def _publish_markers(self):
        with self.lock:
            grid, gate, target = self.grid, self.gate_source, self.active_target
            hypotheses = self._stable_spheres()
            reviewed = set(self.reviewed_hypotheses)
            topology_states = dict(self.topology_states)
            topology_lock = self.topology_lock
            portal_evidence = dict(self.portal_evidence)
        frame = grid.frame_id if grid is not None else "simnav_map"
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = frame
        clear.header.stamp = rospy.Time.now()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        if gate is not None:
            marker = Marker()
            marker.header = clear.header
            marker.ns = "task_gate"
            marker.id = 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = gate[0], gate[1]
            marker.pose.position.z = 0.05
            marker.pose.orientation.z = math.sin(gate[2] / 2.0)
            marker.pose.orientation.w = math.cos(gate[2] / 2.0)
            marker.scale.x, marker.scale.y, marker.scale.z = (
                0.08,
                2.0 * self.virtual_gate_half_width,
                0.08,
            )
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.05, 0.9, 0.95, 0.9
            markers.markers.append(marker)
            if self.show_room_virtual_doors:
                # Show only temporally confirmed map-derived portals.  A
                # one-frame opening is intentionally omitted so RViz depicts
                # the exact doorway set that is allowed to own exploration
                # targets.  These markers never modify the navigation map.
                observed_portals = detect_room_portals(
                    grid,
                    gate[:2],
                    gate[2],
                    self.planner.forward_depth,
                    self.planner.lateral_half_width,
                    self.planner.corridor_half_width,
                    portal_prefix=self.floor_prefix,
                )
                portals = [
                    portal
                    for portal in observed_portals
                    if int(portal_evidence.get(portal.topology_id, 0))
                    >= self.portal_confirm_cycles
                    or portal.topology_id == topology_lock
                ]
                stations = pair_room_portals(observed_portals)
                paired_ids = {
                    portal.topology_id
                    for station in stations
                    if (
                        int(portal_evidence.get(station.left.topology_id, 0))
                        >= self.portal_confirm_cycles
                        and int(portal_evidence.get(station.right.topology_id, 0))
                        >= self.portal_confirm_cycles
                    )
                    for portal in (station.left, station.right)
                }
                normal = (-math.sin(gate[2]), math.cos(gate[2]))
                forward = (math.cos(gate[2]), math.sin(gate[2]))
                for index, portal in enumerate(portals):
                    marker = Marker()
                    marker.header = clear.header
                    marker.ns = "room_virtual_doors"
                    marker.id = 500 + index
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    marker.pose.position.x = gate[0] + forward[0] * portal.along + normal[0] * portal.lateral
                    marker.pose.position.y = gate[1] + forward[1] * portal.along + normal[1] * portal.lateral
                    marker.pose.position.z = 0.08
                    marker.pose.orientation.z = math.sin(gate[2] / 2.0)
                    marker.pose.orientation.w = math.cos(gate[2] / 2.0)
                    # A side doorway lies in the corridor wall: its long axis
                    # is the corridor-forward axis.  The old x/y scales were
                    # swapped and rendered a misleading line across the
                    # corridor, resembling a virtual obstacle.
                    marker.scale.x = max(0.40, float(portal.width))
                    marker.scale.y = 0.08
                    marker.scale.z = 0.06
                    state = topology_states.get(portal.topology_id, {}).get(
                        "state",
                        "PAIRED" if portal.topology_id in paired_ids else "READY",
                    )
                    colors = {
                        "READY": (1.00, 0.45, 0.05),
                        "PAIRED": (0.95, 0.80, 0.05),
                        "APPROACHING": (0.95, 0.15, 0.85),
                        "EXPLORING": (0.10, 0.95, 0.35),
                        "RETURNING": (0.15, 0.45, 1.00),
                        "COMPLETE": (0.10, 0.80, 0.90),
                        "BLOCKED": (0.95, 0.10, 0.10),
                        "NEEDS_REVISIT": (1.00, 0.45, 0.05),
                    }
                    marker.color.r, marker.color.g, marker.color.b = colors.get(
                        state, colors["READY"]
                    )
                    marker.color.a = 0.95
                    markers.markers.append(marker)

                    label = Marker()
                    label.header = clear.header
                    label.ns = "confirmed_room_portal_labels"
                    label.id = 700 + index
                    label.type = Marker.TEXT_VIEW_FACING
                    label.action = Marker.ADD
                    label.pose.position.x = marker.pose.position.x + normal[0] * (
                        0.45 if portal.side == "L" else -0.45
                    )
                    label.pose.position.y = marker.pose.position.y + normal[1] * (
                        0.45 if portal.side == "L" else -0.45
                    )
                    label.pose.position.z = 0.34
                    label.pose.orientation.w = 1.0
                    label.scale.z = 0.24
                    label.color.r = marker.color.r
                    label.color.g = marker.color.g
                    label.color.b = marker.color.b
                    label.color.a = 1.0
                    label.text = "{}  {}  d={:.2f}m  e={}".format(
                        portal.topology_id,
                        "PAIRED" if portal.topology_id in paired_ids else "READY",
                        float(portal.along),
                        int(portal_evidence.get(portal.topology_id, 0)),
                    )
                    markers.markers.append(label)
        for index, item in enumerate(hypotheses):
            marker = Marker()
            marker.header = clear.header
            marker.ns = "sphere_hypotheses"
            marker.id = 100 + index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = item["center"]
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.32
            if item["id"] in reviewed:
                marker.color.r, marker.color.g, marker.color.b = 0.1, 0.85, 0.2
            else:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.55, 0.05
            marker.color.a = 0.75
            markers.markers.append(marker)
        if target is not None:
            marker = Marker()
            marker.header = clear.header
            marker.ns = "coverage_target"
            marker.id = 2
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = target.target
            marker.pose.position.z = 0.15
            yaw = 0.0 if target.look_at is None else math.atan2(target.look_at[1] - target.target[1], target.look_at[0] - target.target[0])
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            marker.scale.x, marker.scale.y, marker.scale.z = 0.65, 0.12, 0.12
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.9, 0.15, 0.85, 0.95
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def _publish_coverage_layers(self):
        """Render laser-only/camera-only/both coverage without mutating maps."""
        with self.lock:
            grid = self.grid
            gate = self.gate_source
            pose = self.pose
            world_pose = self.world_pose
            points = tuple(self.camera_points_world)
            plan = self.current_plan
        if grid is None or gate is None or plan is None:
            return
        extent = infer_task_extent(
            grid,
            gate[:2],
            gate[2],
            self.planner.forward_depth,
            self.planner.back_extension,
            self.planner.lateral_half_width,
            self.planner.corridor_half_width,
        )
        task = task_region_mask(
            grid,
            gate[:2],
            gate[2],
            self.planner.back_extension,
            extent.forward_limit,
            self.planner.lateral_half_width,
            self.planner.corridor_half_width,
            gate_half_width=self.virtual_gate_half_width,
            gate_depth=self.virtual_gate_depth,
        )
        if self.task_entry_buffer > 0.0:
            rows, columns = np.indices(grid.data.shape, dtype=np.float64)
            x = grid.origin_x + (columns + 0.5) * grid.resolution
            y = grid.origin_y + (rows + 0.5) * grid.resolution
            along = (
                (x - gate[0]) * math.cos(gate[2])
                + (y - gate[1]) * math.sin(gate[2])
            )
            task &= along >= self.task_entry_buffer
        camera_seen = (
            self._camera_seen_grid(grid, pose, world_pose, points)
            if pose is not None and world_pose is not None
            else np.zeros(grid.data.shape, dtype=bool)
        )
        layers = coverage_classification(
            grid,
            task,
            camera_seen,
            robot_radius=self.robot_radius,
            safety_margin=self.safety_margin,
        )
        message = MarkerArray()
        clear = Marker()
        clear.header.frame_id = grid.frame_id
        clear.header.stamp = rospy.Time.now()
        clear.action = Marker.DELETEALL
        message.markers.append(clear)
        styles = {
            50: ("laser_only", 0.15, 0.35, 1.0, 0.30),
            75: ("camera_only", 0.10, 0.95, 0.35, 0.34),
            100: ("laser_and_camera", 0.95, 0.85, 0.10, 0.28),
        }
        for value, (namespace, red, green, blue, alpha) in styles.items():
            rows, columns = np.nonzero(layers == value)
            marker = Marker()
            marker.header = clear.header
            marker.ns = namespace
            marker.id = int(value)
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = grid.resolution
            marker.scale.z = 0.018
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                red,
                green,
                blue,
                alpha,
            )
            marker.points = [
                Point(
                    *grid.cell_center(int(row), int(column)),
                    0.045,
                )
                for row, column in zip(rows, columns)
            ]
            message.markers.append(marker)
        self.coverage_layers_pub.publish(message)

    def _publish_status(self):
        with self.lock:
            snapshot = self.latest_snapshot
            progress = 0.0
            if self.forward_anchor is not None and self.pose is not None:
                progress = (
                    (self.pose[0] - self.forward_anchor[0]) * math.cos(self.forward_yaw)
                    + (self.pose[1] - self.forward_anchor[1]) * math.sin(self.forward_yaw)
                )
            target = self.active_target
            stable = self._stable_spheres()
            unreviewed = [item["id"] for item in stable if item["id"] not in self.reviewed_hypotheses]
            camera_ready = self._camera_exploration_ready_locked()
            rear_transit_progress = 0.0
            along, _lateral = self._topology_coordinates()
            if (
                self.rear_transit_active
                and along is not None
                and self.rear_transit_start_along is not None
            ):
                rear_transit_progress = max(
                    0.0, float(along) - float(self.rear_transit_start_along)
                )
            extent = self.current_plan
            room_coverages = {}
            observed_portals = []
            if extent is not None:
                actionable_ids = {
                    item.topology_id for item in extent.actionable_portals
                }
                for topology_id, local in (extent.topology_coverages or {}).items():
                    room_coverages[str(topology_id)] = {
                        "laser": float(local.laser),
                        "camera": float(local.camera),
                        "combined": float(local.combined),
                        "task_cells": int(local.task_cells),
                    }
                observed_portals = [
                    {
                        "topology_id": item.topology_id,
                        "side": item.side,
                        "along": float(item.along),
                        "width": float(item.width),
                        "evidence": int(self.portal_evidence.get(item.topology_id, 0)),
                        "confirmed": bool(
                            int(self.portal_evidence.get(item.topology_id, 0))
                            >= self.portal_confirm_cycles
                        ),
                        "actionable": item.topology_id in actionable_ids,
                    }
                    for item in extent.observed_portals
                ]
            payload = {
                "floor_index": self.floor_index,
                "state": "FLOOR_COMPLETE" if self.floor_complete else "INITIAL_FORWARD" if self.gate_source is None else "COVERAGE_EXPLORATION",
                "topology_region": self.topology_region,
                "initial_forward_active": self.gate_source is None,
                "initial_forward_distance": self.initial_forward_distance,
                "initial_forward_progress": max(0.0, progress),
                "plane_policy_active": self.plane_policy_active,
                "camera_exploration_active": camera_ready,
                "virtual_isolation_door": list(self.gate_world) if self.gate_world is not None else None,
                "virtual_isolation_door_source": list(self.gate_source)
                if self.gate_source is not None else None,
                "elevator_portal": {
                    "id": self.elevator_portal.topology_id,
                    "along": self.elevator_portal.along,
                    "lateral": self.elevator_portal.lateral,
                    "width": self.elevator_portal.width,
                    "evidence": self.elevator_portal_evidence,
                } if self.elevator_portal is not None else None,
                "elevator_portal_world": list(self.elevator_portal_world)
                if self.elevator_portal_world is not None else None,
                "virtual_gate_half_width": self.virtual_gate_half_width,
                "virtual_gate_width": 2.0 * self.virtual_gate_half_width,
                "virtual_gate_depth": self.virtual_gate_depth,
                "show_room_virtual_doors": self.show_room_virtual_doors,
                "active_target_kind": target.kind if target is not None else None,
                "active_target": list(target.target) if target is not None else None,
                "active_target_topology": target.topology_id if target is not None else None,
                "topology_lock": self.topology_lock,
                "returning_topology": self.returning_topology,
                "front_station_along": self.front_station_along,
                "front_station_topologies": sorted(self.front_station_topologies),
                "front_rooms_complete": bool(
                    self.completed_front_sides == {"L", "R"}
                ),
                "completed_front_sides": sorted(self.completed_front_sides),
                "rear_transit_active": self.rear_transit_active,
                "rear_transit_distance": self.rear_transit_distance,
                "rear_transit_progress": rear_transit_progress,
                "rear_rooms_unlocked": self.rear_rooms_unlocked,
                "door_search_active": self.door_search_active,
                "door_search_phase": self.door_search_phase,
                "door_search_step_distance": self.door_search_step_distance,
                "door_search_travel": self.door_search_travel,
                "door_search_limit": (
                    self.door_search_rear_limit
                    if self.door_search_phase == "REAR"
                    else self.door_search_front_limit
                ),
                "completed_topologies": sorted(self.completed_topologies),
                "topology_states": self.topology_states,
                "portal_evidence": dict(self.portal_evidence),
                "portal_confirm_cycles": self.portal_confirm_cycles,
                "room_coverages": room_coverages,
                "observed_portals": observed_portals,
                "actionable_portals": [
                    item.topology_id for item in (extent.actionable_portals if extent is not None else ())
                ],
                "planner_diagnostics": dict(extent.diagnostics or {})
                if extent is not None
                else {},
                "target_replacements": self.target_replacements,
                "targets_reached": self.targets_reached,
                "laser_coverage": snapshot.laser if snapshot is not None else 0.0,
                "camera_coverage": snapshot.camera if snapshot is not None else 0.0,
                "combined_coverage": snapshot.combined if snapshot is not None else 0.0,
                "laser_coverage_target": self.laser_coverage_target,
                "camera_coverage_target": self.camera_coverage_target,
                "room_laser_coverage_target": self.room_laser_coverage_target,
                "room_camera_coverage_target": self.room_camera_coverage_target,
                "room_combined_coverage_target": self.room_combined_coverage_target,
                "expected_rooms_per_floor": self.expected_rooms_per_floor,
                "combined_coverage_target": self.combined_coverage_target,
                "camera_weight": self.camera_weight,
                "coverage_exclusion_clearance": self.robot_radius + self.safety_margin,
                "task_entry_buffer": self.task_entry_buffer,
                "navigation_hard_clearance": self.navigation_clearance,
                "navigation_preferred_clearance": self.preferred_clearance,
                "navigation_reachable_cells": extent.navigation_reachable_cells if extent is not None else 0,
                "navigation_blocks": self.navigation_blocks,
                "plan_cycles": self.plan_cycles,
                "plan_failures": self.plan_failures,
                "last_plan_reason": self.last_plan_reason,
                "last_plan_error": self.last_plan_error,
                "candidate_topologies": list(extent.candidate_topologies) if extent is not None else [],
                "task_forward_limit": extent.task_forward_limit if extent is not None else self.planner.forward_depth,
                "task_extent_confident": extent.task_extent_confident if extent is not None else False,
                "sphere_hypotheses": len(stable),
                "unreviewed_sphere_hypotheses": unreviewed,
                "cmd_vel": {"linear_x": self.last_command[0], "angular_z": self.last_command[1]},
            }
        message = String(data=json.dumps(payload, sort_keys=True))
        self.status_pub.publish(message)
        self.coverage_pub.publish(message)

    def _shutdown(self):
        self.control_timer.shutdown()
        self.plan_timer.shutdown()
        self._stop()


if __name__ == "__main__":
    rospy.init_node("coverage_explorer")
    CoverageExplorer()
    rospy.spin()
