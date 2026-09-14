#!/usr/bin/env python3
"""Room-oriented single-floor explorer with explicit doorway crossing."""

import json
import math
from dataclasses import replace
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rospkg
import rospy
import tf.transformations as transformations
import tf2_ros
from building_generator_interfaces.srv import SetDoorState
from geometry_msgs.msg import Point, Point32, PolygonStamped, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(rospkg.RosPack().get_path("simnav")) / "scripts"))

from room_explorer_core import (  # noqa: E402
    ABORT_RUN,
    ALIGN_TO_DOOR_NORMAL,
    BAD,
    DEGRADED,
    DOOR_CROSSING,
    DISCOVERED,
    DoorCandidateManager,
    DoorEvidence,
    DoorFusion,
    DoorVerifier,
    EXIT_ROOM,
    GOOD,
    GO_TO_PRE_DOOR,
    GridView,
    INCONCLUSIVE,
    LocalizationHealthMonitor,
    MissionFaultMonitor,
    MAP,
    MISSION_FAULT,
    OpeningDetector,
    PENDING,
    RoomFrontierPlanner,
    ROOM_SCAN,
    REJECTED,
    SCAN,
    STALE,
    UNREACHABLE,
    VISITED,
    CorridorEstimator,
    door_discovery_allowed,
    door_corridor_side_reached,
    door_crossing_confirmed,
    door_crossing_errors,
    door_interior_reached,
    corridor_timeout_requires_reversal,
    finite_median_clearance,
    finite_percentile_clearance,
    corridor_approach_motion,
    corridor_steering_correction,
    local_loop_icp,
    closed_opening_measurement,
    normalize_angle,
    opposite_door_at_station,
    point_in_polygon,
    point_beyond_topology_gate,
    projected_travel,
    transform_planar_point,
    transform_planar_yaw,
    voxel_downsample_2d,
)
from pose_continuity import constrain_to_corridor  # noqa: E402


class SingleFloorExplorer:
    def __init__(self):
        self.lock = threading.Lock()
        self.world_frame = rospy.get_param("~world_frame", "world")
        self.map_frame = rospy.get_param("~map_frame", "simnav_map")
        self.odom_frame = rospy.get_param("~odom_frame", "simnav_odom")
        self.grid = None
        self.last_map_update = rospy.Time(0)
        self.pose = None
        self.last_scan_update = rospy.Time(0)
        self.localization_health = STALE
        self.localization_monitor = LocalizationHealthMonitor(
            stale_timeout=float(rospy.get_param("~localization_timeout", 1.0)),
            degraded_translation=float(
                rospy.get_param("~localization_degraded_translation", 0.35)
            ),
            bad_translation=float(rospy.get_param("~localization_bad_translation", 1.0)),
            degraded_rotation=float(
                rospy.get_param("~localization_degraded_rotation", 0.50)
            ),
            bad_rotation=float(rospy.get_param("~localization_bad_rotation", 1.20)),
        )
        self.defer_polygons = []
        anchor_x = float(rospy.get_param("~corridor_anchor_x", float("nan")))
        anchor_y = float(rospy.get_param("~corridor_anchor_y", float("nan")))
        self.configured_corridor_anchor = (
            (anchor_x, anchor_y) if math.isfinite(anchor_x) and math.isfinite(anchor_y) else None
        )
        self.configured_corridor_yaw = float(
            rospy.get_param("~corridor_yaw", math.pi / 2.0)
        )
        self.corridor_anchor = None
        self.corridor_yaw = self.configured_corridor_yaw
        self.corridor_frame_initialized = False
        # The bounded start edge is a distinct topology region.  Its origin
        # separates the entrance from the lobby and its endpoint separates
        # the lobby from the main corridor.  Door candidates only exist on
        # the forward side of the second directed gate.
        self.initial_forward_distance = max(
            0.0, float(rospy.get_param("~initial_forward_distance", 14.5))
        )
        self.initial_forward_speed = max(
            0.05, float(rospy.get_param("~initial_forward_speed", 0.60))
        )
        self.initial_centering_start_distance = max(
            0.0,
            float(rospy.get_param("~initial_centering_start_distance", 10.5)),
        )
        self.initial_forward_active = False
        self.initial_forward_anchor = None
        self.initial_forward_yaw = None
        self.lobby_entry_door = None
        self.virtual_isolation_door = None
        self.topology_region = "LOBBY_TRANSIT"
        self.topology_gate_candidate_margin = max(
            0.0, float(rospy.get_param("~topology_gate_candidate_margin", 0.25))
        )
        self.topology_gate_return_margin = max(
            0.0, float(rospy.get_param("~topology_gate_return_margin", 0.75))
        )
        self.topology_gate_width = max(
            0.5, float(rospy.get_param("~topology_gate_width", 2.2))
        )
        self.topology_rejected_evidence = 0
        self.require_elongated_corridor = bool(
            rospy.get_param("~require_elongated_corridor", True)
        )
        self.corridor_context_confirmed = bool(
            rospy.get_param("~initial_corridor_context", False)
        )
        self.initial_forward_active = bool(
            self.initial_forward_distance > 0.0
            and not self.corridor_context_confirmed
        )
        if not self.initial_forward_active:
            self.topology_region = "MAIN_CORRIDOR"
        self.corridor_evidence_length = float(
            rospy.get_param("~corridor_evidence_length", 5.0)
        )
        self.corridor_min_wall_support = float(
            rospy.get_param("~corridor_min_wall_support", 0.40)
        )
        self.corridor_progress_step = float(rospy.get_param("~corridor_progress_step", 5.0))
        self.max_corridor_progress = float(rospy.get_param("~max_corridor_progress", 30.0))
        self.corridor_progress_speed = float(rospy.get_param("~corridor_progress_speed", 0.45))
        self.corridor_turn_speed = float(rospy.get_param("~corridor_turn_speed", 1.0))
        self.corridor_heading_timeout = float(
            rospy.get_param("~corridor_heading_timeout", 18.0)
        )
        self.corridor_progress_timeout = float(rospy.get_param("~corridor_progress_timeout", 20.0))
        self.prefer_opposite_door = bool(
            rospy.get_param("~prefer_opposite_door", True)
        )
        self.prefer_paired_station = bool(
            rospy.get_param("~prefer_paired_station", False)
        )
        self.opposite_pair_station_tolerance = float(
            rospy.get_param("~opposite_pair_station_tolerance", 1.5)
        )
        self.opposite_door_probe_duration = float(
            rospy.get_param("~opposite_door_probe_duration", 2.0)
        )
        self.opposite_door_probe_until = rospy.Time(0)
        self.max_corridor_stall_windows = int(
            rospy.get_param("~max_corridor_stall_windows", 3)
        )
        self.corridor_stall_windows = 0
        self.corridor_stop_distance = float(rospy.get_param("~corridor_stop_distance", 0.80))
        self.corridor_scan_center_gain = float(
            rospy.get_param("~corridor_scan_center_gain", 0.12)
        )
        self.corridor_wall_avoid_distance = float(
            rospy.get_param("~corridor_wall_avoid_distance", 0.65)
        )
        self.corridor_wall_avoid_speed = float(
            rospy.get_param("~corridor_wall_avoid_speed", 0.20)
        )
        self.corridor_turn_clearance = float(rospy.get_param("~corridor_turn_clearance", 1.20))
        self.corridor_backoff_distance = float(rospy.get_param("~corridor_backoff_distance", 0.50))
        self.corridor_backoff_speed = float(rospy.get_param("~corridor_backoff_speed", 0.60))
        self.corridor_backoff_timeout = float(rospy.get_param("~corridor_backoff_timeout", 6.0))
        self.max_corridor_sweeps = int(rospy.get_param("~max_corridor_sweeps", 1))
        self.corridor_sweep_count = 0
        self.corridor_progress_start = None
        self.corridor_advancing = False
        self.front_clearance = float("inf")
        self.front_nearest_clearance = float("inf")
        self.left_clearance = 0.0
        self.right_clearance = 0.0
        self.narrow_left_clearance = 0.0
        self.narrow_right_clearance = 0.0
        self.left_nearest_clearance = 0.0
        self.right_nearest_clearance = 0.0
        self.corridor_estimator = CorridorEstimator(
            min_width=float(rospy.get_param("~scan_corridor_min_width", 1.7)),
            max_width=float(rospy.get_param("~scan_corridor_max_width", 2.7)),
        )
        self.corridor_scan_timeout = float(
            rospy.get_param("~corridor_scan_timeout", 0.5)
        )
        self.corridor_estimate = self.corridor_estimator.estimate(
            self.corridor_yaw, float("nan"), float("nan"), float("inf")
        )
        self.scan_open_threshold = float(rospy.get_param("~scan_open_threshold", 2.5))
        self.scan_open_min_width = float(rospy.get_param("~scan_open_min_width", 0.8))
        self.scan_open_max_width = float(rospy.get_param("~scan_open_max_width", 1.6))
        self.scan_leading_wall_min_length = float(
            rospy.get_param("~scan_leading_wall_min_length", 0.5)
        )
        self.scan_wall_distance = {-1.0: 1.1, 1.0: 1.1}
        self.scan_leading_wall_start = {-1.0: None, 1.0: None}
        self.scan_open_start = {-1.0: None, 1.0: None}
        self.scan_open_start_pose = {-1.0: None, 1.0: None}
        self.scan_open_samples = {-1.0: 0, 1.0: 0}
        self.scan_open_provisional_emitted = {-1.0: False, 1.0: False}
        self.scan_provisional_openings = bool(
            rospy.get_param("~scan_provisional_openings", True)
        )
        self.scan_provisional_samples = max(
            1, int(rospy.get_param("~scan_provisional_samples", 2))
        )
        self.scan_expected_door_width = max(
            self.scan_open_min_width,
            min(
                self.scan_open_max_width,
                float(rospy.get_param("~scan_expected_door_width", 1.2)),
            ),
        )
        self.pending_scan_evidence = []
        self.scan_opening_hold = False
        self.scan_corridor_start = None
        self.metric_corridor_anchor = None
        self.scan_corridor_evidence_length = float(
            rospy.get_param("~scan_corridor_evidence_length", 2.0)
        )
        self.scan_corridor_min_width = float(
            rospy.get_param("~scan_corridor_min_width", 1.7)
        )
        self.scan_corridor_max_width = float(
            rospy.get_param("~scan_corridor_max_width", 2.7)
        )
        self.pending_corridor_turn = None
        self.pending_corridor_preserve_context = False
        self.corridor_backoff_origin = None
        self.corridor_exhausted = False
        configured_room_count = rospy.get_param("~expected_rooms_per_floor", None)
        if configured_room_count is None:
            configured_room_count = rospy.get_param("~expected_room_count", 4)
        self.expected_rooms_per_floor = max(1, int(configured_room_count))
        # Keep the legacy attribute for older launch files and diagnostics.
        self.expected_room_count = self.expected_rooms_per_floor
        self.entry_validation_mode = bool(
            rospy.get_param("~entry_validation_mode", False)
        )
        self.entry_validation_hold = max(
            0.25, float(rospy.get_param("~entry_validation_hold", 0.75))
        )
        self.detector = OpeningDetector(
            min_width=float(rospy.get_param("~opening_min_width", 0.8)),
            max_width=float(rospy.get_param("~opening_max_width", 1.6)),
            pre_distance=float(rospy.get_param("~pre_door_distance", 0.65)),
            post_distance=float(rospy.get_param("~post_door_distance", 1.0)),
            observation_range=float(rospy.get_param("~opening_observation_range", 4.0)),
            min_jamb_wall_length=float(
                rospy.get_param("~opening_min_jamb_wall_length", 0.65)
            ),
            min_jamb_wall_support=float(
                rospy.get_param("~opening_min_jamb_wall_support", 0.55)
            ),
        )
        self.door_fusion = DoorFusion(
            spatial_gate=float(rospy.get_param("~fusion_spatial_gate", 2.0)),
            normal_gate=float(rospy.get_param("~fusion_normal_gate", 0.35)),
            strong_confidence=float(rospy.get_param("~fusion_strong_confidence", 0.75)),
            medium_confidence=float(rospy.get_param("~fusion_medium_confidence", 0.45)),
            require_scan_verification=bool(
                rospy.get_param("~door_scan_requires_verification", True)
            ),
        )
        self.door_scan_requires_verification = bool(
            rospy.get_param("~door_scan_requires_verification", True)
        )
        self.mission = DoorCandidateManager(
            max_attempts=int(rospy.get_param("~max_door_attempts", 2)),
            max_exit_attempts=int(rospy.get_param("~max_exit_attempts", 2)),
            max_sweeps=self.max_corridor_sweeps,
            quiet_period=float(rospy.get_param("~completion_quiet_period", 3.0)),
            dedup_radius=float(rospy.get_param("~opening_dedup_radius", 2.0)),
            visited_revisit_radius=float(
                rospy.get_param("~visited_door_revisit_radius", 3.0)
            ),
            visited_revisit_normal_gate=float(
                rospy.get_param("~visited_door_revisit_normal_gate", 0.70)
            ),
            opposite_pair_station_tolerance=self.opposite_pair_station_tolerance,
        )
        self.state_start = rospy.Time(0)
        self.state_start_yaw = 0.0
        self.scan_phase = "ROOM_FRONTIER_EXPLORE"
        self.scan_phase_start = rospy.Time(0)
        self.danger_phase_ids = {}
        self.room_valid_frame_count = 0
        self.room_exploration_start_time = None
        self.room_entry_source_pose = None
        self.room_entry_metric_pose = None
        self.room_entry_door_source = None
        self.room_entry_inward_yaw_source = None
        self.room_frontier_plan = None
        self.room_frontier_target_source = None
        self.room_frontier_target_metric = None
        self.room_frontier_path_source = ()
        self.room_frontier_path_index = 0
        self.room_frontier_plan_time = rospy.Time(0)
        self.room_frontier_planned_map_update = rospy.Time(0)
        self.room_frontier_quiet_since = None
        self.room_frontier_visited_targets = []
        self.room_frontier_replans = 0
        self.room_frontier_targets_reached = 0
        self.room_frontier_last_reason = "NOT_STARTED"
        self.room_frontier_coverage = 0.0
        self.room_frontier_raw_coverage = 0.0
        self.room_frontier_coverage_target = float(
            rospy.get_param("~room_frontier_coverage_target", 0.80)
        )
        self.camera_coverage_enabled = bool(
            rospy.get_param("~camera_coverage_enabled", True)
        )
        self.camera_coverage_target = max(
            0.0, min(1.0, float(rospy.get_param("~camera_coverage_target", 0.70)))
        )
        self.camera_coverage_resolution = max(
            0.05, float(rospy.get_param("~camera_coverage_resolution", 0.25))
        )
        self.camera_room_id = None
        self.camera_observed_world_points = set()
        self.camera_observed_update = rospy.Time(0)
        self.camera_observed_count = 0
        self.camera_observed_ray_count = 0
        self.room_frontier_planned_camera_update = rospy.Time(0)
        self.camera_sweep_enabled = bool(
            rospy.get_param("~camera_sweep_enabled", True)
        )
        self.camera_sweep_speed = max(
            0.05, float(rospy.get_param("~camera_sweep_speed", 0.35))
        )
        self.camera_sweep_max_angle = max(
            0.0, float(rospy.get_param("~camera_sweep_max_angle", 2.0 * math.pi))
        )
        self.camera_sweep_active = False
        self.camera_sweep_complete = False
        self.camera_sweep_last_yaw = None
        self.camera_sweep_accumulated = 0.0
        self.room_frontier_stats = {}
        self.room_danger_counts = {}
        self.exit_danger_id_baseline = set()
        self.confirmed_danger_count = 0
        self.confirmed_danger_positions = []
        self.confirmed_danger_ids = set()
        self.room_danger_id_baselines = {}
        self.confirmed_danger_tracks = {}
        self.room_entry_danger_ids = set()
        self.world_pose = None
        self.loop_closure_snapshot = {}
        self.latest_scan_points_base = np.empty((0, 2), dtype=np.float64)
        self.latest_door_points_base = np.empty((0, 3), dtype=np.float64)
        self.latest_door_points_stamp = rospy.Time(0)
        self.door_verifier_request_until = rospy.Time(0)
        self.door_verifier_enabled = bool(
            rospy.get_param("~door_verifier_enabled", True)
        )
        self.door_verifier_max_age = float(
            rospy.get_param("~door_verifier_max_age", 0.75)
        )
        self.door_verifier_stride = max(
            1, int(rospy.get_param("~door_verifier_stride", 4))
        )
        self.door_verifier_max_points = max(
            100, int(rospy.get_param("~door_verifier_max_points", 6000))
        )
        self.door_verifier = DoorVerifier(
            min_points=int(rospy.get_param("~door_verifier_min_points", 16)),
            side_tolerance=float(rospy.get_param("~door_verifier_side_tolerance", 0.28)),
            depth_tolerance=float(rospy.get_param("~door_verifier_depth_tolerance", 0.35)),
            minimum_jamb_height=float(rospy.get_param("~door_verifier_min_jamb_height", 0.80)),
        )
        self.door_verifier_tf = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.door_verifier_tf_listener = tf2_ros.TransformListener(self.door_verifier_tf)
        self.doorway_loop_anchors = {}
        self.room_entry_loop_anchor = None
        self.scan_accumulated_points = np.empty((0, 2), dtype=np.float64)
        self.scan_accumulation_max_points = max(
            500, int(rospy.get_param("~scan_accumulation_max_points", 12000))
        )
        self.loop_voxel_size = float(rospy.get_param("~loop_voxel_size", 0.10))
        self.loop_max_correspondence = float(
            rospy.get_param("~loop_max_correspondence", 0.60)
        )
        self.loop_min_overlap = float(rospy.get_param("~loop_min_overlap", 0.35))
        self.loop_max_rmse = float(rospy.get_param("~loop_max_rmse", 0.18))
        self.loop_max_translation = float(
            rospy.get_param("~loop_max_translation", 1.0)
        )
        self.loop_max_rotation = float(rospy.get_param("~loop_max_rotation", 0.12))
        self.goal_state = None
        self.pre_door_timeout = float(rospy.get_param("~pre_door_timeout", 20.0))
        self.pre_door_speed = float(rospy.get_param("~pre_door_speed", 0.45))
        self.align_timeout = float(rospy.get_param("~align_timeout", 10.0))
        self.align_speed = float(rospy.get_param("~align_speed", 0.40))
        # A wide doorway does not require a near-perfect heading before the
        # robot starts its straight normal crossing.  Keep this bounded and
        # configurable so the dynamic test can trade time for margin.
        self.door_alignment_tolerance = float(
            rospy.get_param("~door_alignment_tolerance", 0.12)
        )
        self.crossing_timeout = float(rospy.get_param("~max_cross_time", 14.0))
        self.minimum_cross_time = float(rospy.get_param("~minimum_cross_time", 1.5))
        self.inside_confirm_duration = float(
            rospy.get_param("~inside_room_confirm_duration", 0.5)
        )
        self.inside_room_entry_depth = float(
            rospy.get_param("~inside_room_entry_depth", 0.65)
        )
        self.door_centering_tolerance = max(
            0.05, float(rospy.get_param("~door_centering_tolerance", 0.20))
        )
        self.door_crossing_lateral_tolerance = max(
            0.05,
            float(rospy.get_param("~door_crossing_lateral_tolerance", 0.30)),
        )
        self.door_crossing_minimum_travel = max(
            0.10, float(rospy.get_param("~door_crossing_minimum_travel", 0.75))
        )
        self.door_centering_yaw_gain = max(
            0.0, float(rospy.get_param("~door_centering_yaw_gain", 0.90))
        )
        self.pre_door_phase = "IDLE"
        self.pre_door_station_error = None
        self.pre_door_normal_error = None
        self.pre_door_face_stable_since = None
        self.pre_door_face_stable_duration = max(
            0.0, float(rospy.get_param("~pre_door_face_stable_duration", 0.25))
        )
        self.door_crossing_origin_metric = None
        self.door_crossing_last_depth = None
        self.door_crossing_last_lateral = None
        self.door_crossing_last_travel = None
        # Give the mapper a short, bounded observation interval after a room
        # crossing.  This is only used when the entry cell is not connected
        # to the currently observed laser free space yet.
        self.room_entry_advance_distance = max(
            0.0, float(rospy.get_param("~room_entry_advance_distance", 1.0))
        )
        self.room_entry_advance_min_reachable_cells = max(
            1, int(rospy.get_param("~room_entry_advance_min_reachable_cells", 12))
        )
        self.max_exit_time = float(rospy.get_param("~max_exit_time", 14.0))
        self.minimum_exit_time = float(rospy.get_param("~minimum_exit_time", 1.0))
        self.corridor_side_entry_depth = float(
            rospy.get_param("~corridor_side_entry_depth", 0.45)
        )
        self.corridor_reacquire_duration = float(
            rospy.get_param("~corridor_reacquire_duration", 0.7)
        )
        self.room_motion_stop_distance = float(
            rospy.get_param("~room_motion_stop_distance", 0.45)
        )
        self.room_frontier_replan_period = float(
            rospy.get_param("~room_frontier_replan_period", 1.0)
        )
        self.room_frontier_target_tolerance = float(
            rospy.get_param("~room_frontier_target_tolerance", 0.35)
        )
        self.room_frontier_quiet_period = float(
            rospy.get_param("~room_frontier_quiet_period", 2.0)
        )
        # Small map-update hysteresis prevents a new wall cell beside a
        # committed path from causing target churn while retaining the hard
        # occupied-cell check.
        self.room_frontier_revalidate_margin = float(
            rospy.get_param("~room_frontier_revalidate_margin", 0.01)
        )
        self.room_frontier_speed = float(
            rospy.get_param("~room_frontier_speed", 0.45)
        )
        self.room_frontier_min_speed = float(
            rospy.get_param("~room_frontier_min_speed", 0.30)
        )
        self.room_frontier_heading_tolerance = float(
            rospy.get_param("~room_frontier_heading_tolerance", 0.25)
        )
        self.room_robot_radius = float(rospy.get_param("~room_robot_radius", 0.38))
        self.room_frontier_planner = RoomFrontierPlanner(
            robot_radius=self.room_robot_radius,
            safety_margin=float(rospy.get_param("~room_safety_margin", 0.04)),
            frontier_cluster_radius=float(
                rospy.get_param("~room_frontier_cluster_radius", 0.45)
            ),
            frontier_min_cluster_cells=int(
                rospy.get_param("~room_frontier_min_cluster_cells", 3)
            ),
            target_revisit_radius=float(
                rospy.get_param("~room_frontier_revisit_radius", 0.75)
            ),
            topology_entry_margin=float(
                rospy.get_param("~room_topology_entry_margin", 0.35)
            ),
            topology_max_depth=float(
                rospy.get_param("~room_topology_max_depth", 12.0)
            ),
            topology_lateral_limit=float(
                rospy.get_param("~room_topology_lateral_limit", 8.0)
            ),
            dead_end_path_threshold=float(
                rospy.get_param("~room_dead_end_path_threshold", 1.5)
            ),
            minimum_escape_cells=int(
                rospy.get_param("~room_minimum_escape_cells", 12)
            ),
            information_gain_weight=float(
                rospy.get_param("~room_information_gain_weight", 1.0)
            ),
            dead_end_weight=float(
                rospy.get_param("~room_dead_end_weight", 1.5)
            ),
            information_radius=float(
                rospy.get_param("~room_information_radius", 2.5)
            ),
            visual_ring_enabled=bool(
                rospy.get_param("~camera_ring_enabled", True)
            ),
            visual_ring_clockwise=bool(
                rospy.get_param("~camera_ring_clockwise", True)
            ),
            visual_ring_max_step=float(
                rospy.get_param("~camera_ring_max_step", 3.0)
            ),
            visual_ring_min_obstacle_cells=int(
                rospy.get_param("~camera_ring_min_obstacle_cells", 8)
            ),
        )
        self.crossing_speed = float(rospy.get_param("~crossing_speed", 0.18))
        self.inside_invalid_start = None
        self.crossing_corridor_seen = False
        self.corridor_reacquire_start = None
        self.danger_confirmation_active = False
        self.controller_health = ""
        self.lio_health = "STALE"
        self.lio_effective_points = None
        self.lio_health_seen = False
        self.mission_fault = None
        self.fault_monitor = MissionFaultMonitor(
            bad_localization_duration=float(
                rospy.get_param("~fault_bad_localization_duration", 1.0)
            ),
            low_points_duration=float(rospy.get_param("~fault_low_points_duration", 1.0)),
            minimum_effective_points=int(
                rospy.get_param("~fault_minimum_effective_points", 5)
            ),
        )
        self.last_pose_metric = None
        self.last_pose_delta = (0.0, 0.0)
        self.finite_scan_count = 0
        self.corridor_opening_state = "CLOSED"
        self.last_command = (0.0, 0.0)
        self.enabled = bool(rospy.get_param("~enabled", True))
        self.control_enabled = bool(rospy.get_param("~control_enabled", True))
        self.command_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.status_pub = rospy.Publisher("/simnav/explorer_status", String, queue_size=5, latch=True)
        self.complete_pub = rospy.Publisher("/simnav/floor_complete", Bool, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/simnav/opening_markers", MarkerArray, queue_size=1, latch=True)
        self.room_frontier_marker_pub = rospy.Publisher(
            "/simnav/room_frontier_markers", MarkerArray, queue_size=1, latch=True
        )
        self.room_frontier_path_pub = rospy.Publisher(
            "/simnav/room_frontier_path", NavPath, queue_size=1, latch=True
        )
        self.gate_pub = rospy.Publisher("/simnav/entrance_gate", PolygonStamped, queue_size=1, latch=True)
        self.defer_pub = rospy.Publisher("/simnav/defer_zone", PolygonStamped, queue_size=10, latch=True)
        self.health_pub = rospy.Publisher(
            "/simnav/localization_health", String, queue_size=5, latch=True
        )
        self.corridor_pub = rospy.Publisher(
            "/simnav/corridor_estimate", String, queue_size=5, latch=True
        )
        self.evidence_pub = rospy.Publisher(
            "/simnav/door_evidence", String, queue_size=5
        )
        self.hypothesis_pub = rospy.Publisher(
            "/simnav/door_hypotheses", String, queue_size=5, latch=True
        )
        self.loop_closure_pub = rospy.Publisher(
            "/simnav/local_loop_closure_request", String, queue_size=5
        )
        self.metric_corridor_constraint_pub = rospy.Publisher(
            "/simnav/metric_corridor_constraint", String, queue_size=2
        )
        self.mission_fault_pub = rospy.Publisher(
            "/simnav/mission_fault", String, queue_size=2, latch=True
        )
        self.corridor_diagnostic_pub = rospy.Publisher(
            "/simnav/corridor_diagnostic", String, queue_size=10
        )
        self.room_entry_pub = rospy.Publisher(
            "/simnav/room_entry", String, queue_size=5, latch=True
        )
        rospy.Subscriber(
            "/simnav/camera_coverage", String, self._camera_coverage_callback, queue_size=2
        )
        rospy.Subscriber("/exploration_map", OccupancyGrid, self._map_callback, queue_size=1)
        rospy.Subscriber("/simnav/odom", Odometry, self._pose_callback, queue_size=10)
        rospy.Subscriber("/scan_2d", LaserScan, self._scan_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~door_verifier_cloud_topic", "/livox/Pointcloud2"),
            PointCloud2,
            self._door_cloud_callback,
            queue_size=1,
        )
        rospy.Subscriber("/simnav/defer_zone", PolygonStamped, self._defer_callback, queue_size=10)
        rospy.Subscriber(
            "/simnav/danger_confirmation_active",
            Bool,
            self._danger_confirmation_callback,
            queue_size=2,
        )
        rospy.Subscriber(
            "/simnav/danger_tracks",
            String,
            self._danger_tracks_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            "/simnav/danger_valid_frame",
            String,
            self._danger_valid_frame_callback,
            queue_size=20,
        )
        rospy.Subscriber(
            "/simnav/world_pose_metric", PoseStamped, self._world_pose_callback, queue_size=10
        )
        rospy.Subscriber(
            "/simnav/local_loop_closure_applied",
            String,
            self._loop_closure_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            "/simnav/controller_health", String,
            self._controller_health_callback, queue_size=2
        )
        rospy.Subscriber(
            "/simnav/lio_health", String,
            self._lio_health_callback, queue_size=5
        )
        self._publish_configured_entrance_gate()
        self._publish_configured_defer_zones()
        self._close_controlled_doors_if_requested()
        self.detect_timer = rospy.Timer(
            rospy.Duration(
                max(0.10, float(rospy.get_param("~door_detection_period", 0.25)))
            ),
            self._detect,
        )
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self._control)
        rospy.on_shutdown(self._shutdown)

    def _map_callback(self, message):
        data = np.asarray(message.data, dtype=np.int16).reshape(message.info.height, message.info.width)
        with self.lock:
            self.grid = GridView(
                data=data,
                resolution=message.info.resolution,
                origin_x=message.info.origin.position.x,
                origin_y=message.info.origin.position.y,
                frame_id=message.header.frame_id.strip() or self.map_frame,
            )
            self.last_map_update = (
                message.header.stamp if message.header.stamp != rospy.Time(0) else rospy.Time.now()
            )

    def _pose_callback(self, message):
        q = message.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = message.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        health = self.localization_monitor.update(
            stamp.to_sec(),
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw,
        )
        with self.lock:
            if self.last_pose_metric is not None:
                self.last_pose_delta = (
                    math.hypot(
                        message.pose.pose.position.x - self.last_pose_metric[0],
                        message.pose.pose.position.y - self.last_pose_metric[1],
                    ),
                    abs(normalize_angle(yaw - self.last_pose_metric[2])),
                )
            self.last_pose_metric = (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                yaw,
            )
            self.pose = (message.pose.pose.position.x, message.pose.pose.position.y, yaw)
            self.localization_health = health
            self._initialize_corridor_frame_locked()
        self._publish_localization_health()

    def _controller_health_callback(self, message):
        try:
            payload = json.loads(message.data)
            self.controller_health = str(payload.get("state", ""))
        except (TypeError, ValueError):
            self.controller_health = str(message.data or "")

    def _lio_health_callback(self, message):
        try:
            payload = json.loads(message.data)
            self.lio_health = str(payload.get("state", "STALE"))
            self.lio_effective_points = payload.get("effective_points")
            self.lio_health_seen = True
        except (TypeError, ValueError):
            self.lio_health = "STALE"
            self.lio_effective_points = None
            self.lio_health_seen = True

    def _danger_confirmation_callback(self, message):
        self.danger_confirmation_active = bool(message.data)

    def _danger_valid_frame_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not (
            payload.get("rgb_valid")
            and payload.get("depth_valid")
            and payload.get("tf_valid")
        ):
            return
        if self._danger_observation_phase() == "ROOM_FRONTIER":
            self.room_valid_frame_count += 1

    def _camera_coverage_callback(self, message):
        """Receive the current room's RGB-D line-of-sight cell summary.

        The detector publishes world-frame cell indices.  Keep metric cell
        centers here so the summary can be transformed into whatever frame
        the latest occupancy grid uses at planning time.
        """
        try:
            payload = json.loads(message.data)
            room_id = str(payload.get("room_id") or "")
            resolution = max(0.05, float(payload.get("resolution", self.camera_coverage_resolution)))
            cells = payload.get("cells", [])
            points = set()
            for cell in cells:
                if not isinstance(cell, (list, tuple)) or len(cell) < 2:
                    continue
                cell_x = int(cell[0])
                cell_y = int(cell[1])
                points.add(
                    (
                        (float(cell_x) + 0.5) * resolution,
                        (float(cell_y) + 0.5) * resolution,
                    )
                )
            ray_count = int(payload.get("ray_count", 0))
        except (TypeError, ValueError, OverflowError):
            return
        with self.lock:
            active = (
                self.mission.candidates.get(self.mission.active_id)
                if self.mission.active_id is not None
                else None
            )
            if active is None or room_id != active.candidate_id:
                return
            self.camera_room_id = room_id
            self.camera_observed_world_points = points
            self.camera_observed_count = len(points)
            self.camera_observed_ray_count = ray_count
            self.camera_observed_update = rospy.Time.now()

    def _camera_seen_grid_locked(self, points, grid):
        """Rasterize world-frame camera cells into the current map grid."""
        seen = np.zeros(grid.data.shape, dtype=bool)
        if not points:
            return seen
        transform = self._lookup_planar_transform_locked(self.world_frame, grid.frame_id)
        if transform is None:
            # The accelerated harness intentionally keeps source and map
            # coordinates aligned while TF is warming up.
            transform = (0.0, 0.0, 0.0)
            if self.pose is not None and self.world_pose is not None:
                transform = None
        cosine = math.cos(transform[2]) if transform is not None else None
        sine = math.sin(transform[2]) if transform is not None else None
        for point in points:
            if transform is not None:
                grid_x = transform[0] + cosine * point[0] - sine * point[1]
                grid_y = transform[1] + sine * point[0] + cosine * point[1]
            elif self.pose is not None and self.world_pose is not None:
                grid_x, grid_y = self._metric_point_to_source_locked(point)
            else:
                grid_x, grid_y = point
            # A summary item represents one camera-observation cell, not an
            # infinitesimal point.  Paint its metric footprint onto the finer
            # laser grid; marking only the center made a 0.25 m camera cell
            # count as one 0.10 m map pixel and capped measured coverage far
            # below the configured target.
            half_extent = 0.5 * self.camera_coverage_resolution
            min_column, min_row = grid.world_to_cell(
                grid_x - half_extent, grid_y - half_extent
            )
            max_column, max_row = grid.world_to_cell(
                grid_x + half_extent, grid_y + half_extent
            )
            row_start = max(0, min(min_row, max_row))
            row_stop = min(seen.shape[0] - 1, max(min_row, max_row))
            column_start = max(0, min(min_column, max_column))
            column_stop = min(seen.shape[1] - 1, max(min_column, max_column))
            if row_start <= row_stop and column_start <= column_stop:
                seen[
                    row_start : row_stop + 1,
                    column_start : column_stop + 1,
                ] = True
        return seen

    def _danger_tracks_callback(self, message):
        try:
            dangers = json.loads(message.data).get("dangers", [])
            previous_ids = set(self.confirmed_danger_ids)
            self.confirmed_danger_count = len(dangers)
            self.confirmed_danger_ids = {
                int(item["id"]) for item in dangers if "id" in item
            }
            self.confirmed_danger_positions = [
                tuple(float(value) for value in item["position_world"][:2])
                for item in dangers
                if len(item.get("position_world", [])) >= 2
            ]
            self.confirmed_danger_tracks = {
                int(item["id"]): tuple(
                    float(value) for value in item["position_world"][:2]
                )
                for item in dangers
                if "id" in item and len(item.get("position_world", [])) >= 2
            }
            newly_confirmed = self.confirmed_danger_ids - previous_ids
            if newly_confirmed:
                phase = self._danger_observation_phase()
                self.danger_phase_ids.setdefault(phase, set()).update(newly_confirmed)
        except (KeyError, TypeError, ValueError):
            return

    def _danger_observation_phase(self):
        active = (
            self.mission.candidates.get(self.mission.active_id)
            if self.mission.active_id is not None
            else None
        )
        if active is None:
            return "OUTSIDE_ROOM"
        if active.status == DOOR_CROSSING:
            return "INGRESS"
        if active.status == EXIT_ROOM:
            return "EGRESS"
        if active.status != ROOM_SCAN:
            return "OUTSIDE_ROOM"
        return "ROOM_FRONTIER" if self.scan_phase == "ROOM_FRONTIER_EXPLORE" else "INGRESS"

    def _world_pose_callback(self, message):
        q = message.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        with self.lock:
            self.world_pose = (message.pose.position.x, message.pose.position.y, yaw)
            self._initialize_corridor_frame_locked()

    def _initialize_corridor_frame_locked(self):
        if (
            self.corridor_frame_initialized
            or self.pose is None
            or self.world_pose is None
        ):
            return
        frame_yaw = normalize_angle(self.world_pose[2] - self.pose[2])
        self.corridor_yaw = normalize_angle(
            self.configured_corridor_yaw + frame_yaw
        )
        if self.configured_corridor_anchor is None:
            self.corridor_anchor = self.world_pose[:2]
        else:
            self.corridor_anchor = self._source_point_to_metric_locked(
                self.configured_corridor_anchor
            )
        self.metric_corridor_anchor = self.corridor_anchor
        self.corridor_frame_initialized = True

    def _source_point_to_metric_locked(self, point):
        return transform_planar_point(point, self.pose, self.world_pose)

    def _metric_point_to_source_locked(self, point):
        return transform_planar_point(point, self.world_pose, self.pose)

    def _source_yaw_to_metric_locked(self, yaw):
        return transform_planar_yaw(yaw, self.pose[2], self.world_pose[2])

    def _metric_yaw_to_source_locked(self, yaw):
        return transform_planar_yaw(yaw, self.world_pose[2], self.pose[2])

    def _lookup_planar_transform_locked(self, source_frame, target_frame):
        """Return (x, y, yaw) for source-frame coordinates in target frame."""
        source_frame = (source_frame or "").strip()
        target_frame = (target_frame or "").strip()
        if not source_frame or not target_frame or source_frame == target_frame:
            return 0.0, 0.0, 0.0
        try:
            transform = self.door_verifier_tf.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(0.08),
            )
        except Exception:
            rospy.logwarn_throttle(
                5.0,
                "Room frontier waiting for TF %s <- %s",
                target_frame,
                source_frame,
            )
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = transformations.euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )[2]
        return float(translation.x), float(translation.y), float(yaw)

    def _transform_planar_pose_locked(self, pose, source_frame, target_frame):
        transform = self._lookup_planar_transform_locked(source_frame, target_frame)
        if transform is None:
            return None
        tx, ty, transform_yaw = transform
        cosine = math.cos(transform_yaw)
        sine = math.sin(transform_yaw)
        return (
            tx + cosine * float(pose[0]) - sine * float(pose[1]),
            ty + sine * float(pose[0]) + cosine * float(pose[1]),
            normalize_angle(float(pose[2]) + transform_yaw),
        )

    def _source_pose_to_grid_locked(self, pose, grid_frame):
        transformed = self._transform_planar_pose_locked(
            pose, self.odom_frame, grid_frame
        )
        if transformed is not None:
            return transformed
        # During startup the accelerated offline harness can publish the map
        # before the bridge has emitted TF.  Its map and odometry coordinates
        # are intentionally numerically identical, so preserve that fallback.
        return tuple(pose)

    def _source_yaw_to_grid_locked(self, yaw, grid_frame):
        transformed = self._transform_planar_pose_locked(
            (0.0, 0.0, yaw), self.odom_frame, grid_frame
        )
        return transformed[2] if transformed is not None else float(yaw)

    def _metric_point_to_grid_locked(self, point, grid_frame):
        transformed = self._transform_planar_pose_locked(
            (point[0], point[1], 0.0), self.world_frame, grid_frame
        )
        if transformed is not None:
            return transformed[:2]
        if self.pose is not None and self.world_pose is not None:
            return self._metric_point_to_source_locked(point)
        return None

    def _grid_point_to_metric_locked(self, point, grid_frame):
        transformed = self._transform_planar_pose_locked(
            (point[0], point[1], 0.0), grid_frame, self.world_frame
        )
        if transformed is not None:
            return transformed[:2]
        if self.pose is not None and self.world_pose is not None:
            return self._source_point_to_metric_locked(point)
        return None

    def _loop_closure_callback(self, message):
        try:
            payload = json.loads(message.data)
            corrected = payload.get("corrected_pose")
        except (TypeError, ValueError):
            return
        self.loop_closure_snapshot = payload
        if not payload.get("accepted") or not corrected:
            return
        stamp = rospy.Time.now().to_sec()
        self.localization_monitor.rebase(
            stamp, float(corrected[0]), float(corrected[1]), float(corrected[2])
        )
        with self.lock:
            self.pose = (
                float(corrected[0]),
                float(corrected[1]),
                float(corrected[2]),
            )
            # A loop-closure correction changes the source-to-metric transform;
            # force a fresh frontier path instead of driving stale waypoints.
            if self.room_frontier_target_source is not None:
                self.room_frontier_target_source = None
                self.room_frontier_target_metric = None
                self.room_frontier_path_source = ()
                self.room_frontier_path_index = 0
                self.room_frontier_plan = None
                self.room_frontier_plan_time = rospy.Time(0)
                self.room_frontier_planned_map_update = rospy.Time(0)
                self.room_frontier_planned_camera_update = rospy.Time(0)

    def _defer_callback(self, message):
        polygon = [(point.x, point.y) for point in message.polygon.points]
        if len(polygon) >= 3:
            with self.lock:
                self.defer_polygons.append(polygon)

    def _door_cloud_callback(self, message):
        if not self.door_verifier_enabled:
            return
        with self.lock:
            if rospy.Time.now() > self.door_verifier_request_until:
                return
        raw = []
        for index, point in enumerate(
            point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True
            )
        ):
            if index % self.door_verifier_stride != 0:
                continue
            raw.append((float(point[0]), float(point[1]), float(point[2])))
            if len(raw) >= self.door_verifier_max_points:
                break
        if not raw:
            return
        points = np.asarray(raw, dtype=np.float64)
        stamp = message.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        frame_id = message.header.frame_id.strip()
        if frame_id not in ("", "base", "base_link"):
            try:
                transform = self.door_verifier_tf.lookup_transform(
                    "base", frame_id, stamp, rospy.Duration(0.05)
                )
            except Exception:
                rospy.logdebug_throttle(
                    5.0,
                    "Door verifier waiting for TF base <- %s",
                    frame_id,
                )
                return
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w
            rotation_matrix = np.asarray(
                [
                    [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
                    [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
                    [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
                ],
                dtype=np.float64,
            )
            points = np.matmul(points, rotation_matrix.T)
            points += np.asarray(
                [translation.x, translation.y, translation.z], dtype=np.float64
            )
        keep = (
            np.all(np.isfinite(points), axis=1)
            & (points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1] <= 64.0)
            & (points[:, 2] >= -0.5)
            & (points[:, 2] <= 3.0)
        )
        points = points[keep]
        with self.lock:
            self.latest_door_points_base = points
            self.latest_door_points_stamp = stamp

    def _scan_callback(self, message):
        half_angle = math.radians(20.0)
        front_ranges = []
        front_center_ranges = []
        left_ranges = []
        right_ranges = []
        narrow_left_ranges = []
        narrow_right_ranges = []
        scan_points = []
        for index, distance in enumerate(message.ranges):
            angle = message.angle_min + index * message.angle_increment
            angle = normalize_angle(angle)
            if (
                abs(angle) <= half_angle
                and math.isfinite(distance)
                and message.range_min <= distance <= message.range_max
            ):
                front_ranges.append(distance)
                if abs(angle) <= math.radians(10.0):
                    front_center_ranges.append(distance)
            side_distance = message.range_max if math.isinf(distance) else distance
            if math.isfinite(side_distance) and message.range_min <= side_distance <= message.range_max:
                if math.radians(30.0) <= angle <= math.radians(110.0):
                    left_ranges.append(side_distance)
                elif math.radians(-110.0) <= angle <= math.radians(-30.0):
                    right_ranges.append(side_distance)
            if math.isfinite(distance) and message.range_min <= distance <= message.range_max:
                if 0.30 <= distance <= min(12.0, message.range_max):
                    scan_points.append(
                        (distance * math.cos(angle), distance * math.sin(angle))
                    )
                if math.radians(75.0) <= angle <= math.radians(105.0):
                    narrow_left_ranges.append(distance)
                elif math.radians(-105.0) <= angle <= math.radians(-75.0):
                    narrow_right_ranges.append(distance)
        with self.lock:
            scan_stamp = message.header.stamp
            self.last_scan_update = (
                scan_stamp if scan_stamp != rospy.Time(0) else rospy.Time.now()
            )
            self.latest_scan_points_base = np.asarray(
                scan_points, dtype=np.float64
            ).reshape((-1, 2))
            self.finite_scan_count = len(scan_points)
            self.front_clearance = finite_percentile_clearance(front_ranges)
            self.front_nearest_clearance = (
                min(front_center_ranges) if front_center_ranges else float("inf")
            )
            self.left_clearance = self._sector_clearance(left_ranges)
            self.right_clearance = self._sector_clearance(right_ranges)
            self.left_nearest_clearance = min(left_ranges) if left_ranges else 0.0
            self.right_nearest_clearance = min(right_ranges) if right_ranges else 0.0
            self.narrow_left_clearance = self._narrow_clearance(narrow_left_ranges)
            self.narrow_right_clearance = self._narrow_clearance(narrow_right_ranges)
            center_error = 0.0
            if self.left_nearest_clearance > 0.0 and self.right_nearest_clearance > 0.0:
                center_error = 0.5 * (
                    self.right_nearest_clearance - self.left_nearest_clearance
                )
            self.corridor_estimate = self.corridor_estimator.estimate(
                self.corridor_yaw,
                self.left_nearest_clearance,
                self.right_nearest_clearance,
                self.front_clearance,
                center_error,
            )
            if not self.initial_forward_active:
                self._track_scan_openings_locked(
                    self.narrow_left_clearance,
                    self.narrow_right_clearance,
                    self.left_nearest_clearance,
                    self.right_nearest_clearance,
                )
            corridor_estimate = self.corridor_estimate
        self._publish_corridor_estimate(corridor_estimate)

    @staticmethod
    def _metric_point_to_base(point, pose):
        delta = np.asarray(point, dtype=np.float64) - np.asarray(pose[:2])
        cosine = math.cos(pose[2])
        sine = math.sin(pose[2])
        return np.asarray(
            [cosine * delta[0] + sine * delta[1], -sine * delta[0] + cosine * delta[1]]
        )

    def _verify_door_evidence(self, evidence, points_base, pose):
        accepted = []
        for item in evidence:
            center_base = self._metric_point_to_base(item.center, pose)
            normal_yaw = normalize_angle(item.normal_yaw - pose[2])
            status = self.door_verifier.verify(
                points_base, center_base, normal_yaw, item.width
            )
            verified_item = replace(item, verification_status=status)
            if status == REJECTED:
                rospy.loginfo(
                    "Door verifier rejected candidate source=%s center=(%.2f, %.2f)",
                    item.source,
                    item.center[0],
                    item.center[1],
                )
                continue
            accepted.append(verified_item)
        return accepted

    @staticmethod
    def _sector_clearance(ranges):
        return float(np.percentile(ranges, 60.0)) if ranges else 0.0

    @staticmethod
    def _narrow_clearance(ranges):
        return finite_median_clearance(ranges)

    def _metric_corridor_progress_locked(self):
        if (
            self.pose is None
            or self.world_pose is None
            or self.metric_corridor_anchor is None
        ):
            return None
        return projected_travel(
            self.metric_corridor_anchor,
            self.world_pose[:2],
            self.corridor_yaw,
        )

    def _topology_gate_depth_locked(self, point):
        """Signed main-corridor depth measured from the lobby exit gate."""
        if self.virtual_isolation_door is None:
            return None
        return projected_travel(
            self.virtual_isolation_door[:2],
            point,
            self.virtual_isolation_door[2],
        )

    def _exit_opposite_probe_active_locked(self):
        """Return whether the bounded exit-time opposite-door probe is active.

        The caller holds ``self.lock``.  This probe is deliberately narrower
        than normal corridor discovery: the robot must already be on the
        corridor side of the active doorway, and the short timer expires
        before regular corridor progression resumes.
        """
        if self.goal_state != "EXIT_TRANSLATING":
            return False
        if self.opposite_door_probe_until == rospy.Time(0):
            return False
        if rospy.Time.now() > self.opposite_door_probe_until:
            return False
        active = self.mission.candidates.get(self.mission.active_id)
        if active is None or self.world_pose is None:
            return False
        return door_corridor_side_reached(
            active.center,
            self.world_pose[:2],
            active.normal_yaw,
            self.corridor_side_entry_depth,
        )

    def _track_scan_openings_locked(
        self,
        left_clearance,
        right_clearance,
        left_wall_return,
        right_wall_return,
    ):
        normal_discovery_allowed = door_discovery_allowed(
            self.mission.active_id, self.goal_state
        )
        if self.world_pose is None or self.corridor_anchor is None or not normal_discovery_allowed:
            return
        direction = np.array([math.cos(self.corridor_yaw), math.sin(self.corridor_yaw)])
        normal = np.array([-direction[1], direction[0]])
        robot = np.asarray(self.world_pose[:2])
        progress = self._metric_corridor_progress_locked()
        if progress is None:
            return
        both_walls = self.corridor_estimate.valid
        if not self.corridor_context_confirmed:
            if both_walls:
                if self.scan_corridor_start is None:
                    self.scan_corridor_start = progress
                elif progress - self.scan_corridor_start >= self.scan_corridor_evidence_length:
                    self.corridor_context_confirmed = True
                    rospy.loginfo(
                        "Elongated corridor confirmed from side scan evidence: "
                        "left=%.2f right=%.2f width=%.2f",
                        left_wall_return,
                        right_wall_return,
                        left_wall_return + right_wall_return,
                    )
            else:
                self.scan_corridor_start = None
            return
        for side, clearance in ((1.0, left_clearance), (-1.0, right_clearance)):
            if not math.isfinite(clearance):
                continue
            if clearance >= self.scan_open_threshold:
                if self.scan_open_start[side] is None:
                    self.scan_open_start[side] = progress
                    self.scan_open_start_pose[side] = tuple(self.world_pose[:2])
                    self.scan_open_samples[side] = 1
                    self.scan_open_provisional_emitted[side] = False
                    self.corridor_opening_state = "OPEN_START"
                else:
                    self.scan_open_samples[side] += 1
                    self.corridor_opening_state = "OPEN"
                if (
                    self.scan_provisional_openings
                    and not self.scan_open_provisional_emitted[side]
                    and self.scan_open_samples[side] >= self.scan_provisional_samples
                ):
                    # In Stage B the virtual entrance gate already excludes
                    # the lobby, and the mission owns exactly four side-room
                    # doors.  Predict the centre from the first jamb instead
                    # of driving a full door width before the far jamb can be
                    # observed.  A closed observation later refines the same
                    # hypothesis and waypoint.
                    side_normal = normal * side
                    start_pose = np.asarray(self.scan_open_start_pose[side])
                    centerline = (
                        start_pose
                        + direction * (0.5 * self.scan_expected_door_width)
                    )
                    center = centerline + side_normal * self.scan_wall_distance[side]
                    pre = center - side_normal * self.detector.pre_distance
                    post = center + side_normal * self.detector.post_distance
                    normal_yaw = math.atan2(side_normal[1], side_normal[0])
                    center_tuple = (float(center[0]), float(center[1]))
                    if not any(
                        point_in_polygon(center_tuple, polygon)
                        for polygon in self.defer_polygons
                    ):
                        self.pending_scan_evidence.append(
                            DoorEvidence(
                                source=SCAN,
                                timestamp=rospy.Time.now().to_sec(),
                                side=side,
                                center=center_tuple,
                                width=self.scan_expected_door_width,
                                normal_yaw=normal_yaw,
                                pre_pose=(float(pre[0]), float(pre[1]), normal_yaw),
                                post_pose=(float(post[0]), float(post[1]), normal_yaw),
                                corridor_confidence=max(
                                    self.corridor_estimate.confidence,
                                    0.8 if self.corridor_context_confirmed else 0.0,
                                ),
                                source_confidence=0.85,
                                opening_complete=True,
                                localization_health=self.localization_health,
                            )
                        )
                        self.scan_open_provisional_emitted[side] = True
                        self.scan_opening_hold = True
                        self.door_verifier_request_until = (
                            rospy.Time.now() + rospy.Duration(1.5)
                        )
                        rospy.loginfo(
                            "Provisional corridor door locked: side=%+.0f center=(%.2f, %.2f)",
                            side,
                            center_tuple[0],
                            center_tuple[1],
                        )
                continue

            opening_start = self.scan_open_start[side]
            if opening_start is not None:
                measurement = closed_opening_measurement(
                    opening_start,
                    progress,
                    self.scan_open_min_width,
                    self.scan_open_max_width,
                    leading_wall_start=self.scan_leading_wall_start[side],
                    min_leading_wall_length=self.scan_leading_wall_min_length,
                )
                if measurement is not None:
                    _center_progress, observed_width = measurement
                    side_normal = normal * side
                    start_pose = np.asarray(self.scan_open_start_pose[side])
                    centerline = 0.5 * (start_pose + robot)
                    center = centerline + side_normal * self.scan_wall_distance[side]
                    pre = center - side_normal * self.detector.pre_distance
                    post = center + side_normal * self.detector.post_distance
                    normal_yaw = math.atan2(side_normal[1], side_normal[0])
                    corridor_confidence = max(
                        self.corridor_estimate.confidence,
                        0.8 if self.corridor_context_confirmed else 0.0,
                    )
                    center_tuple = (float(center[0]), float(center[1]))
                    deferred = any(
                        point_in_polygon(center_tuple, polygon)
                        for polygon in self.defer_polygons
                    )
                    if deferred:
                        # Scan-side evidence must obey the same architecture-
                        # owned defer zones as map detections.
                        rospy.logdebug(
                            "Ignoring deferred lidar opening: side=%+.0f center=(%.2f, %.2f)",
                            side,
                            center_tuple[0],
                            center_tuple[1],
                        )
                    else:
                        self.pending_scan_evidence.append(
                            DoorEvidence(
                                source=SCAN,
                                timestamp=rospy.Time.now().to_sec(),
                                side=side,
                                center=center_tuple,
                                width=observed_width,
                                normal_yaw=normal_yaw,
                                pre_pose=(float(pre[0]), float(pre[1]), normal_yaw),
                                post_pose=(float(post[0]), float(post[1]), normal_yaw),
                                corridor_confidence=corridor_confidence,
                                source_confidence=1.0,
                                opening_complete=True,
                                localization_health=self.localization_health,
                            )
                        )
                        self.door_verifier_request_until = (
                            rospy.Time.now() + rospy.Duration(1.5)
                        )
                        rospy.loginfo(
                            "Closed lidar opening observed online: side=%+.0f width=%.2f",
                            side,
                            observed_width,
                        )
                        self.scan_opening_hold = True
                        self.corridor_opening_state = "OPEN_END"
            if 0.4 <= clearance <= 2.2:
                self.scan_wall_distance[side] = 0.8 * self.scan_wall_distance[side] + 0.2 * clearance
                if self.scan_leading_wall_start[side] is None or opening_start is not None:
                    self.scan_leading_wall_start[side] = progress
            else:
                self.scan_leading_wall_start[side] = None
            self.scan_open_start[side] = None
            self.scan_open_start_pose[side] = None
            self.scan_open_samples[side] = 0
            self.scan_open_provisional_emitted[side] = False
        if not self.scan_opening_hold and not any(
            value is not None for value in self.scan_open_start.values()
        ):
            self.corridor_opening_state = "CLOSED"

    def _detect(self, _event):
        if not self.enabled or self.mission_fault is not None:
            return
        with self.lock:
            if self.initial_forward_active:
                return
            grid = self.grid
            source_pose = self.pose
            pose = self.world_pose
            corridor_yaw = self.corridor_yaw
            source_corridor_yaw = (
                self._metric_yaw_to_source_locked(corridor_yaw)
                if source_pose is not None and pose is not None
                else corridor_yaw
            )
            grid_frame = grid.frame_id if grid is not None else self.map_frame
            grid_pose = (
                self._source_pose_to_grid_locked(source_pose, grid_frame)
                if source_pose is not None
                else None
            )
            grid_corridor_yaw = self._source_yaw_to_grid_locked(
                source_corridor_yaw, grid_frame
            )
            corridor_context_confirmed = self.corridor_context_confirmed
            discovery_allowed = door_discovery_allowed(
                self.mission.active_id, self.goal_state
            )
            pending_scan_evidence = (
                list(self.pending_scan_evidence) if discovery_allowed else []
            )
            defer_polygons = list(self.defer_polygons)
            localization_health = self.localization_health
            corridor_estimate = self.corridor_estimate
            virtual_gate = self.virtual_isolation_door
            door_points = self.latest_door_points_base.copy()
            door_points_stamp = self.latest_door_points_stamp
            exit_opposite_allowed = self._exit_opposite_probe_active_locked()
            active_exit_candidate = (
                self.mission.candidates.get(self.mission.active_id)
                if exit_opposite_allowed
                else None
            )
        if pose is None or source_pose is None:
            return
        if not discovery_allowed and not exit_opposite_allowed:
            return
        map_detection_allowed = grid is not None and localization_health != STALE
        elongated = False
        if (
            map_detection_allowed
            and grid_pose is not None
            and self.require_elongated_corridor
        ):
            elongated = self.corridor_estimator.map_valid(
                grid,
                (grid_pose[0], grid_pose[1]),
                grid_corridor_yaw,
                length=self.corridor_evidence_length,
                min_wall_support=self.corridor_min_wall_support,
            )
            if elongated:
                with self.lock:
                    self.corridor_context_confirmed = True
            else:
                rospy.logdebug_throttle(2.0, "Door detection gated: no elongated corridor evidence")
                map_detection_allowed = False
        map_candidates = []
        if map_detection_allowed and grid_pose is not None:
            # Occupancy-grid detections are expressed in the grid frame.  Keep
            # the detector pose and corridor direction in that frame, then
            # convert each accepted candidate to the metric mission frame.
            map_candidates = self.detector.detect(
                grid,
                (grid_pose[0], grid_pose[1]),
                grid_corridor_yaw,
                (),
            )
        map_corridor_confidence = max(
            corridor_estimate.confidence,
            0.9 if elongated or corridor_context_confirmed else 0.0,
        )
        now = rospy.Time.now().to_sec()
        map_evidence = []
        with self.lock:
            for candidate in map_candidates:
                center = self._grid_point_to_metric_locked(candidate.center, grid_frame)
                if any(
                    point_in_polygon(center, polygon)
                    for polygon in defer_polygons
                ):
                    rospy.logdebug(
                        "Ignoring map opening inside metric defer zone at "
                        "(%.2f, %.2f)",
                        center[0],
                        center[1],
                    )
                    continue
                pre = self._grid_point_to_metric_locked(
                    candidate.pre_pose[:2], grid_frame
                )
                post = self._grid_point_to_metric_locked(
                    candidate.post_pose[:2], grid_frame
                )
                transformed_normal = self._transform_planar_pose_locked(
                    (0.0, 0.0, candidate.normal_yaw),
                    grid_frame,
                    self.world_frame,
                )
                normal_yaw = (
                    transformed_normal[2]
                    if transformed_normal is not None
                    else self._source_yaw_to_metric_locked(candidate.normal_yaw)
                )
                map_evidence.append(
                    DoorEvidence(
                        source=MAP,
                        timestamp=now,
                        side=(
                            1.0
                            if math.sin(normal_yaw - corridor_yaw) >= 0.0
                            else -1.0
                        ),
                        center=center,
                        width=candidate.width,
                        normal_yaw=normal_yaw,
                        pre_pose=(pre[0], pre[1], normal_yaw),
                        post_pose=(post[0], post[1], normal_yaw),
                        corridor_confidence=map_corridor_confidence,
                        source_confidence=0.9,
                        opening_complete=True,
                        localization_health=localization_health,
                    )
                )
            if map_evidence or pending_scan_evidence:
                self.door_verifier_request_until = (
                    rospy.Time.now() + rospy.Duration(1.5)
                )
        evidence = pending_scan_evidence + map_evidence
        if virtual_gate is not None:
            accepted_evidence = []
            rejected_evidence = []
            for item in evidence:
                if point_beyond_topology_gate(
                    virtual_gate[:2],
                    item.center,
                    virtual_gate[2],
                    self.topology_gate_candidate_margin,
                ):
                    accepted_evidence.append(item)
                else:
                    rejected_evidence.append(item)
            evidence = accepted_evidence
            if rejected_evidence:
                with self.lock:
                    self.topology_rejected_evidence += len(rejected_evidence)
                closest = max(
                    projected_travel(
                        virtual_gate[:2], item.center, virtual_gate[2]
                    )
                    for item in rejected_evidence
                )
                rospy.logwarn_throttle(
                    2.0,
                    "Topology gate rejected %d entrance/lobby opening(s); "
                    "closest corridor depth=%.2f m",
                    len(rejected_evidence),
                    closest,
                )
        if exit_opposite_allowed and active_exit_candidate is not None:
            # Exit probing is a read-only shortcut.  Do not consume or reuse
            # an opening that was queued by the normal corridor detector.
            # Keep using the already topology-filtered evidence; restoring
            # raw map evidence here would let a lobby opening bypass the gate.
            evidence = [
                item
                for item in evidence
                if item.source == MAP
                and opposite_door_at_station(
                    active_exit_candidate.center,
                    active_exit_candidate.normal_yaw,
                    item.center,
                    item.normal_yaw,
                    corridor_yaw,
                    self.opposite_pair_station_tolerance,
                )
            ]
        retry_scan_evidence = []
        if self.door_verifier_enabled:
            cloud_age = (
                (rospy.Time.now() - door_points_stamp).to_sec()
                if door_points_stamp != rospy.Time(0)
                else float("inf")
            )
            cloud_available = len(door_points) and 0.0 <= cloud_age <= self.door_verifier_max_age
            if cloud_available:
                verified_evidence = self._verify_door_evidence(
                    evidence, door_points, pose
                )
                if self.door_scan_requires_verification:
                    retry_scan_evidence = [
                        item
                        for item in verified_evidence
                        if item.source == SCAN and item.verification_status == INCONCLUSIVE
                    ]
                    evidence = [
                        item
                        for item in verified_evidence
                        if item.source != SCAN
                        or item.verification_status != INCONCLUSIVE
                    ]
                else:
                    evidence = verified_evidence
            else:
                if self.door_scan_requires_verification and pending_scan_evidence:
                    # Keep Scan-only evidence until a point cloud arrives.  The
                    # old behavior consumed it immediately, leaving a
                    # permanently PENDING hypothesis after a single timeout.
                    retry_scan_evidence = list(pending_scan_evidence)
                    evidence = list(map_evidence)
                else:
                    evidence = [
                        replace(item, verification_status=INCONCLUSIVE)
                        for item in evidence
                    ]
        self._publish_door_evidence(evidence)
        confirmed = self.door_fusion.ingest_many(evidence)
        # Merge every confirmed owner, not only newly confirmed hypotheses.
        # This lets later, better map/scan frames refine a DISCOVERED door's
        # centre and pre-door waypoint before it is selected.
        confirmed_owners = [
            hypothesis
            for hypothesis in self.door_fusion.hypotheses.values()
            if hypothesis.status == "CONFIRMED"
        ]
        new_count = self.mission.merge_hypotheses(confirmed_owners, now)
        if exit_opposite_allowed:
            for hypothesis in confirmed:
                if self.mission.mark_exit_opposite(hypothesis.hypothesis_id):
                    rospy.loginfo(
                        "Exit-time opposite doorway observed: %s",
                        hypothesis.hypothesis_id,
                    )
            if self.mission.exit_opposite_candidate_id is None:
                matched_id = self.mission.mark_exit_opposite_from_evidence(
                    evidence,
                    active_exit_candidate.center,
                    active_exit_candidate.normal_yaw,
                    corridor_yaw,
                    self.opposite_pair_station_tolerance,
                )
                if matched_id is not None:
                    rospy.loginfo(
                        "Exit-time opposite doorway re-observed: %s", matched_id
                    )
        with self.lock:
            del self.pending_scan_evidence[: len(pending_scan_evidence)]
            if retry_scan_evidence:
                self.pending_scan_evidence[0:0] = retry_scan_evidence
            if not self.pending_scan_evidence:
                self.scan_opening_hold = False
                if self.corridor_opening_state == "OPEN_START":
                    self.corridor_opening_state = "OPEN_END"
        if new_count and self.goal_state == "CORRIDOR_PROGRESSION":
            self._stop()
            self.goal_state = None
            self.corridor_progress_start = None
            self.corridor_advancing = False
            self.corridor_stall_windows = 0
            self.corridor_exhausted = False
        self._publish_door_hypotheses()
        self._publish_markers()

    def _control(self, _event):
        if not self.enabled or not self.control_enabled:
            return
        now = rospy.Time.now()
        fault_reason = self.fault_monitor.evaluate(
            now.to_sec(),
            active=self.mission.active_id is not None or bool(self.mission.candidates),
            localization_state=self.localization_health,
            controller_state=self.controller_health,
            lio_state=self.lio_health if self.lio_health_seen else "GOOD",
            effective_points=self.lio_effective_points if self.lio_health_seen else 100,
        )
        if self.mission_fault is not None:
            self._stop()
            self._publish_status(MISSION_FAULT)
            return
        if fault_reason is not None:
            self.mission_fault = fault_reason
            self._stop()
            self.mission_fault_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "state": MISSION_FAULT,
                            "reason": fault_reason,
                            "timestamp": now.to_sec(),
                        },
                        sort_keys=True,
                    )
                )
            )
            rospy.logerr("Stage B mission fault: %s; aborting run", fault_reason)
            self._publish_status(MISSION_FAULT)
            return
        with self.lock:
            pose = self.world_pose
        health = self.localization_monitor.evaluate(now.to_sec())
        with self.lock:
            self.localization_health = health
        if pose is None or health == STALE:
            self._stop()
            self._publish_status("LOCALIZATION_STALE")
            return
        with self.lock:
            initial_forward_active = self.initial_forward_active
        if initial_forward_active:
            self._control_initial_forward(pose)
            self._publish_status("INITIAL_FORWARD")
            return
        if self.mission.active_id is None:
            visited_count = sum(
                item.status == "VISITED" for item in self.mission.candidates.values()
            )
            if visited_count >= self.expected_room_count:
                self._stop()
                if self.mission.complete(
                    now.to_sec(),
                    expected_room_count=self.expected_room_count,
                    danger_confirmation_active=self.danger_confirmation_active,
                ):
                    self.complete_pub.publish(Bool(data=True))
                    self._publish_status("FLOOR_COMPLETE")
                else:
                    self._publish_status("COMPLETION_QUIET")
                return
            candidate = self.mission.next_candidate(
                (pose[0], pose[1]),
                self.expected_room_count,
                corridor_yaw=self.corridor_yaw,
                prefer_opposite=self.prefer_opposite_door,
                opposite_pair_station_tolerance=self.opposite_pair_station_tolerance,
                prefer_paired_station=self.prefer_paired_station,
            )
            if candidate is not None:
                if self.mission.last_selection_reason == "EXIT_OPPOSITE":
                    rospy.loginfo(
                        "Entering exit-time opposite doorway %s",
                        candidate.candidate_id,
                    )
                self._enter_state(candidate.status)
            elif not self.corridor_exhausted and self._control_corridor_progression(pose):
                self._publish_corridor_diagnostic(pose)
                self._publish_status("CORRIDOR_PROGRESSION")
                return
            else:
                self._publish_corridor_diagnostic(pose)
                self._publish_status("WAITING_FOR_OPENINGS")
                return
        candidate = self.mission.candidates[self.mission.active_id]
        if candidate.status == GO_TO_PRE_DOOR:
            self._control_pre_door(candidate, pose)
        elif candidate.status == ALIGN_TO_DOOR_NORMAL:
            self._control_alignment(candidate, pose)
        elif candidate.status == DOOR_CROSSING:
            self._control_crossing(candidate, pose)
        elif candidate.status == ROOM_SCAN:
            self._control_scan(candidate, pose)
        elif candidate.status == EXIT_ROOM:
            self._control_exit_room(candidate, pose)
        self._publish_status(candidate.status)

    def _control_initial_forward(self, pose):
        """Traverse the isolated lobby edge between two topology gates."""
        with self.lock:
            if self.initial_forward_anchor is None:
                self.initial_forward_anchor = (pose[0], pose[1])
                self.initial_forward_yaw = self.corridor_yaw
                self.lobby_entry_door = (pose[0], pose[1], self.corridor_yaw)
            anchor = self.initial_forward_anchor
            base_yaw = self.initial_forward_yaw
            front_clearance = self.front_clearance
            left_clearance = self.left_clearance
            right_clearance = self.right_clearance
            left_nearest_clearance = self.left_nearest_clearance
            right_nearest_clearance = self.right_nearest_clearance
            corridor_estimate = self.corridor_estimate
        direction_x = math.cos(base_yaw)
        direction_y = math.sin(base_yaw)
        progress = (
            (pose[0] - anchor[0]) * direction_x
            + (pose[1] - anchor[1]) * direction_y
        )
        target_yaw = base_yaw
        # Only centre after the dog has reached the narrow corridor mouth.
        # Applying side-wall steering in the wide lobby would make furniture
        # look like a corridor wall and bend the supposedly fixed transit.
        if (
            progress >= self.initial_centering_start_distance
            and corridor_estimate.valid
        ):
            center_correction, _close_wall = corridor_steering_correction(
                corridor_estimate.center_error,
                left_clearance,
                right_clearance,
                left_nearest_clearance,
                right_nearest_clearance,
                self.corridor_scan_center_gain,
                self.corridor_wall_avoid_distance,
            )
            target_yaw = normalize_angle(base_yaw + center_correction)
        if progress >= self.initial_forward_distance:
            self._stop()
            # Use the actual crossing pose, not only the ideal centreline
            # projection.  This makes the RViz gate and all later signed
            # depth checks agree with where the dog really entered the
            # corridor after centring corrections.
            gate_center = (pose[0], pose[1])
            with self.lock:
                self.initial_forward_active = False
                self.corridor_context_confirmed = True
                self.topology_region = "MAIN_CORRIDOR"
                self.corridor_anchor = (pose[0], pose[1])
                self.metric_corridor_anchor = (pose[0], pose[1])
                self.virtual_isolation_door = (
                    gate_center[0], gate_center[1], base_yaw
                )
                self.pending_scan_evidence = []
                self.scan_opening_hold = False
                self.scan_open_start = {-1.0: None, 1.0: None}
                self.scan_open_start_pose = {-1.0: None, 1.0: None}
                self.scan_open_samples = {-1.0: 0, 1.0: 0}
                self.scan_open_provisional_emitted = {-1.0: False, 1.0: False}
                self.scan_leading_wall_start = {-1.0: None, 1.0: None}
                self.corridor_opening_state = "CLOSED"
                # The fixed edge is the entire lobby topology.  Start the
                # corridor with a genuinely fresh candidate pool so entrance
                # and lobby openings can never leak across the virtual door.
                self.door_fusion.hypotheses.clear()
                self.door_fusion.next_id = 0
                self.mission.candidates.clear()
                self.mission.active_id = None
                self.mission.exit_opposite_candidate_id = None
                self.mission.last_discovery_time = 0.0
            rospy.loginfo(
                "Lobby topology complete: progress=%.2f m corridor_gate=(%.2f, %.2f)",
                progress,
                gate_center[0],
                gate_center[1],
            )
            self._publish_markers()
            return
        if front_clearance < self.corridor_stop_distance:
            self._stop()
            rospy.logwarn(
                "Initial corridor isolation blocked: front clearance %.2f m", front_clearance
            )
            return
        yaw_error = normalize_angle(target_yaw - pose[2])
        command = Twist()
        if abs(yaw_error) > 0.10:
            command.angular.z = math.copysign(self.corridor_turn_speed, yaw_error)
        else:
            command.linear.x = self.initial_forward_speed
            command.angular.z = max(-0.08, min(0.08, 0.8 * yaw_error))
        self._publish_command(command)

    def _control_corridor_progression(self, pose):
        with self.lock:
            anchor = self.metric_corridor_anchor
            front_clearance = self.front_clearance
            left_clearance = self.left_clearance
            right_clearance = self.right_clearance
            narrow_left_clearance = self.narrow_left_clearance
            narrow_right_clearance = self.narrow_right_clearance
            left_nearest_clearance = self.left_nearest_clearance
            right_nearest_clearance = self.right_nearest_clearance
            scan_opening_hold = self.scan_opening_hold
            corridor_estimate = self.corridor_estimate
            last_scan_update = self.last_scan_update
            virtual_gate = self.virtual_isolation_door
        if scan_opening_hold:
            self._stop()
            return True
        if anchor is None:
            return False
        if corridor_estimate.valid:
            self._publish_metric_corridor_constraint(
                pose, corridor_estimate.center_error
            )
        if self.pending_corridor_turn is not None:
            backed_distance = math.hypot(
                pose[0] - self.corridor_backoff_origin[0],
                pose[1] - self.corridor_backoff_origin[1],
            )
            if (
                backed_distance >= self.corridor_backoff_distance
                or self._state_elapsed() >= self.corridor_backoff_timeout
            ):
                self._stop()
                with self.lock:
                    self.corridor_yaw = self.pending_corridor_turn
                    self.metric_corridor_anchor = (pose[0], pose[1])
                    if not self.pending_corridor_preserve_context:
                        self.corridor_anchor = (pose[0], pose[1])
                        self.corridor_context_confirmed = False
                    self.scan_corridor_start = None
                    self.scan_leading_wall_start = {-1.0: None, 1.0: None}
                    self.scan_open_start = {-1.0: None, 1.0: None}
                    self.scan_open_start_pose = {-1.0: None, 1.0: None}
                rospy.loginfo(
                    "Corridor backoff complete: distance=%.2f target_yaw=%.2f",
                    backed_distance,
                    self.pending_corridor_turn,
                )
                self.pending_corridor_turn = None
                self.pending_corridor_preserve_context = False
                self.corridor_backoff_origin = None
                self.corridor_progress_start = 0.0
                self.corridor_advancing = False
                self.state_start = rospy.Time.now()
                return True
            command = Twist()
            command.linear.x = -self.corridor_backoff_speed
            self._publish_command(command)
            return True
        direction_x = math.cos(self.corridor_yaw)
        direction_y = math.sin(self.corridor_yaw)
        normal_x = -direction_y
        normal_y = direction_x
        current_progress = (
            (pose[0] - anchor[0]) * direction_x + (pose[1] - anchor[1]) * direction_y
        )
        if virtual_gate is not None:
            gate_depth = projected_travel(
                virtual_gate[:2], pose[:2], virtual_gate[2]
            )
            returning_to_lobby = (
                math.cos(self.corridor_yaw - virtual_gate[2]) < 0.0
            )
            if returning_to_lobby and gate_depth <= self.topology_gate_return_margin:
                self._stop()
                self.goal_state = None
                self.corridor_progress_start = None
                self.corridor_advancing = False
                self.corridor_exhausted = True
                rospy.logwarn(
                    "Reverse sweep stopped at corridor topology gate: depth=%.2f m; "
                    "lobby re-entry is forbidden",
                    gate_depth,
                )
                return False
        anchor_lateral_error = (
            (pose[0] - self.corridor_anchor[0]) * normal_x
            + (pose[1] - self.corridor_anchor[1]) * normal_y
        )
        scan_is_fresh = (
            last_scan_update != rospy.Time(0)
            and (rospy.Time.now() - last_scan_update).to_sec()
            <= self.corridor_scan_timeout
        )
        lateral_error = (
            corridor_estimate.center_error
            if scan_is_fresh and corridor_estimate.confidence > 0.0
            else anchor_lateral_error
        )
        if current_progress >= self.max_corridor_progress:
            self._stop()
            self.goal_state = None
            self.corridor_progress_start = None
            self.corridor_advancing = False
            if self._start_corridor_reversal(pose, "maximum corridor progress"):
                return True
            self.corridor_exhausted = True
            return False

        if self.goal_state != "CORRIDOR_PROGRESSION":
            self.goal_state = "CORRIDOR_PROGRESSION"
            self.corridor_progress_start = current_progress
            self.corridor_advancing = False
            self.state_start = rospy.Time.now()

        center_correction, close_wall = corridor_steering_correction(
            lateral_error,
            left_clearance,
            right_clearance,
            left_nearest_clearance,
            right_nearest_clearance,
            self.corridor_scan_center_gain,
            self.corridor_wall_avoid_distance,
        )
        target_yaw = normalize_angle(self.corridor_yaw + center_correction)
        yaw_error = normalize_angle(target_yaw - pose[2])
        if abs(yaw_error) <= 0.25 and front_clearance < self.corridor_stop_distance:
            self._stop()
            rospy.logwarn(
                "Main corridor endpoint: front=%.2f left=%.2f right=%.2f; "
                "side branches are outside Stage B",
                front_clearance,
                left_clearance,
                right_clearance,
            )
            self.goal_state = None
            self.corridor_progress_start = None
            self.corridor_advancing = False
            if self._start_corridor_reversal(pose, "corridor endpoint"):
                return True
            self.corridor_exhausted = True
            return False
        if current_progress - self.corridor_progress_start >= self.corridor_progress_step:
            self._stop()
            self.goal_state = None
            self.corridor_progress_start = None
            self.corridor_advancing = False
            self.corridor_stall_windows = 0
            return True
        if self.corridor_advancing and self._state_elapsed() > self.corridor_progress_timeout:
            rospy.logwarn("Corridor progression timed out without covering the requested distance")
            self._stop()
            self.goal_state = None
            self.corridor_advancing = False
            if not corridor_timeout_requires_reversal(
                front_clearance,
                self.corridor_stop_distance,
                self.corridor_stall_windows,
                self.max_corridor_stall_windows,
            ):
                self.corridor_stall_windows += 1
                self.corridor_progress_start = current_progress
                self.state_start = rospy.Time.now()
                rospy.loginfo(
                    "Continuing clear corridor after odometry stall window %d/%d; front=%.2f",
                    self.corridor_stall_windows,
                    self.max_corridor_stall_windows,
                    front_clearance,
                )
                return True
            self.corridor_progress_start = None
            if self._start_corridor_reversal(pose, "progress timeout"):
                return True
            self.corridor_exhausted = True
            return False

        command = Twist()
        turn_threshold = 0.06 if close_wall else 0.10
        if abs(yaw_error) > turn_threshold:
            if (
                not self.corridor_advancing
                and self._state_elapsed() > self.corridor_heading_timeout
            ):
                rospy.logwarn(
                    "Corridor heading alignment timed out with yaw error %.2f rad",
                    yaw_error,
                )
                self._stop()
                self.goal_state = None
                self.corridor_progress_start = None
                self.corridor_exhausted = True
                return False
            command.angular.z = math.copysign(self.corridor_turn_speed, yaw_error)
            if front_clearance >= self.corridor_turn_clearance:
                command.linear.x = self.corridor_wall_avoid_speed if close_wall else 0.45
        else:
            if not self.corridor_advancing:
                self.corridor_advancing = True
                self.state_start = rospy.Time.now()
            command.linear.x = self.corridor_progress_speed
            command.angular.z = max(-0.15, min(0.15, 0.8 * yaw_error))
        self._publish_command(command)
        return True

    def _publish_metric_corridor_constraint(self, pose, lateral_error):
        target_x, target_y = constrain_to_corridor(
            pose[:2],
            self.corridor_anchor,
            self.corridor_yaw,
            lateral_error,
        )
        payload = {
            "position": [target_x, target_y],
            "lateral_error": lateral_error,
            "timestamp": rospy.Time.now().to_sec(),
        }
        self.metric_corridor_constraint_pub.publish(
            String(data=json.dumps(payload, sort_keys=True))
        )

    def _publish_corridor_diagnostic(self, pose):
        with self.lock:
            anchor = self.metric_corridor_anchor
            estimate = self.corridor_estimate
            finite_scan_count = self.finite_scan_count
            front = self.front_clearance
            left = self.left_clearance
            right = self.right_clearance
            opening_state = self.corridor_opening_state
            last_delta = self.last_pose_delta
        progress = None
        if anchor is not None:
            progress = projected_travel(anchor, pose[:2], self.corridor_yaw)
        payload = {
            "sim_time": rospy.Time.now().to_sec(),
            "state": "CORRIDOR_PROGRESSION",
            "cmd_vel": {
                "linear_x": self.last_command[0],
                "angular_z": self.last_command[1],
            },
            "lio_pose": [float(value) for value in pose],
            "lio_delta": {
                "translation": float(last_delta[0]),
                "rotation": float(last_delta[1]),
            },
            "corridor_progress_start": self.corridor_progress_start,
            "corridor_progress_current": progress,
            "corridor_progress_requested": self.corridor_progress_step,
            "front_clearance": self._finite_or_none(front),
            "left_clearance": self._finite_or_none(left),
            "right_clearance": self._finite_or_none(right),
            "valid": bool(estimate.valid),
            "confidence": float(estimate.confidence),
            "finite_scan_count": int(finite_scan_count),
            "opening_state": opening_state,
            "door_evidence_count": len(self.pending_scan_evidence),
            "door_hypothesis_count": len(self.door_fusion.hypotheses),
        }
        self.corridor_diagnostic_pub.publish(
            String(data=json.dumps(payload, sort_keys=True, allow_nan=False))
        )

    def _start_corridor_reversal(self, pose, reason):
        statuses = [item.status for item in self.mission.candidates.values()]
        visited_count = sum(1 for status in statuses if status == "VISITED")
        if visited_count >= self.expected_room_count:
            return False
        if not self.mission.recovery.request_reverse_sweep():
            return False
        self.pending_corridor_turn = normalize_angle(self.corridor_yaw + math.pi)
        self.pending_corridor_preserve_context = True
        self.corridor_backoff_origin = (pose[0], pose[1])
        self.corridor_advancing = False
        self.corridor_stall_windows = 0
        self.corridor_sweep_count = self.mission.recovery.sweep_count
        self.state_start = rospy.Time.now()
        rospy.loginfo(
            "Starting corridor reverse sweep %d/%d after %s; visited=%d expected=%d",
            self.corridor_sweep_count,
            self.max_corridor_sweeps,
            reason,
            visited_count,
            self.expected_room_count,
        )
        return True

    def _control_pre_door(self, candidate, pose):
        # A quadruped with forward-only gait cannot safely cut diagonally
        # across a doorway corner.  First reach the door's corridor station,
        # then turn ninety degrees and approach the pre-door point along the
        # verified door normal.
        pre_x, pre_y = candidate.pre_pose[:2]
        delta_x = float(pre_x) - float(pose[0])
        delta_y = float(pre_y) - float(pose[1])
        station_error = (
            delta_x * math.cos(self.corridor_yaw)
            + delta_y * math.sin(self.corridor_yaw)
        )
        normal_error = (
            delta_x * math.cos(candidate.normal_yaw)
            + delta_y * math.sin(candidate.normal_yaw)
        )
        self.pre_door_station_error = station_error
        self.pre_door_normal_error = normal_error
        if self._state_elapsed() > self.pre_door_timeout:
            rospy.logwarn(
                "Pre-door %s timed out station_error=%.2f normal_error=%.2f m",
                self.pre_door_phase,
                station_error,
                normal_error,
            )
            self._stop()
            self._advance(False)
            return
        distance_to_pre = math.hypot(delta_x, delta_y)
        if self.pre_door_phase in ("IDLE", "STATION"):
            self.pre_door_phase = "STATION"
            if abs(station_error) <= self.door_centering_tolerance:
                self._stop()
                self.pre_door_phase = "FACE_PRE_DOOR"
                return
            target_yaw, approach_velocity = corridor_approach_motion(
                station_error, self.corridor_yaw, self.pre_door_speed,
                tolerance=self.door_centering_tolerance,
            )
        else:
            if distance_to_pre <= self.door_centering_tolerance:
                self.pre_door_phase = "COMPLETE"
                self._stop()
                self._advance(True)
                return
            # Point at the actual pre-door waypoint, not merely the detected
            # door normal.  The dog must finish this turn with zero linear
            # command before it is allowed to approach the doorway.
            target_yaw = math.atan2(delta_y, delta_x)
            yaw_error = normalize_angle(target_yaw - pose[2])
            if self.pre_door_phase == "FACE_PRE_DOOR":
                if abs(yaw_error) > self.door_alignment_tolerance:
                    self.pre_door_face_stable_since = None
                    command = Twist()
                    command.angular.z = math.copysign(self.align_speed, yaw_error)
                    self._publish_command(command)
                    return
                now = rospy.Time.now()
                if self.pre_door_face_stable_since is None:
                    self.pre_door_face_stable_since = now
                    self._stop()
                    return
                if (now - self.pre_door_face_stable_since).to_sec() < self.pre_door_face_stable_duration:
                    self._stop()
                    return
                self.pre_door_phase = "APPROACH_PRE_DOOR"
                self.pre_door_face_stable_since = None
                self._stop()
                return
            self.pre_door_phase = "APPROACH_PRE_DOOR"
            approach_velocity = self.pre_door_speed
        if approach_velocity == 0.0:
            self._stop()
            return
        with self.lock:
            front_clearance = self.front_clearance
        yaw_error = normalize_angle(target_yaw - pose[2])
        command = Twist()
        if abs(yaw_error) > 0.10:
            command.angular.z = math.copysign(self.align_speed, yaw_error)
            self._publish_command(command)
            return
        if front_clearance < self.corridor_stop_distance:
            rospy.logwarn(
                "Pre-door forward approach blocked: clearance %.2f m",
                front_clearance,
            )
            self._stop()
            self._advance(False)
            return
        command.linear.x = approach_velocity
        command.angular.z = max(-0.15, min(0.15, 0.8 * yaw_error))
        self._publish_command(command)

    def _control_alignment(self, candidate, pose):
        error = normalize_angle(candidate.normal_yaw - pose[2])
        if abs(error) <= self.door_alignment_tolerance:
            self._stop()
            self._advance(True)
            return
        if self._state_elapsed() > self.align_timeout:
            self._stop()
            self._advance(False)
            return
        command = Twist()
        command.angular.z = math.copysign(self.align_speed, error)
        self._publish_command(command)

    def _control_crossing(self, candidate, pose):
        yaw_error = normalize_angle(candidate.normal_yaw - pose[2])
        elapsed = self._state_elapsed()
        crossing_origin = self.door_crossing_origin_metric or pose[:2]
        depth, lateral_error = door_crossing_errors(
            candidate.center, pose[:2], candidate.normal_yaw
        )
        crossing_travel = projected_travel(
            crossing_origin, pose[:2], candidate.normal_yaw
        )
        self.door_crossing_last_depth = depth
        self.door_crossing_last_lateral = lateral_error
        self.door_crossing_last_travel = crossing_travel
        metric_inside = door_crossing_confirmed(
            candidate.center,
            pose[:2],
            candidate.normal_yaw,
            crossing_origin,
            self.inside_room_entry_depth,
            min(
                self.door_crossing_lateral_tolerance,
                max(0.05, 0.5 * float(candidate.width) - 0.10),
            ),
            self.door_crossing_minimum_travel,
        )
        if elapsed >= self.minimum_cross_time and metric_inside:
            rospy.loginfo(
                "Door interior confirmed depth=%.2f lateral=%.2f travel=%.2f m",
                depth,
                lateral_error,
                crossing_travel,
            )
            self._stop()
            self._advance(True)
            return
        # Corridor-structure disappearance is only supporting evidence.  It
        # must never transition to ROOM_SCAN without the metric crossing gate
        # above; that fallback caused the observed turn-at-door false entry.
        corridor_valid = self._corridor_structure_valid(pose)
        if corridor_valid:
            self.crossing_corridor_seen = True
        if elapsed > self.crossing_timeout:
            rospy.logwarn("Door crossing timed out without an inside-room structure event")
            self._stop()
            self._advance(False)
            return
        with self.lock:
            front_clearance = self.front_clearance
        if front_clearance < self.room_motion_stop_distance:
            rospy.logwarn(
                "Door crossing blocked by front clearance %.2f m", front_clearance
            )
            self._stop()
            self._advance(False)
            return
        command = Twist()
        command.linear.x = self.crossing_speed
        centering_correction = -self.door_centering_yaw_gain * lateral_error
        command.angular.z = max(
            -0.25,
            min(0.25, 0.8 * yaw_error + centering_correction),
        )
        self._publish_command(command)

    def _control_exit_room(self, candidate, pose):
        target_yaw = normalize_angle(candidate.normal_yaw + math.pi)
        yaw_error = normalize_angle(target_yaw - pose[2])
        command = Twist()
        if self.goal_state is None:
            self.goal_state = "EXIT_ALIGNING"
            self.state_start = rospy.Time.now()
        if self.goal_state == "EXIT_ALIGNING" and abs(yaw_error) <= 0.10:
            self.goal_state = "EXIT_TRANSLATING"
            self.state_start = rospy.Time.now()
            self.opposite_door_probe_until = (
                rospy.Time.now() + rospy.Duration(self.opposite_door_probe_duration)
            )
        if self.goal_state == "EXIT_ALIGNING":
            if self._state_elapsed() > self.align_timeout:
                rospy.logwarn(
                    "Room exit alignment timed out with yaw error %.2f rad", yaw_error
                )
                self._stop()
                self._advance(False)
                return
            command.angular.z = math.copysign(self.align_speed, yaw_error)
        else:
            elapsed = self._state_elapsed()
            if elapsed > self.max_exit_time:
                rospy.logwarn("Room exit timed out without stable corridor reacquisition")
                self._stop()
                self._advance(False)
                return
            with self.lock:
                front_clearance = self.front_clearance
            if front_clearance < self.room_motion_stop_distance:
                rospy.logwarn(
                    "Room exit blocked by front clearance %.2f m", front_clearance
                )
                self._stop()
                self._advance(False)
                return
            corridor_side_reached = door_corridor_side_reached(
                candidate.center,
                pose[:2],
                candidate.normal_yaw,
                self.corridor_side_entry_depth,
            )
            if elapsed >= self.minimum_exit_time and corridor_side_reached:
                if self.corridor_reacquire_start is None:
                    self.corridor_reacquire_start = rospy.Time.now()
                elif (
                    rospy.Time.now() - self.corridor_reacquire_start
                ).to_sec() >= self.corridor_reacquire_duration:
                    rospy.loginfo(
                        "Corridor side confirmed by bounded metric depth %.2f m",
                        -projected_travel(
                            candidate.center, pose[:2], candidate.normal_yaw
                        ),
                    )
                    self._stop()
                    self._request_local_loop_closure(candidate, pose)
                    self._advance(True)
                    return
            else:
                self.corridor_reacquire_start = None
            if abs(yaw_error) > 0.25:
                command.angular.z = math.copysign(self.align_speed, yaw_error)
            else:
                command.linear.x = self.crossing_speed
                command.angular.z = max(-0.18, min(0.18, 0.8 * yaw_error))
        self._publish_command(command)

    def _scan_points_in_map(self):
        with self.lock:
            points = self.latest_scan_points_base.copy()
            pose = self.pose
        if len(points) == 0 or pose is None:
            return points
        cosine = math.cos(pose[2])
        sine = math.sin(pose[2])
        rotation = np.asarray([[cosine, -sine], [sine, cosine]])
        return np.matmul(points, rotation.T) + np.asarray(pose[:2])

    def _accumulate_scan_points(self):
        """Add the latest scan to the active room submap with bounded memory."""
        points = self._scan_points_in_map()
        if len(points) == 0:
            return
        with self.lock:
            existing = self.scan_accumulated_points
        if len(existing):
            points = np.vstack((existing, points))
        if len(points) > self.scan_accumulation_max_points:
            # The loop matcher performs the same reduction, but bounding the
            # live accumulator keeps callback work predictable during a scan.
            points = voxel_downsample_2d(points, self.loop_voxel_size)
        if len(points) > self.scan_accumulation_max_points:
            stride = int(math.ceil(len(points) / float(self.scan_accumulation_max_points)))
            points = points[::stride]
        with self.lock:
            self.scan_accumulated_points = np.asarray(points, dtype=np.float64).reshape((-1, 2))

    def _request_local_loop_closure(self, candidate, pose):
        del pose
        # The room-entry submap is a local-loop reference, not a navigation
        # target.  The doorway frame remains as a fallback for older runs.
        reference = self.room_entry_loop_anchor
        if reference is None:
            reference = self.doorway_loop_anchors.get(candidate.candidate_id)
        self._request_scan_loop_closure(
            candidate.candidate_id,
            reference,
            "room_entry_same_door_local_submap",
            preserve_danger_ids=self.room_entry_danger_ids,
        )

    def _request_scan_loop_closure(
        self,
        candidate_id,
        reference,
        constraint,
        preserve_danger_ids=None,
        current_points=None,
    ):
        with self.lock:
            pose = self.pose
        if current_points is None:
            current = self._scan_points_in_map()
        else:
            current = np.asarray(current_points, dtype=np.float64).reshape((-1, 2))
        started = time.perf_counter()
        result = None
        if reference is not None:
            result = local_loop_icp(
                reference,
                current,
                voxel_size=self.loop_voxel_size,
                max_correspondence=self.loop_max_correspondence,
            )
        processing_ms = (time.perf_counter() - started) * 1000.0
        quality_accepted = bool(
            result is not None
            and result["overlap"] >= self.loop_min_overlap
            and result["rmse"] <= self.loop_max_rmse
            and result["translation"] <= self.loop_max_translation
            and result["rotation"] <= self.loop_max_rotation
        )
        transform = result["transform"] if result is not None else np.identity(3)
        payload = {
            "candidate_id": candidate_id,
            "quality_accepted": quality_accepted,
            "correction": [
                float(transform[0, 2]),
                float(transform[1, 2]),
                float(math.atan2(transform[1, 0], transform[0, 0])),
            ],
            "observed": [float(pose[0]), float(pose[1]), float(pose[2])],
            "constraint": constraint,
            "quality": {
                "rmse": result["rmse"] if result is not None else None,
                "overlap": result["overlap"] if result is not None else 0.0,
                "inliers": result["inliers"] if result is not None else 0,
                "reference_points": (
                    result["reference_points"] if result is not None else 0
                ),
                "current_points": (
                    result["current_points"] if result is not None else len(current)
                ),
            },
            "matcher_processing_ms": processing_ms,
            "preserve_danger_ids": sorted(preserve_danger_ids or []),
            "timestamp": rospy.Time.now().to_sec(),
        }
        self.loop_closure_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _corridor_structure_valid(self, pose):
        del pose
        with self.lock:
            estimate = self.corridor_estimate
            grid = self.grid
            source_pose = self.pose
            source_corridor_yaw = (
                self._metric_yaw_to_source_locked(self.corridor_yaw)
                if self.pose is not None and self.world_pose is not None
                else self.corridor_yaw
            )
            grid_frame = grid.frame_id if grid is not None else self.map_frame
            grid_pose = (
                self._source_pose_to_grid_locked(source_pose, grid_frame)
                if source_pose is not None
                else None
            )
            corridor_yaw = self._source_yaw_to_grid_locked(
                source_corridor_yaw, grid_frame
            )
            last_scan_update = self.last_scan_update
        scan_is_fresh = (
            last_scan_update != rospy.Time(0)
            and (rospy.Time.now() - last_scan_update).to_sec()
            <= self.corridor_scan_timeout
        )
        if scan_is_fresh and estimate.valid:
            return True
        return bool(
            grid is not None
            and grid_pose is not None
            and self.corridor_estimator.map_valid(
                grid,
                (grid_pose[0], grid_pose[1]),
                corridor_yaw,
                length=self.corridor_evidence_length,
                min_wall_support=self.corridor_min_wall_support,
            )
        )

    def _control_scan(self, candidate, pose):
        if self.entry_validation_mode:
            if self._state_elapsed() >= self.entry_validation_hold:
                rospy.loginfo(
                    "Entry validation: leaving %s after %.2f s hold",
                    candidate.candidate_id,
                    self._state_elapsed(),
                )
                self._complete_room_exploration(candidate)
            else:
                self._stop()
            return
        self._control_room_frontier(candidate, pose)

    def _control_room_frontier(self, candidate, pose):
        """Explore the active room using safe, reachable frontier targets."""
        self._accumulate_scan_points()
        with self.lock:
            grid = self.grid
            source_pose = self.pose
            door_center = self.room_entry_door_source
            inward_yaw = self.room_entry_inward_yaw_source
            target_source = self.room_frontier_target_source
            target_metric = self.room_frontier_target_metric
            path_source = self.room_frontier_path_source
            path_index = self.room_frontier_path_index
            plan_time = self.room_frontier_plan_time
            planned_map_update = self.room_frontier_planned_map_update
            planned_camera_update = self.room_frontier_planned_camera_update
            camera_update = self.camera_observed_update
            camera_room_id = self.camera_room_id
            camera_points = tuple(self.camera_observed_world_points)
            map_update = self.last_map_update
            front_clearance = self.front_clearance
            front_nearest = self.front_nearest_clearance
            grid_frame = grid.frame_id if grid is not None else self.map_frame
        now = rospy.Time.now()
        target_active = (
            target_source is not None
            and target_metric is not None
            and bool(path_source)
        )
        should_replan = not target_active and (
            plan_time == rospy.Time(0)
            or (now - plan_time).to_sec() >= self.room_frontier_replan_period
        )
        if (
            not target_active
            and self.camera_coverage_enabled
            and camera_room_id == candidate.candidate_id
            and camera_update > planned_camera_update
        ):
            should_replan = True

        # A committed target survives ordinary map growth.  Only invalidate
        # it when the newly received map makes a remaining path cell unsafe;
        # this prevents the one-second target churn seen in the acceptance run.
        if target_active and map_update != rospy.Time(0) and (
            planned_map_update == rospy.Time(0) or map_update > planned_map_update
        ):
            remaining_path = path_source[max(0, min(path_index, len(path_source) - 1)) :]
            if not self.room_frontier_planner.path_is_safe(
                grid,
                remaining_path,
                minimum_clearance=self.room_robot_radius
                + self.room_frontier_revalidate_margin,
            ):
                rospy.loginfo(
                    "ROOM_FRONTIER committed path invalidated by map update; replanning"
                )
                self._stop()
                self.room_frontier_plan = None
                self.room_frontier_target_source = None
                self.room_frontier_target_metric = None
                self.room_frontier_path_source = ()
                self.room_frontier_path_index = 0
                self.room_frontier_plan_time = rospy.Time(0)
                self.room_frontier_planned_map_update = rospy.Time(0)
                target_source = None
                target_metric = None
                path_source = ()
                path_index = 0
                plan_time = rospy.Time(0)
                target_active = False
                should_replan = True
            else:
                self.room_frontier_planned_map_update = map_update
        grid_pose = (
            self._source_pose_to_grid_locked(source_pose, grid_frame)
            if source_pose is not None
            else None
        )
        if (
            should_replan
            and grid is not None
            and grid_pose is not None
            and door_center is not None
        ):
            camera_seen = None
            if self.camera_coverage_enabled and camera_room_id == candidate.candidate_id:
                camera_seen = self._camera_seen_grid_locked(camera_points, grid)
            plan = self.room_frontier_planner.plan(
                grid,
                grid_pose,
                door_center,
                inward_yaw,
                self.room_frontier_visited_targets,
                visual_seen=camera_seen,
                visual_coverage_target=self.camera_coverage_target,
            )
            self.room_frontier_plan = plan
            self.room_frontier_plan_time = now
            self.room_frontier_planned_camera_update = camera_update
            self.room_frontier_replans += 1
            self.room_frontier_last_reason = plan.reason
            self.room_frontier_coverage = plan.coverage
            self.room_frontier_raw_coverage = plan.raw_coverage
            self.room_frontier_stats = {
                "reachable_cells": plan.reachable_cells,
                "safe_cells": plan.safe_cells,
                "unknown_frontier_cells": plan.unknown_frontier_cells,
                "room_free_cells": plan.room_free_cells,
                "coverage": round(plan.coverage, 4),
                "safe_coverage": round(plan.coverage, 4),
                "raw_coverage": round(plan.raw_coverage, 4),
                "coverage_target": round(self.room_frontier_coverage_target, 4),
                "coverage_target_met": plan.coverage >= self.room_frontier_coverage_target,
                "camera_coverage": round(plan.visual_coverage, 4),
                "camera_unseen_cells": plan.visual_unseen_cells,
                "camera_frontier_cells": plan.visual_frontier_cells,
                "camera_coverage_target": round(self.camera_coverage_target, 4),
                "camera_coverage_target_met": plan.visual_complete,
                "ring_detected": plan.ring_detected,
                "ring_center": plan.ring_center,
                "ring_candidate_count": plan.ring_candidate_count,
                "ring_direction": plan.ring_direction,
                "frontier_count": len(plan.frontiers),
            }
            if plan.frontier is None:
                self.room_frontier_target_source = None
                self.room_frontier_target_metric = None
                if plan.reason == "NO_FRONTIER":
                    if self.camera_coverage_enabled and not plan.visual_complete:
                        entry_pose = self.room_entry_source_pose
                        advance_yaw = inward_yaw
                        advance_progress = None
                        if (
                            entry_pose is not None
                            and source_pose is not None
                            and advance_yaw is not None
                        ):
                            delta_x = float(source_pose[0]) - float(entry_pose[0])
                            delta_y = float(source_pose[1]) - float(entry_pose[1])
                            advance_progress = (
                                delta_x * math.cos(float(advance_yaw))
                                + delta_y * math.sin(float(advance_yaw))
                            )
                        if (
                            self.room_entry_advance_distance > 0.0
                            and plan.reachable_cells
                            < self.room_entry_advance_min_reachable_cells
                            and advance_progress is not None
                            and advance_progress < self.room_entry_advance_distance
                            and (
                                not math.isfinite(front_clearance)
                                or front_clearance >= self.room_motion_stop_distance
                            )
                        ):
                            command = Twist()
                            command.linear.x = min(
                                self.room_frontier_min_speed,
                                self.room_frontier_speed,
                            )
                            self._publish_command(command)
                            rospy.loginfo_throttle(
                                2.0,
                                "ROOM_FRONTIER entry advance %.2f/%.2f m; "
                                "laser reachable cells=%d",
                                max(0.0, advance_progress),
                                self.room_entry_advance_distance,
                                plan.reachable_cells,
                            )
                            self.room_frontier_quiet_since = None
                            self.room_frontier_plan_time = now
                            return
                        if (
                            self.camera_sweep_enabled
                            and not self.camera_sweep_complete
                            and self.camera_sweep_max_angle > 0.0
                        ):
                            if not self.camera_sweep_active:
                                self.camera_sweep_active = True
                                self.camera_sweep_last_yaw = float(pose[2])
                            elif self.camera_sweep_last_yaw is not None:
                                yaw_step = abs(
                                    normalize_angle(
                                        float(pose[2]) - self.camera_sweep_last_yaw
                                    )
                                )
                                # Reject a localization discontinuity rather
                                # than counting it as camera motion.
                                if yaw_step <= 0.50:
                                    self.camera_sweep_accumulated += yaw_step
                                self.camera_sweep_last_yaw = float(pose[2])
                            if (
                                self.camera_sweep_accumulated
                                < self.camera_sweep_max_angle
                            ):
                                command = Twist()
                                command.angular.z = self.camera_sweep_speed
                                self._publish_command(command)
                                rospy.loginfo_throttle(
                                    2.0,
                                    "ROOM_FRONTIER bounded camera sweep %.2f/%.2f rad; "
                                    "coverage %.3f/%.3f",
                                    self.camera_sweep_accumulated,
                                    self.camera_sweep_max_angle,
                                    plan.visual_coverage,
                                    self.camera_coverage_target,
                                )
                                self.room_frontier_quiet_since = None
                                self.room_frontier_plan_time = now
                                return
                            self.camera_sweep_active = False
                            self.camera_sweep_complete = True
                            self.camera_sweep_last_yaw = None
                            self._stop()
                            rospy.logwarn(
                                "ROOM_FRONTIER bounded camera sweep exhausted at "
                                "coverage %.3f; no safe translation frontier remains",
                                plan.visual_coverage,
                            )
                        rospy.loginfo_throttle(
                            5.0,
                            "ROOM_FRONTIER laser frontier quiet but camera coverage %.3f "
                            "is below target %.3f; no safe visual frontier remains",
                            plan.visual_coverage,
                            self.camera_coverage_target,
                        )
                    if self.room_frontier_quiet_since is None:
                        self.room_frontier_quiet_since = now
                    if (
                        (now - self.room_frontier_quiet_since).to_sec()
                        >= self.room_frontier_quiet_period
                    ):
                        if plan.coverage < self.room_frontier_coverage_target:
                            rospy.logwarn(
                                "ROOM_FRONTIER quiet with safe coverage %.3f below target %.3f; "
                                "completing because no safe frontier remains",
                                plan.coverage,
                                self.room_frontier_coverage_target,
                            )
                        self._complete_room_exploration(candidate)
                        return
                else:
                    self.room_frontier_quiet_since = None
                self._stop()
                return
            self.room_frontier_quiet_since = None
            self.camera_sweep_active = False
            self.camera_sweep_last_yaw = None
            selected = plan.frontier
            self.room_frontier_path_source = selected.path or (selected.target,)
            self.room_frontier_path_index = min(1, len(self.room_frontier_path_source) - 1)
            self.room_frontier_planned_map_update = map_update
            self.room_frontier_planned_camera_update = camera_update
            target_source = self.room_frontier_path_source[self.room_frontier_path_index]
            self.room_frontier_target_source = target_source
            path_source = self.room_frontier_path_source
            path_index = self.room_frontier_path_index
            with self.lock:
                self.room_frontier_target_metric = (
                    self._grid_point_to_metric_locked(target_source, grid_frame)
                    if self.world_pose is not None
                    else None
                )
                target_metric = self.room_frontier_target_metric
            rospy.loginfo(
                "ROOM_FRONTIER target=(%.2f, %.2f) path=%.2f gain=%.2f "
                "clearance=%.2f branch=%d escape=%d risk=%.2f",
                selected.target[0],
                selected.target[1],
                selected.path_length,
                selected.information_gain,
                selected.min_clearance,
                selected.branch_cells,
                selected.escape_cells,
                selected.dead_end_risk,
            )
        if target_metric is None:
            self._stop()
            return
        selected = self.room_frontier_plan.frontier if self.room_frontier_plan else None
        target_tolerance = self.room_frontier_target_tolerance
        if selected is not None:
            # Keep a turn inside the inflated free-space margin instead of
            # cutting a corner while the waypoint is still considered reached.
            target_tolerance = min(
                target_tolerance,
                max(0.12, 0.45 * (selected.min_clearance - self.room_robot_radius)),
            )
        distance = math.hypot(
            target_metric[0] - pose[0], target_metric[1] - pose[1]
        )
        if distance <= target_tolerance:
            self._stop()
            if path_source and path_index + 1 < len(path_source):
                self.room_frontier_path_index += 1
                next_source = path_source[self.room_frontier_path_index]
                self.room_frontier_target_source = next_source
                with self.lock:
                    self.room_frontier_target_metric = self._grid_point_to_metric_locked(
                        next_source, grid_frame
                    )
                return
            if target_source is not None:
                self.room_frontier_visited_targets.append(tuple(target_source))
            self.room_frontier_targets_reached += 1
            self.room_frontier_target_source = None
            self.room_frontier_target_metric = None
            self.room_frontier_path_source = ()
            self.room_frontier_path_index = 0
            self.room_frontier_plan_time = rospy.Time(0)
            self.room_frontier_planned_map_update = rospy.Time(0)
            self.room_frontier_planned_camera_update = rospy.Time(0)
            return
        target_yaw = math.atan2(
            target_metric[1] - pose[1], target_metric[0] - pose[0]
        )
        yaw_error = normalize_angle(target_yaw - pose[2])
        obstacle_close = (
            math.isfinite(front_nearest)
            and front_nearest < self.room_motion_stop_distance
        ) or (
            math.isfinite(front_clearance)
            and front_clearance < self.room_motion_stop_distance
        )
        if obstacle_close and abs(yaw_error) <= self.room_frontier_heading_tolerance:
            rospy.loginfo(
                "ROOM_FRONTIER target abandoned at obstacle clearance %.2f m; replanning",
                min(front_nearest, front_clearance),
            )
            self._stop()
            if target_source is not None:
                self.room_frontier_visited_targets.append(tuple(target_source))
            self.room_frontier_target_source = None
            self.room_frontier_target_metric = None
            self.room_frontier_path_source = ()
            self.room_frontier_path_index = 0
            self.room_frontier_plan_time = rospy.Time(0)
            self.room_frontier_planned_map_update = rospy.Time(0)
            self.room_frontier_planned_camera_update = rospy.Time(0)
            return
        command = Twist()
        if abs(yaw_error) > self.room_frontier_heading_tolerance:
            command.angular.z = math.copysign(min(self.align_speed, 0.65), yaw_error)
        else:
            # The planner's min_clearance is an inflated-space feasibility
            # threshold.  With a 0.38 m A1 radius and 0.04 m margin it is
            # intentionally about 0.42 m, so using it as a speed cap would
            # reduce the RL command to 0.10 m/s and prevent the gait from
            # starting.  Front clearance below still stops before contact.
            safe_speed = max(self.room_frontier_min_speed, self.room_frontier_speed)
            if math.isfinite(front_clearance):
                slow_distance = max(self.room_motion_stop_distance + 0.25, 0.75)
                if front_clearance < slow_distance:
                    fraction = max(
                        0.0,
                        min(
                            1.0,
                            (front_clearance - self.room_motion_stop_distance)
                            / max(0.05, slow_distance - self.room_motion_stop_distance),
                        ),
                    )
                    safe_speed = self.room_frontier_min_speed + fraction * (
                        safe_speed - self.room_frontier_min_speed
                    )
            command.linear.x = safe_speed
            command.angular.z = max(-0.15, min(0.15, 0.8 * yaw_error))
        self._publish_command(command)

    def _complete_room_exploration(self, candidate):
        self._stop()
        with self.lock:
            accumulated = self.scan_accumulated_points.copy()
        current_points = accumulated if len(accumulated) >= 40 else None
        self._request_scan_loop_closure(
            "{}_room_entry".format(candidate.candidate_id),
            self.room_entry_loop_anchor,
            "room_entry_local_submap",
            preserve_danger_ids=self.room_entry_danger_ids,
            current_points=current_points,
        )
        duration = (
            rospy.Time.now().to_sec() - self.room_exploration_start_time
            if self.room_exploration_start_time is not None
            else 0.0
        )
        rospy.loginfo(
            "ROOM_EXPLORATION_SUMMARY %s",
            json.dumps(
                {
                    "room_id": candidate.candidate_id,
                    "danger_found_ingress": len(self.danger_phase_ids.get("INGRESS", set())),
                    "danger_found_room_frontier": len(self.danger_phase_ids.get("ROOM_FRONTIER", set())),
                    "danger_found_egress": len(self.danger_phase_ids.get("EGRESS", set())),
                    "frontier_replans": self.room_frontier_replans,
                    "frontier_targets_reached": self.room_frontier_targets_reached,
                    "frontier_last_reason": self.room_frontier_last_reason,
                    "frontier_coverage": round(self.room_frontier_coverage, 4),
                    "frontier_safe_coverage": round(self.room_frontier_coverage, 4),
                    "frontier_raw_coverage": round(self.room_frontier_raw_coverage, 4),
                    "frontier_coverage_target": round(self.room_frontier_coverage_target, 4),
                    "frontier_coverage_target_met": (
                        self.room_frontier_coverage >= self.room_frontier_coverage_target
                    ),
                    "camera_coverage": round(
                        self.room_frontier_plan.visual_coverage, 4
                    )
                    if self.room_frontier_plan is not None
                    else 0.0,
                    "camera_unseen_cells": (
                        self.room_frontier_plan.visual_unseen_cells
                        if self.room_frontier_plan is not None
                        else self.camera_observed_count
                    ),
                    "camera_coverage_target": round(self.camera_coverage_target, 4),
                    "camera_coverage_target_met": (
                        self.room_frontier_plan.visual_complete
                        if self.room_frontier_plan is not None
                        else not self.camera_coverage_enabled
                    ),
                    "frontier_stats": self.room_frontier_stats,
                    "frontier_grid_frame": (
                        self.grid.frame_id if self.grid is not None else self.map_frame
                    ),
                    "room_exploration_duration": round(duration, 3),
                },
                sort_keys=True,
            ),
        )
        self._advance(True)

    def _set_room_phase(self, phase):
        self.scan_phase = phase
        self.scan_phase_start = rospy.Time.now()

    def _scan_phase_elapsed(self):
        if self.scan_phase_start == rospy.Time(0):
            return 0.0
        return (rospy.Time.now() - self.scan_phase_start).to_sec()

    def _advance(self, succeeded):
        next_state = self.mission.advance(succeeded, rospy.Time.now().to_sec())
        self.goal_state = None
        if next_state not in (None, "VISITED", "UNREACHABLE", "DISCOVERED"):
            self._enter_state(next_state)

    def _enter_state(self, state):
        self.state_start = rospy.Time.now()
        self.goal_state = None
        active_id = self.mission.active_id
        if state in (GO_TO_PRE_DOOR, ALIGN_TO_DOOR_NORMAL, DOOR_CROSSING):
            if active_id is not None:
                self.room_danger_id_baselines.setdefault(
                    active_id, set(self.confirmed_danger_ids)
                )
        if state == GO_TO_PRE_DOOR:
            self.pre_door_phase = "STATION"
            self.pre_door_station_error = None
            self.pre_door_normal_error = None
            self.pre_door_face_stable_since = None
        if state == ROOM_SCAN:
            with self.lock:
                metric_pose = self.world_pose
                source_pose = self.pose
                grid = self.grid
                self.scan_accumulated_points = np.empty((0, 2), dtype=np.float64)
                self.room_entry_source_pose = tuple(source_pose) if source_pose is not None else None
                self.room_entry_metric_pose = tuple(metric_pose) if metric_pose is not None else None
                active = self.mission.candidates.get(active_id) if active_id is not None else None
                if active is not None and source_pose is not None and metric_pose is not None:
                    grid_frame = grid.frame_id if grid is not None else self.map_frame
                    self.room_entry_door_source = self._metric_point_to_grid_locked(
                        active.center, grid_frame
                    )
                    transformed_yaw = self._transform_planar_pose_locked(
                        (0.0, 0.0, active.normal_yaw),
                        self.world_frame,
                        grid_frame,
                    )
                    self.room_entry_inward_yaw_source = (
                        transformed_yaw[2]
                        if transformed_yaw is not None
                        else self._metric_yaw_to_source_locked(active.normal_yaw)
                    )
                else:
                    self.room_entry_door_source = None
                    self.room_entry_inward_yaw_source = None
            self.danger_phase_ids = {}
            self.room_exploration_start_time = rospy.Time.now().to_sec()
            self.room_entry_loop_anchor = self._scan_points_in_map()
            self.room_entry_danger_ids = set(self.confirmed_danger_ids)
            self.camera_room_id = active.candidate_id if active is not None else None
            self.camera_observed_world_points = set()
            self.camera_observed_update = rospy.Time(0)
            self.camera_observed_count = 0
            self.camera_observed_ray_count = 0
            self.camera_sweep_active = False
            self.camera_sweep_complete = False
            self.camera_sweep_last_yaw = None
            self.camera_sweep_accumulated = 0.0
            if metric_pose is not None and active is not None:
                self.room_entry_pub.publish(
                    String(
                        data=json.dumps(
                            {
                                "candidate_id": active.candidate_id,
                                "pose": [float(value) for value in metric_pose],
                                "timestamp": rospy.Time.now().to_sec(),
                            },
                            sort_keys=True,
                        )
                    )
                )
            self.room_frontier_plan = None
            self.room_frontier_target_source = None
            self.room_frontier_target_metric = None
            self.room_frontier_path_source = ()
            self.room_frontier_path_index = 0
            self.room_frontier_plan_time = rospy.Time(0)
            self.room_frontier_planned_map_update = rospy.Time(0)
            self.room_frontier_planned_camera_update = rospy.Time(0)
            self.room_frontier_quiet_since = None
            self.room_frontier_visited_targets = []
            self.room_frontier_replans = 0
            self.room_frontier_targets_reached = 0
            self.room_frontier_last_reason = "NOT_STARTED"
            self.room_frontier_coverage = 0.0
            self.room_frontier_raw_coverage = 0.0
            self.room_frontier_stats = {}
            self.room_valid_frame_count = 0
            self._set_room_phase("ROOM_FRONTIER_EXPLORE")
        elif state == DOOR_CROSSING:
            with self.lock:
                pose = self.pose
                metric_pose = self.world_pose
            active = (
                self.mission.candidates.get(self.mission.active_id)
                if self.mission.active_id is not None
                else None
            )
            if active is not None and pose is not None:
                self.doorway_loop_anchors[
                    active.candidate_id
                ] = self._scan_points_in_map()
            self.inside_invalid_start = None
            self.door_crossing_origin_metric = (
                tuple(metric_pose[:2]) if metric_pose is not None else None
            )
            self.door_crossing_last_depth = None
            self.door_crossing_last_lateral = None
            self.door_crossing_last_travel = None
            self.crossing_corridor_seen = bool(
                pose is not None and self._corridor_structure_valid(pose)
            )
        elif state == EXIT_ROOM:
            self.corridor_reacquire_start = None
            self.opposite_door_probe_until = rospy.Time(0)
            self.exit_danger_id_baseline = set(self.confirmed_danger_ids)

    def _state_elapsed(self):
        if self.state_start == rospy.Time(0):
            return 0.0
        return (rospy.Time.now() - self.state_start).to_sec()

    def _publish_status(self, state):
        counts = {}
        for item in self.mission.candidates.values():
            counts[item.status] = counts.get(item.status, 0) + 1
        active_candidate = None
        active_attempt = None
        active_exit_attempt = None
        active_candidate_geometry = None
        if self.mission.active_id is not None:
            active = self.mission.candidates.get(self.mission.active_id)
            if active is not None:
                active_candidate = active.candidate_id
                active_attempt = active.attempts
                active_exit_attempt = active.exit_attempts
                active_candidate_geometry = {
                    "center": [float(value) for value in active.center],
                    "width": float(active.width),
                    "normal_yaw": float(active.normal_yaw),
                    "pre_pose": [float(value) for value in active.pre_pose],
                    "post_pose": [float(value) for value in active.post_pose],
                }
        payload = {
            "state": state,
            "state_elapsed": round(self._state_elapsed(), 3),
            "expected_rooms_per_floor": self.expected_rooms_per_floor,
            "counts": counts,
            "candidate_statuses": {
                item.candidate_id: item.status
                for item in self.mission.candidates.values()
            },
            "localization_health": self.localization_health,
            "mission_fault": self.mission_fault,
            "fault_terminal_state": ABORT_RUN if self.mission_fault else None,
            "controller_health": self.controller_health,
            "lio_health": self.lio_health,
            "lio_effective_points": self.lio_effective_points,
            "danger_confirmation_active": self.danger_confirmation_active,
            "reverse_sweeps": self.mission.recovery.sweep_count,
            "door_hypotheses": len(self.door_fusion.hypotheses),
            "visited_door_revisit_rejections": self.mission.revisit_rejections,
            "prefer_opposite_door": self.prefer_opposite_door,
            "prefer_paired_station": self.prefer_paired_station,
            "opposite_pair_station_tolerance": self.opposite_pair_station_tolerance,
            "opposite_door_probe_duration": self.opposite_door_probe_duration,
            "opposite_pair_selections": self.mission.opposite_pair_selections,
            "last_door_selection_reason": self.mission.last_selection_reason,
            "initial_forward_active": self.initial_forward_active,
            "initial_forward_distance": self.initial_forward_distance,
            "initial_forward_progress": self._initial_forward_progress(),
            "topology_region": self.topology_region,
            "topology_rejected_evidence": self.topology_rejected_evidence,
            "lobby_entry_door": (
                [float(value) for value in self.lobby_entry_door]
                if self.lobby_entry_door is not None
                else None
            ),
            "virtual_isolation_door": (
                [float(value) for value in self.virtual_isolation_door]
                if self.virtual_isolation_door is not None
                else None
            ),
            "active_candidate": active_candidate,
            "active_candidate_geometry": active_candidate_geometry,
            "active_door_attempt": active_attempt,
            "active_exit_attempt": active_exit_attempt,
            "pre_door_phase": (
                self.pre_door_phase if state == GO_TO_PRE_DOOR else None
            ),
            "pre_door_station_error": (
                round(self.pre_door_station_error, 4)
                if state == GO_TO_PRE_DOOR
                and self.pre_door_station_error is not None
                else None
            ),
            "pre_door_normal_error": (
                round(self.pre_door_normal_error, 4)
                if state == GO_TO_PRE_DOOR
                and self.pre_door_normal_error is not None
                else None
            ),
            "door_crossing_depth": (
                round(self.door_crossing_last_depth, 4)
                if state in (DOOR_CROSSING, ROOM_SCAN)
                and self.door_crossing_last_depth is not None
                else None
            ),
            "door_crossing_lateral_error": (
                round(self.door_crossing_last_lateral, 4)
                if state in (DOOR_CROSSING, ROOM_SCAN)
                and self.door_crossing_last_lateral is not None
                else None
            ),
            "door_crossing_travel": (
                round(self.door_crossing_last_travel, 4)
                if state in (DOOR_CROSSING, ROOM_SCAN)
                and self.door_crossing_last_travel is not None
                else None
            ),
            "room_scan_phase": self.scan_phase if state == ROOM_SCAN else None,
            "room_frontier_plan": (
                self.room_frontier_stats if state == ROOM_SCAN else None
            ),
            "room_frontier_grid_frame": (
                self.grid.frame_id
                if state == ROOM_SCAN and self.grid is not None
                else None
            ),
            "room_frontier_marker_frame": self.world_frame if state == ROOM_SCAN else None,
            "room_frontier_target": (
                [float(value) for value in self.room_frontier_target_metric]
                if state == ROOM_SCAN and self.room_frontier_target_metric is not None
                else None
            ),
            "room_frontier_replans": self.room_frontier_replans if state == ROOM_SCAN else None,
            "room_frontier_targets_reached": (
                self.room_frontier_targets_reached if state == ROOM_SCAN else None
            ),
            "room_frontier_last_reason": (
                self.room_frontier_last_reason if state == ROOM_SCAN else None
            ),
            "room_frontier_coverage": (
                round(self.room_frontier_coverage, 4) if state == ROOM_SCAN else None
            ),
            "room_frontier_raw_coverage": (
                round(self.room_frontier_raw_coverage, 4) if state == ROOM_SCAN else None
            ),
            "room_frontier_coverage_target": (
                round(self.room_frontier_coverage_target, 4) if state == ROOM_SCAN else None
            ),
            "room_camera_coverage": (
                round(
                    self.room_frontier_plan.visual_coverage
                    if self.room_frontier_plan is not None
                    else 0.0,
                    4,
                )
                if state == ROOM_SCAN
                else None
            ),
            "room_camera_coverage_target": (
                round(self.camera_coverage_target, 4) if state == ROOM_SCAN else None
            ),
            "room_camera_unseen_cells": (
                self.room_frontier_plan.visual_unseen_cells
                if state == ROOM_SCAN and self.room_frontier_plan is not None
                else None
            ),
            "room_camera_observed_cells": (
                self.camera_observed_count if state == ROOM_SCAN else None
            ),
            "room_frontier_valid_frames": (
                self.room_valid_frame_count if state == ROOM_SCAN else None
            ),
            "confirmed_dangers": self.confirmed_danger_count,
            "local_loop_closure": self.loop_closure_snapshot,
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _initial_forward_progress(self):
        with self.lock:
            anchor = self.initial_forward_anchor
            yaw = self.initial_forward_yaw
            pose = self.world_pose
        if anchor is None or yaw is None or pose is None:
            return 0.0
        return max(
            0.0,
            (pose[0] - anchor[0]) * math.cos(yaw)
            + (pose[1] - anchor[1]) * math.sin(yaw),
        )

    def _publish_localization_health(self):
        self.health_pub.publish(
            String(data=json.dumps(self.localization_monitor.snapshot(), sort_keys=True))
        )

    @staticmethod
    def _finite_or_none(value):
        return float(value) if math.isfinite(value) else None

    def _publish_corridor_estimate(self, estimate):
        payload = {
            "valid": estimate.valid,
            "confidence": estimate.confidence,
            "axis_yaw": estimate.axis_yaw,
            "left_wall_distance": self._finite_or_none(estimate.left_wall_distance),
            "right_wall_distance": self._finite_or_none(estimate.right_wall_distance),
            "corridor_width": self._finite_or_none(estimate.corridor_width),
            "center_error": estimate.center_error,
            "front_clearance": self._finite_or_none(estimate.front_clearance),
        }
        self.corridor_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_door_evidence(self, evidence):
        payload = [
            {
                "source": item.source,
                "timestamp": item.timestamp,
                "side": item.side,
                "center": list(item.center),
                "width": item.width,
                "normal_yaw": item.normal_yaw,
                "corridor_confidence": item.corridor_confidence,
                "source_confidence": item.source_confidence,
                "localization_health": item.localization_health,
                "verification_status": item.verification_status,
            }
            for item in evidence
        ]
        self.evidence_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_door_hypotheses(self):
        payload = [
            {
                "id": item.hypothesis_id,
                "side": item.side,
                "center": list(item.center),
                "width": item.width,
                "normal_yaw": item.normal_yaw,
                "scan_support": item.scan_support,
                "map_support": item.map_support,
                "confidence": item.confidence,
                "status": item.status,
            }
            for item in self.door_fusion.hypotheses.values()
        ]
        self.hypothesis_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.world_frame
        clear.header.stamp = rospy.Time.now()
        clear.ns = "opening_candidates"
        clear.id = 0
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, candidate in enumerate(self.mission.candidates.values()):
            # A visited doorway is history, not the next navigation target.
            # Keeping it green made RViz appear to point back into the old room.
            if candidate.status in (VISITED, UNREACHABLE):
                continue
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = clear.header.stamp
            marker.ns = "opening_candidates"
            marker.id = index + 1
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = candidate.center[0]
            marker.pose.position.y = candidate.center[1]
            marker.pose.position.z = 0.15
            marker.pose.orientation.z = math.sin(candidate.normal_yaw / 2.0)
            marker.pose.orientation.w = math.cos(candidate.normal_yaw / 2.0)
            marker.scale.x = 0.55
            marker.scale.y = 0.10
            marker.scale.z = 0.10
            marker.color.a = 0.95
            if candidate.status == DISCOVERED:
                marker.color.r = 0.1
                marker.color.g = 1.0
                marker.color.b = 0.2
            else:
                marker.color.r = 1.0
                marker.color.g = 0.65
                marker.color.b = 0.05
            markers.markers.append(marker)

        with self.lock:
            mission_ids = set(self.mission.candidates)
            pending_hypotheses = [
                hypothesis
                for hypothesis in self.door_fusion.hypotheses.values()
                if hypothesis.status == PENDING
                and hypothesis.hypothesis_id not in mission_ids
            ]
            virtual_gate = self.virtual_isolation_door
            lobby_gate = self.lobby_entry_door
            active = (
                self.mission.candidates.get(self.mission.active_id)
                if self.mission.active_id is not None
                else None
            )

        # Pending scan hypotheses are shown in blue for diagnosis, but are not
        # selectable until DoorVerifier or Map evidence confirms them.
        for offset, hypothesis in enumerate(pending_hypotheses):
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = clear.header.stamp
            marker.ns = "pending_hypotheses"
            marker.id = 1000 + offset
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = hypothesis.center[0]
            marker.pose.position.y = hypothesis.center[1]
            marker.pose.position.z = 0.12
            marker.pose.orientation.z = math.sin(hypothesis.normal_yaw / 2.0)
            marker.pose.orientation.w = math.cos(hypothesis.normal_yaw / 2.0)
            marker.scale.x = 0.45
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.r = 0.15
            marker.color.g = 0.45
            marker.color.b = 1.0
            marker.color.a = 0.65
            markers.markers.append(marker)

        for marker_id, gate, red, green, blue in (
            (2000, lobby_gate, 0.95, 0.65, 0.05),
            (2002, virtual_gate, 0.1, 0.85, 0.95),
        ):
            if gate is None:
                continue
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = clear.header.stamp
            marker.ns = "virtual_isolation"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = gate[0]
            marker.pose.position.y = gate[1]
            marker.pose.position.z = 0.05
            marker.pose.orientation.z = math.sin(gate[2] / 2.0)
            marker.pose.orientation.w = math.cos(gate[2] / 2.0)
            marker.scale.x = 0.08
            marker.scale.y = self.topology_gate_width
            marker.scale.z = 0.08
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = 0.90
            markers.markers.append(marker)

        if virtual_gate is not None:
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = clear.header.stamp
            marker.ns = "virtual_isolation"
            marker.id = 2003
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = virtual_gate[0]
            marker.pose.position.y = virtual_gate[1]
            marker.pose.position.z = 0.11
            marker.pose.orientation.z = math.sin(virtual_gate[2] / 2.0)
            marker.pose.orientation.w = math.cos(virtual_gate[2] / 2.0)
            marker.scale.x = 0.65
            marker.scale.y = 0.10
            marker.scale.z = 0.10
            marker.color.r = 0.1
            marker.color.g = 0.85
            marker.color.b = 0.95
            marker.color.a = 0.90
            markers.markers.append(marker)

        if active is not None and active.status in (DOOR_CROSSING, ROOM_SCAN, EXIT_ROOM):
            depth = (
                self.inside_room_entry_depth
                if active.status in (DOOR_CROSSING, ROOM_SCAN)
                else -self.corridor_side_entry_depth
            )
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = clear.header.stamp
            marker.ns = "virtual_isolation"
            marker.id = 2001
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = active.center[0] + depth * math.cos(active.normal_yaw)
            marker.pose.position.y = active.center[1] + depth * math.sin(active.normal_yaw)
            marker.pose.position.z = 0.11
            marker.pose.orientation.z = math.sin(active.normal_yaw / 2.0)
            marker.pose.orientation.w = math.cos(active.normal_yaw / 2.0)
            marker.scale.x = 0.38
            marker.scale.y = 0.10
            marker.scale.z = 0.10
            marker.color.r = 0.95
            marker.color.g = 0.25
            marker.color.b = 0.75
            marker.color.a = 0.85
            markers.markers.append(marker)
        self.marker_pub.publish(markers)
        self._publish_room_frontier_markers()

    def _publish_room_frontier_markers(self):
        """Expose the active room target and path for RViz diagnosis."""
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.world_frame
        clear.header.stamp = rospy.Time.now()
        clear.ns = "room_frontier"
        clear.id = 0
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        with self.lock:
            active = (
                self.mission.candidates.get(self.mission.active_id)
                if self.mission.active_id is not None
                else None
            )
            if active is None or active.status != ROOM_SCAN:
                empty_path = NavPath()
                empty_path.header.frame_id = self.world_frame
                empty_path.header.stamp = clear.header.stamp
                self.room_frontier_path_pub.publish(empty_path)
                self.room_frontier_marker_pub.publish(markers)
                return
            path_source = tuple(self.room_frontier_path_source)
            target_metric = self.room_frontier_target_metric
            plan = self.room_frontier_plan
            grid_frame = self.grid.frame_id if self.grid is not None else self.map_frame
            if self.pose is None or self.world_pose is None:
                path_metric = []
                ring_center_metric = None
                ring_candidates_metric = []
            else:
                path_metric = [
                    self._grid_point_to_metric_locked(point, grid_frame)
                    for point in path_source
                ]
                ring_center_metric = (
                    self._grid_point_to_metric_locked(plan.ring_center, grid_frame)
                    if plan is not None and plan.ring_center is not None
                    else None
                )
                ring_candidates_metric = [
                    self._grid_point_to_metric_locked(frontier.target, grid_frame)
                    for frontier in (plan.frontiers if plan is not None else ())
                    if frontier.visual and math.isfinite(frontier.ring_progress)
                ]

        path_message = NavPath()
        path_message.header.frame_id = self.world_frame
        path_message.header.stamp = clear.header.stamp
        for x, y in path_metric:
            pose = PoseStamped()
            pose.header = path_message.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.18
            pose.pose.orientation.w = 1.0
            path_message.poses.append(pose)
        self.room_frontier_path_pub.publish(path_message)

        if len(path_metric) >= 2:
            path = Marker()
            path.header.frame_id = self.world_frame
            path.header.stamp = rospy.Time.now()
            path.ns = "room_frontier"
            path.id = 1
            path.type = Marker.LINE_STRIP
            path.action = Marker.ADD
            path.pose.orientation.w = 1.0
            path.scale.x = 0.06
            path.color.a = 0.95
            path.color.r = 0.1
            path.color.g = 0.8
            path.color.b = 1.0
            for x, y in path_metric:
                point = Point()
                point.x = float(x)
                point.y = float(y)
                point.z = 0.18
                path.points.append(point)
            markers.markers.append(path)

        if target_metric is not None:
            target = Marker()
            target.header.frame_id = self.world_frame
            target.header.stamp = rospy.Time.now()
            target.ns = "room_frontier"
            target.id = 2
            target.type = Marker.SPHERE
            target.action = Marker.ADD
            target.pose.position.x = float(target_metric[0])
            target.pose.position.y = float(target_metric[1])
            target.pose.position.z = 0.28
            target.pose.orientation.w = 1.0
            target.scale.x = 0.32
            target.scale.y = 0.32
            target.scale.z = 0.32
            target.color.a = 1.0
            target.color.r = 0.1
            target.color.g = 1.0
            target.color.b = 0.15
            markers.markers.append(target)

        if ring_center_metric is not None:
            center = Marker()
            center.header.frame_id = self.world_frame
            center.header.stamp = rospy.Time.now()
            center.ns = "room_frontier_ring"
            center.id = 3
            center.type = Marker.CYLINDER
            center.action = Marker.ADD
            center.pose.position.x = float(ring_center_metric[0])
            center.pose.position.y = float(ring_center_metric[1])
            center.pose.position.z = 0.12
            center.pose.orientation.w = 1.0
            center.scale.x = 0.50
            center.scale.y = 0.50
            center.scale.z = 0.24
            center.color.a = 0.90
            center.color.r = 1.0
            center.color.g = 0.50
            center.color.b = 0.05
            markers.markers.append(center)

        if ring_candidates_metric:
            candidates = Marker()
            candidates.header.frame_id = self.world_frame
            candidates.header.stamp = rospy.Time.now()
            candidates.ns = "room_frontier_ring"
            candidates.id = 4
            candidates.type = Marker.SPHERE_LIST
            candidates.action = Marker.ADD
            candidates.pose.orientation.w = 1.0
            candidates.scale.x = 0.14
            candidates.scale.y = 0.14
            candidates.scale.z = 0.14
            candidates.color.a = 0.75
            candidates.color.r = 1.0
            candidates.color.g = 0.55
            candidates.color.b = 0.05
            for x, y in ring_candidates_metric:
                point = Point()
                point.x = float(x)
                point.y = float(y)
                point.z = 0.18
                candidates.points.append(point)
            markers.markers.append(candidates)
        self.room_frontier_marker_pub.publish(markers)

    def _publish_configured_entrance_gate(self):
        points = rospy.get_param("~entrance_gate", [])
        if len(points) != 2:
            return
        gate = PolygonStamped()
        gate.header.frame_id = "simnav_map"
        gate.header.stamp = rospy.Time.now()
        for coordinates in points:
            gate.polygon.points.append(Point32(x=float(coordinates[0]), y=float(coordinates[1])))
        self.gate_pub.publish(gate)

    def _publish_configured_defer_zones(self):
        """Publish architecture-owned regions where openings are not room doors."""
        zones = rospy.get_param("~defer_zones", [])
        if not isinstance(zones, list):
            return
        for coordinates in zones:
            if not isinstance(coordinates, list) or len(coordinates) < 3:
                continue
            polygon = []
            message = PolygonStamped()
            message.header.frame_id = "simnav_map"
            message.header.stamp = rospy.Time.now()
            for point in coordinates:
                if not isinstance(point, (list, tuple)) or len(point) < 2:
                    continue
                polygon.append((float(point[0]), float(point[1])))
                message.polygon.points.append(
                    Point32(x=float(point[0]), y=float(point[1]))
                )
            if len(message.polygon.points) >= 3:
                with self.lock:
                    if polygon not in self.defer_polygons:
                        self.defer_polygons.append(polygon)
                self.defer_pub.publish(message)

    def _close_controlled_doors_if_requested(self):
        door_ids = []
        if rospy.get_param("~close_main_entrance", False):
            door_ids.append(rospy.get_param("~entrance_door_id", "main_entrance"))
        if rospy.get_param("~close_floor_elevator", False):
            door_ids.append(rospy.get_param("~elevator_door_id", "elevator_floor_0"))
        if not door_ids:
            return
        service_name = rospy.get_param("~door_service", "/set_door_state")
        try:
            rospy.wait_for_service(service_name, timeout=3.0)
            service = rospy.ServiceProxy(service_name, SetDoorState)
            for door_id in door_ids:
                response = service(door_id=door_id, open=False)
                rospy.loginfo(
                    "Door close request %s: accepted=%s state=%s",
                    door_id,
                    response.accepted,
                    response.state,
                )
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn("Could not close controlled doors through %s: %s", service_name, error)

    def _stop(self):
        self._publish_command(Twist())

    def _publish_command(self, command):
        self.last_command = (float(command.linear.x), float(command.angular.z))
        self.localization_monitor.set_command(command.linear.x, command.angular.z)
        self.command_pub.publish(command)

    def _shutdown(self):
        self.detect_timer.shutdown()
        self.control_timer.shutdown()
        self._stop()


if __name__ == "__main__":
    rospy.init_node("single_floor_explorer")
    SingleFloorExplorer()
    rospy.spin()
