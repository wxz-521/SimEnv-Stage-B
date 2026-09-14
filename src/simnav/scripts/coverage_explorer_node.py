#!/usr/bin/env python3
"""Direct corridor-and-room exploration using laser/camera joint coverage."""

import json
from dataclasses import replace
import faulthandler
import math
import os
import sys
import threading
import time
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
    leg_speed,
    FrontierTarget,
    GridView,
    RoomPortal,
    TaskCoveragePlanner,
    clamp_linear_speed,
    coverage_classification,
    detect_lobby_portals,
    detect_room_portals,
    detect_sphere_like_clusters,
    door_crossing_along_offsets,
    doorways_match,
    infer_task_extent,
    interior_targets_only,
    measure_corridor_walls,
    normalize_angle,
    opposite_room_portal,
    pair_room_portals,
    region_of_point,
    region_status,
    room_lock_for_target,
    target_kind_allowed_for_topology_state,
    target_switch_allowed,
    task_region_mask,
    topology_completion_ready,
    topology_id_for_point,
    topology_state_for_new_target,
    unsafe_path_replan_needed,
)


class CoverageExplorer:
    def __init__(self):
        self.lock = threading.RLock()
        self.grid = None
        self.raw_grid = None
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
        # On an upper floor the elevator node has already published the gate by
        # the time this node starts, so ``gate_source`` is not None and the
        # entrance-transit branch used to be skipped entirely: the robot began
        # exploring from the lift mouth instead of sweeping the front doorways
        # first (run112 floor 1: 12 stuck target drops, every room ~0.05, and no
        # room-interior candidate was dispatchable).  This flag keeps the
        # transit running until the configured distance is actually covered.
        self.initial_forward_complete = False
        # Far-zone committed transit (the "fixed forward"): get onto the corridor
        # centreline, align with the corridor axis, then run one measured
        # distance to the next unfinished doorway.  Same shape as the lift
        # return leg in the elevator node.
        self.committed_transit = None
        self.committed_center_tolerance = max(
            0.05, float(rospy.get_param("~committed_center_tolerance", 0.20))
        )
        self.committed_arrival_tolerance = max(
            0.05, float(rospy.get_param("~committed_arrival_tolerance", 0.30))
        )
        self.committed_speed = max(
            0.10, float(rospy.get_param("~committed_speed", 0.45))
        )
        # The robot has just stood up and switched to the RL gait when the
        # transit starts, so the commanded speed ramps in instead of hitting the
        # target on the first tick.  run70 tipped over (imu_roll -> pi) at
        # sim 11 s while being commanded 0.90 m/s from a standstill, within the
        # fall detector's startup grace, so the run continued on its side.
        self.initial_forward_start_speed = max(
            0.05, float(rospy.get_param("~initial_forward_start_speed", 0.25))
        )
        self.initial_forward_ramp_seconds = max(
            0.0, float(rospy.get_param("~initial_forward_ramp_seconds", 4.0))
        )
        self.initial_forward_settle_seconds = max(
            0.0, float(rospy.get_param("~initial_forward_settle_seconds", 2.0))
        )
        self.forward_started_stamp = None
        self.initial_forward_speed = max(
            0.05, float(rospy.get_param("~initial_forward_speed", 0.60))
        )
        self.initial_centering_start_distance = max(
            0.0, float(rospy.get_param("~initial_centering_start_distance", 10.5))
        )
        self.elevator_detection_start_distance = max(
            0.0, float(rospy.get_param("~elevator_detection_start_distance", 4.0))
        )
        self.elevator_detection_end_distance = max(
            self.elevator_detection_start_distance,
            float(
                rospy.get_param(
                    "~elevator_detection_end_distance",
                    self.initial_centering_start_distance,
                )
            ),
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
        # Re-anchor the virtual gate onto the observed corridor midline so a
        # lateral bias at the entrance does not become the axis for every
        # doorway decision in the run.
        self.recenter_gate_on_walls = bool(
            rospy.get_param("~recenter_gate_on_walls", True)
        )
        self.maximum_gate_recenter = max(
            0.0, float(rospy.get_param("~maximum_gate_recenter", 0.80))
        )
        self.forward_anchor = None
        self.forward_yaw = None
        self.transit_started_wall = None
        self.transit_fallbacks = 0
        self.transit_timeout_seconds = max(
            15.0, float(rospy.get_param("~transit_timeout_seconds", 60.0))
        )
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
        # The floor-completion latch is also driven from the control loop, so it
        # cannot be blocked by a paused planning callback; throttled here.
        self.last_completion_check = None
        # Diagnostic: plan cycles that could not run because an input was missing.
        self.plan_input_gaps = 0
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
            # Gain and coverage are deliberately different objectives: coverage
            # (camera_weight) decides when a room is finished, the gain decides
            # which viewpoint is worth driving to.  A lower camera share in the
            # gain lets laser/map knowledge steer the camera to places it can
            # actually see, while the target kind stays CAMERA_FRONTIER.
            gain_camera_weight=float(
                rospy.get_param("~gain_camera_weight", 0.50)
            ),
            target_rank_mode=str(
                rospy.get_param("~target_rank_mode", "gain_efficiency")
            ),
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
                rospy.get_param("~front_station_search_limit", 35.0)
            ),
            virtual_gate_half_width=float(
                rospy.get_param("~virtual_gate_half_width", 1.1)
            ),
            virtual_gate_depth=float(rospy.get_param("~virtual_gate_depth", 0.30)),
            min_room_interior_cells=int(
                rospy.get_param("~min_room_interior_cells", 800)
            ),
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
        # Diagnostic only: a plan cycle that never returns freezes the explorer
        # (run146: an unguarded A* predecessor walk burned a core for minutes
        # while the robot sat still on a stale plan target).  Crossing this means
        # "stuck", not "busy"; faulthandler then dumps every thread's stack into
        # this node's ROS log.  0 disables it.
        self.plan_watchdog_seconds = max(
            0.0, float(rospy.get_param("~plan_watchdog_seconds", 30.0))
        )
        self.target_tolerance = max(0.15, float(rospy.get_param("~target_tolerance", 0.35)))
        # Pure-pursuit look-ahead: steer at the point this far AHEAD along the
        # path instead of at the next 0.1 m cell.  Steering at the next cell made
        # a 0.1 m lateral error (at the 0.35 m arrival tolerance) look like a
        # 0.28 rad bearing error, i.e. an in-place turn, and the re-anchored path
        # flipped it back - the endless circling.  0 restores the old behaviour.
        self.heading_tolerance = max(0.08, float(rospy.get_param("~heading_tolerance", 0.25)))
        self.motion_stop_distance = max(0.30, float(rospy.get_param("~motion_stop_distance", 0.48)))
        self.motion_speed = max(0.30, float(rospy.get_param("~motion_speed", 0.60)))
        # Hard ceiling for every linear command in this node (mission decision:
        # 0.60 m/s everywhere until three floors are stable).
        self.max_linear_speed = max(
            0.10, float(rospy.get_param("~max_linear_speed", 0.60))
        )
        # Lobby/entrance transit ceiling: the doorway is ~1.1 m wide, so full
        # mission speed is too fast to correct a lateral drift before the jamb.
        self.transit_speed_cap = max(
            0.10, float(rospy.get_param("~transit_speed_cap", 0.35))
        )
        # How far in front of the entrance the reduced transit cap applies.
        self.transit_jamb_zone = max(
            0.5, float(rospy.get_param("~transit_jamb_zone", 2.0))
        )
        self.turn_speed = max(0.15, float(rospy.get_param("~turn_speed", 0.65)))
        # Path-leg speed shaping (user rule): do not hold the maximum speed the
        # whole time - ramp in from a stop and out near the goal, with a floor so
        # the ramp never crawls and wastes time.  Same shape as the entry
        # transit's ramp, applied to ordinary target driving.
        self.leg_accel_seconds = max(
            0.0, float(rospy.get_param("~leg_accel_seconds", 1.5))
        )
        self.leg_decel_distance = max(
            0.0, float(rospy.get_param("~leg_decel_distance", 1.0))
        )
        self.leg_ramp_floor_fraction = min(
            1.0, max(0.0, float(rospy.get_param("~leg_ramp_floor_fraction", 0.45)))
        )
        self.leg_minimum_speed = max(
            0.0, float(rospy.get_param("~leg_minimum_speed", 0.20))
        )
        # Ramp clock: None while turning/stopped, set when a walk resumes.
        self.walk_ramp_started = None
        self.review_hold_duration = max(0.5, float(rospy.get_param("~sphere_review_hold", 2.0)))
        # Bounded alignment.  Both the waypoint bearing and a camera frontier's
        # look_at can flip while the robot is turning towards them (the unseen
        # centroid moves as cells are observed, and the planner may replace the
        # target).  The control law is "rotate on the spot while |error| is
        # above tolerance", so a bearing that flips faster than the robot can
        # turn pins it in place for ever: run33 sat just inside ROOM_L_15 with
        # cmd_vx == 0 for 8.4 sim seconds while cmd_wz alternated +0.65/-0.65,
        # never making progress.  After this long without the error coming down,
        # trade perfect alignment for motion.
        self.max_align_seconds = max(
            2.0, float(rospy.get_param("~max_align_seconds", 8.0))
        )
        self.align_progress_error = max(
            0.05, float(rospy.get_param("~align_progress_error", 0.30))
        )
        # In-node liveness backstop.  A target can be held while the control
        # loop keeps taking a stop path and never issues a drive command:
        # run41 stood on the corridor centreline with an active ROOM_R_15
        # camera frontier for 246 s, publishing cmd_vel == (0, 0) in 674 of
        # 681 telemetry samples while the planner still reported reason=TARGET.
        # An external watchdog can only kill such a run; dropping the stuck
        # target lets the planner choose something else and the mission go on.
        # Legitimate holds are shorter (review hold 2.0 s, camera hold 0.6 s).
        self.stuck_target_seconds = max(
            2.0, float(rospy.get_param("~stuck_target_seconds", 5.0))
        )
        # A dropped or collision-blocked target must stay out of the pool long
        # enough that the planner is forced to choose a different action.  A
        # 5 s cooldown let run65 re-dispatch the identical 19.5 m corridor
        # frontier every ~4 minutes: dispatch -> controller stops (front
        # blocked) -> backstop drops -> same target again.
        self.stuck_target_cooldown = max(
            10.0, float(rospy.get_param("~stuck_target_cooldown_seconds", 120.0))
        )
        self.collision_block_seconds = max(
            6.0, float(rospy.get_param("~collision_block_seconds", 45.0))
        )
        self.stuck_target_drops = 0
        self.control_faults = 0
        self.plan_faults = 0
        self.empty_path_cycles = 0
        # Last-centimetre creep so a frontier just ahead of the robot is
        # actually driven to before it counts as reached.
        self.creep_stop_distance = max(
            0.10, float(rospy.get_param("~creep_stop_distance", 0.20))
        )
        self.creep_speed = max(
            0.10, float(rospy.get_param("~creep_speed", 0.25))
        )
        # Debounce a path judged unsafe so one scan dropout cannot churn the
        # target, and cool the abandoned target so the planner cannot hand it
        # straight back (run47 doorway rotation, two opposite targets every
        # ~5.5 s).
        self.unsafe_path_cycles = 0
        # Route refreshes are keyed by goal position: refreshing a route is a
        # routing fix, and only a bounded number of failures means the target is
        # genuinely bad.
        self.route_refresh_counts = {}
        self.route_refreshes = 0
        self.max_route_refreshes = max(
            1, int(rospy.get_param("~max_route_refreshes", 3))
        )
        self.unsafe_path_replans = 0
        self.unsafe_path_replan_cycles = max(
            1, int(rospy.get_param("~unsafe_path_replan_cycles", 3))
        )
        self.unsafe_path_block_seconds = max(
            0.0, float(rospy.get_param("~unsafe_path_block_seconds", 45.0))
        )
        self.last_motion_command_stamp = rospy.Time.now()
        # The liveness backstop must watch real displacement, not commands.  In
        # run44 the A/B flip-flop at the ROOM_R_15 approach made the robot emit
        # brief non-zero rotation commands every few seconds without ever
        # moving: the command stamp kept resetting, so a command-based timer
        # never fired (stuck_target_drops == 0 while the robot sat still for
        # 245 s until an external watchdog killed the run).
        self.last_progress_position = None
        self.last_progress_yaw = 0.0
        self.last_progress_stamp = rospy.Time.now()
        self.align_since = None
        self.align_best_error = math.pi
        self.camera_resolution = max(0.05, float(rospy.get_param("~camera_resolution", 0.25)))

        self.camera_points_world = set()
        self.camera_points_order = deque()
        self.camera_points_memory_limit = max(
            1000, int(rospy.get_param("~camera_points_memory_limit", 120000))
        )
        self.camera_update = rospy.Time(0)
        self.current_plan = None
        # How long a target must be held before a fresher one may replace it,
        # and the gain margin the fresher one must clear (see
        # target_switch_allowed).  The user requirement is that the target point
        # may change before it is reached; these bounds are what stops that from
        # turning into the old oscillation.
        self.target_switch_dwell = max(
            0.0, float(rospy.get_param("~target_switch_dwell", 8.0))
        )
        # Planning cost, for "why does the target change take so long" questions.
        # plan_cycles already existed; only the duration and the switch counter
        # are new.
        self.last_plan_ms = 0.0
        self.target_switches = 0
        self.active_target = None
        self.active_path = ()
        self.path_index = 0
        # T-E observable: distance between the robot's own pose and the first
        # point of the path it was just given.  A* starts from the nearest
        # traversable cell, so a non-zero value is T1 ("路径起点不在机器人脚下")
        # made measurable instead of inferred from a screenshot.  Pure
        # diagnostic: nothing reads it back.
        self.path_start_offset = None
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
        self.front_station_along = None
        self.front_station_topologies = set()
        # Doorway identity tolerance for matching a completed room to a front
        # station whose transient id has drifted (run46 floor 2).
        self.front_station_match_tolerance = max(
            0.1, float(rospy.get_param("~front_station_match_tolerance", 2.5))
        )
        # A live doorway this close to a seeded/remembered one is the same
        # physical opening: the remembered id keeps the bookkeeping and the live
        # geometry replaces the seeded estimate.
        self.doorway_merge_tolerance = max(
            0.2, float(rospy.get_param("~doorway_merge_tolerance", 1.5))
        )
        self.completed_front_sides = set()
        self.room_scope_announced = None
        self.portal_evidence = {}
        self.portal_last_seen = {}
        # Preserve the exact confirmed doorway used to enter each topology.
        # A sparse scan near/inside the room must not erase the only safe exit.
        self.topology_portals = {}
        # Doorway ids inherited from the reference floor.  They are already
        # confirmed map structure and must not be re-derived on this floor.
        self.reused_portals = set()
        self.portal_confirm_cycles = max(
            1, int(rospy.get_param("~portal_confirm_cycles", 1))
        )
        self.active_target_started = rospy.Time(0)
        self.active_target_last_progress = None
        self.active_target_last_progress_stamp = rospy.Time(0)
        self.active_target_last_distance = None
        self.room_completion_miss_cycles = max(
            2, int(rospy.get_param("~room_completion_miss_cycles", 3))
        )
        # When a room approach fails before the doorway is crossed the topology
        # is parked as BLOCKED.  If that leaves the planner with no target at
        # all, the mission used to idle until the run timeout (observed on
        # floor 0 of the 0.84 three-floor run: ROOM_L_43 BLOCKED, corridor fully
        # known, NO_FRONTIER, cmd_vel 0 for the rest of the run).  Re-open the
        # least-covered blocked room a bounded number of times so the doorway
        # gets another attempt instead of ending the mission.
        self.max_room_retries = max(
            1, int(rospy.get_param("~max_room_retries", 3))
        )
        self.room_retry_counts = {}
        # Bounded wait for an opposing room that currently has no executable
        # target.  Large enough to ride out a sparse map, small enough that a
        # physically unreachable partner cannot hang the floor forever.
        self.station_partner_wait_cycles = max(
            5, int(rospy.get_param("~station_partner_wait_cycles", 20))
        )
        # Bounded wait for a room whose frontiers are exhausted while its
        # camera coverage is still low.  0 keeps the validated pre-change
        # behaviour (hold the lock); a positive value releases the room after
        # that many planner cycles.  The experiment defaulted to 20 and is not
        # part of the validated baseline.
        self.room_frontier_wait_cycles = int(
            rospy.get_param("~room_frontier_wait_cycles", 0)
        )
        # A room is released only when its coverage target is met *and* it has no
        # executable candidate left.  Reaching the target alone used to hand the
        # lock back while a large part of the room was still unseen (observed in
        # RViz: a big camera-unseen area left behind in the left room), which
        # also made the robot look like it entered and immediately left.  The
        # grace bounds the extra work so a candidate that can never be cleared
        # cannot hold the floor: after this many sim seconds past the target the
        # room is released regardless.
        self.room_exhaust_grace_seconds = max(
            0.0, float(rospy.get_param("~room_exhaust_grace_seconds", 60.0))
        )
        # A remaining candidate only keeps the room alive while it is worth the
        # travel, measured as the local information gain it would reveal (m^2).
        # Once the room is past its target this is what stops the robot
        # sweeping marginal wall pockets: measured on run35, ROOM_R_15 was at
        # 0.966 coverage and still chasing far-wall viewpoints.
        self.room_exhaust_min_gain = max(
            0.0, float(rospy.get_param("~room_exhaust_min_gain", 1.5))
        )
        # Whether a met target still has to wait for its candidates to run out.
        # ``False`` (default) releases the room the moment the target is met,
        # which is what makes a lowered target actually reduce exploration time.
        # ``True`` keeps sweeping while a candidate worth more than
        # ``room_exhaust_min_gain`` remains; that was added to stop the robot
        # abandoning a large unseen area at a 0.84 target, but at 0.55 it simply
        # overrides the target: run35 kept working ROOM_R_15 up to 0.966 while
        # the target was 0.55.
        self.room_exit_requires_gain = bool(
            rospy.get_param("~room_exit_requires_gain", False)
        )
        # Doorway commitment band (metres of |lateral| about the corridor
        # centreline).  While the robot is inside it, swapping the active target
        # turns a doorway crossing into a shuttle: measured on run38 the robot
        # alternated between lateral 0.27 and 1.27 across the ROOM_L_15 door
        # line for ~35 sim seconds with cmd_vx == 0 most of the time and
        # cmd_wz flipping between +0.65 and -0.65, and never got into the room.
        # The doorway hold band that used to refuse target swaps here is gone:
        # with the corridor staging waypoint removed the crossing route stays
        # monotone, and the plan install no longer swaps targets at all (the
        # active target is only cleared when it is reached or has failed).
        # The corridor return used to be considered finished as soon as the
        # robot was within corridor_half_width - 0.10 of the centreline, which
        # the doorway itself satisfies.  Floors therefore ended inside a door
        # frame: run29 parked at a cell with 0.22 m clearance, where A* refuses
        # to plan (safe[start] is False below navigation_clearance) and the
        # elevator transition faulted with ROUTE_UNREACHABLE.  A floor may now
        # only latch once the robot is genuinely on the centreline and its own
        # cell is navigable on the same map the elevator plans on.
        self.floor_end_corridor_tolerance = max(
            0.15, float(rospy.get_param("~floor_end_corridor_tolerance", 0.35))
        )
        self.room_entry_counts = {}
        # T-E observable: every confirmed doorway crossing, in order, as
        # (sim, floor, room_id, nth-entry-on-that-floor).  Run-level on purpose:
        # ``room_entry_counts`` is reset per floor, so it cannot answer "did the
        # robot leave a room and come back later in the run".  Pure diagnostic.
        self.room_entry_sequence = []
        self.lock_release_counts = {}
        # Interior-target discipline: after entering a room the target may not be
        # moved back to the corridor until the interior has failed this many
        # times (user policy: free re-orientation inside, bounded escape out).
        self.room_interior_retries = {}
        self.interior_retry_limit = max(
            1, int(rospy.get_param("~interior_retry_limit", 3))
        )
        self.max_lock_releases = max(
            1, int(rospy.get_param("~max_lock_releases", 3))
        )
        self._end_pose_clearance_cache = None
        # Topologies this run has already finished or parked.  A retired room is
        # never entered again, which makes "enter the door, come back out, go
        # in again" structurally impossible instead of merely unlikely.
        self.retired_topologies = set()
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
        # Danger-seeking: a partially observed red cluster already earns a
        # review target instead of waiting for repeated hits.  Measured recall
        # on the three-floor runs was 0.0-0.67 with zero false alarms, so the
        # detection itself is precise and the misses come from never pointing
        # the camera at the source.  Acting on the first observation steers the
        # robot towards what it has already glimpsed.  Confirmation still needs
        # the full sphere_min_hits count, so this cannot create false alarms.
        self.danger_candidate_min_hits = max(
            1, int(rospy.get_param("~danger_candidate_min_hits", 1))
        )
        # Graded red-sphere guidance: 0 detection only, 1 fallback review,
        # 2 in-topology priority, 3 global priority + preemption + cross-lock.
        # Raised one level at a time with recall/time evidence.
        self.danger_guidance_level = max(
            0, min(3, int(rospy.get_param("~danger_guidance_level", 1)))
        )
        self.sphere_point_stride = max(1, int(rospy.get_param("~sphere_point_stride", 2)))
        self.sphere_process_period = max(
            0.20, float(rospy.get_param("~sphere_process_period", 0.50))
        )
        self.last_sphere_process = rospy.Time(0)
        self.sphere_stale_duration = max(
            2.0, float(rospy.get_param("~sphere_stale_duration", 8.0))
        )
        # Red-object-directed exploration.  The RGB-D danger detector publishes
        # its still-unconfirmed red-blob tracks; each one becomes a review
        # target so the fixed forward camera is deliberately pointed at a
        # partially observed red object instead of waiting for the coverage
        # sweep to stumble across it.  These hints are never written to
        # detected_danger.json and never gate floor completion: only the
        # detector's own shape/colour confirmation can do that.
        self.detector_candidates = {}
        self.detector_candidate_timeout = max(
            2.0, float(rospy.get_param("~detector_candidate_timeout", 20.0))
        )
        self.detector_reviewed = set()
        # Red-ball guidance pays for itself by letting a room finish early once
        # the danger source inside it is already confirmed.  0.0 disables this
        # (validated baseline): a room then always needs the full 0.84 combined
        # coverage.  Lowering it step by step trades area coverage for time only
        # in rooms whose red sphere has actually been confirmed, so recall is
        # not given up -- raise/lower one step at a time with recall evidence.
        self.danger_early_exit_coverage = max(
            0.0, float(rospy.get_param("~danger_early_exit_coverage", 0.0))
        )
        self.confirmed_danger_positions = []

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

        rospy.Subscriber("/map", OccupancyGrid, self._raw_map_callback, queue_size=1)
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
        rospy.Subscriber(
            "/simnav/danger_candidates", String,
            self._danger_candidate_callback, queue_size=2,
        )
        rospy.Subscriber(
            "/simnav/danger_tracks", String,
            self._danger_tracks_callback, queue_size=2,
        )
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

    def _raw_map_callback(self, message):
        with self.lock:
            self.raw_grid = GridView(
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

    def _reused_floor_portals(self, payload):
        """Rebuild the reference floor's doorway geometry for an upper floor.

        The generated floors share one x/y topology, so an upper floor must
        reuse the ground floor's resolved doorways instead of rediscovering
        them.  Rediscovery re-bins the same physical door under a new id when
        SLAM refines the wall, which splits one room into two topologies.
        """
        entries = payload.get("reused_topology")
        if not isinstance(entries, list):
            return ()
        portals = []
        for item in entries:
            if not isinstance(item, dict):
                continue
            topology_id = str(item.get("topology_id") or "")
            side = str(item.get("side") or "")
            if not topology_id or side not in ("L", "R"):
                continue
            try:
                portals.append(
                    RoomPortal(
                        topology_id=topology_id,
                        side=side,
                        along=float(item["along"]),
                        lateral=float(item.get("lateral", 0.0)),
                        width=float(item.get("width", 0.0)),
                    )
                )
            except (KeyError, TypeError, ValueError):
                continue
        return tuple(portals)

    def _floor_context_callback(self, message):
        """Switch to a new floor, reusing the reference topology verbatim."""
        try:
            payload = json.loads(message.data)
            floor_index = int(payload["floor_index"])
            gate = tuple(float(value) for value in payload["gate_source"][:3])
            gate_world = tuple(float(value) for value in payload["gate_world"][:3])
        except (TypeError, ValueError, KeyError, json.JSONDecodeError):
            return
        if floor_index <= 0:
            return
        # Upper floors are isomorphic to the ground floor, so seed the reference
        # floor's doorways here.  Reuse was deleted once because the seeded
        # geometry was trusted verbatim: on run80's top floor those portals kept
        # evidence=1 while the live map detected the real openings with evidence
        # 61-72, and every crossing route aimed at the stale geometry failed
        # (CANDIDATE_PATH_UNREACHABLE).  Neither extreme works - without seeding
        # an upper floor produces only corridor targets and wanders (run126
        # floor 1: "reused 0 topology portals", no ROOM target for 110+ s).  So
        # seed AND refresh: the seeded ids are confirmed immediately (room
        # targets exist from the first cycle) and the live map overwrites their
        # geometry through ``doorways_match`` in ``_plan_impl``.
        reused_portals = self._reused_floor_portals(payload)
        with self.lock:
            if self.floor_index == floor_index and not self.floor_complete:
                return
            self.floor_index = floor_index
            # The doorways carry the reference floor's own ids.  Detection on
            # this floor must use the same prefix, otherwise the reused
            # geometry is treated as unconfirmed and rediscovered alongside it.
            self.floor_prefix = "ROOM"
            # Reused topology is already confirmed map structure, so the live
            # lidar does not have to prove the walls from scratch again.
            self.floor_laser_isolated = not reused_portals
            self.floor_complete = False
            self.initial_forward_complete = False
            self.committed_transit = None
            self.gate_source = gate
            self.gate_world = gate_world
            self.topology_region = "CORRIDOR"
            self.completed_topologies.clear()
            self.topology_states.clear()
            self.topology_miss_cycles.clear()
            # A blocked room is retried a bounded number of times per floor.
            self.room_retry_counts.clear()
            self.topology_portals.clear()
            self.portal_evidence.clear()
            self.portal_last_seen.clear()
            self.reused_portals = set()
            for portal in reused_portals:
                self.topology_portals[portal.topology_id] = portal
                # Structure inherited from the reference floor is confirmed by
                # construction; only this floor's exploration is outstanding.
                self.portal_evidence[portal.topology_id] = self.portal_confirm_cycles
                self.topology_states[portal.topology_id] = {
                    "state": "APPROACHING",
                    "targets": 0,
                }
                self.reused_portals.add(portal.topology_id)
            self.topology_lock = None
            self.returning_topology = None
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
            # Entry bookkeeping is per floor: upper floors reuse the same 2-D
            # frame and the same doorway ids, so keeping the ground floor's
            # counts would make every upper room look like a re-entry.
            self.room_entry_counts = {}
            self.lock_release_counts = {}
            self.room_interior_retries = {}
            self.retired_topologies = set()
        self._publish_gate()
        self.complete_pub.publish(Bool(data=False))
        self._publish_status()
        rospy.loginfo(
            "Floor context switched to floor %d; prefix=%s; reused %d topology portals",
            floor_index,
            self.floor_prefix,
            len(reused_portals),
        )

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

    def _danger_candidate_callback(self, message):
        """Accept the detector's unconfirmed red tracks as review hints."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        now = rospy.Time.now().to_sec()
        with self.lock:
            for item in payload.get("candidates", []):
                position = item.get("position") or []
                if len(position) < 2:
                    continue
                candidate_id = "DET_{}".format(item.get("id"))
                self.detector_candidates[candidate_id] = {
                    "id": candidate_id,
                    "center": (float(position[0]), float(position[1])),
                    "hits": self.danger_candidate_min_hits,
                    "last_seen": now,
                    "observations": int(item.get("observations", 1)),
                }
            stale = [
                candidate_id
                for candidate_id, item in self.detector_candidates.items()
                if now - item["last_seen"] > self.detector_candidate_timeout
            ]
            for candidate_id in stale:
                del self.detector_candidates[candidate_id]

    def _detector_candidates_locked(self):
        if self.danger_guidance_level <= 0:
            return []
        now = rospy.Time.now().to_sec()
        return [
            {
                "id": item["id"],
                "center": item["center"],
                "hits": self.danger_candidate_min_hits,
                "last_seen": item["last_seen"],
            }
            for item in self.detector_candidates.values()
            if item["id"] not in self.detector_reviewed
            and now - item["last_seen"] <= self.detector_candidate_timeout
        ]

    def _update_detector_reviewed(self, grid, camera_seen):
        """Retire a hint once the camera has actually covered its location."""
        radius_cells = max(1, int(math.ceil(0.40 / grid.resolution)))
        with self.lock:
            candidates = list(self.detector_candidates.values())
        for item in candidates:
            if item["id"] in self.detector_reviewed:
                continue
            row, column = grid.world_to_cell(*item["center"][:2])
            row_start, row_stop = max(0, row - radius_cells), min(camera_seen.shape[0], row + radius_cells + 1)
            column_start, column_stop = max(0, column - radius_cells), min(camera_seen.shape[1], column + radius_cells + 1)
            if row_start < row_stop and column_start < column_stop and np.any(
                camera_seen[row_start:row_stop, column_start:column_stop]
            ):
                self.detector_reviewed.add(item["id"])
                rospy.loginfo(
                    "Camera reviewed danger-detector candidate %s", item["id"]
                )

    def _danger_tracks_callback(self, message):
        """Track confirmed danger sources for the red-ball early-exit gate."""
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        positions = []
        for item in payload.get("dangers", []):
            position = item.get("position_world") or item.get("position") or []
            if len(position) >= 2:
                positions.append((float(position[0]), float(position[1])))
        with self.lock:
            self.confirmed_danger_positions = positions

    def _danger_confirmed_in_topology(self, plan, topology_id):
        """True when a confirmed red source is mapped to this room topology."""
        if self.danger_early_exit_coverage <= 0.0:
            return False
        positions = list(self.confirmed_danger_positions)
        portals = tuple(getattr(plan, "actionable_portals", ()) or ())
        if not positions or not portals or self.gate_source is None:
            return False
        for point in positions:
            owner = topology_id_for_point(
                point,
                self.gate_source[:2],
                self.gate_source[2],
                self.planner.corridor_half_width,
                portals,
            )
            if str(owner) == str(topology_id):
                return True
        return False

    def _camera_exploration_ready_locked(self):
        # Camera observations accumulate as soon as the task corridor is
        # established.  Corridor target selection remains lidar-only, but
        # this early observation avoids rescanning the doorway view after a
        # room crossing.
        return bool(self.gate_source is not None and not self.floor_complete)

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
        # Plan-cycle cost is reported in the status so "it looks stuck while it
        # computes" can be answered with a number instead of an inference: the
        # whole cycle benchmarks at tens of milliseconds (128 camera candidates
        # x 600-point paths = 23 ms on a 600x800 map), while the simulator runs
        # at RTF ~0.11, so one simulated second costs about nine wall seconds.
        started = time.time()
        watchdog = self.plan_watchdog_seconds
        if watchdog > 0.0:
            faulthandler.dump_traceback_later(watchdog, exit=False)
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
        finally:
            if watchdog > 0.0:
                faulthandler.cancel_dump_traceback_later()
            with self.lock:
                self.last_plan_ms = (time.time() - started) * 1000.0

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
        # Same guard as the control loop: a raising rospy.Timer callback kills
        # its thread for good.  run57 stopped logging and commanding at
        # sim 219 with an active target and cmd_vel exactly (0, 0) for the rest
        # of the run, which is the signature of a dead planning thread.
        try:
            self._plan_impl(_event)
        except Exception:  # noqa: BLE001 - a plan fault must not be fatal
            self.plan_faults += 1
            rospy.logerr(
                "PLAN_FAULT #%d (planning loop kept alive):\n%s",
                int(self.plan_faults),
                traceback.format_exc(),
            )

    def _plan_impl(self, _event):
        with self.lock:
            if self.floor_complete:
                return
            if (
                self.topology_lock is not None
                and self.topology_states.get(self.topology_lock, {}).get("state")
                == "RETURNING"
                and self.active_target is not None
                and self.active_target.kind == "RETURN_TO_CORRIDOR"
            ):
                # The control loop owns an already validated return route and
                # releases the topology lock as soon as the corridor boundary
                # is crossed.  Rebuilding all room camera frontiers here can
                # take longer than the complete return manoeuvre.
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
            # Partial observations are enough to dispatch a review target; the
            # confirmation gate in _check_completion still uses the stable set.
            spheres = [
                dict(item)
                for item in self.sphere_hypotheses.values()
                if int(item.get("hits", 0)) >= self.danger_candidate_min_hits
            ]
            # Red-object-directed hints from the RGB-D detector take part in
            # the same review path but stay out of the floor-completion gate.
            spheres = spheres + self._detector_candidates_locked()
            reviewed = set(self.reviewed_hypotheses)
            topology_lock = self.topology_lock
            completed_topologies = tuple(self.completed_topologies)
            confirmed_topologies = tuple(
                topology_id
                for topology_id, count in self.portal_evidence.items()
                if int(count) >= self.portal_confirm_cycles
            )
        if grid is None or pose is None or world_pose is None:
            # This return used to be completely silent: no log line and no status
            # publish, so the whole node looked hung while it was simply missing
            # an input.  run152 spent 74.3 s + 61.3 s of standstill here (31.9 %
            # of the run was zero-command time, and the floor-completion latch
            # with it), and the only way to see it afterwards was that
            # ``plan_cycles`` and ``last_plan_ms`` (0.05 ms) stopped advancing.
            # Name the missing input instead, and keep the status topic alive.
            self.plan_input_gaps += 1
            rospy.logwarn_throttle(
                5.0,
                "Planning paused (%d): missing input grid=%s pose=%s "
                "world_pose=%s; the robot stands until it returns",
                int(self.plan_input_gaps),
                grid is not None,
                pose is not None,
                world_pose is not None,
            )
            self._publish_status()
            return
        if gate_source is None:
            if self._adopt_provisional_gate():
                self._publish_status()
                return
            self._publish_status()
            return
        camera_seen = self._camera_seen_grid(grid, pose, world_pose, points)
        with self.lock:
            self._update_reviewed(grid, camera_seen)
            self._update_detector_reviewed(grid, camera_seen)
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
            front_station_along_hint=self.front_station_along,
            completed_front_sides=tuple(self.completed_front_sides),
            portal_prefix=self.floor_prefix,
            force_laser_unknown=self.floor_laser_isolated,
            portal_grid=self.raw_grid,
            danger_guidance_level=self.danger_guidance_level,
            interior_only=self._interior_targets_only_locked(),
        )
        with self.lock:
            seen_portal_ids = set()
            for portal in plan.observed_portals:
                remembered = None
                if str(portal.topology_id) not in self.topology_portals:
                    remembered = next(
                        (
                            item
                            for item in self.topology_portals.values()
                            if doorways_match(
                                item, portal, self.doorway_merge_tolerance
                            )
                        ),
                        None,
                    )
                if remembered is not None:
                    # Live geometry wins; the remembered id keeps the room state,
                    # coverage denominator and entry bookkeeping.  This is the
                    # "seed + refresh" rule: a seeded upper-floor doorway is
                    # corrected by this floor's own map instead of being trusted
                    # verbatim (run80) or rediscovered under a new bin id
                    # (run126 floor 1: no room target at all).
                    key = str(remembered.topology_id)
                    self.topology_portals[key] = replace(portal, topology_id=key)
                else:
                    key = str(portal.topology_id)
                seen_portal_ids.add(key)
                self.portal_evidence[key] = int(
                    self.portal_evidence.get(key, 0)
                ) + 1
                self.portal_last_seen[key] = rospy.Time.now().to_sec()
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
                    despeckle=True,
                )
            )
            if target_active and not active_safe:
                self.unsafe_path_cycles += 1
            else:
                self.unsafe_path_cycles = 0
            unsafe_replan = unsafe_path_replan_needed(
                target_active,
                active_safe,
                self.unsafe_path_cycles,
                self.unsafe_path_replan_cycles,
            )
            sphere_preemption = bool(
                self.danger_guidance_level >= 3
                and plan.target is not None
                and plan.target.kind == "SPHERE_REVIEW"
                and (self.active_target is None or self.active_target.kind != "SPHERE_REVIEW")
                and (
                    self.active_target is None
                    or self.active_target.kind != "RETURN_TO_CORRIDOR"
                )
            )
            lifecycle_changed = self._update_topology_lifecycle(plan, target_active)
            self._check_completion()
            if self.floor_complete:
                self._publish_status()
                return
            if (
                plan.target is None
                and not target_active
                and not lifecycle_changed
                and self._retry_blocked_room(plan)
            ):
                # The re-opened lock takes effect on the next planner cycle.
                self._publish_path()
                self._publish_markers()
                self._publish_coverage_layers()
                self._publish_status()
                return
            if (
                plan.target is None
                and not target_active
                and not lifecycle_changed
                and self.topology_lock is not None
                and not self.floor_complete
            ):
                if self._recover_idle_lock(plan):
                    self._publish_path()
                    self._publish_markers()
                    self._publish_coverage_layers()
                    self._publish_status()
                    return
            if lifecycle_changed and plan.target is None:
                # Lifecycle bookkeeping alone must not suppress a dispatchable
                # target.  Returning unconditionally here left the robot with
                # plan_reason=TARGET but active_target=None for tens of seconds
                # (run138 floor 0: CAMERA_FRONTIER/ROOM_R_15 produced every
                # cycle, never adopted, control stopped, target dropped by the
                # backstop, repeat).
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
            if (
                (active_safe or (target_active and not unsafe_replan))
                and not sphere_preemption
            ):
                if self._switch_active_target(plan):
                    self._publish_path()
                    self._publish_markers()
                    self._publish_coverage_layers()
                    self._publish_status()
                    return
                self._refresh_active_heading(plan)
                self._publish_markers()
                self._publish_coverage_layers()
                self._publish_status()
                return
            if unsafe_replan and self.active_target is not None:
                # An unsafe path is a ROUTING problem, not a bad target.  Refresh
                # the route to the SAME target instead of switching away from it:
                # switching (with a 45 s cool-down on the abandoned target) made
                # the explorer alternate between a corridor and a room target for
                # a full hour - 15:07 to 16:10 in the run73 logs - because every
                # new target reset the debounce counter.
                goal_key = (
                    round(float(self.active_target.target[0]), 2),
                    round(float(self.active_target.target[1]), 2),
                )
                refreshes = int(self.route_refresh_counts.get(goal_key, 0))
                if refreshes < self.max_route_refreshes:
                    self.route_refresh_counts[goal_key] = refreshes + 1
                    self.route_refreshes += 1
                    with self.lock:
                        self.active_path = ()
                        self.path_index = 0
                        self.last_plan_time = rospy.Time(0)
                    self.unsafe_path_cycles = 0
                    rospy.logwarn(
                        "Unsafe path for %s/%s after %d cycles; refreshing the "
                        "route to the same target (%d/%d, route_refreshes=%d)",
                        self.active_target.kind,
                        self.active_target.topology_id,
                        int(self.unsafe_path_cycles),
                        refreshes + 1,
                        int(self.max_route_refreshes),
                        int(self.route_refreshes),
                    )
                    self._publish_path()
                    self._publish_markers()
                    self._publish_coverage_layers()
                    self._publish_status()
                    return
                # Bounded: only after repeated failed refreshes is the target
                # itself treated as bad (the pre-existing drop-and-cool path).
                with self.lock:
                    self.blocked_targets.append(
                        (
                            float(self.active_target.target[0]),
                            float(self.active_target.target[1]),
                            rospy.Time.now().to_sec() + self.unsafe_path_block_seconds,
                        )
                    )
                self.unsafe_path_replans += 1
                rospy.logwarn(
                    "Target %s/%s dropped after %d route refreshes "
                    "(unsafe_path_replans=%d, blocked for %.0fs)",
                    self.active_target.kind,
                    self.active_target.topology_id,
                    refreshes,
                    int(self.unsafe_path_replans),
                    self.unsafe_path_block_seconds,
                )
            self.unsafe_path_cycles = 0
            self.active_target = None
            self.active_path = ()
            self.path_index = 0
            if plan.target is not None:
                self.active_target = plan.target
                self.active_path = self._anchor_path_to_current_pose(
                    plan.target.path or (plan.target.target,), plan.target
                )
                self.path_index = (
                    min(1, len(self.active_path) - 1) if self.active_path else 0
                )
                self._note_path_start()
                self.review_hold_since = None
                # A fresh target gets a fresh alignment budget (fields are
                # owned by the control timer, hence set directly here).
                self.align_since = None
                self.align_best_error = math.pi
                # Keep-fix: stamp the adoption with the CURRENT time, not the
                # ``now`` captured before planner.plan() ran.  A plan cycle has
                # been measured at 6.3 s (and 137 s in the worst case), so a
                # backdated stamp made a brand-new target look as if it had
                # already produced no displacement for the whole planning
                # latency, and the liveness backstop dropped it 0.13 s after it
                # was chosen (run147: LASER_FRONTIER/ROOM_L_35 adopted at
                # 00:52:38,653, dropped at 00:52:38,785 "after 6.3s").
                adoption_stamp = rospy.Time.now()
                self.active_target_started = adoption_stamp
                self.active_target_last_progress = self.path_index
                self.active_target_last_progress_stamp = adoption_stamp
                self.active_target_last_distance = None
                if (
                    plan.target.topology_id != "CORRIDOR"
                    and "UNASSIGNED" not in plan.target.topology_id
                    and plan.target.topology_id not in self.retired_topologies
                ):
                    # Same helper the mid-route handover uses, so both adoption
                    # paths arm the room lock identically.
                    self._arm_room_approach(plan.target.topology_id)
                rospy.loginfo(
                    "COVERAGE target kind=%s topology=%s path=%.2f laser_gain=%.2f camera_gain=%.2f combined_gain=%.2f%s",
                    plan.target.kind,
                    plan.target.topology_id,
                    plan.target.path_length,
                    plan.target.laser_gain,
                    plan.target.camera_gain,
                    plan.target.combined_gain,
                    "",
                )
            self._publish_path()
            self._publish_markers()
            self._publish_coverage_layers()
            self._publish_status()

    def _align_stalled(self, error_magnitude):
        """True once an alignment has outlasted its budget without improving.

        A bearing that keeps flipping direction never satisfies the
        rotate-in-place test, so this is what stops it pinning the robot: the
        clock restarts every time a *new best* error is seen, so a monotonic
        approach is never cut short.  Deliberately lock-free; only the control
        timer touches these fields.
        """
        now = rospy.Time.now()
        if self.align_since is None or error_magnitude < self.align_best_error:
            self.align_since = now
            self.align_best_error = error_magnitude
        age = (now - self.align_since).to_sec()
        return (
            age >= self.max_align_seconds
            and error_magnitude > self.align_progress_error
        )

    def _clear_align_stall(self):
        self.align_since = None
        self.align_best_error = math.pi

    def _region_of_pose(self):
        """Region membership of the robot's own pose: corridor / room / unknown.

        One truth shared with the coverage denominator (``topology_region_mask``
        builds the same Voronoi bands).  Entry is "the pose is inside this room's
        region" and exit is "the pose is back in the corridor" - a state, so no
        replan can lose it, and no door identity, evidence count or wall-offset
        proxy is involved for rooms.  Returns ``None`` while the portal set is
        still unknown, in which case the caller keeps its geometric fallback.
        """
        if self.pose is None or self.gate_source is None:
            return None
        portals = list(self.topology_portals.values())
        if not portals:
            return None
        return region_of_point(
            self.pose,
            self.gate_source,
            self.gate_source[2],
            self.planner.corridor_half_width,
            portals,
        )

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
        # has scanned the room, and the drift can exceed the cached doorway
        # width: run51 finished ROOM_R_38's coverage and then sat in the room
        # because "return-to-corridor path is unavailable".  Search the cached
        # doorway first and widen only when that fails, exactly like the outward
        # crossing.  This remains a return through the confirmed door, not a
        # generic wall-gap fallback.
        for along_offset in door_crossing_along_offsets(
            portal.width,
            self.planner.door_crossing_wide_search,
            self.planner.door_crossing_scan_step,
        ):
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
        self._note_path_start()
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
            if self._finish_corridor_return_locked():
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
        if not entered:
            entered = self._mark_room_entered_locked()
        if entered:
            state["state"] = "EXPLORING"
            self.topology_region = "ROOM_EXPLORING"
        elif state.get("state") not in ("COMPLETE", "BLOCKED"):
            state["state"] = "APPROACHING"

        local = (plan.topology_coverages or {}).get(lock)
        # ``candidate_topologies`` describes the global pool before the room
        # lock is applied.  Use the filtered list here: a different room being
        # available must not keep an exhausted locked room alive forever.
        has_room_candidates = any(
            item.topology_id == lock for item in plan.targets
        )
        # A room whose red sphere is already confirmed has met the task
        # objective; from `danger_early_exit_coverage` upward it may finish
        # before the full area target.  0.0 keeps the validated 0.84-only gate.
        danger_exit = bool(
            state.get("state") == "EXPLORING"
            and local is not None
            and self.danger_early_exit_coverage > 0.0
            and local.combined >= self.danger_early_exit_coverage
            and self._danger_confirmed_in_topology(plan, lock)
        )
        coverage_ok = bool(
            state.get("state") == "EXPLORING"
            and local is not None
            and (
                local.combined >= self.room_combined_coverage_target
                or danger_exit
            )
        )
        # Only start the grace clock once the target is actually met, and reset
        # it if coverage drops back below (the map can still grow).
        now_seconds = rospy.Time.now().to_sec()
        if coverage_ok:
            if state.get("coverage_met_since") is None:
                state["coverage_met_since"] = now_seconds
        else:
            state["coverage_met_since"] = None
        met_for = (
            now_seconds - float(state.get("coverage_met_since") or now_seconds)
            if coverage_ok
            else 0.0
        )
        # Target met AND nothing worth doing left in this room.  "Worth" is a
        # gain threshold, not merely "a candidate exists": with the target met
        # at, say, 0.55 while the room is already at 0.96, the survivors are
        # marginal wall pockets whose local gain is a fraction of a square
        # metre, and chasing them looked exactly like exploring the whole floor
        # for its own sake.  A large unseen pocket still produces a large local
        # gain, so the earlier fix (never abandon a big unseen area) is kept.
        # The time grace stays as a backstop for a high-gain candidate that can
        # never be cleared.
        best_room_gain = max(
            (
                float(item.combined_gain)
                for item in plan.targets
                if item.topology_id == lock
            ),
            default=0.0,
        )
        significant_candidate = bool(
            self.room_exit_requires_gain
            and best_room_gain >= self.room_exhaust_min_gain
        )
        local_coverage_ok = bool(
            coverage_ok
            and (
                not significant_candidate
                or met_for >= self.room_exhaust_grace_seconds
            )
        )
        if coverage_ok and significant_candidate:
            rospy.loginfo_throttle(
                10.0,
                "Room %s at combined=%.3f (target %.3f) but a candidate of "
                "gain %.2f m2 (>= %.2f) is still worth visiting; staying "
                "(%.0fs/%.0fs)",
                lock,
                local.combined if local is not None else float("nan"),
                self.room_combined_coverage_target,
                best_room_gain,
                self.room_exhaust_min_gain,
                met_for,
                self.room_exhaust_grace_seconds,
            )
        if danger_exit and local.combined < self.room_combined_coverage_target:
            rospy.loginfo(
                "Room %s finishes early at combined=%.3f (threshold %.3f): "
                "red source already confirmed in this topology",
                lock, local.combined, self.danger_early_exit_coverage,
            )
        if local_coverage_ok:
            state["state"] = "RETURNING"
            self.topology_region = "ROOM_RETURNING"
            if self._start_corridor_return(plan, lock):
                rospy.loginfo(
                    "Topology room %s coverage complete at combined=%.3f "
                    "(target %.3f, task_cells=%d, best_remaining_gain=%.2f); "
                    "returning to CORRIDOR before next room",
                    lock,
                    local.combined,
                    self.room_combined_coverage_target,
                    int(local.task_cells),
                    best_room_gain,
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
        # (has_room_candidates is computed with the coverage check above.)
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
            if state.get("station_partner") is not None:
                # The first room of this station is already complete.  A
                # transient sparse map must not release the station and let a
                # distant corridor frontier bypass the opposing room.  The wait
                # must still be bounded: when the partner is physically
                # unreachable the planner produces no target forever and the
                # mission hangs with the floor unfinished.  Observed on the
                # second floor: the robot finished the rear-right room, sat
                # inside it, and waited on the rear-left room for over 100 s
                # with room_reachable_cells = 0.
                waited = int(state.get("partner_wait_cycles", 0)) + 1
                state["partner_wait_cycles"] = waited
                state["state"] = "APPROACHING"
                self.topology_miss_cycles[lock] = 0
                if waited < self.station_partner_wait_cycles:
                    rospy.logwarn_throttle(
                        5.0,
                        "Opposite room %s has no executable target yet; keeping station lock",
                        lock,
                    )
                    return False
                # Bounded wait expired: release the station so another portal or
                # corridor frontier can be tried instead of blocking the floor.
                state["state"] = "BLOCKED"
                self.topology_lock = None
                self.topology_region = "CORRIDOR"
                rospy.logwarn(
                    "Opposite room %s unreachable for %d cycles; releasing station lock",
                    lock,
                    waited,
                )
                return True
            # We never crossed the doorway.  This is a failed approach, not a
            # completed room.  Release the lock so another portal can be tried
            # while the blocked target remains on a short cooldown.
            state["state"] = "BLOCKED"
            self.topology_lock = None
            self.topology_region = "CORRIDOR"
            rospy.logwarn("Topology room %s approach failed before doorway crossing", lock)
            return True
        # Never dispatch another topology while the robot is still inside this
        # room.  Keep the lock and let later map/camera updates expose another
        # local frontier.  (Validated pre-change behaviour: the bounded release
        # below was an unvalidated experiment and is kept available only via
        # room_frontier_wait_cycles > 0.)
        state["state"] = "EXPLORING"
        self.topology_miss_cycles[lock] = 0
        if self.room_frontier_wait_cycles <= 0:
            rospy.logwarn_throttle(
                5.0,
                "Topology room %s has no executable frontier but local coverage is low "
                "(laser=%.3f camera=%.3f); keeping room lock",
                lock,
                local.laser if local is not None else 0.0,
                local.camera if local is not None else 0.0,
            )
            return False
        idle = int(state.get("no_frontier_cycles", 0)) + 1
        state["no_frontier_cycles"] = idle
        if idle < self.room_frontier_wait_cycles:
            rospy.logwarn_throttle(
                5.0,
                "Topology room %s has no executable frontier but local coverage is low "
                "(laser=%.3f camera=%.3f); keeping room lock (%d/%d)",
                lock,
                local.laser if local is not None else 0.0,
                local.camera if local is not None else 0.0,
                idle,
                self.room_frontier_wait_cycles,
            )
            return False
        state["state"] = "BLOCKED"
        state["no_frontier_cycles"] = 0
        self.topology_lock = None
        self.topology_region = "CORRIDOR"
        rospy.logwarn(
            "Topology room %s exhausted frontiers for %d cycles at camera=%.3f; "
            "releasing the room lock",
            lock,
            idle,
            local.camera if local is not None else 0.0,
        )
        return True

    def _mark_room_entered_locked(self):
        """Capture the narrow doorway crossing at control-loop frequency."""
        lock = self.topology_lock
        if lock is None:
            return False
        state = self.topology_states.get(lock, {})
        if state.get("state") == "EXPLORING":
            return True
        if state.get("state") != "APPROACHING":
            return False
        portal = self.topology_portals.get(str(lock))
        along, lateral = self._topology_coordinates()
        if along is None or lateral is None:
            return False
        side = None
        wall = None
        longitudinal_ok = True
        crossing_ok = None
        if portal is not None and self.gate_source is not None:
            # Door-opening sanity: the robot must be at this opening, not
            # somewhere else along the same wall.
            longitudinal_ok = abs(along - portal.along) <= max(
                1.0, 0.5 * float(portal.width) + 0.45
            )
            # Entry is a STATE, not an event: the robot's own pose is on the
            # room side of the door plane.  The plan-progress test that used to
            # live here (``path_index`` past the first in-room waypoint) was an
            # event derived from a path that is rebuilt every ``replan_period``
            # and resets ``path_index``, so it could be reset before it ever
            # fired - run78 sat on the door line with ``room_entry_counts``
            # still empty.  This is also the simple form of the room-region
            # membership test that replaces the whole door-proof machinery.
            # Entry is a STATE: the robot's own pose is inside this room's
            # region.  Membership is the single truth shared with the coverage
            # denominator; the door-plane test stays only as the fallback used
            # while the portal set is still unknown.
            region = self._region_of_pose()
            if region is not None:
                crossing_ok = str(region) == str(lock)
            else:
                sign = 1.0 if str(portal.side).upper().startswith("L") else -1.0
                crossing_ok = sign * (lateral - float(portal.lateral)) > 0.0
        if crossing_ok is None:
            # No cached portal for this id (an upper floor re-bins a reused
            # doorway as a fresh ROOM_*_NN).  The map-measured wall offset is the
            # only evidence available then, and refusing entry here would leave
            # such a floor unfinished, so keep it as a fallback only.
            side = "L" if lateral > 0.0 else "R"
            left_wall, right_wall = measure_corridor_walls(
                self.raw_grid,
                self.gate_source[:2],
                self.gate_source[2],
                self.planner.corridor_half_width,
            ) if self.raw_grid is not None and self.gate_source is not None else (None, None)
            measured = left_wall if side == "L" else right_wall
            wall = float(measured) if measured else float(self.planner.corridor_half_width)
            crossing_ok = (
                lateral > wall + 0.05 if side == "L" else lateral < -wall - 0.05
            )
        if not (longitudinal_ok and crossing_ok):
            return False
        state["state"] = "EXPLORING"
        self.topology_region = "ROOM_EXPLORING"
        if self.world_pose is not None:
            self.room_entry_pub.publish(
                String(
                    data=json.dumps(
                        {
                            "candidate_id": str(lock),
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
            self.room_scope_announced = str(lock)
        rospy.loginfo("Topology room %s doorway crossing confirmed", lock)
        # Count every physical crossing so "entered, came back out, went in
        # again" is measurable rather than a matter of watching RViz.
        self.room_entry_counts[str(lock)] = (
            self.room_entry_counts.get(str(lock), 0) + 1
        )
        # A fresh entry gets a fresh interior budget.
        self.room_interior_retries[str(lock)] = 0
        # T-E: keep the ordered run-level history, not just the per-floor total.
        # ``room_entry_counts`` is reset on every floor change, so "entered room
        # X, left, re-entered later" was only visible as a warn line at the
        # moment it happened, if the 120 s sampler caught it at all.
        self.room_entry_sequence.append(
            [
                round(rospy.Time.now().to_sec(), 3),
                int(self.floor_index),
                str(lock),
                int(self.room_entry_counts[str(lock)]),
            ]
        )
        if self.room_entry_counts[str(lock)] > 1:
            rospy.logwarn(
                "Room %s entered %d times; a retired room must never be "
                "re-entered (retired=%s)",
                lock,
                self.room_entry_counts[str(lock)],
                sorted(self.retired_topologies),
            )
        return True

    def _end_pose_clearance(self):
        """Clearance at the robot's own cell on the map the elevator plans on.

        The transition node plans with ``navigation_clearance`` on the
        despeckled navigation map, so the end-of-floor pose has to be judged on
        exactly that map; anything else would let a floor latch at a pose where
        the very next A* call returns no route at all.  Cached briefly because
        the underlying distance transform is not cheap and the status payload
        also reports it.
        """
        grid = self.navigation_grid if self.navigation_grid is not None else self.grid
        if grid is None or self.pose is None:
            return None
        cell = grid.world_to_cell(float(self.pose[0]), float(self.pose[1]))
        now = rospy.Time.now().to_sec()
        cached = self._end_pose_clearance_cache
        if cached is not None and cached[0] == cell and now - cached[1] <= 1.0:
            return cached[2]
        try:
            _safe, clearance = self.planner._navigation_fields(
                grid, None, despeckle=True
            )
        except Exception:
            return None
        if not self.planner._in_bounds(clearance.shape, *cell):
            return None
        value = float(clearance[cell])
        self._end_pose_clearance_cache = (cell, now, value)
        return value

    def _end_pose_is_navigable(self):
        clearance = self._end_pose_clearance()
        if clearance is None:
            # Never invent a new blocking condition when the map is not
            # available; the transition has its own bounded recovery.
            return True
        return clearance >= self.planner.navigation_clearance

    def _finish_corridor_return_locked(self):
        """Finish one room and keep its station locked until its opposite."""
        lock = self.topology_lock
        if lock is None:
            return False
        state = self.topology_states.get(lock, {})
        if state.get("state") != "RETURNING":
            return False
        region = self._region_of_pose()
        if region is not None:
            # Exit is the same membership test from the other side: the pose is
            # back in the corridor region.
            if str(region) != "CORRIDOR":
                return False
        else:
            _along, lateral = self._topology_coordinates()
            if lateral is None or abs(lateral) > self.floor_end_corridor_tolerance:
                return False
        if not self._end_pose_is_navigable():
            rospy.logwarn_throttle(
                5.0,
                "Room %s reached the corridor centreline but the robot cell is "
                "not navigable (clearance %.2f < %.2f); holding the return so a "
                "floor cannot latch inside a doorway",
                lock,
                self._end_pose_clearance() or float("nan"),
                self.planner.navigation_clearance,
            )
            return False
        state["state"] = "COMPLETE"
        self.completed_topologies.add(lock)
        # Retire the room for the rest of the run: it has been entered, worked
        # and left, so no future cycle may dispatch a doorway crossing into it
        # again.  This is what makes the observed "in through the door, back to
        # the corridor, in through the door again" structurally impossible.
        self.retired_topologies.add(str(lock))
        if lock in self.front_station_topologies:
            self.completed_front_sides.add(lock.split("_")[-2])
        else:
            # The lock id can differ from the front station id for the very same
            # physical doorway: run46 floor 2 explored and completed the front
            # pair as ROOM_L_10/ROOM_R_10 while the station identity had drifted
            # to ROOM_L_15/ROOM_R_15, so this side was never recorded and the
            # FRONT door search could not advance to REAR.  Match the doorway,
            # not the transient label.
            lock_portal = self.topology_portals.get(str(lock))
            if lock_portal is not None:
                for front_id in sorted(self.front_station_topologies):
                    front_portal = self.topology_portals.get(str(front_id))
                    if front_portal is None:
                        continue
                    if doorways_match(
                        lock_portal,
                        front_portal,
                        self.front_station_match_tolerance,
                    ):
                        self.completed_front_sides.add(str(front_portal.side))
                        break
        self.returning_topology = None
        self.active_target = None
        self.active_path = ()
        self.path_index = 0
        self.last_plan_time = rospy.Time(0)
        partner_id = state.get("station_partner")
        if partner_id is None:
            source = self.topology_portals.get(str(lock))
            if source is not None:
                opposite = opposite_room_portal(
                    source, tuple(self.topology_portals.values())
                )
                if opposite.topology_id not in self.completed_topologies:
                    self.topology_portals.setdefault(opposite.topology_id, opposite)
                    opposite_state = self.topology_states.setdefault(
                        opposite.topology_id,
                        {"state": "APPROACHING", "targets": 0},
                    )
                    opposite_state["state"] = "APPROACHING"
                    opposite_state["station_partner"] = str(lock)
                    self.topology_lock = opposite.topology_id
                    self.topology_region = "OPPOSITE_ROOM_APPROACHING"
                    rospy.loginfo(
                        "Topology room %s complete; station remains locked for opposite room %s",
                        lock,
                        opposite.topology_id,
                    )
                    return True
        self.topology_lock = None
        self.topology_region = "CORRIDOR"
        rospy.loginfo(
            "Opposing room pair complete at %s; releasing station to CORRIDOR",
            lock,
        )
        return True

    def _interior_targets_only_locked(self):
        """Whether the robot is inside a locked room and must stay targeting it.

        Free re-orientation inside is allowed; moving the target back out into
        the corridor is not, until the interior has failed
        ``interior_retry_limit`` times - a bounded escape, so a room with no
        candidate still cannot deadlock.
        """
        with self.lock:
            lock = self.topology_lock
            if lock is None:
                return False
            inside = (
                int(self.room_entry_counts.get(str(lock), 0)) > 0
                and self.returning_topology is None
            )
            retries = int(self.room_interior_retries.get(str(lock), 0))
            limit = int(self.interior_retry_limit)
        return interior_targets_only(inside, retries, limit)

    def _recover_idle_lock(self, plan):
        """Do something useful when a locked room has no candidate at all.

        The planner can legitimately produce nothing for a locked room - its
        interior is still unknown, so there is no frontier and no camera
        viewpoint to seed from.  run55 floor 0 locked ROOM_R_3 in exactly that
        state, produced ``target=None`` and stood 1 m from the door until the
        position watchdog killed the run.

        Recovery is bounded and in this order:

        1. already inside the room -> drive the verified return-to-corridor path
           (the room keeps its lock and stays unfinished, so it can be finished
           from the corridor side later);
        2. not inside yet -> release the lock so the corridor assignment can work
           the rest of the zone and re-open this room on a later cycle;
        3. after ``max_lock_releases`` -> park the doorway as BLOCKED so the
           corridor cannot keep re-locking a room it cannot reach.
        """
        lock = str(self.topology_lock)
        entered = int(self.room_entry_counts.get(lock, 0))
        if entered > 0:
            self.room_interior_retries[lock] = int(
                self.room_interior_retries.get(lock, 0)
            ) + 1
            # The corridor return is only finished by
            # _finish_corridor_return_locked, which requires
            # state == "RETURNING".  Without this assignment the return never
            # terminates, returning_topology stays set forever, and
            # _interior_targets_only_locked() reports inside=False for the rest
            # of the mission - which re-opens corridor targets while the robot
            # is still inside the room (the enter -> leave -> re-enter shuttle).
            self.topology_states.setdefault(
                lock, {"state": "APPROACHING", "targets": 0}
            )["state"] = "RETURNING"
            self.topology_region = "ROOM_RETURNING"
        if entered > 0 and self._start_corridor_return(plan, self.topology_lock):
            rospy.loginfo(
                "Locked room %s produced no candidate; returning to corridor "
                "(entries=%d)",
                lock,
                entered,
            )
            return True
        releases = int(self.lock_release_counts.get(lock, 0))
        if releases < self.max_lock_releases:
            self.lock_release_counts[lock] = releases + 1
            # A released room must not keep a state that claims it is already
            # being explored: the next lock inherits EXPLORING, which skips the
            # entry proof entirely (no crossing count, no EXPLORING transition)
            # and lets the robot cross the doorway unrecorded.  Reset to
            # APPROACHING so the next attempt re-proves entry.
            self.topology_states.setdefault(
                lock, {"state": "APPROACHING", "targets": 0}
            )["state"] = "APPROACHING"
            self.topology_lock = None
            self.topology_region = "CORRIDOR"
            self.active_target = None
            self.active_path = ()
            self.path_index = 0
            self.last_plan_time = rospy.Time(0)
            rospy.logwarn(
                "Locked room %s produced no candidate (entries=%d); releasing "
                "the lock so the zone can continue (%d/%d)",
                lock,
                entered,
                releases + 1,
                self.max_lock_releases,
            )
            return True
        state = self.topology_states.setdefault(
            lock, {"state": "APPROACHING", "targets": 0}
        )
        state["state"] = "BLOCKED"
        self.retired_topologies.add(lock)
        self.topology_lock = None
        self.topology_region = "CORRIDOR"
        self.active_target = None
        self.active_path = ()
        self.path_index = 0
        self.last_plan_time = rospy.Time(0)
        rospy.logwarn(
            "Parking %s as BLOCKED after %d releases with no candidate",
            lock,
            releases,
        )
        return True

    def _retry_blocked_room(self, plan):
        """Re-open a blocked, unfinished room when nothing else is left.

        The planner only emits targets for CONFIRMED topology owners.  A room
        whose approach failed is parked as BLOCKED and its doorway is never
        attempted again, so once the corridor is fully mapped the plan contains
        no target at all.  Re-opening the least-covered blocked room resets that
        room's miss counter and lock so the next planner cycle can produce a
        fresh approach.  Bounded by ``max_room_retries`` so an unreachable room
        cannot consume the whole run.
        """
        if plan is None or plan.target is not None or self.floor_complete:
            return False
        coverages = plan.topology_coverages or {}
        candidates = []
        for topology_id, state in self.topology_states.items():
            if state.get("state") != "BLOCKED":
                continue
            if topology_id in self.completed_topologies:
                continue
            if topology_id in self.retired_topologies:
                # A room retired by _recover_idle_lock was parked on purpose
                # ("so the corridor cannot keep re-locking a room it cannot
                # reach"); re-opening it here would contradict that park.
                continue
            if not str(topology_id).startswith("ROOM"):
                continue
            retries = int(self.room_retry_counts.get(topology_id, 0))
            if retries >= self.max_room_retries:
                continue
            local = coverages.get(topology_id)
            combined = float(local.combined) if local is not None else 0.0
            candidates.append((combined, topology_id, retries))
        if not candidates:
            return False
        candidates.sort()
        _combined, topology_id, retries = candidates[0]
        self.room_retry_counts[topology_id] = retries + 1
        self.topology_states.setdefault(topology_id, {})["state"] = "APPROACHING"
        self.topology_states[topology_id]["targets"] = 0
        self.topology_miss_cycles[topology_id] = 0
        self.topology_lock = topology_id
        self.topology_region = "ROOM_APPROACHING"
        # The earlier failed checkpoint must not suppress the retry viewpoint.
        self.blocked_targets.clear()
        rospy.logwarn(
            "Re-opening blocked room %s for approach retry %d/%d",
            topology_id, retries + 1, self.max_room_retries,
        )
        return True

    def _check_completion(self):
        unreviewed = [
            item for item in self._stable_spheres() if item["id"] not in self.reviewed_hypotheses
        ]
        # Floor completion depends only on the four rooms reaching COMPLETE.
        # A stable lidar sphere hypothesis whose cell the camera never covers
        # must not block the floor: the RGB-D danger detector is the task
        # sensor, and gating on the lidar hint made the robot loop back into an
        # already-complete room (run22: rooms 4/4, floor never latched, 200+ s
        # wasted).  The unreviewed count is kept as a diagnostic only.
        meets = topology_completion_ready(
            self.completed_topologies,
            self.expected_rooms_per_floor,
            0,
        )
        now = rospy.Time.now()
        # A completed floor that never latches is otherwise invisible.  Report
        # the predicate inputs; unreviewed lidar hints no longer block the gate
        # but are still worth seeing.
        rooms = len(self.completed_topologies)
        if rooms >= int(self.expected_rooms_per_floor) and not meets:
            rospy.logwarn_throttle(
                5.0,
                "Completion blocked by topology state: rooms=%d/%d unreviewed=%d stable=%d reviewed=%d",
                rooms,
                int(self.expected_rooms_per_floor),
                len(unreviewed),
                len(self._stable_spheres()),
                len(self.reviewed_hypotheses),
            )
        elif rooms >= int(self.expected_rooms_per_floor) and unreviewed:
            rospy.loginfo_throttle(
                10.0,
                "Floor completion proceeding with %d unreviewed lidar hint(s) "
                "(RGB-D detector is the danger path)",
                len(unreviewed),
            )
        if not meets:
            self.completion_since = None
            return
        if self.completion_since is None:
            self.completion_since = now
            rospy.loginfo(
                "Floor completion pending: rooms=%d/%d stable %.1fs",
                rooms,
                int(self.expected_rooms_per_floor),
                self.completion_stable_duration,
            )
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

    def _check_completion_from_control(self):
        """Run the floor-completion latch from the CONTROL loop as well.

        ``_check_completion`` used to be called only by the planning callback,
        and that callback returns immediately (and used to do so silently) when
        one of its inputs is momentarily missing.  The completion latch - and
        with it the elevator handover - therefore waited for the planner to come
        back: run152 stood still for 74.3 s and then another 61.3 s around
        "Floor completion pending: rooms=4/4 stable 3.0s", even though the
        predicate was already true and only needs 3 s to be stable.  The control
        loop is alive throughout, so it latches the floor on its own.
        """
        if self.floor_complete:
            return
        now = rospy.Time.now()
        last = self.last_completion_check
        if last is not None and (now - last).to_sec() < 0.2:
            return
        self.last_completion_check = now
        self._check_completion()

    def _control(self, _event):
        # A raising rospy.Timer callback kills its thread for good.  run44 froze
        # with cmd_vel == (0, 0) for the rest of the mission because an
        # UnboundLocalError in the body terminated the control loop; the
        # traceback only ever reached the supervisor log, so the freeze looked
        # like a planning stall.  Keep the loop alive and make it observable.
        try:
            self._control_impl(_event)
        except Exception:  # noqa: BLE001 - a control fault must not be fatal
            self.control_faults += 1
            rospy.logerr(
                "CONTROL_FAULT #%d (control loop kept alive):\n%s",
                int(self.control_faults),
                traceback.format_exc(),
            )
        # Independent of the planning callback, which can be paused on a missing
        # input (see _check_completion_from_control).
        try:
            self._check_completion_from_control()
        except Exception:  # noqa: BLE001
            self.control_faults += 1
            rospy.logerr(
                "CONTROL_FAULT #%d in the completion latch:\n%s",
                int(self.control_faults),
                traceback.format_exc(),
            )

    def _control_impl(self, _event):
        with self.lock:
            self._mark_room_entered_locked()
            self._finish_corridor_return_locked()
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
        if not self.initial_forward_complete:
            self._control_initial_forward(pose, world_pose, front)
            self._publish_status()
            return
        # Far-zone committed transit takes precedence over target chasing: the
        # corridor fixed step is a manoeuvre (centre, align, measured run), not a
        # target point, so a map update cannot re-aim it mid-leg.  Once a leg is
        # under way its own stored landing is used, so a later plan (a dispatched
        # target point) cannot interrupt it.
        committed_along = None
        with self.lock:
            if self.committed_transit is not None:
                committed_along = self.committed_transit.get("target_along")
        if committed_along is None:
            committed_along = self._committed_transit_along()
        if committed_along is not None and self._control_committed_transit(
            pose, committed_along
        ):
            return
        if pose is not None:
            # Displacement-based progress stamp (see the constructor note).
            if (
                self.last_progress_position is None
                or math.hypot(
                    float(pose[0]) - self.last_progress_position[0],
                    float(pose[1]) - self.last_progress_position[1],
                )
                >= 0.15
                or abs(
                    normalize_angle(float(pose[2]) - self.last_progress_yaw)
                )
                >= 0.20
            ):
                self.last_progress_position = (float(pose[0]), float(pose[1]))
                self.last_progress_yaw = float(pose[2])
                self.last_progress_stamp = rospy.Time.now()
        # Liveness backstop, deliberately placed BEFORE the path check: an
        # active target whose path went empty takes the stop-and-return below
        # and would bypass a backstop placed after it.  run43 stood at the
        # corridor centreline with a ROOM_L_15 camera frontier, cmd_vel ==
        # (0, 0) and stuck_target_drops == 0 for 40+ sim seconds for exactly
        # that reason.  Judged on the node's own ``active_target`` so both the
        # empty-path and the no-progress cases are covered.
        if not complete and self.stuck_target_seconds > 0.0 and target is not None:
            # Idleness is measured from the LATER of the last physical progress
            # and the moment this target became active.  ``last_progress_stamp``
            # only advances while the robot actually moves (>=0.15 m / >=0.20
            # rad), so a target adopted while the robot was still standing still
            # inherited the PREVIOUS target's idle time and was condemned 0.15 s
            # after being chosen - before it could possibly have moved.  Dropping
            # it stopped the robot again, which kept the stamp fresh, so the next
            # target was condemned the same way: run150 spent 9.55 s + 15.45 s +
            # 12.25 s of its 22.8 % standstill in exactly this loop (a fresh
            # target dropped "after 5.4s" 0.15 s after adoption), and run146 /
            # run147 froze for 43.9 s / 59.1 s the same way.  A freshly adopted
            # target must always get the full window, while a target that has
            # genuinely made no progress since it started is still dropped.
            anchor = self.last_progress_stamp
            started = self.active_target_started
            if started is not None and started > anchor:
                anchor = started
            idle = (rospy.Time.now() - anchor).to_sec()
            if idle >= self.stuck_target_seconds:
                with self.lock:
                    self.blocked_targets.append(
                        (
                            float(target.target[0]),
                            float(target.target[1]),
                            rospy.Time.now().to_sec() + self.stuck_target_cooldown,
                        )
                    )
                    self.active_target = None
                    self.active_path = ()
                    self.path_index = 0
                    self.last_plan_time = rospy.Time(0)
                    self.stuck_target_drops += 1
                    self.last_motion_command_stamp = rospy.Time.now()
                    self.last_progress_stamp = rospy.Time.now()
                rospy.logwarn(
                    "Dropping stuck target %s/%s after %.1fs with no actual "
                    "displacement (backstop drops=%d, path_was_empty=%s)",
                    target.kind,
                    target.topology_id,
                    idle,
                    int(self.stuck_target_drops),
                    "yes" if not path else "no",
                )
                self._stop()
                return
        if target is None or not path:
            # An active target with no executable path used to stop silently for
            # ever: run57 held a CAMERA_FRONTIER/CORRIDOR target with cmd_vel
            # exactly (0, 0) while the planner still reported one.  Make it
            # observable and bounded - after a few cycles the target is cleared
            # so the next plan can offer a routable alternative.
            if target is not None:  # inside this branch the path is empty
                self.empty_path_cycles += 1
                rospy.logwarn_throttle(
                    5.0,
                    "Active target %s/%s has no executable path "
                    "(cycles=%d, backstop drops=%d)",
                    getattr(target, "kind", "?"),
                    getattr(target, "topology_id", "?"),
                    int(self.empty_path_cycles),
                    int(self.stuck_target_drops),
                )
                if self.empty_path_cycles >= 2:
                    with self.lock:
                        self.active_target = None
                        self.active_path = ()
                        self.path_index = 0
                        self.last_plan_time = rospy.Time(0)
                    self.empty_path_cycles = 0
            else:
                self.empty_path_cycles = 0
                plan_target = getattr(self.current_plan, "target", None)
                rospy.logwarn_throttle(
                    5.0,
                    "Control stop: no active target (plan_reason=%s, "
                    "plan_target=%s/%s, active_path_len=%d, committed=%s, "
                    "path_index=%s)",
                    self.last_plan_reason,
                    getattr(plan_target, "kind", None),
                    getattr(plan_target, "topology_id", None),
                    len(path),
                    self.committed_transit is not None,
                    path_index,
                )
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
            if (
                distance > self.creep_stop_distance
                and target.kind in ("LASER_FRONTIER", "CAMERA_FRONTIER")
                and self.front_clearance >= self.motion_stop_distance
            ):
                # Creep the last few centimetres instead of declaring arrival.
                # A frontier inside the stop tolerance but still ahead of the
                # robot was marked reached without any motion, so the planner
                # re-dispatched the same 0.28 m target for ever and the corridor
                # ahead never got mapped (run59 floor 1: cmd_vel (0, 0) for 58
                # sim seconds, path_unreachable=87 for the room behind it).
                command = Twist()
                command.linear.x = min(self.motion_speed, self.creep_speed)
                bearing = math.atan2(waypoint[1] - pose[1], waypoint[0] - pose[0])
                command.angular.z = max(
                    -0.20, min(0.20, 0.8 * normalize_angle(bearing - pose[2]))
                )
                self._publish_command(command)
                return
            self._stop()
            with self.lock:
                if self.path_index + 1 < len(self.active_path):
                    self.path_index += 1
                    self._publish_path()
                    return
                if target.kind in ("SPHERE_REVIEW", "CAMERA_FRONTIER") and target.look_at is not None:
                    look_yaw = math.atan2(target.look_at[1] - pose[1], target.look_at[0] - pose[0])
                    error = normalize_angle(look_yaw - pose[2])
                    if abs(error) > 0.12 and not self._align_stalled(abs(error)):
                        command = Twist()
                        command.angular.z = math.copysign(self.turn_speed, error)
                        self._publish_command(command)
                        self.review_hold_since = None
                        return
                    self._clear_align_stall()
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
        heading_tolerance = self.heading_tolerance
        # Doorway heading discipline (run83 report): the path still owns the
        # heading - there is deliberately NO override to the portal normal - but
        # inside the door band the envelope is tightened so the robot lines up
        # before crossing instead of clipping the jamb.  Leaving 0.25 rad
        # everywhere was the relaxation that made door entry hard.
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
            if (
                abs(along - topology_portal.along)
                <= max(0.75, 0.5 * float(topology_portal.width) + 0.30)
                and side_sign * waypoint_lateral > side_sign * lateral + 0.08
            ):
                heading_tolerance = min(heading_tolerance, 0.12)
        yaw_error = normalize_angle(target_yaw - pose[2])
        command = Twist()
        if abs(yaw_error) > heading_tolerance:
            # Turning (in place or while moving) ends the walking leg: the next
            # walk ramps in again from the floor speed.
            self.walk_ramp_started = None
            if self._align_stalled(abs(yaw_error)):
                # The bearing keeps flipping: move while turning instead of
                # rotating on the spot for ever.
                command.linear.x = min(float(self.motion_speed), 0.30)
                command.angular.z = max(-0.6, min(0.6, 1.2 * yaw_error))
            else:
                command.angular.z = math.copysign(self.turn_speed, yaw_error)
        elif front < self.motion_stop_distance:
            self._clear_align_stall()
            with self.lock:
                # A local collision stop is a navigation failure, not proof
                # that this viewpoint has been explored.  Suppress it only
                # briefly so a map update can produce a new approach.
                self.blocked_targets.append(
                    (
                        float(target.target[0]),
                        float(target.target[1]),
                        rospy.Time.now().to_sec() + self.collision_block_seconds,
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
            self._clear_align_stall()
            now = rospy.Time.now()
            if self.walk_ramp_started is None:
                self.walk_ramp_started = now
            goal = path[-1] if path else waypoint
            remaining = math.hypot(goal[0] - pose[0], goal[1] - pose[1])
            command.linear.x = leg_speed(
                self.motion_speed,
                (now - self.walk_ramp_started).to_sec(),
                remaining,
                accel_seconds=self.leg_accel_seconds,
                decel_distance=self.leg_decel_distance,
                ramp_floor_fraction=self.leg_ramp_floor_fraction,
                minimum_speed=self.leg_minimum_speed,
            )
            command.angular.z = max(-0.15, min(0.15, 0.8 * yaw_error))
        self._publish_command(command)

    def _adopt_provisional_gate(self):
        """Fall back to the configured entrance gate when detection stalls.

        The transit phase ends only when the entrance gate is resolved from the
        map.  The gate is by construction a fixed offset from the spawn anchor
        (``virtual_gate_forward_distance`` along the configured heading), so that
        provisional formulation is a valid entrance definition.  run66 wandered
        the lobby for more than 100 sim seconds with
        ``initial_forward_progress`` stuck at 7/14.5 m and never resolved one,
        which blocked every later phase (no gate -> no task region -> no
        targets).
        """
        with self.lock:
            anchor = self.forward_anchor
            yaw = self.forward_yaw
            entry_world = self.lobby_entry_world
            world_yaw = self.world_forward_yaw
        if anchor is None:
            return False
        if self.transit_started_wall is None:
            # Sim seconds, like every other budget: at RTF ~0.1 a wall-clock
            # timeout would fire after only a few sim seconds and pre-empt the
            # normal detection path.
            self.transit_started_wall = rospy.Time.now().to_sec()
        elapsed = rospy.Time.now().to_sec() - self.transit_started_wall
        if elapsed < self.transit_timeout_seconds:
            return False
        distance = float(self.virtual_gate_forward_distance)
        gate = (
            anchor[0] + distance * math.cos(yaw),
            anchor[1] + distance * math.sin(yaw),
            float(yaw),
        )
        with self.lock:
            self.gate_source = gate
            if entry_world is not None and world_yaw is not None:
                self.gate_world = (
                    entry_world[0] + distance * math.cos(world_yaw),
                    entry_world[1] + distance * math.sin(world_yaw),
                    float(world_yaw),
                )
            # Adopting the configured gate is the documented way out of a stalled
            # entrance transit (run66: progress stuck at 7/14.5 m), so it also
            # ends the transit phase; otherwise the distance-gated control loop
            # would keep driving instead of letting exploration start.
            self.initial_forward_complete = True
            self.transit_fallbacks += 1
        rospy.logwarn(
            "Entrance gate not detected from the map after %.0f s; adopting the "
            "configured provisional gate at (%.2f, %.2f) yaw=%.2f "
            "(transit fallbacks=%d)",
            elapsed,
            gate[0],
            gate[1],
            gate[2],
            int(self.transit_fallbacks),
        )
        return True

    def _ramped_forward_speed(self):
        """Speed for the transit, ramped in from a standstill.

        Returns 0.0 while the post-stand settling window is open, then a linear
        ramp to ``initial_forward_speed``.  A quadruped that just stood up tips
        over if it is asked for full speed immediately (run70: imu_roll -> pi at
        sim 11 s under a 0.90 m/s command issued ~5 s after the RL switch).
        """
        if self.forward_started_stamp is None:
            self.forward_started_stamp = rospy.Time.now()
        elapsed = (rospy.Time.now() - self.forward_started_stamp).to_sec()
        if elapsed < self.initial_forward_settle_seconds:
            return 0.0
        ramp_elapsed = elapsed - self.initial_forward_settle_seconds
        span = self.initial_forward_ramp_seconds
        target = min(self.motion_speed, self.initial_forward_speed)
        if span <= 0.0:
            return target
        fraction = min(1.0, ramp_elapsed / span)
        speed = (
            self.initial_forward_start_speed
            + (target - self.initial_forward_start_speed) * fraction
        )
        return max(0.0, min(target, speed))

    def _committed_transit_along(self):
        """Landing along of the far-zone committed leg the node should run now.

        ONE source only: the core-requested landing
        (``committed_transit_along``, set together with ``fixed_step_transit``).

        Everything the node used to add on its own is deliberately gone (user,
        2026-09-14: "stop changing things, roll it back - waiting 30 s at least
        it can still go"): the default-forward-step fallback re-aimed the leg
        right after the floor-entry transit, ran the corridor past the near-zone
        doorways, and competed with the plan's own targets.  Waiting for the plan
        is the behaviour that actually reaches targets, so the plan is the only
        authority here.
        """
        plan = self.current_plan
        diagnostics = getattr(plan, "diagnostics", None) or {}
        along = diagnostics.get("committed_transit_along")
        if diagnostics.get("fixed_step_transit") and along is not None:
            return float(along)
        return None

    def _control_committed_transit(self, pose, target_along):
        """Far-zone transit as a committed manoeuvre, not a chased target.

        Same primitive as the lift return: get onto the corridor centreline,
        align with the corridor axis, then run one measured distance to the
        target doorway along.  Being open-loop, nothing can re-aim it mid-leg,
        which is exactly why the frontier-driven version failed to reach the far
        zone.  Returns True when it handled this control cycle.
        """
        # Phase gate (defensive): this leg owns the corridor only after the
        # floor-entry transit is done and before the floor completes.
        if not self.initial_forward_complete or self.floor_complete:
            self.committed_transit = None
            return False
        with self.lock:
            gate = self.gate_source
            committed = self.committed_transit
        if gate is None:
            self.committed_transit = None
            return False
        cosine, sine = math.cos(float(gate[2])), math.sin(float(gate[2]))
        dx = float(pose[0]) - float(gate[0])
        dy = float(pose[1]) - float(gate[1])
        along_now = dx * cosine + dy * sine
        lateral = -dx * sine + dy * cosine
        if committed is None:
            if target_along - along_now <= self.committed_arrival_tolerance:
                return False
            if abs(lateral) > self.committed_center_tolerance:
                heading = normalize_angle(
                    float(gate[2]) - math.pi / 2.0
                    if lateral > 0.0
                    else float(gate[2]) + math.pi / 2.0
                )
                distance = abs(lateral)
            else:
                heading = normalize_angle(float(gate[2]))
                distance = target_along - along_now
            committed = {
                "heading": heading,
                "distance": float(distance),
                "anchor": None,
                # Remembered so a later plan cannot cancel the leg: a committed
                # manoeuvre is immune to plan changes while it runs.
                "target_along": float(target_along),
            }
            with self.lock:
                self.committed_transit = committed
            rospy.loginfo(
                "Committed transit: heading=%.2f distance=%.2f m "
                "(lateral=%.2f, along=%.2f -> %.2f)",
                heading,
                distance,
                lateral,
                along_now,
                target_along,
            )
            return True
        error = normalize_angle(float(committed["heading"]) - float(pose[2]))
        command = Twist()
        if abs(error) > self.heading_tolerance:
            command.angular.z = math.copysign(self.turn_speed, error)
            self._publish_command(command)
            return True
        if committed.get("anchor") is None:
            committed = dict(committed)
            committed["anchor"] = (float(pose[0]), float(pose[1]))
            with self.lock:
                self.committed_transit = committed
            return True
        anchor = committed["anchor"]
        travelled = math.hypot(float(pose[0]) - anchor[0], float(pose[1]) - anchor[1])
        if travelled >= float(committed["distance"]):
            with self.lock:
                self.committed_transit = None
                self.active_target = None
                self.active_path = ()
                self.path_index = 0
                self.last_plan_time = rospy.Time(0)
            self._stop()
            return True
        command.linear.x = min(self.motion_speed, self.committed_speed)
        command.angular.z = max(-0.15, min(0.15, 0.8 * error))
        self._publish_command(command)
        return True

    def _control_initial_forward(self, pose, world_pose, front):
        with self.lock:
            if self.forward_anchor is None:
                self.forward_anchor = pose[:2]
                self.forward_yaw = normalize_angle(self.configured_yaw)
                self.lobby_entry_source = (pose[0], pose[1], self.forward_yaw)
            if self.lobby_entry_world is None and world_pose is not None:
                # Latch the world-frame entry as soon as the metric pose exists,
                # not only on the anchor's first cycle.  The metric pose can
                # arrive a cycle later, and the world-frame gate
                # (``virtual_isolation_door``) and the world-frame elevator
                # portal are both derived from this anchor; the elevator
                # transition node refuses to leave a completed floor while the
                # world gate is None.  Measured on run117 floor 0: both stayed
                # None, so after TASK_REGION_COMPLETE 4/4 the elevator node sat
                # in "Waiting for map-confirmed elevator portal before
                # transition" for the rest of the run.
                frame_yaw = world_pose[2] - pose[2]
                self.world_forward_yaw = normalize_angle(self.forward_yaw + frame_yaw)
                self.lobby_entry_world = (
                    world_pose[0],
                    world_pose[1],
                    self.world_forward_yaw,
                )
            anchor, yaw = self.forward_anchor, self.forward_yaw
            left, right = self.left_clearance, self.right_clearance
        progress = (pose[0] - anchor[0]) * math.cos(yaw) + (pose[1] - anchor[1]) * math.sin(yaw)
        # Detect the lobby/elevator opening while traversing the entrance,
        # rather than waiting until the 14.5 m task gate is committed.  The
        # configured gate offset is already fixed relative to the entrance
        # anchor, so it provides the same map frame for passive detection.
        gate_distance = self.virtual_gate_forward_distance
        provisional_gate = (
            anchor[0] + gate_distance * math.cos(yaw),
            anchor[1] + gate_distance * math.sin(yaw),
            yaw,
        )
        provisional_gate_world = None
        if self.lobby_entry_world is not None:
            provisional_gate_world = (
                self.lobby_entry_world[0]
                + gate_distance * math.cos(self.world_forward_yaw),
                self.lobby_entry_world[1]
                + gate_distance * math.sin(self.world_forward_yaw),
                self.world_forward_yaw,
            )
        if (
            self.elevator_detection_start_distance
            <= progress
            < self.elevator_detection_end_distance
        ):
            self._detect_elevator_portal(
                self.raw_grid, provisional_gate, provisional_gate_world
            )
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
            if int(self.floor_index) > 0:
                # Upper-floor entry.  The elevator node already owns this
                # floor's gate (the floors share x/y topology; it reuses floor
                # 0's gate), so this transit only owed two things: cover the
                # configured forward distance past the lift mouth - sweeping the
                # front doorways into the map - and restore the plane gait after
                # the stair policy the ride needs.  Do not replace a validated
                # gate with a lift-mouth extrapolation.
                with self.lock:
                    self.initial_forward_complete = True
                    self.topology_region = "CORRIDOR"
                    self.active_target = None
                    self.last_plan_time = rospy.Time(0)
                self._publish_markers()
                rospy.loginfo(
                    "Floor-entry transit complete: %.2f m covered on floor %d; "
                    "exploration enabled",
                    progress,
                    int(self.floor_index),
                )
                return
            with self.lock:
                gate_distance = self.virtual_gate_forward_distance
                gate_source = (
                    anchor[0] + gate_distance * math.cos(yaw),
                    anchor[1] + gate_distance * math.sin(yaw),
                    yaw,
                )
                gate_world = None
                if self.lobby_entry_world is not None:
                    gate_world = (
                        self.lobby_entry_world[0]
                        + gate_distance * math.cos(self.world_forward_yaw),
                        self.lobby_entry_world[1]
                        + gate_distance * math.sin(self.world_forward_yaw),
                        self.world_forward_yaw,
                    )
                # The axis above is a straight extrapolation of the robot's
                # starting pose, so any lateral bias at the entrance becomes
                # the corridor axis for the whole run.  Measured on a real
                # three-floor run the axis sat 0.53 m off centre (left wall
                # 0.77 m, right wall 1.63 m), which misplaced every doorway
                # band and mis-binned the rear doors.  Re-anchor the gate on
                # the wall midline once the corridor walls are observed.
                raw = self.raw_grid
            if self.recenter_gate_on_walls and raw is not None:
                left_wall, right_wall = measure_corridor_walls(
                    raw,
                    gate_source[:2],
                    gate_source[2],
                    self.planner.corridor_half_width,
                )
                if left_wall is not None and right_wall is not None:
                    spacing = float(left_wall) + float(right_wall)
                    expected = 2.0 * self.planner.corridor_half_width
                    if 0.60 * expected <= spacing <= 1.60 * expected:
                        offset = 0.5 * (float(left_wall) - float(right_wall))
                        offset = max(
                            -self.maximum_gate_recenter,
                            min(self.maximum_gate_recenter, offset),
                        )
                        # Positive offset means the left wall is farther away,
                        # so the axis sits left of centre and the gate must
                        # move right, i.e. towards negative lateral.
                        cosine, sine = math.cos(yaw), math.sin(yaw)
                        gate_source = (
                            gate_source[0] - offset * sine,
                            gate_source[1] + offset * cosine,
                            yaw,
                        )
                        if gate_world is not None:
                            world_cosine = math.cos(self.world_forward_yaw)
                            world_sine = math.sin(self.world_forward_yaw)
                            gate_world = (
                                gate_world[0] - offset * world_sine,
                                gate_world[1] + offset * world_cosine,
                                self.world_forward_yaw,
                            )
                        rospy.loginfo(
                            "Gate re-anchored on wall midline: offset=%.2f m "
                            "(left=%.2f right=%.2f)",
                            offset,
                            float(left_wall),
                            float(right_wall),
                        )
            with self.lock:
                self.gate_source = gate_source
                self.gate_world = gate_world
                self.initial_forward_complete = True
                self.topology_region = "CORRIDOR"
                self.active_target = None
                self.last_plan_time = rospy.Time(0)
            self._publish_gate()
            self._publish_markers()
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
            # Entrance/lobby transit is the narrowest, most cluttered part of
            # the route: run74 tumbled here after drifting into the door jamb at
            # 0.60 m/s (roll rate spiked to 3.4 rad/s before the fall).  Cap the
            # speed and centre laterally on the measured free span, exactly like
            # the lift entry does.
            # Speed profile: only the jamb zone in front of the entrance needs
            # the reduced cap (run74 tumbled there); the rest of the lobby
            # transit keeps full mission speed so the start is not needlessly
            # slow.  distance_to_gate is along the transit axis.
            distance_to_gate = self.virtual_gate_forward_distance - progress
            near_jamb = 0.0 <= distance_to_gate <= self.transit_jamb_zone
            speed_cap = (
                self.transit_speed_cap if near_jamb else self.motion_speed
            )
            command.linear.x = min(self._ramped_forward_speed(), speed_cap)
            lateral_term = 0.0
            if math.isfinite(left) and math.isfinite(right):
                lateral_term = max(-0.18, min(0.18, 0.10 * (right - left)))
            command.angular.z = max(-0.25, min(0.25, 0.8 * error + lateral_term))
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
        command.linear.x = clamp_linear_speed(command.linear.x, self.max_linear_speed)
        if abs(command.linear.x) > 1e-3 or abs(command.angular.z) > 1e-3:
            self.last_motion_command_stamp = rospy.Time.now()
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
        for point in path[max(0, index - 1):]:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def _note_path_start(self):
        """T-E observable: how far the A* start sits from the robot's own pose.

        Called immediately after ``active_path`` is replaced, while ``self.pose``
        is still the pose the plan was made from.  The route now starts in the
        robot's own cell (``_robot_local_safe``), so this should stay near zero;
        a large value means the start moved away from the robot and is the T1
        shape that used to be argued from RViz screenshots.  Diagnostic only:
        no control flow reads ``path_start_offset`` back.
        """
        if not self.active_path or self.pose is None:
            self.path_start_offset = None
            return
        first = self.active_path[0]
        self.path_start_offset = math.hypot(
            float(first[0]) - float(self.pose[0]),
            float(first[1]) - float(self.pose[1]),
        )

    def _anchor_path_to_current_pose(self, path, target=None):
        """Start the adopted route at the robot's *current* pose.

        Planning is slow (``last_plan_ms`` up to 8 s), so the pose a route was
        planned from is stale by the time it is adopted.  Splicing the old route
        to the live pose is not enough: when the robot is off that route the
        next surviving point can be metres away, so the controller's first leg -
        and the drawn path - still started away from the robot.  Re-plan the
        remainder from the live pose instead; the splice stays as the fallback
        when no route can be recomputed.
        """
        path = tuple(path or ())
        if not path or self.pose is None:
            return path
        x, y = float(self.pose[0]), float(self.pose[1])
        goal = getattr(target, "target", None) if target is not None else None
        if goal is not None:
            nav_grid = (
                self.navigation_grid if self.navigation_grid is not None else self.grid
            )
            if nav_grid is not None:
                replanned, _length, _clearance = self.planner.navigation_path(
                    nav_grid,
                    (x, y, float(self.pose[2]) if len(self.pose) > 2 else 0.0),
                    (float(goal[0]), float(goal[1])),
                )
                if replanned:
                    return tuple(replanned)
        # Fallback: splice the stale route at the nearest point ahead.
        best_index, best_distance = 0, None
        for index, point in enumerate(path):
            distance = math.hypot(float(point[0]) - x, float(point[1]) - y)
            if best_distance is None or distance < best_distance:
                best_index, best_distance = index, distance
        # Continue from the point *after* the nearest one: starting at the
        # nearest point itself can put a behind-the-robot waypoint first and
        # reintroduce the turn-back this exists to remove.
        tail = path[best_index + 1:] if best_index + 1 < len(path) else path[best_index:]
        return ((x, y),) + tuple(tail)

    def _active_remaining_information(self, plan):
        """Fraction of the active viewpoint's information still unobserved.

        The active target's gains were frozen when it was chosen, so the live
        number is read from the fresh candidate sitting at the same place: if
        its gain has collapsed (or the viewpoint no longer appears in the pool
        at all, i.e. it has nothing left to see) the target has been observed
        and may be handed over.
        """
        active = self.active_target
        if active is None or active.look_at is None and active.camera_gain <= 0.0:
            return None
        metric = (
            "camera_gain"
            if active.kind in ("CAMERA_FRONTIER", "SPHERE_REVIEW")
            else "combined_gain"
        )
        recorded = float(getattr(active, metric, 0.0))
        if recorded <= 0.0:
            return None
        fresh = None
        for item in plan.targets or ():
            if item.kind != active.kind:
                continue
            if math.hypot(
                item.target[0] - active.target[0], item.target[1] - active.target[1]
            ) <= 0.4:
                fresh = item
                break
        if fresh is None:
            # The viewpoint is gone from the pool: nothing left to observe.
            return 0.0
        return max(0.0, float(getattr(fresh, metric, 0.0)) / recorded)

    def _arm_room_approach(self, topology_id):
        """Lock the room a freshly adopted target belongs to.

        Shared by the ordinary plan dispatch and ``_switch_active_target``.  A
        mid-route handover is a real adoption of a target, so it must arm the
        same room state; it used to skip this, leaving ``locked_topology`` at
        ``None`` so the next handover had no room constraint.  Measured on
        run107 floor 0: 13 handovers in 196 sim seconds, alternating between
        ROOM_L_15 and ROOM_R_15 (``along`` 4.1-8.9 m, ``lateral`` +0.4 to
        -3.7 m) and never entering either room.
        """
        lock = room_lock_for_target(topology_id, self.retired_topologies)
        if lock is None:
            return
        self.topology_lock = lock
        self.topology_region = "ROOM_APPROACHING"
        state = self.topology_states.setdefault(
            lock, {"state": "APPROACHING", "targets": 0}
        )
        # A new frontier in the same locked room is not a new doorway approach:
        # preserve geometric proof that the robot already crossed into it.
        state["state"] = topology_state_for_new_target(state.get("state"))

    def _switch_active_target(self, plan):
        """Adopt a better viewpoint for the same task before arriving.

        See ``target_switch_allowed`` for the bounds; this only supplies the
        live state (dwell time and room lock) and performs the swap.
        """
        active = self.active_target
        candidate = plan.target
        if active is None or candidate is None:
            return False
        started = getattr(self, "active_target_started", None)
        dwell = 1e9
        if started is not None and started != rospy.Time(0):
            dwell = (rospy.Time.now() - started).to_sec()
        if not target_switch_allowed(
            active,
            candidate,
            dwell,
            locked_topology=self.topology_lock,
            dwell_seconds=self.target_switch_dwell,
            active_remaining=self._active_remaining_information(plan),
        ):
            return False
        rospy.loginfo(
            "Switching target before arrival: %s (%s) -> %s (%s) at %.2f m vs %.2f m",
            active.kind, active.topology_id, candidate.kind, candidate.topology_id,
            candidate.path_length, active.path_length,
        )
        self.target_switches += 1
        self.active_target = candidate
        self.active_path = self._anchor_path_to_current_pose(
            tuple(candidate.path), candidate
        )
        # path[0] is now the robot's live pose, so aim at path[1] exactly as the
        # ordinary dispatch does.
        self.path_index = min(1, len(self.active_path) - 1) if self.active_path else 0
        self._note_path_start()
        # A handover into a room is a room approach: without this the lock stays
        # None and target_switch_allowed cannot keep the next handover in the
        # same room.
        self._arm_room_approach(getattr(candidate, "topology_id", None))
        self.active_target_started = rospy.Time.now()
        self.active_target_last_progress = None
        self.active_target_last_progress_stamp = rospy.Time.now()
        self.active_target_last_distance = None
        return True

    def _refresh_active_heading(self, plan):
        """Re-aim the active viewpoint from the newest map while still driving.

        The viewpoint POSITION is deliberately left where it was chosen --
        swapping targets by gain comparison is what made the old replacement
        mechanism oscillate ("the target changed back and forth for an hour").
        The ORIENTATION is different: it is a pure function of what is still
        unseen, so recomputing it every plan cycle costs nothing and means the
        look-at bearing keeps improving as the robot walks up to the doorway
        instead of being frozen at whatever the map looked like when the target
        was first picked.  FrontierTarget is frozen, so the refreshed copy is
        swapped in wholesale.
        """
        active = self.active_target
        if active is None or active.look_at is None:
            return
        if active.kind not in ("CAMERA_FRONTIER", "SPHERE_REVIEW"):
            return
        best = None
        for item in plan.targets or ():
            if item.kind != active.kind or item.look_at is None:
                continue
            distance = math.hypot(
                item.target[0] - active.target[0], item.target[1] - active.target[1]
            )
            if distance <= 0.4 and (best is None or distance < best[0]):
                best = (distance, item)
        if best is not None and best[1].look_at != active.look_at:
            self.active_target = replace(active, look_at=best[1].look_at)

    def _publish_markers(self):
        with self.lock:
            grid, gate, target = self.grid, self.gate_source, self.active_target
            portal_grid = self.raw_grid if self.raw_grid is not None else grid
            hypotheses = self._stable_spheres()
            reviewed = set(self.reviewed_hypotheses)
            topology_states = dict(self.topology_states)
            topology_lock = self.topology_lock
            cached_portals = dict(self.topology_portals)
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
                    portal_grid,
                    gate[:2],
                    gate[2],
                    self.planner.forward_depth,
                    self.planner.lateral_half_width,
                    self.planner.corridor_half_width,
                    portal_prefix=self.floor_prefix,
                )
                portals_by_id = {
                    portal.topology_id: portal for portal in observed_portals
                }
                # Once a candidate owns a room lock it is a persistent
                # topological gate.  Do not let a later sparse scan erase its
                # RViz representation while the scheduler still enforces it.
                portals_by_id.update(cached_portals)
                portals = [
                    portal
                    for portal in portals_by_id.values()
                    if int(portal_evidence.get(portal.topology_id, 0))
                    >= self.portal_confirm_cycles
                    or portal.topology_id == topology_lock
                    or portal.topology_id in topology_states
                ]
                portals.sort(key=lambda item: (item.along, item.side))
                stations = pair_room_portals(portals)
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
                        "lateral": float(item.lateral),
                        "measured_wall": float(
                            getattr(item, "measured_wall", 0.0) or abs(float(item.lateral))
                        ),
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
                "initial_forward_active": not self.initial_forward_complete,
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
                "completed_topologies": sorted(self.completed_topologies),
                "reused_topology_ids": sorted(self.reused_portals),
                "topology_states": self.topology_states,
                "portal_evidence": dict(self.portal_evidence),
                "portal_confirm_cycles": self.portal_confirm_cycles,
                "camera_union_points": len(self.camera_points_world),
                "camera_union_limit": int(self.camera_points_memory_limit),
                "room_coverages": room_coverages,
                # Region state as one derived value per room (UNSEEN / ACTIVE /
                # COVERED).  This is the single truth the completion bookkeeping
                # is being folded into; it is published first so a run can be
                # checked for completeness room by room.
                "room_region_status": {
                    str(room): region_status(
                        (data or {}).get("combined", 0.0),
                        self.room_combined_coverage_target,
                        active=(str(room) == str(self.topology_lock)),
                    )
                    for room, data in (room_coverages or {}).items()
                },
                "observed_portals": observed_portals,
                "actionable_portals": [
                    item.topology_id for item in (extent.actionable_portals if extent is not None else ())
                ],
                "planner_diagnostics": dict(extent.diagnostics or {})
                if extent is not None
                else {},
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
                # A paused planner (missing input) is otherwise only visible as a
                # frozen plan_cycles counter.
                "plan_input_gaps": int(self.plan_input_gaps),
                "plan_failures": self.plan_failures,
                "last_plan_reason": self.last_plan_reason,
                "last_plan_ms": round(float(self.last_plan_ms), 2),
                "target_switches": int(self.target_switches),
                "last_plan_error": self.last_plan_error,
                "candidate_topologies": list(extent.candidate_topologies) if extent is not None else [],
                "task_forward_limit": extent.task_forward_limit if extent is not None else self.planner.forward_depth,
                "task_extent_confident": extent.task_extent_confident if extent is not None else False,
                "sphere_hypotheses": len(stable),
                "unreviewed_sphere_hypotheses": unreviewed,
                "room_entry_counts": dict(self.room_entry_counts),
                "room_entry_sequence": [list(item) for item in self.room_entry_sequence],
                "path_start_offset": (
                    round(float(self.path_start_offset), 3)
                    if self.active_path and self.path_start_offset is not None
                    else None
                ),
                # Diagnostic: an active committed corridor leg is otherwise
                # invisible in the payload (its absence was read as "no leg").
                "committed_transit": (
                    dict(self.committed_transit)
                    if self.committed_transit is not None
                    else None
                ),
                "retired_topologies": sorted(self.retired_topologies),
                "end_pose_clearance": self._end_pose_clearance(),
                "floor_end_corridor_tolerance": self.floor_end_corridor_tolerance,
                "room_exhaust_grace_seconds": self.room_exhaust_grace_seconds,
                "room_exhaust_min_gain": self.room_exhaust_min_gain,
                "room_exit_requires_gain": self.room_exit_requires_gain,
                "stuck_target_drops": int(self.stuck_target_drops),
                "control_faults": int(self.control_faults),
                "plan_faults": int(self.plan_faults),
                "transit_fallbacks": int(self.transit_fallbacks),
                "empty_path_cycles": int(self.empty_path_cycles),
                "unsafe_path_cycles": int(self.unsafe_path_cycles),
                "unsafe_path_replans": int(self.unsafe_path_replans),
                "route_refreshes": int(self.route_refreshes),
                "lock_release_counts": dict(self.lock_release_counts),
                "room_interior_retries": dict(self.room_interior_retries),
                "interior_only": bool(self._interior_targets_only_locked()),
                "progress_idle_seconds": (
                    (rospy.Time.now() - self.last_progress_stamp).to_sec()
                    if self.last_progress_stamp is not None
                    else 0.0
                ),
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
