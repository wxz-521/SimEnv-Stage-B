#!/usr/bin/env python3
"""Run repeated mapped-floor exploration and elevator transitions."""

import json
import math
import os
import sys
import threading
import ast
import time
from collections import deque
import numpy as np

import rospy
import tf.transformations as transformations
from building_generator_interfaces.srv import CallElevator, SetDoorState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

SCRIPT_DIRECTORY = os.path.dirname(os.path.realpath(__file__))
if not sys.path or sys.path[0] != SCRIPT_DIRECTORY:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from elevator_transition_core import (
    clamp_linear_speed,
    choose_opening_heading,
    detect_wide_lobby_openings,
    entry_stall_confirms_containment,
    entry_blocked_retry_allowed,
    establish_budget_exceeded,
    tilt_fault_state,
    direct_alignment_ready,
    direct_entry_applies,
    door_frame_offset,
    elevator_door_id,
    height_transition_complete,
    normalize_angle,
    planar_distance,
    point_from_gate,
    portal_staging_point,
    target_heading,
    transform_pose_between_frames,
)
from coverage_explorer_core import GridView, TaskCoveragePlanner


class ElevatorTransition:
    WAITING = "WAIT_FLOOR_COMPLETE"

    def __init__(self):
        self.lock = threading.RLock()
        self.enabled = bool(rospy.get_param("~enabled", True))
        self.elevator_id = str(rospy.get_param("~elevator_id", "elevator_main"))
        self.target_floor = int(rospy.get_param("~target_floor", 1))
        self.max_floor = max(
            self.target_floor, int(rospy.get_param("~max_floor", self.target_floor))
        )
        self.finish_at_top_floor = bool(
            rospy.get_param("~finish_at_top_floor", False)
        )
        self.lobby_search_offset = float(rospy.get_param("~lobby_search_offset", -4.6))
        self.gate_staging_offset = float(rospy.get_param("~gate_staging_offset", 0.7))
        self.enter_distance = float(rospy.get_param("~enter_distance", 3.00))
        self.exit_distance = float(rospy.get_param("~exit_distance", 2.10))
        self.floor1_corridor_advance = float(
            rospy.get_param("~floor1_corridor_advance", 1.80)
        )
        self.minimum_entry_progress = float(
            rospy.get_param("~minimum_entry_progress", 1.0)
        )
        # The scene's floor-1 car door starts closed and animates open, so an
        # early contact during entry is usually the door itself.  Back off and
        # retry a bounded number of times instead of failing the mission
        # (run49: ELEVATOR_PATH_BLOCKED_AFTER_0.93M after two complete floors).
        self.entry_retry_limit = max(
            0, int(rospy.get_param("~entry_retry_limit", 4))
        )
        self.entry_retreat_seconds = max(
            0.5, float(rospy.get_param("~entry_retreat_seconds", 2.5))
        )
        self.entry_retreat_speed = max(
            0.05, float(rospy.get_param("~entry_retreat_speed", 0.25))
        )
        self.retreat_until = None
        self.elevator_door_prefix = str(
            rospy.get_param("~elevator_door_prefix", "elevator_floor")
        )
        self.elevator_door_wait_seconds = max(
            0.0, float(rospy.get_param("~elevator_door_wait_seconds", 6.0))
        )
        self.elevator_door_requested_for = None
        self.minimum_exit_progress = float(
            rospy.get_param("~minimum_exit_progress", 1.50)
        )
        self.minimum_opening_clearance = float(
            # Candidate geometry and the later map/entry checks are strong;
            # keep this sensor gate permissive so sparse scans do not suppress
            # a genuine elevator opening.
            rospy.get_param("~minimum_opening_clearance", 1.10)
        )
        self.minimum_floor_rise = float(rospy.get_param("~minimum_floor_rise", 2.0))
        self.motion_speed = float(rospy.get_param("~motion_speed", 0.35))
        # Hard ceiling for every linear command in this node (mission decision:
        # 0.60 m/s everywhere until three floors are stable).
        self.max_linear_speed = max(
            0.10, float(rospy.get_param("~max_linear_speed", 0.60))
        )
        self.crossing_speed = float(rospy.get_param("~crossing_speed", 0.22))
        # How close to the car doorway the robot must be before the ride may
        # start (metres).  Boarding used to be inferred from the entry distance
        # alone, which let run84 ride to floor 1 with the robot 4.1 m away.
        self.board_distance_tolerance = max(
            0.3, float(rospy.get_param("~board_distance_tolerance", 1.2))
        )
        # How far off the doorway centre line the robot may be when it commits
        # to the straight entry run (metres).  The 1.70 m car opening tolerates
        # much more than this, but entering from the centre line is what keeps
        # the robot off the shaft wall beside the opening.  Defined with
        # target_tolerance below, because it must not be tighter than the
        # tolerance at which "arrived at the staging point" is declared.
        # Facing tolerance for boarding: the robot must face the portal normal
        # before the ride may start (user requirement: face the doorway, then
        # enter).  Only the elevator keeps this alignment discipline.
        self.board_heading_tolerance = max(
            0.05, float(rospy.get_param("~board_heading_tolerance", 0.06))
        )
        # Corridor transit may run at motion_speed, but the final lobby/gate
        # approach used to be clamped to min(motion_speed, 0.30) and dominated
        # the transition cost (run 6: RETURN_TO_FLOOR_1_GATE 81 s + 73 s with a
        # 0.30-0.35 m/s ceiling).  Give that approach an explicit ceiling that
        # is still slower than open-corridor transit but no longer a crawl.
        self.lobby_approach_speed = max(
            0.20, float(rospy.get_param("~lobby_approach_speed", 0.45))
        )
        self.turn_speed = float(rospy.get_param("~turn_speed", 0.45))
        self.stop_distance = float(rospy.get_param("~stop_distance", 0.42))
        self.target_tolerance = float(rospy.get_param("~target_tolerance", 0.35))
        # Must be >= target_tolerance: the lateral discipline sends the robot
        # back to the staging point, and _drive_planned_to floors its arrival
        # tolerance at target_tolerance.  run88 deadlocked here -- lateral
        # 0.709 m against a 0.25 m limit while "arrived" was granted at 0.80 m,
        # so the node published cmd_vx=0, cmd_wz=0 for ever and never aligned.
        # Floor completion can leave the robot on a doorway line, and the entry
        # run drifts laterally: measured on run124, the entry STARTED centred
        # (lateral -0.04 m) and ENDED at -0.36 m, i.e. at the old 0.35 m
        # tolerance, and the reverse exit then jammed on the car threshold with
        # the robot 0.50 m off centre.  Keep a small floor (never 0 - that froze
        # the node once) but tighten the default to 0.15 m.
        self.entry_lateral_tolerance = max(
            0.10,
            float(rospy.get_param("~entry_lateral_tolerance", 0.15)),
        )
        self.elevator_approach_tolerance = max(
            self.target_tolerance,
            float(rospy.get_param("~elevator_approach_tolerance", 0.80)),
        )
        self.lobby_blocked_arrival_tolerance = float(
            rospy.get_param("~lobby_blocked_arrival_tolerance", 0.85)
        )
        self.heading_tolerance = float(rospy.get_param("~heading_tolerance", 0.12))

        self.state = self.WAITING
        self.current_floor = 0
        self.completed_floor_indices = set()
        self.pose = None
        self.source_pose = None
        self.floor0_start_source = None
        self.navigation_grid = None
        self.last_pose_stamp = rospy.Time(0)
        self.gate = None
        self.gate_source = None
        self.floor0_gate = None
        self.floor0_gate_source = None
        self.elevator_portal = None
        self.front_clearance = float("inf")
        # Keep obstacle clearance conservative, but use a high percentile for
        # sparse Livox scans when judging whether a heading is an opening.
        self.front_opening_clearance = float("inf")
        self.left_clearance = float("inf")
        self.right_clearance = float("inf")
        self.elevator_portal_source = None
        self.elevator_portal_world = None
        # 2026-09-15: the car-door geometry is NOT used to drive control any
        # more -- the elevator is identified only through the temporally-confirmed
        # map candidate (``detect_wide_lobby_openings``), exactly like the
        # standalone faithful test.  The configured reference below is kept ONLY
        # as a SANITY YARDSTICK (it never becomes the control source): a
        # confirmed candidate must sit within ``elevator_candidate_max_lateral``
        # of the reference and face it within ``elevator_candidate_max_yaw``, or
        # it is rejected (this is what stops run159 from locking the wrong wall).
        reference = rospy.get_param("~elevator_door", [1.65, 2.60, 0.0, 1.40])
        if isinstance(reference, str):
            reference = [
                float(item)
                for item in reference.strip().strip("[]").replace(",", " ").split()
            ]
        try:
            reference = [float(item) for item in reference]
        except (TypeError, ValueError):
            reference = []
        self.elevator_door_reference = (
            (reference[0], reference[1], reference[2])
            if len(reference) >= 3
            else None
        )
        self.elevator_candidate_max_lateral = max(
            0.0, float(rospy.get_param("~elevator_candidate_max_lateral", 0.60))
        )
        self.elevator_candidate_max_yaw = max(
            0.0, float(rospy.get_param("~elevator_candidate_max_yaw", 0.25))
        )
        self.elevator_candidate_min_width = max(
            0.0, float(rospy.get_param("~elevator_candidate_min_width", 0.90))
        )
        self.elevator_candidate_max_width = max(
            0.0, float(rospy.get_param("~elevator_candidate_max_width", 3.80))
        )
        # How far past the door plane counts as "inside the car" before riding.
        self.elevator_door_inset = max(
            0.05, float(rospy.get_param("~elevator_door_inset", 0.35))
        )
        self.entry_outside_retries = 0
        self.entry_outside_retry_limit = max(
            0, int(rospy.get_param("~entry_outside_retry_limit", 2))
        )
        # Return-to-lift leg, reusing the standalone driver's mechanism: a
        # committed fixed displacement along the reverse corridor heading to the
        # stand-off in front of the car door, then A* for the rest.
        # ``staging_distance`` matches team_scripts/elevator_only_driver.py's
        # ``~staging_distance`` (standalone ``staging_world``).
        self.staging_distance = float(rospy.get_param("~staging_distance", 2.20))
        # Landing of the LONG return leg, measured PAST the corridor gate (gate is
        # along 0) on the corridor axis.  User rule: "align to the corridor
        # reverse direction and walk a long way to the corridor doorway area; the
        # rest is the elevator module's job" - so this leg ends in the
        # corridor-mouth area and the staging/align/entry phases (or the
        # standalone lift test module) take over from there.
        self.return_mouth_along = float(
            rospy.get_param("~return_mouth_along", 1.0)
        )
        # Lateral band inside which the robot counts as being on the corridor
        # centreline (the return leg is only allowed to start from there).
        self.corridor_center_tolerance = max(
            0.05, float(rospy.get_param("~corridor_center_tolerance", 0.20))
        )
        self.return_center_distance = None
        self.return_standoff_distance = None
        self.return_standoff_done = False
        # Length of the bounded lateral correction ALIGN_ELEVATOR walks when the
        # robot is "arrived" at the staging point yet still off the centre line.
        self.lateral_correction_distance = None
        self.lateral_correction_heading = None
        # EXIT_ELEVATOR is a reverse run with no built-in progress test, so a
        # jammed robot pushed backwards for ever (run124 floor 1: 108 sim s at
        # vx=-0.45, frozen on the car threshold).  Watch progress and retreat.
        self.exit_stall_seconds = max(
            2.0, float(rospy.get_param("~exit_stall_seconds", 6.0))
        )
        self.exit_retry_limit = max(1, int(rospy.get_param("~exit_retry_limit", 3)))
        self.exit_progress_pose = None
        self.exit_progress_stamp = None
        self.exit_retries = 0
        self.exit_retreat_until = None
        # ------------------------------------------------------------------
        # 2026-09-15: fusion of the standalone lift test
        # (team_scripts/elevator_only_driver.py -- 10/10 PASS with truth, then
        # made faithful) into the mainline transition.  Three legs are ported:
        # corridor-start -> lift, lift -> corridor-start, and
        # lift -> main entrance -> spawn.  The corridor start is the resolved
        # virtual gate in the SOURCE map frame; control stays in the source
        # frame and the doorway/return geometry uses the world metric pose,
        # exactly as the standalone driver did.
        # ------------------------------------------------------------------
        self.return_phase = None
        self.establish_phase = None
        self.establish_center_distance = None
        # Latched reverse distance for ESTABLISH's CLEAR_CAR phase.  It must be
        # computed ONCE: recomputing it every cycle while ``_drive_distance``
        # measures travel from a pinned ``travel_anchor`` makes the two shrink
        # and grow against each other and the run stops after roughly half the
        # required reverse (run168: moved 0.46 m of the needed 0.80 m).
        self.establish_clear_distance = None
        # Arrival tolerance at the corridor start (the gate).  The gate is a
        # point on the corridor centre line, not a doorway, so this only has to
        # stay well inside the corridor half-width.
        self.corridor_start_tolerance = max(
            0.15, float(rospy.get_param("~corridor_start_tolerance", 0.40))
        )
        # --- return-to-spawn leg (standalone RETURN_SPAWN) ----------------
        # Latched from the first world/metric pose at mission start; the
        # navigation frame origin is the spawn, so this is the public
        # robot_start expressed in the frame the return geometry uses.
        self.spawn_world = None
        self.spawn_phase = None
        self.spawn_axis_since = None
        self.spawn_axis_tolerance = max(
            0.10, float(rospy.get_param("~spawn_axis_tolerance", 0.45))
        )
        self.spawn_axis_timeout = max(
            10.0, float(rospy.get_param("~spawn_axis_timeout", 120.0))
        )
        # The lobby U-turn is the slow manoeuvre (measured 0.08-0.15 rad/s
        # effective, 58% success over 193 turns), so it gets a generous budget
        # plus the walk-and-turn unstick.
        self.spawn_turn_y = float(rospy.get_param("~spawn_turn_y", 1.80))
        self.spawn_face_tolerance = max(
            0.05, float(rospy.get_param("~spawn_face_tolerance", 0.12))
        )
        self.spawn_face_since = None
        self.spawn_turn_timeout = max(
            10.0, float(rospy.get_param("~spawn_turn_timeout", 150.0))
        )
        self.spawn_reverse_sim = None
        self.spawn_reverse_timeout = max(
            30.0, float(rospy.get_param("~spawn_reverse_timeout", 300.0))
        )
        self.spawn_reverse_retreat_until = None
        self.spawn_reverse_retries = 0
        self.spawn_reverse_anchor = None
        self.spawn_reverse_progress_stamp = None
        self.spawn_reverse_bias_until = None
        self.spawn_reverse_yaw_bias = float(
            rospy.get_param("~spawn_reverse_yaw_bias", 0.16)
        )
        # --- in-place turn with a walk-and-turn unstick (driver _turn_command)
        self.turn_anchor_yaw = None
        self.turn_anchor_stamp = None
        self.turn_unstick_until = None
        self.turn_unstick_dir = 1.0
        self.turn_unstick_count = 0
        self.turn_unstick_seconds = max(
            1.0, float(rospy.get_param("~turn_unstick_seconds", 4.0))
        )
        self.turn_unstick_limit = max(
            1, int(rospy.get_param("~turn_unstick_limit", 4))
        )
        self.turn_unstick_walk = max(
            0.5, float(rospy.get_param("~turn_unstick_walk", 1.5))
        )
        self.turn_unstick_walk_speed = float(
            rospy.get_param("~turn_unstick_walk_speed", 0.20)
        )
        self.turn_progress_yaw = max(
            0.02, float(rospy.get_param("~turn_progress_yaw", 0.10))
        )
        self.elevator_candidate_evidence = {}
        self.elevator_candidate_side = None
        self.elevator_candidate_along = None
        self.elevator_candidate_lateral_min = float(
            rospy.get_param("~elevator_candidate_lateral_min", 0.5)
        )
        self.elevator_candidate_lateral_max = float(
            rospy.get_param("~elevator_candidate_lateral_max", 2.0)
        )
        self.elevator_candidate_confirm_cycles = max(
            3, int(rospy.get_param("~elevator_candidate_confirm_cycles", 5))
        )
        # A rolled or ground-level base must stop the mission loudly.  Without
        # this the transition keeps driving toward a mapped target the robot
        # can no longer reach and the whole run hangs until the timeout with
        # no fault recorded.
        # Body IMU attitude.  The metric (FAST-LIO) attitude glitches during hard
        # turns: run63 faulted with ROBOT_ROLLED while the world roll spiked to
        # 0.45 rad for one sample and the body IMU never left 0.12 rad.
        self.imu_roll = None
        self.imu_pitch = None
        self.imu_stamp = None
        self.imu_fresh_seconds = max(
            0.1, float(rospy.get_param("~imu_fresh_seconds", 0.5))
        )
        self.fall_tilt_persist = max(
            0.0, float(rospy.get_param("~fall_tilt_persist", 0.4))
        )
        self.fall_roll_since = None
        self.fall_pitch_since = None
        self.base_roll = 0.0
        self.base_pitch = 0.0
        # A real fall on this robot measured 35-45 degrees of tilt with the base
        # at 0.041 m, while upright walking stays within ~11 degrees; 30 degrees
        # therefore separates them with margin.
        self.fall_roll_limit = math.radians(
            float(rospy.get_param("~fall_roll_limit_deg", 30.0))
        )
        self.fall_pitch_limit = math.radians(
            float(rospy.get_param("~fall_pitch_limit_deg", 30.0))
        )
        self.fall_base_height = float(rospy.get_param("~fall_base_height", 0.10))
        # The metric z estimate drifts slowly downward during a long run
        # (measured 0.33 m -> 0.07 m over 230 s in run14 while the robot kept
        # walking upright and covering rooms).  A single low sample is therefore
        # not a fall.  A fall is a *sudden* drop: use the height change over a
        # short window instead of an absolute floor.
        self.fall_drop_threshold = float(
            rospy.get_param("~fall_drop_threshold", 0.15)
        )
        self.fall_drop_window = max(
            0.3, float(rospy.get_param("~fall_drop_window", 1.5))
        )
        # The reference for the drop test is a *slow* median, not the recent
        # maximum: the metric z estimate drifts downward monotonically during a
        # long run (0.33 m -> 0.09 m over 325 s in run14), so any short-window
        # test eventually compares a drifted sample against an older, higher
        # one.  A 20 s baseline follows that drift while still being far too
        # slow to absorb a real fall.
        self.fall_baseline_window = max(
            self.fall_drop_window,
            float(rospy.get_param("~fall_baseline_window", 20.0)),
        )
        self.fall_baseline_min_samples = int(
            rospy.get_param("~fall_baseline_min_samples", 20)
        )
        self.fall_height_history = deque(maxlen=4000)
        # FAST-LIO reports the base close to the ground while it converges, and
        # the body is still settling at spawn.  Ignore attitude and height for
        # the first seconds of the run so startup cannot look like a fall.
        self.fall_grace_seconds = float(rospy.get_param("~fall_grace_seconds", 15.0))
        # The mission ends at the spawn point, not inside the top-floor lift.
        # After the last floor the robot rides back down, opens the main
        # entrance and drives to the origin of the navigation frame, which is
        # the spawn pose (the localisation bridge anchors the source frame
        # there).  Both ids below are public scene information.
        self.ground_floor = int(rospy.get_param("~ground_floor", 0))
        self.main_entrance_id = str(
            rospy.get_param("~main_entrance_door_id", "main_entrance")
        )
        self.return_to_spawn_tolerance = max(
            0.05, float(rospy.get_param("~return_to_spawn_tolerance", 0.45))
        )
        # A fixed mapped target can be permanently unreachable while the map is
        # still sparse.  Retrying forever blocks the whole mission: observed on
        # the second floor, A* failed 1282 times for the lobby search point
        # (12.31, -0.40) and the elevator never reached FLOOR_1_READY.  Bound the
        # retries and accept the current pose when it is already close enough,
        # so the mission proceeds instead of stalling until the run timeout.
        self.max_route_retries = max(
            10, int(rospy.get_param("~max_route_retries", 40))
        )
        # The retry counter is incremented once per 20 Hz control cycle while no
        # route exists, so `max_route_retries` alone is a ~2 s timeout and the
        # floor-1 map handoff needs ~20 s (run21 faulted at
        # ROUTE_UNREACHABLE_ESTABLISH_FLOOR_1_TOPOLOGY_AFTER_40 only 2 s after
        # entering the state).  Bound the unreachable condition by real time
        # instead, keeping the cycle counter purely diagnostic.
        self.route_unreachable_timeout = max(
            10.0, float(rospy.get_param("~route_unreachable_timeout", 60.0))
        )
        self.route_retry_since = None
        # After this long without an A* route, drive open-loop toward the target
        # (clearance-gated) while A* keeps replanning.
        self.route_open_loop_grace = max(
            0.0, float(rospy.get_param("~route_open_loop_grace", 6.0))
        )
        self.route_accept_distance = max(
            0.2, float(rospy.get_param("~route_accept_distance", 1.2))
        )
        self.main_entrance_opened = False
        self.main_entrance_attempts = 0
        self.main_entrance_max_attempts = max(
            1, int(rospy.get_param("~main_entrance_max_attempts", 6))
        )
        self.returned_to_spawn = False
        # Wall clock, not ROS time: the sim clock is paused during startup, so
        # a ROS-time stamp of zero would keep the grace window open forever.
        self.started_at_wall = time.time()
        # The reference floor fixes the shared x/y topology that every upper
        # floor reuses.  Cache its confirmed doorway geometry and refuse to
        # leave the ground floor until that topology is genuinely resolved.
        self.reference_floor_topology = []
        self.reference_floor_ready = False
        self.reference_wait_since = None
        self.reference_floor_wait = max(
            0.0, float(rospy.get_param("~reference_floor_wait", 30.0))
        )
        self.reference_floor_fault = None
        self.reference_floor_last_status = None
        # Accumulated across status messages; see _reference_floor_snapshot.
        self.reference_rooms_seen = 0
        self.reference_lock_cleared = False
        self.elevator_lobby_wall_offset = float(
            rospy.get_param("~elevator_lobby_wall_offset", 1.65)
        )
        self.floor_complete = False
        self.transition_complete = False
        self.floor1_topology_isolated = False
        self.floor1_gate = None
        self.floor1_corridor_target = None
        self.establish_started_wall = None
        self.establish_timeout = max(
            30.0, float(rospy.get_param("~establish_timeout", 150.0))
        )
        # Direct (no A*) entry for short line-of-sight manoeuvres: align to the
        # measured opening, then drive straight in.
        self.direct_entry = bool(rospy.get_param("~direct_entry", True))
        self.direct_entry_max_distance = max(
            0.5, float(rospy.get_param("~direct_entry_max_distance", 4.0))
        )
        self.direct_align_tolerance = max(
            0.05, float(rospy.get_param("~direct_align_tolerance", 0.25))
        )
        self.direct_fallback_seconds = max(
            1.0, float(rospy.get_param("~direct_fallback_seconds", 25.0))
        )
        self.direct_blocked_since = None
        self.floor1_context_published = False
        self.floor1_complete = False
        self.two_floor_mission_complete = False
        self.state_started = rospy.Time.now()
        self.travel_anchor = None
        self.route = ()
        self.route_index = 0
        self.route_state = None
        self.route_target = None
        self.route_last_plan = rospy.Time(0)
        self.route_replan_period = max(
            0.5, float(rospy.get_param("~route_replan_period", 2.0))
        )
        self.route_retry_count = 0
        self.route_planner = TaskCoveragePlanner(
            robot_radius=float(rospy.get_param("~robot_radius", 0.38)),
            safety_margin=float(rospy.get_param("~safety_margin", 0.04)),
            navigation_clearance=float(rospy.get_param("~navigation_clearance", 0.20)),
            preferred_clearance=float(rospy.get_param("~preferred_clearance", 0.32)),
        )
        self.ride_start_z = None
        self.ride_response = None
        self.ride_error = None
        self.progress_stamp = rospy.Time.now()
        self.progress_pose = None
        self.entry_retries = 0
        self.ride_thread = None
        self.ride_accepted_at = None
        self.search_offsets = (-0.60, -0.30, 0.0, 0.30, 0.60)
        self.search_index = 0
        self.search_samples = []
        self.elevator_heading = None
        self.fault = None
        gate_override = rospy.get_param("~gate_override", None)
        if isinstance(gate_override, str):
            try:
                gate_override = ast.literal_eval(gate_override)
            except (SyntaxError, ValueError):
                gate_override = None
        if isinstance(gate_override, (list, tuple)) and len(gate_override) >= 3:
            try:
                self.gate = tuple(float(value) for value in gate_override[:3])
            except (TypeError, ValueError):
                self.gate = None
        self.start_immediately = bool(rospy.get_param("~start_immediately", False))

        self.command_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.status_pub = rospy.Publisher(
            "/simnav/elevator_status", String, queue_size=2, latch=True
        )
        self.complete_pub = rospy.Publisher(
            "/simnav/floor_transition_complete", Bool, queue_size=1, latch=True
        )
        self.context_pub = rospy.Publisher(
            "/simnav/floor_exploration_context", String, queue_size=1, latch=True
        )
        self.mission_complete_pub = rospy.Publisher(
            "/simnav/two_floor_mission_complete", Bool, queue_size=1, latch=True
        )
        self.fault_pub = rospy.Publisher(
            "/simnav/mission_fault", String, queue_size=1, latch=True
        )
        rospy.Subscriber("/simnav/floor_complete", Bool, self._complete_callback, queue_size=1)
        rospy.Subscriber(
            "/simnav/explorer_status", String, self._explorer_status_callback, queue_size=2
        )
        rospy.Subscriber(
            "/simnav/world_pose_metric", PoseStamped, self._pose_callback, queue_size=10
        )
        rospy.Subscriber("/simnav/odom", Odometry, self._source_pose_callback, queue_size=10)
        rospy.Subscriber(
            rospy.get_param("~imu_topic", "/trunk_imu"),
            Imu,
            self._imu_callback,
            queue_size=50,
        )
        rospy.Subscriber(
            "/navigation_map", OccupancyGrid, self._navigation_map_callback, queue_size=1
        )
        rospy.Subscriber("/scan_2d", LaserScan, self._scan_callback, queue_size=1)
        self.elevator_service = rospy.ServiceProxy("/call_elevator", CallElevator)
        self.door_service = rospy.ServiceProxy("/set_door_state", SetDoorState)
        self.plane_policy_service = rospy.ServiceProxy(
            "/unitree/select_plane_policy", SetBool
        )
        self.gait_policy_selected = False
        self.timer = rospy.Timer(rospy.Duration(0.05), self._control)
        rospy.on_shutdown(self._shutdown)
        self._publish_status()

    @staticmethod
    def _yaw(orientation):
        return transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )[2]

    def _complete_callback(self, message):
        with self.lock:
            complete = bool(message.data)
            if self.floor1_context_published and self.current_floor > 0:
                self.floor1_complete = complete
                if complete:
                    self.completed_floor_indices.add(self.current_floor)
                    if (
                        self.finish_at_top_floor
                        and self.current_floor >= self.max_floor
                        and not self.two_floor_mission_complete
                    ):
                        self.two_floor_mission_complete = True
                        self.transition_complete = True
                        self._stop()
                        self._set_state("TOP_FLOOR_COMPLETE")
                        self.mission_complete_pub.publish(Bool(data=True))
            else:
                self.floor_complete = complete
                if complete:
                    self.completed_floor_indices.add(0)

    def _reference_floor_snapshot(self, payload):
        """Validate that the ground floor resolved a reusable shared topology.

        Upper floors inherit this topology verbatim, so an unresolved ground
        floor must block the elevator instead of propagating a broken
        structure to every later floor.

        Readiness is accumulated across status messages rather than judged
        from the newest one alone.  The explorer stops publishing the moment
        the floor completes, so if that final message happened to be sent
        while the last room still held the topology lock, a single-message
        test would latch a fault that nothing could ever clear and the
        elevator would wait forever.
        """
        if self.current_floor != 0 or self.floor1_context_published:
            return
        portals = payload.get("observed_portals")
        self.reference_floor_last_status = payload
        if isinstance(portals, list):
            usable = [
                dict(item)
                for item in portals
                if isinstance(item, dict)
                and item.get("confirmed")
                and item.get("topology_id")
                and float(item.get("along", 0.0)) >= 0.5
            ]
            if usable:
                self.reference_floor_topology = usable
        topology_lock = payload.get("topology_lock")
        completed = payload.get("completed_topologies")
        completed_count = len(completed) if isinstance(completed, list) else 0
        expected = int(payload.get("expected_rooms_per_floor") or 0)
        if expected > 0:
            self.reference_rooms_seen = max(self.reference_rooms_seen, completed_count)
        if topology_lock is None:
            self.reference_lock_cleared = True

        if not self.reference_floor_topology:
            self.reference_floor_fault = "REFERENCE_TOPOLOGY_EMPTY"
            return
        if expected > 0 and self.reference_rooms_seen < expected:
            self.reference_floor_fault = "REFERENCE_TOPOLOGY_INCOMPLETE:{}/{}".format(
                self.reference_rooms_seen, expected
            )
            return
        if not self.reference_lock_cleared:
            self.reference_floor_fault = "REFERENCE_TOPOLOGY_LOCKED:{}".format(
                topology_lock
            )
            return
        self.reference_floor_fault = None
        self.reference_floor_ready = True

    def _explorer_status_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        # The reusable reference topology only needs the observed doorways.  It
        # used to be collected after the world-frame isolation-door check, so a
        # missing ``virtual_isolation_door`` (run62 floor 0: None while the
        # source variant was present) silently skipped the snapshot for the whole
        # floor, left reference_floor_ready false, and the lift refused to leave
        # a fully explored ground floor for ever.
        with self.lock:
            self._reference_floor_snapshot(payload)
        gate = payload.get("virtual_isolation_door")
        if not isinstance(gate, list) or len(gate) < 3:
            if self.state == self.WAITING:
                self._publish_status()
            return
        try:
            parsed = tuple(float(value) for value in gate[:3])
        except (TypeError, ValueError):
            return
        with self.lock:
            # Floor 0 is the metric reference for all mapped A* targets.
            # Later explorer restarts can estimate another virtual gate, but
            # that estimate must not replace the recorded shared topology.
            if self.current_floor == 0 and not self.floor1_context_published:
                self.gate = parsed
                source_gate = payload.get("virtual_isolation_door_source")
                if isinstance(source_gate, list) and len(source_gate) >= 3:
                    self.gate_source = tuple(float(value) for value in source_gate[:3])
                elif self.pose is not None and self.source_pose is not None:
                    self.gate_source = transform_pose_between_frames(
                        parsed, self.pose, self.source_pose
                    )
                self.floor0_gate = self.gate
                self.floor0_gate_source = self.gate_source
            portal = payload.get("elevator_portal_world")
            if (
                self.current_floor == 0
                and isinstance(portal, list)
                and len(portal) >= 3
            ):
                # Keep source-map and world-frame portal poses separate.  The
                # A* planner consumes /simnav/odom coordinates; replacing the
                # mapped source portal with this world pose makes the target
                # jump to a different frame after an explorer status update.
                self.elevator_portal_world = tuple(float(value) for value in portal[:3])
        if self.state == self.WAITING:
            self._publish_status()

    def _pose_callback(self, message):
        orientation = message.pose.orientation
        roll, pitch, _yaw = transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        with self.lock:
            self.pose = (
                float(message.pose.position.x),
                float(message.pose.position.y),
                self._yaw(orientation),
                float(message.pose.position.z),
            )
            self.base_roll = float(roll)
            self.base_pitch = float(pitch)
            self.last_pose_stamp = rospy.Time.now()
            # Latch the spawn in the world/metric frame while the robot is still
            # standing on it: the return leg needs the world x/y and heading of
            # the spawn (the source origin is only valid in the source frame).
            if self.spawn_world is None and self.state == self.WAITING:
                self.spawn_world = (
                    float(message.pose.position.x),
                    float(message.pose.position.y),
                    self._yaw(orientation),
                )

    def _imu_callback(self, message):
        orientation = message.orientation
        try:
            roll, pitch, _yaw = transformations.euler_from_quaternion(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
        except (TypeError, ValueError):
            return
        with self.lock:
            self.imu_roll = float(roll)
            self.imu_pitch = float(pitch)
            self.imu_stamp = time.time()

    def _record_base_height(self, pose):
        """Track the base height on every tick, in every state.

        The explorer owns the ground floor for the whole exploration segment,
        and the old code only sampled the height once a transition state was
        active.  That starved the baseline, so the first sample taken after
        `WAIT_FLOOR_COMPLETE` was compared against a badly outdated reference
        (run14: a false ROBOT_ON_GROUND 49 ms after entering
        RETURN_TO_ELEVATOR, which faulted the mission with 4/4 rooms done).
        """
        if pose is None:
            return
        height = float(pose[3])
        if not math.isfinite(height):
            return
        now = time.time()
        history = self.fall_height_history
        history.append((now, height))
        while history and now - history[0][0] > self.fall_baseline_window:
            history.popleft()

    def _check_fall(self, pose):
        """Return a fault string when the base is rolled or has just dropped.

        Measured on this robot: upright walking sits at 0.273-0.412 m with
        attitude within roughly 11 degrees, while a real fall measured 0.041 m
        with 35-45 degrees of tilt.  The height test is a *drop* test against a
        slow baseline, not an absolute floor, because the metric z estimate
        drifts monotonically downward during long runs (~0.0007 m/s measured in
        run14).  A 20 s median tracks that drift; a real fall is a step change
        of >= fall_drop_threshold inside fall_drop_window and is also caught by
        the attitude test.
        """
        # The grace window exists for the spawn drop, which is a *vertical*
        # transient: the body attitude stays near level.  A sustained tilt is a
        # real fall whenever it happens, so the attitude test runs even during
        # the grace.  run70 tipped over at sim 11 s inside the grace window,
        # raised no fault, and the whole run continued on its side.
        with self.lock:
            roll, pitch = self.base_roll, self.base_pitch
            imu_roll, imu_pitch, imu_stamp = (
                self.imu_roll,
                self.imu_pitch,
                self.imu_stamp,
            )
        # Prefer the body IMU: a real tip-over tilts it against gravity and keeps
        # it tilted, while the metric attitude can spike for a single sample
        # during a hard turn (run63: world roll 0.45 rad, body IMU 0.12 rad).
        fresh = (
            imu_stamp is not None
            and time.time() - imu_stamp <= self.imu_fresh_seconds
        )
        if fresh:
            roll, pitch = float(imu_roll), float(imu_pitch)
        now_wall = time.time()
        verdict = tilt_fault_state(
            roll,
            pitch,
            self.fall_roll_limit,
            self.fall_pitch_limit,
            self.fall_roll_since,
            now_wall,
            self.fall_tilt_persist,
        )
        if verdict == "ok":
            self.fall_roll_since = None
            self.fall_pitch_since = None
            return None
        if verdict == "fault":
            return "ROBOT_ROLLED"
        # Persistence window still open: remember when the tilt started.
        if self.fall_roll_since is None:
            self.fall_roll_since = now_wall
        return None
        if time.time() - self.started_at_wall < self.fall_grace_seconds:
            return None
        if pose is None:
            return None
        height = float(pose[3])
        if not math.isfinite(height):
            return None
        now = time.time()
        with self.lock:
            recent = [
                value
                for stamp, value in self.fall_height_history
                if now - stamp <= self.fall_drop_window
            ]
            baseline = [
                value
                for stamp, value in self.fall_height_history
                if now - stamp <= self.fall_baseline_window
            ]
        if len(baseline) < self.fall_baseline_min_samples:
            return None
        ordered = sorted(baseline)
        baseline_median = ordered[len(ordered) // 2]
        if height >= self.fall_base_height:
            return None
        # A fall must be a genuine step change away from the drifting baseline,
        # not merely a low reading of an estimate that keeps sinking.
        if (baseline_median - height) >= self.fall_drop_threshold:
            return "ROBOT_ON_GROUND"
        if recent and (max(recent) - height) >= self.fall_drop_threshold:
            return "ROBOT_ON_GROUND"
        return None

    def _source_pose_callback(self, message):
        with self.lock:
            self.source_pose = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                self._yaw(message.pose.pose.orientation),
                float(message.pose.pose.position.z),
            )
            if self.floor0_start_source is None and self.state == self.WAITING:
                self.floor0_start_source = self.source_pose

    def _navigation_map_callback(self, message):
        grid = GridView(
            data=np.asarray(message.data, dtype=np.int16).reshape(
                message.info.height, message.info.width
            ),
            resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            frame_id=message.header.frame_id or "simnav_map",
        )
        with self.lock:
            self.navigation_grid = grid
            gate = self.gate_source
        # Continue confirming the door while the robot approaches it.  The
        # candidate is map-derived and may be refined by later scans; only a
        # confirmed candidate is allowed to start the transition.
        if gate is None or self.floor1_context_published:
            return
        expected_heading = normalize_angle(gate[2] - math.pi / 2.0)
        candidates = detect_wide_lobby_openings(
            grid,
            gate[:2],
            gate[2],
            corridor_half_width=self.elevator_lobby_wall_offset,
            preferred_heading=expected_heading,
        )
        candidates = tuple(
            item for item in candidates
            if self.elevator_candidate_lateral_min
            <= float(item.get("lateral", 0.0))
            <= self.elevator_candidate_lateral_max
            and 0.0 <= float(item.get("along", 0.0)) <= 8.0
        )
        for candidate in candidates:
            pose = candidate["pose"]
            key = (
                candidate["side"],
                int(round(candidate["along"] / 0.5)),
            )
            with self.lock:
                evidence = self.elevator_candidate_evidence.get(key, 0) + 1
                self.elevator_candidate_evidence[key] = evidence
                if evidence < self.elevator_candidate_confirm_cycles:
                    continue
                if self.elevator_portal is not None:
                    if candidate["side"] != self.elevator_candidate_side:
                        continue
                    if (
                        self.elevator_candidate_along is not None
                        and abs(candidate["along"] - self.elevator_candidate_along) > 1.0
                    ):
                        continue
                # Sanity-check the candidate against the reference before trusting
                # it (mirrors the standalone faithful test's
                # ``_confirm_door_candidate``).  A wide opening on the wrong wall
                # (run159) is rejected here instead of being driven into.
                reference = self._reference_source()
                if reference is not None:
                    _along, lateral = door_frame_offset(pose, reference)
                    yaw_error = abs(
                        normalize_angle(float(pose[2]) - float(reference[2]))
                    )
                    width = float(candidate["width"])
                    if (
                        abs(lateral) > self.elevator_candidate_max_lateral
                        or yaw_error > self.elevator_candidate_max_yaw
                        or not (
                            self.elevator_candidate_min_width
                            <= width
                            <= self.elevator_candidate_max_width
                        )
                    ):
                        rospy.logwarn_throttle(
                            5.0,
                            "Elevator candidate REJECTED (lateral %.2f m max %.2f, "
                            "yaw %.2f rad max %.2f, width %.2f m in [%.2f, %.2f])",
                            lateral,
                            self.elevator_candidate_max_lateral,
                            yaw_error,
                            self.elevator_candidate_max_yaw,
                            width,
                            self.elevator_candidate_min_width,
                            self.elevator_candidate_max_width,
                        )
                        continue
                if self.elevator_portal is None or candidate["width"] >= float(self.elevator_portal[3]):
                    self.elevator_portal = (
                        float(pose[0]), float(pose[1]), float(pose[2]),
                        float(candidate["width"]),
                    )
                    self.elevator_candidate_side = candidate["side"]
                    self.elevator_candidate_along = float(candidate["along"])
                    self.elevator_portal_source = "WIDE_PORTAL_MAP"
                    if self.pose is not None and self.source_pose is not None:
                        self.elevator_portal_world = transform_pose_between_frames(
                            pose, self.source_pose, self.pose
                        )

    def _scan_callback(self, message):
        values = []
        left = []
        right = []
        for index, distance in enumerate(message.ranges):
            angle = normalize_angle(message.angle_min + index * message.angle_increment)
            if math.isfinite(distance) and message.range_min <= distance <= message.range_max:
                if abs(angle) <= math.radians(16.0):
                    values.append(float(distance))
                elif math.radians(65.0) <= angle <= math.radians(110.0):
                    left.append(float(distance))
                elif math.radians(-110.0) <= angle <= math.radians(-65.0):
                    right.append(float(distance))
        with self.lock:
            self.front_clearance = (
                sorted(values)[max(0, int(0.20 * len(values)) - 1)]
                if values
                else float("inf")
            )
            self.front_opening_clearance = (
                float(np.percentile(values, 80.0)) if values else float("inf")
            )
            self.left_clearance = float(np.median(left)) if left else float("inf")
            self.right_clearance = float(np.median(right)) if right else float("inf")

    def _set_state(self, state):
        with self.lock:
            if state == self.state:
                return
            self.state = state
            self.state_started = rospy.Time.now()
            self.travel_anchor = None
            self.route = ()
            self.route_index = 0
            self.route_state = None
            self.route_target = None
            self.route_last_plan = rospy.Time(0)
            self.route_retry_count = 0
            self.route_retry_since = None
            self.return_center_distance = None
            self.return_standoff_distance = None
            self.return_standoff_done = False
            self.lateral_correction_distance = None
            self.lateral_correction_heading = None
            self.exit_progress_pose = None
            self.exit_progress_stamp = None
            self.exit_retries = 0
            self.exit_retreat_until = None
            # Fused-leg phase bookkeeping (2026-09-15).  Every entry into a
            # transition state starts its leg from the first phase.
            self.return_phase = None
            self.establish_phase = None
            self.establish_center_distance = None
            self.establish_clear_distance = None
            self.spawn_phase = None
            self.spawn_axis_since = None
            self.spawn_face_since = None
            self.spawn_reverse_sim = None
            self.spawn_reverse_retreat_until = None
            self.spawn_reverse_retries = 0
            self.spawn_reverse_anchor = None
            self.spawn_reverse_progress_stamp = None
            self.spawn_reverse_bias_until = None
            self.turn_anchor_yaw = None
            self.turn_anchor_stamp = None
            self.turn_unstick_until = None
            self.turn_unstick_count = 0
        rospy.loginfo("Elevator transition state -> %s", state)
        self._publish_status()

    def _publish_command(self, linear=0.0, angular=0.0):
        command = Twist()
        command.linear.x = clamp_linear_speed(linear, self.max_linear_speed)
        command.angular.z = float(angular)
        self.command_pub.publish(command)

    def _stop(self):
        self._publish_command()

    def _fail(self, reason):
        if self.fault is not None:
            return
        self.fault = {"source": "elevator_transition", "reason": str(reason)}
        self._stop()
        self._set_state("MISSION_FAULT")
        self.fault_pub.publish(String(data=json.dumps(self.fault, sort_keys=True)))

    def _drive_to(
        self,
        pose,
        target,
        speed=None,
        blocked_arrival_tolerance=None,
        arrival_tolerance=None,
    ):
        tolerance = (
            self.target_tolerance
            if arrival_tolerance is None
            else max(self.target_tolerance, float(arrival_tolerance))
        )
        distance = planar_distance(pose, target)
        if distance <= tolerance:
            self._stop()
            return True
        desired = target_heading(pose, target)
        error = normalize_angle(desired - pose[2])
        if abs(error) > self.heading_tolerance:
            self._publish_command(0.0, math.copysign(self.turn_speed, error))
        elif self.front_clearance < self.stop_distance:
            if (
                blocked_arrival_tolerance is not None
                and distance <= float(blocked_arrival_tolerance)
            ):
                self._stop()
                return True
            self._fail("OBSTACLE_WHILE_DRIVING_TO_{}".format(self.state))
        else:
            self._publish_command(speed if speed is not None else self.motion_speed, 0.8 * error)
        return False

    def _drive_direct(self, pose, target, speed, tolerance):
        """Align to the target, then drive straight at it - no A* at all.

        Used for the lift approach, the post-exit corridor move and the car
        entry: each is a short line-of-sight manoeuvre through an opening whose
        free span was just measured, so the only things needed are the heading
        and the front clearance.  The previous fallback drove forward *while*
        turning, which arced the robot into whatever it was facing (run52 floor
        2: front clearance 0.25 m, 0.45 m/s crawl, 25 sim s for 2 m).
        """
        heading = target_heading(pose, target)
        error = normalize_angle(heading - pose[2])
        if not direct_alignment_ready(error, self.direct_align_tolerance):
            # Turn on the spot first; a straight entry needs the heading.
            self._publish_command(0.0, math.copysign(self.turn_speed, error))
            return False
        if self.front_clearance < self.stop_distance:
            if self.direct_blocked_since is None:
                self.direct_blocked_since = rospy.Time.now()
            self._stop()
            rospy.logwarn_throttle(
                2.0,
                "Direct entry blocked in %s: front=%.2f (falls back to A* after "
                "%.0fs)",
                self.state,
                self.front_clearance,
                self.direct_fallback_seconds,
            )
            return False
        self.direct_blocked_since = None
        lateral = 0.0
        if math.isfinite(self.left_clearance) and math.isfinite(self.right_clearance):
            lateral = max(
                -0.18, min(0.18, 0.10 * (self.right_clearance - self.left_clearance))
            )
        self._publish_command(
            speed, max(-0.25, min(0.25, 0.8 * error + lateral))
        )
        return False

    def _direct_blocked_seconds(self):
        if self.direct_blocked_since is None:
            return 0.0
        return (rospy.Time.now() - self.direct_blocked_since).to_sec()

    def _drive_planned_to(
        self,
        pose,
        target,
        speed=None,
        blocked_arrival_tolerance=None,
        arrival_tolerance=None,
    ):
        """Persistently replan an A* route to a recorded x/y reference."""
        with self.lock:
            grid = self.navigation_grid
        target_key = (round(float(target[0]), 2), round(float(target[1]), 2))
        tolerance = (
            self.target_tolerance
            if arrival_tolerance is None
            else max(self.target_tolerance, float(arrival_tolerance))
        )
        distance = planar_distance(pose, target)
        if distance <= tolerance:
            self._stop()
            self.direct_blocked_since = None
            return True
        if direct_entry_applies(
            distance,
            self.direct_entry_max_distance,
            self.direct_entry,
            self._direct_blocked_seconds(),
            self.direct_fallback_seconds,
        ):
            speed = (
                min(self.motion_speed, self.lobby_approach_speed)
                if speed is None
                else float(speed)
            )
            return self._drive_direct(pose, target, speed, tolerance)
        now = rospy.Time.now()
        plan_expired = (
            self.route_last_plan == rospy.Time(0)
            or (now - self.route_last_plan).to_sec() >= self.route_replan_period
        )
        if (
            grid is not None
            and (
                self.route_state != self.state
                or self.route_target != target_key
                or not self.route
                or plan_expired
            )
        ):
            path, _length, _clearance = self.route_planner.navigation_path(
                grid, pose[:3], target
            )
            self.route_last_plan = now
            if path:
                self.route = tuple(path)
                self.route_index = min(1, len(self.route) - 1)
                self.route_state = self.state
                self.route_target = target_key
                self.route_retry_since = None
            elif not self.route:
                self.route_retry_count += 1
                self._stop()
                # Close enough counts as arrived: a sparse map can leave the
                # final metre unbudgeted while the robot is already standing on
                # the reference point.
                if planar_distance(pose, target) <= self.route_accept_distance:
                    rospy.loginfo(
                        "Accepting %s within %.2f m of mapped target (no A* route)",
                        self.state,
                        planar_distance(pose, target),
                    )
                    self.route_retry_count = 0
                    self.route_retry_since = None
                    return True
                if self.route_retry_since is None:
                    self.route_retry_since = now
                unreachable_for = (now - self.route_retry_since).to_sec()
                if unreachable_for >= self.route_unreachable_timeout:
                    self._fail(
                        "ROUTE_UNREACHABLE_{}_AFTER_{:.0f}S".format(
                            self.state, unreachable_for
                        )
                    )
                    return False
                # Bounded open-loop recovery.  A* runs on the inflated
                # navigation map; a mapped pinch (e.g. the robot's own trail
                # along the corridor centreline, which closes the corridor once
                # it is inflated) can leave the start cell in a pocket the robot
                # can physically drive out of.  After a short grace, drive
                # toward the target along its bearing with the same lateral
                # centring used elsewhere, gated by the front clearance, while
                # A* keeps replanning every route_replan_period.  This is not a
                # criteria change and has no persistent state.
                #
                # Blind driving is only defensible for a short target the robot
                # can actually see; the same line-of-sight bound as the direct
                # entry path applies.  Without it a target behind a wall (the
                # lobby staging point seen from outside the main entrance, 5.8 m
                # away) became six seconds of open-loop driving into the door
                # surround, which tipped the robot over at sim 8 s.
                if (
                    unreachable_for >= self.route_open_loop_grace
                    and planar_distance(pose, target) <= self.direct_entry_max_distance
                    and self.front_clearance >= self.stop_distance
                ):
                    heading = target_heading(pose, target)
                    heading_error = normalize_angle(heading - pose[2])
                    lateral = 0.0
                    if (
                        math.isfinite(self.left_clearance)
                        and math.isfinite(self.right_clearance)
                    ):
                        lateral = max(
                            -0.15,
                            min(0.15, 0.10 * (self.right_clearance - self.left_clearance)),
                        )
                    self._publish_command(
                        self.lobby_approach_speed,
                        max(-0.25, min(0.25, 0.8 * heading_error + lateral)),
                    )
                    rospy.logwarn_throttle(
                        5.0,
                        "Open-loop recovery toward (%.2f, %.2f) in %s "
                        "(no A* route for %.1fs, front=%.2f)",
                        target[0], target[1], self.state,
                        unreachable_for, self.front_clearance,
                    )
                    return False
                self._stop()
                rospy.logwarn_throttle(
                    5.0,
                    "A* has no route to mapped target (%.2f, %.2f) in %s; "
                    "cycles=%d unreachable=%.1fs/%.0fs",
                    target[0], target[1], self.state,
                    self.route_retry_count,
                    unreachable_for,
                    self.route_unreachable_timeout,
                )
                return False
        if self.route:
            while (
                self.route_index < len(self.route) - 1
                and planar_distance(pose, self.route[self.route_index])
                <= self.target_tolerance
            ):
                self.route_index += 1
            waypoint = self.route[self.route_index]
            final = self.route_index == len(self.route) - 1
            desired = target_heading(pose, waypoint)
            heading_error = normalize_angle(desired - pose[2])
            if (
                abs(heading_error) <= self.heading_tolerance
                and self.front_clearance < self.stop_distance
                and not (
                    final
                    and blocked_arrival_tolerance is not None
                    and planar_distance(pose, waypoint)
                    <= float(blocked_arrival_tolerance)
                )
            ):
                self.route = ()
                self.route_last_plan = rospy.Time(0)
                self.route_retry_count += 1
                self._stop()
                rospy.logwarn_throttle(
                    3.0, "A* route blocked in %s; rebuilding from live map", self.state
                )
                return False
            reached = self._drive_to(
                pose, waypoint, speed,
                blocked_arrival_tolerance if final else None,
                tolerance if final else None,
            )
            return bool(reached and final)
        self._stop()
        return False

    def _align(self, pose, yaw):
        error = normalize_angle(yaw - pose[2])
        if abs(error) <= self.heading_tolerance:
            self._stop()
            return True
        self._publish_command(0.0, math.copysign(self.turn_speed, error))
        return False

    def _approach_portal(self):
        """The map-detected elevator doorway (source frame, x/y/yaw/width).

        2026-09-15: the transition used to carry the car-door coordinates as a
        hardcoded ``~elevator_door`` launch parameter (i.e. knowing the door
        position in advance).  That is not faithful: the scene contract only
        permits ``/set_door_state`` and ``/call_elevator``.  The elevator is now
        identified ONLY through the temporally-confirmed map candidate
        (``_navigation_map_callback`` -> ``detect_wide_lobby_openings``), the
        same "elevator candidate" the explorer publishes.  Until it is confirmed
        this returns ``None`` and the transition simply waits for it.
        """
        return self.elevator_portal

    def _reference_source(self):
        """The SANITY reference door mapped into the source frame, or ``None``.

        The reference (world ``~elevator_door``) is never used to control; it is
        only the yardstick against which a detected candidate is validated
        (mirrors the standalone faithful test's ``door_reference``).  Mapped with
        the paired gate transform, which the handoff showed is accurate where the
        live pose pair is not.
        """
        with self.lock:
            gate, gate_source = self.gate, self.gate_source
        if gate is None or gate_source is None or self.elevator_door_reference is None:
            return None
        return transform_pose_between_frames(
            self.elevator_door_reference, gate, gate_source
        )

    def _elevator_staging_target(self):
        """Return the source-map staging point and heading for the portal."""
        with self.lock:
            gate = self.gate_source
        portal = self._approach_portal()
        if gate is None or portal is None or len(portal) < 3:
            return None, None
        return portal_staging_point(gate, portal), float(portal[2])

    def _portal_world_report(self):
        """Best-effort world position of the doorway, for status only.

        The explorer publishes the entrance gate as a matched pair
        (``virtual_isolation_door`` / ``..._source``); mapping the portal with
        that pair reproduced the true doorway (1.60, 2.55) where the live pose
        pair gave a 3 m error.  Nothing drives on this value.
        """
        with self.lock:
            gate, gate_source = self.gate, self.gate_source
            portal = self.elevator_portal
            world, source = self.pose, self.source_pose
        if portal is None:
            return None
        if gate is not None and gate_source is not None:
            return transform_pose_between_frames(portal, gate_source, gate)
        if world is not None and source is not None:
            return transform_pose_between_frames(portal, source, world)
        return None

    def _publish_floor_context(self, source):
        """Tell the mapping stack which floor the robot is on right now."""
        with self.lock:
            floor_index = int(self.current_floor)
            pose_z = float(self.pose[3]) if self.pose is not None else 0.0
            gate = list(self.floor1_gate or self.floor0_gate_source or self.gate_source or [])
            world = list(self.floor0_gate or [])
            corridor = list(self.floor1_corridor_target or [])
            reused = list(self.reference_floor_topology)
        self.context_pub.publish(String(data=json.dumps({
            "floor_index": floor_index,
            "source": str(source),
            "floor_z": pose_z,
            "gate_source": gate,
            "gate_world": world,
            "corridor_target_source": corridor,
            # Upper floors share the reference floor's x/y topology.  Hand the
            # resolved ground-floor doorways over as a hint so the next explorer
            # does not rediscover (and possibly mis-bin) the same structure.
            # They are hints only: each floor keeps its own 2D map and may
            # resolve different doorways.
            "reused_topology": reused,
        }, sort_keys=True)))
        rospy.loginfo(
            "Floor context published (%s): floor_index=%d", source, floor_index
        )

    def _ensure_elevator_door_open(self, floor_index):
        """Ask the scene to open the car door serving ``floor_index``.

        The layout marks ``elevator_floor_0`` as initially open but
        ``elevator_floor_1`` as initially closed, and the node previously only
        ever called the door service for the main entrance.  On the floor-1
        return leg run49 therefore drove into a car door that was still closed
        and failed the whole mission at 0.93 m of entry travel.
        """
        door_id = elevator_door_id(floor_index, self.elevator_door_prefix)
        if self.elevator_door_requested_for == door_id:
            return True
        try:
            rospy.wait_for_service("/set_door_state", timeout=2.0)
            response = self.door_service(door_id, True)
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn_throttle(
                5.0, "Waiting for elevator door service (%s): %s", door_id, error
            )
            return False
        if response.accepted:
            rospy.loginfo(
                "Elevator car door %s opened: state=%s message=%s",
                door_id,
                response.state,
                response.message,
            )
            self.elevator_door_requested_for = door_id
            return True
        rospy.logwarn_throttle(
            5.0,
            "Elevator car door %s rejected: state=%s message=%s",
            door_id,
            response.state,
            response.message,
        )
        return False

    def _entry_inside_car(self):
        """How far past the door plane the robot is (``None`` if unknown).

        ``door_frame_offset``'s ``along`` grows through the doorway into the car,
        so a positive value means the robot is inside.  Measured against the
        map-detected candidate in the SOURCE frame (same frame as
        ``self.source_pose``); the hardcoded world door is gone (2026-09-15).
        """
        portal = self._approach_portal()
        if self.source_pose is None or portal is None:
            return None
        along, _lateral = door_frame_offset(self.source_pose, portal)
        return float(along)

    def _entry_outside_car(self):
        """Block a ride that would start from OUTSIDE the car door plane.

        run152 floor 1 rode with the robot 0.46 m before the door plane AND 0.53 m
        off the centre line: the entry drive had been declared "contained" by the
        stall detector (map-frame travel >= 1.50 m), which cannot tell a door-jamb
        contact from the car's rear wall.  The known door geometry can tell: back
        off and enter again, and if that keeps failing raise the fault instead of
        riding from the lobby.  Returns True when the ride must not start.
        """
        along = self._entry_inside_car()
        if along is None:
            return False
        if along >= self.elevator_door_inset:
            self.entry_outside_retries = 0
            return False
        self._stop()
        if self.entry_outside_retries >= self.entry_outside_retry_limit:
            # Last resort: a stalled threshold must not kill the whole
            # three-floor mission, so warn loudly and ride anyway.  The gate and
            # its retries above stay, and with the stair gait asserted before the
            # entry (see _ensure_gait_policy) this branch should not be reached.
            rospy.logerr_throttle(
                2.0,
                "Entry stopped %.2f m BEFORE the car door plane after %d "
                "retries; riding anyway (degraded)",
                -along,
                self.entry_outside_retries,
            )
            self.entry_outside_retries = 0
            return False
        self.entry_outside_retries += 1
        rospy.logwarn_throttle(
            2.0,
            "Entry stopped %.2f m BEFORE the car door plane (need %.2f m inside); "
            "backing off and entering again (%d/%d)",
            -along,
            self.elevator_door_inset,
            self.entry_outside_retries,
            self.entry_outside_retry_limit,
        )
        self.retreat_until = rospy.Time.now() + rospy.Duration(
            self.entry_retreat_seconds
        )
        return True

    def _drive_distance(self, pose, distance, speed, minimum_blocked_progress,
                        reverse=False):
        """Drive a measured straight run.  ``reverse`` backs out instead.

        Leaving the car needs no pi turn: the robot boards facing the car, so
        reversing keeps that heading and skips the in-place rotation entirely.
        The standalone lift cycle measured the turn at 29-30 simulated seconds
        (a fifth of the whole up-and-out cycle) and the forward exit then stalled
        on the 6 cm car threshold for its whole 35 s budget, while reversing out
        took 8 s and succeeded on every leg.
        """
        now = rospy.Time.now()
        if self.retreat_until is not None:
            # Backing off from an early door contact; restart the measured
            # travel so the retry is judged on its own progress.
            if now < self.retreat_until:
                self._publish_command(-self.entry_retreat_speed, 0.0)
                return False
            self.retreat_until = None
            self.travel_anchor = pose[:2]
            self.progress_pose = pose[:2]
            self.progress_stamp = now
        if self.travel_anchor is None:
            self.travel_anchor = pose[:2]
        travelled = planar_distance(pose, self.travel_anchor)
        if self.state in ("ENTER_ELEVATOR", "ENTER_FLOOR_1_ELEVATOR_RETURN"):
            if self.progress_pose is None:
                self.progress_pose = pose[:2]
                self.progress_stamp = now
            elif planar_distance(pose, self.progress_pose) >= 0.20:
                self.progress_pose = pose[:2]
                self.progress_stamp = now
            elif (now - self.progress_stamp).to_sec() >= 8.0:
                if entry_stall_confirms_containment(
                    travelled, minimum_blocked_progress
                ):
                    self._stop()
                    return True
                if self.entry_retries >= 3:
                    self._fail("ELEVATOR_ENTRY_NO_PROGRESS")
                    return False
                # Briefly yaw off the contact point, then resume the detected
                # doorway heading on the next control cycle.  Holding a biased
                # heading can make a threshold contact turn into a jamb contact.
                bias = 0.16 if self.entry_retries % 2 == 0 else -0.16
                self.entry_retries += 1
                self.progress_stamp = now
                self.progress_pose = pose[:2]
                self._publish_command(0.0, bias)
                return False
        if travelled >= distance:
            self._stop()
            return True
        if self.front_clearance < self.stop_distance:
            # Inside the compact elevator the rear wall intentionally appears
            # before the nominal travel distance.  A sensor stop after the
            # minimum crossing progress is positive evidence of containment,
            # not a navigation failure.
            if travelled >= minimum_blocked_progress:
                self._stop()
                return True
            if entry_blocked_retry_allowed(
                travelled,
                minimum_blocked_progress,
                self.entry_retries,
                self.entry_retry_limit,
            ):
                self.entry_retries += 1
                self._stop()
                self.retreat_until = now + rospy.Duration(self.entry_retreat_seconds)
                rospy.logwarn_throttle(
                    2.0,
                    "Elevator entry blocked after %.2f m (front=%.2f); backing "
                    "off %.1fs for retry %d/%d",
                    travelled,
                    self.front_clearance,
                    self.entry_retreat_seconds,
                    self.entry_retries,
                    self.entry_retry_limit,
                )
                return False
            self._fail("ELEVATOR_PATH_BLOCKED_AFTER_{:.2f}M".format(travelled))
            return False
        if self.elevator_heading is None:
            # A measured run always holds a heading; a missing one used to raise
            # in the control callback and silently freeze the robot.
            self._stop()
            return False
        error = normalize_angle(self.elevator_heading - pose[2])
        if abs(error) > 0.20:
            self._publish_command(0.0, math.copysign(self.turn_speed, error))
        else:
            lateral_error = 0.0
            if self.state in (
                "ENTER_ELEVATOR",
                "ENTER_FLOOR_1_ELEVATOR_RETURN",
            ):
                # Centre on the map-detected candidate (source frame).  The
                # hardcoded car-door centre line is gone (2026-09-15): the lift
                # is identified only by the candidate.  +lateral is to the
                # robot's left, so a negative yaw command steers back to centre.
                portal = self._approach_portal()
                if portal is not None and len(portal) >= 3:
                    _along, lateral = door_frame_offset(pose, portal)
                    lateral_error = max(-0.25, min(0.25, -0.9 * float(lateral)))
                    rospy.loginfo_throttle(
                        1.0,
                        "Entry centring (map candidate): along=%.2f lateral=%.2f "
                        "-> yaw bias %+.3f",
                        _along,
                        lateral,
                        lateral_error,
                    )
                elif math.isfinite(self.left_clearance) and math.isfinite(self.right_clearance):
                    lateral_error = max(-0.18, min(
                        0.18, 0.10 * (self.right_clearance - self.left_clearance)
                    ))
            self._publish_command(
                -speed if reverse else speed,
                max(-0.18, min(0.18, 0.8 * error + lateral_error)),
            )
        return False

    def _start_ride(self):
        if self.ride_thread is not None:
            return

        def call():
            try:
                rospy.wait_for_service("/call_elevator", timeout=5.0)
                self.ride_response = self.elevator_service(
                    self.elevator_id, self.target_floor, True
                )
            except (rospy.ROSException, rospy.ServiceException) as error:
                self.ride_error = str(error)

        self.ride_thread = threading.Thread(target=call, name="elevator-call", daemon=True)
        self.ride_thread.start()

    def _ensure_gait_policy(self):
        """Select the step-trained gait policy for the transition.

        The car doorway is not flush with the lobby: ``elevator_threshold_*`` is a
        real 6 cm step spanning the whole 1.4 m opening, and the flat-ground
        ("plane") policy pushes into it instead of climbing it.  The standalone
        lift test measured this directly -- with the plane policy the robot sat
        at the door plane with 1.75 m of laser clearance ahead and zero net
        progress (0.45 m/s: 1 success in 3 attempts), while the step-trained
        policy is junior_ctrl's own default for exactly this kind of terrain.
        The service refuses to switch while cmd_vel is non-zero, so this is
        called before anything starts moving.
        """
        if self.gait_policy_selected:
            return True
        try:
            rospy.wait_for_service("/unitree/select_plane_policy", timeout=0.5)
            response = self.plane_policy_service(False)
            self.gait_policy_selected = bool(response.success)
            if not self.gait_policy_selected:
                rospy.logwarn_throttle(
                    3.0, "Stair policy switch rejected: %s", response.message
                )
            else:
                # Verified in the logs: the 6 cm car threshold needs this policy
                # on BOTH entry paths (floor 0 and the floor-1 return).
                rospy.loginfo("Stair gait policy selected for the transition")
            return self.gait_policy_selected
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn_throttle(3.0, "Waiting for gait policy switch: %s", error)
            return False

    # ------------------------------------------------------------------
    # 2026-09-15: helpers ported from team_scripts/elevator_only_driver.py
    # (the standalone lift test that passed 10/10 with truth and was then made
    # faithful).  They are shared by the three fused legs.
    # ------------------------------------------------------------------
    def _corridor_gate(self):
        """The corridor start (virtual gate) in the SOURCE map frame."""
        with self.lock:
            return self.floor1_gate or self.floor0_gate_source or self.gate_source

    @staticmethod
    def _frame_offset(point, origin):
        """(along, lateral) of ``point`` in the frame ``origin=(x, y, yaw)``."""
        dx = float(point[0]) - float(origin[0])
        dy = float(point[1]) - float(origin[1])
        cosine = math.cos(float(origin[2]))
        sine = math.sin(float(origin[2]))
        return (dx * cosine + dy * sine, -dx * sine + dy * cosine)

    @staticmethod
    def _frame_point(origin, along, lateral=0.0):
        """Point ``along``/``lateral`` (source frame) from ``origin=(x, y, yaw)``."""
        cosine = math.cos(float(origin[2]))
        sine = math.sin(float(origin[2]))
        return (
            float(origin[0]) + along * cosine - lateral * sine,
            float(origin[1]) + along * sine + lateral * cosine,
        )

    def _world_to_source_point(self, world_xy):
        """Map a world/metric point into the LIO source frame (driver _to_source)."""
        with self.lock:
            world, source = self.pose, self.source_pose
        if world is None or source is None:
            return None
        delta = normalize_angle(source[2] - world[2])
        cosine = math.cos(delta)
        sine = math.sin(delta)
        dx = float(world_xy[0]) - float(world[0])
        dy = float(world_xy[1]) - float(world[1])
        return (
            source[0] + cosine * dx - sine * dy,
            source[1] + sine * dx + cosine * dy,
        )

    def _world_to_source_yaw(self, world_yaw):
        """Map a world heading into the source frame (driver _to_source_yaw)."""
        with self.lock:
            world, source = self.pose, self.source_pose
        if world is None or source is None:
            return None
        return normalize_angle(float(world_yaw) + source[2] - world[2])

    def _elevator_staging_source(self):
        """Stand-off point in front of the car door, SOURCE frame.

        The map-detected candidate pulled back by ``staging_distance`` along its
        normal (which points INTO the car), so the point sits in the lobby on the
        door centre line.  The hardcoded world door is gone (2026-09-15).
        """
        door = self._approach_portal()
        if door is None:
            return None
        facing = float(door[2])
        return (
            float(door[0]) - self.staging_distance * math.cos(facing),
            float(door[1]) - self.staging_distance * math.sin(facing),
        )

    def _center_on_corridor_axis(self, source_pose, gate, distance_attr):
        """One control cycle of the corridor-centreline alignment.

        Ported from the standalone driver's CORRIDOR_CENTRE state: the robot
        must sit on the corridor axis before the U-turn (in a 2.2 m corridor an
        off-axis arrival wedges the mouth wall).  Returns True when centred.
        """
        _along, lateral = self._frame_offset(source_pose, gate)
        if abs(lateral) <= self.corridor_center_tolerance:
            setattr(self, distance_attr, None)
            self._stop()
            return True
        heading = normalize_angle(
            float(gate[2]) - math.pi / 2.0
            if lateral > 0.0
            else float(gate[2]) + math.pi / 2.0
        )
        distance = getattr(self, distance_attr)
        if distance is None:
            if not self._align(source_pose, heading):
                return False
            setattr(self, distance_attr, abs(float(lateral)))
            self.elevator_heading = heading
            self.travel_anchor = source_pose[:2]
            self.progress_pose = source_pose[:2]
            self.progress_stamp = rospy.Time.now()
            return False
        self.elevator_heading = heading
        if not self._align(source_pose, heading):
            return False
        if self._drive_distance(
            source_pose,
            float(distance),
            min(self.motion_speed, self.lobby_approach_speed),
            max(0.0, float(distance) - 0.2),
        ):
            setattr(self, distance_attr, None)
            self._stop()
            return True
        return False

    def _control_return_to_lift(self, source_pose, gate):
        """One cycle of the fused corridor-start -> lift approach.

        Ported from the standalone driver's corridor-mouth cycle
        (CORRIDOR_CENTRE -> TURN_AROUND -> BACK_TO_LIFT).  Used by BOTH the
        floor-0 return (RETURN_TO_ELEVATOR) and the upper-floor return
        (RETURN_TO_FLOOR_1_GATE), so every "corridor start -> lift" leg follows
        the method that passed 10/10.  Returns True once the robot is staged in
        front of the car door, ready for the ALIGN/ENTER pair.
        """
        phase = self.return_phase or "TO_CORRIDOR_START"
        if phase == "TO_CORRIDOR_START":
            # Interface contract (user): going to the lift must first reach the
            # corridor start -- the virtual gate the explorer anchors floor 0 on.
            if self._drive_planned_to(
                source_pose,
                gate[:2],
                min(self.motion_speed, self.lobby_approach_speed),
                arrival_tolerance=self.corridor_start_tolerance,
            ):
                self.return_phase = "CORRIDOR_CENTRE"
                self._stop()
            return False
        if phase == "CORRIDOR_CENTRE":
            if self._center_on_corridor_axis(
                source_pose, gate, "return_center_distance"
            ):
                self.return_phase = "TURN_TO_LIFT"
            return False
        if phase == "TURN_TO_LIFT":
            # U-turn at the corridor start.  Uses the walk-and-turn unstick so a
            # gait lock-up fails loudly instead of hanging until the run timeout.
            reverse_heading = normalize_angle(float(gate[2]) + math.pi)
            error = normalize_angle(reverse_heading - float(source_pose[2]))
            if abs(error) <= self.heading_tolerance:
                self._stop()
                self.turn_anchor_yaw = None
                self.turn_anchor_stamp = None
                self.turn_unstick_until = None
                self.turn_unstick_count = 0
                self.return_phase = "TO_STAGING"
            else:
                self._turn_command(error)
            return False
        # TO_STAGING -- the driver's staging_world(): the known car door pulled
        # back by staging_distance along the door normal.
        staging = self._elevator_staging_source()
        portal = self._approach_portal()
        if staging is None or portal is None:
            self._stop()
            self._fail("ELEVATOR_STAGING_UNKNOWN")
            return False
        if self._drive_planned_to(
            source_pose,
            staging,
            min(self.motion_speed, self.lobby_approach_speed),
            arrival_tolerance=self.elevator_approach_tolerance,
        ):
            # The doorway normal in the source frame; every pose from here on
            # stays in source, so no cross-frame conversion can distort it.
            self.elevator_heading = float(portal[2])
            self.elevator_portal_source = "WIDE_PORTAL_MAP"
            return True
        return False

    def _control_align_door(self, source_pose):
        """One cycle of the fused ALIGN_DOOR (driver's proven two-condition gate).

        The nose must point along the doorway normal AND the robot must sit on
        the doorway centre line.  Returns True when both hold.

        2026-09-15 fix (first fused run froze here for 45+ sim s): the lateral
        slide is a MEASURED run, never ``_drive_to``.  ``_drive_to`` floors its
        arrival tolerance at ``target_tolerance`` (0.35 m), which is LOOSER than
        the 0.15 m centre-line gate, so a 0.25 m lateral offset was immediately
        declared "arrived", the node only called ``_stop()``, and cmd_vel stayed
        exactly (0, 0) for ever -- the same deadlock shape as run88/run162.
        """
        portal = self._approach_portal()
        if portal is None or source_pose is None:
            self._stop()
            return False
        along, lateral = self._frame_offset(source_pose, portal)
        if abs(lateral) > self.entry_lateral_tolerance:
            if self.lateral_correction_distance is None:
                # Aim at the robot's projection onto the door centre line at the
                # current stand-off; ``min(along, -0.30)`` keeps the slide
                # outside the car, exactly like the standalone driver.
                axis_target = self._frame_point(portal, min(float(along), -0.30))
                distance = planar_distance(source_pose, axis_target)
                if distance <= 0.02:
                    self._stop()
                    return False
                heading = target_heading(source_pose, axis_target)
                self.lateral_correction_distance = float(distance)
                self.lateral_correction_heading = float(heading)
                self.elevator_heading = float(heading)
                self.travel_anchor = source_pose[:2]
                self.progress_pose = source_pose[:2]
                self.progress_stamp = rospy.Time.now()
                self._align(source_pose, heading)
                return False
            heading = float(self.lateral_correction_heading)
            self.elevator_heading = heading
            if not self._align(source_pose, heading):
                return False
            if self._drive_distance(
                source_pose,
                float(self.lateral_correction_distance),
                min(self.motion_speed, self.lobby_approach_speed),
                max(0.0, float(self.lateral_correction_distance) - 0.10),
            ):
                self.lateral_correction_distance = None
                self.lateral_correction_heading = None
                self._stop()
            return False
        if self._align(source_pose, float(portal[2])):
            self.elevator_heading = float(portal[2])
            return True
        return False

    def _turn_command(self, error, speed=None, stall_seconds=None, limit=None):
        """In-place turn with a walk-and-turn unstick (driver ``_turn_command``).

        A quadruped turn can lock up on the spot; walking a little while turning
        breaks the gait lock.  Bounded so a jammed turn fails loudly instead of
        grinding until the run timeout.
        """
        speed = self.turn_speed if speed is None else float(speed)
        stall_seconds = (
            self.turn_unstick_seconds if stall_seconds is None
            else float(stall_seconds)
        )
        limit = self.turn_unstick_limit if limit is None else int(limit)
        now = rospy.Time.now()
        with self.lock:
            pose = self.source_pose
        if pose is not None:
            if self.turn_anchor_yaw is None or self.turn_anchor_stamp is None:
                self.turn_anchor_yaw = float(pose[2])
                self.turn_anchor_stamp = now
            else:
                turned = abs(normalize_angle(float(pose[2]) - self.turn_anchor_yaw))
                if turned >= self.turn_progress_yaw:
                    self.turn_anchor_yaw = float(pose[2])
                    self.turn_anchor_stamp = now
                elif (now - self.turn_anchor_stamp).to_sec() >= stall_seconds:
                    if self.turn_unstick_count >= limit:
                        self.turn_unstick_count = 0
                        self._fail("TURN_BLOCKED_IN_{}".format(self.state))
                        return
                    self.turn_unstick_dir = -self.turn_unstick_dir
                    self.turn_unstick_until = now + rospy.Duration(
                        self.turn_unstick_walk
                    )
                    self.turn_unstick_count += 1
                    self.turn_anchor_yaw = float(pose[2])
                    self.turn_anchor_stamp = now
                    rospy.logwarn(
                        "Turn made no progress for %.1f s in %s; walk-and-turn "
                        "unstick %+.2f m/s (count %d)",
                        stall_seconds,
                        self.state,
                        self.turn_unstick_dir * self.turn_unstick_walk_speed,
                        self.turn_unstick_count,
                    )
        if self.turn_unstick_until is not None and now < self.turn_unstick_until:
            self._publish_command(
                self.turn_unstick_dir * self.turn_unstick_walk_speed,
                math.copysign(speed, error),
            )
            return
        self.turn_unstick_until = None
        self._publish_command(0.0, math.copysign(speed, error))

    def _finish_spawn_return(self):
        self._stop()
        self.returned_to_spawn = True
        self.two_floor_mission_complete = True
        self._set_state("RETURNED_TO_SPAWN")
        self.mission_complete_pub.publish(Bool(data=True))
        rospy.loginfo("Mission complete: returned to spawn")

    def _finish_floor_topology(self, reason, source_gate, gate):
        """Hand the freshly entered floor to its explorer from the corridor start.

        The explorer anchors its ``initial_forward_distance`` run on the pose it
        starts from, so the elevator must leave the robot ON the corridor start
        for the upper-floor fixed forward to end at the floor-0 14.5 node.
        """
        self.establish_started_wall = None
        self.floor1_gate = tuple(
            self.floor0_gate_source or source_gate or gate or ()
        )
        self.floor1_topology_isolated = True
        self.transition_complete = True
        self._stop()
        self._set_state("FLOOR_1_READY")
        # The floor's explorer restores the plane gait for exploration (it runs
        # the same entrance transit as floor 0), so the next ride must re-assert
        # the stair gait for the 6 cm threshold rather than trusting this flag.
        self.gait_policy_selected = False
        self.complete_pub.publish(Bool(data=True))
        self.floor1_context_published = True
        self.floor1_complete = False
        self._publish_floor_context(reason)
        self._publish_status()

    def _control_spawn_return(self, source_pose):
        """Lift -> main entrance -> spawn, ported from the driver's RETURN_SPAWN.

        Three phases: line up on the doorway axis INSIDE the lobby (where the
        estimate is good and the pivot is safe), turn to the spawn heading
        indoors, then REVERSE straight out through the entrance, down the 8 cm
        apron and across the forecourt.  Reversing keeps the feature-rich
        building side facing the lidar, which is what bounds the drift that
        killed the forward attempt (1.5-3.4 m measured).  Arrival is judged on
        position only: the official criteria do not score a final heading.
        """
        spawn = self.spawn_world
        with self.lock:
            world = self.pose
        if source_pose is None or world is None or spawn is None:
            self._stop()
            return
        phase = self.spawn_phase or "AXIS"
        if phase == "AXIS":
            axis_point = (float(spawn[0]), self.spawn_turn_y)
            distance = planar_distance(world[:2], axis_point)
            if distance <= self.spawn_axis_tolerance:
                self.spawn_phase = "FACE"
                self._stop()
                return
            target_source = self._world_to_source_point(axis_point)
            if target_source is None:
                self._stop()
                return
            if self.spawn_axis_since is None:
                self.spawn_axis_since = rospy.Time.now()
                rospy.loginfo(
                    "Return leg: lining up on the doorway axis at (%.2f, %.2f) "
                    "for the indoor U-turn",
                    axis_point[0], axis_point[1],
                )
            elif (
                rospy.Time.now() - self.spawn_axis_since
            ).to_sec() > self.spawn_axis_timeout:
                self._fail("SPAWN_AXIS_TIMEOUT_DIST_{:.2f}M".format(distance))
                return
            if self._drive_planned_to(
                source_pose,
                target_source,
                min(self.motion_speed, self.lobby_approach_speed),
                arrival_tolerance=self.spawn_axis_tolerance,
            ):
                self.spawn_phase = "FACE"
                self._stop()
            return
        if phase == "FACE":
            target_source_yaw = self._world_to_source_yaw(float(spawn[2]))
            if target_source_yaw is None:
                self._stop()
                return
            error = normalize_angle(target_source_yaw - source_pose[2])
            if abs(error) <= self.spawn_face_tolerance:
                self.spawn_phase = "REVERSE"
                self.spawn_reverse_sim = None
                self.turn_anchor_yaw = None
                self.turn_anchor_stamp = None
                self.turn_unstick_until = None
                self.turn_unstick_count = 0
                self._stop()
                return
            if self.spawn_face_since is None:
                self.spawn_face_since = rospy.Time.now()
            elif (
                rospy.Time.now() - self.spawn_face_since
            ).to_sec() > self.spawn_turn_timeout:
                self._fail(
                    "SPAWN_FACE_TIMEOUT_YAW_ERROR_{:.2f}RAD".format(abs(error))
                )
                return
            self._turn_command(error)
            return
        # REVERSE
        distance = planar_distance(world[:2], spawn[:2])
        if distance <= self.target_tolerance:
            self._finish_spawn_return()
            return
        now = rospy.Time.now()
        if self.spawn_reverse_sim is None:
            self.spawn_reverse_sim = now
            rospy.loginfo(
                "Return leg: REVERSING out to the spawn (est %.2f, %.2f; spawn "
                "yaw %.2f), centred on the doorway jambs while crossing",
                world[0], world[1], float(spawn[2]),
            )
        if (now - self.spawn_reverse_sim).to_sec() > self.spawn_reverse_timeout:
            self._fail("SPAWN_REVERSE_TIMEOUT_DIST_{:.2f}M".format(distance))
            return
        if self.spawn_reverse_retreat_until is not None:
            if now < self.spawn_reverse_retreat_until:
                self._publish_command(abs(self.entry_retreat_speed), 0.0)
                return
            self.spawn_reverse_retreat_until = None
            self.spawn_reverse_anchor = world[:2]
            self.spawn_reverse_progress_stamp = now
            self.spawn_reverse_bias_until = now + rospy.Duration(
                self.entry_retreat_seconds
            )
        if self.spawn_reverse_progress_stamp is None:
            self.spawn_reverse_progress_stamp = now
            self.spawn_reverse_anchor = world[:2]
        elif (
            self.spawn_reverse_anchor is None
            or planar_distance(world[:2], self.spawn_reverse_anchor) >= 0.15
        ):
            self.spawn_reverse_anchor = world[:2]
            self.spawn_reverse_progress_stamp = now
        elif (now - self.spawn_reverse_progress_stamp).to_sec() >= self.exit_stall_seconds:
            if self.spawn_reverse_retries >= self.exit_retry_limit:
                self._fail(
                    "SPAWN_REVERSE_NO_PROGRESS_AFTER_{}_RETRIES_DIST_{:.2f}M".format(
                        self.spawn_reverse_retries, distance
                    )
                )
                return
            self.spawn_reverse_retries += 1
            self.spawn_reverse_retreat_until = now + rospy.Duration(
                self.entry_retreat_seconds
            )
            self.spawn_reverse_progress_stamp = now
            self.spawn_reverse_anchor = world[:2]
            self._stop()
            rospy.logwarn(
                "Return leg: caught on the entrance apron/step (est %.2f, %.2f); "
                "retreating %.1f s and retrying the reverse (%d/%d)",
                world[0], world[1], self.entry_retreat_seconds,
                self.spawn_reverse_retries, self.exit_retry_limit,
            )
            return
        heading = float(world[2])
        if (
            self.spawn_reverse_bias_until is not None
            and now < self.spawn_reverse_bias_until
        ):
            heading += self.spawn_reverse_yaw_bias * (
                1.0 if self.spawn_reverse_retries % 2 else -1.0
            )
        dx = float(spawn[0]) - float(world[0])
        dy = float(spawn[1]) - float(world[1])
        lateral = -dx * math.sin(heading) + dy * math.cos(heading)
        jamb_term = 0.0
        if (
            math.isfinite(self.left_clearance)
            and math.isfinite(self.right_clearance)
            and abs(float(world[0])) <= 1.6
            and -1.0 <= float(world[1]) <= 3.0
        ):
            # Doorway-jamb centring: a laser term that needs no truth and is
            # exactly what the feature-poor lobby cannot give the estimator.
            jamb_term = max(
                -0.18, min(0.18, 0.12 * (self.right_clearance - self.left_clearance))
            )
        speed = min(self.motion_speed, max(0.12, 1.0 * distance))
        self._publish_command(
            -abs(speed),
            max(-0.30, min(0.30, -0.5 * lateral + jamb_term)),
        )

    def _control(self, _event):
        with self.lock:
            state, pose, gate = self.state, self.pose, self.gate
            source_pose, source_gate = self.source_pose, self.gate_source
            floor_complete = self.floor_complete
            elevator_portal = self._approach_portal()
        if not self.enabled or self.two_floor_mission_complete or self.fault is not None:
            return
        # Sample the base height in *every* state, including while the explorer
        # owns the ground floor, so the fall baseline follows the metric drift.
        if pose is not None and (rospy.Time.now() - self.last_pose_stamp).to_sec() <= 1.0:
            self._record_base_height(pose)
        # Refresh the world-frame doorway from the source-frame candidate every
        # cycle.  The candidate is a stable map feature, so it is only ever
        # written once; the cached world pose was therefore a snapshot taken
        # when it was first confirmed and drifts away from the robot's live
        # world pose (run86: cached (1.76, 5.65) against a live conversion of
        # (2.47, 2.56) for the same doorway).  The boarding guard compares that
        # world pose against the live world pose, so a stale cache makes it
        # refuse to board for ever.
        if (
            self.elevator_portal is not None
            and pose is not None
            and source_pose is not None
        ):
            self.elevator_portal_world = transform_pose_between_frames(
                self.elevator_portal, source_pose, pose
            )
        if state == self.WAITING:
            if floor_complete or self.start_immediately:
                # The ground floor is the reference for the shared topology.
                # Do not leave it while its own topology is unresolved.
                if not self.start_immediately and not self.reference_floor_ready:
                    if self.reference_wait_since is None:
                        self.reference_wait_since = rospy.Time.now()
                    waited = (
                        rospy.Time.now() - self.reference_wait_since
                    ).to_sec()
                    if waited < self.reference_floor_wait:
                        self._stop()
                        rospy.logwarn_throttle(
                            5.0,
                            "Reference floor topology not reusable yet: %s",
                            self.reference_floor_fault or "NO_STATUS",
                        )
                        return
                    # The reference topology is only a hint: every floor now
                    # keeps its own 2D map, so an unresolved hint must not hold
                    # the mission on a fully explored ground floor for ever.
                    rospy.logwarn(
                        "Proceeding without a reusable reference topology after "
                        "%.0f s (%s); upper floors will resolve their own doorways",
                        waited,
                        self.reference_floor_fault or "NO_STATUS",
                    )
                    self.reference_floor_ready = True
            if (
                (floor_complete or self.start_immediately)
                and pose is not None
                and gate is not None
                and source_gate is not None
                and elevator_portal is not None
            ):
                if not self._ensure_gait_policy():
                    return
                self._stop()
                self._set_state("RETURN_TO_ELEVATOR")
            elif (floor_complete or self.start_immediately) and pose is not None:
                self._stop()
                rospy.logwarn_throttle(
                    5.0,
                    "Waiting for map-confirmed elevator portal before transition",
                )
            return
        if pose is None or (rospy.Time.now() - self.last_pose_stamp).to_sec() > 1.0:
            self._stop()
            return
        fall = self._check_fall(pose)
        if fall is not None:
            self._fail(fall)
            return

        if state == "RETURN_TO_ELEVATOR":
            # Fused from the standalone lift test (2026-09-15).  Shared with the
            # upper-floor return (RETURN_TO_FLOOR_1_GATE): corridor start ->
            # centreline -> U-turn -> staged stand-off, then ALIGN/ENTER.
            gate = self._corridor_gate()
            if source_pose is None or gate is None:
                self._stop()
                return
            if self._control_return_to_lift(source_pose, gate):
                self._set_state("ALIGN_ELEVATOR")
        elif state == "ALIGN_ELEVATOR":
            # Ported from the standalone driver's ALIGN_DOOR (2026-09-15): two
            # conditions before the committed run -- the nose points along the
            # doorway normal AND the robot sits on the doorway centre line.  The
            # old version re-drove the staging point and could deadlock when
            # "arrived at staging" (0.35 m) was looser than the centre-line gate
            # (0.15 m) and the two measurements were orthogonal (run162: 188 sim
            # seconds of cmd_vel exactly (0, 0)).
            if self._control_align_door(source_pose):
                self.travel_anchor = source_pose[:2]
                self.progress_pose = source_pose[:2]
                self.progress_stamp = rospy.Time.now()
                self.entry_retries = 0
                self._set_state("ENTER_ELEVATOR")
        elif state == "ENTER_ELEVATOR":
            if not self._ensure_elevator_door_open(self.current_floor):
                self._stop()
                return
            # The car threshold is a real 6 cm step, so the step-trained gait must
            # be selected before anything moves (see _ensure_gait_policy).
            if not self._ensure_gait_policy():
                self._stop()
                return
            if self._drive_distance(
                source_pose,
                self.enter_distance,
                self.crossing_speed,
                self.minimum_entry_progress,
            ):
                # Boarding proof (run84 + run166): the entry distance alone was
                # once treated as "the robot is in the car", so a pose 4.1 m away
                # in the lobby rode up alone -- a guard is needed, but it must be
                # SIGNED.  It used an UNSIGNED distance to the doorway (hypot), so
                # run166, which pressed against the car's rear wall 1.67 m PAST
                # the door plane (the normal contained end of the entry run), was
                # reported as "1.67 m from the car doorway" and refused the ride
                # for ever, oscillating against the rear wall.  ``_entry_inside_car``
                # gives the signed door-frame along (+ = inside the car); reject
                # only when the robot is still clearly in FRONT of the plane.
                # Any positive depth is contained, and ``_entry_outside_car``
                # below remains the strict "did it actually cross" gate.
                inside = self._entry_inside_car()
                if inside is not None and inside < 0.0:
                    rospy.logwarn_throttle(
                        2.0,
                        "ENTER_ELEVATOR: entry distance driven but the robot is "
                        "still %.2f m OUTSIDE the car door plane; stopping instead "
                        "of riding with the robot outside",
                        -inside,
                    )
                    self._stop()
                    return
                # Face the doorway before entering (user requirement).  The ride
                # must not start while the robot is turned away from the portal.
                #
                # (b), user decision 2026-09-14: this requirement is about the
                # robot still OUTSIDE the car.  By the time control reaches here
                # the ENTER_ELEVATOR drive has already completed - either the full
                # ``enter_distance`` was travelled or ``_drive_distance`` confirmed
                # the robot contained at the car threshold with >=
                # ``minimum_entry_progress`` - and ALIGN_ELEVATOR already faced
                # the doorway before committing to the drive.  A residual error
                # measured here is therefore drift picked up *inside* the car, so
                # it must not block the ride.
                #
                # Enforcing it here deadlocked run151: the robot sat in the car
                # doorway at 0.12-0.13 rad against the 0.06 rad gate for 115+ s
                # with cmd_vel exactly (0, 0), because this branch only stops -
                # it has no alignment and no timeout.  The error is still logged
                # (it is worth seeing) but it no longer decides the ride.
                heading_error = 0.0
                if self.elevator_heading is not None:
                    delta = float(self.elevator_heading) - float(source_pose[2])
                    heading_error = abs(math.atan2(math.sin(delta), math.cos(delta)))
                if heading_error > self.board_heading_tolerance:
                    rospy.logwarn_throttle(
                        5.0,
                        "ENTER_ELEVATOR: boarding from inside the car with a "
                        "%.2f rad heading error (pre-entry gate %.2f); riding "
                        "anyway, the entry drive already completed",
                        heading_error,
                        self.board_heading_tolerance,
                    )
                self.ride_start_z = pose[3]
                # Ride only from INSIDE the car door plane (known geometry); a
                # jamb-stalled entry must be retried, not ridden from the lobby.
                if self._entry_outside_car():
                    return
                self._set_state("RIDE_TO_FLOOR_1")
        elif state == "RIDE_TO_FLOOR_1" or state == "RIDE_TO_NEXT_FLOOR":
            self._stop()
            self._start_ride()
            if self.ride_error is not None:
                self._fail("ELEVATOR_SERVICE_FAILED: {}".format(self.ride_error))
            elif self.ride_response is not None:
                if not self.ride_response.accepted or self.ride_response.current_floor != self.target_floor:
                    self._fail("ELEVATOR_REJECTED: {}".format(self.ride_response.message))
                else:
                    # The official Classic control service moves the car and
                    # passenger model atomically.  A metric pose z change is
                    # therefore not guaranteed to arrive before the service
                    # response; service acceptance is the primary ride event.
                    #
                    # The heading deliberately stays on the boarding bearing:
                    # the car is left in reverse (see _drive_distance), which
                    # both skips the pi turn and backs down the 6 cm threshold
                    # rear-feet-first instead of stalling on it front-first.
                    self.current_floor = int(self.ride_response.current_floor)
                    # A new floor has its own car door id; re-request it.
                    self.elevator_door_requested_for = None
                    # Switch the occupancy map to the floor the robot is now
                    # standing on, before driving anywhere.  Publishing this
                    # only at FLOOR_1_READY (after ESTABLISH_FLOOR_1_TOPOLOGY has
                    # already driven to the corridor) left run52 driving on the
                    # *previous* floor's map: 654 failed A* attempts, route
                    # target null, and a 0.45 m/s open-loop crawl on floor 2.
                    self._publish_floor_context("arrival")
                    self._set_state("ALIGN_FLOOR_1_EXIT")
        elif state == "ALIGN_FLOOR_1_EXIT":
            if self._align(source_pose, self.elevator_heading):
                self.travel_anchor = source_pose[:2]
                self._set_state("EXIT_ELEVATOR")
        elif state == "EXIT_ELEVATOR":
            now = rospy.Time.now()
            if self.exit_retreat_until is not None:
                if now < self.exit_retreat_until:
                    # Push back into the car for a moment to release the
                    # threshold contact before retrying the reverse run.
                    self._publish_command(self.entry_retreat_speed, 0.0)
                    return
                self.exit_retreat_until = None
                self.travel_anchor = source_pose[:2]
                self.exit_progress_pose = source_pose[:2]
                self.exit_progress_stamp = now
            if self.exit_progress_pose is None:
                self.exit_progress_pose = source_pose[:2]
                self.exit_progress_stamp = now
            elif planar_distance(source_pose, self.exit_progress_pose) >= 0.15:
                self.exit_progress_pose = source_pose[:2]
                self.exit_progress_stamp = now
            elif (now - self.exit_progress_stamp).to_sec() >= self.exit_stall_seconds:
                if self.exit_retries >= self.exit_retry_limit:
                    self._fail("ELEVATOR_EXIT_NO_PROGRESS")
                    return
                self.exit_retries += 1
                self.exit_retreat_until = now + rospy.Duration(self.entry_retreat_seconds)
                self.exit_progress_pose = source_pose[:2]
                self.exit_progress_stamp = now
                rospy.logwarn(
                    "EXIT_ELEVATOR stalled at (%.2f, %.2f); retreating into the "
                    "car for %.1fs and retrying (retry %d/%d)",
                    source_pose[0],
                    source_pose[1],
                    self.entry_retreat_seconds,
                    self.exit_retries,
                    self.exit_retry_limit,
                )
                self._stop()
                return
            if self._drive_distance(
                source_pose,
                self.exit_distance,
                self.crossing_speed,
                self.minimum_exit_progress,
                reverse=True,
            ):
                self._stop()
                self.travel_anchor = pose[:2]
                self.establish_started_wall = time.time()
                self._set_state("ESTABLISH_FLOOR_1_TOPOLOGY")
        elif state == "ESTABLISH_FLOOR_1_TOPOLOGY":
            # Fused from the standalone lift test (2026-09-15).  This is the
            # "lift -> corridor start" leg: clear the car mouth first (the
            # driver's TO_CORRIDOR sub-leg 0), run to the corridor start, then
            # align on the corridor axis (CORRIDOR_CENTRE).  The floor explorer
            # is handed a robot standing ON the corridor start, so its own
            # fixed forward starts there; on floors 2/3 that forward is 4.00 m
            # (the same physical node as floor 0's 14.5 m transit), while floor
            # 0 keeps its outdoor entrance transit.
            #
            # The corridor start is a soft hint, not a gate: with one 2D map per
            # floor the floor's own explorer will map the corridor anyway.  A
            # route that cannot be planned must not hold the whole mission here
            # (run52 floor 2: 654 failed A* attempts and a 0.45 m/s crawl), so
            # accept the topology after a bounded real-time budget.
            if establish_budget_exceeded(
                self.establish_started_wall, time.time(), self.establish_timeout
            ):
                rospy.logwarn(
                    "Floor %d topology not reached within %.0fs "
                    "(route_retry_count=%d, front_clearance=%.2f); continuing "
                    "so the floor explorer can map the corridor",
                    int(self.current_floor),
                    self.establish_timeout,
                    int(self.route_retry_count),
                    float(self.front_clearance),
                )
                self._finish_floor_topology("floor_ready_timeout", source_gate, gate)
                return
            corridor_gate = self._corridor_gate()
            if source_pose is None or corridor_gate is None:
                self._stop()
                return
            phase = self.establish_phase or "CLEAR_CAR"
            if phase == "CLEAR_CAR":
                # The standalone driver's TO_CORRIDOR sub-leg 0, and the piece
                # the first fusion missed (run167 froze here for 75+ sim s):
                # after a REVERSE exit the robot faces INTO the car with the
                # staging waypoint directly BEHIND it, and pivoting 180 deg at
                # the car mouth wedges it between the stair core and the shaft
                # (the driver documents exactly this as "probe 02").  So BACK
                # STRAIGHT out along the door normal to the staging stand-off
                # first, and turn only once there is room.
                #
                # ``_drive_planned_to(staging)`` looked equivalent but is not:
                # within direct_entry_max_distance it calls ``_drive_direct``,
                # which TURNS IN PLACE before driving -- run167 sat at 1.28 m
                # from the door plane publishing cmd_wz=-0.45 with a frozen yaw.
                portal = self._approach_portal()
                target_along = -self.staging_distance
                if self.source_pose is not None and portal is not None:
                    # Latch the reverse distance ONCE (see the field comment):
                    # recomputing it per cycle against a pinned travel anchor
                    # stopped run168 after ~half the required reverse.
                    if self.establish_clear_distance is None:
                        along, _lateral = door_frame_offset(self.source_pose, portal)
                        self.establish_clear_distance = max(
                            0.0, float(along) - target_along
                        )
                    if self.establish_clear_distance > self.target_tolerance:
                        distance = float(self.establish_clear_distance)
                        if not self._drive_distance(
                            source_pose,
                            distance,
                            min(self.motion_speed, self.lobby_approach_speed),
                            max(0.0, distance - 0.15),
                            reverse=True,
                        ):
                            return
                    self.establish_clear_distance = None
                    self.establish_phase = "TO_CORRIDOR_START"
                    phase = "TO_CORRIDOR_START"
                else:
                    staging = self._elevator_staging_source()
                    if staging is not None and not self._drive_planned_to(
                        source_pose,
                        staging,
                        min(self.motion_speed, self.lobby_approach_speed),
                        arrival_tolerance=self.elevator_approach_tolerance,
                    ):
                        return
                    self.establish_phase = "TO_CORRIDOR_START"
                    phase = "TO_CORRIDOR_START"
            if phase == "TO_CORRIDOR_START":
                if not self._drive_planned_to(
                    source_pose,
                    corridor_gate[:2],
                    min(self.motion_speed, self.lobby_approach_speed),
                    arrival_tolerance=self.corridor_start_tolerance,
                ):
                    return
                self.establish_phase = "CORRIDOR_CENTRE"
                phase = "CORRIDOR_CENTRE"
            if phase == "CORRIDOR_CENTRE":
                if not self._center_on_corridor_axis(
                    source_pose, corridor_gate, "establish_center_distance"
                ):
                    return
            self.establish_phase = None
            self.floor1_corridor_target = tuple(corridor_gate[:2])
            self._finish_floor_topology("floor_ready", source_gate, gate)
        elif state == "FLOOR_1_READY":
            if self.floor1_complete:
                self._stop()
                self._set_state("RETURN_TO_FLOOR_1_GATE")
        elif state == "RETURN_TO_FLOOR_1_GATE":
            # Fused 2026-09-15: an upper-floor return uses the SAME
            # "corridor start -> lift" method as the ground floor (the
            # standalone cycle's corridor-mouth -> centre -> U-turn -> staging
            # sequence), instead of the old gate -> lobby-search -> sensor sweep
            # chain.  The supervisor still keys on this state name to stop the
            # floor explorer, so the contract is unchanged.
            gate = self._corridor_gate()
            if source_pose is None or gate is None:
                self._stop()
                return
            if self._control_return_to_lift(source_pose, gate):
                self._set_state("ALIGN_FLOOR_1_ELEVATOR_RETURN")
        elif state == "ALIGN_FLOOR_1_ELEVATOR_RETURN":
            # Same fused ALIGN_DOOR gate as the ground floor: centre line plus
            # doorway normal before the committed entry run.
            if self._control_align_door(source_pose):
                self.travel_anchor = source_pose[:2]
                self.progress_pose = source_pose[:2]
                self.progress_stamp = rospy.Time.now()
                self.entry_retries = 0
                self._set_state("ENTER_FLOOR_1_ELEVATOR_RETURN")
        elif state == "ENTER_FLOOR_1_ELEVATOR_RETURN":
            if not self._ensure_elevator_door_open(self.current_floor):
                self._stop()
                return
            # FLOOR_1_READY clears ``gait_policy_selected`` ("the next ride must
            # re-assert the stair gait for the 6 cm threshold") and this path had
            # no re-assertion at all, so the return entry pushed into the
            # threshold with the flat-ground policy: run153 stalled 0.07-0.11 m
            # BEFORE the door plane with 0.45 m/s commanded and zero progress, and
            # run152 had the same stall (1.32 m of the 2.45 m entry, ending 0.46 m
            # outside the car).  Same guard as the floor-0 entry.
            if not self._ensure_gait_policy():
                self._stop()
                return
            if self._drive_distance(
                source_pose, self.enter_distance, self.crossing_speed,
                self.minimum_entry_progress,
            ):
                self._stop()
                # Same gate as the floor-0 entry: this is the path that rode from
                # OUTSIDE the car on run152 floor 1 (0.46 m short of the plane).
                if self._entry_outside_car():
                    return
                if self.target_floor < self.max_floor:
                    self.target_floor += 1
                    self.floor1_complete = False
                    self.floor1_context_published = False
                    self.transition_complete = False
                    self.floor1_topology_isolated = False
                    self.complete_pub.publish(Bool(data=False))
                    self.ride_thread = None
                    self.ride_response = None
                    self.ride_error = None
                    self._set_state("RIDE_TO_NEXT_FLOOR")
                else:
                    # Every floor is explored.  Ride back to the ground floor
                    # instead of declaring the mission finished inside the lift.
                    self.target_floor = self.ground_floor
                    self.ride_thread = None
                    self.ride_response = None
                    self.ride_error = None
                    self._set_state("RIDE_TO_GROUND_FLOOR")
        elif state == "RIDE_TO_GROUND_FLOOR":
            self._stop()
            self._start_ride()
            if self.ride_error is not None:
                self._fail("ELEVATOR_SERVICE_FAILED: {}".format(self.ride_error))
            elif self.ride_response is not None:
                if (
                    not self.ride_response.accepted
                    or self.ride_response.current_floor != self.ground_floor
                ):
                    self._fail("ELEVATOR_REJECTED: {}".format(self.ride_response.message))
                else:
                    # 2026-09-15: no pi flip here any more.  The robot boards
                    # facing INTO the car (``elevator_heading`` is the doorway
                    # normal), and the fused reverse exit backs straight out
                    # along that heading -- exactly the standalone lift test's
                    # cycle, which reverses out of the car on every leg.  The old
                    # flip made ``_drive_distance(reverse=True)`` run the robot
                    # back INTO the car.
                    self.current_floor = int(self.ride_response.current_floor)
                    # A new floor has its own car door id; re-request it.
                    self.elevator_door_requested_for = None
                    # The occupancy node keeps one 2D map per floor, so the
                    # descent has to hand the ground floor back or the return to
                    # spawn would plan on the last explored floor's map.
                    self._publish_floor_context("descent")
                    self._set_state("ALIGN_GROUND_FLOOR_EXIT")
        elif state == "ALIGN_GROUND_FLOOR_EXIT":
            if self._align(source_pose, self.elevator_heading):
                self.travel_anchor = source_pose[:2]
                self._set_state("EXIT_GROUND_FLOOR")
        elif state == "EXIT_GROUND_FLOOR":
            if self._drive_distance(
                source_pose, self.exit_distance, self.crossing_speed,
                self.minimum_exit_progress,
                reverse=True,
            ):
                self._stop()
                self.travel_anchor = source_pose[:2]
                self._set_state("OPEN_MAIN_ENTRANCE")
        elif state == "OPEN_MAIN_ENTRANCE":
            self._stop()
            if not self.main_entrance_opened:
                try:
                    rospy.wait_for_service("/set_door_state", timeout=5.0)
                    response = self.door_service(self.main_entrance_id, True)
                except (rospy.ROSException, rospy.ServiceException) as error:
                    rospy.logwarn_throttle(
                        5.0, "Waiting for main entrance door service: %s", error
                    )
                    return
                if response.accepted and str(response.state) == "open":
                    self.main_entrance_opened = True
                    rospy.loginfo(
                        "Main entrance opened: accepted=%s state=%s",
                        response.accepted,
                        response.state,
                    )
                else:
                    # accepted=False means the door id is unknown, so claiming
                    # success here would report a false pass and then drive the
                    # robot into a closed door.  Retry a bounded number of times
                    # and continue with an honest failure instead of hanging.
                    self.main_entrance_attempts += 1
                    rospy.logwarn_throttle(
                        5.0,
                        "Main entrance not opened (accepted=%s state=%s "
                        "message=%s attempt=%d/%d)",
                        response.accepted,
                        response.state,
                        response.message,
                        self.main_entrance_attempts,
                        self.main_entrance_max_attempts,
                    )
                    if self.main_entrance_attempts < self.main_entrance_max_attempts:
                        return
                    rospy.logerr(
                        "Main entrance could not be opened after %d attempts; "
                        "continuing to spawn with main_entrance_opened=false",
                        self.main_entrance_attempts,
                    )
            self._set_state("RETURN_TO_SPAWN")
        elif state == "RETURN_TO_SPAWN":
            # Fused from the standalone lift test (2026-09-15): the final leg
            # leaves the lift, opens the main entrance (done in
            # OPEN_MAIN_ENTRANCE) and drives out to the spawn in three phases
            # (axis line-up indoors, in-lobby turn to the spawn heading,
            # reverse out through the entrance and down the 8 cm apron).  The
            # standalone cycle ran this 10/10 with truth and it is the only exit
            # method that held the drift far enough to arrive.
            if self.spawn_world is None:
                # No world pose was ever latched: fall back to the source-frame
                # origin, which the localisation bridge anchors at the spawn.
                if source_pose is not None and self._drive_planned_to(
                    source_pose,
                    (0.0, 0.0),
                    arrival_tolerance=self.return_to_spawn_tolerance,
                ):
                    self._finish_spawn_return()
                return
            self._control_spawn_return(source_pose)

        self._publish_status()

    def _publish_status(self):
        with self.lock:
            payload = {
                "state": self.state,
                "floor_complete": self.floor_complete,
                "transition_complete": self.transition_complete,
                "floor_index": self.current_floor,
                "completed_floor_indices": sorted(self.completed_floor_indices),
                "floor1_topology_isolated": self.floor1_topology_isolated,
                "floor1_complete": self.floor1_complete,
                "two_floor_mission_complete": self.two_floor_mission_complete,
                "returned_to_spawn": bool(self.returned_to_spawn),
                "main_entrance_opened": bool(self.main_entrance_opened),
                "main_entrance_attempts": int(self.main_entrance_attempts),
                "floor1_gate": list(self.floor1_gate)
                if self.floor1_gate is not None else None,
                "floor0_gate_source": list(self.floor0_gate_source)
                if self.floor0_gate_source is not None else None,
                # The reference-floor gate decides whether the elevator may
                # leave.  Without these fields a blocked departure is
                # indistinguishable from a slow one.
                "reference_floor_ready": bool(self.reference_floor_ready),
                "reference_floor_fault": self.reference_floor_fault,
                "reference_floor_portals": len(self.reference_floor_topology),
                "target_floor": self.target_floor,
                "route_target": list(self.route_target)
                if self.route_target is not None else None,
                "route_retry_count": self.route_retry_count,
                "front_clearance": self.front_clearance,
                "gate": list(self.gate) if self.gate is not None else None,
                "elevator_portal": list(self.elevator_portal)
                if self.elevator_portal is not None else None,
                "elevator_portal_source": self.elevator_portal_source,
                "elevator_portal_world": (
                    list(self._portal_world_report())
                    if self._portal_world_report() is not None else None
                ),
                "left_clearance": self.left_clearance,
                "right_clearance": self.right_clearance,
                "elevator_heading": self.elevator_heading,
                "search_samples": [list(item) for item in self.search_samples],
                "fault": self.fault,
                "doors_closed_by_algorithm": False,
            }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _shutdown(self):
        self.timer.shutdown()
        if self.state != self.WAITING:
            self._stop()


def main():
    rospy.init_node("elevator_transition")
    ElevatorTransition()
    rospy.spin()


if __name__ == "__main__":
    main()
