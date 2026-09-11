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
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

SCRIPT_DIRECTORY = os.path.dirname(os.path.realpath(__file__))
if not sys.path or sys.path[0] != SCRIPT_DIRECTORY:
    sys.path.insert(0, SCRIPT_DIRECTORY)

from elevator_transition_core import (
    choose_opening_heading,
    detect_wide_lobby_openings,
    entry_stall_confirms_containment,
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
        self.enter_distance = float(rospy.get_param("~enter_distance", 2.45))
        self.exit_distance = float(rospy.get_param("~exit_distance", 2.10))
        self.floor1_corridor_advance = float(
            rospy.get_param("~floor1_corridor_advance", 1.80)
        )
        self.minimum_entry_progress = float(
            rospy.get_param("~minimum_entry_progress", 1.0)
        )
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
        self.crossing_speed = float(rospy.get_param("~crossing_speed", 0.22))
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
        self.route_accept_distance = max(
            0.2, float(rospy.get_param("~route_accept_distance", 1.2))
        )
        self.main_entrance_opened = False
        self.returned_to_spawn = False
        # Wall clock, not ROS time: the sim clock is paused during startup, so
        # a ROS-time stamp of zero would keep the grace window open forever.
        self.started_at_wall = time.time()
        # The reference floor fixes the shared x/y topology that every upper
        # floor reuses.  Cache its confirmed doorway geometry and refuse to
        # leave the ground floor until that topology is genuinely resolved.
        self.reference_floor_topology = []
        self.reference_floor_ready = False
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
            "/navigation_map", OccupancyGrid, self._navigation_map_callback, queue_size=1
        )
        rospy.Subscriber("/scan_2d", LaserScan, self._scan_callback, queue_size=1)
        self.elevator_service = rospy.ServiceProxy("/call_elevator", CallElevator)
        self.door_service = rospy.ServiceProxy("/set_door_state", SetDoorState)
        self.plane_policy_service = rospy.ServiceProxy(
            "/unitree/select_plane_policy", SetBool
        )
        self.plane_policy_active = False
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
            gate = payload.get("virtual_isolation_door")
            if not isinstance(gate, list) or len(gate) < 3:
                return
            parsed = tuple(float(value) for value in gate[:3])
        except (TypeError, ValueError):
            return
        with self.lock:
            self._reference_floor_snapshot(payload)
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
        if time.time() - self.started_at_wall < self.fall_grace_seconds:
            return None
        with self.lock:
            roll, pitch = self.base_roll, self.base_pitch
        if abs(roll) > self.fall_roll_limit or abs(pitch) > self.fall_pitch_limit:
            return "ROBOT_ROLLED"
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
        rospy.loginfo("Elevator transition state -> %s", state)
        self._publish_status()

    def _publish_command(self, linear=0.0, angular=0.0):
        command = Twist()
        command.linear.x = float(linear)
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
        if planar_distance(pose, target) <= tolerance:
            self._stop()
            return True
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
                    return True
                if self.route_retry_count >= self.max_route_retries:
                    self._fail(
                        "ROUTE_UNREACHABLE_{}_AFTER_{}".format(
                            self.state, self.route_retry_count
                        )
                    )
                    return False
                rospy.logwarn_throttle(
                    5.0,
                    "A* has no route to mapped target (%.2f, %.2f) in %s; retry %d/%d",
                    target[0], target[1], self.state,
                    self.route_retry_count, self.max_route_retries,
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

    def _elevator_staging_target(self):
        """Return the source-map staging point and heading for the portal."""
        with self.lock:
            gate = self.gate_source
            portal = self.elevator_portal
        if gate is None or portal is None or len(portal) < 3:
            return None, None
        return portal_staging_point(gate, portal), float(portal[2])

    def _drive_distance(self, pose, distance, speed, minimum_blocked_progress):
        if self.travel_anchor is None:
            self.travel_anchor = pose[:2]
        travelled = planar_distance(pose, self.travel_anchor)
        now = rospy.Time.now()
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
            self._fail("ELEVATOR_PATH_BLOCKED_AFTER_{:.2f}M".format(travelled))
            return False
        error = normalize_angle(self.elevator_heading - pose[2])
        if abs(error) > 0.20:
            self._publish_command(0.0, math.copysign(self.turn_speed, error))
        else:
            lateral_error = 0.0
            if self.state == "ENTER_ELEVATOR":
                if math.isfinite(self.left_clearance) and math.isfinite(self.right_clearance):
                    lateral_error = max(-0.18, min(
                        0.18, 0.10 * (self.right_clearance - self.left_clearance)
                    ))
            self._publish_command(speed, max(-0.18, min(0.18, 0.8 * error + lateral_error)))
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

    def _ensure_plane_policy(self):
        if self.plane_policy_active:
            return True
        try:
            rospy.wait_for_service("/unitree/select_plane_policy", timeout=0.5)
            response = self.plane_policy_service(True)
            self.plane_policy_active = bool(response.success)
            if not self.plane_policy_active:
                rospy.logwarn_throttle(3.0, "Plane policy switch rejected: %s", response.message)
            return self.plane_policy_active
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn_throttle(3.0, "Waiting for plane policy switch: %s", error)
            return False

    def _control(self, _event):
        with self.lock:
            state, pose, gate = self.state, self.pose, self.gate
            source_pose, source_gate = self.source_pose, self.gate_source
            floor_complete = self.floor_complete
            elevator_portal = self.elevator_portal
        if not self.enabled or self.two_floor_mission_complete or self.fault is not None:
            return
        # Sample the base height in *every* state, including while the explorer
        # owns the ground floor, so the fall baseline follows the metric drift.
        if pose is not None and (rospy.Time.now() - self.last_pose_stamp).to_sec() <= 1.0:
            self._record_base_height(pose)
        if state == self.WAITING:
            if floor_complete or self.start_immediately:
                # The ground floor is the reference for the shared topology.
                # Do not leave it while its own topology is unresolved.
                if not self.start_immediately and not self.reference_floor_ready:
                    self._stop()
                    rospy.logwarn_throttle(
                        5.0,
                        "Reference floor topology not reusable yet: %s",
                        self.reference_floor_fault or "NO_STATUS",
                    )
                    return
            if (
                (floor_complete or self.start_immediately)
                and pose is not None
                and gate is not None
                and source_gate is not None
                and elevator_portal is not None
            ):
                if not self._ensure_plane_policy():
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
            staging_target, portal_heading = self._elevator_staging_target()
            if source_pose is not None and staging_target is not None:
                reached = self._drive_planned_to(
                    source_pose,
                    staging_target,
                    arrival_tolerance=self.elevator_approach_tolerance,
                )
            else:
                self._stop()
                reached = False
            if reached:
                self.elevator_heading = portal_heading
                self.elevator_portal_source = "WIDE_PORTAL_MAP"
                self._set_state("ALIGN_ELEVATOR")
        elif state == "ALIGN_ELEVATOR":
            # The map candidate is rechecked during the A* approach.  Refresh
            # the heading once more at the staging point so a small candidate
            # correction does not send the robot into the jamb.
            _staging_target, mapped_heading = self._elevator_staging_target()
            if mapped_heading is not None and self.elevator_portal_source == "WIDE_PORTAL_MAP":
                self.elevator_heading = mapped_heading
            if self._align(pose, self.elevator_heading):
                self.travel_anchor = pose[:2]
                self.progress_pose = pose[:2]
                self.progress_stamp = rospy.Time.now()
                self.entry_retries = 0
                self._set_state("ENTER_ELEVATOR")
        elif state == "ENTER_ELEVATOR":
            if self._drive_distance(
                pose,
                self.enter_distance,
                self.crossing_speed,
                self.minimum_entry_progress,
            ):
                self.ride_start_z = pose[3]
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
                    self.elevator_heading = normalize_angle(self.elevator_heading + math.pi)
                    self.current_floor = int(self.ride_response.current_floor)
                    self._set_state("ALIGN_FLOOR_1_EXIT")
        elif state == "ALIGN_FLOOR_1_EXIT":
            if self._align(pose, self.elevator_heading):
                self.travel_anchor = pose[:2]
                self._set_state("EXIT_ELEVATOR")
        elif state == "EXIT_ELEVATOR":
            if self._drive_distance(
                pose,
                self.exit_distance,
                self.crossing_speed,
                self.minimum_exit_progress,
            ):
                self._stop()
                self.travel_anchor = pose[:2]
                self._set_state("ESTABLISH_FLOOR_1_TOPOLOGY")
        elif state == "ESTABLISH_FLOOR_1_TOPOLOGY":
            # Floors share x/y topology.  The elevator already exits into the
            # lobby, so route directly to the floor-0 gate's corresponding
            # corridor-side point instead of replaying the outdoor approach.
            if source_pose is not None and source_gate is not None:
                corridor_target = point_from_gate(
                    source_gate, self.floor1_corridor_advance
                )
                reached = self._drive_planned_to(
                    source_pose, corridor_target,
                    min(self.motion_speed, self.lobby_approach_speed),
                )
            else:
                corridor_target = point_from_gate(
                    gate, self.floor1_corridor_advance
                )
                self._stop()
                reached = False
            if reached:
                self.floor1_gate = tuple(
                    self.floor0_gate_source or source_gate or gate
                )
                self.floor1_topology_isolated = True
                self.transition_complete = True
                self._stop()
                self._set_state("FLOOR_1_READY")
                self.complete_pub.publish(Bool(data=True))
                self.floor1_context_published = True
                self.floor1_complete = False
                self.context_pub.publish(String(data=json.dumps({
                    "floor_index": self.current_floor,
                    "floor_z": pose[3],
                    "gate_source": list(self.floor1_gate),
                    "gate_world": list(self.floor0_gate or gate),
                    "corridor_target_source": list(corridor_target[:2]),
                    # Upper floors share the reference floor's x/y topology.
                    # Hand the resolved ground-floor doorways over verbatim so
                    # the next explorer reuses them instead of rediscovering
                    # (and possibly mis-binning) the same structure.
                    "reused_topology": list(self.reference_floor_topology),
                }, sort_keys=True)))
        elif state == "FLOOR_1_READY":
            if self.floor1_complete:
                self._stop()
                self._set_state("RETURN_TO_FLOOR_1_GATE")
        elif state == "RETURN_TO_FLOOR_1_GATE":
            if source_pose is not None:
                reached = self._drive_planned_to(source_pose, self.floor1_gate[:2])
            else:
                self._stop()
                reached = False
            if reached:
                self._set_state("ENTER_FLOOR_1_LOBBY")
        elif state == "ENTER_FLOOR_1_LOBBY":
            target = point_from_gate(self.floor1_gate, self.lobby_search_offset)
            if source_pose is not None:
                reached = self._drive_planned_to(
                    source_pose, target,
                    min(self.motion_speed, self.lobby_approach_speed),
                )
            else:
                self._stop()
                reached = False
            if reached:
                self.search_index = 0
                self.search_samples = []
                self._set_state("SEARCH_FLOOR_1_ELEVATOR")
        elif state == "SEARCH_FLOOR_1_ELEVATOR":
            # A* references use the source map; scan steering uses the metric
            # world pose and therefore the current world-frame gate heading.
            base_heading = normalize_angle(gate[2] - math.pi / 2.0)
            sample_heading = normalize_angle(base_heading + self.search_offsets[self.search_index])
            if self._align(pose, sample_heading):
                # Opening selection must not use the conservative collision
                # percentile: sparse side returns otherwise close a genuine
                # doorway even when most rays see free space.
                self.search_samples.append((sample_heading, self.front_opening_clearance))
                self.search_index += 1
                if self.search_index >= len(self.search_offsets):
                    selected = choose_opening_heading(
                        self.search_samples,
                        self.minimum_opening_clearance,
                        preferred_heading=base_heading,
                    )
                    if selected is None:
                        self._fail("NO_SENSOR_CONFIRMED_FLOOR_1_ELEVATOR_OPENING")
                    else:
                        self.elevator_heading = selected
                        self._set_state("ALIGN_FLOOR_1_ELEVATOR_RETURN")
        elif state == "ALIGN_FLOOR_1_ELEVATOR_RETURN":
            if self._align(pose, self.elevator_heading):
                self.travel_anchor = pose[:2]
                self.progress_pose = pose[:2]
                self.progress_stamp = rospy.Time.now()
                self.entry_retries = 0
                self._set_state("ENTER_FLOOR_1_ELEVATOR_RETURN")
        elif state == "ENTER_FLOOR_1_ELEVATOR_RETURN":
            if self._drive_distance(
                pose, self.enter_distance, self.crossing_speed,
                self.minimum_entry_progress,
            ):
                self._stop()
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
                    self.elevator_heading = normalize_angle(self.elevator_heading + math.pi)
                    self.current_floor = int(self.ride_response.current_floor)
                    self._set_state("ALIGN_GROUND_FLOOR_EXIT")
        elif state == "ALIGN_GROUND_FLOOR_EXIT":
            if self._align(pose, self.elevator_heading):
                self.travel_anchor = pose[:2]
                self._set_state("EXIT_GROUND_FLOOR")
        elif state == "EXIT_GROUND_FLOOR":
            if self._drive_distance(
                pose, self.exit_distance, self.crossing_speed,
                self.minimum_exit_progress,
            ):
                self._stop()
                self.travel_anchor = pose[:2]
                self._set_state("OPEN_MAIN_ENTRANCE")
        elif state == "OPEN_MAIN_ENTRANCE":
            self._stop()
            if not self.main_entrance_opened:
                try:
                    rospy.wait_for_service("/set_door_state", timeout=5.0)
                    response = self.door_service(self.main_entrance_id, True)
                    self.main_entrance_opened = True
                    rospy.loginfo(
                        "Main entrance opened: accepted=%s state=%s",
                        response.accepted,
                        response.state,
                    )
                except (rospy.ROSException, rospy.ServiceException) as error:
                    rospy.logwarn_throttle(
                        5.0, "Waiting for main entrance door service: %s", error
                    )
                    return
            self._set_state("RETURN_TO_SPAWN")
        elif state == "RETURN_TO_SPAWN":
            # The navigation frame origin is the spawn pose, so the final goal
            # needs no layout information beyond the public start position.
            spawn = (0.0, 0.0)
            if source_pose is not None:
                reached = self._drive_planned_to(
                    source_pose,
                    spawn,
                    arrival_tolerance=self.return_to_spawn_tolerance,
                )
            else:
                self._stop()
                reached = False
            if reached:
                self._stop()
                self.returned_to_spawn = True
                self.two_floor_mission_complete = True
                self._set_state("RETURNED_TO_SPAWN")
                self.mission_complete_pub.publish(Bool(data=True))
                rospy.loginfo("Mission complete: returned to spawn")

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
                "elevator_portal_world": list(self.elevator_portal_world)
                if self.elevator_portal_world is not None else None,
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
