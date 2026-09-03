#!/usr/bin/env python3
"""Return to the lobby, enter the waiting elevator, ride to floor 1, and exit."""

import json
import math
import os
import sys
import threading
import ast
import numpy as np

import rospy
import tf.transformations as transformations
from building_generator_interfaces.srv import CallElevator
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
    height_transition_complete,
    normalize_angle,
    planar_distance,
    point_from_gate,
    target_heading,
)
from coverage_explorer_core import GridView, TaskCoveragePlanner


class ElevatorTransition:
    WAITING = "WAIT_FLOOR_COMPLETE"

    def __init__(self):
        self.lock = threading.RLock()
        self.enabled = bool(rospy.get_param("~enabled", True))
        self.elevator_id = str(rospy.get_param("~elevator_id", "elevator_main"))
        self.target_floor = int(rospy.get_param("~target_floor", 1))
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
            rospy.get_param("~minimum_opening_clearance", 1.55)
        )
        self.minimum_floor_rise = float(rospy.get_param("~minimum_floor_rise", 2.0))
        self.motion_speed = float(rospy.get_param("~motion_speed", 0.35))
        self.crossing_speed = float(rospy.get_param("~crossing_speed", 0.22))
        self.turn_speed = float(rospy.get_param("~turn_speed", 0.45))
        self.stop_distance = float(rospy.get_param("~stop_distance", 0.42))
        self.target_tolerance = float(rospy.get_param("~target_tolerance", 0.35))
        self.lobby_blocked_arrival_tolerance = float(
            rospy.get_param("~lobby_blocked_arrival_tolerance", 0.85)
        )
        self.heading_tolerance = float(rospy.get_param("~heading_tolerance", 0.12))

        self.state = self.WAITING
        self.pose = None
        self.source_pose = None
        self.navigation_grid = None
        self.last_pose_stamp = rospy.Time(0)
        self.gate = None
        self.gate_source = None
        self.elevator_portal = None
        self.front_clearance = float("inf")
        self.left_clearance = float("inf")
        self.right_clearance = float("inf")
        self.elevator_portal_source = None
        self.elevator_portal_world = None
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
        self.entry_heading_bias = 0.0
        self.entry_bias_anchor = None
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
            if self.floor1_context_published:
                self.floor1_complete = complete
            else:
                self.floor_complete = complete

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
            self.gate = parsed
            source_gate = payload.get("virtual_isolation_door_source")
            if isinstance(source_gate, list) and len(source_gate) >= 3:
                self.gate_source = tuple(float(value) for value in source_gate[:3])
            portal = payload.get("elevator_portal_world")
            if isinstance(portal, list) and len(portal) >= 3:
                self.elevator_portal = tuple(float(value) for value in portal[:3])
        if self.state == self.WAITING:
            self._publish_status()

    def _pose_callback(self, message):
        with self.lock:
            self.pose = (
                float(message.pose.position.x),
                float(message.pose.position.y),
                self._yaw(message.pose.orientation),
                float(message.pose.position.z),
            )
            self.last_pose_stamp = rospy.Time.now()

    def _source_pose_callback(self, message):
        with self.lock:
            self.source_pose = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                self._yaw(message.pose.pose.orientation),
                float(message.pose.pose.position.z),
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

    def _drive_to(self, pose, target, speed=None, blocked_arrival_tolerance=None):
        distance = planar_distance(pose, target)
        if distance <= self.target_tolerance:
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
        self, pose, target, speed=None, blocked_arrival_tolerance=None
    ):
        """Follow an A* route in the localization frame, with direct fallback."""
        with self.lock:
            grid = self.navigation_grid
        target_key = (round(float(target[0]), 2), round(float(target[1]), 2))
        if (
            grid is not None
            and (
                self.route_state != self.state
                or self.route_target != target_key
                or not self.route
            )
        ):
            path, _length, _clearance = self.route_planner.navigation_path(
                grid, pose[:3], target
            )
            if path:
                self.route = tuple(path)
                self.route_index = min(1, len(self.route) - 1)
                self.route_state = self.state
                self.route_target = target_key
        if self.route:
            while (
                self.route_index < len(self.route) - 1
                and planar_distance(pose, self.route[self.route_index])
                <= self.target_tolerance
            ):
                self.route_index += 1
            waypoint = self.route[self.route_index]
            final = self.route_index == len(self.route) - 1
            reached = self._drive_to(
                pose,
                waypoint,
                speed,
                blocked_arrival_tolerance if final else None,
            )
            if reached and final:
                return True
            return False
        return self._drive_to(pose, target, speed, blocked_arrival_tolerance)

    def _align(self, pose, yaw):
        error = normalize_angle(yaw - pose[2])
        if abs(error) <= self.heading_tolerance:
            self._stop()
            return True
        self._publish_command(0.0, math.copysign(self.turn_speed, error))
        return False

    def _drive_distance(self, pose, distance, speed, minimum_blocked_progress):
        if self.travel_anchor is None:
            self.travel_anchor = pose[:2]
        travelled = planar_distance(pose, self.travel_anchor)
        now = rospy.Time.now()
        if self.state == "ENTER_ELEVATOR":
            if (
                self.entry_bias_anchor is not None
                and planar_distance(pose, self.entry_bias_anchor) >= 0.35
            ):
                self.entry_heading_bias = 0.0
                self.entry_bias_anchor = None
            if self.progress_pose is None:
                self.progress_pose = pose[:2]
                self.progress_stamp = now
            elif planar_distance(pose, self.progress_pose) >= 0.20:
                self.progress_pose = pose[:2]
                self.progress_stamp = now
            elif (now - self.progress_stamp).to_sec() >= 8.0:
                if self.entry_retries >= 3:
                    self._fail("ELEVATOR_ENTRY_NO_PROGRESS")
                    return False
                # Hold a small offset long enough to walk around a jamb.  ROS
                # scan angles and angular.z are both positive to the left, so
                # turn toward the side with more measured clearance first.
                if math.isfinite(self.left_clearance) and math.isfinite(self.right_clearance):
                    direction = 1.0 if self.left_clearance >= self.right_clearance else -1.0
                else:
                    direction = 1.0
                if self.entry_retries % 2:
                    direction *= -1.0
                self.entry_heading_bias = direction * 0.24
                self.entry_bias_anchor = pose[:2]
                self.entry_retries += 1
                self.progress_stamp = now
                self.progress_pose = pose[:2]
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
        desired_heading = self.elevator_heading
        if self.state == "ENTER_ELEVATOR":
            desired_heading = normalize_angle(desired_heading + self.entry_heading_bias)
        error = normalize_angle(desired_heading - pose[2])
        if abs(error) > 0.20:
            self._publish_command(0.0, math.copysign(self.turn_speed, error))
        else:
            lateral_error = 0.0
            if self.state == "ENTER_ELEVATOR":
                if math.isfinite(self.left_clearance) and math.isfinite(self.right_clearance):
                    lateral_error = max(
                        -0.18,
                        min(0.18, 0.10 * (self.left_clearance - self.right_clearance)),
                    )
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
        if not self.enabled or self.two_floor_mission_complete or self.fault is not None:
            return
        if state == self.WAITING:
            if (floor_complete or self.start_immediately) and pose is not None and gate is not None:
                if not self._ensure_plane_policy():
                    return
                self._stop()
                self._set_state("RETURN_TO_GATE")
            return
        if pose is None or (rospy.Time.now() - self.last_pose_stamp).to_sec() > 1.0:
            self._stop()
            return

        if state == "RETURN_TO_GATE":
            route_pose = source_pose if source_pose is not None and source_gate is not None else pose
            route_gate = source_gate if source_pose is not None and source_gate is not None else gate
            target = point_from_gate(route_gate, self.gate_staging_offset)
            if self._drive_planned_to(route_pose, target):
                self._set_state("ENTER_LOBBY")
        elif state == "ENTER_LOBBY":
            # The passively mapped portal center may sit close to one jamb and
            # is not guaranteed to be directly reachable from the corridor.
            # Keep it as evidence, but approach the proven clear lobby scan
            # point before choosing the crossing heading from live lidar.
            route_pose = source_pose if source_pose is not None and source_gate is not None else pose
            route_gate = source_gate if source_pose is not None and source_gate is not None else gate
            target = point_from_gate(route_gate, self.lobby_search_offset)
            if self._drive_planned_to(
                route_pose,
                target,
                min(self.motion_speed, 0.30),
                self.lobby_blocked_arrival_tolerance,
            ):
                self.search_index = 0
                self.search_samples = []
                self._set_state("SEARCH_ELEVATOR")
        elif state == "SEARCH_ELEVATOR":
            # The building topology fixes the elevator core on the right side
            # of the entrance corridor.  Scan a local angular fan and select
            # the deepest sensor-confirmed opening; no layout coordinates are used.
            base_heading = normalize_angle(gate[2] - math.pi / 2.0)
            sample_heading = normalize_angle(base_heading + self.search_offsets[self.search_index])
            if self._align(pose, sample_heading):
                self.search_samples.append((sample_heading, self.front_clearance))
                self.search_index += 1
                if self.search_index >= len(self.search_offsets):
                    selected = choose_opening_heading(
                        self.search_samples,
                        self.minimum_opening_clearance,
                        preferred_heading=base_heading,
                    )
                    if selected is None:
                        self._fail("NO_SENSOR_CONFIRMED_ELEVATOR_OPENING")
                    else:
                        self.elevator_heading = selected
                        self.elevator_portal_source = "LASER_SCAN"
                        if self.pose is not None:
                            self.elevator_portal_world = (
                                self.pose[0], self.pose[1], selected
                            )
                        self._set_state("ALIGN_ELEVATOR")
        elif state == "ALIGN_ELEVATOR":
            if self._align(pose, self.elevator_heading):
                self.travel_anchor = pose[:2]
                self.progress_pose = pose[:2]
                self.progress_stamp = rospy.Time.now()
                self.entry_retries = 0
                self.entry_heading_bias = 0.0
                self.entry_bias_anchor = None
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
        elif state == "RIDE_TO_FLOOR_1":
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
            # The generated floors are topologically aligned in x/y.  Reuse
            # the observed floor-0 lobby/corridor gate on floor 1, then give
            # the single-floor explorer a clean floor-specific context.
            if self._drive_to(pose, gate[:2], min(self.motion_speed, 0.30)):
                self.floor1_gate = tuple(gate)
                self.floor1_topology_isolated = True
                self.transition_complete = True
                self._stop()
                self._set_state("FLOOR_1_READY")
                self.complete_pub.publish(Bool(data=True))
                self.floor1_context_published = True
                self.floor1_complete = False
                self.context_pub.publish(String(data=json.dumps({
                    "floor_index": self.target_floor,
                    "floor_z": pose[3],
                    "gate_source": list(self.gate_source or self.floor1_gate),
                    "gate_world": list(self.floor1_gate),
                }, sort_keys=True)))
        elif state == "FLOOR_1_READY":
            if self.floor1_complete:
                self._stop()
                self._set_state("RETURN_TO_FLOOR_1_GATE")
        elif state == "RETURN_TO_FLOOR_1_GATE":
            if self._drive_to(pose, self.floor1_gate[:2]):
                self._set_state("ENTER_FLOOR_1_LOBBY")
        elif state == "ENTER_FLOOR_1_LOBBY":
            if self._drive_to(
                pose, point_from_gate(self.floor1_gate, self.lobby_search_offset),
                min(self.motion_speed, 0.30),
            ):
                self.search_index = 0
                self.search_samples = []
                self._set_state("SEARCH_FLOOR_1_ELEVATOR")
        elif state == "SEARCH_FLOOR_1_ELEVATOR":
            base_heading = normalize_angle(self.floor1_gate[2] - math.pi / 2.0)
            sample_heading = normalize_angle(base_heading + self.search_offsets[self.search_index])
            if self._align(pose, sample_heading):
                self.search_samples.append((sample_heading, self.front_clearance))
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
                self._set_state("ENTER_FLOOR_1_ELEVATOR_RETURN")
        elif state == "ENTER_FLOOR_1_ELEVATOR_RETURN":
            if self._drive_distance(
                pose, self.enter_distance, self.crossing_speed,
                self.minimum_entry_progress,
            ):
                self.two_floor_mission_complete = True
                self._stop()
                self._set_state("RETURNED_TO_FLOOR_1_ELEVATOR")
                self.mission_complete_pub.publish(Bool(data=True))

        self._publish_status()

    def _publish_status(self):
        with self.lock:
            payload = {
                "state": self.state,
                "floor_complete": self.floor_complete,
                "transition_complete": self.transition_complete,
                "floor_index": self.target_floor if self.transition_complete else 0,
                "floor1_topology_isolated": self.floor1_topology_isolated,
                "floor1_complete": self.floor1_complete,
                "two_floor_mission_complete": self.two_floor_mission_complete,
                "floor1_gate": list(self.floor1_gate)
                if self.floor1_gate is not None else None,
                "target_floor": self.target_floor,
                "front_clearance": self.front_clearance,
                "state_target_distance": (
                    planar_distance(
                        self.pose,
                        point_from_gate(self.gate, self.lobby_search_offset),
                    )
                    if self.pose is not None
                    and self.gate is not None
                    and self.state == "ENTER_LOBBY"
                    else None
                ),
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
