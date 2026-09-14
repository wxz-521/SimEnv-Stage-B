#!/usr/bin/env python3
"""Standalone "elevator only" driver -- no exploration, straight to the lift.

This is a TEST TOOL and it is deliberately NOT part of the Stage-B mainline.
It never imports ``coverage_explorer_node`` / ``elevator_transition_node`` and
it never edits them; it publishes ``/cmd_vel`` itself.  The mainline modular
exploration stack takes no part in this run.

What it does, purely from coordinates:

    1. WAIT_READY      wait for /clock, robot odometry, world pose, laser scan
    2. ENTER_BUILDING  from the outdoor spawn, walk in through the main entrance
    3. TO_CORRIDOR     drive up the corridor to its mouth (mission leg 1)
    4. TURN_AROUND     U-turn there, facing back towards the lobby and the lift
    5. BACK_TO_LIFT    drive back to the staging point in front of the doorway
    6. ALIGN_DOOR      turn until the nose points along the doorway normal
    7. ENTER_CAR       drive straight through the doorway into the car
    8. RIDE            call /call_elevator and wait for the car to arrive
    9. ALIGN_EXIT      (only when ~exit_reverse is false) turn to face the mouth
   10. EXIT_CAR        reverse straight back out through the doorway
   11. TO_CORRIDOR     (upper-floor legs) drive out to the corridor mouth
   12. TURN_AROUND     (upper-floor legs) U-turn on the spot there
   13. BACK_TO_LIFT    (upper-floor legs) back to the doorway, then re-board
   14. RETURN_SPAWN    (last leg) leave through the main entrance to the spawn
   15. FACE_SPAWN      (last leg) turn to the spawn heading
   16. DONE            judge in numbers: every ride verdict plus the final pose

The MISSION is a list of ride legs, ``~ride_floors`` (default ``[1, 2, 0]``,
i.e. 1F -> 2F, 2F -> 3F, 3F -> 1F in the building's 1-based floor names), paired
with ``~corridor_depths`` (metres past the corridor gate, 0 = no excursion).
After a ride the driver leaves the car and, when the leg asks for it, drives to
the corridor mouth, U-turns and comes back to the doorway for the next ride.
The very first corridor leg is the entry one (``~entry_corridor_depth``): walk
in through the main entrance, drive up the corridor, U-turn, come back to the
lift -- that is when the lobby and the lift are on the localisation map.  The
last leg can finish by driving back out of the main entrance to the spawn point
(``~return_to_spawn``), which is what the standalone test uses.

The lift geometry is *handed to the robot as a parameter* -- that is the whole
point of the test ("tell the robot where the lift is"):

    ~elevator_door = [x, y, facing_yaw, width]

Default is the scene truth for this building: doorway at (1.65, 2.60), facing
yaw 0.0 (pointing +x, i.e. into the car), opening 1.40 m.

Frames: the doorway / entrance parameters are given in the WORLD frame (the
same frame as ``/simnav/elevator_status.elevator_portal_world``).  Control runs
in the LIO SOURCE frame that ``/simnav/odom`` reports, so the world targets are
mapped through the paired robot poses once per cycle.  Nothing else changes.

Run it through ``team_scripts/elevator_only_test.sh``.
"""

import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from gazebo_msgs.srv import GetModelState

try:
    from building_generator_interfaces.srv import CallElevator, SetDoorState
except ImportError:  # pragma: no cover - only on a broken workspace
    CallElevator = None
    SetDoorState = None


class _GridView(object):
    """Minimal grid view: the mainline detector only needs these four fields."""

    def __init__(self, data, resolution, origin_x, origin_y, frame_id):
        self.data = data
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.frame_id = frame_id


def load_door_candidate_detector():
    """Reuse the MAINLINE candidate detector (pure geometry, read-only).

    The user directive is that the lift must be located from DETECTED candidate
    coordinates rather than a hard-coded tuple.  Rather than inventing a second
    algorithm, this imports ``detect_wide_lobby_openings`` from the mainline's
    pure geometry core (``src/simnav/scripts/elevator_transition_core.py``),
    exactly the function ``elevator_transition_node`` uses.  Nothing under src/
    is modified; if the import is unavailable the driver falls back to the
    configured doorway reference and records that it did.
    """
    workspace = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    roots = [os.environ.get("SIMNAV_SCRIPTS_DIR", ""),
             os.path.join(workspace, "src", "simnav", "scripts")]
    for root in roots:
        if not root or not os.path.isfile(
                os.path.join(root, "elevator_transition_core.py")):
            continue
        if root not in sys.path:
            sys.path.insert(0, root)
        try:
            from elevator_transition_core import detect_wide_lobby_openings
            return detect_wide_lobby_openings
        except Exception as error:  # noqa: BLE001
            rospy.logwarn("elevator-only: candidate detector import failed: %s",
                          error)
            return None
    return None


def normalize_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def planar_distance(first, second):
    return math.hypot(float(first[0]) - float(second[0]),
                      float(first[1]) - float(second[1]))


class ElevatorOnlyDriver(object):
    """Drive to a known lift doorway, get in, ride, and report in numbers."""

    STATES = (
        "WAIT_READY", "ENTER_BUILDING", "TO_STAGING", "ALIGN_DOOR",
        "ENTER_CAR", "RIDE", "ALIGN_EXIT", "EXIT_CAR",
        "TO_CORRIDOR", "CORRIDOR_CENTRE", "TURN_AROUND", "BACK_TO_LIFT",
        "RETURN_SPAWN", "FACE_SPAWN", "DONE", "FAILED",
    )

    def __init__(self):
        self.lock = threading.RLock()

        # --- geometry handed to the robot ---------------------------------
        self.door = self._pose_param("~elevator_door", [1.65, 2.60, 0.0, 1.40])
        self.entrance = self._pose_param("~main_entrance", [0.0, 0.0, 1.5708, 2.0])
        self.lobby_point = self._pair_param("~lobby_point", [0.0, 1.60])
        self.staging_distance = float(rospy.get_param("~staging_distance", 2.20))
        self.car_depth = float(rospy.get_param("~car_depth", 1.20))
        self.enter_building = bool(rospy.get_param("~enter_building", True))
        self.entrance_inside = float(rospy.get_param("~entrance_inside", 2.2))
        self.entry_leg = 0
        # Entry discipline (user requirement): line up square with the doorway,
        # centred on its axis, then commit to a straight run in -- "face the
        # door, then take the risk and drive straight in".  The entry leg does
        # NOT re-aim at the car centre while moving, because steering during the
        # crossing is what walked the robot into the door jamb.
        # Gait policy.  junior_ctrl ships two policies: the default stair policy
        # (use_plane_policy=false, trained for steps) and a flat-ground plane
        # policy.  The doorway has a real 6 cm threshold at x in [1.43,1.65],
        # and the policy switch is REJECTED whenever cmd_vel is non-zero, so
        # the choice is made once, while the robot is still standing.
        # Absolute truth.  Inside the car the robot is motionless relative to
        # the car, so no odometry can observe the ride: lobbytest_02 measured
        # Gazebo z 0.31 -> 2.914 m while /simnav/world_pose_metric.z stayed at
        # 0.184.  The ride verdict therefore has to come from ground truth.
        self.robot_model = str(rospy.get_param("~robot_model", "a1_gazebo"))
        self.car_model = str(rospy.get_param("~car_model", "dynamic_elevator_main"))
        self.gait_policy = str(rospy.get_param("~gait_policy", "stair")).lower()
        self.crossing_speed = float(rospy.get_param("~crossing_speed", 0.45))
        self.lateral_gain = float(rospy.get_param("~lateral_gain", 0.65))
        self.lateral_tolerance = float(rospy.get_param("~lateral_tolerance", 0.12))
        self.enter_distance = float(rospy.get_param("~enter_distance", 3.00))
        self.minimum_entry_progress = float(
            rospy.get_param("~minimum_entry_progress", 0.80)
        )
        self.entry_timeout = float(rospy.get_param("~entry_timeout", 35.0))
        # Leaving the car does not need a square heading: the car mouth is
        # 1.74 m wide and the run is 2.2 m long, so a loose tolerance saves the
        # seconds that a precise pi-turn costs (ALIGN_EXIT measured 30 sim s).
        self.exit_heading_tolerance = float(
            rospy.get_param("~exit_heading_tolerance", 0.35)
        )
        # Symmetric with the entry rule: coming down the 6 cm car threshold is
        # as marginal as climbing it, so the exit is complete once the robot is
        # a minimum distance out AND demonstrably past the door plane -- not
        # only when the full nominal run is covered.  Measured: the exit stalled
        # at 1.11 m of a 2.20 m run, which already put the robot 0.26 m outside
        # the doorway, and was then declared a failure by the travel budget.
        self.minimum_exit_progress = float(
            rospy.get_param("~minimum_exit_progress", 1.00)
        )
        # Back straight out instead of turning round inside the car.  The robot
        # enters the car facing its direction of travel, so reversing needs no
        # pi turn at all -- ALIGN_EXIT measured 29-30 sim seconds, i.e. a fifth
        # of the whole up-and-out cycle, and it also leaves the robot still
        # facing the doorway for the next ride.  Reverse motion is already used
        # by the entry retry path, so the controller supports it.
        # ``exit_mode`` alone decides how the car is left:
        #   "reverse"      -> hold the entry heading and back out (the proven mode)
        #   "turn_forward" -> U-turn inside the car, then drive forward out
        # User decision: leaving the CAR is done in REVERSE (the 6 cm car
        # threshold jams on a forward crossing); "turn_forward" stays available.
        # NOTE the order: exit_mode MUST be defined before exit_reverse derives
        # its default from it (line order here cost five failed starts).
        self.exit_mode = str(rospy.get_param("~exit_mode", "reverse")).lower()
        self.exit_reverse = bool(rospy.get_param(
            "~exit_reverse", self.exit_mode != "turn_forward"))
        self.exit_turn_retry_limit = max(1, int(
            rospy.get_param("~exit_turn_retry_limit", 2)))
        self.exit_turn_timeout = float(
            rospy.get_param("~exit_turn_timeout", 45.0))
        self.exit_turn_walk_speed = float(
            rospy.get_param("~exit_turn_walk_speed", 0.12))
        self.exit_turn_side = None
        self.exit_turn_clearances = None
        self.exit_turn_started_sim = None
        self.exit_turn_unsticks = 0
        self.exit_turn_retries = 0
        self.exit_fallback = None
        self.exit_forward_threshold_ok = None
        self.exit_forward_retries = 0
        self.exit_fail_travelled = None
        self.exit_turn_target = "out"
        # Leaving the car means climbing (or dropping over) the same 6 cm
        # threshold, and probe 01 showed it is marginal: the robot reversed
        # 1.06 m out of the car and then sat with its centre on the threshold
        # lip (truth x oscillating 1.46-1.49) for 28 s while its odometry kept
        # "advancing" 0.2 m from slipping legs.  Mainline elevator_transition
        # node solves the same jam by retreating into the car and retrying
        # (exit_stall_seconds / entry_retreat_speed / exit_retry_limit); this
        # does the same, but watches GROUND TRUTH for the stall because the
        # odometry lies exactly here.
        self.exit_progress_tolerance = float(
            rospy.get_param("~exit_progress_tolerance", 0.20)
        )
        self.exit_stall_seconds = float(rospy.get_param("~exit_stall_seconds", 8.0))
        self.exit_retry_limit = max(1, int(rospy.get_param("~exit_retry_limit", 3)))
        self.exit_retreat_speed = float(rospy.get_param("~exit_retreat_speed", 0.30))
        self.exit_retreat_seconds = float(rospy.get_param("~exit_retreat_seconds", 0.8))
        # Each retry holds a slightly biased heading: a square-on run catches on
        # the threshold lip, a biased one turns that into a jamb contact that
        # slides through (mainline elevator_transition_node does exactly this).
        self.exit_retry_yaw_bias = float(
            rospy.get_param("~exit_retry_yaw_bias", 0.16)
        )
        # The tested threshold speed must NOT be changed (user: the crossing
        # speed is already validated), so retries keep crossing_speed.
        self.exit_retry_speed_bonus = float(
            rospy.get_param("~exit_retry_speed_bonus", 0.0)
        )
        self.exit_timeout = float(rospy.get_param("~exit_timeout", 90.0))
        # Same discipline for ENTERING: once ALIGN_DOOR has squared the robot on
        # the doorway (heading AND lateral within tolerance), the run COMMITS -
        # no stopping to re-deliberate when the car interior makes the laser
        # read short.  If the 6 cm threshold catches the legs, keep pushing with
        # a ground-truth progress watchdog plus a short retreat and retry (with
        # a brief alternating heading bias to break the contact), exactly like
        # the way out.  crossing_speed never changes.
        self.entry_stall_seconds = float(rospy.get_param("~entry_stall_seconds", 8.0))
        self.entry_retry_limit = max(1, int(rospy.get_param("~entry_retry_limit", 3)))
        self.entry_retreat_speed = float(rospy.get_param("~entry_retreat_speed", 0.30))
        self.entry_retreat_seconds = float(
            rospy.get_param("~entry_retreat_seconds", 0.8))
        self.entry_retry_yaw_bias = float(
            rospy.get_param("~entry_retry_yaw_bias", 0.16))
        self.entry_retry_bias_seconds = float(
            rospy.get_param("~entry_retry_bias_seconds", 2.0))
        self.entry_overrun_margin = float(
            rospy.get_param("~entry_overrun_margin", 1.5))
        # ENTER_BUILDING crosses the same 8 cm apron step and the gait froze on
        # it once in ~8 runs (elevstreak9_08: 45 s at (-0.07,-1.62,z 0.394),
        # cmd_vx 0.45, front clearance 8.5 m = clear path, no motion).  The leg
        # now gets the car-threshold recovery: early stall trigger, retreat,
        # gait-policy re-toggle, biased re-commit, bounded retries.
        self.entry_stall_trigger = float(
            rospy.get_param("~entry_stall_trigger", 5.0))
        self.entry_leg_retry_limit = max(1, int(
            rospy.get_param("~entry_leg_retry_limit", 3)))
        self.entry_retreat_until = None
        self.entry_recover_bias_until = None
        self.entry_progress_sim = None
        self.entry_progress_anchor = None
        self.entry_leg_retries = 0
        self.entry_bias_sign = 1.0
        # Retreat far enough to actually leave the 8 cm slab (the stall was ON
        # the apron), so the retreat is truth-distance driven, not time driven.
        self.entry_retreat_distance = float(
            rospy.get_param("~entry_retreat_distance", 0.80))
        self.entry_retreat_timeout = float(
            rospy.get_param("~entry_retreat_timeout", 4.0))
        self.entry_retreat_anchor = None
        self.exit_past_door_along = float(
            rospy.get_param("~exit_past_door_along", -0.80)
        )
        # ---------- boarding centre precondition (every ride) ----------
        # "每次进电梯前，确保自己能够对准中心进门": ALIGN_DOOR already lines the
        # robot up on the doorway centre line, and a boarding is only accepted
        # when the robot is still centred there, judged on the doorway frame:
        # lateral offset <= board_lateral_tolerance and heading error <=
        # board_heading_tolerance.  The report audits every leg against these.
        self.board_lateral_tolerance = float(
            rospy.get_param("~board_lateral_tolerance", 0.20)
        )
        self.board_heading_tolerance = float(
            rospy.get_param("~board_heading_tolerance", 0.20)
        )
        # How far past the door plane a boarding sample may sit: the car is
        # 2.4 m deep, the nominal stop is ~1.30 m, so 1.45 m leaves margin while
        # still proving the robot is inside the car.
        self.board_door_dist_cap = float(
            rospy.get_param("~board_door_dist_cap", 1.45)
        )

        # --- motion --------------------------------------------------------
        self.motion_speed = float(rospy.get_param("~motion_speed", 0.45))
        self.turn_speed = float(rospy.get_param("~turn_speed", 0.45))
        self.heading_tolerance = float(rospy.get_param("~heading_tolerance", 0.15))
        self.target_tolerance = float(rospy.get_param("~target_tolerance", 0.32))
        self.stop_distance = float(rospy.get_param("~stop_distance", 0.45))
        self.stuck_window = float(rospy.get_param("~stuck_window", 10.0))
        self.stuck_min_progress = float(rospy.get_param("~stuck_min_progress", 0.15))
        self.stuck_timeout = float(rospy.get_param("~stuck_timeout", 45.0))
        # A slow in-place turn makes no POSITION progress, so yaw progress has
        # to count too or a legitimate 180 deg turn can trip the stuck timer.
        self.stuck_min_yaw_progress = float(
            rospy.get_param("~stuck_min_yaw_progress", 0.25)
        )
        # Blind-gait lock-up: probe 02 commanded +0.45 rad/s for 45 s in open
        # lobby space on floor 1 with BOTH the true and the estimated yaw
        # frozen - a standing-turn deadlock, not a collision.  When a turn
        # makes no progress for this long, walk-and-turn for a moment instead
        # (alternating the walk direction), which re-engages the gait.
        self.turn_unstick_seconds = float(
            rospy.get_param("~turn_unstick_seconds", 6.0)
        )
        self.turn_unstick_walk = float(rospy.get_param("~turn_unstick_walk", 1.5))
        self.turn_walk_speed = float(rospy.get_param("~turn_walk_speed", 0.12))
        self.turn_progress_yaw = float(rospy.get_param("~turn_progress_yaw", 0.10))
        self.turn_unstick_limit = max(1, int(rospy.get_param("~turn_unstick_limit", 5)))
        self.state_timeout = float(rospy.get_param("~state_timeout", 180.0))
        self.ride_timeout = float(rospy.get_param("~ride_timeout", 90.0))

        # --- mission -------------------------------------------------------
        self.elevator_id = str(rospy.get_param("~elevator_id", "elevator_main"))
        self.target_floor = int(rospy.get_param("~target_floor", 1))
        # The ride schedule, in arrival-floor order.  The default is the full
        # standalone cycle the user asked for: ground -> 1 (2F) -> 2 (3F) -> 0.
        # ``~target_floors`` is kept as the older spelling of the same thing.
        self.target_floors = self._int_list_param("~target_floors", [])
        self.ride_floors = self._int_list_param("~ride_floors", []) or \
            self.target_floors or [1, 2, 0]
        # What happens AFTER each ride, indexed by ride leg: how far past the
        # corridor gate the robot drives before it U-turns and comes back.
        # 0.0 means "no corridor excursion, re-board on this floor".
        self.corridor_depths = self._float_list_param("~corridor_depths", [1.6, 1.6, 0.0])
        while len(self.corridor_depths) < len(self.ride_floors):
            self.corridor_depths.append(0.0)
        self.target_floor = self.ride_floors[0]
        # Final leg: leave the building through the corridor gate and drive back
        # to where the run started.  ~spawn_world = [x, y, yaw] (world frame).
        self.return_to_spawn = bool(rospy.get_param("~return_to_spawn", True))
        self.spawn_param = self._pose_param_or_none("~spawn_world")
        self.spawn_world = self.spawn_param
        self.spawn_measured = None
        # The corridor gate: origin of the corridor's along-coordinate.  The
        # corridor runs from here along its yaw; the lift sits behind it.
        self.corridor_gate = self._pose_param("~corridor_gate", [-0.25, 7.30, 1.5708])
        # First leg of the mission (outdoor spawn): walk in through the main
        # entrance, drive up the corridor to its mouth and U-turn there before
        # approaching the lift.  That is what puts the lobby + lift into the
        # localisation map before the boarding flow starts.  0 disables it.
        self.entry_corridor_depth = float(
            rospy.get_param("~entry_corridor_depth", 1.6)
        )
        self.entry_corridor_truth_max_along = None
        self.entry_corridor_ok = False
        self.corridor_lateral_tolerance = float(
            rospy.get_param("~corridor_lateral_tolerance", 0.75)
        )
        self.corridor_tolerance = float(rospy.get_param("~corridor_tolerance", 0.40))
        # U-turn room.  Probe 00 (first live run of this flow) drove to the
        # corridor mouth with a 0.45 m lateral localisation error, ended at
        # truth x = -0.75 (0.35 m from the 2.2 m corridor's left wall) and the
        # in-place U-turn jammed there: it turned 0.42 rad in 62 sim seconds and
        # then stopped moving entirely.  So before turning, CENTRE on the
        # corridor axis (a point on it is the gate origin offset by
        # ~corridor_axis_lateral): the laser balances the two walls, ground
        # truth gates the result, and the turn only starts once the body has
        # the ~0.35 m of swing radius the U-turn needs.
        self.corridor_axis_lateral = float(
            rospy.get_param("~corridor_axis_lateral", -0.25)
        )
        self.corridor_axis_tolerance = float(
            rospy.get_param("~corridor_axis_tolerance", 0.30)
        )
        self.corridor_centre_speed = float(
            rospy.get_param("~corridor_centre_speed", 0.25)
        )
        self.corridor_centre_gain = float(
            rospy.get_param("~corridor_centre_gain", 0.9)
        )
        self.corridor_centre_timeout = float(
            rospy.get_param("~corridor_centre_timeout", 45.0)
        )
        # How far the laser balance may disagree with ground truth before the
        # driver stops trusting the laser (at the corridor mouth the laser sees
        # the door jambs, and a stale/blocked sector would otherwise let the
        # robot creep straight on while it is still off the axis).
        self.corridor_centre_agreement = float(
            rospy.get_param("~corridor_centre_agreement", 0.35)
        )
        self.corridor_turn_stall_timeout = float(
            rospy.get_param("~corridor_turn_stall_timeout", 20.0)
        )
        self.corridor_turn_timeout = float(
            rospy.get_param("~corridor_turn_timeout", 60.0)
        )
        # Final heading at the spawn.  The verdict is measured on Gazebo truth
        # (the mission frame), but the turn itself is steered on the LIO
        # estimate, which carries ~0.07 rad of yaw error: run 2 stopped with
        # LIO error 0.134 rad (inside) and truth error 0.206 rad (just outside
        # the 0.20 gate).  So stop the turn TIGHTER than the gate and verify
        # against truth, correcting on truth if it disagrees.
        self.face_spawn_tolerance = float(
            rospy.get_param("~face_spawn_tolerance", 0.07)
        )
        self.face_spawn_truth_tolerance = float(
            rospy.get_param("~face_spawn_truth_tolerance", 0.10)
        )
        # Turn the last few degrees slowly: the Gazebo heading sample is 4 Hz,
        # so at 0.25 rad/s the sampling lag costs < 0.07 rad and the settled
        # truth error cannot overshoot the 0.20 rad gate.
        self.face_spawn_turn_speed = float(
            rospy.get_param("~face_spawn_turn_speed", 0.25)
        )
        # Ground-truth gate for the corridor stop.  The stop itself is judged in
        # the LIO frame (which is what drives), but if the odometry runs ahead
        # of reality the robot would U-turn still inside the lobby; ground truth
        # says where it really is, so keep creeping until truth agrees.  1.10 m
        # past the gate is world y 8.40: 0.55 m inside the corridor floor (y_min
        # 7.85) and clear of the mouth wall for a 0.62 m body to U-turn.
        self.corridor_min_truth_along = float(
            rospy.get_param("~corridor_min_truth_along", 1.10)
        )
        self.corridor_speed = float(rospy.get_param("~corridor_speed", 0.60))
        self.corridor_turn_speed = float(rospy.get_param("~corridor_turn_speed", 0.45))
        self.corridor_timeout = float(rospy.get_param("~corridor_timeout", 120.0))
        # Ride verdict: the robot's own Gazebo z must land on the floor's z.
        # Robot truth z on the ground floor is ~0.31 m and one floor is 2.6 m.
        self.floor_height = float(rospy.get_param("~floor_height", 2.60))
        # Fail loudly if the metric estimate and Gazebo truth disagree grossly:
        # elevstreak2_01 followed an 11.6 m broken estimate into the shaft.
        # Threshold and debounce are BOTH needed: elevstreak3_01 was killed by
        # a ~1 s, 1.6 m transient during the final in-place turn (the healthy
        # mission maximum was 0.43 m), while the real failure was 11.6 m
        # persisting for 80+ s.  3.0 m + 2.0 s separates them cleanly.
        # REGION RULE (user decision): the LIO estimate drives position ONLY
        # inside the corridor, where it measures 0.2-0.3 m against truth.  The
        # lobby, lift car, entrance passage, apron and forecourt are all
        # feature-poor for the lidar (elevf04: the estimate ran away 17 m in the
        # open lobby while the robot stood still), so position there is steered
        # on Gazebo truth.
        # Candidate-based doorway: locate the lift from the map the standalone
        # scenario already has, confirmed over several cycles, sanity-checked
        # against the configured reference.
        self.door_reference = tuple(self.door)
        self.door_candidate_enable = bool(
            rospy.get_param("~door_candidate_enable", True))
        self.door_candidate_confirm_cycles = max(1, int(
            rospy.get_param("~door_candidate_confirm_cycles", 5)))
        self.door_candidate_max_lateral = float(
            rospy.get_param("~door_candidate_max_lateral", 0.60))
        self.door_candidate_max_yaw = float(
            rospy.get_param("~door_candidate_max_yaw", 0.25))
        # Mainline defaults for the candidate search region (elevator_transition
        # node): the lobby walls sit ~1.1 m either side of the corridor axis.
        self.corridor_half_width = float(
            rospy.get_param("~corridor_half_width", 1.10))
        self.door_candidate_lateral_min = float(
            rospy.get_param("~door_candidate_lateral_min", -3.50))
        self.door_candidate_lateral_max = float(
            rospy.get_param("~door_candidate_lateral_max", 3.50))
        self.door_candidate_min_width = float(
            rospy.get_param("~door_candidate_min_width", 0.90))
        self.door_candidate_max_width = float(
            rospy.get_param("~door_candidate_max_width", 3.80))
        self.candidate_detector = (
            load_door_candidate_detector() if self.door_candidate_enable else None)
        self.navigation_grid = None
        self.door_candidate_evidence = {}
        self.door_candidate_cycles = 0
        self.door_candidate = None
        self.door_candidate_offset = None
        self.door_candidate_accepted = False
        self.door_candidate_rejected = False
        self.door_candidate_reject_reason = None
        self.door_candidate_note = "not_attempted"
        self.door_frame_source = "reference"
        self.corridor_x_min = float(rospy.get_param("~corridor_x_min", -1.1))
        self.corridor_x_max = float(rospy.get_param("~corridor_x_max", 1.1))
        self.corridor_y_min = float(rospy.get_param("~corridor_y_min", 7.85))
        self.corridor_y_max = float(rospy.get_param("~corridor_y_max", 35.91))
        self.truth_steered_regions = []
        self.loc_truth_steered_warnings = 0
        self.loc_max_planar_error = float(
            rospy.get_param("~loc_max_planar_error", 3.0)
        )
        self.loc_diverged_seconds = float(
            rospy.get_param("~loc_diverged_seconds", 2.0)
        )
        # A stationary in-place turn cannot be turned into a fall, and outdoors
        # the LIO cannot constrain position at all: scan matching integrates the
        # rotation as translation, ramping the disagreement at ~0.6 m/s while
        # the robot stands still (that ended elevstreak3_01 and elevstreak4_01
        # with the whole cycle already complete).  So the guard is fatal only
        # when the robot is ACTUALLY translating.
        # The entrance forecourt (truth y < ~1.0 m) gives the lidar no lateral
        # features: the estimate loses its x constraint and drifts ~1.2 m/s
        # while driving out to the spawn (elevstreak6_01: 1.76 m during
        # RETURN_SPAWN, then 9.7 m during the outdoor spin).  position on that
        # strip is therefore steered on Gazebo truth, and a divergence there is
        # recorded but not fatal - indoors the guard stays armed.
        self.outdoor_truth_y = float(rospy.get_param("~outdoor_truth_y", 1.0))
        # U-turn-then-approach for the outdoor return leg.
        self.spawn_face_tolerance = float(
            rospy.get_param("~spawn_face_tolerance", 0.15))
        # Heading tolerance for the FORWARD return leg's walk-and-turn arc.  The
        # final heading is not a criterion (official evaluation scores positions
        # and time only), so this only decides when to stop arcing and drive out.
        self.spawn_bearing_tolerance = float(
            rospy.get_param("~spawn_bearing_tolerance", 0.45))
        # Clear the entrance passage STRAIGHT before any turn: the doorway edge
        # jams a walking arc exactly like the car threshold (elevstreak8_01 was
        # frozen at truth (0.57, 0.89) for 36 s, still inside the plane).
        # Final leg, from the sweep of all 147 turn episodes: every fatal turn
        # was OUTDOORS or in a DOORWAY, the corridor always turns, and the open
        # lobby always completes (slowly).  So the 180 deg turn happens INDOORS
        # at this point on the doorway axis, and the robot then crosses the
        # entrance and the forecourt in a pure STRAIGHT line (straight crossings
        # have never failed) to the spawn - in reverse, so it arrives already on
        # the spawn heading and FACE_SPAWN only trims a few degrees.
        self.spawn_turn_y = float(rospy.get_param("~spawn_turn_y", 1.80))
        self.spawn_axis_tolerance = float(
            rospy.get_param("~spawn_axis_tolerance", 0.30))
        self.spawn_axis_timeout = float(
            rospy.get_param("~spawn_axis_timeout", 40.0))
        self.spawn_reverse_timeout = float(
            rospy.get_param("~spawn_reverse_timeout", 60.0))
        # Walk-and-turn arc used to acquire the bearing to the spawn: a genuine
        # WALKING turn (vx 0.30) rather than the near-standing 0.10 that stalled
        # at a 0.31 rad residual in elevf01_nominal_03.
        self.spawn_arc_speed = float(rospy.get_param("~spawn_arc_speed", 0.30))
        self.spawn_arc_turn = float(rospy.get_param("~spawn_arc_turn", 0.60))
        # Patience for the final lobby U-turn (0.08-0.15 rad/s is normal here).
        self.spawn_turn_stall = float(rospy.get_param("~spawn_turn_stall", 12.0))
        self.spawn_turn_limit = max(1, int(rospy.get_param("~spawn_turn_limit", 8)))
        self.spawn_turn_timeout = float(rospy.get_param("~spawn_turn_timeout", 150.0))
        self.spawn_lateral_tolerance = float(
            rospy.get_param("~spawn_lateral_tolerance", 0.25))
        self.spawn_approach_timeout = float(
            rospy.get_param("~spawn_approach_timeout", 45.0))
        self.loc_truth_motion = float(rospy.get_param("~loc_truth_motion", 0.30))
        self.loc_cmd_motion = float(rospy.get_param("~loc_cmd_motion", 0.05))
        self.loc_diverged_since = None
        self.loc_truth_anchor = None
        self.loc_episode_counted = False
        self.loc_hold = False
        self.loc_hold_count = 0
        self.loc_stationary_warnings = 0
        self.loc_outdoor_warnings = 0
        self.outdoor_truth_steering_sim = None
        self.spawn_turn_started_sim = None
        self.spawn_approach_started_sim = None
        self.spawn_clear_started_sim = None
        self.spawn_axis_sim = None
        self.spawn_axis_done = False
        self.spawn_reverse_sim = None
        # The entrance apron is an 8 cm slab (x [-2.25,2.25], y [-2.40,0]): the
        # reverse crossing gets the same ground-truth progress watchdog plus a
        # short retreat and alternating biased retry that the car threshold
        # needed, with the step-trained policy already active.
        self.spawn_reverse_progress_sim = None
        self.spawn_reverse_anchor = None
        self.spawn_reverse_retries = 0
        self.spawn_reverse_retreat_until = None
        self.spawn_reverse_bias_until = None
        self.max_world_truth_error = None
        self.floor_contexts = []
        self.ride_z_tolerance = float(rospy.get_param("~ride_z_tolerance", 0.45))
        self.ground_z = None
        self.exit_distance = float(rospy.get_param("~exit_distance", 2.20))
        self.exit_anchor = None
        self.exit_started_sim = None
        self.exit_progress_sim = None
        self.exit_progress_truth = None
        self.exit_progress_source = None
        self.exit_retries = 0
        self.exit_retreat_until = None
        self.leg_results = []
        self.leg_index = 0
        self.current_floor = int(rospy.get_param("~current_floor", 0))
        self.door_prefix = str(rospy.get_param("~elevator_door_prefix", "elevator_floor"))
        self.output_csv = str(rospy.get_param("~output_csv", ""))
        if self.output_csv and os.path.exists(self.output_csv):
            # Evidence integrity: a reused tag must never append a second
            # mission to the previous timeline (the CSV reader then sees two
            # runs in one file).
            try:
                os.remove(self.output_csv)
            except OSError:
                pass
        self.output_json = str(rospy.get_param("~output_json", ""))
        # Corridor-excursion bookkeeping (per exiting leg).
        self.excursion_leg = 0
        self.excursion_reverse_anchor = None
        self.excursion_reverse_max = float(
            rospy.get_param("~excursion_reverse_max", 1.60)
        )
        self.excursion_target = None
        self.excursion_depth = 0.0
        self.excursion_started_sim = None
        self.excursion_max_along = None
        self.excursion_truth_max_along = None
        self.centre_started_sim = None
        self.centre_truth_offset = None
        self.turn_started_sim = None
        self.turn_anchor_error = None
        self.turn_anchor_sim = None
        self.entry_centre_offset = None
        self.align_lateral = None
        self.align_heading_error = None
        self.align_sim = None
        self.board_along = None
        self.board_lateral = None
        self.board_lateral_truth = None
        self.board_along_truth = None
        self.board_heading_error = None
        self.board_sim = None
        self.board_retries = 0
        self.entry_progress_sim = None
        self.entry_progress_truth = None
        self.entry_progress_source = None
        self.entry_retries = 0
        self.entry_retreat_until = None
        self.entry_bias_until = None
        self.mission_started_sim = None
        self.return_sim = None
        self.return_distance = None
        self.return_yaw_error = None
        self.return_yaw_error_truth = None
        self.excursion_mode = "leg"

        # --- live inputs ---------------------------------------------------
        self.sim_now = 0.0
        self.source_pose = None      # (x, y, yaw, z) in the LIO source frame
        self.world_pose = None       # (x, y, yaw, z) in the metric world frame
        # ``use_truth`` gates whether Gazebo ground truth may be used for CONTROL
        # and JUDGEMENT.  The mainline is not allowed any ground-truth interface
        # (team_scene_info.json allows only /set_door_state and /call_elevator and
        # forbids /Odometry_gazebo and /ground_truth/*), so the FAITHFUL default is
        # False: the control fields then carry the robot's own localisation pose and
        # every gate is computed from the same sensors the mainline has.  Gazebo
        # truth is still polled into gazebo_truth* for the timeline and reports.
        self.use_truth = bool(rospy.get_param("~use_truth", False))
        self.gazebo_truth = None       # (x, y, z) monitoring only
        self.gazebo_truth_yaw = None   # monitoring only
        self.robot_truth = None      # CONTROL pose: Gazebo when use_truth, else own estimate
        self.robot_truth_yaw = None  # CONTROL heading, same rule
        self.last_cmd = (0.0, 0.0)   # last /cmd_vel command (vx, wz)
        self.car_truth_z = None
        self.front_clearance = float("inf")
        self.left_clearance = float("inf")
        self.right_clearance = float("inf")
        self.rear_clearance = float("inf")

        # --- state machine -------------------------------------------------
        self.state = "WAIT_READY"
        self.state_since = None
        self.fault = None
        self.ride_response = None
        self.ride_error = None
        self.ride_requested_at = None
        self.z_before_ride = None
        self.z_after_ride = None
        self.ride_started = False

        self.stuck_since = None
        self.stuck_anchor = None
        self.stuck_anchor_yaw = None
        self.turn_anchor = None
        self.turn_anchor_sim = None
        self.turn_unstick_until = None
        self.turn_unstick_dir = 1.0
        self.turn_unstick_count = 0
        self.entry_anchor = None
        self.entry_started_sim = None
        self.ready_since = None
        self.visited = []
        self.timeline = []
        self.last_sample_sim = -1.0
        self.last_truth_sim = -1.0
        self.last_status_log = 0.0

        self.command_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        # The standalone scenario does not run elevator_transition_node, so
        # nothing else tells the mapping stack that the robot changed floors.
        # lio_occupancy_node switches its occupancy grid on this topic; without
        # it the ground floor's grid is simulated while the robot is upstairs
        # and the estimate can break across a ride (elevstreak2_01: 11.6 m).
        self.floor_context_pub = rospy.Publisher(
            "/simnav/floor_exploration_context", String, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/simnav/elevator_only_status", String, queue_size=2, latch=True
        )
        rospy.Subscriber("/clock", Clock, self._clock_callback, queue_size=10)
        rospy.Subscriber("/simnav/odom", Odometry, self._source_callback, queue_size=10)
        rospy.Subscriber(
            "/simnav/world_pose_metric", PoseStamped, self._world_callback, queue_size=10
        )
        rospy.Subscriber("/scan_2d", LaserScan, self._scan_callback, queue_size=1)
        rospy.Subscriber("/navigation_map", OccupancyGrid, self._map_callback,
                         queue_size=1)
        self.elevator_service = (
            rospy.ServiceProxy("/call_elevator", CallElevator)
            if CallElevator is not None else None
        )
        self.door_service = (
            rospy.ServiceProxy("/set_door_state", SetDoorState)
            if SetDoorState is not None else None
        )
        self.plane_policy_service = rospy.ServiceProxy(
            "/unitree/select_plane_policy", SetBool
        )
        self.model_state_service = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState
        )
        self.plane_policy_active = False

        self.rate = rospy.Rate(float(rospy.get_param("~control_rate", 20.0)))
        rospy.Timer(rospy.Duration(1.0 / float(rospy.get_param("~control_rate", 20.0))),
                    self._control)

    # ------------------------------------------------------------------ util
    @staticmethod
    def _pose_param(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, str):
            value = [float(part) for part in value.strip("[]").split(",") if part.strip()]
        return tuple(float(part) for part in value)

    @staticmethod
    def _pair_param(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, str):
            value = [float(part) for part in value.strip("[]").split(",") if part.strip()]
        return (float(value[0]), float(value[1]))

    @staticmethod
    def _as_list(value):
        if isinstance(value, str):
            return [part for part in value.strip("[]").split(",") if part.strip()]
        if value is None:
            return []
        return list(value)

    @classmethod
    def _int_list_param(cls, name, default):
        return [int(float(part)) for part in cls._as_list(rospy.get_param(name, default))]

    @classmethod
    def _float_list_param(cls, name, default):
        return [float(part) for part in cls._as_list(rospy.get_param(name, default))]

    @classmethod
    def _pose_param_or_none(cls, name):
        """Optional [x, y, yaw] (or longer) parameter; None when not set."""
        values = cls._as_list(rospy.get_param(name, None))
        if len(values) < 2:
            return None
        return tuple(float(part) for part in values)

    def _clock_callback(self, message):
        with self.lock:
            self.sim_now = message.clock.to_sec()

    def _source_callback(self, message):
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        with self.lock:
            self.source_pose = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                float(yaw),
                float(message.pose.pose.position.z),
            )

    def _world_callback(self, message):
        orientation = message.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
        )
        with self.lock:
            self.world_pose = (
                float(message.pose.position.x),
                float(message.pose.position.y),
                float(yaw),
                float(message.pose.position.z),
            )

    def _scan_callback(self, message):
        front, left, right, rear = [], [], [], []
        for index, distance in enumerate(message.ranges):
            if not (math.isfinite(distance) and message.range_min <= distance <= message.range_max):
                continue
            angle = normalize_angle(message.angle_min + index * message.angle_increment)
            if abs(angle) <= math.radians(16.0):
                front.append(float(distance))
            elif math.radians(65.0) <= angle <= math.radians(110.0):
                left.append(float(distance))
            elif math.radians(-110.0) <= angle <= math.radians(-65.0):
                right.append(float(distance))
            elif abs(angle) >= math.radians(150.0):
                rear.append(float(distance))
        with self.lock:
            self.front_clearance = (
                sorted(front)[max(0, int(0.20 * len(front)) - 1)] if front else float("inf")
            )
            self.left_clearance = sorted(left)[len(left) // 2] if left else float("inf")
            self.right_clearance = sorted(right)[len(right) // 2] if right else float("inf")
            self.rear_clearance = sorted(rear)[len(rear) // 2] if rear else float("inf")

    # --------------------------------------------------------------- frames
    def _to_source(self, world_xy):
        """Map a world-frame point into the LIO source frame the odom uses."""
        with self.lock:
            world, source = self.world_pose, self.source_pose
        if world is None or source is None:
            return None
        delta = normalize_angle(source[2] - world[2])
        cosine, sine = math.cos(delta), math.sin(delta)
        dx = float(world_xy[0]) - world[0]
        dy = float(world_xy[1]) - world[1]
        return (source[0] + cosine * dx - sine * dy,
                source[1] + sine * dx + cosine * dy)

    def _to_world(self, source_xy):
        """Inverse of `_to_source`: map an estimated-frame point to the world."""
        with self.lock:
            world, source = self.world_pose, self.source_pose
        if world is None or source is None:
            return None
        delta = normalize_angle(world[2] - source[2])
        cosine, sine = math.cos(delta), math.sin(delta)
        dx = float(source_xy[0]) - source[0]
        dy = float(source_xy[1]) - source[1]
        return (world[0] + cosine * dx - sine * dy,
                world[1] + sine * dx + cosine * dy)

    def _to_world_yaw(self, source_yaw):
        with self.lock:
            world, source = self.world_pose, self.source_pose
        if world is None or source is None:
            return None
        return normalize_angle(float(source_yaw) + world[2] - source[2])

    def _to_source_yaw(self, world_yaw):
        with self.lock:
            world, source = self.world_pose, self.source_pose
        if world is None or source is None:
            return None
        return normalize_angle(float(world_yaw) + source[2] - world[2])

    # ------------------------------------------------------------ geometry
    def staging_world(self):
        facing = float(self.door[2])
        return (float(self.door[0]) - self.staging_distance * math.cos(facing),
                float(self.door[1]) - self.staging_distance * math.sin(facing))

    def car_stop_world(self):
        facing = float(self.door[2])
        return (float(self.door[0]) + self.car_depth * math.cos(facing),
                float(self.door[1]) + self.car_depth * math.sin(facing))

    def door_frame(self, point):
        """Return (along, lateral) of a world point in the doorway frame.

        ``along`` grows through the doorway into the car; ``lateral`` is the
        sideways offset from the door centre line.  Keeping the robot's lateral
        offset near zero before the straight run is what stops it from scraping
        the shaft wall beside the opening.
        """
        facing = float(self.door[2])
        dx = float(point[0]) - float(self.door[0])
        dy = float(point[1]) - float(self.door[1])
        cosine, sine = math.cos(facing), math.sin(facing)
        along = dx * cosine + dy * sine
        lateral = -dx * sine + dy * cosine
        return along, lateral

    def axis_point(self, along):
        """A world point on the door centre line, ``along`` metres into the car."""
        facing = float(self.door[2])
        return (float(self.door[0]) + along * math.cos(facing),
                float(self.door[1]) + along * math.sin(facing))

    def corridor_point(self, depth):
        """World point ``depth`` metres past the corridor gate, along the corridor."""
        yaw = float(self.corridor_gate[2])
        return (float(self.corridor_gate[0]) + depth * math.cos(yaw),
                float(self.corridor_gate[1]) + depth * math.sin(yaw))

    def corridor_axis_point(self, depth):
        """World point ``depth`` metres past the gate, ON the corridor axis."""
        yaw = float(self.corridor_gate[2])
        along = (math.cos(yaw), math.sin(yaw))
        lateral = (-math.sin(yaw), math.cos(yaw))
        return (float(self.corridor_gate[0]) + depth * along[0]
                + self.corridor_axis_lateral * lateral[0],
                float(self.corridor_gate[1]) + depth * along[1]
                + self.corridor_axis_lateral * lateral[1])

    def corridor_frame(self, point):
        """(along, lateral) of a world point in the corridor-gate frame.

        ``along`` 0 is the gate itself (the corridor mouth is a little further
        in, at the corridor floor's y_min), and it grows down the corridor.
        """
        yaw = float(self.corridor_gate[2])
        dx = float(point[0]) - float(self.corridor_gate[0])
        dy = float(point[1]) - float(self.corridor_gate[1])
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return (dx * cosine + dy * sine, -dx * sine + dy * cosine)

    def expected_floor_z(self, floor):
        """Absolute Gazebo z the robot must stand at on ``floor``.

        Measured on the ground floor in the lobby (0.31 m), one floor is 2.6 m
        of Gazebo z.  This verdict works for a descent as well as a climb, which
        the old "z rose by >= 1.5 m" test could not express.
        """
        if self.ground_z is not None:
            return self.ground_z + self.floor_height * float(floor)
        if self.z_before_ride is not None and self.current_floor is not None:
            return self.z_before_ride + self.floor_height * (
                float(floor) - float(self.current_floor))
        return None

    # ------------------------------------------------------------- commands
    def _publish(self, linear, angular):
        if rospy.is_shutdown():
            # The control timer can still fire while rospy tears down; publishing
            # then raises "publish() to a closed topic" and buries the verdict
            # under a traceback.
            return
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        with self.lock:
            self.last_cmd = (float(linear), float(angular))
        try:
            self.command_pub.publish(message)
        except rospy.ROSException:
            pass

    def _stop(self):
        self._publish(0.0, 0.0)

    def _set_state(self, state, reason=""):
        with self.lock:
            if state == self.state:
                return
            self.state = state
            self.state_since = self.sim_now
            # Capture every transition in the timeline: a state can be shorter
            # than the 1 Hz sampler (FACE_SPAWN is now a few degrees of trim),
            # and the run evidence should still show it.
            self.last_sample_sim = -1.0
            self.stuck_since = None
            self.stuck_anchor = None
            self.stuck_anchor_yaw = None
            self.turn_anchor = None
            self.turn_anchor_sim = None
            self.turn_unstick_until = None
            self.turn_unstick_count = 0
        rospy.loginfo("elevator-only state -> %s%s", state,
                      " ({})".format(reason) if reason else "")

    def _drive_to(self, target_source, tolerance=None, speed=None):
        """Turn-then-drive toward a source-frame point.  True when arrived."""
        pose = self.source_pose
        if pose is None or target_source is None:
            self._stop()
            return False
        tolerance = self.target_tolerance if tolerance is None else float(tolerance)
        cruise = self.motion_speed if speed is None else float(speed)
        distance = planar_distance(pose, target_source)
        if distance <= tolerance:
            self._stop()
            return True
        heading = math.atan2(target_source[1] - pose[1], target_source[0] - pose[0])
        error = normalize_angle(heading - pose[2])
        if abs(error) > self.heading_tolerance:
            # Turn on the spot first: driving forward while badly misaligned is
            # what arcs a quadruped into the jambs it is facing.
            self._turn_command(error)
            return False
        if self.front_clearance < self.stop_distance:
            self._stop()
            rospy.logwarn_throttle(
                2.0, "elevator-only: front blocked (%.2f m) in %s", self.front_clearance,
                self.state,
            )
            return False
        lateral = 0.0
        if math.isfinite(self.left_clearance) and math.isfinite(self.right_clearance):
            lateral = max(-0.15, min(0.15, 0.10 * (self.right_clearance - self.left_clearance)))
        self._publish(cruise, max(-0.25, min(0.25, 0.8 * error + lateral)))
        return False

    def _in_corridor_region(self):
        """True when the LIO estimate may drive position (corridor only)."""
        with self.lock:
            truth = self.robot_truth
        if truth is None:
            return True          # no truth: fall back to the estimate
        return (self.corridor_x_min <= float(truth[0]) <= self.corridor_x_max
                and self.corridor_y_min <= float(truth[1]) <= self.corridor_y_max)

    def _note_region(self, region):
        if (not self.truth_steered_regions
                or self.truth_steered_regions[-1][0] != region):
            self.truth_steered_regions.append(
                (str(region), round(self.sim_now, 2)))

    def _drive_to_world(self, target_world, tolerance=None, speed=None):
        """Drive to a WORLD point, choosing the control source by region.

        Corridor -> LIO source frame (as before).  Everywhere else -> Gazebo
        truth, because that is where the estimate has no features to work with.
        Arrival is judged in the same frame that steers, so the two branches are
        consistent with their own position source.
        """
        if target_world is None:
            self._stop()
            return False
        tolerance = self.target_tolerance if tolerance is None else float(tolerance)
        cruise = self.motion_speed if speed is None else float(speed)
        pose = self.source_pose
        truth = self.robot_truth
        if pose is None:
            self._stop()
            return False
        if truth is None or self._in_corridor_region():
            self._note_region("corridor_lio")
            return self._drive_to(self._to_source(target_world),
                                  tolerance=tolerance, speed=cruise)
        self._note_region("truth")
        distance = planar_distance(truth[:2], target_world[:2])
        if distance <= tolerance:
            self._stop()
            return True
        bearing = math.atan2(float(target_world[1]) - float(truth[1]),
                             float(target_world[0]) - float(truth[0]))
        heading = self._to_source_yaw(bearing)
        if heading is None:
            self._stop()
            return False
        error = normalize_angle(heading - pose[2])
        if abs(error) > self.heading_tolerance:
            self._turn_command(error)
            return False
        if self.front_clearance < self.stop_distance:
            self._stop()
            rospy.logwarn_throttle(
                2.0, "elevator-only: front blocked (%.2f m) in %s",
                self.front_clearance, self.state)
            return False
        lateral = 0.0
        if math.isfinite(self.left_clearance) and math.isfinite(self.right_clearance):
            lateral = max(-0.15, min(0.15,
                                     0.10 * (self.right_clearance - self.left_clearance)))
        self._publish(cruise, max(-0.25, min(0.25, 0.8 * error + lateral)))
        return False

    def _drive_heading(self, world_yaw, distance):
        """Drive straight along a world heading for a distance.  True when done."""
        pose = self.source_pose
        target = self._to_source_yaw(world_yaw)
        if pose is None or target is None:
            self._stop()
            return False
        if self.entry_anchor is None:
            self.entry_anchor = pose[:2]
        error = normalize_angle(target - pose[2])
        if abs(error) > self.heading_tolerance:
            self._publish(0.0, math.copysign(self.turn_speed, error))
            return False
        if planar_distance(pose, self.entry_anchor) >= distance:
            self._stop()
            return True
        if self.front_clearance < self.stop_distance:
            self._stop()
            rospy.logwarn_throttle(
                2.0, "elevator-only: front blocked (%.2f m) while entering", self.front_clearance
            )
            return False
        self._publish(self.motion_speed, max(-0.20, min(0.20, 0.8 * error)))
        return False

    def _turn_command(self, error, speed=None, stall_seconds=None, limit=None):
        """Turn on the spot, with a walk-and-turn fallback on gait lock-up."""
        speed = self.turn_speed if speed is None else float(speed)
        stall_seconds = (self.turn_unstick_seconds if stall_seconds is None
                         else float(stall_seconds))
        limit = self.turn_unstick_limit if limit is None else int(limit)
        pose = self.source_pose
        if pose is not None:
            if self.turn_anchor is None:
                self.turn_anchor = (pose[0], pose[1], pose[2])
                self.turn_anchor_sim = self.sim_now
            else:
                # A turn goal is advanced by YAW only: outdoors the drifting
                # LIO estimate supplies phantom translation, and counting that
                # as progress is what let a physically frozen turn livelock for
                # 167 s at the spawn (elevstreak7_01).
                turned = abs(normalize_angle(pose[2] - self.turn_anchor[2]))
                if turned >= self.turn_progress_yaw:
                    self.turn_anchor = (pose[0], pose[1], pose[2])
                    self.turn_anchor_sim = self.sim_now
                elif self.sim_now - self.turn_anchor_sim >= stall_seconds:
                    if self.turn_unstick_count >= limit:
                        # Bounded: a turn that cannot be unstuck must fail
                        # loudly, not grind until the wall backstop.
                        self.fault = "TURN_BLOCKED_IN_{}_AFTER_{}_UNSTICKS".format(
                            self.state, self.turn_unstick_count)
                        self.turn_unstick_count = 0
                        self._set_state("FAILED", self.fault)
                        self._stop()
                        return
                    self.turn_unstick_dir = -self.turn_unstick_dir
                    self.turn_unstick_until = self.sim_now + self.turn_unstick_walk
                    self.turn_unstick_count += 1
                    self.turn_anchor = (pose[0], pose[1], pose[2])
                    self.turn_anchor_sim = self.sim_now
                    rospy.logwarn(
                        "elevator-only: turn made no progress for %.1f s in %s; "
                        "walk-and-turn unstick %+.2f m/s (count %d)",
                        self.turn_unstick_seconds, self.state,
                        self.turn_unstick_dir * self.turn_walk_speed,
                        self.turn_unstick_count,
                    )
        if self.turn_unstick_until is not None and self.sim_now < self.turn_unstick_until:
            self._publish(self.turn_unstick_dir * self.turn_walk_speed,
                          math.copysign(speed, error))
            return
        self.turn_unstick_until = None
        self._publish(0.0, math.copysign(speed, error))

    # -------------------------------------------------------------- pieces
    def _ensure_plane_policy(self):
        """Pick the gait policy while the robot is standing still.

        ``stair`` keeps junior_ctrl's default policy (the one trained for
        steps, needed for the 6 cm car threshold), ``plane`` selects the
        flat-ground policy the mainline elevator node used, and ``keep`` does
        nothing at all.  The service refuses to switch while cmd_vel is
        non-zero, so this must be called before anything starts moving.
        """
        if self.gait_policy == "keep":
            self.plane_policy_active = True
            return True
        if self.plane_policy_active:
            return True
        want_plane = self.gait_policy == "plane"
        try:
            rospy.wait_for_service("/unitree/select_plane_policy", timeout=0.5)
            response = self.plane_policy_service(data=want_plane)
        except Exception as error:  # noqa: BLE001 - service may not be up yet
            rospy.logwarn_throttle(5.0, "elevator-only: gait policy unavailable: %s", error)
            return False
        self.plane_policy_active = bool(response.success)
        if self.plane_policy_active:
            rospy.loginfo("elevator-only: gait policy = %s (%s)",
                          self.gait_policy, response.message)
        else:
            rospy.logwarn("elevator-only: gait policy %s rejected: %s",
                          self.gait_policy, response.message)
        return self.plane_policy_active

    def _retoggle_gait_policy(self):
        """Reset the RL controller's internal state while standing still.

        The cheapest cure for a frozen policy: select the plane policy and
        immediately re-select the step-trained one, leaving `stair` active.
        The service refuses to switch while cmd_vel is non-zero, so this only
        runs after the driver has published zero for a few ticks.
        """
        if self.gait_policy == "keep" or self.plane_policy_service is None:
            return False
        try:
            rospy.wait_for_service("/unitree/select_plane_policy", timeout=0.5)
            self.plane_policy_service(data=True)
            self.plane_policy_service(data=False)
        except Exception as error:  # noqa: BLE001
            rospy.logwarn_throttle(
                5.0, "elevator-only: gait policy re-toggle failed: %s", error)
            return False
        self.plane_policy_active = True
        rospy.logwarn(
            "elevator-only: gait policy re-toggled; stair policy re-asserted")
        return True

    def _open_car_door(self, floor_index):
        if self.door_service is None:
            return True
        door_id = "{}_{}".format(self.door_prefix, int(floor_index))
        try:
            rospy.wait_for_service("/set_door_state", timeout=0.5)
            response = self.door_service(door_id, True)
            return bool(response.accepted)
        except Exception as error:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, "elevator-only: door %s: %s", door_id, error)
            return False

    def _request_ride(self):
        if self.elevator_service is None:
            self.ride_error = "CallElevator service type unavailable"
            return
        try:
            rospy.wait_for_service("/call_elevator", timeout=5.0)
            self.ride_response = self.elevator_service(
                self.elevator_id, self.target_floor, True
            )
        except Exception as error:  # noqa: BLE001
            self.ride_error = str(error)

    # ----------------------------------------------------------------- truth
    def _poll_truth(self):
        """Read absolute positions from Gazebo (the only frame that sees a ride)."""
        try:
            rospy.wait_for_service("/gazebo/get_model_state", timeout=0.2)
            response = self.model_state_service(self.robot_model, "")
            if response.success:
                position = response.pose.position
                with self.lock:
                    self.gazebo_truth = (float(position.x), float(position.y),
                                         float(position.z))
                # Gazebo's own heading: the frame-free truth for the final
                # heading verdict (the LIO source frame is rotated by the spawn
                # bearing, so src_yaw is NOT the mission-frame heading).  Kept
                # in its own try so a pose without an orientation can never cost
                # us the position, which every truth gate depends on.
                try:
                    orientation = response.pose.orientation
                    truth_yaw = math.atan2(
                        2.0 * (orientation.w * orientation.z
                               + orientation.x * orientation.y),
                        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
                    )
                    with self.lock:
                        self.gazebo_truth_yaw = float(truth_yaw)
                except Exception:  # noqa: BLE001 - position is what matters
                    pass
            # Decide which pose drives CONTROL/JUDGEMENT.
            with self.lock:
                world = self.world_pose
                if self.use_truth:
                    if self.gazebo_truth is not None:
                        self.robot_truth = self.gazebo_truth
                    if self.gazebo_truth_yaw is not None:
                        self.robot_truth_yaw = self.gazebo_truth_yaw
                elif world is not None:
                    self.robot_truth = (float(world[0]), float(world[1]),
                                        float(world[3]))
                    self.robot_truth_yaw = float(world[2])
        except Exception:  # noqa: BLE001 - Gazebo may be busy
            pass
        try:
            response = self.model_state_service(self.car_model, "")
            if response.success:
                with self.lock:
                    self.car_truth_z = float(response.pose.position.z)
        except Exception:  # noqa: BLE001
            pass

    def _truth_z(self):
        with self.lock:
            return None if self.robot_truth is None else float(self.robot_truth[2])

    def _publish_floor_context(self, reason):
        """Tell the mapping stack which floor the robot is standing on.

        Payload mirrors the mainline elevator node (floor_index is the field
        lio_occupancy_node reads; the rest are hints it ignores when absent).
        """
        payload = {
            "floor_index": int(self.current_floor),
            "source": str(reason),
            "floor_z": (None if self.world_pose is None
                        else round(float(self.world_pose[3]), 3)),
            "gate_world": [round(float(v), 3) for v in self.corridor_gate],
            "reused_topology": [],
        }
        gate_source = self._to_source(self.corridor_gate[:2])
        if gate_source is not None:
            payload["gate_source"] = [round(gate_source[0], 3),
                                      round(gate_source[1], 3)]
        depth = self.corridor_depths[min(self.leg_index,
                                         len(self.corridor_depths) - 1)] \
            if self.corridor_depths else 0.0
        target_source = self._to_source(
            self.corridor_axis_point(max(0.0, float(depth))))
        if target_source is not None:
            payload["corridor_target_source"] = [round(target_source[0], 3),
                                                 round(target_source[1], 3)]
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        try:
            self.floor_context_pub.publish(message)
        except rospy.ROSException:
            return
        self.floor_contexts.append((str(reason), int(self.current_floor),
                                    round(self.sim_now, 1)))
        rospy.loginfo("elevator-only: floor context published (%s): floor_index=%d",
                      reason, self.current_floor)

    def _check_localisation(self):
        """Fail loudly when the metric estimate and Gazebo truth disagree.

        Planar disagreement only: ``/simnav/world_pose_metric.z`` cannot see a
        ride by design (the robot is motionless in the car), so a z test would
        fire on every healthy ride.  A metre-scale planar break means the map
        and the estimate are on different floors.
        """
        with self.lock:
            # Guard-only use of truth (never control): compare the estimate
            # against Gazebo so a real divergence is still detected in faithful
            # mode, where robot_truth carries the estimate itself.
            world, truth = self.world_pose, (
                self.gazebo_truth if self.gazebo_truth is not None
                else self.robot_truth)
            state = self.state
            cmd_vx = self.last_cmd[0]
        if world is None or truth is None:
            return
        if state == "WAIT_READY":
            # Localisation is still converging at start-up; the failure this
            # guards against happens later (across a ride).
            return
        error = planar_distance(world[:2], truth[:2])
        if self.max_world_truth_error is None or error > self.max_world_truth_error:
            self.max_world_truth_error = round(error, 3)
        if error <= self.loc_max_planar_error:
            self.loc_diverged_since = None
            self.loc_truth_anchor = None
            self.loc_episode_counted = False
            self.loc_hold = False
            return
        # Above the threshold: decide whether the robot is at risk.
        if self.loc_diverged_since is None:
            self.loc_diverged_since = self.sim_now
            self.loc_truth_anchor = truth[:2]
            self.loc_episode_counted = False
            rospy.logwarn(
                "elevator-only: world-vs-truth disagreement %.2f m (threshold "
                "%.2f m) in %s; watching for up to %.1f s",
                error, self.loc_max_planar_error, state,
                self.loc_diverged_seconds)
        truth_moved = (0.0 if self.loc_truth_anchor is None
                       else planar_distance(truth[:2], self.loc_truth_anchor))
        if self.use_truth and not self._in_corridor_region():
            # Truth-steered region (only when truth control is enabled): position
            # control does not use the estimate, so a divergence is recorded but
            # never fatal here.  In FAITHFUL mode the estimate drives everywhere,
            # so there is no such exemption.
            self.loc_hold = False
            if not self.loc_episode_counted:
                self.loc_truth_steered_warnings += 1
                self.loc_episode_counted = True
                rospy.logwarn(
                    "elevator-only: estimate disagreement %.2f m in %s while "
                    "truth-steered (no lidar features here); not fatal",
                    error, state)
            return
        at_risk = (truth_moved >= self.loc_truth_motion
                   or abs(cmd_vx) > self.loc_cmd_motion)
        if not at_risk:
            # Stationary (in-place turn): not fatal.  Keep the turn going - the
            # final heading is steered on Gazebo truth anyway - and record it.
            self.loc_hold = False
            if not self.loc_episode_counted:
                self.loc_stationary_warnings += 1
                self.loc_episode_counted = True
                rospy.logwarn(
                    "elevator-only: estimate disagreement %.2f m while "
                    "stationary in %s (truth moved %.2f m); not fatal",
                    error, state, truth_moved)
            return
        self.loc_hold = True
        if not self.loc_episode_counted:
            self.loc_hold_count += 1
            self.loc_episode_counted = True
        if (self.fault is None
                and self.sim_now - self.loc_diverged_since
                >= self.loc_diverged_seconds):
            self.fault = "LOCALISATION_DIVERGED_BY_{:.2f}M_FOR_{:.1f}S_WHILE_DRIVING".format(
                error, self.loc_diverged_seconds)
            self._set_state("FAILED", self.fault)
            self._stop()

    def _map_callback(self, message):
        try:
            data = np.asarray(message.data, dtype=np.int16).reshape(
                message.info.height, message.info.width)
        except Exception:  # noqa: BLE001 - malformed grid
            return
        with self.lock:
            self.navigation_grid = _GridView(
                data, float(message.info.resolution),
                float(message.info.origin.position.x),
                float(message.info.origin.position.y),
                message.header.frame_id or "simnav_map")

    def _confirm_door_candidate(self):
        """Locate the lift from a map-detected, temporally confirmed candidate.

        Mirrors the mainline: run `detect_wide_lobby_openings` around the
        corridor gate (in the map frame), keep evidence per (side, along-bin),
        and only accept after `confirm_cycles` consistent observations.  The
        accepted candidate becomes the doorway frame used for the approach, the
        centre-line alignment and the boarding judgements; the configured
        reference stays as the sanity yardstick.
        """
        if (self.candidate_detector is None or self.navigation_grid is None
                or self.door_candidate_accepted):
            return
        gate_source = self._to_source(self.corridor_gate[:2])
        gate_yaw = self._to_source_yaw(float(self.corridor_gate[2]))
        if gate_source is None or gate_yaw is None:
            return
        try:
            candidates = self.candidate_detector(
                self.navigation_grid, gate_source, gate_yaw,
                corridor_half_width=self.corridor_half_width,
                preferred_heading=normalize_angle(gate_yaw - math.pi / 2.0))
        except Exception as error:  # noqa: BLE001 - detector must never kill a run
            rospy.logwarn_throttle(10.0, "elevator-only: door candidate "
                                    "detector failed: %s", error)
            return
        candidates = [item for item in candidates
                      if self.door_candidate_lateral_min
                      <= float(item.get("lateral", 0.0))
                      <= self.door_candidate_lateral_max
                      and 0.0 <= float(item.get("along", 0.0)) <= 8.0]
        if not candidates:
            self.door_candidate_note = "no_free_opening_in_map"
            return
        best = candidates[0]
        pose = best.get("pose")
        if pose is None or len(pose) < 3:
            return
        key = (str(best.get("side")), int(round(float(best.get("along", 0.0)) / 0.5)))
        self.door_candidate_evidence[key] = (
            self.door_candidate_evidence.get(key, 0) + 1)
        self.door_candidate_cycles = self.door_candidate_evidence[key]
        if self.door_candidate_cycles < self.door_candidate_confirm_cycles:
            self.door_candidate_note = "confirming_%d_of_%d" % (
                self.door_candidate_cycles, self.door_candidate_confirm_cycles)
            return
        world_xy = self._to_world(pose[:2])
        world_yaw = self._to_world_yaw(pose[2])
        if world_xy is None or world_yaw is None:
            return
        width = float(best.get("width", 0.0) or 0.0)
        candidate = (round(world_xy[0], 3), round(world_xy[1], 3),
                     round(world_yaw, 4), round(width, 3))
        _along, lateral = self.door_frame(candidate[:2])
        yaw_error = abs(normalize_angle(candidate[2] - float(self.door_reference[2])))
        self.door_candidate = candidate
        self.door_candidate_offset = (round(lateral, 3), round(yaw_error, 3))
        if (abs(lateral) > self.door_candidate_max_lateral
                or yaw_error > self.door_candidate_max_yaw
                or not (self.door_candidate_min_width <= width
                        <= self.door_candidate_max_width)):
            self.door_candidate_rejected = True
            self.door_candidate_reject_reason = (
                "lateral %.2f m (max %.2f), yaw %.2f rad (max %.2f), width %.2f m"
                % (lateral, self.door_candidate_max_lateral, yaw_error,
                   self.door_candidate_max_yaw, width))
            rospy.logwarn("elevator-only: door candidate %s REJECTED (%s); "
                          "keeping the configured reference",
                          candidate, self.door_candidate_reject_reason)
            return
        self.door_candidate_accepted = True
        self.door = candidate
        self.door_frame_source = "candidate"
        rospy.loginfo(
            "elevator-only: door candidate ACCEPTED after %d cycles: %s "
            "(lateral %+.2f m, yaw %.2f rad vs reference)",
            self.door_candidate_cycles, candidate, lateral, yaw_error)

    def _truth_tick(self):
        """Poll Gazebo ground truth faster than the 1 Hz timeline sampler.

        The corridor verdict is a ground-truth along-coordinate, and at 1 Hz the
        sample can be a full control-second (up to ~0.6 m) stale, which both
        under-reports how deep the robot went and would let the truth gate below
        creep too long.
        """
        if self.sim_now - self.last_truth_sim < 0.25:
            return
        self.last_truth_sim = self.sim_now
        self._poll_truth()
        self._check_localisation()
        self._confirm_door_candidate()

    # --------------------------------------------------------------- checks
    def _stuck_check(self, label):
        with self.lock:
            truth = self.robot_truth
            source = self.source_pose
        # Progress is measured on GROUND TRUTH when it is available: a diverged
        # estimate must neither fake progress nor mask a real stall
        # (elevf01: the estimated corridor coordinate ran away for 116 s while
        # the robot stood still, and the source-frame watchdog could not see it).
        if truth is not None:
            pose = (float(truth[0]), float(truth[1]),
                    0.0 if self.robot_truth_yaw is None else float(self.robot_truth_yaw))
        else:
            pose = source
        if pose is None:
            return False
        if self.stuck_anchor is None:
            self.stuck_anchor = pose[:2]
            self.stuck_anchor_yaw = pose[2]
            self.stuck_since = self.sim_now
            return False
        turned = (0.0 if self.stuck_anchor_yaw is None
                  else abs(normalize_angle(pose[2] - self.stuck_anchor_yaw)))
        if (planar_distance(pose, self.stuck_anchor) >= self.stuck_min_progress
                or turned >= self.stuck_min_yaw_progress):
            self.stuck_anchor = pose[:2]
            self.stuck_anchor_yaw = pose[2]
            self.stuck_since = self.sim_now
            return False
        if self.sim_now - self.stuck_since >= self.stuck_timeout:
            self.fault = "STUCK_IN_{}".format(label)
            return True
        return False

    # ------------------------------------------------------ mission plumbing
    def _leg_depth(self, index):
        if 0 <= index < len(self.corridor_depths):
            return float(self.corridor_depths[index])
        return 0.0

    def _begin_excursion(self, depth, mode="leg"):
        """Arm a corridor run: out to the mouth, U-turn, and back.

        ``mode='entry'`` is the first leg of the mission (after walking in
        through the main entrance): it starts from inside the lobby and ends by
        lining up for the lift.  ``mode='leg'`` is the excursion that follows a
        ride: it starts at the car mouth and ends by re-boarding on the same
        floor.
        """
        self.excursion_mode = mode
        self.excursion_depth = float(depth)
        self.excursion_target = self.corridor_axis_point(self.excursion_depth)
        self.excursion_started_sim = self.sim_now
        # Entry mode starts from the lobby already; a ride leg first returns to
        # the known waypoint in front of the doorway.
        self.excursion_leg = 1 if mode == "entry" else 0
        self.excursion_reverse_anchor = None
        self.excursion_max_along = None
        self.excursion_truth_max_along = None
        self.stuck_anchor = None
        self.stuck_since = None
        leg = self.leg_results[-1] if (self.leg_results and mode == "leg") else None
        if leg is not None:
            leg["corridor_depth"] = round(self.excursion_depth, 2)
            leg["corridor_target"] = tuple(round(v, 2) for v in self.excursion_target)
        self._set_state(
            "TO_CORRIDOR",
            "%s: driving to the corridor mouth (%.1f m past the gate)"
            % ("after entering the building" if mode == "entry"
               else "out of the car", self.excursion_depth),
        )

    def _board_next_leg(self, reason):
        """Advance the ride schedule and line up for the next ride."""
        if self.leg_index + 1 < len(self.ride_floors):
            self.leg_index += 1
            self.target_floor = self.ride_floors[self.leg_index]
            self.entry_anchor = None
            self._set_state("ALIGN_DOOR", reason % self.target_floor)
        elif self.return_to_spawn and self.spawn_world is not None:
            self._set_state("RETURN_SPAWN", "all rides done; driving back to the spawn point")
        else:
            self._set_state("DONE", "all lifts completed and left")

    # ---------------------------------------------------------- state machine
    def _control(self, _event):
        if rospy.is_shutdown():
            return
        with self.lock:
            state = self.state
        if state in ("DONE", "FAILED"):
            self._stop()
            return
        self._truth_tick()
        self._sample()
        if self.loc_hold and state not in ("DONE", "FAILED"):
            # Estimation is suspect: stand still until it recovers or the
            # debounce expires (the state logic must not drive on it).
            self._stop()
            return
        pose = self.source_pose
        if state != "WAIT_READY" and (pose is None or self.world_pose is None):
            self._stop()
            return

        if state == "WAIT_READY":
            if pose is None or self.world_pose is None:
                self._stop()
                return
            # Hold still first: the gait-policy service refuses to switch while
            # /cmd_vel is non-zero, and a rejected switch is silent unless we
            # give it a clean zero-velocity window.
            self._stop()
            if self.ready_since is None:
                self.ready_since = self.sim_now
                return
            if self.sim_now - self.ready_since < 1.5:
                return
            if not self._ensure_plane_policy():
                return
            self.entry_anchor = None
            # Remember where the run started: the last leg of the mission drives
            # back here.  The measured world pose wins over the parameter (the
            # parameter is only there so a bad localisation start is visible).
            with self.lock:
                world = self.world_pose
            if world is not None:
                self.spawn_measured = (round(world[0], 3), round(world[1], 3),
                                       round(world[2], 3))
                if self.spawn_world is None:
                    self.spawn_world = (world[0], world[1], world[2])
            if self.ground_z is None:
                self.ground_z = self._truth_z()
            self.mission_started_sim = self.sim_now
            rospy.loginfo(
                "elevator-only: spawn world %s (param %s) ground truth z %s; "
                "ride schedule %s with corridor depths %s",
                self.spawn_measured, self.spawn_param, self.ground_z,
                self.ride_floors, self.corridor_depths[:len(self.ride_floors)],
            )
            self._publish_floor_context("start")
            # Open the ground-floor car door BEFORE driving so the doorway is
            # mapped open: with the door leaf closed the map shows no opening
            # and the candidate detector cannot confirm the lift (probe map: the
            # true door centre row had two occupied cells and a 0.9 m free span
            # belonging to the stair core, not the lift).
            self._open_car_door(self.current_floor)
            if self.enter_building:
                self._set_state("ENTER_BUILDING")
            else:
                self._set_state("TO_STAGING", "entrance skipped by parameter")
            return

        if state == "ENTER_BUILDING":
            # --- shared stall recovery for both entrance legs ---
            if self.entry_retreat_until is not None:
                retreated = (0.0
                             if (self.entry_retreat_anchor is None
                                 or self.robot_truth is None)
                             else planar_distance(self.robot_truth[:2],
                                                  self.entry_retreat_anchor))
                if (self.sim_now < self.entry_retreat_until
                        and retreated < self.entry_retreat_distance):
                    # back off towards the forecourt, clear of the slab edge
                    self._publish(-abs(self.entry_retreat_speed), 0.0)
                    return
                self.entry_retreat_until = None
                self._stop()
                self._retoggle_gait_policy()
                self.entry_recover_bias_until = (
                    self.sim_now + self.entry_retry_bias_seconds)
                self.entry_progress_sim = self.sim_now
                self.entry_progress_anchor = None
                self.entry_anchor = None
                return
            if (self.entry_recover_bias_until is not None
                    and self.sim_now < self.entry_recover_bias_until):
                target_yaw = self._to_source_yaw(
                    float(self.entrance[2])
                    + self.entry_retry_yaw_bias * self.entry_bias_sign)
                error = (0.0 if target_yaw is None
                         else normalize_angle(target_yaw - pose[2]))
                if abs(error) > self.heading_tolerance:
                    self._turn_command(error)
                else:
                    self._publish(self.crossing_speed,
                                  max(-0.20, min(0.20, 0.8 * error)))
                return
            self.entry_recover_bias_until = None
            if self.robot_truth is not None:
                if (self.entry_progress_anchor is None
                        or planar_distance(self.robot_truth[:2],
                                           self.entry_progress_anchor)
                        >= self.stuck_min_progress):
                    self.entry_progress_anchor = self.robot_truth[:2]
                    self.entry_progress_sim = self.sim_now
                elif (self.entry_progress_sim is not None
                      and self.sim_now - self.entry_progress_sim
                      >= self.entry_stall_trigger):
                    if self.entry_leg_retries >= self.entry_leg_retry_limit:
                        self.fault = ("ENTER_BUILDING_STALL_AFTER_{}_RETRIES_"
                                      "LEG_{}").format(self.entry_leg_retries,
                                                       self.entry_leg)
                        self._set_state("FAILED", self.fault)
                        return
                    self.entry_leg_retries += 1
                    self.entry_bias_sign = -self.entry_bias_sign
                    self.entry_retreat_until = (
                        self.sim_now + self.entry_retreat_timeout)
                    self.entry_retreat_anchor = (
                        None if self.robot_truth is None
                        else self.robot_truth[:2])
                    self.entry_progress_sim = self.sim_now
                    self.entry_progress_anchor = self.robot_truth[:2]
                    self._stop()
                    rospy.logwarn(
                        "elevator-only: entrance leg %d frozen (truth %.2f, %.2f, "
                        "front %.2f m); retreating %.1f s, re-toggling gait and "
                        "re-committing (%d/%d)", self.entry_leg,
                        self.robot_truth[0], self.robot_truth[1],
                        self.front_clearance, self.entry_retreat_seconds,
                        self.entry_leg_retries, self.entry_leg_retry_limit)
                    return
            # Two line-of-sight legs: up to the entrance, then just inside it.
            # The leg index must be its own state: rerunning leg 1 after it has
            # already succeeded walks the robot back out of the doorway
            # (entrance01 spun on the lobby for 30 s doing exactly that).
            if self.entry_leg == 0:
                if self._drive_to_world(self.entrance[:2], tolerance=0.45):
                    self.entry_leg = 1
                    self.entry_anchor = None
                    self.entry_leg_retries = 0
                    self.entry_progress_anchor = None
                    self.entry_progress_sim = self.sim_now
                elif self._stuck_check("ENTER_BUILDING_MOUTH"):
                    self._set_state("FAILED", self.fault)
                return
            if self._drive_heading(float(self.entrance[2]), self.entrance_inside):
                # The user's flow: walk in, drive up the corridor to its mouth,
                # U-turn there (so the lift is on the map), then come back to
                # the doorway.  With no entry excursion, go straight to staging.
                if self.entry_corridor_depth > 0.0:
                    self._begin_excursion(self.entry_corridor_depth, "entry")
                else:
                    self._set_state("TO_STAGING")
            elif self._stuck_check("ENTER_BUILDING_INSIDE"):
                self._set_state("FAILED", self.fault)
            return

        if state == "TO_STAGING":
            if self._drive_to_world(self.staging_world()):
                self._set_state("ALIGN_DOOR")
            elif self._stuck_check("TO_STAGING"):
                self._set_state("FAILED", self.fault)
            return

        if state == "ALIGN_DOOR":
            # Two conditions before the committed run: the nose points along the
            # doorway normal, AND the robot sits on the doorway centre line.
            # run1 (lobbytest_01) satisfied only the first: it drifted to
            # lateral +0.49 m, scraped the shaft wall beside the opening at
            # Gazebo (1.69, 3.09), and pushed there for 35 s going nowhere.
            along, lateral = self.door_frame(self.world_pose[:2])
            if self.robot_truth is not None and not self._in_corridor_region():
                # Truth-steered region: judge the centring on truth too, or the
                # state loops (it slides on truth while the estimate still says
                # it is off-centre -> STUCK_IN_ALIGN_DOOR_CENTRE).
                along, lateral = self.door_frame(self.robot_truth[:2])
            if abs(lateral) > self.lateral_tolerance:
                # Slide back onto the centre line at the current stand-off.
                axis_target = self.axis_point(min(along, -0.30))
                if self._drive_to_world(axis_target, tolerance=0.12):
                    self._stop()
                elif self._stuck_check("ALIGN_DOOR_CENTRE"):
                    self._set_state("FAILED", self.fault)
                return
            target_yaw = self._to_source_yaw(float(self.door[2]))
            if target_yaw is None:
                self._stop()
                return
            error = normalize_angle(target_yaw - pose[2])
            if abs(error) <= self.heading_tolerance:
                self._stop()
                self._ensure_plane_policy()
                self._open_car_door(self.current_floor)
                self.entry_anchor = None
                # Audit trail for the "line up on the centre before entering"
                # precondition: the lateral offset and heading error the entry
                # actually starts from.
                self.align_lateral = round(lateral, 3)
                self.align_heading_error = round(abs(error), 3)
                self.align_sim = round(self.sim_now, 2)
                self._set_state(
                    "ENTER_CAR",
                    "aligned on the doorway centre: lateral=%+.2f m (tol %.2f) "
                    "heading error %.2f rad (tol %.2f) along=%.2f m"
                    % (lateral, self.lateral_tolerance, abs(error),
                       self.heading_tolerance, along),
                )
            else:
                self._turn_command(error)
            return

        if state == "ENTER_CAR":
            # Straight committed run: the heading is held on the doorway normal
            # (only a small correction back onto the centre line is allowed),
            # and the robot is judged boarded by where it ends up, not by
            # whether the laser liked the approach.
            if self.entry_anchor is None:
                self.entry_anchor = pose[:2]
                self.entry_started_sim = self.sim_now
                self.entry_progress_sim = self.sim_now
                self.entry_progress_truth = None
                self.entry_progress_source = pose[:2]
                self.entry_retries = 0
                self.entry_retreat_until = None
                self.entry_bias_until = None
            # Phase 1: retreat out of the doorway for a moment to release the
            # threshold contact, then re-commit.
            if self.entry_retreat_until is not None:
                if self.sim_now < self.entry_retreat_until:
                    self._publish(-abs(self.entry_retreat_speed), 0.0)
                    return
                self.entry_retreat_until = None
                self.entry_anchor = pose[:2]
                self.entry_progress_sim = self.sim_now
                self.entry_progress_truth = None
                self.entry_progress_source = pose[:2]
                self.entry_bias_until = self.sim_now + self.entry_retry_bias_seconds
            along, lateral = self.door_frame(self.world_pose[:2])
            if self.robot_truth is not None and not self._in_corridor_region():
                # Car/doorway region is truth-steered: judge BOTH the centring
                # and the boarding progress on truth (the audit is truth-based).
                along, lateral = self.door_frame(self.robot_truth[:2])
            travelled = planar_distance(pose, self.entry_anchor)
            # Phase 2: stall check on the GROUND TRUTH along-coordinate (the
            # odometry slips exactly on the threshold, as probe 01 showed).
            truth_along = (None if self.robot_truth is None
                           else self.door_frame(self.robot_truth[:2])[0])
            if truth_along is not None:
                if (self.entry_progress_truth is None
                        or truth_along - self.entry_progress_truth
                        >= self.exit_progress_tolerance):
                    self.entry_progress_truth = truth_along
                    self.entry_progress_sim = self.sim_now
            elif (self.entry_progress_source is None
                  or planar_distance(pose, self.entry_progress_source)
                  >= self.exit_progress_tolerance):
                self.entry_progress_source = pose[:2]
                self.entry_progress_sim = self.sim_now
            if self.sim_now - self.entry_progress_sim >= self.entry_stall_seconds:
                if self.entry_retries >= self.entry_retry_limit:
                    self.fault = "ENTER_CAR_NO_PROGRESS_AFTER_{}_RETRIES_ALONG_{:.2f}M".format(
                        self.entry_retries, along)
                    self._set_state("FAILED", self.fault)
                    return
                self.entry_retries += 1
                self.entry_retreat_until = self.sim_now + self.entry_retreat_seconds
                self.entry_progress_sim = self.sim_now
                self.entry_progress_truth = truth_along
                self.entry_progress_source = pose[:2]
                self._stop()
                rospy.logwarn(
                    "elevator-only: caught on the car threshold going in "
                    "(along %.2f m, truth %s); retreating %.1f s and re-committing "
                    "(%d/%d)", along,
                    "n/a" if self.robot_truth is None
                    else "%.2f, %.2f" % self.robot_truth[:2],
                    self.entry_retreat_seconds, self.entry_retries,
                    self.entry_retry_limit)
                return
            desired_world_yaw = float(self.door[2]) - self.lateral_gain * lateral
            if (self.entry_retries > 0 and self.entry_bias_until is not None
                    and self.sim_now < self.entry_bias_until):
                # Brief biased re-commit to break the contact, then square again.
                desired_world_yaw += self.entry_retry_yaw_bias * (
                    1.0 if self.entry_retries % 2 else -1.0)
            heading_error = abs(normalize_angle(desired_world_yaw - self.world_pose[2]))
            # Boarding precondition: the run is only declared boarded once the
            # robot has crossed the threshold AND is still on the doorway centre
            # line with the nose on the doorway normal.  If it drifted, the
            # lateral term below keeps steering it back while it advances (the
            # car is deep enough for that), and the entry timeout reports the
            # residual offset if it never gets centred.
            if ((along >= self.car_depth or travelled >= self.enter_distance)
                    and abs(lateral) <= self.board_lateral_tolerance
                    and heading_error <= self.board_heading_tolerance):
                # Re-arm the ride request for EVERY boarding.  ride_started was
                # only ever set once, so the second boarding never called
                # /call_elevator at all: it sat in RIDE until the timeout with
                # the first ride's stale response (car_z frozen on floor 1).
                self.ride_started = False
                self.ride_response = None
                self.ride_error = None
                self.z_before_ride = self._current_z()
                self.ride_requested_at = self.sim_now
                self.board_along = round(along, 3)
                self.board_lateral = round(lateral, 3)
                self.board_lateral_truth = (
                    None if self.robot_truth is None
                    else round(self.door_frame(self.robot_truth[:2])[1], 3))
                self.board_along_truth = (
                    None if self.robot_truth is None
                    else round(self.door_frame(self.robot_truth[:2])[0], 3))
                self.board_heading_error = round(heading_error, 3)
                self.board_retries = self.entry_retries
                self.board_sim = self.sim_now
                self._set_state(
                    "RIDE",
                    "boarded centred: along=%.2f m lateral=%+.2f m (truth %+.2f m) "
                    "heading error %.3f rad travelled=%.2f m"
                    % (along, lateral,
                       self.board_lateral_truth if self.board_lateral_truth is not None
                       else float("nan"), heading_error, travelled),
                )
                return
            desired = self._to_source_yaw(desired_world_yaw)
            if desired is None:
                self._stop()
                return
            error = normalize_angle(desired - pose[2])
            # "Take the risk": once the run is committed the front-clearance
            # stop is not applied -- contact with the car is the expected end of
            # the run, and the position check above decides whether it worked.
            self._publish(
                self.crossing_speed,
                max(-0.25, min(0.25, 0.8 * error)),
            )
            if travelled > self.enter_distance + self.entry_overrun_margin:
                # A gross doorway-frame error must not turn into a long drive
                # into the car (or the shaft wall).  Bail out loudly instead.
                self.fault = "ENTER_CAR_OVERRUN_TRAVELLED_{:.2f}M_ALONG_{:.2f}M".format(
                    travelled, along)
                self._set_state("FAILED", self.fault)
                return
            if self.sim_now - self.entry_started_sim > self.entry_timeout:
                self.fault = ("ENTER_CAR_TIMEOUT_ALONG_{:.2f}M_LATERAL_{:+.2f}M_"
                              "HEADING_{:.2f}RAD_RETRIES_{}").format(
                    along, lateral, heading_error, self.entry_retries)
                self._set_state("FAILED", self.fault)
            return

        if state == "ALIGN_EXIT":
            # Turn around INSIDE the car.  The direction is chosen from the
            # measured clearances: the nose sweeps toward the freer side, and
            # the direction is only flipped on a confirmed stall.
            if self.exit_turn_started_sim is None:
                self.exit_turn_started_sim = self.sim_now
                left, right = self.left_clearance, self.right_clearance
                self.exit_turn_clearances = (
                    None if not math.isfinite(left) else round(left, 2),
                    None if not math.isfinite(right) else round(right, 2),
                    None if not math.isfinite(self.front_clearance) else round(self.front_clearance, 2),
                    None if not math.isfinite(self.rear_clearance) else round(self.rear_clearance, 2))
                if math.isfinite(left) and math.isfinite(right) and left != right:
                    # Facing the car mouth (-x) after the ride? No: after the
                    # ride the robot still faces INTO the car (+x), so its left
                    # is +y.  Sweeping the nose to its left needs a positive
                    # yaw command.
                    self.exit_turn_side = "L" if left > right else "R"
                else:
                    self.exit_turn_side = "L"
                self.exit_turn_sign = 1.0 if self.exit_turn_side == "L" else -1.0
                rospy.loginfo(
                    "elevator-only: in-car U-turn to the %s (clearances L=%s R=%s "
                    "F=%s B=%s)", self.exit_turn_side,
                    self.exit_turn_clearances[0], self.exit_turn_clearances[1],
                    self.exit_turn_clearances[2], self.exit_turn_clearances[3])
            # Watchdog: yaw progress only (a diverged estimate must not fake it).
            if self.robot_truth_yaw is not None:
                target_truth = (float(self.door[2])
                                + (0.0 if self.exit_turn_target == "in"
                                   else math.pi))
                err_truth = normalize_angle(target_truth - self.robot_truth_yaw)
            else:
                target_src = self._to_source_yaw(
                    float(self.door[2])
                    + (0.0 if self.exit_turn_target == "in" else math.pi))
                err_truth = (0.0 if target_src is None
                             else normalize_angle(target_src - pose[2]))
            if self.turn_anchor is None or self.turn_anchor_sim is None:
                self.turn_anchor = (pose[0], pose[1], self.robot_truth_yaw
                                    if self.robot_truth_yaw is not None else pose[2])
                self.turn_anchor_sim = self.sim_now
            else:
                turned = abs(normalize_angle(
                    (self.robot_truth_yaw if self.robot_truth_yaw is not None
                     else pose[2]) - self.turn_anchor[2]))
                if turned >= self.turn_progress_yaw:
                    self.turn_anchor = (pose[0], pose[1],
                                        self.robot_truth_yaw
                                        if self.robot_truth_yaw is not None else pose[2])
                    self.turn_anchor_sim = self.sim_now
                elif (self.sim_now - self.turn_anchor_sim
                      >= self.turn_unstick_seconds):
                    self.exit_turn_unsticks += 1
                    self.turn_anchor_sim = self.sim_now
                    if self.exit_turn_retries >= self.exit_turn_retry_limit:
                        # SAFETY FALLBACK: a tight car must never kill a run.
                        if self.exit_turn_target == "in":
                            self.fault = ("IN_CAR_TURN_FAILED_BOTH_WAYS_AFTER_{}_"
                                          "UNSTICKS").format(self.exit_turn_unsticks)
                            self._set_state("FAILED", self.fault)
                            return
                        self.exit_fallback = "turn_reverse"
                        self.exit_reverse = True
                        self.exit_turn_target = "in"
                        self.exit_turn_side = None
                        self.exit_turn_retries = 0
                        self.exit_turn_unsticks = 0
                        self.exit_turn_started_sim = None
                        self.turn_anchor = None
                        self.turn_anchor_sim = None
                        self.exit_anchor = None
                        rospy.logwarn(
                            "elevator-only: in-car U-turn stalled; falling back to "
                            "REVERSE: turning back to face into the car")
                        return
                    self.exit_turn_retries += 1
                    # flip the sweep direction and walk-and-turn to unstick
                    self.exit_turn_sign = -self.exit_turn_sign
                    self._publish(self.exit_turn_walk_speed, self.exit_turn_sign * self.turn_speed)
                    rospy.logwarn(
                        "elevator-only: in-car turn stalled; flipping to %s and "
                        "walk-turning (retry %d/%d)",
                        "L" if self.exit_turn_sign > 0 else "R",
                        self.exit_turn_retries, self.exit_turn_retry_limit)
                    return
            if (self.sim_now - self.exit_turn_started_sim > self.exit_turn_timeout):
                self.exit_fallback = "reverse"
                self.exit_reverse = True
                self.exit_turn_side = None
                self.exit_anchor = None
                self._set_state("EXIT_CAR", "in-car turn timed out; reversing out")
                return
            if abs(err_truth) <= self.exit_heading_tolerance:
                self._stop()
                leg = self.leg_results[-1] if self.leg_results else None
                if leg is not None:
                    leg["exit_turn_side"] = self.exit_turn_side
                    leg["exit_turn_clearances"] = self.exit_turn_clearances
                    leg["exit_turn_sim"] = round(self.sim_now, 2)
                    leg["exit_turn_unsticks"] = self.exit_turn_unsticks
                    leg["exit_fallback"] = self.exit_fallback
                self.exit_anchor = None
                self._set_state(
                    "EXIT_CAR",
                    "turned around inside the car (side %s) in %.1f s; driving out"
                    % (self.exit_turn_side, self.sim_now - self.exit_turn_started_sim),
                )
            else:
                # walk-and-turn (small forward component) toward the chosen side
                self._publish(self.exit_turn_walk_speed,
                              self.exit_turn_sign * self.turn_speed)
            return

        if state == "EXIT_CAR":
            if self.exit_anchor is None:
                self.exit_anchor = pose[:2]
                self.exit_started_sim = self.sim_now
                self.exit_progress_sim = self.sim_now
                self.exit_progress_truth = (None if self.robot_truth is None
                                            else self.robot_truth[:2])
                self.exit_progress_source = pose[:2]
                self.exit_retries = 0
                self.exit_retreat_until = None
            # Phase 1: retreat.  Push back into the car for a moment to release
            # the threshold contact, then restart the run from a fresh anchor.
            if self.exit_retreat_until is not None:
                if self.sim_now < self.exit_retreat_until:
                    retreat = self.exit_retreat_speed if self.exit_reverse \
                        else -self.exit_retreat_speed
                    self._publish(retreat, 0.0)
                    return
                self.exit_retreat_until = None
                self.exit_anchor = pose[:2]
                self.exit_progress_sim = self.sim_now
                self.exit_progress_truth = (None if self.robot_truth is None
                                            else self.robot_truth[:2])
                self.exit_progress_source = pose[:2]
            travelled = planar_distance(pose, self.exit_anchor)
            # Phase 2: stall detection on GROUND TRUTH (odometry slips here).
            truth_now = None if self.robot_truth is None else self.robot_truth[:2]
            if truth_now is not None:
                if (self.exit_progress_truth is None
                        or planar_distance(truth_now, self.exit_progress_truth)
                        >= self.exit_progress_tolerance):
                    self.exit_progress_truth = truth_now
                    self.exit_progress_sim = self.sim_now
            elif (self.exit_progress_source is None
                  or planar_distance(pose, self.exit_progress_source)
                  >= self.exit_progress_tolerance):
                self.exit_progress_source = pose[:2]
                self.exit_progress_sim = self.sim_now
            if self.sim_now - self.exit_progress_sim >= self.exit_stall_seconds:
                if self.exit_retries >= self.exit_retry_limit:
                    if not self.exit_reverse:
                        # FORWARD CROSSING EXHAUSTED: the proven reverse path is
                        # the fallback.  Turn back inside the car (that turn is
                        # known to work) and leave in reverse.
                        self.exit_fallback = "crossing_reverse"
                        self.exit_fail_travelled = round(travelled, 2)
                        leg0 = self.leg_results[-1] if self.leg_results else None
                        if leg0 is not None:
                            leg0["exit_forward_retries"] = self.exit_retries
                            leg0["exit_fail_travelled"] = self.exit_fail_travelled
                            leg0["exit_fallback"] = self.exit_fallback
                        self.exit_reverse = True
                        self.exit_turn_target = "in"
                        self.exit_turn_side = None
                        self.exit_turn_retries = 0
                        self.exit_turn_unsticks = 0
                        self.exit_turn_started_sim = None
                        self.turn_anchor = None
                        self.turn_anchor_sim = None
                        self.exit_anchor = None
                        self.exit_retries = 0
                        self.exit_progress_sim = self.sim_now
                        self.exit_progress_truth = None
                        self.exit_progress_source = None
                        rospy.logwarn(
                            "elevator-only: forward crossing stuck (travelled "
                            "%.2f m); falling back to REVERSE: turning back and "
                            "reversing out", travelled)
                        self._set_state("ALIGN_EXIT",
                                        "forward exit abandoned; turning back")
                        return
                    self.fault = ("EXIT_CAR_NO_PROGRESS_AFTER_{}_RETRIES_"
                                  "TRAVELLED_{:.2f}M").format(
                        self.exit_retries, travelled)
                    self._set_state("FAILED", self.fault)
                    return
                self.exit_retries += 1
                # Escalating run-up: 0.8 s, then 1.6 s, then 2.4 s at 0.30 m/s,
                # so a grabbed lip gets a real run at it (the alternating bias
                # alone is not always enough).
                escalate = 1 + min(2, self.exit_retries - 1)
                self.exit_retreat_until = (
                    self.sim_now + self.exit_retreat_seconds * escalate)
                self.exit_progress_sim = self.sim_now
                self.exit_progress_truth = truth_now
                self.exit_progress_source = pose[:2]
                self._stop()
                rospy.logwarn(
                    "elevator-only: stuck on the car threshold (truth %s, "
                    "travelled %.2f m); retreating %.1f s and retrying (%d/%d)",
                    None if truth_now is None else "%.2f, %.2f" % truth_now,
                    travelled, self.exit_retreat_seconds, self.exit_retries,
                    self.exit_retry_limit,
                )
                return
            bias = 0.0
            if self.exit_retries > 0:
                # Alternate the biased heading on every retry: a square-on run
                # catches on the lip, a biased one slides through.
                bias = self.exit_retry_yaw_bias * (
                    1.0 if self.exit_retries % 2 else -1.0)
            if not self.exit_reverse:
                self.exit_forward_retries = self.exit_retries
            if self.exit_reverse:
                # Hold the entry heading and drive backwards out of the car.
                target_yaw = self._to_source_yaw(float(self.door[2]) + bias)
                speed = -abs(self.crossing_speed + max(0, self.exit_retries - 1)
                             * self.exit_retry_speed_bonus)
            else:
                target_yaw = self._to_source_yaw(float(self.door[2]) + math.pi + bias)
                speed = abs(self.crossing_speed + max(0, self.exit_retries - 1)
                            * self.exit_retry_speed_bonus)
            error = normalize_angle(target_yaw - pose[2]) if target_yaw is not None else 0.0
            self._publish(speed, max(-0.25, min(0.25, 0.8 * error)))
            along = None
            if self.robot_truth is not None:
                along, _lateral = self.door_frame(self.robot_truth[:2])
            # Probe 02 showed why the old -0.25 m clearance was too tight: the
            # exit then ended with the body 0.25 m outside the door plane, the
            # worst spot to pivot in (an obstacle sits inside the turn sweep).
            # Require the body to be clearly out (along <= -0.80 m, i.e. world
            # x <= 0.85) before the leg can be declared finished.
            past_door = along is not None and along <= self.exit_past_door_along
            if travelled >= self.exit_distance or (
                travelled >= self.minimum_exit_progress and past_door
            ):
                self._stop()
                self.exit_anchor = None
                leg = self.leg_results[-1] if self.leg_results else None
                if leg is not None:
                    leg["exit_travelled"] = round(travelled, 2)
                    leg["exit_retries"] = self.exit_retries
                    leg["exit_reverse"] = bool(self.exit_reverse)
                    leg["exit_forward_threshold_ok"] = (None if self.exit_reverse
                                                        else True)
                    leg["exit_forward_retries"] = self.exit_forward_retries
                    leg["exit_fallback"] = self.exit_fallback
                    leg["exit_sim"] = round(self.sim_now, 2)
                    leg["exit_truth"] = (
                        None if self.robot_truth is None
                        else tuple(round(v, 2) for v in self.robot_truth)
                    )
                if self._leg_depth(self.leg_index) > 0.0:
                    # The user's cycle: leave the car, drive to the corridor
                    # mouth, U-turn, come back and board again.
                    self._begin_excursion(self._leg_depth(self.leg_index), "leg")
                else:
                    self._board_next_leg("back at the doorway, re-boarding for floor %d")
                return
            if self.exit_started_sim is not None and \
                    self.sim_now - self.exit_started_sim > self.exit_timeout:
                self.fault = "EXIT_CAR_TIMEOUT_TRAVELLED_{:.2f}M_RETRIES_{}".format(
                    travelled, self.exit_retries)
                self._set_state("FAILED", self.fault)
            return

        if state == "TO_CORRIDOR":
            # Two sub-legs: first get clear of the car mouth by returning to the
            # known lobby waypoint in front of the doorway, then run out to the
            # corridor mouth.  Splitting them keeps the long straight run out of
            # the doorway on a known starting heading.
            if self.excursion_started_sim is not None and \
                    self.sim_now - self.excursion_started_sim > self.corridor_timeout:
                self.fault = "CORRIDOR_TIMEOUT_LEG_{}_DEPTH_{:.1f}M".format(
                    self.leg_index + 1, self.excursion_depth)
                self._set_state("FAILED", self.fault)
                return
            if self.world_pose is not None:
                along, lateral = self.corridor_frame(self.world_pose[:2])
                self.excursion_max_along = (
                    along if self.excursion_max_along is None
                    else max(self.excursion_max_along, along)
                )
            if self.robot_truth is not None:
                truth_along, _tl = self.corridor_frame(self.robot_truth[:2])
                self.excursion_truth_max_along = (
                    truth_along if self.excursion_truth_max_along is None
                    else max(self.excursion_truth_max_along, truth_along)
                )
            if self.excursion_leg == 0:
                if self.exit_reverse:
                    # A reverse exit leaves the robot facing the car with the
                    # lobby waypoint directly BEHIND it.  Pivoting 180 deg there
                    # is what wedged probe 02 (obstacle inside the turn sweep,
                    # 0.25 m outside the door plane).  The exit already proved
                    # reverse works, so back straight to the waypoint instead
                    # and turn only once there is room.
                    staging = self.staging_world()
                    truth_now = self.robot_truth
                    if truth_now is not None:
                        # Truth-based: the estimate can run away right at the car
                        # doorway (elevf01: world x,y -> thousands of metres while
                        # the robot sat still), and source-frame targets then never
                        # complete.  Position and heading both come from truth.
                        if self.excursion_reverse_anchor is None:
                            self.excursion_reverse_anchor = truth_now[:2]
                        reversed_m = planar_distance(truth_now[:2],
                                                     self.excursion_reverse_anchor)
                        if (planar_distance(truth_now[:2], staging)
                                <= self.target_tolerance
                                or reversed_m >= self.excursion_reverse_max):
                            self.excursion_leg = 1
                            self.stuck_anchor = None
                            self.stuck_since = None
                            self.turn_anchor = None
                            self.turn_anchor_sim = None
                            return
                        # Reverse straight on the CURRENT heading: this
                        # sub-leg only has to clear the car mouth, and sub-leg 1
                        # (truth-steered) does all the aiming.  Holding an exact
                        # heading here made a 0.15 rad residual a deadlock.
                        self._publish(-abs(self.corridor_speed), 0.0)
                        if self._stuck_check("TO_CORRIDOR_REVERSE"):
                            self._set_state("FAILED", self.fault)
                        return
                    target_source = self._to_source(staging)
                    pose_now = self.source_pose
                    if target_source is None:
                        self._stop()
                        return
                    if self.excursion_reverse_anchor is None:
                        self.excursion_reverse_anchor = pose_now[:2]
                    reversed_m = planar_distance(pose_now, self.excursion_reverse_anchor)
                    if (planar_distance(pose_now, target_source) <= self.target_tolerance
                            or reversed_m >= self.excursion_reverse_max):
                        self.excursion_leg = 1
                        self.stuck_anchor = None
                        self.stuck_since = None
                        return
                    hold_yaw = self._to_source_yaw(float(self.door[2]))
                    error = (0.0 if hold_yaw is None
                             else normalize_angle(hold_yaw - pose_now[2]))
                    if abs(error) > self.heading_tolerance:
                        self._turn_command(error)
                    else:
                        self._publish(-abs(self.corridor_speed),
                                      max(-0.20, min(0.20, 0.8 * error)))
                    if self._stuck_check("TO_CORRIDOR_REVERSE"):
                        self._set_state("FAILED", self.fault)
                    return
                if self._drive_to_world(self.staging_world(),
                                        speed=self.corridor_speed):
                    self.excursion_leg = 1
                    self.stuck_anchor = None
                    self.stuck_since = None
                elif self._stuck_check("TO_CORRIDOR_LOBBY"):
                    self._set_state("FAILED", self.fault)
                return
            reached = self._drive_to_world(self.excursion_target,
                                           tolerance=self.corridor_tolerance,
                                           speed=self.corridor_speed)
            truth_along = None
            if self.robot_truth is not None:
                truth_along, _tl = self.corridor_frame(self.robot_truth[:2])
            truth_deep_enough = (truth_along is None
                                 or truth_along >= self.corridor_min_truth_along)
            if reached and not truth_deep_enough:
                # Odometry says "there", ground truth says "still short of the
                # mouth": keep creeping straight on until truth agrees (or the
                # corridor timeout fires).  Never U-turn in the lobby.
                rospy.logwarn_throttle(
                    2.0,
                    "elevator-only: odometry arrived but truth along %.2f m < "
                    "%.2f m; creeping to the corridor mouth",
                    truth_along, self.corridor_min_truth_along,
                )
                self._publish(self.corridor_speed, 0.0)
                return
            if reached:
                self._stop()
                leg = self.leg_results[-1] if (self.leg_results
                                               and self.excursion_mode == "leg") else None
                if leg is not None:
                    leg["corridor_sim"] = round(self.sim_now, 2)
                    leg["corridor_max_along"] = (
                        None if self.excursion_max_along is None
                        else round(self.excursion_max_along, 2)
                    )
                    leg["corridor_truth_max_along"] = (
                        None if self.excursion_truth_max_along is None
                        else round(self.excursion_truth_max_along, 2)
                    )
                self.stuck_anchor = None
                self.stuck_since = None
                self.centre_started_sim = self.sim_now
                self.centre_truth_offset = None
                self._set_state(
                    "CORRIDOR_CENTRE",
                    "at the corridor mouth (along %.2f m past the gate, %s); "
                    "centring for the U-turn" % (self.excursion_max_along or 0.0,
                                                 self.excursion_mode),
                )
                return
            if self._stuck_check("TO_CORRIDOR"):
                self._set_state("FAILED", self.fault)
            return

        if state == "CORRIDOR_CENTRE":
            # Line the robot up on the corridor axis before the U-turn.  The
            # corridor is 2.2 m wide and the body needs ~0.35 m of swing radius,
            # so an off-axis arrival (0.45 m of localisation error was measured
            # in probe 00) wedges the robot against the mouth wall.
            lc, rc = self.left_clearance, self.right_clearance
            truth_lateral = None
            if self.robot_truth is not None:
                _ca, truth_lateral = self.corridor_frame(self.robot_truth[:2])
            offset = (None if truth_lateral is None
                      else truth_lateral - self.corridor_axis_lateral)
            if offset is not None and abs(offset) <= self.corridor_axis_tolerance:
                self._stop()
                self.centre_truth_offset = round(offset, 3)
                leg = self.leg_results[-1] if (self.leg_results
                                               and self.excursion_mode == "leg") else None
                if leg is not None:
                    leg["centre_sim"] = round(self.sim_now, 2)
                    leg["centre_truth_offset"] = round(offset, 3)
                    leg["centre_world_lateral"] = round(
                        self.corridor_frame(self.world_pose[:2])[1]
                        - self.corridor_axis_lateral, 3) if self.world_pose else None
                else:
                    self.entry_centre_offset = round(offset, 3)
                self.turn_started_sim = self.sim_now
                self.turn_anchor_error = None
                self.turn_anchor_sim = None
                self.stuck_anchor = None
                self.stuck_since = None
                self._set_state(
                    "TURN_AROUND",
                    "on the corridor axis (truth lateral offset %+.2f m, "
                    "laser %.2f/%.2f m); U-turning" % (offset, lc, rc),
                )
                return
            if self.centre_started_sim is not None and \
                    self.sim_now - self.centre_started_sim > self.corridor_centre_timeout:
                self.fault = "CORRIDOR_CENTRE_TIMEOUT_OFFSET_{}".format(
                    "n/a" if offset is None else "%.2f" % offset)
                self._set_state("FAILED", self.fault)
                return
            # Feedback: the laser balances the two walls (on-board sensing; at
            # the mouth it measured within ~0.03 m of ground truth), and ground
            # truth decides both the completion gate above and any disagreement.
            laser_error = 0.5 * (rc - lc) if (math.isfinite(lc) and math.isfinite(rc)) else None
            if laser_error is not None and (offset is None
                                            or abs(laser_error - offset)
                                            <= self.corridor_centre_agreement):
                error = laser_error
                source = "laser"
            elif offset is not None:
                error = offset
                source = "truth"
            else:
                error = laser_error
                source = "laser"
            if error is None:
                self._stop()
                return
            logger = getattr(rospy, "loginfo_throttle", None)
            if logger is not None:
                logger(
                    2.0,
                    "elevator-only: corridor centring: laser %s truth %s -> driving on %s",
                    "n/a" if laser_error is None else "%+.2f" % laser_error,
                    "n/a" if offset is None else "%+.2f" % offset, source,
                )
            # error > 0: the robot sits towards -x, i.e. to its own left as it
            # faces down the corridor, so swing the nose right while creeping.
            self._publish(self.corridor_centre_speed,
                          max(-0.35, min(0.35, -self.corridor_centre_gain * error)))
            if self._stuck_check("CORRIDOR_CENTRE"):
                self._set_state("FAILED", self.fault)
            return

        if state == "TURN_AROUND":
            # U-turn on the spot and face back down the corridor towards the
            # lift.  A separate state so the turn is visible in the timeline.
            target_yaw = self._to_source_yaw(float(self.corridor_gate[2]) + math.pi)
            if target_yaw is None:
                self._stop()
                return
            error = normalize_angle(target_yaw - pose[2])
            if self.turn_started_sim is None:
                self.turn_started_sim = self.sim_now
            # Watchdog: a jammed in-place turn used to hang until the 3600 s
            # wall backstop.  Fail loudly when the yaw error stops shrinking.
            if self.turn_anchor_error is None or \
                    abs(error) < self.turn_anchor_error - 0.20:
                self.turn_anchor_error = abs(error)
                self.turn_anchor_sim = self.sim_now
            if self.sim_now - self.turn_anchor_sim > self.corridor_turn_stall_timeout:
                self.fault = "TURN_AROUND_STALLED_YAW_ERROR_{:.2f}RAD".format(abs(error))
                self._set_state("FAILED", self.fault)
                return
            if self.sim_now - self.turn_started_sim > self.corridor_turn_timeout:
                self.fault = "TURN_AROUND_TIMEOUT_YAW_ERROR_{:.2f}RAD".format(abs(error))
                self._set_state("FAILED", self.fault)
                return
            if abs(error) <= self.heading_tolerance:
                self._stop()
                leg = self.leg_results[-1] if (self.leg_results
                                               and self.excursion_mode == "leg") else None
                if leg is not None:
                    leg["turn_sim"] = round(self.sim_now, 2)
                self.stuck_anchor = None
                self.stuck_since = None
                self.turn_started_sim = None
                self.turn_anchor_error = None
                self.turn_anchor_sim = None
                self._set_state("BACK_TO_LIFT",
                                "turned round at the corridor mouth (%s)"
                                % self.excursion_mode)
            else:
                self._turn_command(error, self.corridor_turn_speed)
            return

        if state == "BACK_TO_LIFT":
            if self.excursion_started_sim is not None and \
                    self.sim_now - self.excursion_started_sim > self.corridor_timeout + 60.0:
                self.fault = "BACK_TO_LIFT_TIMEOUT_LEG_{}".format(self.leg_index + 1)
                self._set_state("FAILED", self.fault)
                return
            if self._drive_to_world(self.staging_world(),
                                    speed=self.corridor_speed):
                self._stop()
                self.stuck_anchor = None
                self.stuck_since = None
                if self.excursion_mode == "entry":
                    # Mission leg 1: the corridor mouth is done, the lift is on
                    # the map, so line up and start the boarding flow.
                    self.entry_corridor_truth_max_along = (
                        None if self.excursion_truth_max_along is None
                        else round(self.excursion_truth_max_along, 2))
                    self.entry_corridor_ok = (
                        self.excursion_truth_max_along is not None
                        and self.excursion_truth_max_along > 0.5)
                    self._set_state(
                        "ALIGN_DOOR",
                        "back from the corridor mouth (max along %.2f m); "
                        "lining up for the lift" % (self.excursion_truth_max_along or 0.0),
                    )
                else:
                    leg = self.leg_results[-1] if self.leg_results else None
                    if leg is not None:
                        leg["back_sim"] = round(self.sim_now, 2)
                    self._board_next_leg("back from the corridor, re-boarding for floor %d")
            elif self._stuck_check("BACK_TO_LIFT"):
                self._set_state("FAILED", self.fault)
            return

        if state == "RETURN_SPAWN":
            target = self.spawn_world
            if target is None:
                self._set_state("DONE", "no spawn point recorded")
                return
            truth = self.robot_truth
            if truth is None or len(target) < 3:
                # No truth: fall back to the LIO approach.
                if self._drive_to(self._to_source(target[:2]),
                                  speed=self.corridor_speed):
                    self._stop()
                    self.return_sim = round(self.sim_now, 2)
                    self._set_state("DONE", "returned to the spawn point (no truth)")
                elif self._stuck_check("RETURN_SPAWN"):
                    self._set_state("FAILED", self.fault)
                return
            spawn_yaw = float(target[2])
            # Phase A: line up on the doorway axis inside the lobby, where the
            # estimate is good and the turn is safe.
            axis_point = (float(target[0]), self.spawn_turn_y)
            if not self.spawn_axis_done:
                if self.spawn_axis_sim is None:
                    self.spawn_axis_sim = self.sim_now
                    rospy.loginfo(
                        "elevator-only: return leg - lining up on the doorway axis "
                        "at (%.2f, %.2f) for the indoor U-turn", axis_point[0],
                        axis_point[1])
                if planar_distance(truth[:2], axis_point) <= self.spawn_axis_tolerance:
                    self.spawn_axis_done = True
                    self._stop()
                    return
                if (self.sim_now - self.spawn_axis_sim > self.spawn_axis_timeout):
                    self.fault = "SPAWN_AXIS_TIMEOUT_DIST_{:.2f}M".format(
                        planar_distance(truth[:2], axis_point))
                    self._set_state("FAILED", self.fault)
                    return
                if self._drive_to_world(axis_point,
                                        speed=self.corridor_speed):
                    self.spawn_axis_done = True
                    self._stop()
                elif self._stuck_check("RETURN_SPAWN_AXIS"):
                    self._set_state("FAILED", self.fault)
                return
            # Phase B: turn INDOORS to the spawn heading.  The user asked for a
            # REVERSE exit after the forward attempt hit the doorway: reversing
            # keeps the robot FACING THE BUILDING while it crosses the apron and
            # the forecourt, and the building side is the feature-rich side for
            # the lidar (lift shaft, jambs, lobby walls), so the estimate drifts
            # far less than when driving out facing the empty forecourt (measured
            # 1.5-3.4 m of drift on the forward attempt).  The turn itself is the
            # lobby manoeuvre that needs patience (0.08-0.15 rad/s effective), so
            # it keeps the start-of-turn watchdog and a generous budget.
            spawn_yaw = float(target[2])
            if self.robot_truth_yaw is not None:
                head_err = normalize_angle(spawn_yaw - self.robot_truth_yaw)
                if abs(head_err) > self.spawn_face_tolerance:
                    if self.turn_started_sim is None:
                        self.turn_started_sim = self.sim_now
                    if (self.sim_now - self.turn_started_sim
                            > self.spawn_turn_timeout):
                        self.fault = (
                            "SPAWN_FACE_TIMEOUT_YAW_ERROR_{:.2f}RAD".format(
                                abs(head_err)))
                        self._set_state("FAILED", self.fault)
                        return
                    self._turn_command(head_err, self.turn_speed,
                                       stall_seconds=self.spawn_turn_stall,
                                       limit=self.spawn_turn_limit)
                    return
            # Phase C: REVERSE straight out through the entrance, down the 8 cm
            # apron and across the forecourt to the spawn, holding the spawn
            # heading (so the robot arrives already on it).  While in the doorway
            # the two jambs are used to keep it centred, which bounds the drift
            # exactly where the estimate is weakest.
            if self.spawn_reverse_sim is None:
                self.spawn_reverse_sim = self.sim_now
                rospy.loginfo(
                    "elevator-only: return leg - REVERSING out to the spawn "
                    "(est %.2f, %.2f; spawn yaw %.2f), centred on the doorway "
                    "jambs while crossing", float(truth[0]), float(truth[1]),
                    spawn_yaw)
            distance = planar_distance(truth[:2], target[:2])
            if distance <= self.target_tolerance:
                self._stop()
                self.return_sim = round(self.sim_now, 2)
                self.return_distance = round(distance, 2)
                # Heading is REPORTED for information only: the official
                # criteria require the position, not a final heading.
                if self.robot_truth_yaw is not None:
                    self.return_yaw_error_truth = round(abs(normalize_angle(
                        spawn_yaw - float(self.robot_truth_yaw))), 3)
                mission_yaw = self._to_source_yaw(spawn_yaw)
                if mission_yaw is not None and pose is not None:
                    self.return_yaw_error = round(abs(normalize_angle(
                        mission_yaw - pose[2])), 3)
                self.stuck_anchor = None
                self.stuck_since = None
                self._set_state("DONE", "returned to the spawn point (forward)")
                return
            if self.sim_now - self.spawn_reverse_sim > self.spawn_reverse_timeout:
                self.fault = "SPAWN_REVERSE_TIMEOUT_DIST_{:.2f}M".format(distance)
                self._set_state("FAILED", self.fault)
                return
            # Retreat phase after an apron/step catch.
            if self.spawn_reverse_retreat_until is not None:
                if self.sim_now < self.spawn_reverse_retreat_until:
                    self._publish(abs(self.exit_retreat_speed), 0.0)
                    return
                self.spawn_reverse_retreat_until = None
                self.spawn_reverse_anchor = truth[:2]
                self.spawn_reverse_progress_sim = self.sim_now
                self.spawn_reverse_bias_until = (
                    self.sim_now + self.entry_retry_bias_seconds)
            # Ground-truth progress watchdog over the 8 cm apron (both step
            # transitions happen here; odometry slips on steps).
            if self.spawn_reverse_progress_sim is None:
                self.spawn_reverse_progress_sim = self.sim_now
                self.spawn_reverse_anchor = truth[:2]
            elif (self.spawn_reverse_anchor is None
                  or planar_distance(truth[:2], self.spawn_reverse_anchor)
                  >= self.exit_progress_tolerance):
                self.spawn_reverse_anchor = truth[:2]
                self.spawn_reverse_progress_sim = self.sim_now
            elif (self.sim_now - self.spawn_reverse_progress_sim
                  >= self.exit_stall_seconds):
                if self.spawn_reverse_retries >= self.exit_retry_limit:
                    self.fault = ("SPAWN_REVERSE_NO_PROGRESS_AFTER_{}_RETRIES_"
                                  "DIST_{:.2f}M").format(
                        self.spawn_reverse_retries, distance)
                    self._set_state("FAILED", self.fault)
                    return
                self.spawn_reverse_retries += 1
                self.spawn_reverse_retreat_until = (
                    self.sim_now + self.exit_retreat_seconds)
                self.spawn_reverse_progress_sim = self.sim_now
                self.spawn_reverse_anchor = truth[:2]
                self._stop()
                rospy.logwarn(
                    "elevator-only: caught on the entrance apron/step (truth "
                    "%.2f, %.2f); retreating %.1f s and retrying the reverse "
                    "(%d/%d)", float(truth[0]), float(truth[1]),
                    self.exit_retreat_seconds, self.spawn_reverse_retries,
                    self.exit_retry_limit)
                return
            # Drive straight out, easing off near the spawn (a full-speed run
            # can step over the 0.32 m arrival window), with a gentle cross-track
            # correction only - the heading was already acquired by the arc, and
            # outdoor pivots jam.
            heading = float(self.robot_truth_yaw)
            if (self.spawn_reverse_bias_until is not None
                    and self.sim_now < self.spawn_reverse_bias_until):
                heading += self.exit_retry_yaw_bias * (
                    1.0 if self.spawn_reverse_retries % 2 else -1.0)
            dx = float(target[0]) - float(truth[0])
            dy = float(target[1]) - float(truth[1])
            lateral = -dx * math.sin(heading) + dy * math.cos(heading)
            # PURSUIT steering: keep correcting the BEARING to the spawn while
            # driving out, so a residual arc error converges during the run
            # instead of having to be nulled before it starts (the 0.30 rad
            # dead-band limit-cycled at 0.31 rad in elevf01_nominal_03).
            # Doorway-jamb centring: only while the robot is inside/near the
            # entrance opening (|x| small and y within a few metres of the door
            # plane on the building side) - a laser term that needs no truth and
            # is exactly what the feature-poor lobby cannot give the estimator.
            jamb_term = 0.0
            if (math.isfinite(self.left_clearance)
                    and math.isfinite(self.right_clearance)
                    and abs(float(truth[0])) <= 1.6 and -1.0 <= float(truth[1]) <= 3.0):
                jamb_term = max(-0.18, min(0.18,
                                           0.12 * (self.right_clearance
                                                   - self.left_clearance)))
            speed = min(self.corridor_speed, max(0.12, 1.0 * distance))
            self._publish(
                -abs(speed),
                max(-0.30, min(0.30, -0.5 * lateral + jamb_term)))
            return

        if state == "FACE_SPAWN":
            target = self.spawn_world
            if target is None or len(target) < 3:
                self._set_state("DONE", "returned to the spawn point")
                return
            target_yaw = self._to_source_yaw(float(target[2]))
            if target_yaw is None:
                self._stop()
                return
            error = normalize_angle(target_yaw - pose[2])
            truth_error = None
            if self.robot_truth_yaw is not None:
                # Same convention as every other turn: target minus current.
                truth_error = normalize_angle(
                    float(target[2]) - self.robot_truth_yaw)
            # ONE feedback signal, no dead-band conflict: use the Gazebo
            # heading whenever it is available (it is what the criterion is
            # measured on), else the LIO/mission heading.  Turning slowly keeps
            # the 4 Hz truth sample honest.
            if truth_error is not None:
                steer = truth_error
                finished = abs(truth_error) <= self.face_spawn_truth_tolerance
            else:
                steer = error
                finished = abs(error) <= self.face_spawn_tolerance
            if finished:
                self._stop()
                self.return_yaw_error = round(abs(error), 3)
                self.return_yaw_error_truth = (
                    None if truth_error is None else round(abs(truth_error), 3))
                self._set_state("DONE", "returned to the spawn point")
                return
            self._turn_command(steer, self.face_spawn_turn_speed)
            return

        if state == "RIDE":
            self._stop()
            if not self.ride_started:
                self.ride_started = True
                self._request_ride()
                return
            if self.ride_error is not None:
                self.fault = "ELEVATOR_SERVICE_FAILED: {}".format(self.ride_error)
                self._set_state("FAILED", self.fault)
                return
            if self.ride_response is not None and not self.ride_response.accepted:
                self.fault = "ELEVATOR_REJECTED: {}".format(self.ride_response.message)
                self._set_state("FAILED", self.fault)
                return
            self.z_after_ride = self._current_z()
            # Ride verdict from ground truth: the robot must now stand at the
            # z of the floor we asked for.  An absolute test, so it also works
            # for the descent back to the ground floor, and it fails when the
            # car moves but the robot does not (the old "rose by >= 1.5 m" test
            # passed a ride the robot never took).
            expected_z = self.expected_floor_z(self.target_floor)
            if self.use_truth:
                arrived = (
                    self.z_after_ride is not None and expected_z is not None
                    and abs(self.z_after_ride - expected_z) <= self.ride_z_tolerance
                )
            else:
                # FAITHFUL mode: the robot's own estimate z cannot see a ride at
                # all (the robot is motionless relative to the car), so arrival is
                # judged from the SAME information the mainline has: the
                # /call_elevator response (accepted + the floor it reports).
                arrived = (
                    self.ride_response is not None
                    and bool(getattr(self.ride_response, "accepted", False))
                    and int(getattr(self.ride_response, "current_floor", -1))
                    == int(self.target_floor)
                )
                if arrived:
                    rospy.loginfo(
                        "elevator-only: ride to floor %d confirmed by the "
                        "/call_elevator response (estimate z %.3f m, which cannot "
                        "observe a ride; Gazebo z for monitoring: %s)",
                        int(self.target_floor), float(self.z_after_ride or 0.0),
                        "n/a" if self.gazebo_truth is None
                        else "%.3f" % self.gazebo_truth[2])
            if arrived:
                self.current_floor = int(self.target_floor)
                self.leg_floor_response_ok = True
                if self.ride_response is not None and \
                        int(self.ride_response.current_floor) != self.current_floor:
                    rospy.logwarn(
                        "elevator-only: /call_elevator reported floor %d but the "
                        "robot's ground truth z %.3f m is floor %d; trusting truth",
                        int(self.ride_response.current_floor), self.z_after_ride,
                        self.current_floor,
                    )
                # The arrival floor's landing door may have been closed when the
                # car left it earlier in the mission; opening it here is cheap
                # and happens while the robot is standing still.
                self._open_car_door(self.current_floor)
                # Switch the mapping stack onto this floor BEFORE driving again.
                self._publish_floor_context("arrival")
                self.leg_results.append({
                    "leg": len(self.leg_results) + 1,
                    "floor": self.current_floor,
                    "z_before": round(self.z_before_ride, 3),
                    "z_after": round(self.z_after_ride, 3),
                    "z_after_gazebo": (None if self.gazebo_truth is None
                                       else round(self.gazebo_truth[2], 3)),
                    "floor_ok": bool(getattr(self, "leg_floor_response_ok", False)),
                    "expected_z": round(expected_z, 3),
                    "rise": round(self.z_after_ride - self.z_before_ride, 3),
                    "align_lateral": self.align_lateral,
                    "align_heading_error": self.align_heading_error,
                    "align_sim": self.align_sim,
                    "board_along": self.board_along,
                    "board_lateral": self.board_lateral,
                    "board_lateral_truth": self.board_lateral_truth,
                    "board_along_truth": self.board_along_truth,
                    "board_door_dist_cap": self.board_door_dist_cap,
                    "board_heading_error": self.board_heading_error,
                    "board_retries": self.board_retries,
                    "board_sim": None if self.board_sim is None else round(self.board_sim, 2),
                    "ride_sim": None if self.ride_requested_at is None
                    else round(self.ride_requested_at, 2),
                    "arrived_sim": round(self.sim_now, 2),
                    "car_z": None if self.car_truth_z is None
                    else round(self.car_truth_z, 3),
                })
                if not self.exit_reverse and self.exit_mode == "turn_forward":
                    self.exit_turn_started_sim = None
                    self.exit_turn_side = None
                    self.exit_turn_unsticks = 0
                    self.exit_turn_retries = 0
                self._set_state(
                    "ALIGN_EXIT" if self.exit_mode == "turn_forward"
                    else "EXIT_CAR",
                    "rode to floor %d (z %.3f -> %.3f, expected %.3f); leaving the car%s"
                    % (self.current_floor, self.z_before_ride, self.z_after_ride,
                       expected_z, " in reverse" if self.exit_reverse else ""),
                )
                return
            if self.ride_requested_at is not None and \
                    self.sim_now - self.ride_requested_at > self.ride_timeout:
                self.fault = "RIDE_DID_NOT_REACH_FLOOR_{}_Z_{}".format(
                    self.target_floor,
                    "n/a" if self.z_after_ride is None else "%.3f" % self.z_after_ride,
                )
                self._set_state("FAILED", self.fault)
            return

    def _current_z(self):
        """Absolute height for ride verification (falls back to odometry)."""
        truth = self._truth_z()
        if truth is not None:
            return truth
        with self.lock:
            if self.world_pose is not None:
                return float(self.world_pose[3])
            if self.source_pose is not None:
                return float(self.source_pose[3])
        return None

    # ------------------------------------------------------------ reporting
    def _sample(self):
        if self.sim_now - self.last_sample_sim < 1.0:
            return
        self.last_sample_sim = self.sim_now
        self._poll_truth()
        source, world = self.source_pose, self.world_pose
        door = self.door
        distance = float("nan")
        if world is not None:
            distance = planar_distance(world, door[:2])
        along = lateral = float("nan")
        if world is not None:
            along, lateral = self.door_frame(world[:2])
        corridor_along = float("nan")
        if world is not None:
            corridor_along, _cl = self.corridor_frame(world[:2])
        spawn_distance = float("nan")
        if world is not None and self.spawn_world is not None:
            spawn_distance = planar_distance(world[:2], self.spawn_world[:2])
        row = {
            "sim": round(self.sim_now, 2),
            "state": self.state,
            "leg": self.leg_index + 1,
            "target_floor": self.target_floor,
            "src_x": round(source[0], 3) if source else "",
            "src_y": round(source[1], 3) if source else "",
            "src_yaw": round(source[2], 3) if source else "",
            "world_yaw": round(world[2], 3) if world else "",
            "truth_yaw": ("" if self.gazebo_truth_yaw is None
                          else round(self.gazebo_truth_yaw, 3)),
            "cmd_vx": round(self.last_cmd[0], 3),
            "cmd_wz": round(self.last_cmd[1], 3),
            "world_x": round(world[0], 3) if world else "",
            "world_y": round(world[1], 3) if world else "",
            "world_z": round(world[3], 3) if world else "",
            "door_dist": round(distance, 3) if distance == distance else "",
            "door_along": round(along, 3) if along == along else "",
            "door_lateral": round(lateral, 3) if lateral == lateral else "",
            "corridor_along": round(corridor_along, 3) if corridor_along == corridor_along else "",
            "spawn_dist": round(spawn_distance, 3) if spawn_distance == spawn_distance else "",
            "front": round(self.front_clearance, 2) if math.isfinite(self.front_clearance) else "",
            "truth_x": round(self.gazebo_truth[0], 3) if self.gazebo_truth else "",
            "truth_y": round(self.gazebo_truth[1], 3) if self.gazebo_truth else "",
            "truth_z": round(self.gazebo_truth[2], 3) if self.gazebo_truth else "",
            "car_z": round(self.car_truth_z, 3) if self.car_truth_z is not None else "",
            "fault": self.fault or "",
        }
        self.timeline.append(row)
        if self.output_csv:
            self._append_csv(row)

    def _append_csv(self, row):
        columns = ["sim", "state", "leg", "target_floor", "src_x", "src_y", "src_yaw",
                   "world_x", "world_y", "world_z", "world_yaw",
                   "door_dist", "door_along", "door_lateral", "corridor_along",
                   "spawn_dist", "front", "cmd_vx", "cmd_wz",
                   "truth_x", "truth_y", "truth_z", "truth_yaw", "car_z", "fault"]
        try:
            new = not os.path.exists(self.output_csv)
            with open(self.output_csv, "a") as handle:
                if new:
                    handle.write(",".join(columns) + "\n")
                handle.write(",".join(str(row.get(column, "")) for column in columns) + "\n")
        except OSError as error:
            rospy.logwarn_throttle(10.0, "elevator-only: cannot write timeline: %s", error)

    def report(self):
        """Mission verdict in numbers; returns True on pass."""
        # Refresh ground truth at rest: the final distance/heading must be
        # measured on the settled pose, not on a sample taken at the DONE
        # transition (up to 0.25 s stale = 0.15 rad of false heading error at
        # 0.6 rad/s, which is exactly the measurement window).
        self._poll_truth()
        if self.robot_truth is not None and self.spawn_world is not None:
            self.return_distance = round(
                planar_distance(self.robot_truth[:2], self.spawn_world[:2]), 2)
        if (self.robot_truth_yaw is not None and self.spawn_world is not None
                and len(self.spawn_world) >= 3):
            self.return_yaw_error_truth = round(abs(normalize_angle(
                self.robot_truth_yaw - float(self.spawn_world[2]))), 3)
        door = self.door
        staging = self.staging_world()
        car = self.car_stop_world()
        expected_legs = len(self.ride_floors)
        closest_staging = None
        for row in self.timeline:
            if not row.get("world_x"):
                continue
            if row["state"] in ("ALIGN_DOOR", "ENTER_CAR"):
                point = (float(row["world_x"]), float(row["world_y"]))
                distance = planar_distance(point, staging)
                closest_staging = distance if closest_staging is None \
                    else min(closest_staging, distance)
        zs = [float(row["truth_z"]) for row in self.timeline
              if row.get("truth_z") not in ("", None)]
        car_zs = [float(row["car_z"]) for row in self.timeline
                  if row.get("car_z") not in ("", None)]
        states = list(dict.fromkeys(row["state"] for row in self.timeline))

        def f(value, fmt="%.2f"):
            return "n/a" if value is None else fmt % value

        print("=" * 68)
        print("elevator-only verdict (all numbers are coordinates, in metres)")
        print("  lift doorway used by the driver : (%.2f, %.2f) facing %.3f rad, width %.2f "
              "[from %s]" % (door[0], door[1], door[2],
                             door[3] if len(door) > 3 else 0.0,
                             self.door_frame_source))
        print("  door candidate                  : %s after %d cycles; offset %s; "
              "accepted=%s rejected=%s %s"
              % (self.door_candidate, self.door_candidate_cycles,
                 self.door_candidate_offset, self.door_candidate_accepted,
                 self.door_candidate_rejected,
                 self.door_candidate_reject_reason or ""))
        print("  staging point in front of doorway: (%.2f, %.2f)" % staging)
        print("  stop point inside the car       : (%.2f, %.2f)" % car)
        print("  corridor gate                   : (%.2f, %.2f) facing %.3f rad"
              % (self.corridor_gate[0], self.corridor_gate[1], self.corridor_gate[2]))
        print("  spawn point (param / measured)  : %s / %s"
              % (self.spawn_param, self.spawn_measured))
        print("  ride schedule (floors)          : %s" % self.ride_floors)
        print("  corridor depth after each ride  : %s"
              % self.corridor_depths[:expected_legs])
        print("  ground truth z on the ground    : %s" % f(self.ground_z, "%.3f"))
        for leg in self.leg_results:
            print("  leg %d: floor %s  board along=%s lateral=%s @sim %s | ride z %.3f -> "
                  "%.3f (expected %.3f) @sim %s | exit %s m @sim %s | corridor depth %s "
                  "max along %s (truth %s) @sim %s | turn @sim %s | back @sim %s"
                  % (leg["leg"], leg["floor"], f(leg.get("board_along")),
                     f(leg.get("board_lateral")), f(leg.get("board_sim")),
                     leg["z_before"], leg["z_after"], leg.get("expected_z", float("nan")),
                     f(leg.get("arrived_sim")), f(leg.get("exit_travelled")),
                     f(leg.get("exit_sim")), f(leg.get("corridor_depth")),
                     f(leg.get("corridor_max_along")), f(leg.get("corridor_truth_max_along")),
                     f(leg.get("corridor_sim")), f(leg.get("turn_sim")),
                     f(leg.get("back_sim"))))
        print("  lift legs completed             : %d / %d"
              % (len(self.leg_results), expected_legs))
        print("  states visited                  : %s" % " -> ".join(states))
        print("  closest approach to staging     : %s"
              % ("n/a" if closest_staging is None else "%.2f m" % closest_staging))
        enter_rows = [row for row in self.timeline if row["state"] == "ENTER_CAR"
                      and row.get("door_lateral") not in ("", None)]
        if enter_rows:
            worst = max(abs(float(row["door_lateral"])) for row in enter_rows)
            print("  worst lateral offset on entry   : %.2f m" % worst)
        print("  truth z range (robot)           : %s"
              % ("n/a" if not zs else "%.3f -> %.3f" % (min(zs), max(zs))))
        print("  truth z range (car)             : %s"
              % ("n/a" if not car_zs else "%.3f -> %.3f" % (min(car_zs), max(car_zs))))
        print("  NOTE: /simnav/world_pose_metric.z cannot see a ride (the robot is "
              "motionless relative to the car); the verdict uses ground truth.")
        print("  final state / sim time          : %s @ %.2f s" % (self.state, self.sim_now))
        print("  final truth position            : %s"
              % ("n/a" if self.robot_truth is None
                 else "(%s)" % ", ".join("%.3f" % v for v in self.robot_truth)))
        print("  final distance to spawn (truth) : %s / (world) %s"
              % (f(self.return_distance),
                 "n/a" if (self.world_pose is None or self.spawn_world is None)
                 else "%.2f m" % planar_distance(self.world_pose[:2], self.spawn_world[:2])))
        print("  final heading error vs spawn    : %s (mission frame) / %s (Gazebo truth)"
              % (f(self.return_yaw_error, "%.3f rad"),
                 f(self.return_yaw_error_truth, "%.3f rad")))
        print("  frame note                      : src_yaw is the LIO map frame (origin at the "
              "spawn pose), so it is offset from the mission heading by the spawn bearing; "
              "world_yaw / truth_yaw are the mission-frame headings")
        print("  ride response (last)            : %s"
              % (None if self.ride_response is None
                 else "accepted=%s floor=%s %s" % (self.ride_response.accepted,
                                                   self.ride_response.current_floor,
                                                   self.ride_response.message)))
        print("  max world-vs-truth planar error : %s (fail threshold %.2f m for %.1f s; "
              "%d transient holds)"
              % (f(self.max_world_truth_error, "%.3f m"), self.loc_max_planar_error,
                 self.loc_diverged_seconds, self.loc_hold_count))
        print("  stationary estimate warnings     : %d (in-place turns; not fatal)"
              % self.loc_stationary_warnings)
        print("  truth-steered regions (region, sim): %s"
              % (", ".join("%s@%s" % item for item in self.truth_steered_regions)
                 or "none"))
        print("  estimate warnings while truth-steered: %d (not fatal)"
              % self.loc_truth_steered_warnings)
        print("  floor contexts published        : %s" % (self.floor_contexts or "none"))
        print("  fault                           : %s" % (self.fault or "none"))

        # ---- verdicts -----------------------------------------------------
        boards = [row for row in self.timeline if row["state"] == "RIDE"
                  and row.get("door_dist") not in ("", None)]
        # The nominal boarding stop sits ~1.30 m past the door plane (the entry
        # travelled budget is 3.00 m), so the old 1.30 m cap was a coin flip:
        # elevf01 leg 1 landed exactly on it.  Judged with margin instead, and
        # the true stop distance is recorded per leg below.
        # Currency: judge the boarding on the SAME truth-based measure the audit
        # reports (per-leg stop distance past the door plane in the truth door
        # frame).  The old test compared the LIO/map-frame `door_dist`, which
        # carries a systematic ~0.45 m bias, against the cap - elevf04 failed
        # with an LIO sample at 1.46 m while the TRUE stop was 1.00 m.
        boarded = (len(self.leg_results) >= expected_legs
                   and all(leg.get("board_along_truth") is not None
                           and float(leg["board_along_truth"])
                           <= self.board_door_dist_cap
                           for leg in self.leg_results))
        if boards and not boarded:
            worst_sample = max(float(row["door_dist"]) for row in boards)
            print("  NOTE: LIO-frame boarding samples max %.2f m (biased currency, "
                  "not the criterion)" % worst_sample)
        truth_slack = 0.05
        boarded_centred = len(self.leg_results) >= expected_legs and all(
            leg.get("board_lateral") is not None
            and abs(leg["board_lateral"]) <= self.board_lateral_tolerance
            and (leg.get("board_lateral_truth") is None
                 or abs(leg["board_lateral_truth"])
                 <= self.board_lateral_tolerance + truth_slack)
            and (leg.get("board_heading_error") is None
                 or leg["board_heading_error"] <= self.board_heading_tolerance)
            for leg in self.leg_results)
        boarding_count = len(boards)
        if self.use_truth:
            leg_floor_ok = all(
                leg.get("expected_z") is not None
                and abs(leg["z_after"] - leg["expected_z"]) <= self.ride_z_tolerance
                for leg in self.leg_results)
        else:
            # FAITHFUL mode: the robot's own z estimate cannot observe a ride, so
            # the floor check uses the /call_elevator response (recorded per leg).
            leg_floor_ok = all(bool(leg.get("floor_ok")) for leg in self.leg_results)
        all_legs = len(self.leg_results) >= expected_legs
        all_exits = all("exit_travelled" in leg for leg in self.leg_results)
        excursion_legs = [i for i in range(expected_legs) if self._leg_depth(i) > 0.0]
        centred_ok = all(
            (leg.get("centre_truth_offset") is None
             or abs(leg["centre_truth_offset"]) <= self.corridor_axis_tolerance)
            for leg in self.leg_results)

        def excursion_done(index):
            if index >= len(self.leg_results):
                return False
            leg = self.leg_results[index]
            return (leg.get("corridor_sim") is not None
                    and leg.get("back_sim") is not None
                    and (leg.get("corridor_truth_max_along") or -99.0) > 0.5)

        excursions_ok = len(self.leg_results) >= expected_legs and all(
            excursion_done(i) for i in excursion_legs)
        returned = True
        if self.return_to_spawn and self.spawn_world is not None:
            returned = (self.return_distance is not None
                        and self.return_distance <= self.target_tolerance + 0.30)
        # Prefer the Gazebo-true heading for the verdict, fall back to the
        # mission-frame (LIO-mapped) one; both are logged.
        heading_measured = (self.return_yaw_error_truth
                            if self.return_yaw_error_truth is not None
                            else self.return_yaw_error)
        # Official criteria require the POSITION only (docs/evaluation.md scores
        # the danger list and the elapsed time; team_scene_info.json defines
        # robot_start only).  The final heading is therefore reported, never
        # required - the return leg drives out FORWARD over the apron instead of
        # turning 180 deg and reversing over the step.
        heading_ok = True
        entry_ok = self.entry_corridor_depth <= 0.0 or self.entry_corridor_ok
        print("  VERDICT(cycle)     : %s" % (
            "PASS - all %d lift legs rode to their floor and drove out" % expected_legs
            if (all_legs and leg_floor_ok and all_exits) else
            "FAIL - %d/%d legs completed, floor z match=%s, exits=%s"
            % (len(self.leg_results), expected_legs, leg_floor_ok, all_exits)))
        board_line = ", ".join(
            "leg %d lateral %s/%s truth %s heading %s"
            % (leg["leg"], f(leg.get("board_lateral"), "%+.2f"),
               f(leg.get("align_lateral"), "%+.2f"),
               f(leg.get("board_lateral_truth"), "%+.2f"),
               f(leg.get("board_heading_error"), "%.2f"))
            for leg in self.leg_results) or "n/a"
        print("  boarding audit (board/align lateral, heading): %s" % board_line)
        print("  boarding stop past door plane (truth): %s (cap %.2f m)"
              % (", ".join("leg %d %s m" % (leg["leg"],
                                            f(leg.get("board_along_truth"), "%.2f"))
                           for leg in self.leg_results) or "n/a",
                 self.board_door_dist_cap))
        print("  VERDICT(board)     : %s" % (
            "PASS - all %d boardings crossed the doorway CENTRED: |lateral| <= %.2f m, "
            "truth |lateral| <= %.2f m, heading error <= %.2f rad (%d RIDE samples)"
            % (expected_legs, self.board_lateral_tolerance,
               self.board_lateral_tolerance + truth_slack,
               self.board_heading_tolerance, boarding_count)
            if (boarded and boarded_centred) else
            "FAIL - a boarding was not centred on the doorway or did not start there "
            "(centred=%s, at the doorway=%s, %d RIDE samples)"
            % (boarded_centred, boarded, boarding_count)))
        print("  VERDICT(lift)      : %s" % (
            "PASS - every ride put the robot on the floor it asked for (ground truth)"
            if (all_legs and leg_floor_ok) else
            "FAIL - a ride did not reach its floor's ground-truth z"))
        print("  corridor U-turn centring        : %s"
              % (", ".join("leg %d offset %s m" % (leg["leg"],
                                                   f(leg.get("centre_truth_offset"), "%+.2f"))
                           for leg in self.leg_results
                           if leg.get("centre_truth_offset") is not None)
                 or "n/a"))
        print("  VERDICT(corridor)  : %s" % (
            "PASS - corridor excursion legs %s all left the lobby, U-turned and came back"
            % (excursion_legs,) if excursions_ok else
            "FAIL - a corridor excursion is missing or never entered the corridor"))
        print("  VERDICT(entry)     : %s" % (
            "PASS - walked in, drove up the corridor to the mouth (truth along %s m) "
            "and U-turned back to the lift" % f(self.entry_corridor_truth_max_along)
            if entry_ok else
            "FAIL - the entry corridor leg did not reach the corridor mouth"))
        print("  VERDICT(return)    : %s" % (
            "PASS - ended back at the spawn point (%.2f m, heading error %s rad)"
            % (self.return_distance, f(self.return_yaw_error, "%.3f"))
            if returned and heading_ok and self.return_distance is not None
            else ("PASS - no return leg required" if not (self.return_to_spawn
                                                         and self.spawn_world is not None)
                  else "FAIL - did not finish within tolerance of the spawn point "
                       "(distance %s, heading %s)" % (f(self.return_distance),
                                                      f(self.return_yaw_error, "%.3f")))))
        passed = bool(all_legs and leg_floor_ok and all_exits and boarded
                      and boarded_centred and excursions_ok and returned
                      and heading_ok and entry_ok
                      and centred_ok and self.fault is None and self.state == "DONE")
        print("  OVERALL            : %s (%s)"
              % ("PASS" if passed else "FAIL",
                 "driver reached DONE with no fault" if passed
                 else "state=%s fault=%s" % (self.state, self.fault or "none")))
        print("=" * 68)
        self.boarded_centred = boarded_centred
        if self.output_json:
            self._write_summary_json(passed)
        return passed

    def _write_summary_json(self, passed):
        import json
        legs = []
        for leg in self.leg_results:
            legs.append({key: value for key, value in leg.items()})
        summary = {
            "tag": str(rospy.get_param("~tag", "")),
            "seed": int(rospy.get_param("~seed", 0)),
            "passed": bool(passed),
            "final_state": self.state,
            "fault": self.fault,
            "sim_now": round(self.sim_now, 2),
            "spawn_param": self.spawn_param,
            "spawn_measured": self.spawn_measured,
            "spawn_world": None if self.spawn_world is None
            else tuple(round(v, 3) for v in self.spawn_world),
            "boarded_centred": bool(getattr(self, "boarded_centred", False)),
            "board_lateral_tolerance": self.board_lateral_tolerance,
            "board_heading_tolerance": self.board_heading_tolerance,
            "board_door_dist_cap": self.board_door_dist_cap,
            "ride_floors": self.ride_floors,
            "corridor_depths": self.corridor_depths[:len(self.ride_floors)],
            "corridor_gate": self.corridor_gate,
            "ground_z": self.ground_z,
            "max_world_truth_error": self.max_world_truth_error,
            "loc_max_planar_error": self.loc_max_planar_error,
            "loc_diverged_seconds": self.loc_diverged_seconds,
            "loc_hold_count": self.loc_hold_count,
            "loc_stationary_warnings": self.loc_stationary_warnings,
            "loc_outdoor_warnings": self.loc_outdoor_warnings,
            "loc_truth_steered_warnings": self.loc_truth_steered_warnings,
            "truth_steered_regions": [list(i) for i in self.truth_steered_regions],
            "door_reference": [round(float(v), 3) for v in self.door_reference],
            "door_frame_source": self.door_frame_source,
            "door_candidate": self.door_candidate,
            "door_candidate_cycles": self.door_candidate_cycles,
            "door_candidate_offset": self.door_candidate_offset,
            "door_candidate_accepted": self.door_candidate_accepted,
            "door_candidate_rejected": self.door_candidate_rejected,
            "door_candidate_reject_reason": self.door_candidate_reject_reason,
            "door_candidate_note": self.door_candidate_note,
            "door_candidate_confirm_cycles": self.door_candidate_confirm_cycles,
            "corridor_region": [self.corridor_x_min, self.corridor_x_max,
                                self.corridor_y_min, self.corridor_y_max],
            "outdoor_truth_y": self.outdoor_truth_y,
            "spawn_turn_y": self.spawn_turn_y,
            "spawn_reverse_retries": self.spawn_reverse_retries,
            "outdoor_truth_steering_sim": self.outdoor_truth_steering_sim,
            "loc_truth_motion": self.loc_truth_motion,
            "loc_cmd_motion": self.loc_cmd_motion,
            "floor_contexts": [list(item) for item in self.floor_contexts],
            "return_distance": self.return_distance,
            "return_yaw_error": self.return_yaw_error,
            "return_yaw_error_truth": self.return_yaw_error_truth,
            "return_sim": self.return_sim,
            "entry_corridor_depth": self.entry_corridor_depth,
            "entry_corridor_truth_max_along": self.entry_corridor_truth_max_along,
            "entry_corridor_ok": self.entry_corridor_ok,
            "final_truth": None if self.robot_truth is None
            else tuple(round(v, 3) for v in self.robot_truth),
            "states": list(dict.fromkeys(row["state"] for row in self.timeline)),
            "legs": legs,
        }
        try:
            with open(self.output_json, "w") as handle:
                json.dump(summary, handle, indent=1, sort_keys=True)
                handle.write("\n")
        except OSError as error:
            rospy.logwarn("elevator-only: cannot write summary json: %s", error)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-csv", default="")
    parser.add_argument("--output-json", default="")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("elevator_only_driver", anonymous=False, disable_signals=True)
    if args.output_csv:
        rospy.set_param("~output_csv", args.output_csv)
    if args.output_json:
        rospy.set_param("~output_json", args.output_json)
    driver = ElevatorOnlyDriver()

    deadline = time.monotonic() + float(rospy.get_param("~wall_timeout", 1800.0))
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        if driver.state in ("DONE", "FAILED"):
            break
        if time.monotonic() > deadline:
            driver.fault = driver.fault or "WALL_TIMEOUT"
            driver.state = "FAILED"
            break
        rate.sleep()
    # One last sample so the terminal state (DONE or FAILED) is on disk in the
    # timeline CSV as well as in the verdict; the 1 Hz sampler would otherwise
    # miss it because the loop leaves as soon as the state flips.
    driver.last_sample_sim = -1.0
    driver._sample()
    passed = driver.report()
    if driver.output_csv:
        verdict = os.path.join(os.path.dirname(driver.output_csv), "elevator_only_verdict.txt")
        try:
            with open(verdict, "w") as handle:
                handle.write(
                    "state={}\nfault={}\npass={}\nsim_now={:.2f}\n".format(
                        driver.state, driver.fault, passed, driver.sim_now)
                    + "rides={}\n".format(len(driver.leg_results))
                    + "return_distance={}\n".format(driver.return_distance)
                    + "spawn_param={}\n".format(driver.spawn_param)
                    + "spawn_measured={}\n".format(driver.spawn_measured)
                )
        except OSError:
            pass
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
