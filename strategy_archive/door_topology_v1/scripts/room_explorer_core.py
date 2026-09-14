#!/usr/bin/env python3
"""Map geometry and deterministic task state for single-floor exploration."""

from dataclasses import dataclass
from collections import deque
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import distance_transform_edt


DISCOVERED = "DISCOVERED"
GO_TO_PRE_DOOR = "GO_TO_PRE_DOOR"
ALIGN_TO_DOOR_NORMAL = "ALIGN_TO_DOOR_NORMAL"
DOOR_CROSSING = "DOOR_CROSSING"
ROOM_SCAN = "ROOM_SCAN"
EXIT_ROOM = "EXIT_ROOM"
VISITED = "VISITED"
UNREACHABLE = "UNREACHABLE"
MISSION_FAULT = "MISSION_FAULT"
RECOVERY = "RECOVERY"
ABORT_RUN = "ABORT_RUN"

GOOD = "GOOD"
DEGRADED = "DEGRADED"
BAD = "BAD"
STALE = "STALE"

SCAN = "SCAN"
MAP = "MAP"
PENDING = "PENDING"
CONFIRMED = "CONFIRMED"
VERIFIED = "VERIFIED"
REJECTED = "REJECTED"
INCONCLUSIVE = "INCONCLUSIVE"


class MissionFaultMonitor:
    """Small deterministic gate for terminating a physically invalid run.

    Hard controller faults are immediate. Localization and lidar faults must
    persist for a bounded simulated interval so one dropped message does not
    abort an otherwise valid exploration.
    """

    def __init__(
        self,
        bad_localization_duration=1.0,
        low_points_duration=1.0,
        minimum_effective_points=5,
    ):
        self.bad_localization_duration = float(bad_localization_duration)
        self.low_points_duration = float(low_points_duration)
        self.minimum_effective_points = int(minimum_effective_points)
        self.bad_localization_since = None
        self.low_points_since = None
        self.active_seen = False

    def reset(self):
        self.bad_localization_since = None
        self.low_points_since = None
        self.active_seen = False

    def evaluate(
        self,
        now,
        active,
        localization_state,
        controller_state="",
        lio_state="",
        effective_points=None,
    ):
        now = float(now)
        if not active:
            self.reset()
            return None
        if controller_state == "FALL":
            return "controller_fall"
        if controller_state == "PASSIVE" and self.active_seen:
            return "controller_passive"
        if controller_state in ("RL", "ACTIVE"):
            self.active_seen = True

        if localization_state == BAD:
            if self.bad_localization_since is None:
                self.bad_localization_since = now
            elif now - self.bad_localization_since >= self.bad_localization_duration:
                return "localization_bad_sustained"
        else:
            self.bad_localization_since = None

        points_low = lio_state in ("NO_EFFECTIVE_POINTS", "STALE")
        if effective_points is not None:
            try:
                points_low = points_low or int(effective_points) < self.minimum_effective_points
            except (TypeError, ValueError):
                points_low = True
        if points_low:
            if self.low_points_since is None:
                self.low_points_since = now
            elif now - self.low_points_since >= self.low_points_duration:
                return "lio_no_effective_points"
        else:
            self.low_points_since = None
        return None


@dataclass(frozen=True)
class GridView:
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str = "simnav_map"

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        column = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return column, row

    def value_at(self, x: float, y: float) -> int:
        column, row = self.world_to_cell(x, y)
        if row < 0 or column < 0 or row >= self.data.shape[0] or column >= self.data.shape[1]:
            return -1
        return int(self.data[row, column])


@dataclass
class DetectedOpening:
    candidate_id: str
    center: Tuple[float, float]
    width: float
    normal_yaw: float
    pre_pose: Tuple[float, float, float]
    post_pose: Tuple[float, float, float]


@dataclass
class OpeningCandidate:
    candidate_id: str
    center: Tuple[float, float]
    width: float
    normal_yaw: float
    pre_pose: Tuple[float, float, float]
    post_pose: Tuple[float, float, float]
    status: str = DISCOVERED
    attempts: int = 0
    exit_attempts: int = 0
    scan_support: bool = False
    map_support: bool = False
    confidence: float = 0.0


@dataclass(frozen=True)
class CorridorEstimate:
    valid: bool
    confidence: float
    axis_yaw: float
    left_wall_distance: float
    right_wall_distance: float
    corridor_width: float
    center_error: float
    front_clearance: float


@dataclass(frozen=True)
class DoorEvidence:
    source: str
    timestamp: float
    side: float
    center: Tuple[float, float]
    width: float
    normal_yaw: float
    pre_pose: Tuple[float, float, float]
    post_pose: Tuple[float, float, float]
    corridor_confidence: float
    source_confidence: float
    opening_complete: bool
    localization_health: str = GOOD
    verification_status: str = INCONCLUSIVE


class DoorVerifier:
    """Short-window local 2.5-D check for a door frame and open passage."""

    def __init__(
        self,
        min_points: int = 16,
        side_tolerance: float = 0.28,
        depth_tolerance: float = 0.35,
        minimum_jamb_height: float = 0.80,
    ):
        self.min_points = int(min_points)
        self.side_tolerance = float(side_tolerance)
        self.depth_tolerance = float(depth_tolerance)
        self.minimum_jamb_height = float(minimum_jamb_height)

    def verify(
        self,
        points_base,
        center_base: Tuple[float, float],
        normal_yaw: float,
        width: float,
    ) -> str:
        """Classify a candidate using vertical jamb support and a free gap.

        The check intentionally has an inconclusive result. Missing or sparse
        point-cloud data must not turn into a hard rejection of a real door.
        """
        points = np.asarray(points_base, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] < 3:
            return INCONCLUSIVE
        points = points[:, :3]
        points = points[np.all(np.isfinite(points), axis=1)]
        if len(points) < self.min_points:
            return INCONCLUSIVE

        normal = np.array([math.cos(normal_yaw), math.sin(normal_yaw)])
        tangent = np.array([-normal[1], normal[0]])
        delta = points[:, :2] - np.asarray(center_base, dtype=np.float64)
        depth = np.dot(delta, normal)
        lateral = np.dot(delta, tangent)
        region = (
            (np.abs(depth) <= self.depth_tolerance)
            & (np.abs(lateral) <= max(1.0, 0.5 * float(width) + 0.5))
            & (points[:, 2] >= 0.05)
            & (points[:, 2] <= 2.5)
        )
        depth = depth[region]
        lateral = lateral[region]
        heights = points[:, 2][region]
        if len(depth) < self.min_points:
            return INCONCLUSIVE

        half_width = max(0.35, 0.5 * float(width))
        jambs = []
        for side in (-1.0, 1.0):
            side_mask = (
                (np.abs(lateral - side * half_width) <= self.side_tolerance)
                & (np.abs(depth) <= self.depth_tolerance)
            )
            side_heights = heights[side_mask]
            jambs.append(
                len(side_heights) >= 4
                and float(np.max(side_heights) - np.min(side_heights))
                >= self.minimum_jamb_height
            )

        gap_mask = (
            (np.abs(lateral) <= max(0.18, half_width - 0.12))
            & (np.abs(depth) <= min(self.depth_tolerance, 0.25))
            & (heights >= 0.15)
            & (heights <= 1.25)
        )
        gap_points = int(np.count_nonzero(gap_mask))
        if all(jambs) and gap_points <= max(8, int(0.30 * len(depth))):
            return VERIFIED
        if gap_points >= max(12, self.min_points // 2):
            return REJECTED
        return INCONCLUSIVE


@dataclass
class DoorHypothesis:
    hypothesis_id: str
    side: float
    center: Tuple[float, float]
    width: float
    normal_yaw: float
    pre_pose: Tuple[float, float, float]
    post_pose: Tuple[float, float, float]
    first_seen: float
    last_seen: float
    scan_support: bool = False
    map_support: bool = False
    scan_confidence: float = 0.0
    map_confidence: float = 0.0
    confidence: float = 0.0
    status: str = PENDING


class CorridorEstimator:
    """Single owner for scan-based corridor geometry used by Stage B."""

    def __init__(
        self,
        min_wall_distance: float = 0.3,
        max_wall_distance: float = 2.0,
        min_width: float = 1.7,
        max_width: float = 2.7,
    ):
        self.min_wall_distance = min_wall_distance
        self.max_wall_distance = max_wall_distance
        self.min_width = min_width
        self.max_width = max_width

    def estimate(
        self,
        axis_yaw: float,
        left_wall_distance: float,
        right_wall_distance: float,
        front_clearance: float,
        center_error: float = 0.0,
    ) -> CorridorEstimate:
        left_valid = math.isfinite(left_wall_distance) and (
            self.min_wall_distance <= left_wall_distance <= self.max_wall_distance
        )
        right_valid = math.isfinite(right_wall_distance) and (
            self.min_wall_distance <= right_wall_distance <= self.max_wall_distance
        )
        width = (
            left_wall_distance + right_wall_distance
            if left_valid and right_valid
            else float("nan")
        )
        width_valid = math.isfinite(width) and self.min_width <= width <= self.max_width
        evidence_count = int(left_valid) + int(right_valid) + int(width_valid)
        confidence = evidence_count / 3.0
        return CorridorEstimate(
            valid=left_valid and right_valid and width_valid,
            confidence=confidence,
            axis_yaw=normalize_angle(axis_yaw),
            left_wall_distance=left_wall_distance,
            right_wall_distance=right_wall_distance,
            corridor_width=width,
            center_error=center_error,
            front_clearance=front_clearance,
        )

    @staticmethod
    def map_valid(
        grid: GridView,
        robot_xy: Tuple[float, float],
        axis_yaw: float,
        length: float = 5.0,
        min_wall_support: float = 0.55,
    ) -> bool:
        return corridor_is_elongated(
            grid,
            robot_xy,
            axis_yaw,
            length=length,
            min_wall_support=min_wall_support,
        )


class LocalizationHealthMonitor:
    """Classify pose continuity without depending on a specific SLAM backend."""

    def __init__(
        self,
        stale_timeout: float = 1.0,
        degraded_translation: float = 0.35,
        bad_translation: float = 1.0,
        degraded_rotation: float = 0.50,
        bad_rotation: float = 1.20,
        degraded_linear_speed: float = 1.2,
        bad_linear_speed: float = 2.0,
        degraded_angular_speed: float = 2.5,
        bad_angular_speed: float = 4.0,
        recovery_samples: int = 10,
    ):
        self.stale_timeout = stale_timeout
        self.degraded_translation = degraded_translation
        self.bad_translation = bad_translation
        self.degraded_rotation = degraded_rotation
        self.bad_rotation = bad_rotation
        self.degraded_linear_speed = degraded_linear_speed
        self.bad_linear_speed = bad_linear_speed
        self.degraded_angular_speed = degraded_angular_speed
        self.bad_angular_speed = bad_angular_speed
        self.recovery_samples = recovery_samples
        self.previous = None
        self.recent_poses = deque()
        self.commanded_linear_speed = 0.0
        self.commanded_angular_speed = 0.0
        self.state = STALE
        self.reason = "no_pose"
        self.stable_samples = 0
        self.max_translation_jump = 0.0
        self.max_rotation_jump = 0.0
        self.state_changes = 0

    def set_command(self, linear_speed: float, angular_speed: float) -> None:
        self.commanded_linear_speed = abs(linear_speed)
        self.commanded_angular_speed = abs(angular_speed)

    def update(self, timestamp: float, x: float, y: float, yaw: float) -> str:
        if self.previous is None:
            self.previous = (timestamp, x, y, yaw)
            self.recent_poses.append((timestamp, x, y))
            self.state = GOOD
            self.reason = "first_pose"
            self.stable_samples = 1
            return self.state

        previous_time, previous_x, previous_y, previous_yaw = self.previous
        self.previous = (timestamp, x, y, yaw)
        delta_time = timestamp - previous_time
        if delta_time <= 1e-4:
            self.state = DEGRADED
            self.reason = "non_monotonic_timestamp"
            self.stable_samples = 0
            return self.state

        translation = math.hypot(x - previous_x, y - previous_y)
        rotation = abs(normalize_angle(yaw - previous_yaw))
        self.max_translation_jump = max(self.max_translation_jump, translation)
        self.max_rotation_jump = max(self.max_rotation_jump, rotation)
        linear_speed = translation / delta_time
        angular_speed = rotation / delta_time
        self.recent_poses.append((timestamp, x, y))
        while self.recent_poses and timestamp - self.recent_poses[0][0] > 1.0:
            self.recent_poses.popleft()
        cumulative_translation = 0.0
        if len(self.recent_poses) >= 2:
            cumulative_translation = math.hypot(
                x - self.recent_poses[0][1], y - self.recent_poses[0][2]
            )
        bad_linear_speed = max(
            self.bad_linear_speed, self.commanded_linear_speed * 2.0 + 0.6
        )
        bad_angular_speed = max(
            self.bad_angular_speed, self.commanded_angular_speed * 2.0 + 0.8
        )
        previous_state = self.state
        bad = (
            translation >= self.bad_translation
            or rotation >= self.bad_rotation
            or linear_speed >= bad_linear_speed
            or angular_speed >= bad_angular_speed
            or cumulative_translation >= max(2.0, self.commanded_linear_speed * 2.0 + 0.8)
        )
        degraded = (
            translation >= self.degraded_translation
            or rotation >= self.degraded_rotation
            or linear_speed >= self.degraded_linear_speed
            or angular_speed >= self.degraded_angular_speed
        )
        if bad:
            self.state = BAD
            self.reason = "nonphysical_pose_delta"
            self.stable_samples = 0
        elif degraded:
            self.state = DEGRADED
            self.reason = "large_pose_delta"
            self.stable_samples = 0
        else:
            self.stable_samples += 1
            if self.state == BAD and self.stable_samples < self.recovery_samples:
                self.reason = "recovering_from_bad"
            elif self.stable_samples < self.recovery_samples and self.state != GOOD:
                self.state = DEGRADED
                self.reason = "recovering"
            else:
                self.state = GOOD
                self.reason = "continuous_pose"
        if self.state != previous_state:
            self.state_changes += 1
        return self.state

    def evaluate(self, now: float) -> str:
        if self.previous is None or now - self.previous[0] > self.stale_timeout:
            if self.state != STALE:
                self.state_changes += 1
            self.state = STALE
            self.reason = "pose_timeout"
            self.stable_samples = 0
        return self.state

    def rebase(self, timestamp: float, x: float, y: float, yaw: float) -> None:
        """Accept a validated loop-closure correction without calling it motion."""
        self.previous = (timestamp, x, y, yaw)
        self.recent_poses.clear()
        self.recent_poses.append((timestamp, x, y))
        self.state = GOOD
        self.reason = "local_loop_closure"
        self.stable_samples = 1

    def snapshot(self) -> dict:
        return {
            "state": self.state,
            "reason": self.reason,
            "stable_samples": self.stable_samples,
            "max_translation_jump": self.max_translation_jump,
            "max_rotation_jump": self.max_rotation_jump,
            "state_changes": self.state_changes,
        }


class DoorFusion:
    """Fuse Scan/Map evidence; detectors never own candidate lifecycle."""

    def __init__(
        self,
        spatial_gate: float = 2.0,
        normal_gate: float = 0.35,
        strong_confidence: float = 0.75,
        medium_confidence: float = 0.45,
        require_scan_verification: bool = False,
    ):
        self.spatial_gate = spatial_gate
        self.normal_gate = normal_gate
        self.strong_confidence = strong_confidence
        self.medium_confidence = medium_confidence
        self.require_scan_verification = bool(require_scan_verification)
        self.hypotheses: Dict[str, DoorHypothesis] = {}
        self.next_id = 0

    def ingest(self, evidence: DoorEvidence) -> List[DoorHypothesis]:
        if (
            not evidence.opening_complete
            or evidence.localization_health == STALE
            or evidence.verification_status == REJECTED
        ):
            return []
        hypothesis = self._match(evidence)
        map_cannot_create = evidence.source == MAP and evidence.localization_health != GOOD
        if hypothesis is None and map_cannot_create:
            return []
        if hypothesis is None:
            hypothesis = self._create(evidence)
        else:
            self._update(hypothesis, evidence)

        if hypothesis.status == CONFIRMED or not self._confirmable(hypothesis, evidence):
            return []
        hypothesis.status = CONFIRMED
        return [hypothesis]

    def ingest_many(self, evidences: Iterable[DoorEvidence]) -> List[DoorHypothesis]:
        confirmed = []
        for evidence in sorted(evidences, key=lambda item: item.timestamp):
            confirmed.extend(self.ingest(evidence))
        return confirmed

    def _match(self, evidence: DoorEvidence) -> Optional[DoorHypothesis]:
        matches = [
            hypothesis
            for hypothesis in self.hypotheses.values()
            if hypothesis.side == evidence.side
            and math.hypot(
                hypothesis.center[0] - evidence.center[0],
                hypothesis.center[1] - evidence.center[1],
            )
            <= self.spatial_gate
            and abs(normalize_angle(hypothesis.normal_yaw - evidence.normal_yaw))
            <= self.normal_gate
        ]
        if not matches:
            return None
        return min(
            matches,
            key=lambda item: math.hypot(
                item.center[0] - evidence.center[0], item.center[1] - evidence.center[1]
            ),
        )

    def _create(self, evidence: DoorEvidence) -> DoorHypothesis:
        hypothesis_id = "door_{:04d}".format(self.next_id)
        self.next_id += 1
        hypothesis = DoorHypothesis(
            hypothesis_id=hypothesis_id,
            side=evidence.side,
            center=evidence.center,
            width=evidence.width,
            normal_yaw=evidence.normal_yaw,
            pre_pose=evidence.pre_pose,
            post_pose=evidence.post_pose,
            first_seen=evidence.timestamp,
            last_seen=evidence.timestamp,
        )
        self.hypotheses[hypothesis_id] = hypothesis
        self._update(hypothesis, evidence)
        return hypothesis

    @staticmethod
    def _update(hypothesis: DoorHypothesis, evidence: DoorEvidence) -> None:
        # Keep the navigation waypoints geometrically tied to the fused door
        # centre.  Previously ``center`` was averaged while ``pre_pose`` and
        # ``post_pose`` were copied from the newest observation.  A candidate
        # could therefore point at one doorway centre but command a waypoint
        # belonging to a different (noisy) centre.
        normal = np.array(
            [math.cos(evidence.normal_yaw), math.sin(evidence.normal_yaw)],
            dtype=np.float64,
        )
        evidence_center = np.asarray(evidence.center, dtype=np.float64)
        pre_distance = max(
            0.10,
            -float(
                np.dot(
                    np.asarray(evidence.pre_pose[:2], dtype=np.float64)
                    - evidence_center,
                    normal,
                )
            ),
        )
        post_distance = max(
            0.10,
            float(
                np.dot(
                    np.asarray(evidence.post_pose[:2], dtype=np.float64)
                    - evidence_center,
                    normal,
                )
            ),
        )
        evidence_confidence = min(evidence.source_confidence, evidence.corridor_confidence)
        previous_confidence = max(hypothesis.confidence, 1e-6)
        weight = evidence_confidence / (previous_confidence + evidence_confidence)
        hypothesis.center = (
            hypothesis.center[0] * (1.0 - weight) + evidence.center[0] * weight,
            hypothesis.center[1] * (1.0 - weight) + evidence.center[1] * weight,
        )
        hypothesis.width = hypothesis.width * (1.0 - weight) + evidence.width * weight
        hypothesis.normal_yaw = evidence.normal_yaw
        fused_center = np.asarray(hypothesis.center, dtype=np.float64)
        pre = fused_center - normal * pre_distance
        post = fused_center + normal * post_distance
        hypothesis.pre_pose = (
            float(pre[0]), float(pre[1]), hypothesis.normal_yaw
        )
        hypothesis.post_pose = (
            float(post[0]), float(post[1]), hypothesis.normal_yaw
        )
        hypothesis.last_seen = max(hypothesis.last_seen, evidence.timestamp)
        if evidence.source == SCAN:
            hypothesis.scan_support = True
            hypothesis.scan_confidence = max(hypothesis.scan_confidence, evidence_confidence)
        elif evidence.source == MAP:
            hypothesis.map_support = True
            hypothesis.map_confidence = max(hypothesis.map_confidence, evidence_confidence)
        hypothesis.confidence = max(
            hypothesis.scan_confidence,
            hypothesis.map_confidence,
            min(1.0, hypothesis.scan_confidence + hypothesis.map_confidence),
        )

    def _confirmable(self, hypothesis: DoorHypothesis, evidence: DoorEvidence) -> bool:
        scan_verified = (
            not self.require_scan_verification
            or evidence.verification_status == VERIFIED
        )
        scan_strong = hypothesis.scan_confidence >= self.strong_confidence and scan_verified
        map_strong = (
            hypothesis.map_confidence >= self.strong_confidence
            and evidence.localization_health == GOOD
        )
        complementary = (
            hypothesis.scan_support
            and hypothesis.map_support
            and hypothesis.scan_confidence >= self.medium_confidence
            and hypothesis.map_confidence >= self.medium_confidence
            and scan_verified
        )
        return scan_strong or map_strong or complementary


class RecoveryManager:
    """Single owner for bounded door, exit, and reverse-sweep budgets."""

    def __init__(self, max_door_attempts: int = 2, max_exit_attempts: int = 2, max_sweeps: int = 1):
        self.max_door_attempts = max_door_attempts
        self.max_exit_attempts = max_exit_attempts
        self.max_sweeps = max_sweeps
        self.door_attempts: Dict[str, int] = {}
        self.exit_attempts: Dict[str, int] = {}
        self.sweep_count = 0

    def begin_door_attempt(self, candidate_id: str) -> bool:
        count = self.door_attempts.get(candidate_id, 0)
        if count >= self.max_door_attempts:
            return False
        self.door_attempts[candidate_id] = count + 1
        return True

    def begin_exit_attempt(self, candidate_id: str) -> bool:
        count = self.exit_attempts.get(candidate_id, 0)
        if count >= self.max_exit_attempts:
            return False
        self.exit_attempts[candidate_id] = count + 1
        return True

    def can_retry_door(self, candidate_id: str) -> bool:
        return self.door_attempts.get(candidate_id, 0) < self.max_door_attempts

    def can_retry_exit(self, candidate_id: str) -> bool:
        return self.exit_attempts.get(candidate_id, 0) < self.max_exit_attempts

    def request_reverse_sweep(self) -> bool:
        if self.sweep_count >= self.max_sweeps:
            return False
        self.sweep_count += 1
        return True


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def new_room_danger_ids(current_ids, approach_baseline_ids):
    """Return tracks first observed after this room approach began."""
    return set(current_ids).difference(approach_baseline_ids)


def projected_travel(
    origin: Tuple[float, float], position: Tuple[float, float], yaw: float
) -> float:
    """Return signed travel from origin along a bounded survey heading."""
    return (
        (position[0] - origin[0]) * math.cos(yaw)
        + (position[1] - origin[1]) * math.sin(yaw)
    )


def point_beyond_topology_gate(
    gate_center: Tuple[float, float],
    position: Tuple[float, float],
    forward_yaw: float,
    minimum_depth: float = 0.0,
) -> bool:
    """Return whether a point belongs to the forward side of a topology gate.

    The entrance/lobby geometry and the main corridor share one occupancy
    grid, so connected free space alone cannot keep their openings in
    separate candidate pools.  A directed virtual gate supplies that missing
    topological boundary without depending on scene-global coordinates.
    """
    return projected_travel(gate_center, position, forward_yaw) >= float(
        minimum_depth
    )


def door_interior_reached(
    door_center: Tuple[float, float],
    position: Tuple[float, float],
    inward_yaw: float,
    minimum_depth: float,
) -> bool:
    """Confirm bounded metric travel from the doorway into a room."""
    return projected_travel(door_center, position, inward_yaw) >= minimum_depth


def door_crossing_errors(
    door_center: Tuple[float, float],
    position: Tuple[float, float],
    inward_yaw: float,
) -> Tuple[float, float]:
    """Return inward depth and signed cross-door lateral error."""
    delta_x = float(position[0]) - float(door_center[0])
    delta_y = float(position[1]) - float(door_center[1])
    cosine = math.cos(float(inward_yaw))
    sine = math.sin(float(inward_yaw))
    return (
        delta_x * cosine + delta_y * sine,
        -delta_x * sine + delta_y * cosine,
    )


def door_crossing_confirmed(
    door_center: Tuple[float, float],
    position: Tuple[float, float],
    inward_yaw: float,
    crossing_origin: Tuple[float, float],
    minimum_depth: float,
    maximum_lateral_error: float,
    minimum_travel: float,
) -> bool:
    """Require centered, measurable motion through the current doorway."""
    depth, lateral_error = door_crossing_errors(
        door_center, position, inward_yaw
    )
    travel = projected_travel(crossing_origin, position, inward_yaw)
    return (
        depth >= float(minimum_depth)
        and abs(lateral_error) <= float(maximum_lateral_error)
        and travel >= float(minimum_travel)
    )


def door_corridor_side_reached(
    door_center: Tuple[float, float],
    position: Tuple[float, float],
    inward_yaw: float,
    minimum_depth: float,
) -> bool:
    """Confirm bounded metric travel from the doorway back into the corridor."""
    return projected_travel(door_center, position, inward_yaw) <= -minimum_depth


def opposite_door_at_station(
    reference_center: Tuple[float, float],
    reference_normal_yaw: float,
    candidate_center: Tuple[float, float],
    candidate_normal_yaw: float,
    corridor_yaw: float,
    station_tolerance: float = 1.5,
) -> bool:
    """Check an observed doorway across the corridor at the same station."""
    reference_side = math.sin(normalize_angle(reference_normal_yaw - corridor_yaw))
    candidate_side = math.sin(normalize_angle(candidate_normal_yaw - corridor_yaw))
    if abs(reference_side) < 1e-6 or abs(candidate_side) < 1e-6:
        return False
    direction = (math.cos(corridor_yaw), math.sin(corridor_yaw))
    station_delta = abs(
        (candidate_center[0] - reference_center[0]) * direction[0]
        + (candidate_center[1] - reference_center[1]) * direction[1]
    )
    return reference_side * candidate_side < 0.0 and station_delta <= abs(
        float(station_tolerance)
    )


def metric_axis_progress(
    metric_origin: Tuple[float, float],
    metric_position: Tuple[float, float],
    source_axis_yaw: float,
    source_pose_yaw: float,
    metric_pose_yaw: float,
) -> float:
    """Measure progress in a metric frame while the map frame drifts."""
    frame_yaw = normalize_angle(metric_pose_yaw - source_pose_yaw)
    metric_axis_yaw = normalize_angle(source_axis_yaw + frame_yaw)
    return projected_travel(metric_origin, metric_position, metric_axis_yaw)


def transform_planar_point(
    point: Tuple[float, float],
    source_pose: Tuple[float, float, float],
    target_pose: Tuple[float, float, float],
) -> Tuple[float, float]:
    """Move a nearby point between two frames tied to the same robot pose."""
    frame_yaw = normalize_angle(target_pose[2] - source_pose[2])
    cosine = math.cos(frame_yaw)
    sine = math.sin(frame_yaw)
    delta_x = float(point[0]) - source_pose[0]
    delta_y = float(point[1]) - source_pose[1]
    return (
        target_pose[0] + cosine * delta_x - sine * delta_y,
        target_pose[1] + sine * delta_x + cosine * delta_y,
    )


def transform_planar_yaw(
    yaw: float,
    source_pose_yaw: float,
    target_pose_yaw: float,
) -> float:
    return normalize_angle(yaw + target_pose_yaw - source_pose_yaw)


def voxel_downsample_2d(points, voxel_size: float = 0.10):
    values = np.asarray(points, dtype=np.float64).reshape((-1, 2))
    if values.size == 0:
        return values
    cells = np.floor(values / voxel_size).astype(np.int64)
    _, indices = np.unique(cells, axis=0, return_index=True)
    return values[np.sort(indices)]


@dataclass(frozen=True)
class RoomFrontier:
    """A reachable approach point for one connected unknown-space frontier."""

    target: Tuple[float, float]
    cluster_size: int
    path_length: float
    information_gain: float
    min_clearance: float
    branch_cells: int
    escape_cells: int
    dead_end_risk: float
    score: float
    path: Tuple[Tuple[float, float], ...] = ()
    visual: bool = False
    ring_progress: float = math.inf


@dataclass(frozen=True)
class RoomFrontierPlan:
    """Topology and frontier snapshot used by the room controller."""

    frontier: Optional[RoomFrontier]
    frontiers: Tuple[RoomFrontier, ...]
    reachable_cells: int
    safe_cells: int
    unknown_frontier_cells: int
    room_free_cells: int
    coverage: float
    reason: str
    # Diagnostic-only ratio before body-clearance filtering.  ``coverage``
    # deliberately uses the safe traversable denominator.
    raw_coverage: float = 0.0
    # Camera coverage is tracked independently from laser/map reachability.
    # It is optional so legacy/offline callers keep the previous semantics.
    visual_coverage: float = 1.0
    visual_unseen_cells: int = 0
    visual_frontier_cells: int = 0
    visual_complete: bool = True
    ring_detected: bool = False
    ring_center: Optional[Tuple[float, float]] = None
    ring_candidate_count: int = 0
    ring_direction: str = "NONE"


class RoomFrontierPlanner:
    """Plan local room coverage from reachable frontiers.

    The planner deliberately knows only the doorway boundary and the local
    occupancy grid.  A connected safe-space component is the room topology;
    corridor and neighbouring-room unknown space is excluded by the inward
    doorway half-plane.  Targets are free approach cells, so the robot never
    has to drive its body into an unknown or occupied cell.
    """

    # Four-connected paths avoid diagonal corner cutting through a wall gap.
    _NEIGHBOURS = (
        (-1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (1, 0, 1.0),
    )
    _CARDINALS = ((-1, 0), (0, -1), (0, 1), (1, 0))

    def __init__(
        self,
        robot_radius: float = 0.38,
        safety_margin: float = 0.04,
        free_threshold: int = 20,
        frontier_cluster_radius: float = 0.45,
        frontier_min_cluster_cells: int = 3,
        target_revisit_radius: float = 0.75,
        topology_entry_margin: float = 0.35,
        topology_max_depth: float = 12.0,
        topology_lateral_limit: float = 8.0,
        dead_end_path_threshold: float = 1.5,
        minimum_escape_cells: int = 12,
        information_gain_weight: float = 1.0,
        dead_end_weight: float = 1.5,
        information_radius: float = 2.5,
        visual_ring_enabled: bool = True,
        visual_ring_clockwise: bool = True,
        visual_ring_max_step: float = 3.0,
        visual_ring_min_obstacle_cells: int = 8,
    ):
        self.robot_radius = max(0.1, float(robot_radius))
        self.safety_margin = max(0.0, float(safety_margin))
        self.free_threshold = int(free_threshold)
        self.frontier_cluster_radius = max(0.1, float(frontier_cluster_radius))
        self.frontier_min_cluster_cells = max(1, int(frontier_min_cluster_cells))
        self.target_revisit_radius = max(0.1, float(target_revisit_radius))
        self.topology_entry_margin = max(0.0, float(topology_entry_margin))
        self.topology_max_depth = max(self.topology_entry_margin, float(topology_max_depth))
        self.topology_lateral_limit = max(0.5, float(topology_lateral_limit))
        self.dead_end_path_threshold = max(0.0, float(dead_end_path_threshold))
        self.minimum_escape_cells = max(1, int(minimum_escape_cells))
        self.information_gain_weight = max(0.0, float(information_gain_weight))
        self.dead_end_weight = max(0.0, float(dead_end_weight))
        self.information_radius = max(0.5, float(information_radius))
        self.visual_ring_enabled = bool(visual_ring_enabled)
        self.visual_ring_clockwise = bool(visual_ring_clockwise)
        self.visual_ring_max_step = max(0.5, float(visual_ring_max_step))
        self.visual_ring_min_obstacle_cells = max(
            1, int(visual_ring_min_obstacle_cells)
        )

    def _central_obstacle_center(
        self,
        grid: GridView,
        data: np.ndarray,
        reachable: np.ndarray,
        room_mask: np.ndarray,
        door_center: Tuple[float, float],
        inward_yaw: float,
    ) -> Optional[Tuple[float, float]]:
        """Detect an occupied island inside the reachable room envelope.

        The room walls sit near the envelope boundary, while the competition
        obstacle sits near its middle.  Evaluate the middle in doorway-local
        coordinates so the test remains valid when the room is rotated.
        """
        rows, columns = np.nonzero(reachable)
        if len(rows) < self.minimum_escape_cells:
            return None
        x = grid.origin_x + (columns.astype(np.float64) + 0.5) * grid.resolution
        y = grid.origin_y + (rows.astype(np.float64) + 0.5) * grid.resolution
        cosine = math.cos(float(inward_yaw))
        sine = math.sin(float(inward_yaw))
        delta_x = x - float(door_center[0])
        delta_y = y - float(door_center[1])
        along = delta_x * cosine + delta_y * sine
        lateral = -delta_x * sine + delta_y * cosine
        along_min, along_max = float(np.min(along)), float(np.max(along))
        lateral_min, lateral_max = float(np.min(lateral)), float(np.max(lateral))
        along_span = along_max - along_min
        lateral_span = lateral_max - lateral_min
        if along_span < 1.5 or lateral_span < 1.5:
            return None
        along_center = 0.5 * (along_min + along_max)
        lateral_center = 0.5 * (lateral_min + lateral_max)

        all_rows, all_columns = np.indices(data.shape, dtype=np.float64)
        all_x = grid.origin_x + (all_columns + 0.5) * grid.resolution
        all_y = grid.origin_y + (all_rows + 0.5) * grid.resolution
        all_delta_x = all_x - float(door_center[0])
        all_delta_y = all_y - float(door_center[1])
        all_along = all_delta_x * cosine + all_delta_y * sine
        all_lateral = -all_delta_x * sine + all_delta_y * cosine
        middle = (
            room_mask
            & (np.abs(all_along - along_center) <= 0.30 * along_span)
            & (np.abs(all_lateral - lateral_center) <= 0.30 * lateral_span)
        )
        central_occupied = middle & (data > self.free_threshold)
        obstacle_rows, obstacle_columns = np.nonzero(central_occupied)
        if len(obstacle_rows) < self.visual_ring_min_obstacle_cells:
            return None
        return (
            float(
                np.mean(
                    grid.origin_x
                    + (obstacle_columns.astype(np.float64) + 0.5)
                    * grid.resolution
                )
            ),
            float(
                np.mean(
                    grid.origin_y
                    + (obstacle_rows.astype(np.float64) + 0.5)
                    * grid.resolution
                )
            ),
        )

    @staticmethod
    def _cell_center(grid: GridView, row: int, column: int) -> Tuple[float, float]:
        return (
            grid.origin_x + (float(column) + 0.5) * grid.resolution,
            grid.origin_y + (float(row) + 0.5) * grid.resolution,
        )

    @staticmethod
    def _in_bounds(shape, row: int, column: int) -> bool:
        return 0 <= row < shape[0] and 0 <= column < shape[1]

    def _room_half_plane(
        self,
        grid: GridView,
        door_center: Tuple[float, float],
        inward_yaw: float,
    ) -> np.ndarray:
        rows, columns = np.indices(grid.data.shape, dtype=np.float64)
        x = grid.origin_x + (columns + 0.5) * grid.resolution
        y = grid.origin_y + (rows + 0.5) * grid.resolution
        inward = np.asarray([math.cos(inward_yaw), math.sin(inward_yaw)])
        delta = np.stack((x - float(door_center[0]), y - float(door_center[1])), axis=0)
        along = np.sum(delta * inward.reshape((2, 1, 1)), axis=0)
        lateral = delta[0] * (-inward[1]) + delta[1] * inward[0]
        return (
            (along >= self.topology_entry_margin)
            & (along <= self.topology_max_depth)
            & (np.abs(lateral) <= self.topology_lateral_limit)
        )

    def _nearest_seed(
        self,
        grid: GridView,
        safe_room: np.ndarray,
        robot_pose: Tuple[float, float, float],
    ) -> Optional[Tuple[int, int]]:
        rows, columns = np.nonzero(safe_room)
        if len(rows) == 0:
            return None
        distances = np.square(
            grid.origin_x + (columns + 0.5) * grid.resolution - float(robot_pose[0])
        ) + np.square(
            grid.origin_y + (rows + 0.5) * grid.resolution - float(robot_pose[1])
        )
        index = int(np.argmin(distances))
        return int(rows[index]), int(columns[index])

    def _reachable_distances(
        self,
        safe_room: np.ndarray,
        seed: Tuple[int, int],
    ) -> Tuple[np.ndarray, np.ndarray]:
        distances = np.full(safe_room.shape, np.inf, dtype=np.float64)
        predecessor = np.full(safe_room.shape + (2,), -1, dtype=np.int32)
        distances[seed] = 0.0
        queue = deque([seed])
        while queue:
            row, column = queue.popleft()
            for d_row, d_column, cost in self._NEIGHBOURS:
                next_row = row + d_row
                next_column = column + d_column
                if not self._in_bounds(safe_room.shape, next_row, next_column):
                    continue
                if not safe_room[next_row, next_column] or math.isfinite(
                    distances[next_row, next_column]
                ):
                    continue
                distances[next_row, next_column] = distances[row, column] + cost
                predecessor[next_row, next_column] = (row, column)
                queue.append((next_row, next_column))
        return distances, predecessor

    def _cluster_frontier_cells(
        self,
        frontier_mask: np.ndarray,
        grid: GridView,
    ) -> List[List[Tuple[int, int]]]:
        """Group nearby frontier cells without joining distant room branches."""
        radius_cells = max(1, int(math.ceil(self.frontier_cluster_radius / grid.resolution)))
        pending = set(zip(*np.nonzero(frontier_mask)))
        clusters = []
        while pending:
            seed = pending.pop()
            cluster = [seed]
            queue = deque([seed])
            while queue:
                row, column = queue.popleft()
                nearby = []
                for other in pending:
                    if max(abs(other[0] - row), abs(other[1] - column)) > radius_cells:
                        continue
                    dx = (other[1] - column) * grid.resolution
                    dy = (other[0] - row) * grid.resolution
                    if math.hypot(dx, dy) <= self.frontier_cluster_radius:
                        nearby.append(other)
                for other in nearby:
                    pending.remove(other)
                    cluster.append(other)
                    queue.append(other)
            clusters.append(cluster)
        return clusters

    def _escape_cells(
        self,
        reachable: np.ndarray,
        approach: Tuple[int, int],
        grid: GridView,
        radius: float = 0.9,
    ) -> int:
        radius_cells = max(1, int(math.ceil(radius / grid.resolution)))
        row, column = approach
        row_min = max(0, row - radius_cells)
        row_max = min(reachable.shape[0], row + radius_cells + 1)
        column_min = max(0, column - radius_cells)
        column_max = min(reachable.shape[1], column + radius_cells + 1)
        count = 0
        for candidate_row in range(row_min, row_max):
            for candidate_column in range(column_min, column_max):
                if not reachable[candidate_row, candidate_column]:
                    continue
                if math.hypot(
                    (candidate_column - column) * grid.resolution,
                    (candidate_row - row) * grid.resolution,
                ) <= radius:
                    count += 1
        return count

    @staticmethod
    def _metric_to_cell(grid: GridView, point: Tuple[float, float]):
        column = int(math.floor((float(point[0]) - grid.origin_x) / grid.resolution))
        row = int(math.floor((float(point[1]) - grid.origin_y) / grid.resolution))
        return row, column

    def path_is_safe(
        self,
        grid: Optional[GridView],
        path: Sequence[Tuple[float, float]],
        minimum_clearance: Optional[float] = None,
    ) -> bool:
        """Validate every cell of a committed path against the latest map.

        A target is kept while the map evolves.  This check is intentionally
        local and conservative: a newly occupied cell or a newly narrowed
        clearance invalidates the remaining path, while harmless unknown-space
        growth does not cause target churn.
        """
        if grid is None or not path:
            return False
        data = np.asarray(grid.data)
        free = (data >= 0) & (data <= self.free_threshold)
        obstacle_free = ((data < 0) | (data <= self.free_threshold)).astype(np.uint8)
        clearance = distance_transform_edt(obstacle_free) * float(grid.resolution)
        clearance_limit = (
            self.robot_radius + self.safety_margin
            if minimum_clearance is None
            else float(minimum_clearance)
        )
        cells = [self._metric_to_cell(grid, point) for point in path]
        for row, column in cells:
            if not self._in_bounds(data.shape, row, column):
                return False
            if not free[row, column] or clearance[row, column] < clearance_limit:
                return False
        for first, second in zip(cells, cells[1:]):
            delta_row = second[0] - first[0]
            delta_column = second[1] - first[1]
            if delta_row != 0 and delta_column != 0:
                return False
            steps = max(abs(delta_row), abs(delta_column), 1)
            for step in range(steps + 1):
                fraction = float(step) / float(steps)
                row = int(round(first[0] + fraction * delta_row))
                column = int(round(first[1] + fraction * delta_column))
                if not self._in_bounds(data.shape, row, column):
                    return False
                if not free[row, column] or clearance[row, column] < clearance_limit:
                    return False
        return True

    def _information_gain(
        self,
        cluster: Sequence[Tuple[int, int]],
        approach: Tuple[int, int],
        unknown: np.ndarray,
        room_mask: np.ndarray,
        grid: GridView,
        visual_unseen: Optional[np.ndarray] = None,
    ) -> float:
        """Estimate newly useful unknown area around a frontier.

        Counting only the boundary length made a large room and a narrow seam
        look almost identical.  A bounded room-ROI disk captures the amount of
        unexplored space likely to be revealed after reaching the frontier,
        without treating the entire unbounded unknown map as gain.
        """
        radius_cells = max(1, int(math.ceil(self.information_radius / grid.resolution)))
        row, column = approach
        row_min = max(0, row - radius_cells)
        row_max = min(unknown.shape[0], row + radius_cells + 1)
        column_min = max(0, column - radius_cells)
        column_max = min(unknown.shape[1], column + radius_cells + 1)
        local_unknown = 0
        gain_mask = unknown
        if visual_unseen is not None:
            gain_mask = unknown | visual_unseen
        radius_squared = self.information_radius * self.information_radius
        for candidate_row in range(row_min, row_max):
            for candidate_column in range(column_min, column_max):
                if not gain_mask[candidate_row, candidate_column] or not room_mask[candidate_row, candidate_column]:
                    continue
                distance_squared = (
                    (candidate_column - column) * grid.resolution
                ) ** 2 + ((candidate_row - row) * grid.resolution) ** 2
                if distance_squared <= radius_squared:
                    local_unknown += 1
        boundary_gain = len(cluster)
        return float(boundary_gain + local_unknown) * float(grid.resolution) ** 2

    def _sample_cardinal_path(
        self,
        path_cells: Sequence[Tuple[int, int]],
        grid: GridView,
    ) -> List[Tuple[int, int]]:
        """Sample long straight runs while retaining every cardinal turn.

        A fixed-stride slice can skip a turn and make the motion controller
        connect two points diagonally through an occupied corner.  Keeping the
        cell immediately before each direction change preserves the topology
        of the four-connected BFS path after metric downsampling.
        """
        if len(path_cells) <= 1:
            return list(path_cells)
        stride = max(1, int(math.ceil(0.50 / grid.resolution)))
        sampled = [path_cells[0]]
        last_added = 0
        previous_direction = None
        for index in range(1, len(path_cells)):
            current = path_cells[index]
            previous = path_cells[index - 1]
            direction = (current[0] - previous[0], current[1] - previous[1])
            if previous_direction is not None and direction != previous_direction:
                corner = path_cells[index - 1]
                if sampled[-1] != corner:
                    sampled.append(corner)
                last_added = index - 1
            if index - last_added >= stride:
                if sampled[-1] != current:
                    sampled.append(current)
                last_added = index
            previous_direction = direction
        if sampled[-1] != path_cells[-1]:
            sampled.append(path_cells[-1])
        return sampled

    def plan(
        self,
        grid: Optional[GridView],
        robot_pose: Optional[Tuple[float, float, float]],
        door_center: Optional[Tuple[float, float]],
        inward_yaw: Optional[float],
        visited_targets: Iterable[Tuple[float, float]] = (),
        visual_seen: Optional[np.ndarray] = None,
        visual_coverage_target: float = 0.70,
    ) -> RoomFrontierPlan:
        if grid is None:
            return RoomFrontierPlan(None, (), 0, 0, 0, 0, 0.0, "NO_GRID")
        if robot_pose is None or door_center is None or inward_yaw is None:
            return RoomFrontierPlan(None, (), 0, 0, 0, 0, 0.0, "NO_ROOM_TOPOLOGY")
        data = np.asarray(grid.data)
        free = (data >= 0) & (data <= self.free_threshold)
        room_mask = self._room_half_plane(grid, door_center, float(inward_yaw))
        room_free = free & room_mask
        if not np.any(room_free):
            return RoomFrontierPlan(None, (), 0, 0, 0, 0, 0.0, "NO_FREE_ROOM")

        # Unknown cells are not traversable, but they are not physical walls.
        # Inflate only observed occupied cells so a legitimate frontier can
        # remain adjacent to unknown space while cracks between wall returns
        # are still removed by body-radius inflation.
        obstacle_free = ((data < 0) | (data <= self.free_threshold)).astype(np.uint8)
        clearance = distance_transform_edt(obstacle_free) * float(grid.resolution)
        minimum_clearance = self.robot_radius + self.safety_margin
        safe_room = room_free & (clearance >= minimum_clearance)
        seed = self._nearest_seed(grid, safe_room, robot_pose)
        if seed is None:
            return RoomFrontierPlan(None, (), 0, 0, 0, int(np.count_nonzero(room_free)), 0.0, "NO_SAFE_ENTRY")

        distances, predecessor = self._reachable_distances(safe_room, seed)
        reachable = np.isfinite(distances)
        ring_center = None
        if self.visual_ring_enabled:
            ring_center = self._central_obstacle_center(
                grid,
                data,
                reachable,
                room_mask,
                door_center,
                float(inward_yaw),
            )
        ring_detected = ring_center is not None
        unknown = data < 0
        visual_mask = None
        visual_unseen = np.zeros(data.shape, dtype=bool)
        visual_coverage = 1.0
        visual_complete = True
        if visual_seen is not None:
            candidate_seen = np.asarray(visual_seen, dtype=bool)
            if candidate_seen.shape != data.shape:
                candidate_seen = np.zeros(data.shape, dtype=bool)
            visual_mask = candidate_seen & safe_room
            safe_cell_count = max(1, int(np.count_nonzero(safe_room)))
            visual_coverage = float(np.count_nonzero(visual_mask)) / float(safe_cell_count)
            visual_unseen = safe_room & ~candidate_seen
            visual_complete = visual_coverage >= max(0.0, min(1.0, float(visual_coverage_target)))
        frontier_mask = np.zeros(data.shape, dtype=bool)
        for row, column in zip(*np.nonzero(reachable)):
            for d_row, d_column in self._CARDINALS:
                next_row = row + d_row
                next_column = column + d_column
                if self._in_bounds(data.shape, next_row, next_column) and unknown[next_row, next_column]:
                    frontier_mask[next_row, next_column] = True
                    break

        # A laser-complete room can still be visually incomplete.  Sample
        # independent camera-unseen viewpoints instead of turning the entire
        # unseen room into one connected frontier.  With one giant cluster a
        # single unsafe approach discarded every remaining viewpoint after a
        # map update.  Spacing the samples also suppresses near-duplicate
        # camera poses and the associated spinning.
        visual_frontier_mask = np.zeros(data.shape, dtype=bool)
        if visual_mask is not None and not visual_complete:
            visual_spacing = max(
                self.target_revisit_radius,
                2.0 * self.frontier_cluster_radius,
                0.75,
            )
            visual_stride = max(1, int(math.ceil(visual_spacing / grid.resolution)))
            bucket_candidates = {}
            for row, column in zip(*np.nonzero(reachable & visual_unseen)):
                bucket = (int(row) // visual_stride, int(column) // visual_stride)
                candidate = (int(row), int(column))
                previous = bucket_candidates.get(bucket)
                if previous is None or clearance[candidate] > clearance[previous]:
                    bucket_candidates[bucket] = candidate
            for row, column in bucket_candidates.values():
                visual_frontier_mask[row, column] = True
        frontier_mask |= visual_frontier_mask

        clusters = self._cluster_frontier_cells(frontier_mask, grid)
        visited = tuple((float(point[0]), float(point[1])) for point in visited_targets)
        frontiers: List[RoomFrontier] = []
        for cluster in clusters:
            visual_cluster = bool(
                visual_mask is not None
                and any(visual_frontier_mask[row, column] for row, column in cluster)
            )
            if len(cluster) < self.frontier_min_cluster_cells and not visual_cluster:
                continue
            approaches = []
            for row, column in zip(*np.nonzero(reachable)):
                if visual_cluster and not visual_unseen[row, column]:
                    continue
                if any(
                    abs(row - frontier_row) <= 1
                    and abs(column - frontier_column) <= 1
                    for frontier_row, frontier_column in cluster
                ):
                    approaches.append((int(row), int(column)))
            if not approaches:
                continue
            approaches.sort(
                key=lambda item: (
                    distances[item],
                    -clearance[item],
                )
            )
            approach = None
            target = None
            fallback = None
            for candidate_approach in approaches:
                candidate_target = self._cell_center(
                    grid, candidate_approach[0], candidate_approach[1]
                )
                if any(
                    math.hypot(candidate_target[0] - point[0], candidate_target[1] - point[1])
                    < self.target_revisit_radius
                    for point in visited
                ):
                    continue
                if (
                    visual_cluster
                    and not visual_complete
                    and distances[candidate_approach] * grid.resolution
                    < max(self.target_revisit_radius, 0.60)
                ):
                    if fallback is None:
                        fallback = (candidate_approach, candidate_target)
                    continue
                approach = candidate_approach
                target = candidate_target
                break
            if approach is None and fallback is not None:
                approach, target = fallback
            if approach is None:
                continue
            branch_cells = sum(
                self._in_bounds(reachable.shape, approach[0] + d_row, approach[1] + d_column)
                and reachable[approach[0] + d_row, approach[1] + d_column]
                for d_row, d_column in self._CARDINALS
            )
            escape_cells = self._escape_cells(reachable, approach, grid)
            path_length = float(distances[approach]) * float(grid.resolution)
            # A long single-branch route ending in a low-escape pocket is
            # commonly a wall crack or an inaccessible recess.  Do not send
            # the dog there merely because its Euclidean distance is small.
            if (
                path_length >= self.dead_end_path_threshold
                and escape_cells < self.minimum_escape_cells
            ):
                continue
            dead_end_risk = 0.0
            if path_length >= self.dead_end_path_threshold and branch_cells <= 1:
                dead_end_risk += 1.0
            if escape_cells < self.minimum_escape_cells:
                dead_end_risk += float(self.minimum_escape_cells - escape_cells) / float(self.minimum_escape_cells)
            # A long single-branch route is not worth entering even when its
            # local escape window happens to contain enough cells.  The dog
            # would otherwise have to reverse through the same narrow neck.
            if dead_end_risk >= 1.0:
                continue
            information_gain = self._information_gain(
                cluster,
                approach,
                unknown,
                room_mask,
                grid,
                visual_unseen if visual_mask is not None else None,
            )
            path_cells = []
            current = approach
            while current != seed and current[0] >= 0:
                path_cells.append(current)
                previous = predecessor[current]
                if previous[0] < 0:
                    break
                current = (int(previous[0]), int(previous[1]))
            path_cells.append(seed)
            path_cells.reverse()
            path_cells = self._sample_cardinal_path(path_cells, grid)
            path = tuple(self._cell_center(grid, row, column) for row, column in path_cells)
            # Use the narrowest point of the complete route for speed and
            # waypoint tolerances; the approach cell alone may be wider than
            # a wall-adjacent turn along the way.
            min_clearance = min(
                float(clearance[row, column]) for row, column in path_cells
            )
            # Safety is enforced above.  Among safe candidates, information
            # gain is the primary objective; path length is only a late
            # tie-breaker so a nearby but unproductive frontier cannot starve
            # a larger unexplored branch.
            score = (
                self.dead_end_weight * dead_end_risk
                - self.information_gain_weight * information_gain
                + 0.01 * path_length
            )
            ring_progress = math.inf
            if visual_cluster and ring_center is not None:
                robot_angle = math.atan2(
                    float(robot_pose[1]) - ring_center[1],
                    float(robot_pose[0]) - ring_center[0],
                )
                target_angle = math.atan2(
                    float(target[1]) - ring_center[1],
                    float(target[0]) - ring_center[0],
                )
                if self.visual_ring_clockwise:
                    ring_progress = (robot_angle - target_angle) % (2.0 * math.pi)
                else:
                    ring_progress = (target_angle - robot_angle) % (2.0 * math.pi)
                # Keep successive ring goals local.  A target on the far side
                # remains available after the nearer sector is traversed.
                if path_length > self.visual_ring_max_step:
                    continue
            frontiers.append(
                RoomFrontier(
                    target=target,
                    cluster_size=len(cluster),
                    path_length=path_length,
                    information_gain=information_gain,
                    min_clearance=min_clearance,
                    branch_cells=int(branch_cells),
                    escape_cells=int(escape_cells),
                    dead_end_risk=dead_end_risk,
                    score=score,
                    path=path,
                    visual=visual_cluster,
                    ring_progress=ring_progress,
                )
            )
        if ring_detected and any(item.visual for item in frontiers):
            frontiers.sort(
                key=lambda item: (
                    not item.visual,
                    item.ring_progress,
                    item.path_length,
                    -item.information_gain,
                )
            )
        else:
            frontiers.sort(
                key=lambda item: (
                    -item.information_gain,
                    item.dead_end_risk,
                    item.path_length,
                )
            )
        room_free_cells = int(np.count_nonzero(room_free))
        reachable_cells = int(np.count_nonzero(reachable))
        safe_cells = int(np.count_nonzero(safe_room))
        frontier_cells = int(np.count_nonzero(frontier_mask))
        visual_unseen_cells = int(np.count_nonzero(visual_unseen)) if visual_mask is not None else 0
        visual_frontier_cells = int(np.count_nonzero(visual_frontier_mask))
        ring_candidate_count = sum(
            item.visual and math.isfinite(item.ring_progress) for item in frontiers
        )
        safe_coverage = float(reachable_cells) / float(max(1, safe_cells))
        raw_coverage = float(reachable_cells) / float(max(1, room_free_cells))
        reason = "FRONTIER" if frontiers else "NO_FRONTIER"
        return RoomFrontierPlan(
            frontier=frontiers[0] if frontiers else None,
            frontiers=tuple(frontiers),
            reachable_cells=reachable_cells,
            safe_cells=safe_cells,
            unknown_frontier_cells=frontier_cells,
            room_free_cells=room_free_cells,
            coverage=safe_coverage,
            reason=reason,
            raw_coverage=raw_coverage,
            visual_coverage=visual_coverage,
            visual_unseen_cells=visual_unseen_cells,
            visual_frontier_cells=visual_frontier_cells,
            visual_complete=visual_complete,
            ring_detected=ring_detected,
            ring_center=ring_center,
            ring_candidate_count=int(ring_candidate_count),
            ring_direction=(
                "CLOCKWISE"
                if ring_detected and self.visual_ring_clockwise
                else "COUNTERCLOCKWISE"
                if ring_detected
                else "NONE"
            ),
        )


def local_loop_icp(
    reference_points,
    current_points,
    voxel_size: float = 0.10,
    max_correspondence: float = 0.60,
    max_iterations: int = 15,
    minimum_inliers: int = 40,
):
    """Estimate a bounded SE(2) correction from a repeated local viewpoint."""
    reference = voxel_downsample_2d(reference_points, voxel_size)
    current = voxel_downsample_2d(current_points, voxel_size)
    if len(reference) < minimum_inliers or len(current) < minimum_inliers:
        return None
    tree = cKDTree(reference)
    transform = np.identity(3)
    previous_rmse = float("inf")
    for _ in range(max_iterations):
        homogeneous = np.column_stack((current, np.ones(len(current))))
        moved = np.matmul(transform, homogeneous.T).T[:, :2]
        distances, indices = tree.query(moved, k=1)
        mask = distances <= max_correspondence
        if int(np.count_nonzero(mask)) < minimum_inliers:
            return None
        source = moved[mask]
        target = reference[indices[mask]]
        source_center = np.mean(source, axis=0)
        target_center = np.mean(target, axis=0)
        covariance = np.matmul(
            (source - source_center).T, target - target_center
        )
        left, _, right_transpose = np.linalg.svd(covariance)
        rotation = np.matmul(right_transpose.T, left.T)
        if np.linalg.det(rotation) < 0.0:
            right_transpose[-1, :] *= -1.0
            rotation = np.matmul(right_transpose.T, left.T)
        translation = target_center - np.matmul(rotation, source_center)
        increment = np.identity(3)
        increment[:2, :2] = rotation
        increment[:2, 2] = translation
        transform = np.matmul(increment, transform)
        rmse = float(np.sqrt(np.mean(np.square(distances[mask]))))
        if abs(previous_rmse - rmse) < 1e-4:
            break
        previous_rmse = rmse
    moved = np.matmul(
        transform, np.column_stack((current, np.ones(len(current)))).T
    ).T[:, :2]
    distances, _ = tree.query(moved, k=1)
    mask = distances <= max_correspondence
    inliers = int(np.count_nonzero(mask))
    if inliers < minimum_inliers:
        return None
    return {
        "transform": transform,
        "translation": float(np.linalg.norm(transform[:2, 2])),
        "rotation": abs(math.atan2(transform[1, 0], transform[0, 0])),
        "rmse": float(np.sqrt(np.mean(np.square(distances[mask])))),
        "overlap": float(inliers) / float(len(current)),
        "inliers": inliers,
        "reference_points": int(len(reference)),
        "current_points": int(len(current)),
    }


def corridor_sweep_needed(
    statuses: Iterable[str],
    expected_room_count: int,
    sweep_count: int,
    max_sweeps: int,
) -> bool:
    """Return whether another bounded corridor pass is needed."""
    visited_count = sum(1 for status in statuses if status == VISITED)
    return visited_count < expected_room_count and sweep_count < max_sweeps


def door_discovery_allowed(active_candidate_id: Optional[str], motion_state: Optional[str]) -> bool:
    """Discover rooms only while the robot is autonomously progressing a corridor."""
    return active_candidate_id is None and motion_state == "CORRIDOR_PROGRESSION"


def corridor_timeout_requires_reversal(
    front_clearance: float,
    stop_distance: float,
    completed_stall_windows: int,
    max_stall_windows: int,
) -> bool:
    """Use observed clearance, not scan-matching distance alone, to end a sweep."""
    return (
        front_clearance < stop_distance
        or completed_stall_windows >= max_stall_windows
    )


def finite_percentile_clearance(
    ranges: Iterable[float], percentile: float = 20.0
) -> float:
    """Estimate obstacle clearance without treating missing returns as free samples."""
    values = np.asarray(list(ranges), dtype=np.float64)
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return float("inf")
    return float(np.percentile(finite, percentile))


def finite_median_clearance(ranges: Iterable[float]) -> float:
    """Measure a sparse side sector without interpreting missing rays as open space."""
    values = np.asarray(list(ranges), dtype=np.float64)
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return float("nan")
    return float(np.median(finite))


def corridor_approach_motion(
    along_error: float,
    corridor_yaw: float,
    speed: float,
    tolerance: float = 0.45,
) -> Tuple[float, float]:
    """Face and walk forward toward a detected corridor feature."""
    if abs(along_error) < tolerance:
        return corridor_yaw, 0.0
    target_yaw = corridor_yaw
    if along_error < 0.0:
        target_yaw = normalize_angle(corridor_yaw + math.pi)
    return target_yaw, speed


def corridor_steering_correction(
    lateral_error: float,
    left_clearance: float,
    right_clearance: float,
    narrow_left_clearance: float,
    narrow_right_clearance: float,
    scan_center_gain: float = 0.12,
    wall_avoid_distance: float = 0.65,
) -> Tuple[float, bool]:
    """Return a bounded yaw offset and whether close-wall avoidance is active."""
    correction = -0.25 * lateral_error
    if 0.2 <= left_clearance <= 2.0 and 0.2 <= right_clearance <= 2.0:
        scan_correction = scan_center_gain * (left_clearance - right_clearance)
        correction += max(-0.15, min(0.15, scan_correction))

    close_wall = False
    if 0.05 <= narrow_left_clearance < wall_avoid_distance:
        correction -= min(0.30, 0.60 * (wall_avoid_distance - narrow_left_clearance))
        close_wall = True
    if 0.05 <= narrow_right_clearance < wall_avoid_distance:
        correction += min(0.30, 0.60 * (wall_avoid_distance - narrow_right_clearance))
        close_wall = True
    limit = 0.35 if close_wall else 0.20
    return max(-limit, min(limit, correction)), close_wall


def corridor_side_walls_present(
    left_nearest: float,
    right_nearest: float,
    min_distance: float = 0.3,
    max_distance: float = 2.0,
    min_corridor_width: float = 1.7,
    max_corridor_width: float = 2.7,
) -> bool:
    """Confirm parallel side-wall evidence from sparse finite lidar returns."""
    observed_width = left_nearest + right_nearest
    return (
        min_distance <= left_nearest <= max_distance
        and min_distance <= right_nearest <= max_distance
        and min_corridor_width <= observed_width <= max_corridor_width
    )


def select_observed_corridor_turn(
    corridor_yaw: float,
    left_clearance: float,
    right_clearance: float,
    turn_clearance: float,
    left_is_corridor: bool,
    right_is_corridor: bool,
) -> Optional[float]:
    """Select a perpendicular branch only when map geometry confirms a corridor."""
    options = []
    if left_clearance >= turn_clearance and left_is_corridor:
        options.append((left_clearance, 1.0))
    if right_clearance >= turn_clearance and right_is_corridor:
        options.append((right_clearance, -1.0))
    if not options:
        return None
    _clearance, turn_sign = max(options)
    return normalize_angle(corridor_yaw + turn_sign * math.pi / 2.0)


def closed_opening_measurement(
    opening_start: float,
    opening_end: float,
    min_width: float,
    max_width: float,
    leading_wall_start: Optional[float] = None,
    min_leading_wall_length: float = 0.0,
) -> Optional[Tuple[float, float]]:
    """Measure a bounded opening after both wall edges are observed."""
    if min_leading_wall_length > 0.0:
        if leading_wall_start is None:
            return None
        leading_wall_length = abs(float(opening_start) - float(leading_wall_start))
        if leading_wall_length < min_leading_wall_length:
            return None
    width = abs(opening_end - opening_start)
    if not min_width <= width <= max_width:
        return None
    return 0.5 * (opening_start + opening_end), width


def corridor_is_elongated(
    grid: GridView,
    robot_xy: Tuple[float, float],
    corridor_yaw: float,
    length: float = 5.0,
    min_wall_support: float = 0.55,
    wall_min_distance: float = 0.6,
    wall_max_distance: float = 2.2,
    max_wall_deviation: float = 0.25,
) -> bool:
    """Require stable parallel walls before interpreting bounded gaps as room doors."""
    direction = np.array([math.cos(corridor_yaw), math.sin(corridor_yaw)])
    normal = np.array([-direction[1], direction[0]])
    along_samples = np.arange(-1.0, max(-0.8, length - 1.0) + 0.001, 0.20)
    lateral_samples = np.arange(
        wall_min_distance, wall_max_distance + grid.resolution, grid.resolution
    )
    profiles = []
    origin = np.asarray(robot_xy)
    for side in (-1.0, 1.0):
        hits = []
        for along in along_samples:
            center = origin + direction * along
            hit = next(
                (
                    float(distance)
                    for distance in lateral_samples
                    if grid.value_at(
                        float(center[0] + normal[0] * side * distance),
                        float(center[1] + normal[1] * side * distance),
                    )
                    >= 50
                ),
                None,
            )
            if hit is not None:
                hits.append(hit)
        support = len(hits) / float(len(along_samples))
        if support < min_wall_support:
            return False
        median = float(np.median(hits))
        deviation = float(np.median(np.abs(np.asarray(hits) - median)))
        profiles.append((median, deviation))
    return all(deviation <= max_wall_deviation for _median, deviation in profiles)


def corridor_entry_is_wide(
    grid: GridView,
    robot_xy: Tuple[float, float],
    corridor_yaw: float,
    min_width: float = 1.7,
    probe_length: float = 2.0,
    max_half_width: float = 2.0,
) -> bool:
    """Reject elongated rooms whose entrance is a door-width bottleneck."""
    direction = np.array([math.cos(corridor_yaw), math.sin(corridor_yaw)])
    normal = np.array([-direction[1], direction[0]])
    origin = np.asarray(robot_xy)
    along_samples = np.arange(0.2, probe_length + 0.001, max(0.1, grid.resolution))
    lateral_samples = np.arange(0.3, max_half_width + grid.resolution, grid.resolution)
    observed_widths = []
    for along in along_samples:
        center = origin + direction * along
        side_hits = []
        for side in (-1.0, 1.0):
            hit = next(
                (
                    float(distance)
                    for distance in lateral_samples
                    if grid.value_at(
                        float(center[0] + normal[0] * side * distance),
                        float(center[1] + normal[1] * side * distance),
                    )
                    >= 50
                ),
                None,
            )
            side_hits.append(hit)
        if all(hit is not None for hit in side_hits):
            observed_widths.append(side_hits[0] + side_hits[1])
    return bool(observed_widths) and min(observed_widths) >= min_width


class OpeningDetector:
    """Detect bounded gaps in the two dominant walls of a mapped corridor."""

    def __init__(
        self,
        min_width: float = 0.8,
        max_width: float = 1.6,
        wall_min_distance: float = 0.7,
        wall_max_distance: float = 2.0,
        wall_tolerance: float = 0.14,
        pre_distance: float = 0.65,
        post_distance: float = 1.0,
        observation_range: Optional[float] = None,
        min_jamb_wall_length: float = 0.65,
        min_jamb_wall_support: float = 0.55,
    ):
        self.min_width = min_width
        self.max_width = max_width
        self.wall_min_distance = wall_min_distance
        self.wall_max_distance = wall_max_distance
        self.wall_tolerance = wall_tolerance
        self.pre_distance = pre_distance
        self.post_distance = post_distance
        self.observation_range = observation_range
        self.min_jamb_wall_length = max(0.0, float(min_jamb_wall_length))
        self.min_jamb_wall_support = max(
            0.0, min(1.0, float(min_jamb_wall_support))
        )

    def detect(
        self,
        grid: GridView,
        robot_xy: Tuple[float, float],
        corridor_yaw: float,
        defer_polygons: Sequence[Sequence[Tuple[float, float]]] = (),
    ) -> List[DetectedOpening]:
        occupied_rows, occupied_columns = np.nonzero(grid.data >= 50)
        if len(occupied_rows) < 20:
            return []
        occupied_x = grid.origin_x + (occupied_columns + 0.5) * grid.resolution
        occupied_y = grid.origin_y + (occupied_rows + 0.5) * grid.resolution
        direction = np.array([math.cos(corridor_yaw), math.sin(corridor_yaw)])
        normal = np.array([-direction[1], direction[0]])
        relative = np.column_stack((occupied_x - robot_xy[0], occupied_y - robot_xy[1]))
        along = relative.dot(direction)
        lateral = relative.dot(normal)
        if self.observation_range is None:
            local_mask = np.ones_like(along, dtype=bool)
        else:
            local_mask = np.abs(along) <= self.observation_range
        candidates = []
        for side in (-1.0, 1.0):
            side_mask = (
                local_mask
                & (side * lateral >= self.wall_min_distance)
                & (side * lateral <= self.wall_max_distance)
            )
            if np.count_nonzero(side_mask) < 10:
                continue
            wall_lateral = self._dominant_wall_lateral(lateral[side_mask], grid.resolution)
            wall_mask = local_mask & (np.abs(lateral - wall_lateral) <= self.wall_tolerance)
            side_along = np.sort(along[wall_mask])
            if len(side_along) < 8:
                continue
            for gap_start, gap_end in self._bounded_gaps(side_along, grid.resolution):
                width = gap_end - gap_start
                if not self.min_width <= width <= self.max_width:
                    continue
                # A door must be a gap in one continuous corridor wall.  A
                # single occupied bin at either edge also occurs around short
                # wall fragments at lobby/corridor junctions, so require both
                # jambs to retain a useful length of the same dominant wall.
                if not self._has_bilateral_wall_support(
                    side_along, gap_start, gap_end, grid.resolution
                ):
                    continue
                center_along = (gap_start + gap_end) / 2.0
                center = np.asarray(robot_xy) + direction * center_along + normal * wall_lateral
                side_normal = normal * side
                pre = center - side_normal * self.pre_distance
                post = center + side_normal * self.post_distance
                if not self._crossing_is_observed_free(grid, pre, post):
                    continue
                center_tuple = (float(center[0]), float(center[1]))
                if any(point_in_polygon(center_tuple, polygon) for polygon in defer_polygons):
                    continue
                yaw = math.atan2(side_normal[1], side_normal[0])
                candidate_id = "opening_{:+07d}_{:+07d}".format(
                    int(round(center[0] * 20.0)), int(round(center[1] * 20.0))
                )
                candidates.append(
                    DetectedOpening(
                        candidate_id=candidate_id,
                        center=center_tuple,
                        width=width,
                        normal_yaw=yaw,
                        pre_pose=(float(pre[0]), float(pre[1]), yaw),
                        post_pose=(float(post[0]), float(post[1]), yaw),
                    )
                )
        candidates.sort(key=lambda item: math.hypot(item.pre_pose[0] - robot_xy[0], item.pre_pose[1] - robot_xy[1]))
        return self._merge_nearby_fragments(candidates)

    @staticmethod
    def _merge_nearby_fragments(
        candidates: Sequence[DetectedOpening],
    ) -> List[DetectedOpening]:
        merged = []
        for candidate in candidates:
            match_index = next(
                (
                    index
                    for index, existing in enumerate(merged)
                    if math.hypot(
                        candidate.center[0] - existing.center[0],
                        candidate.center[1] - existing.center[1],
                    ) < 2.0
                    and abs(normalize_angle(candidate.normal_yaw - existing.normal_yaw)) < 0.25
                ),
                None,
            )
            if match_index is None:
                merged.append(candidate)
            elif abs(candidate.width - 1.2) < abs(merged[match_index].width - 1.2):
                merged[match_index] = candidate
        return merged

    @staticmethod
    def _dominant_wall_lateral(values: np.ndarray, resolution: float) -> float:
        lower = float(np.min(values))
        bins = np.floor((values - lower) / resolution).astype(np.int32)
        counts = np.bincount(bins)
        return lower + (int(np.argmax(counts)) + 0.5) * resolution

    def _bounded_gaps(self, along: np.ndarray, resolution: float) -> Iterable[Tuple[float, float]]:
        bins = np.unique(np.round(along / resolution).astype(np.int32))
        if len(bins) < 2:
            return []
        gaps = []
        for left, right in zip(bins[:-1], bins[1:]):
            empty_width = (right - left - 1) * resolution
            if empty_width >= self.min_width - 2.0 * resolution:
                gaps.append(((left + 0.5) * resolution, (right - 0.5) * resolution))
        return gaps

    def _has_bilateral_wall_support(
        self,
        along: np.ndarray,
        gap_start: float,
        gap_end: float,
        resolution: float,
    ) -> bool:
        """Require continuous parent-wall evidence before and after a gap."""
        if self.min_jamb_wall_length <= 0.0:
            return True
        occupied_bins = set(np.round(along / resolution).astype(np.int32).tolist())
        left_jamb = int(round(gap_start / resolution - 0.5))
        right_jamb = int(round(gap_end / resolution + 0.5))
        support_bins = max(
            2, int(math.ceil(self.min_jamb_wall_length / resolution))
        )
        expected = (
            range(left_jamb - support_bins + 1, left_jamb + 1),
            range(right_jamb, right_jamb + support_bins),
        )
        for wall_bins in expected:
            support = sum(bin_index in occupied_bins for bin_index in wall_bins)
            if support / float(support_bins) < self.min_jamb_wall_support:
                return False
        return True

    @staticmethod
    def _crossing_is_observed_free(grid: GridView, pre: np.ndarray, post: np.ndarray) -> bool:
        for ratio in np.linspace(0.0, 1.0, 9):
            point = pre + ratio * (post - pre)
            if grid.value_at(float(point[0]), float(point[1])) != 0:
                return False
        return True


def point_in_polygon(point: Tuple[float, float], polygon: Sequence[Tuple[float, float]]) -> bool:
    if len(polygon) < 3:
        return False
    x, y = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / float(y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
        previous = current
    return inside


class DoorCandidateManager:
    """Own confirmed room candidates, task state, and bounded recovery budgets."""

    _SUCCESS_TRANSITIONS = {
        GO_TO_PRE_DOOR: ALIGN_TO_DOOR_NORMAL,
        ALIGN_TO_DOOR_NORMAL: DOOR_CROSSING,
        DOOR_CROSSING: ROOM_SCAN,
        ROOM_SCAN: EXIT_ROOM,
        EXIT_ROOM: VISITED,
    }

    def __init__(
        self,
        max_attempts: int = 2,
        max_exit_attempts: int = 2,
        max_sweeps: int = 1,
        quiet_period: float = 3.0,
        dedup_radius: float = 2.0,
        visited_revisit_radius: float = 3.0,
        visited_revisit_normal_gate: float = 0.70,
        opposite_pair_station_tolerance: float = 1.5,
    ):
        self.quiet_period = quiet_period
        self.dedup_radius = dedup_radius
        self.visited_revisit_radius = float(visited_revisit_radius)
        self.visited_revisit_normal_gate = float(visited_revisit_normal_gate)
        self.revisit_rejections = 0
        self.opposite_pair_selections = 0
        self.exit_opposite_candidate_id = None
        self.last_selection_reason = None
        self.candidates = {}
        self.active_id = None
        self.last_discovery_time = 0.0
        self.completion_ready_time = None
        self.recovery = RecoveryManager(max_attempts, max_exit_attempts, max_sweeps)

    def merge_hypotheses(
        self, hypotheses: Iterable[DoorHypothesis], sim_time: float
    ) -> int:
        candidates = [
            OpeningCandidate(
                candidate_id=hypothesis.hypothesis_id,
                center=hypothesis.center,
                width=hypothesis.width,
                normal_yaw=hypothesis.normal_yaw,
                pre_pose=hypothesis.pre_pose,
                post_pose=hypothesis.post_pose,
                scan_support=hypothesis.scan_support,
                map_support=hypothesis.map_support,
                confidence=hypothesis.confidence,
            )
            for hypothesis in hypotheses
        ]
        return self._merge_owned_candidates(candidates, sim_time)

    def merge_candidates(self, detections: Iterable[DetectedOpening], sim_time: float) -> int:
        """Compatibility path for geometry tests; lifecycle objects are still made here."""
        candidates = [
            OpeningCandidate(
                candidate_id=detection.candidate_id,
                center=detection.center,
                width=detection.width,
                normal_yaw=detection.normal_yaw,
                pre_pose=detection.pre_pose,
                post_pose=detection.post_pose,
                scan_support=getattr(detection, "scan_support", False),
                map_support=getattr(detection, "map_support", False),
                confidence=getattr(detection, "confidence", 0.0),
            )
            for detection in detections
        ]
        return self._merge_owned_candidates(candidates, sim_time)

    def _merge_owned_candidates(
        self, candidates: Iterable[OpeningCandidate], sim_time: float
    ) -> int:
        new_count = 0
        for candidate in candidates:
            if candidate.candidate_id in self.candidates:
                self._refresh_candidate_geometry(
                    self.candidates[candidate.candidate_id], candidate
                )
                continue
            visited_revisit = any(
                existing.status == VISITED
                and math.hypot(
                    candidate.center[0] - existing.center[0],
                    candidate.center[1] - existing.center[1],
                ) <= self.visited_revisit_radius
                and abs(normalize_angle(candidate.normal_yaw - existing.normal_yaw))
                < self.visited_revisit_normal_gate
                for existing in self.candidates.values()
            )
            if visited_revisit:
                # A localization shift can make the same visited doorway get
                # a new detector ID. Keep it out of the pending queue so the
                # robot continues toward a genuinely new opening.
                self.revisit_rejections += 1
                continue
            duplicate = next(
                (
                    existing
                    for existing in self.candidates.values()
                    if math.hypot(
                        candidate.center[0] - existing.center[0],
                        candidate.center[1] - existing.center[1],
                    ) <= self.dedup_radius
                    and abs(
                        normalize_angle(candidate.normal_yaw - existing.normal_yaw)
                    )
                    < 0.35
                ),
                None,
            )
            if duplicate is not None:
                self._refresh_candidate_geometry(duplicate, candidate)
                continue
            self.candidates[candidate.candidate_id] = candidate
            new_count += 1
        if new_count:
            self.last_discovery_time = sim_time
        return new_count

    @staticmethod
    def _refresh_candidate_geometry(existing, observed) -> None:
        """Refresh an unvisited door from later, stronger observations.

        Candidate ownership remains stable, but its metric waypoint must not
        freeze at the first partially built map frame.  Do not move a visited
        or currently traversed door because those coordinates also anchor the
        room topology and exit operation.
        """
        if existing.status not in (DISCOVERED, GO_TO_PRE_DOOR):
            return
        stronger = (
            observed.confidence + 1e-6 >= existing.confidence
            or (observed.map_support and not existing.map_support)
        )
        if not stronger:
            return
        existing.center = observed.center
        existing.width = observed.width
        existing.normal_yaw = observed.normal_yaw
        existing.pre_pose = observed.pre_pose
        existing.post_pose = observed.post_pose
        existing.scan_support = existing.scan_support or observed.scan_support
        existing.map_support = existing.map_support or observed.map_support
        existing.confidence = max(existing.confidence, observed.confidence)

    def next_candidate(
        self,
        robot_xy: Tuple[float, float],
        expected_room_count: Optional[int] = None,
        corridor_yaw: Optional[float] = None,
        prefer_opposite: bool = True,
        opposite_pair_station_tolerance: Optional[float] = None,
        prefer_paired_station: bool = True,
    ) -> Optional[OpeningCandidate]:
        if self.active_id is not None:
            return self.candidates[self.active_id]
        if expected_room_count is not None:
            visited_count = sum(
                item.status == VISITED for item in self.candidates.values()
            )
            if visited_count >= expected_room_count:
                return None
        pending = [item for item in self.candidates.values() if item.status == DISCOVERED]
        if not pending:
            return None
        exit_opposite_candidate_id = self.exit_opposite_candidate_id
        self.exit_opposite_candidate_id = None
        if prefer_opposite and exit_opposite_candidate_id is not None:
            preferred = next(
                (
                    item
                    for item in pending
                    if item.candidate_id == exit_opposite_candidate_id
                ),
                None,
            )
            if preferred is not None:
                pending = [preferred]
                self.opposite_pair_selections += 1
                self.last_selection_reason = "EXIT_OPPOSITE"
        if self.last_selection_reason != "EXIT_OPPOSITE":
            self.last_selection_reason = "NEAREST_DISCOVERED"
        if (
            prefer_paired_station
            and corridor_yaw is not None
            and opposite_pair_station_tolerance is not None
        ):
            # Prefer a station with evidence on both corridor sides.  Map
            # fragments can be confirmed as isolated openings; keeping them
            # as a fallback prevents one fragment from consuming the finite
            # room quota when a paired, physically plausible station is
            # already available.
            paired = [
                item
                for item in pending
                if any(
                    other.candidate_id != item.candidate_id
                    and other.status in (DISCOVERED, VISITED)
                    and opposite_door_at_station(
                        item.center,
                        item.normal_yaw,
                        other.center,
                        other.normal_yaw,
                        corridor_yaw,
                        opposite_pair_station_tolerance,
                    )
                    for other in self.candidates.values()
                )
            ]
            if paired:
                pending = paired
        pending.sort(
            key=lambda item: math.hypot(item.pre_pose[0] - robot_xy[0], item.pre_pose[1] - robot_xy[1])
        )
        active = pending[0]
        if not self.recovery.begin_door_attempt(active.candidate_id):
            active.status = UNREACHABLE
            return self.next_candidate(
                robot_xy,
                expected_room_count,
                corridor_yaw=corridor_yaw,
                prefer_opposite=prefer_opposite,
                opposite_pair_station_tolerance=opposite_pair_station_tolerance,
                prefer_paired_station=prefer_paired_station,
            )
        active.status = GO_TO_PRE_DOOR
        active.attempts = self.recovery.door_attempts[active.candidate_id]
        self.active_id = active.candidate_id
        return active

    def advance(self, succeeded: bool, sim_time: Optional[float] = None) -> Optional[str]:
        if self.active_id is None:
            return None
        candidate = self.candidates[self.active_id]
        if succeeded:
            candidate.status = self._SUCCESS_TRANSITIONS[candidate.status]
            if candidate.status == EXIT_ROOM:
                if not self.recovery.begin_exit_attempt(candidate.candidate_id):
                    candidate.status = UNREACHABLE
                    self.active_id = None
                    return candidate.status
                candidate.exit_attempts = self.recovery.exit_attempts[candidate.candidate_id]
            if candidate.status == VISITED:
                if sim_time is not None:
                    self.completion_ready_time = sim_time
                elif self.completion_ready_time is None:
                    self.completion_ready_time = self.last_discovery_time
                self.active_id = None
            return candidate.status
        if candidate.status == EXIT_ROOM and self.recovery.can_retry_exit(candidate.candidate_id):
            self.recovery.begin_exit_attempt(candidate.candidate_id)
            candidate.exit_attempts = self.recovery.exit_attempts[candidate.candidate_id]
            return EXIT_ROOM
        if candidate.status != EXIT_ROOM and self.recovery.can_retry_door(candidate.candidate_id):
            candidate.status = DISCOVERED
        else:
            candidate.status = UNREACHABLE
        self.active_id = None
        return candidate.status

    def mark_exit_opposite(self, candidate_id: str) -> bool:
        """Arm one newly observed opposite door for the next normal selection."""
        candidate = self.candidates.get(candidate_id)
        if candidate is None or candidate.status != DISCOVERED:
            return False
        self.exit_opposite_candidate_id = candidate_id
        return True

    def mark_exit_opposite_from_evidence(
        self,
        evidence: Iterable[DoorEvidence],
        reference_center: Tuple[float, float],
        reference_normal_yaw: float,
        corridor_yaw: float,
        station_tolerance: float = 1.5,
    ) -> Optional[str]:
        """Arm an already-known opposite candidate after exit-time evidence.

        The evidence must have been acquired during the bounded exit probe.
        Matching it to an existing DISCOVERED candidate lets the probe save a
        corridor traversal without changing normal candidate discovery.
        """
        for observation in evidence:
            if not observation.opening_complete or observation.verification_status == REJECTED:
                continue
            for candidate in self.candidates.values():
                if candidate.status != DISCOVERED:
                    continue
                if not opposite_door_at_station(
                    reference_center,
                    reference_normal_yaw,
                    candidate.center,
                    candidate.normal_yaw,
                    corridor_yaw,
                    station_tolerance,
                ):
                    continue
                if (
                    math.hypot(
                        candidate.center[0] - observation.center[0],
                        candidate.center[1] - observation.center[1],
                    )
                    > self.dedup_radius
                    or abs(
                        normalize_angle(candidate.normal_yaw - observation.normal_yaw)
                    )
                    >= 0.35
                ):
                    continue
                if self.mark_exit_opposite(candidate.candidate_id):
                    return candidate.candidate_id
        return None

    def complete(
        self,
        sim_time: float,
        expected_room_count: Optional[int] = None,
        danger_confirmation_active: bool = False,
    ) -> bool:
        visited_count = sum(item.status == VISITED for item in self.candidates.values())
        expected = expected_room_count if expected_room_count is not None else len(self.candidates)
        if (
            expected <= 0
            or visited_count != expected
            or self.active_id is not None
            or danger_confirmation_active
        ):
            return False
        if self.completion_ready_time is None:
            self.completion_ready_time = sim_time
        return sim_time - self.completion_ready_time >= self.quiet_period


# Backward-compatible import name for existing tools while the single owner is renamed.
RoomMission = DoorCandidateManager
