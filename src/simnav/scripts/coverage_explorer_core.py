#!/usr/bin/env python3
"""Task-region coverage planning shared by the live explorer and tests."""

from collections import deque
from dataclasses import dataclass, replace
import heapq
import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt, label
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class GridView:
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    frame_id: str = "simnav_map"

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (
            int(math.floor((y - self.origin_y) / self.resolution)),
            int(math.floor((x - self.origin_x) / self.resolution)),
        )

    def cell_center(self, row: int, column: int) -> Tuple[float, float]:
        return (
            self.origin_x + (float(column) + 0.5) * self.resolution,
            self.origin_y + (float(row) + 0.5) * self.resolution,
        )


@dataclass(frozen=True)
class CoverageSnapshot:
    laser: float
    camera: float
    combined: float
    task_cells: int
    laser_known_cells: int
    camera_seen_cells: int


@dataclass(frozen=True)
class FrontierTarget:
    kind: str
    target: Tuple[float, float]
    path: Tuple[Tuple[float, float], ...]
    path_length: float
    laser_gain: float
    camera_gain: float
    combined_gain: float
    min_clearance: float
    look_at: Optional[Tuple[float, float]] = None
    hypothesis_id: Optional[str] = None
    # Online topological owner.  ``CORRIDOR`` is the connector; room IDs are
    # derived from observed door portals, never from Gazebo ground truth.
    topology_id: str = "CORRIDOR"


@dataclass(frozen=True)
class CoveragePlan:
    snapshot: CoverageSnapshot
    target: Optional[FrontierTarget]
    targets: Tuple[FrontierTarget, ...]
    reason: str
    task_forward_limit: float = 0.0
    task_extent_confident: bool = False
    navigation_reachable_cells: int = 0
    navigation_hard_clearance: float = 0.0
    # IDs represented by the unfiltered candidate pool.  This lets the live
    # node release a room lock only when that room really has no remaining
    # executable frontier, instead of mistaking a filtered global pool for
    # completion.
    candidate_topologies: Tuple[str, ...] = ()
    # All portal observations in this map update.  The live node uses these
    # to require temporal evidence before a width-only gap can own a room.
    observed_portals: "Tuple[RoomPortal, ...]" = ()
    # Temporally confirmed single doors that are allowed to own exploration
    # targets in the current front/rear phase.  Pairing is scheduling metadata,
    # not an entry prerequisite.  Raw observations remain available above.
    actionable_portals: "Tuple[RoomPortal, ...]" = ()
    # Local sensor coverage for confirmed room topologies.  Room lifecycle
    # must use this rather than a count of visited waypoints.
    topology_coverages: Optional[dict] = None
    # The entrance-nearest doorway station is a hard phase boundary.  Both
    # sides must be completed before the live node may start rear transit.
    front_station_portals: "Tuple[RoomPortal, ...]" = ()
    front_station_along: Optional[float] = None
    front_rooms_complete: bool = False
    # Bounded, read-only counters describing where a room candidate pool was
    # reduced.  These are intentionally part of the pure planner result so a
    # live NO_FRONTIER can be diagnosed without changing any thresholds.
    diagnostics: Optional[dict] = None


@dataclass(frozen=True)
class RoomPortal:
    """A stable, map-derived doorway crossing on one side of the corridor."""

    topology_id: str
    side: str
    along: float
    lateral: float
    width: float = 0.0
    # Distance from the corridor axis to the wall this doorway was measured
    # against.  ``lateral`` is the nominal side offset; this records what the
    # raw map actually showed, which is what the entry and return tests need.
    measured_wall: float = 0.0


@dataclass(frozen=True)
class PortalStation:
    """An opposing left/right doorway pair at one corridor station."""

    station_id: str
    along: float
    left: RoomPortal
    right: RoomPortal


def opposite_room_portal(
    source: RoomPortal,
    portals: "Sequence[RoomPortal]" = (),
    station_tolerance: float = 2.5,
) -> RoomPortal:
    """Return the observed opposite door, or mirror the stable source gate.

    Door observations on opposite sides may land in adjacent 0.5 m ID bins as
    the map is refined.  Match by physical station first; the synthetic form
    is only a cached topological gate and retains the measured door width.
    """
    opposite_side = "R" if source.side == "L" else "L"
    candidates = [item for item in portals if item.side == opposite_side]
    if candidates:
        nearest = min(candidates, key=lambda item: abs(item.along - source.along))
        if abs(float(nearest.along) - float(source.along)) <= float(station_tolerance):
            return nearest
    try:
        prefix, side, stable_bin = str(source.topology_id).rsplit("_", 2)
    except ValueError:
        prefix, side, stable_bin = "ROOM", source.side, str(
            int(round(float(source.along) / 0.5))
        )
    if side not in ("L", "R"):
        prefix = str(source.topology_id)
        stable_bin = str(int(round(float(source.along) / 0.5)))
    return RoomPortal(
        "{}_{}_{}".format(prefix, opposite_side, stable_bin),
        opposite_side,
        float(source.along),
        -float(source.lateral),
        float(source.width),
    )


def pair_room_portals(
    portals: Sequence[RoomPortal],
    pairing_tolerance: float = 1.0,
    minimum_along: float = 2.0,
    minimum_station_separation: float = 6.0,
) -> Tuple[PortalStation, ...]:
    """Return actionable opposing doorway stations.

    A single same-width gap on one corridor wall is not enough to create a
    room topology.  Competition rooms occur as opposing left/right pairs, so
    dispatch uses only mutually nearest portal pairs.  The small exclusion at
    the topology gate rejects map seams generated immediately after the fixed
    lobby transit, while station de-duplication prevents one physical pair
    from being split into two adjacent topology stations.
    """
    tolerance = max(0.1, float(pairing_tolerance))
    minimum_along = max(0.0, float(minimum_along))
    minimum_station_separation = max(0.0, float(minimum_station_separation))
    left = sorted(
        (item for item in portals if item.side == "L" and item.along >= minimum_along),
        key=lambda item: item.along,
    )
    right = sorted(
        (item for item in portals if item.side == "R" and item.along >= minimum_along),
        key=lambda item: item.along,
    )
    candidates = []
    for left_portal in left:
        if not right:
            continue
        nearest_right = min(right, key=lambda item: abs(item.along - left_portal.along))
        nearest_left = min(left, key=lambda item: abs(item.along - nearest_right.along))
        mismatch = abs(left_portal.along - nearest_right.along)
        if nearest_left.topology_id != left_portal.topology_id or mismatch > tolerance:
            continue
        along = 0.5 * (left_portal.along + nearest_right.along)
        width = 0.5 * (left_portal.width + nearest_right.width)
        # Prefer a tightly aligned pair, then a doorway-sized opening.  The
        # latter is only a de-duplication tie-breaker, not a detection gate.
        score = (mismatch, abs(width - 1.2), -along)
        candidates.append((along, score, left_portal, nearest_right))

    selected = []
    for along, score, left_portal, right_portal in sorted(candidates, key=lambda item: item[0]):
        if selected and along - selected[-1][0] < minimum_station_separation:
            if score < selected[-1][1]:
                selected[-1] = (along, score, left_portal, right_portal)
            continue
        selected.append((along, score, left_portal, right_portal))

    return tuple(
        PortalStation(
            "STATION_{}".format(int(round(along / 0.5))),
            float(along),
            left_portal,
            right_portal,
        )
        for along, _score, left_portal, right_portal in selected
    )


@dataclass(frozen=True)
class TaskExtent:
    """Online estimate of where the room-bearing task region ends."""

    forward_limit: float
    confident: bool
    lateral_span: float
    left_area: float
    right_area: float
    terminal_corridor_depth: float
    terminal_corridor_start: Optional[float] = None
    room_end_wall: Optional[float] = None


def interior_targets_only(inside_room, retries, retry_limit):
    """Whether only in-room targets may be dispatched right now.

    Policy (user-specified): after entering a room the robot may re-orient and
    manoeuvre freely, but the target must not be moved back out into the
    corridor until the interior has genuinely been retried and failed several
    times.  That keeps the entry meaningful (no enter-then-immediately-leave
    shuttle) while preserving a bounded escape from the "no interior candidate"
    deadlock.
    """
    if not inside_room:
        return False
    return int(retries) < max(1, int(retry_limit))


def clamp_linear_speed(value, maximum):
    """Clamp a commanded linear speed into [-maximum, +maximum].

    One auditable enforcement point: the mission is required to stay at or below
    0.60 m/s everywhere (transit, elevator approach, door crossings), because a
    faster command tipped the robot over during startup (run70) and makes the
    whole run unusable.
    """
    limit = max(0.0, float(maximum))
    return max(-limit, min(limit, float(value)))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def measure_corridor_walls(
    grid: GridView,
    gate_center: Tuple[float, float],
    forward_yaw: float,
    corridor_half_width: float,
    forward_depth: float = 35.0,
    lateral_half_width: float = 9.5,
    minimum_cells: int = 20,
) -> Tuple[Optional[float], Optional[float]]:
    """Return the observed lateral distance to the left and right corridor walls.

    The corridor axis is anchored to the robot's initial pose, so it can sit
    off the physical centreline.  Measured on a three-floor run, the axis was
    0.53 m off centre: the left wall read 0.77 m and the right 1.63 m while the
    true wall spacing is 1.10 m per side.  Every downstream lateral judgement
    (doorway bands, room entry, return to corridor) inherits that bias unless
    the offset is known.

    Only occupied cells within a short distance of the axis are considered, so
    room partitions and furniture several metres away cannot drag the estimate.
    """
    data = np.asarray(grid.data)
    rows, columns = np.indices(data.shape, dtype=np.float64)
    dx = grid.origin_x + (columns + 0.5) * grid.resolution - float(gate_center[0])
    dy = grid.origin_y + (rows + 0.5) * grid.resolution - float(gate_center[1])
    cosine, sine = math.cos(float(forward_yaw)), math.sin(float(forward_yaw))
    along = dx * cosine + dy * sine
    lateral = -dx * sine + dy * cosine
    in_search = (
        (along >= 0.0)
        & (along <= float(forward_depth))
        & (np.abs(lateral) <= float(lateral_half_width))
    )
    near_limit = max(
        1.60 * float(corridor_half_width),
        float(corridor_half_width) + 1.90,
    )
    walls = []
    for sign in (1.0, -1.0):
        signed = sign * lateral
        evidence = (
            (data >= 50)
            & in_search
            & (signed > 0.0)
            & (signed <= near_limit)
        )
        if np.count_nonzero(evidence) >= int(minimum_cells):
            walls.append(float(np.median(signed[evidence])))
        else:
            walls.append(None)
    return walls[0], walls[1]


def topology_state_for_new_target(current_state: Optional[str]) -> str:
    """Keep room-entry proof when selecting another target in that room."""
    if current_state in ("EXPLORING", "RETURNING", "COMPLETE", "BLOCKED"):
        return str(current_state)
    return "APPROACHING"


def target_kind_allowed_for_topology_state(
    topology_state: Optional[str], target_kind: Optional[str]
) -> bool:
    """Enforce exclusive target ownership for safety-critical room states."""
    if str(topology_state) == "RETURNING":
        return str(target_kind) == "RETURN_TO_CORRIDOR"
    return True


def portal_return_along_offsets(portal_width: float) -> Tuple[float, ...]:
    """Return a bounded centre-out search that stays inside a known doorway."""
    half_search = max(0.0, min(0.35, 0.5 * float(portal_width) - 0.12))
    result = [0.0]
    for offset in (0.10, 0.20, 0.30):
        if offset <= half_search + 1e-6:
            result.extend((-offset, offset))
    return tuple(result)


def zone_split_along(portals, fallback_along):
    """Midpoint of the corridor in the gate's ``along`` coordinate.

    Rooms are explored a corridor half at a time: the near half and its two
    rooms form one zone, the far half and its rooms the next.  Splitting the
    corridor means a locked room is never the only place to go - the rest of the
    zone (its half-corridor and the opposite room) stays available, which is what
    stops "entered the room, no frontier candidate left, stood still".
    """
    values = sorted(
        float(portal.along)
        for portal in (portals or ())
        if getattr(portal, "along", None) is not None and float(portal.along) > 0.0
    )
    if len(values) >= 2:
        return 0.5 * (values[0] + values[-1])
    return 0.5 * max(0.0, float(fallback_along or 0.0))


def zone_of_along(along, split, band=1.5):
    """Corridor half a doorway belongs to, with a band around the split.

    A doorway inside the band stays in the near zone so an opposing pair that
    straddles the midpoint is not torn between two zones.
    """
    return "A" if float(along) <= float(split) + float(band) else "B"


# A doorway narrow enough to be a wall seam rather than a door may not take part
# in the near/far zone decision.  Measured on this building: the real doorways
# are 0.90-1.00 m wide, while the corridor-mouth seams come back at 0.30 m
# (ROOM_L_0) and 0.40 m (ROOM_R_0) with evidence 40.  Those seams can never be
# "completed", so counting them held ``active_zone`` at "A" for ever and
# ``zone_admits`` then rejected every candidate past the split: run151's raw
# candidate pool collapsed to the already-retired ROOM_L_15, the plan reason
# became NO_FRONTIER and the robot stood still for 40+ s while unexplored far-zone
# rooms were actionable (the same class of failure the core already documents for
# the drifted bin ROOM_L_36 on the transit path).
MINIMUM_ZONE_PORTAL_WIDTH = 0.70


def active_zone(portals, completed, split, band=1.5, minimum_width=None):
    """Near zone until every near-zone doorway is complete, then the far zone.

    Implements "finish the lower half before moving to the upper half" while
    staying derived from observed doorways rather than a schedule.

    ``minimum_width`` excludes seam-sized "doorways" from this decision (see
    ``MINIMUM_ZONE_PORTAL_WIDTH``); pass ``0.0`` to count every portal.
    """
    if minimum_width is None:
        minimum_width = MINIMUM_ZONE_PORTAL_WIDTH
    minimum_width = max(0.0, float(minimum_width))
    completed = {str(item) for item in (completed or ())}
    near = 0
    far = 0
    for portal in portals or ():
        if str(getattr(portal, "topology_id", "")) in completed:
            continue
        if float(getattr(portal, "width", 0.0) or 0.0) < minimum_width:
            continue
        if zone_of_along(float(portal.along), split, band) == "A":
            near += 1
        else:
            far += 1
    if near:
        return "A"
    if far:
        return "B"
    return "A"


def zone_admits(along, split, active, band=1.5):
    """Whether a point at ``along`` may be used while ``active`` zone runs."""
    return zone_of_along(along, split, band) == str(active)


CORRIDOR_TRANSIT_REACHES = (2.0, 4.0, 6.0, 9.0, 14.0, 20.0, 26.0, 32.0)
# The committed corridor leg lands this far SHORT of the doorway it aims at.
# User-tuned: "reduce by 2 m" was too short, so it now stops 1 m short - the
# doorway approach and room entry own the last metre.
COMMITTED_LANDING_SHORTEN = 1.0


def corridor_transit_reaches(forward_only=False, descending=False):
    """Ordered centreline offsets a corridor transit step may pick.

    P3b: a far-zone transit is a *committed forward leg*, not a choice from the
    frontier pool.  Ranking the pool re-decided the target on every map update
    (the pool changes as the map grows), which is what produced the stop-and-go
    corridor legs.  ``forward_only`` drops the backward half so a transit leg
    only ever makes progress along the positive corridor axis; ``descending``
    tries the longest leg first and shortens it until one is navigable.
    """
    reaches = list(CORRIDOR_TRANSIT_REACHES)
    if not forward_only:
        reaches = [value for step in reaches for value in (step, -step)]
    if descending:
        reaches = sorted(reaches, reverse=True)
    return tuple(reaches)


def prefer_room_approach(
    chosen_topology, failed_room_ids, chosen_gain=None, unobserved_rooms=0,
    weak_gain=1.0,
):
    """Whether a door approach should outrank the chosen corridor target.

    Two reasons to divert to a door:

    * a room candidate could not be routed yet (``failed_room_ids``) - the
      interior becomes partly known on the way there, which is what makes the
      crossing plannable at all; and
    * observation is worth buying even when nothing has failed: if the best
      remaining action is a nearly worthless corridor target while an
      unobserved doorway is still pending, the expected value of going to look
      is higher than the certain-but-tiny gain of the corridor move.  This is
      the user-specified "trigger it more readily, because apparently worthless
      tasks can pay off unexpectedly" policy, kept bounded by requiring the
      corridor alternative to be below ``weak_gain``.
    """
    if str(chosen_topology) != "CORRIDOR":
        return False
    if failed_room_ids:
        return True
    if unobserved_rooms and chosen_gain is not None:
        return float(chosen_gain) < float(weak_gain)
    return False


def door_approach_is_new(stage, visited, revisit_radius):
    """Whether a door-approach viewpoint has not already been driven to.

    Keeps the approach fallback from re-dispatching the same viewpoint for ever
    once the robot has been there: if the room still cannot be routed after
    standing in front of its door, the honest outcome is NO_FRONTIER rather than
    an endless shuttle.
    """
    for point in visited or ():
        if math.hypot(
            float(stage[0]) - float(point[0]), float(stage[1]) - float(point[1])
        ) < float(revisit_radius):
            return False
    return True


def door_crossing_along_offsets(
    portal_width, wide_search=2.5, scan_step=0.5
) -> Tuple[float, ...]:
    """Centre-out along-corridor search for a doorway crossing.

    ``portal_return_along_offsets`` only scans inside the measured doorway
    (+/-0.30 m), which cannot compensate for a portal whose reported ``along``
    sits off the real opening.  Portal ids are 0.5 m bins that drift as SLAM
    refines a wall, and run48 floor 0 dead-locked on exactly that: the opposite
    room was locked as ROOM_L_47 with 4015 reachable camera-unseen cells, but
    every candidate failed to route and the verified door band held 0 cells at
    the reported along, so the planner reported NO_FRONTIER and the robot stood
    still for the rest of the floor.  This bounded fallback runs only after the
    in-doorway search fails.
    """
    offsets = list(portal_return_along_offsets(portal_width))
    step = max(0.1, float(scan_step))
    count = int(round(max(0.0, float(wide_search)) / step))
    for index in range(1, count + 1):
        offsets.extend((-index * step, index * step))
    return tuple(offsets)


def projected_travel(origin, point, yaw: float) -> float:
    return (
        (float(point[0]) - float(origin[0])) * math.cos(yaw)
        + (float(point[1]) - float(origin[1])) * math.sin(yaw)
    )


def weighted_harmonic_coverage(
    laser: float, camera: float, camera_weight: float = 0.65
) -> float:
    """Combine coverages without allowing one strong sensor to hide the other."""
    laser = max(0.0, min(1.0, float(laser)))
    camera = max(0.0, min(1.0, float(camera)))
    camera_weight = max(0.0, min(1.0, float(camera_weight)))
    if laser <= 0.0 or camera <= 0.0:
        return 0.0
    laser_weight = 1.0 - camera_weight
    return 1.0 / (laser_weight / laser + camera_weight / camera)


def weighted_linear_coverage(
    laser: float, camera: float, camera_weight: float = 0.95
) -> float:
    """Combine room coverage with an explicit sensor contribution split."""
    laser = max(0.0, min(1.0, float(laser)))
    camera = max(0.0, min(1.0, float(camera)))
    camera_weight = max(0.0, min(1.0, float(camera_weight)))
    return (1.0 - camera_weight) * laser + camera_weight * camera


def topology_completion_ready(
    completed_topologies: Iterable[str], expected_rooms: int, unreviewed_count: int
) -> bool:
    rooms = {str(item) for item in completed_topologies if str(item).startswith("ROOM_")}
    return len(rooms) >= max(1, int(expected_rooms)) and int(unreviewed_count) == 0


def task_region_mask(
    grid: GridView,
    gate_center: Tuple[float, float],
    forward_yaw: float,
    back_extension: float = 3.6,
    forward_depth: float = 25.0,
    lateral_half_width: float = 9.5,
    corridor_half_width: float = 1.1,
    terminal_corridor_start: Optional[float] = None,
    gate_half_width: Optional[float] = None,
    gate_depth: float = 0.30,
) -> np.ndarray:
    """Return the corridor-plus-four-rooms task mask in gate-local coordinates.

    The back boundary is placed at the online-observed corridor entrance.  It
    excludes the lobby while retaining the whole near section of the corridor.
    When ``gate_half_width`` is supplied, the entrance is represented as a
    closed *topological* gate: cells in a short slab around the gate plane
    covers only the configured corridor width.  This separates the lobby from
    the corridor without adding an obstacle to the navigation map.  The gate
    has no artificial centre opening; the robot has already crossed this
    boundary by the time task-region planning starts.
    """
    rows, columns = np.indices(grid.data.shape, dtype=np.float64)
    x = grid.origin_x + (columns + 0.5) * grid.resolution
    y = grid.origin_y + (rows + 0.5) * grid.resolution
    dx = x - float(gate_center[0])
    dy = y - float(gate_center[1])
    cosine = math.cos(float(forward_yaw))
    sine = math.sin(float(forward_yaw))
    along = dx * cosine + dy * sine
    lateral = -dx * sine + dy * cosine
    envelope = (
        (along >= -abs(float(back_extension)))
        & (along <= float(forward_depth))
        & (np.abs(lateral) <= float(lateral_half_width))
    )
    if gate_half_width is not None:
        gate_half_width = max(0.0, min(float(lateral_half_width), float(gate_half_width)))
        gate_depth = max(0.0, float(gate_depth))
        gate_slab = (along >= -gate_depth) & (along <= gate_depth)
        # Close only the corridor cross-section.  Cells outside this bounded
        # width are room/lobby space and are not part of the virtual door.
        # The fixed lobby transit finishes before task planning starts and
        # ``minimum_forward`` places the first executable goal beyond the
        # slab.  The raw navigation map is never modified.
        envelope &= ~(gate_slab & (np.abs(lateral) <= gate_half_width))
    # ``terminal_corridor_start`` remains an optional diagnostic input.  The
    # mask ends at the online-observed common far wall, so the main corridor
    # between the last doors and that wall remains part of task coverage.
    return envelope


def infer_task_extent(
    grid: GridView,
    gate_center: Tuple[float, float],
    forward_yaw: float,
    fallback_forward_depth: float,
    back_extension: float = 3.6,
    lateral_half_width: float = 9.5,
    corridor_half_width: float = 1.1,
    minimum_lateral_span: float = 11.0,
    minimum_side_area: float = 3.0,
    terminal_evidence_depth: float = 1.2,
    terminal_padding: float = 0.45,
    minimum_end_wall_span: float = 3.5,
) -> TaskExtent:
    """Infer the terminal task boundary using only the online occupancy map.

    A boundary is accepted only after broad free space has been observed on
    both sides over a long longitudinal span (evidence for the room pairs),
    and the known centre corridor continues beyond that broad space.  This
    distinguishes a terminal narrow corridor from an undiscovered room pair.
    Until all evidence exists, the conservative fallback extent is returned
    and completion must remain disabled.
    """
    data = np.asarray(grid.data)
    rows, columns = np.indices(data.shape, dtype=np.float64)
    x = grid.origin_x + (columns + 0.5) * grid.resolution
    y = grid.origin_y + (rows + 0.5) * grid.resolution
    dx = x - float(gate_center[0])
    dy = y - float(gate_center[1])
    cosine = math.cos(float(forward_yaw))
    sine = math.sin(float(forward_yaw))
    along = dx * cosine + dy * sine
    lateral = -dx * sine + dy * cosine
    known_free = (data >= 0) & (data < 50)
    occupied = data >= 50
    in_search = (
        (along >= -abs(float(back_extension)))
        & (along <= float(fallback_forward_depth))
        & (np.abs(lateral) <= float(lateral_half_width))
    )
    # Broad lateral free space is room evidence.  It is not by itself enough
    # to locate the last opening because the rooms continue beyond their door.
    room_offset = float(corridor_half_width) + 0.45
    left = known_free & in_search & (lateral >= room_offset)
    right = known_free & in_search & (lateral <= -room_offset)
    lateral_free = left | right
    cell_area = grid.resolution ** 2
    left_area = float(np.count_nonzero(left)) * cell_area
    right_area = float(np.count_nonzero(right)) * cell_area
    lateral_along = along[lateral_free]
    if lateral_along.size:
        lateral_min = max(0.0, float(np.percentile(lateral_along, 2.0)))
        lateral_max = float(np.percentile(lateral_along, 98.0))
        lateral_span = max(0.0, lateral_max - lateral_min)
    else:
        lateral_max = 0.0
        lateral_span = 0.0

    # Detect wall-crossing free bands.  This is deliberately permissive and is
    # only used to trim the coverage denominator, never to command a crossing.
    # Requiring two separated openings on each side prevents a single mapping
    # gap from being mistaken for the last room pair.
    def opening_centres(side_sign):
        signed_lateral = side_sign * lateral
        crossing = (
            known_free
            & in_search
            & (signed_lateral >= float(corridor_half_width) - 0.08)
            & (signed_lateral <= float(corridor_half_width) + 0.08)
            & (along >= 0.0)
        )
        bin_count = max(1, int(math.ceil(float(fallback_forward_depth) / grid.resolution)))
        indices = np.floor(along[crossing] / grid.resolution).astype(np.int64)
        indices = indices[(indices >= 0) & (indices < bin_count)]
        counts = np.bincount(indices, minlength=bin_count)
        active = np.nonzero(counts > 0)[0]
        if not len(active):
            return []
        groups = [[int(active[0])]]
        for index in active[1:]:
            if int(index) - groups[-1][-1] <= 2:
                groups[-1].append(int(index))
            else:
                groups.append([int(index)])
        return [
            (group[0] + group[-1] + 1) * 0.5 * grid.resolution
            for group in groups
            if 0.40 <= (group[-1] - group[0] + 1) * grid.resolution <= 2.0
        ]

    left_openings = opening_centres(1.0)
    right_openings = opening_centres(-1.0)
    paired_last_opening = None
    if len(left_openings) >= 2 and len(right_openings) >= 2:
        left_last, right_last = left_openings[-1], right_openings[-1]
        if abs(left_last - right_last) <= 1.5:
            paired_last_opening = 0.5 * (left_last + right_last)
    terminal_start = (
        paired_last_opening + float(terminal_padding)
        if paired_last_opening is not None
        else None
    )
    centre = known_free & in_search & (np.abs(lateral) <= 0.65 * float(corridor_half_width))
    centre_ahead = along[
        centre
        & (along > terminal_start if terminal_start is not None else np.zeros_like(along, dtype=bool))
    ]
    centre_max = float(np.max(centre_ahead)) if centre_ahead.size else (terminal_start or 0.0)
    terminal_depth = max(0.0, centre_max - (terminal_start or centre_max))

    room_end_wall = None
    if terminal_start is not None:
        wall_offset = float(corridor_half_width) + 0.55
        left_wall = (
            occupied
            & in_search
            & (lateral >= wall_offset)
            & (lateral <= float(lateral_half_width))
            & (along >= terminal_start + 1.0)
        )
        right_wall = (
            occupied
            & in_search
            & (lateral <= -wall_offset)
            & (lateral >= -float(lateral_half_width))
            & (along >= terminal_start + 1.0)
        )
        bin_count = max(1, int(math.ceil(float(fallback_forward_depth) / grid.resolution)))
        def longitudinal_counts(mask):
            indices = np.floor(along[mask] / grid.resolution).astype(np.int64)
            indices = indices[(indices >= 0) & (indices < bin_count)]
            return np.bincount(indices, minlength=bin_count)
        left_counts = longitudinal_counts(left_wall)
        right_counts = longitudinal_counts(right_wall)
        end_bins = np.nonzero(
            (left_counts * grid.resolution >= float(minimum_end_wall_span))
            & (right_counts * grid.resolution >= float(minimum_end_wall_span))
        )[0]
        if len(end_bins):
            room_end_wall = (float(end_bins[0]) + 0.5) * grid.resolution
    confident = bool(
        lateral_span >= float(minimum_lateral_span)
        and left_area >= float(minimum_side_area)
        and right_area >= float(minimum_side_area)
        and terminal_start is not None
        and terminal_depth >= float(terminal_evidence_depth)
        and room_end_wall is not None
    )
    return TaskExtent(
        forward_limit=float(room_end_wall) if confident else float(fallback_forward_depth),
        confident=confident,
        lateral_span=lateral_span,
        left_area=left_area,
        right_area=right_area,
        terminal_corridor_depth=terminal_depth,
        terminal_corridor_start=terminal_start if confident else None,
        room_end_wall=room_end_wall if confident else None,
    )


def distinct_room_count(topology_ids, portals, station_tolerance=2.5) -> int:
    """Number of distinct physical rooms among a set of completed topology ids.

    Doorway ids are 0.5 m bins that drift as SLAM refines a wall, so one physical
    doorway can appear as several ids: floor 0 of run79 reported nine ids for
    four rooms.  Counting ids lets four copies of a single door satisfy
    ``expected_rooms_per_floor``, which would latch the floor with a real room
    unvisited.  Counting *doors* - same side, ``along`` within
    ``station_tolerance`` - keeps floor completion honest without introducing a
    second completion ledger.
    """
    by_id = {}
    for portal in portals or ():
        by_id[str(getattr(portal, "topology_id", ""))] = portal
    groups = []
    for item in topology_ids or ():
        portal = by_id.get(str(item))
        if portal is None:
            groups.append(None)
            continue
        for index, reference in enumerate(groups):
            if reference is not None and doorways_match(
                portal, reference, station_tolerance
            ):
                break
        else:
            groups.append(portal)
    return len(groups)


def detect_room_portals(
    grid: GridView,
    gate_center: Tuple[float, float],
    forward_yaw: float,
    forward_depth: float,
    lateral_half_width: float = 9.5,
    corridor_half_width: float = 1.1,
    minimum_along: float = 0.0,
    portal_prefix: str = "ROOM",
    max_portal_width: float = 2.4,
    minimum_jamb_support: float = 0.0,
    passage_depth: float = 0.70,
) -> Tuple[RoomPortal, ...]:

    """Find persistent side openings from the occupancy map.

    This is intentionally only a *topology hint*.  It uses free cells that
    cross the observed corridor wall and groups them longitudinally.  No
    generated layout, room coordinates, or Gazebo truth is consulted.  A
    planner must still check the navigation map before using a portal.
    """
    data = np.asarray(grid.data)
    rows, columns = np.indices(data.shape, dtype=np.float64)
    dx = grid.origin_x + (columns + 0.5) * grid.resolution - float(gate_center[0])
    dy = grid.origin_y + (rows + 0.5) * grid.resolution - float(gate_center[1])
    cosine, sine = math.cos(float(forward_yaw)), math.sin(float(forward_yaw))
    along = dx * cosine + dy * sine
    lateral = -dx * sine + dy * cosine
    known_free = (data >= 0) & (data < 50)
    in_search = (
        (along >= float(minimum_along))
        & (along <= float(forward_depth))
        & (np.abs(lateral) <= float(lateral_half_width))
    )
    bin_count = max(1, int(math.ceil(float(forward_depth) / grid.resolution)))
    portals = []
    # Search just outside the corridor edge.  The portal grid is the raw map,
    # so this band follows the observed wall instead of the inflated nav map.
    # A width-only gap is not a doorway: furniture edges and mapping holes can
    # have the same apparent width.  A real room door must remain a gap in a
    # *continuous parent wall*, with occupied jamb evidence immediately before
    # and after the opening and a short free passage on the room side.
    jamb_length = 0.30
    jamb_support = float(minimum_jamb_support)
    measured_walls = {}
    for side, sign in (("L", 1.0), ("R", -1.0)):
        signed = sign * lateral
        # The corridor axis is anchored to the robot's initial pose and can
        # therefore sit off the physical centreline.  Estimate each wall from
        # raw occupied cells so the doorway band follows the observed wall.
        wall_band = 0.22
        # Only the corridor's own wall may define the doorway band.  Room
        # partitions and furniture sit several metres further out; taking a
        # median over the whole lateral search window let them dominate the
        # estimate (a furnished rear pair drifted from the true 1.15 m to
        # 4.25 m), which pushed the doorway band off the opening entirely.
        wall_near_limit = max(
            1.60 * float(corridor_half_width),
            float(corridor_half_width) + 1.90,
        )
        wall_evidence = (
            (data >= 50)
            & in_search
            & (signed > 0.0)
            & (signed <= wall_near_limit)
        )
        measured_wall = None
        if np.count_nonzero(wall_evidence) >= 20:
            measured_wall = float(np.median(signed[wall_evidence]))
        if measured_wall is None:
            measured_wall = float(corridor_half_width)
        # A spurious deep-wall median must not drag the band into a room.
        measured_wall = max(
            0.55 * float(corridor_half_width),
            min(1.60 * float(corridor_half_width), measured_wall),
        )
        measured_walls[side] = float(measured_wall)
        # A free cell on the corridor edge is not evidence of a doorway.  The
        # previous implementation used this whole band directly; because the
        # real wall inner face is at ``corridor_half_width``, ordinary corridor
        # free cells made every longitudinal bin look open.  The resulting
        # room-length group was then rejected as wider than a door.  A portal
        # bin must instead contain observed free space *and no occupied wall
        # evidence* across the wall band.
        crossing = (
            known_free
            & in_search
            & (signed >= measured_wall + 0.05)
            & (signed <= measured_wall + 0.60)
        )
        indices = np.floor(along[crossing] / grid.resolution).astype(np.int64)
        indices = indices[(indices >= 0) & (indices < bin_count)]
        free_counts = np.bincount(indices, minlength=bin_count)
        wall_cells = (data >= 50) & in_search & (
            signed >= measured_wall - wall_band
        ) & (signed <= measured_wall + 0.60)
        wall_indices_array = np.floor(
            along[wall_cells] / grid.resolution
        ).astype(np.int64)
        wall_indices_array = wall_indices_array[
            (wall_indices_array >= 0) & (wall_indices_array < bin_count)
        ]
        wall_counts = np.bincount(wall_indices_array, minlength=bin_count)
        active = np.nonzero((free_counts > 0) & (wall_counts == 0))[0]
        wall_indices = set(int(value) for value in wall_indices_array)
        groups = []
        for index in active:
            index = int(index)
            if not groups or index - groups[-1][-1] > 3:
                groups.append([index])
            else:
                groups[-1].append(index)
        candidates = []
        for group in groups:
            width = (group[-1] - group[0] + 1) * grid.resolution
            # Width is a hard physical sanity range; the remaining decision
            # is a low-threshold score so sparse maps do not veto a doorway.
            if 0.30 <= width <= float(max_portal_width):
                start, end = int(group[0]), int(group[-1])
                support_bins = max(2, int(math.ceil(jamb_length / grid.resolution)))
                left_expected = range(start - support_bins, start)
                right_expected = range(end + 1, end + 1 + support_bins)
                left_ratio = sum(index in wall_indices for index in left_expected) / float(support_bins)
                right_ratio = sum(index in wall_indices for index in right_expected) / float(support_bins)
                if min(left_ratio, right_ratio) < jamb_support:
                    continue
                # Require that the gap opens into observed free room space.
                # This rejects a short same-width discontinuity whose far side
                # is still occupied/unknown, while keeping the test permissive
                # enough for a partially mapped genuine doorway.
                centre_along = (start + end + 1) * 0.5 * grid.resolution
                # Validate the room side, not the apparent doorway width.
                # A small rectangular sample is robust to inflation clipping
                # the threshold itself while still rejecting isolated map
                # holes that do not open into a room.
                passage = []
                unknown_passage = 0
                for depth in (0.20, 0.45, 0.70, 0.95):
                    for lateral_offset in (-0.45, 0.0, 0.45):
                        point_x = float(gate_center[0]) + cosine * centre_along - sine * sign * (measured_wall + depth) + cosine * lateral_offset
                        point_y = float(gate_center[1]) + sine * centre_along + cosine * sign * (measured_wall + depth) + sine * lateral_offset
                        row, column = grid.world_to_cell(point_x, point_y)
                        if 0 <= row < data.shape[0] and 0 <= column < data.shape[1]:
                            value = int(data[row, column])
                            passage.append(value == 0)
                            unknown_passage += int(value < 0)
                known_cells = len(passage)
                free_cells = sum(passage)
                # Interior mapping is intentionally not a hard condition:
                # laser structure and repeated observations decide validity.
                if known_cells < 1 and unknown_passage < 1:
                    continue
                width_score = 25.0 if width <= 1.8 else 15.0 + 10.0 * (2.4 - width) / 0.6
                depth_score = min(30.0, 30.0 * (free_cells + 0.5 * unknown_passage) / 12.0)
                map_score = 15.0 if min(left_ratio, right_ratio) > 0.0 else 10.0
                # Passage/depth is supporting evidence only.  A clear raw-map
                # wall break with valid physical width is actionable even
                # before the room interior has been fully mapped.
                if width_score + depth_score + map_score >= 30.0:
                    candidates.append((centre_along, width))
        for value, width in candidates:
            # Quantise the observed longitudinal coordinate rather than using
            # the list index.  As SLAM reveals a nearer doorway, list indices
            # can shift; a coordinate-based ID keeps an already active room
            # stable across map updates.
            stable_bin = int(round(float(value) / 0.5))
            portals.append(
                RoomPortal(
                    "{}_{}_{}".format(portal_prefix, side, stable_bin),
                    side,
                    float(value),
                    float(sign * measured_wall),
                    float(width),
                    measured_wall=float(measured_wall),
                )
            )
    return tuple(portals)


def detect_lobby_portals(
    grid: GridView,
    gate_center: Tuple[float, float],
    forward_yaw: float,
    lobby_depth: float = 8.0,
    lateral_half_width: float = 4.5,
    corridor_half_width: float = 1.1,
) -> Tuple[RoomPortal, ...]:
    """Detect wide lobby-side openings with the same wall-gap evidence.

    Lobby/elevator openings are kept out of the ROOM_* topology pool.  The
    detector reuses the room doorway checks but searches the negative side of
    the entrance gate, where the elevator lobby is located.
    """
    # Reverse the gate tangent so the lobby becomes a normal positive-along
    # search window; this preserves the established non-negative binning and
    # all room-door evidence checks.
    portals = detect_room_portals(
        grid,
        gate_center,
        forward_yaw + math.pi,
        forward_depth=lobby_depth,
        lateral_half_width=lateral_half_width,
        corridor_half_width=corridor_half_width,
        portal_prefix="LOBBY",
        max_portal_width=6.0,
        minimum_jamb_support=0.0,
        passage_depth=0.45,
    )
    if portals:
        return portals

    # Elevator lobbies often appear as a square/half-open free region rather
    # than a thin doorway. Use connected map geometry as a permissive fallback.
    data = np.asarray(grid.data)
    rows, columns = np.indices(data.shape, dtype=np.float64)
    yaw = float(forward_yaw) + math.pi
    cosine, sine = math.cos(yaw), math.sin(yaw)
    dx = grid.origin_x + (columns + 0.5) * grid.resolution - float(gate_center[0])
    dy = grid.origin_y + (rows + 0.5) * grid.resolution - float(gate_center[1])
    along = dx * cosine + dy * sine
    lateral = -dx * sine + dy * cosine
    free = ((data == 0) & (along >= 0.0) & (along <= float(lobby_depth)) &
            (np.abs(lateral) <= float(lateral_half_width)))
    components, count = label(free, structure=np.ones((3, 3), dtype=np.int8))
    candidates = []
    min_cells = max(20, int(round(1.2 / grid.resolution)) ** 2)
    for component_id in range(1, int(count) + 1):
        component = components == component_id
        if int(np.count_nonzero(component)) < min_cells:
            continue
        span_along = float(np.ptp(along[component]))
        span_lateral = float(np.ptp(lateral[component]))
        if span_along < 0.8 or span_lateral < 0.8:
            continue
        candidates.append((float(np.mean(along[component])),
                           float(np.mean(lateral[component])), span_lateral))
    if not candidates:
        return ()
    along_value, centre_lateral, span_lateral = sorted(candidates, key=lambda item: item[0])[0]
    side = "L" if centre_lateral >= 0.0 else "R"
    return (RoomPortal("LOBBY_{}_{}".format(side, int(round(along_value / 0.5))),
                       side, along_value, centre_lateral, span_lateral),)


def topology_id_for_point(
    point: Tuple[float, float],
    gate_center: Tuple[float, float],
    forward_yaw: float,
    corridor_half_width: float,
    portals: Sequence[RoomPortal] = (),
) -> str:
    """Assign a point to the corridor or the nearest observed side room."""
    along = projected_travel(gate_center, point, forward_yaw)
    lateral = (
        -(float(point[0]) - float(gate_center[0])) * math.sin(float(forward_yaw))
        + (float(point[1]) - float(gate_center[1])) * math.cos(float(forward_yaw))
    )
    if abs(lateral) <= float(corridor_half_width) + 0.45:
        return "CORRIDOR"
    side = "L" if lateral > 0.0 else "R"
    candidates = [item for item in portals if item.side == side]
    if candidates:
        nearest = min(candidates, key=lambda item: abs(float(item.along) - along))
        # Rooms in this layout are wider than the corridor and extend several
        # metres beyond their doorway.  Keep the association conservative when
        # only a single portal has been observed, but do not split its interior.
        spacing = [
            abs(float(second.along) - float(first.along))
            for first, second in zip(candidates, candidates[1:])
        ]
        radius = max(8.0, 0.60 * (float(np.median(spacing)) if spacing else 8.0))
        if abs(float(nearest.along) - along) <= radius:
            return nearest.topology_id
    # An unassigned side region must remain isolated from the corridor.  It is
    # deliberately one temporary bucket until a stable doorway is observed.
    return "ROOM_{}_UNASSIGNED".format(side)


def region_of_point(
    point,
    gate_center,
    forward_yaw,
    corridor_half_width,
    portals,
    corridor_margin=0.15,
):
    """Which topology region a point belongs to: corridor, a room, or unknown.

    The room side reuses the same Voronoi band ``topology_region_mask`` builds
    (nearest confirmed doorway on that side of the corridor), so point
    membership and the coverage denominator can never disagree.  Room entry and
    exit then become a test on the robot's own pose - a state - instead of a
    plan-progress event that every replan can reset, and no doorway identity,
    evidence count or wall-offset proxy is needed for the room case.
    """
    dx = float(point[0]) - float(gate_center[0])
    dy = float(point[1]) - float(gate_center[1])
    cosine = math.cos(float(forward_yaw))
    sine = math.sin(float(forward_yaw))
    along = dx * cosine + dy * sine
    lateral = -dx * sine + dy * cosine
    if abs(lateral) <= float(corridor_half_width) + max(0.0, float(corridor_margin)):
        return "CORRIDOR"
    side = "L" if lateral > 0.0 else "R"
    best_id = None
    best_distance = None
    for portal in portals or ():
        if str(getattr(portal, "side", "")).upper()[:1] != side:
            continue
        distance = abs(along - float(portal.along))
        if best_distance is None or distance < best_distance:
            best_id = str(portal.topology_id)
            best_distance = distance
    if best_id is None:
        return "UNKNOWN"
    return best_id


def region_status(combined_coverage, target, active=False) -> str:
    """Single region state: ``COVERED`` / ``ACTIVE`` / ``UNSEEN``.

    One derived value replaces the four parallel completion bookkeeping sets
    (completed / retired / state=="COMPLETE" / completed front sides): a region
    is COVERED once its coverage target is met, ACTIVE while it is the one
    being worked, and UNSEEN otherwise.
    """
    try:
        value = float(combined_coverage)
    except (TypeError, ValueError):
        value = 0.0
    try:
        goal = float(target)
    except (TypeError, ValueError):
        goal = 1.0
    if value >= goal:
        return "COVERED"
    if active:
        return "ACTIVE"
    return "UNSEEN"


def floor_regions_complete(statuses, expected_rooms) -> bool:
    """True when every expected room region of the floor is COVERED."""
    values = [str(item) for item in (statuses or ())]
    expected = max(1, int(expected_rooms))
    if len(values) < expected:
        return False
    return all(item == "COVERED" for item in values)


def topology_region_mask(
    grid: GridView,
    gate_center: Tuple[float, float],
    forward_yaw: float,
    forward_limit: float,
    lateral_half_width: float,
    corridor_half_width: float,
    portals: "Sequence[RoomPortal]",
    topology_id: str,
) -> np.ndarray:
    """Return the map-derived side-room interval owned by one portal.

    Room intervals are Voronoi bands along one side of the corridor, bounded
    by the midpoints between adjacent *confirmed* portals.  This keeps the
    local completion denominator inside one room instead of allowing a door
    marker or a neighbouring room to make it appear complete.
    """
    result = np.zeros(grid.data.shape, dtype=bool)
    matching = [item for item in portals if item.topology_id == str(topology_id)]
    if not matching:
        return result
    portal = matching[0]
    same_side = sorted(
        (item for item in portals if item.side == portal.side),
        key=lambda item: float(item.along),
    )
    index = next(
        (index for index, item in enumerate(same_side)
         if item.topology_id == portal.topology_id),
        None,
    )
    if index is None:
        return result
    # Before the next station is observed, do not let one near doorway own the
    # entire unexplored side of the floor.  A conservative 15 m longitudinal
    # room span covers the generated room family without consulting Gazebo
    # metadata.  Once adjacent stations are available their midpoint replaces
    # this provisional bound automatically.
    provisional_half_span = 7.5
    lower = max(0.0, float(portal.along) - provisional_half_span) if index == 0 else 0.5 * (
        float(same_side[index - 1].along) + float(portal.along)
    )
    upper = min(
        float(forward_limit), float(portal.along) + provisional_half_span
    ) if index + 1 >= len(same_side) else 0.5 * (
        float(portal.along) + float(same_side[index + 1].along)
    )
    if upper <= lower:
        return result
    rows, columns = np.indices(grid.data.shape, dtype=np.float64)
    x = grid.origin_x + (columns + 0.5) * grid.resolution
    y = grid.origin_y + (rows + 0.5) * grid.resolution
    dx = x - float(gate_center[0])
    dy = y - float(gate_center[1])
    cosine, sine = math.cos(float(forward_yaw)), math.sin(float(forward_yaw))
    along = dx * cosine + dy * sine
    lateral = -dx * sine + dy * cosine
    # The navigation clearance test below already removes cells too close to
    # the wall.  Keeping an additional 0.35 m topological offset here can
    # erase a narrow doorway/room entirely, especially after SLAM wall drift.
    # Start just outside the corridor and let the occupancy-distance mask
    # decide which cells are physically usable.
    side_ok = lateral >= float(corridor_half_width) + 0.05 if portal.side == "L" else lateral <= -float(corridor_half_width) - 0.05
    result[:] = (
        (along >= lower)
        & (along <= upper)
        & (along <= float(forward_limit))
        & (np.abs(lateral) <= float(lateral_half_width))
        & side_ok
    )
    return result


def coverage_snapshot(
    grid: GridView,
    task_mask: np.ndarray,
    camera_seen: Optional[np.ndarray],
    robot_radius: float = 0.38,
    safety_margin: float = 0.04,
    occupied_threshold: int = 50,
    camera_weight: float = 0.65,
) -> Tuple[CoverageSnapshot, np.ndarray, np.ndarray]:
    """Measure coverage over potentially traversable task cells only."""
    data = np.asarray(grid.data)
    occupied = data >= int(occupied_threshold)
    obstacle_distance = distance_transform_edt(~occupied) * grid.resolution
    eligible = task_mask & (
        obstacle_distance >= float(robot_radius) + float(safety_margin)
    )
    known = data >= 0
    seen = np.zeros(data.shape, dtype=bool)
    if camera_seen is not None and np.asarray(camera_seen).shape == data.shape:
        seen = np.asarray(camera_seen, dtype=bool)
    denominator = max(1, int(np.count_nonzero(eligible)))
    laser_known = int(np.count_nonzero(eligible & known))
    camera_known = int(np.count_nonzero(eligible & seen))
    laser = float(laser_known) / float(denominator)
    camera = float(camera_known) / float(denominator)
    snapshot = CoverageSnapshot(
        laser=laser,
        camera=camera,
        combined=weighted_linear_coverage(laser, camera, camera_weight),
        task_cells=denominator,
        laser_known_cells=laser_known,
        camera_seen_cells=camera_known,
    )
    return snapshot, eligible, obstacle_distance


def coverage_classification(
    grid: GridView,
    task_mask: np.ndarray,
    camera_seen: Optional[np.ndarray],
    robot_radius: float = 0.38,
    safety_margin: float = 0.04,
    occupied_threshold: int = 50,
) -> np.ndarray:
    """Encode the two independent sensor layers for RViz diagnostics.

    Values are deliberately not fed back into planning: ``-1`` is outside
    the task envelope, ``0`` is an eligible cell seen by neither sensor,
    ``50`` is laser-only, ``75`` is camera-only, and ``100`` is seen by both.
    Cells inside the envelope but excluded from the coverage denominator
    (wall/obstacle band) use ``25`` so the distinction is visible.
    """
    data = np.asarray(grid.data)
    task = np.asarray(task_mask, dtype=bool)
    if task.shape != data.shape:
        raise ValueError("task_mask shape does not match grid")
    seen = np.zeros(data.shape, dtype=bool)
    if camera_seen is not None and np.asarray(camera_seen).shape == data.shape:
        seen = np.asarray(camera_seen, dtype=bool)
    known = data >= 0
    # Occupied cells are map evidence but never a sensor-coverage class.
    free = known & (data < occupied_threshold)
    laser = free
    _, eligible, _ = coverage_snapshot(
        grid,
        task,
        seen,
        robot_radius=robot_radius,
        safety_margin=safety_margin,
        occupied_threshold=occupied_threshold,
    )
    result = np.full(data.shape, -1, dtype=np.int8)
    result[task] = 0
    result[task & (data >= occupied_threshold)] = 25
    result[eligible & laser & ~seen] = 50
    result[eligible & ~laser & seen] = 75
    result[eligible & laser & seen] = 100
    return result


def doorways_match(candidate, reference, station_tolerance=2.5):
    """Whether two doorways are the same physical opening.

    Portal ids are 0.5 m longitudinal bins and are re-derived as SLAM refines a
    wall, so the id is not a stable identity.  Opposite sides stay distinct and
    real stations are separated by much more than ``station_tolerance``.
    """
    return bool(
        str(getattr(candidate, "topology_id", ""))
        == str(getattr(reference, "topology_id", ""))
        or (
            getattr(candidate, "side", None) == getattr(reference, "side", None)
            and abs(float(candidate.along) - float(reference.along))
            <= float(station_tolerance)
        )
    )


def admit_candidate(
    candidate,
    gate_center,
    forward_yaw,
    robot_pose,
    active_topologies=(),
    corridor_in_active_zone=False,
    room_status=None,
):
    """The single gate every candidate must pass.  Returns ``(admitted, reason)``.

    The three regressions found by hand this session (the robot walked down to
    the lobby, walked back to a finished front room, and toured the corridor,
    all with rooms unexplored) were each a *different* filter that a newly
    admitted candidate pool slipped past.  Rather than adding a fourth filter,
    every branch funnels through here, and the reason string is counted so the
    next leak is visible in the diagnostics instead of in RViz.

    Reasons, in check order:
      LOBBY            behind the entrance gate -- never a task region
      KIND             not an explorable frontier kind
      CORRIDOR_CAMERA  a camera viewpoint in the corridor (corridor is transit)
      BEHIND           behind the robot along the corridor axis
      ZONE             outside the active near/far zone
      ROOM_COVERED     belongs to a room already at its target
    """
    if candidate is None:
        return False, "NONE"
    if str(getattr(candidate, "kind", "")) not in VIEWPOINT_KINDS:
        return False, "KIND"
    point = getattr(candidate, "target", None)
    if point is None:
        return False, "KIND"
    cosine, sine = math.cos(float(forward_yaw)), math.sin(float(forward_yaw))
    along = (float(point[0]) - float(gate_center[0])) * cosine + (
        float(point[1]) - float(gate_center[1])
    ) * sine
    if along < -0.5:
        return False, "LOBBY"
    topology = str(getattr(candidate, "topology_id", "CORRIDOR"))
    if topology == "CORRIDOR" and candidate.kind == "CAMERA_FRONTIER":
        return False, "CORRIDOR_CAMERA"
    if robot_pose is not None:
        ahead = (float(point[0]) - float(robot_pose[0])) * cosine + (
            float(point[1]) - float(robot_pose[1])
        ) * sine
        if ahead < -0.5:
            return False, "BEHIND"
    if topology == "CORRIDOR":
        if not corridor_in_active_zone:
            return False, "ZONE"
    elif active_topologies and topology not in {
        str(item) for item in active_topologies
    }:
        return False, "ZONE"
    if room_status is not None and topology != "CORRIDOR":
        if str(room_status.get(topology, "")).upper() == "COVERED":
            return False, "ROOM_COVERED"
    return True, "ADMITTED"


VIEWPOINT_KINDS = ("LASER_FRONTIER", "CAMERA_FRONTIER", "SPHERE_REVIEW")


def crossing_zone(zone_now, robot_along, zone_split, margin=0.5):
    """Whether the robot still has to CROSS into the active (far) zone.

    User rule (2026-09-14): "allow the corridor to select camera frontier points,
    but only use them when crossing zones".  The corridor is normally transit and
    a corridor camera viewpoint is rejected outright (see the camera fallback);
    while the far zone is active and the robot is still in the near half, a
    corridor camera viewpoint beyond the split is exactly the pull that gets the
    robot to the far zone - so it is admitted only then.
    """
    if str(zone_now) != "B":
        return False
    return float(robot_along) < float(zone_split) - float(margin)


def leg_speed(
    cruise,
    elapsed,
    remaining,
    accel_seconds=1.5,
    decel_distance=1.0,
    ramp_floor_fraction=0.45,
    minimum_speed=0.20,
):
    """Speed for one path leg: ramp in from a stop, ramp out near the goal.

    User rule (2026-09-14): "do not hold the robot at its maximum speed the whole
    time - speed up gradually and slow down gradually; when planning/exploring
    toward a target point there should be a clear deceleration at the start and
    the end, but not so slow that it wastes efficiency".

    ``ramp_floor_fraction``/``minimum_speed`` keep the ramps from crawling: the
    speed never drops below ``max(minimum_speed, cruise * fraction)``, so the last
    metre and the first metre still move properly.  With ``remaining=0`` the leg
    returns the floor, and the arrival tolerance stops the robot before that.

    Pure so the offline suite can hold the shape; 0 for either ramp disables it
    (``accel_seconds<=0`` -> cruise immediately, ``decel_distance<=0`` -> no
    ramp-out), which is the pre-ramp behaviour for that side.
    """
    cruise = max(0.0, float(cruise))
    if cruise <= 0.0:
        return 0.0
    floor = min(cruise, max(float(minimum_speed), cruise * float(ramp_floor_fraction)))
    speed = cruise
    if accel_seconds > 0.0 and float(elapsed) < float(accel_seconds):
        fraction = max(0.0, float(elapsed)) / float(accel_seconds)
        speed = floor + (cruise - floor) * fraction
    if decel_distance > 0.0 and float(remaining) < float(decel_distance):
        fraction = max(0.0, float(remaining)) / float(decel_distance)
        speed = min(speed, floor + (cruise - floor) * fraction)
    return max(0.0, min(cruise, speed))


def room_lock_for_target(topology_id, retired=()):
    """Topology a dispatched room target must lock, or ``None`` for transit.

    Both ways of adopting a target - the ordinary plan dispatch and the
    mid-route handover in ``_switch_active_target`` - must arm the room lock
    identically.  The handover path used to skip it, which left
    ``locked_topology`` at ``None`` after a handover: ``target_switch_allowed``
    then had no room constraint at all and the explorer alternated between the
    two front rooms every ~10 sim seconds while never entering either.  Measured
    on run107 floor 0: 13 handovers in 196 sim seconds, ``along`` oscillating
    between 4.1 and 8.9 m and ``lateral`` between +0.4 and -3.7 m.

    Extracted as a pure function so the shared invariant is testable in the
    ordinary core suite instead of living only in the node.
    """
    if topology_id is None:
        return None
    topology_id = str(topology_id)
    if topology_id == "CORRIDOR" or "UNASSIGNED" in topology_id:
        return None
    if topology_id in {str(item) for item in (retired or ())}:
        return None
    return topology_id


def target_switch_allowed(
    active,
    candidate,
    dwell_elapsed,
    locked_topology=None,
    dwell_seconds=8.0,
    observed_ratio=0.25,
    active_remaining=None,
):
    """Whether the target being driven may be replaced mid-route.

    The trigger is the current viewpoint's OWN obsolescence, not a comparison
    between two candidates (user requirement): while walking, if the target
    viewpoint has become clearly observed -- its remaining information has
    collapsed -- the planner may hand over to whatever it now prefers.  A pure
    score comparison is what made the earlier mechanism oscillate, and the
    guards below still forbid the obvious instabilities:

    * the current target has been held for ``dwell_seconds``;
    * the successor is room-owned (the corridor is transit) and belongs to the
      locked room when a lock is held;
    * the successor is not the same point.

    ``active_remaining`` is the fraction of the active viewpoint's original
    information that is still unobserved (``None`` means "cannot tell", which
    refuses the switch).
    """
    if active is None or candidate is None:
        return False
    if float(dwell_elapsed) < float(dwell_seconds):
        return False
    if candidate.kind not in VIEWPOINT_KINDS or active.kind not in VIEWPOINT_KINDS:
        return False
    if str(candidate.topology_id) == "CORRIDOR":
        return False
    if locked_topology is not None and str(candidate.topology_id) != str(locked_topology):
        return False
    if (
        abs(float(candidate.target[0]) - float(active.target[0])) < 1e-6
        and abs(float(candidate.target[1]) - float(active.target[1])) < 1e-6
    ):
        return False
    if active_remaining is None:
        return False
    return float(active_remaining) <= float(observed_ratio)


def front_stations_complete(
    front_station_portals,
    completed,
    completed_portals,
    completed_front_sides=(),
    station_tolerance=2.5,
):
    """Whether the first room pair (both front sides) has been explored.

    run46 floor 2 dead-locked here because this test matched ids only: the pair
    was explored and completed as ROOM_L_10/ROOM_R_10 while the front station
    identity had drifted to ROOM_L_15/ROOM_R_15.  ``completed_sides`` stayed
    empty, ``front_rooms_complete`` stayed False, every newly generated frontier
    was rejected as ``wrong_topology`` (129 rejections) and the planner reported
    NO_FRONTIER while the robot stood still in the corridor.
    """
    front_sides = {
        str(getattr(portal, "side", "")) for portal in front_station_portals
    }
    front_sides.discard("")
    completed = completed or ()
    completed_portals = tuple(completed_portals or ())
    completed_sides = {
        str(side) for side in (completed_front_sides or ()) if str(side) in ("L", "R")
    }
    for portal in front_station_portals:
        if getattr(portal, "topology_id", None) in completed:
            completed_sides.add(portal.side)
        elif any(
            doorways_match(portal, old, station_tolerance) for old in completed_portals
        ):
            completed_sides.add(portal.side)
    return bool(front_sides == {"L", "R"} and completed_sides == {"L", "R"})


def unsafe_path_replan_needed(
    target_active, active_safe, unsafe_cycles, replan_cycles
):
    """Whether a path judged unsafe should be abandoned for another target.

    One unsafe reading must not churn the target.  run47 floor 0 room 4 kept a
    robot rotating on the spot at a doorway while two targets of the same room
    alternated every ~5.5 s (combined gains 5.36 and 3.12, neither marked
    ``(replaced)``): each rotation changed the map, the remaining active path
    was re-judged unsafe and the planner returned the room's other target, which
    pointed the opposite way.  Requiring the condition to persist for a few
    planning cycles preserves the doorway commitment so the controller can
    drive through; a genuine blockage is still handled by the collision stop.
    """
    if not target_active or active_safe:
        return False
    return int(unsafe_cycles) >= max(1, int(replan_cycles))


def clear_doorway_speckle(
    data, mask, max_component=2, occupied_value=50, dilation=1
):
    """Free isolated occupied blobs that sit *entirely* inside a doorway span.

    The occupancy map never converts an occupied cell back to free (rays only
    clear *unknown* cells), so a single stray endpoint on the door line seals a
    real doorway for the rest of the run - measured as verified door bands of 0
    cells while the opening was physically clear.  Inside a doorway whose span
    the portal detector has already measured, a one- or two-cell blob cannot be
    a wall, so it is removed.

    Two details matter and both were learned the hard way:

    * components are labelled in the mask *dilated* by one cell, so a wall that
      crosses the doorway boundary is still connected to the rest of the wall
      and is never mistaken for an isolated blob; and
    * a component is freed only when **every** cell lies inside the mask, so a
      small blob just outside the doorway is left alone.

    Everything larger, and everything outside the doorway span, is untouched:
    real walls and real obstacles still block.  Scoping to doorways (instead of
    despeckling the whole map) preserves the coverage-plan semantics that the
    run30 regression showed must not change.  Returns the number of cells freed;
    ``data`` is modified in place.
    """
    import numpy as np
    from scipy.ndimage import binary_dilation, label

    values = np.asarray(data)
    mask = np.asarray(mask, dtype=bool)
    if not mask.any():
        return 0
    region = binary_dilation(mask, iterations=max(0, int(dilation)))
    occupied = (values >= int(occupied_value)) & region
    if not occupied.any():
        return 0
    components, count = label(occupied, np.ones((3, 3), dtype=np.int8))
    if count <= 0:
        return 0
    freed = 0
    for index in range(1, count + 1):
        cells = components == index
        size = int(np.count_nonzero(cells))
        if size > int(max_component):
            continue
        if not bool(np.all(mask[cells])):
            continue
        values[cells] = 0
        freed += size
    return freed


class TaskCoveragePlanner:
    """Nearest-safe-frontier planner with camera-biased local information gain."""

    CARDINALS = ((-1, 0), (0, -1), (0, 1), (1, 0))
    DIRECTIONS = (
        (-1, 0),
        (-1, 1),
        (0, 1),
        (1, 1),
        (1, 0),
        (1, -1),
        (0, -1),
        (-1, -1),
    )

    def __init__(
        self,
        robot_radius: float = 0.38,
        safety_margin: float = 0.04,
        frontier_cluster_radius: float = 0.45,
        revisit_radius: float = 0.70,
        information_radius: float = 2.5,
        camera_weight: float = 0.65,
        gain_camera_weight: Optional[float] = None,
        target_rank_mode: str = "gain_efficiency",
        back_extension: float = 3.6,
        forward_depth: float = 25.0,
        lateral_half_width: float = 9.5,
        corridor_half_width: float = 1.1,
        navigation_clearance: float = 0.30,
        preferred_clearance: float = 0.42,
        clearance_cost_weight: float = 1.4,
        turn_cost_weight: float = 0.10,
        far_room_first: bool = False,
        minimum_room_stations: int = 2,
        front_station_search_limit: float = 35.0,
        virtual_gate_half_width: Optional[float] = None,
        virtual_gate_depth: float = 0.30,
        min_room_interior_cells: int = 800,
        portal_merge_radius: float = 2.5,
    ):
        self.robot_radius = float(robot_radius)
        self.safety_margin = float(safety_margin)
        self.frontier_cluster_radius = max(0.15, float(frontier_cluster_radius))
        # Bounded wider along search used only when the in-doorway crossing
        # search fails (see door_crossing_along_offsets); the offset that
        # succeeded is reported so the next run can prove the drift.
        self.door_crossing_wide_search = 2.5
        # Below this combined gain a corridor move is treated as
        # nearly worthless, so an observation episode is preferred.
        self.observation_weak_gain = 1.0
        # Doorway-scoped speckle removal: freeing a mis-detected obstacle inside
        # a confirmed doorway is required, because the map never frees an
        # occupied cell otherwise.
        self.door_speckle_max_component = 2
        self.door_mask_margin = 0.35
        self.door_mask_depth = 1.20
        self.door_crossing_scan_step = 0.5
        self.last_door_crossing_offset = None
        self.revisit_radius = max(0.2, float(revisit_radius))
        self.information_radius = max(0.5, float(information_radius))
        self.camera_weight = max(0.0, min(1.0, float(camera_weight)))
        # Gain and coverage are separate objectives and must not share one
        # weight.  ``camera_weight`` answers "how much of the floor has been
        # observed", which is the completion criterion and stays as validated.
        # Target *selection* is a different question: which viewpoint is worth
        # the travel.  Raising the laser share there makes the planner prefer
        # the camera viewpoints that also extend the map and the corridor
        # geometry, which is what later lets the camera be driven to somewhere
        # it can actually see a red source.  The selected kind is still
        # CAMERA_FRONTIER; only its ranking changes.
        if gain_camera_weight is None:
            self.gain_camera_weight = self.camera_weight
        else:
            self.gain_camera_weight = max(0.0, min(1.0, float(gain_camera_weight)))
        rank_mode = str(target_rank_mode or "gain_efficiency").strip().lower()
        self.target_rank_mode = (
            rank_mode if rank_mode in ("gain_efficiency", "nearest") else "gain_efficiency"
        )
        self.back_extension = max(0.0, float(back_extension))
        self.forward_depth = max(1.0, float(forward_depth))
        self.lateral_half_width = max(1.0, float(lateral_half_width))
        self.corridor_half_width = max(0.2, float(corridor_half_width))
        # A doorway candidate is only actionable when a usable room interior
        # exists behind it.  The virtual corridor-entrance gate is anchored off
        # the corridor axis, so the wall gaps around it are frequently promoted
        # to false doorways; a real room leaves thousands of eligible cells,
        # while a seam leaves a few hundred (run17: real rooms 4251-8712,
        # seams 185-730).  0 disables the guard.
        self.min_room_interior_cells = max(0, int(min_room_interior_cells))
        # One physical doorway is reported as several 0.5 m longitudinal bins,
        # and every bin is a separate candidate with its own lock id, owner
        # label and approach geometry.  That ambiguity is what let the robot
        # re-approach the same room through a slightly different candidate.
        # Collapse same-side portals whose along values are within this radius
        # (the same tolerance the completion test already uses) so a doorway
        # enters the planner exactly once.  The comparison is along the corridor
        # axis and per side, NOT plain Cartesian distance: the left and right
        # doorways of a pair sit only about 2.2 m apart across the corridor and
        # are genuinely different rooms.  0 disables the merge.
        self.portal_merge_radius = max(0.0, float(portal_merge_radius))
        # Coverage excludes cells close to walls using robot_radius +
        # safety_margin.  Navigation is intentionally separate: the A1
        # footprint is 0.36 m wide plus 0.02 m padding on either side, so a
        # forward doorway crossing needs 0.20 m lateral clearance, not the
        # footprint's roughly 0.40 m circumscribed radius.
        self.navigation_clearance = max(0.05, float(navigation_clearance))
        self.preferred_clearance = max(
            self.navigation_clearance, float(preferred_clearance)
        )
        self.clearance_cost_weight = max(0.0, float(clearance_cost_weight))
        self.turn_cost_weight = max(0.0, float(turn_cost_weight))
        self.far_room_first = bool(far_room_first)
        self.minimum_room_stations = max(1, int(minimum_room_stations))
        self.front_station_search_limit = max(
            3.0, float(front_station_search_limit)
        )
        if virtual_gate_half_width is None:
            virtual_gate_half_width = self.corridor_half_width
        self.virtual_gate_half_width = max(
            self.robot_radius + self.navigation_clearance,
            min(self.corridor_half_width, float(virtual_gate_half_width)),
        )
        self.virtual_gate_depth = max(0.0, float(virtual_gate_depth))
        # Diagnostic: how often the A* backwards walk had to be abandoned
        # because ``predecessor`` had closed a cycle (see ``_astar_cells``).
        self.predecessor_cycle_breaks = 0

    @staticmethod
    def _in_bounds(shape, row, column):
        return 0 <= row < shape[0] and 0 <= column < shape[1]

    def _nearest_seed(self, grid, safe, pose):
        rows, columns = np.nonzero(safe)
        if len(rows) == 0:
            return None
        x = grid.origin_x + (columns + 0.5) * grid.resolution
        y = grid.origin_y + (rows + 0.5) * grid.resolution
        index = int(np.argmin((x - pose[0]) ** 2 + (y - pose[1]) ** 2))
        return int(rows[index]), int(columns[index])

    def _robot_local_safe(self, grid, safe, pose):
        """``safe`` plus the 3x3 window the robot physically occupies.

        The robot is standing there, so those cells *are* traversable; the
        clearance-inflated ``safe`` can still exclude them (its own leg returns,
        a door frame, or a cell the map has not observed yet).  There is no
        "nearest safe cell" fallback any more: the route always starts in the
        robot's own cell, which is what the user requires.  Bounded to one cell
        around the robot, so this can never open a shortcut through a wall.

        Returns ``(mask, start_cell)``; ``(None, None)`` when the pose is
        outside the map.
        """
        data = np.asarray(grid.data)
        if pose is None:
            return None, None
        cell = grid.world_to_cell(float(pose[0]), float(pose[1]))
        if not self._in_bounds(data.shape, *cell):
            return None, None
        row, column = int(cell[0]), int(cell[1])
        window = np.asarray(data[row - 1: row + 2, column - 1: column + 2]) < 50
        # The robot's own cell is trusted unconditionally: it is standing in it.
        if window.size == 9:
            window[1, 1] = True
        local = safe[row - 1: row + 2, column - 1: column + 2]
        if not np.any(window & ~local):
            return safe, (row, column)
        mask = safe.copy()
        mask[row - 1: row + 2, column - 1: column + 2] |= window
        return mask, (row, column)

    def _reachable(self, safe, seed):
        distance = np.full(safe.shape, -1, dtype=np.int32)
        predecessor = np.full(safe.shape + (2,), -1, dtype=np.int32)
        distance[seed] = 0
        queue = deque([seed])
        while queue:
            row, column = queue.popleft()
            for d_row, d_column in self.CARDINALS:
                next_row, next_column = row + d_row, column + d_column
                if not self._in_bounds(safe.shape, next_row, next_column):
                    continue
                if not safe[next_row, next_column] or distance[next_row, next_column] >= 0:
                    continue
                distance[next_row, next_column] = distance[row, column] + 1
                predecessor[next_row, next_column] = (row, column)
                queue.append((next_row, next_column))
        return distance, predecessor

    def _navigation_fields(self, grid, task_mask=None, despeckle=False):
        data = np.asarray(grid.data)
        occupied = data >= 50
        # FAST-LIO endpoints are accumulated permanently by the lightweight
        # occupancy node, so a single-frame return leaves isolated one- and
        # two-cell occupied speckle behind.  That speckle is harmless on its
        # own, but inflating it by the navigation clearance closes a passage
        # the robot can physically drive through, and A* then reports "no
        # route" for a corridor the robot has already traversed.  Measured on
        # run29 floor 0: the rear-corridor pocket reachable from the robot's
        # pose held 28732 cells at the node's 0.30 m clearance with the
        # elevator staging point (5.75, -0.35) unreachable; removing 349 cells
        # of <=2-cell speckle (5.7% of the occupied set) reconnected it to
        # 47458 cells with the staging point reachable again.  Real obstacles
        # are larger than two cells and are left untouched.  This is the same
        # technique, with the same justification, that
        # ``navigation_path_from_room_through_portal`` already applies to the
        # door-normal crossing.
        #
        # It is deliberately opt-in and applied to *routing* only.  Turning it
        # on for the coverage plan's own reachability set changed the explorer
        # as well: run30 then held ROOM_L_43 for 297 sim seconds spinning in
        # place at its doorway (position pinned inside a 0.5 x 1.5 m box,
        # cmd_vx == 0 in 87% of samples, no "doorway crossing confirmed"),
        # where run29 with the identical code minus this flag had finished the
        # same room in 62 s.  Routing must see through the speckle; the
        # frontier/eligibility semantics that decide what to explore must not.
        if despeckle and occupied.any():
            components, _count = label(occupied, np.ones((3, 3), dtype=np.int8))
            sizes = np.bincount(components.reshape(-1))
            occupied = occupied & (sizes[components] > 2)
        clearance = distance_transform_edt(~occupied) * grid.resolution
        safe = (
            (data >= 0)
            & (data < 50)
            & (clearance >= self.navigation_clearance)
        )
        if task_mask is not None and np.asarray(task_mask).shape == safe.shape:
            safe &= np.asarray(task_mask, dtype=bool)
        return safe, clearance

    def verified_door_band(
        self, grid, gate_center, forward_yaw, portals, clearance,
        portal_clearance=0.12,
    ):
        """Cells of an already-confirmed doorway that are physically crossable.

        ``safe`` requires ``clearance >= navigation_clearance`` (0.24 m), but
        FAST-LIO endpoints are accumulated permanently, so a single stray
        occupied pixel on the wall line seals a doorway once it is inflated and
        cuts its room out of the reachable set.  Measured on run27 floor 0: the
        front-right room held 11869 task cells of which only 260 stayed
        reachable, ``room_reachable_camera_unseen_cells`` was 0 while 7654
        cells were unseen, and the planner could therefore never emit a target
        that would take the robot through the door.

        This marks only the straight door-normal band at the *cached* door
        centre of a portal the node has already confirmed, and only when every
        cell of that band is known-free and its un-inflated clearance still
        exceeds the same verified-crossing threshold the accepted return path
        uses (``navigation_path_from_room_through_portal``).  It is not a
        generic wall-gap fallback: unknown cells, walls and real obstacles all
        still refuse the band.
        """
        data = np.asarray(grid.data)
        occupied = data >= 50
        band = np.zeros(data.shape, dtype=bool)
        for portal in portals:
            sign = 1.0 if portal.side == "L" else -1.0
            corridor_stage = self._portal_waypoint(
                gate_center, forward_yaw, float(portal.along), 0.0
            )
            for along_offset in portal_return_along_offsets(portal.width):
                room_stage = self._portal_waypoint(
                    gate_center,
                    forward_yaw,
                    float(portal.along) + along_offset,
                    sign * (self.corridor_half_width + 1.2),
                )
                length = math.hypot(
                    room_stage[0] - corridor_stage[0],
                    room_stage[1] - corridor_stage[1],
                )
                count = max(1, int(math.ceil(length / max(grid.resolution, 0.05))))
                sample_cells = []
                for index in range(count + 1):
                    point = (
                        corridor_stage[0]
                        + (room_stage[0] - corridor_stage[0]) * index / count,
                        corridor_stage[1]
                        + (room_stage[1] - corridor_stage[1]) * index / count,
                    )
                    sample_cells.append(grid.world_to_cell(*point))
                # Walk a 4-connected digital line through the samples.  Marking
                # only the sample cells leaves single-cell gaps (the run27 test
                # grid skipped exactly one row at y = 1.25), and a one-cell gap
                # is enough to break the cardinal flood fill that produces
                # ``reachable``, which would silently defeat the whole band.
                cells = []
                for index, start in enumerate(sample_cells):
                    cells.append(start)
                    if index + 1 >= len(sample_cells):
                        break
                    current = start
                    goal = sample_cells[index + 1]
                    while current != goal:
                        delta_row = goal[0] - current[0]
                        delta_column = goal[1] - current[1]
                        if abs(delta_row) >= abs(delta_column):
                            current = (
                                current[0] + (1 if delta_row > 0 else -1),
                                current[1],
                            )
                        else:
                            current = (
                                current[0],
                                current[1] + (1 if delta_column > 0 else -1),
                            )
                        cells.append(current)
                if any(
                    not self._in_bounds(data.shape, *cell) for cell in cells
                ):
                    continue
                if any(
                    data[cell] < 0 or occupied[cell] or clearance[cell] < portal_clearance
                    for cell in cells
                ):
                    continue
                for cell in cells:
                    band[cell] = True
        return band

    @staticmethod
    def _direction_index(yaw):
        values = [math.atan2(d_row, d_column) for d_row, d_column in TaskCoveragePlanner.DIRECTIONS]
        return min(
            range(len(values)),
            key=lambda index: abs(normalize_angle(float(yaw) - values[index])),
        )

    @staticmethod
    def _turn_angle(first, second):
        steps = abs(int(first) - int(second))
        return min(steps, 8 - steps) * (math.pi / 4.0)

    def _astar_cells(self, grid, safe, clearance, start, goal, initial_yaw):
        """Orientation-aware 8-neighbour A* over the hard-safe grid."""
        if not safe[start] or not safe[goal]:
            return ()
        initial_direction = self._direction_index(initial_yaw)
        initial_state = (int(start[0]), int(start[1]), initial_direction)
        queue = [(0.0, 0.0, initial_state)]
        costs = {initial_state: 0.0}
        predecessor = {}
        closed = set()
        goal_state = None
        while queue:
            _, cost, state = heapq.heappop(queue)
            if state in closed or cost > costs.get(state, float("inf")) + 1e-9:
                continue
            closed.add(state)
            row, column, direction = state
            if (row, column) == goal:
                goal_state = state
                break
            for next_direction, (d_row, d_column) in enumerate(self.DIRECTIONS):
                next_row, next_column = row + d_row, column + d_column
                if not self._in_bounds(safe.shape, next_row, next_column):
                    continue
                if not safe[next_row, next_column]:
                    continue
                # Do not cut diagonally between two occupied/inflated cells.
                if d_row and d_column and not (
                    safe[row, next_column] and safe[next_row, column]
                ):
                    continue
                step = grid.resolution * math.hypot(d_row, d_column)
                clearance_deficit = max(
                    0.0,
                    self.preferred_clearance - float(clearance[next_row, next_column]),
                ) / self.preferred_clearance
                transition = step * (
                    1.0 + self.clearance_cost_weight * clearance_deficit
                )
                transition += self.turn_cost_weight * self._turn_angle(
                    direction, next_direction
                )
                next_cost = cost + transition
                next_state = (next_row, next_column, next_direction)
                if next_cost + 1e-9 >= costs.get(next_state, float("inf")):
                    continue
                costs[next_state] = next_cost
                predecessor[next_state] = state
                heuristic = grid.resolution * math.hypot(
                    goal[0] - next_row, goal[1] - next_column
                )
                heapq.heappush(
                    queue,
                    (next_cost + heuristic, next_cost, next_state),
                )
        if goal_state is None:
            return ()
        return self._reconstruct_path(predecessor, initial_state, goal_state)

    def _reconstruct_path(self, predecessor, initial_state, goal_state):
        """Walk ``predecessor`` back from goal to start, cycle-safe.

        ``predecessor`` is NOT guaranteed to be acyclic, because the search
        re-opens states (a cheaper route may be found after a state was already
        expanded) and overwrites their predecessor when it does.  If X was first
        routed through Y and Y is later routed through X, an unguarded walk never
        reaches ``initial_state``: it spins for ever at ~100 % CPU and the
        planning thread never publishes again.  Measured on run146: from sim ~245
        the planning thread was the only running thread in the node (277 s of CPU
        against 359 s for the whole process) and the robot sat still at the
        ROOM_*_49 doorway for the rest of the run while the control loop kept
        reporting a stale ``plan_target``.  Abandon the walk on the first
        repeated state: an unroutable answer is recoverable (the candidate is
        skipped and another target is chosen), a hung planner is not.
        """
        states = [goal_state]
        seen_states = {goal_state}
        while states[-1] != initial_state:
            previous = predecessor.get(states[-1])
            if previous is None or previous in seen_states:
                self.predecessor_cycle_breaks += 1
                return ()
            seen_states.add(previous)
            states.append(previous)
        states.reverse()
        return tuple((state[0], state[1]) for state in states)

    def _line_is_safe(self, safe, first, second):
        span = max(abs(second[0] - first[0]), abs(second[1] - first[1]))
        if span <= 0:
            return bool(safe[first])
        # Half-cell sampling is conservative enough to catch wall corners and
        # produces the line-of-sight shortcut expected in open areas.
        count = max(1, int(math.ceil(2.0 * span)))
        previous = first
        for index in range(1, count + 1):
            fraction = float(index) / float(count)
            cell = (
                int(round(first[0] + (second[0] - first[0]) * fraction)),
                int(round(first[1] + (second[1] - first[1]) * fraction)),
            )
            if not self._in_bounds(safe.shape, *cell) or not safe[cell]:
                return False
            if cell[0] != previous[0] and cell[1] != previous[1]:
                if not (safe[previous[0], cell[1]] and safe[cell[0], previous[1]]):
                    return False
            previous = cell
        return True

    def _shortcut_cells(self, safe, cells):
        if len(cells) <= 2:
            return tuple(cells)
        result = [cells[0]]
        anchor = 0
        while anchor < len(cells) - 1:
            candidate = len(cells) - 1
            while candidate > anchor + 1 and not self._line_is_safe(
                safe, cells[anchor], cells[candidate]
            ):
                candidate -= 1
            result.append(cells[candidate])
            anchor = candidate
        return tuple(result)

    def _resample_world_path(self, grid, cells, maximum_spacing=0.40):
        if not cells:
            return ()
        corners = [grid.cell_center(*cell) for cell in cells]
        result = [corners[0]]
        for start, finish in zip(corners, corners[1:]):
            length = math.hypot(finish[0] - start[0], finish[1] - start[1])
            count = max(1, int(math.ceil(length / float(maximum_spacing))))
            result.extend(
                (
                    start[0] + (finish[0] - start[0]) * index / count,
                    start[1] + (finish[1] - start[1]) * index / count,
                )
                for index in range(1, count + 1)
            )
        return tuple(result)

    def navigation_path(self, grid, robot_pose, target, task_mask=None):
        """Plan the executable A* path independently of coverage eligibility."""
        if grid is None or robot_pose is None or target is None:
            return (), 0.0, 0.0
        # Routing sees through single-frame endpoint speckle, which the
        # coverage plan's own reachability set deliberately does not (see
        # _navigation_fields).
        safe, clearance = self._navigation_fields(grid, task_mask, despeckle=True)
        # No "nearest safe cell": the route starts in the cell the robot is
        # standing in, plus the 3x3 window it physically occupies.
        safe, start = self._robot_local_safe(grid, safe, robot_pose)
        if start is None:
            return (), 0.0, 0.0
        goal = grid.world_to_cell(float(target[0]), float(target[1]))
        if not self._in_bounds(safe.shape, *goal) or not safe[goal]:
            return (), 0.0, 0.0
        cells = self._astar_cells(
            grid, safe, clearance, start, goal, float(robot_pose[2])
        )
        if not cells:
            return (), 0.0, 0.0
        cells = self._shortcut_cells(safe, cells)
        path = self._resample_world_path(grid, cells)
        length = sum(
            math.hypot(second[0] - first[0], second[1] - first[1])
            for first, second in zip(path, path[1:])
        )
        # The robot's own cell is an unavoidable part of the route, not a
        # property of it: reporting its (possibly sub-threshold) clearance as
        # the route minimum made every target chosen while hugging a wall look
        # unsafe.  Judge the route from the first cell the robot has to drive
        # to, matching ``path_is_safe``, which skips the seed via ``path_index``.
        measured = path
        if len(path) > 1 and float(
            clearance[grid.world_to_cell(*path[0])]
        ) < self.navigation_clearance:
            measured = path[1:]
        minimum = min(float(clearance[grid.world_to_cell(*point)]) for point in measured)
        return path, length, minimum

    def navigation_path_via(self, grid, robot_pose, waypoints, task_mask=None):
        """Plan a sequence of A* legs without shortcutting across a turn gate.

        Door traversal needs an explicit corridor-centre staging point followed
        by a straight wall crossing.  Running the line-of-sight shortcut once
        over the whole route can replace that manoeuvre with a diagonal whose
        point clearance is valid but whose rotating A1 footprint hits a jamb.
        """
        current = (float(robot_pose[0]), float(robot_pose[1]), float(robot_pose[2]))
        result = []
        total_length = 0.0
        minimum_clearance = float("inf")
        for waypoint in waypoints:
            waypoint = (float(waypoint[0]), float(waypoint[1]))
            if math.hypot(waypoint[0] - current[0], waypoint[1] - current[1]) < 0.12:
                continue
            path, length, clearance = self.navigation_path(
                grid, current, waypoint, task_mask
            )
            if not path:
                return (), 0.0, 0.0
            if result and path[0] == result[-1]:
                path = path[1:]
            result.extend(path)
            total_length += length
            minimum_clearance = min(minimum_clearance, clearance)
            if len(result) >= 2:
                yaw = math.atan2(
                    result[-1][1] - result[-2][1],
                    result[-1][0] - result[-2][0],
                )
            else:
                yaw = current[2]
            current = (waypoint[0], waypoint[1], yaw)
        return (
            tuple(result),
            total_length,
            0.0 if not math.isfinite(minimum_clearance) else minimum_clearance,
        )

    def navigation_path_from_room_through_portal(
        self,
        grid,
        robot_pose,
        room_stage,
        corridor_stage,
        portal_clearance=0.12,
    ):
        """Use normal A* in the room, then a verified straight door crossing.

        This is only for returning through a portal the robot already crossed
        on entry.  Mapping speckle may reduce the inferred clearance at the
        wall line below the global 0.20 m navigation threshold even though
        the cached opening is physically traversable.
        """
        room_path, room_length, room_minimum = self.navigation_path(
            grid, robot_pose, room_stage, None
        )
        if not room_path:
            return (), 0.0, 0.0
        data = np.asarray(grid.data)
        occupied = data >= 50
        # FAST-LIO endpoints are accumulated permanently by the lightweight
        # occupancy node.  A one-frame return can therefore leave a single
        # occupied pixel across a doorway that this topology has already
        # traversed.  Ignore only isolated 1-2 cell components for the
        # verified door-normal segment; continuous walls and real obstacles
        # remain occupied, and room A* still uses the untouched map.
        components, _count = label(occupied, np.ones((3, 3), dtype=np.int8))
        component_sizes = np.bincount(components.reshape(-1))
        isolated = occupied & (component_sizes[components] <= 2)
        crossing_occupied = occupied & ~isolated
        clearance = distance_transform_edt(~crossing_occupied) * grid.resolution
        crossing_length = math.hypot(
            corridor_stage[0] - room_stage[0], corridor_stage[1] - room_stage[1]
        )
        count = max(1, int(math.ceil(crossing_length / max(grid.resolution, 0.05))))
        crossing = tuple(
            (
                room_stage[0] + (corridor_stage[0] - room_stage[0]) * index / count,
                room_stage[1] + (corridor_stage[1] - room_stage[1]) * index / count,
            )
            for index in range(1, count + 1)
        )
        crossing_minimum = float("inf")
        for point in crossing:
            cell = grid.world_to_cell(*point)
            if not self._in_bounds(data.shape, *cell):
                return (), 0.0, 0.0
            if data[cell] < 0 or crossing_occupied[cell]:
                return (), 0.0, 0.0
            crossing_minimum = min(crossing_minimum, float(clearance[cell]))
        if crossing_minimum < float(portal_clearance):
            return (), 0.0, 0.0
        path = list(room_path)
        if math.hypot(path[-1][0] - room_stage[0], path[-1][1] - room_stage[1]) > 0.03:
            path.append(tuple(room_stage))
        path.extend(crossing)
        minimum = min(room_minimum, crossing_minimum)
        return tuple(path), room_length + crossing_length, minimum

    def navigation_path_through_portal(
        self,
        grid,
        robot_pose,
        gate_center,
        forward_yaw,
        portal,
        target,
        task_mask=None,
        wide_search=None,
    ):
        """Build a staged door path using the deepest known entry point.

        The door centre remains mandatory, so the robot cannot shortcut a
        corner or enter sideways.  Only the room-side staging depth adapts:
        the conservative navigation map can lag the exploration map by one
        cell at the sensor horizon, making a fixed 0.80 m point unknown even
        though a slightly shallower crossing is safe.
        """
        sign = 1.0 if portal.side == "L" else -1.0
        # A single noisy occupied cell can permanently survive in the raw LIO
        # occupancy map.  Do not make the entire confirmed doorway depend on
        # one exact corridor staging cell: shift the complete door-normal
        # crossing together, bounded strictly by the observed doorway width.
        # The widened scan is a bounded fallback for a portal bin that drifted
        # off the real opening.  It is always available because the drift also
        # hits corridor-owned candidates: run50 floor 0 reported NO_FRONTIER in
        # the corridor with four actionable portals and one remaining candidate
        # (ROOM_R_32) that failed as path_unreachable with a 0-cell verified door
        # band.  The in-doorway offsets are always tried first and the loop
        # stops at the first success, so a healthy doorway costs nothing extra.
        search_wide = (
            self.door_crossing_wide_search
            if wide_search is None
            else max(0.0, float(wide_search))
        )
        for along_offset in door_crossing_along_offsets(
            portal.width,
            search_wide,
            self.door_crossing_scan_step,
        ):
            shifted_along = float(portal.along) + along_offset
            # No corridor-axis staging waypoint.  Sending the robot to the
            # corridor centreline before the door meant that every replan -
            # and the room target is replanned every ``replan_period`` - turned
            # a robot that was already inside the room back out through the
            # doorway and in again ("enter, leave, re-enter").  A* straight to
            # the door centre from wherever the robot stands keeps the crossing
            # monotone; the door centre itself remains mandatory.
            door_centre = self._portal_waypoint(
                gate_center, forward_yaw, shifted_along, portal.lateral
            )
            for depth in (0.80, 0.70, 0.60, 0.50, 0.40, 0.30, 0.20):
                room_entry = self._portal_waypoint(
                    gate_center,
                    forward_yaw,
                    shifted_along,
                    sign * (self.corridor_half_width + depth),
                )
                path, length, minimum = self.navigation_path_via(
                    grid,
                    robot_pose,
                    (door_centre, room_entry, target),
                    task_mask,
                )
                if path:
                    self.last_door_crossing_offset = float(along_offset)
                    return path, length, minimum, depth
        return (), 0.0, 0.0, None

    @staticmethod
    def _portal_waypoint(
        gate_center, forward_yaw, along, lateral
    ) -> Tuple[float, float]:
        cosine, sine = math.cos(float(forward_yaw)), math.sin(float(forward_yaw))
        return (
            float(gate_center[0]) + cosine * float(along) - sine * float(lateral),
            float(gate_center[1]) + sine * float(along) + cosine * float(lateral),
        )

    def _clusters(self, mask, grid):
        pending = set(zip(*np.nonzero(mask)))
        radius = max(1, int(math.ceil(self.frontier_cluster_radius / grid.resolution)))
        clusters = []
        while pending:
            seed = pending.pop()
            cluster = [seed]
            queue = deque([seed])
            while queue:
                row, column = queue.popleft()
                neighbours = []
                for next_row in range(row - radius, row + radius + 1):
                    for next_column in range(column - radius, column + radius + 1):
                        item = (next_row, next_column)
                        if item in pending:
                            neighbours.append(item)
                for item in neighbours:
                    pending.remove(item)
                    cluster.append(item)
                    queue.append(item)
            clusters.append(cluster)
        return clusters

    def _path(self, grid, predecessor, seed, target):
        cells = []
        current = target
        while current != seed and current[0] >= 0:
            cells.append(current)
            previous = predecessor[current]
            if previous[0] < 0:
                break
            current = (int(previous[0]), int(previous[1]))
        cells.append(seed)
        cells.reverse()
        # The reachable predecessor chain is optimized for discovery, not
        # for a smooth vehicle trajectory.  Collapse every safe line-of-sight
        # run before exposing the path to the controller.
        occupied = np.asarray(grid.data) >= 50
        clearance = distance_transform_edt(~occupied) * grid.resolution
        safe = (
            (np.asarray(grid.data) >= 0)
            & (np.asarray(grid.data) < 50)
            & (clearance >= self.navigation_clearance)
        )
        cells = list(self._shortcut_cells(safe, cells))
        stride = max(1, int(math.ceil(0.45 / grid.resolution)))
        sampled = [cells[0]]
        previous_direction = None
        last_index = 0
        for index in range(1, len(cells)):
            direction = (
                cells[index][0] - cells[index - 1][0],
                cells[index][1] - cells[index - 1][1],
            )
            if previous_direction is not None and direction != previous_direction:
                corner = cells[index - 1]
                if sampled[-1] != corner:
                    sampled.append(corner)
                last_index = index - 1
            if index - last_index >= stride:
                if sampled[-1] != cells[index]:
                    sampled.append(cells[index])
                last_index = index
            previous_direction = direction
        if sampled[-1] != cells[-1]:
            sampled.append(cells[-1])
        return tuple(grid.cell_center(row, column) for row, column in sampled)

    def _local_gains(self, cell, laser_unknown, camera_unseen, grid):
        radius = max(1, int(math.ceil(self.information_radius / grid.resolution)))
        row, column = cell
        row_start, row_stop = max(0, row - radius), min(laser_unknown.shape[0], row + radius + 1)
        column_start, column_stop = max(0, column - radius), min(laser_unknown.shape[1], column + radius + 1)
        local_rows, local_columns = np.indices(
            (row_stop - row_start, column_stop - column_start)
        )
        local_rows += row_start
        local_columns += column_start
        disk = (
            ((local_rows - row) * grid.resolution) ** 2
            + ((local_columns - column) * grid.resolution) ** 2
            <= self.information_radius ** 2
        )
        laser_count = int(np.count_nonzero(laser_unknown[row_start:row_stop, column_start:column_stop] & disk))
        camera_count = int(np.count_nonzero(camera_unseen[row_start:row_stop, column_start:column_stop] & disk))
        cell_area = grid.resolution ** 2
        laser_gain = laser_count * cell_area
        camera_gain = camera_count * cell_area
        combined = (
            (1.0 - self.gain_camera_weight) * laser_gain
            + self.gain_camera_weight * camera_gain
        )
        return laser_gain, camera_gain, combined

    # 16 bearing bins of 22.5 degrees for the camera look-at direction.
    LOOK_AT_BINS = 16

    def _room_interior_yaw(self, cell, grid, gate_center, forward_yaw):
        """Bearing from a viewpoint away from the corridor, i.e. into its room.

        Used only to break look-at ties, so it never overrides a genuine
        asymmetry in the unseen area.
        """
        point = grid.cell_center(*cell)
        normal = (-math.sin(float(forward_yaw)), math.cos(float(forward_yaw)))
        lateral = (point[0] - float(gate_center[0])) * normal[0] + (
            point[1] - float(gate_center[1])
        ) * normal[1]
        return float(forward_yaw) + (math.pi / 2.0 if lateral >= 0.0 else -math.pi / 2.0)

    def _camera_look_at(self, cell, camera_unseen, grid, preferred_yaw=None):
        """Aim RGB-D at the bearing that carries the most unseen area.

        The previous version returned the *centroid* of the unseen cells in the
        annulus.  A centroid is degenerate whenever the unseen region is roughly
        symmetric about the viewpoint, which is the ordinary case: a room centre,
        the corridor, or any freshly entered floor whose camera map is still
        empty.  The centroid then lands on the viewpoint itself, ``atan2(0, 0)``
        collapses the heading to 0 rad -- a fixed *world* axis -- and the RViz
        target arrow is drawn sideways.  Measured on run90 floor 1 every one of
        the 61 door-area viewpoints came out with ``look_at == target`` and
        yaw 0.0 deg, i.e. exactly the 90 deg-off "sideways at the doorway /
        facing out of the door" heading.

        Binning the unseen cells by bearing and taking the heaviest bin is the
        direct "face the direction with the largest information gain" objective,
        and it cannot collapse while any unseen cell exists.  ``preferred_yaw``
        breaks ties (all bins equal, i.e. a fully symmetric unseen region) in
        favour of the caller's room-interior bearing instead of an arbitrary
        world axis.
        """
        radius = max(1, int(math.ceil(self.information_radius / grid.resolution)))
        row, column = cell
        row_start, row_stop = max(0, row - radius), min(camera_unseen.shape[0], row + radius + 1)
        column_start, column_stop = max(0, column - radius), min(camera_unseen.shape[1], column + radius + 1)
        local = camera_unseen[row_start:row_stop, column_start:column_stop]
        rows, columns = np.nonzero(local)
        if not len(rows):
            return None
        rows = rows + row_start
        columns = columns + column_start
        squared = ((rows - row) * grid.resolution) ** 2 + ((columns - column) * grid.resolution) ** 2
        keep = (squared >= 0.6 ** 2) & (squared <= self.information_radius ** 2)
        if not np.any(keep):
            return None
        rows = rows[keep]
        columns = columns[keep]
        bearings = np.arctan2(
            (rows - row) * grid.resolution, (columns - column) * grid.resolution
        )
        width = 2.0 * math.pi / float(self.LOOK_AT_BINS)
        indices = np.floor((bearings + math.pi) / width).astype(np.int64)
        np.clip(indices, 0, self.LOOK_AT_BINS - 1, out=indices)
        weights = np.bincount(indices, minlength=self.LOOK_AT_BINS).astype(np.float64)
        best = float(weights.max())
        if best <= 0.0:
            return None
        # Ties (a symmetric unseen region) resolve toward the room interior when
        # the caller knows it; otherwise the heaviest bin wins outright.
        candidates = np.nonzero(weights >= 0.9 * best)[0]
        centres = -math.pi + (candidates + 0.5) * width
        if preferred_yaw is None:
            chosen = float(centres[int(np.argmax(weights[candidates]))])
        else:
            deltas = np.abs(
                np.arctan2(
                    np.sin(centres - float(preferred_yaw)),
                    np.cos(centres - float(preferred_yaw)),
                )
            )
            chosen = float(centres[int(np.argmin(deltas))])
        # Return a point at a real stand-off distance so the caller's
        # ``atan2(look_at - target)`` is always well defined.
        distance = max(0.75, 0.6 * self.information_radius)
        return (
            grid.origin_x + (column + 0.5) * grid.resolution + math.cos(chosen) * distance,
            grid.origin_y + (row + 0.5) * grid.resolution + math.sin(chosen) * distance,
        )

    def path_is_safe(
        self,
        grid: Optional[GridView],
        path: Sequence[Tuple[float, float]],
        minimum_clearance: Optional[float] = None,
        despeckle: bool = False,
    ) -> bool:
        if grid is None or not path:
            return False
        data = np.asarray(grid.data)
        # Judge the path on the same map the router used.  Routing already
        # despeckles (navigation_path), but validating on the raw speckled grid
        # made a single spurious occupied cell - routine at a door frame - mark
        # the path unsafe, so the explorer abandoned a valid target every cycle
        # (run47 floor 0 room 4: the robot rotated on the spot at the doorway
        # while two opposite targets alternated every ~5.5 s).
        if despeckle:
            _safe, clearance = self._navigation_fields(grid, None, despeckle=True)
        else:
            occupied = data >= 50
            clearance = distance_transform_edt(~occupied) * grid.resolution
        minimum = (
            self.navigation_clearance
            if minimum_clearance is None
            else max(0.05, float(minimum_clearance))
        )
        previous_point = None
        for point in path:
            if previous_point is None:
                samples = (point,)
            else:
                segment = math.hypot(point[0] - previous_point[0], point[1] - previous_point[1])
                count = max(1, int(math.ceil(segment / max(0.5 * grid.resolution, 0.03))))
                samples = tuple(
                    (
                        previous_point[0] + (point[0] - previous_point[0]) * index / count,
                        previous_point[1] + (point[1] - previous_point[1]) * index / count,
                    )
                    for index in range(1, count + 1)
                )
            for sample in samples:
                cell = grid.world_to_cell(*sample)
                if not self._in_bounds(data.shape, *cell):
                    return False
                if data[cell] < 0 or data[cell] >= 50 or clearance[cell] < minimum:
                    return False
            previous_point = point
        return True

    def _door_approach_target(
        self,
        nav_grid,
        robot_pose,
        gate_center,
        forward_yaw,
        assignment_portals,
        owner_ids,
        visited,
        diagnostics,
    ):
        """A reachable viewpoint in front of a door whose room will not route.

        Entering a room that has never been seen cannot be planned: the staged
        crossing ends inside the room and that point is unknown by definition.
        run50 floor 0 stalled in the corridor with four actionable portals where
        every candidate was rejected as ``path_unreachable`` and the verified
        door band held no cells -- the robot stood 8.5 m short of the door with
        no corridor frontier left to drive to, so it never got close enough to
        see through the opening and the room could never become plannable.

        The returned target sits at the corridor staging point in front of that
        door and carries the room as its topology owner, so the node treats it
        as a doorway approach; the next cycle plans the crossing on the grown
        map.
        """
        attempted_ids = {str(item) for item in (owner_ids or ()) if str(item)}
        if not attempted_ids:
            return None
        best = None
        for portal in assignment_portals or ():
            topology_id = str(getattr(portal, "topology_id", ""))
            if topology_id not in attempted_ids:
                continue
            stage = self._portal_waypoint(
                gate_center, forward_yaw, float(portal.along), 0.0
            )
            if not door_approach_is_new(stage, visited, self.revisit_radius):
                continue
            path, length, clearance = self.navigation_path(
                nav_grid, robot_pose, stage, None
            )
            if not path:
                continue
            if best is None or float(length) < float(best[1]):
                best = (portal, float(length), tuple(path), float(clearance), stage)
        if best is None:
            return None
        portal, length, path, clearance, stage = best
        diagnostics["door_approach_portal"] = str(portal.topology_id)
        return FrontierTarget(
            kind="LASER_FRONTIER",
            target=(float(stage[0]), float(stage[1])),
            path=path,
            path_length=length,
            laser_gain=0.0,
            camera_gain=0.0,
            combined_gain=0.0,
            min_clearance=clearance,
            topology_id=str(portal.topology_id),
        )

    def _review_target(self, grid, reachable, distance, point):
        rows, columns = np.nonzero(reachable)
        if len(rows) == 0:
            return None
        x = grid.origin_x + (columns + 0.5) * grid.resolution
        y = grid.origin_y + (rows + 0.5) * grid.resolution
        separation = np.hypot(x - point[0], y - point[1])
        valid = (separation >= 0.75) & (separation <= 2.2)
        if not np.any(valid):
            return None
        candidate_indices = np.nonzero(valid)[0]
        best = min(
            candidate_indices,
            key=lambda index: (distance[rows[index], columns[index]], separation[index]),
        )
        return int(rows[best]), int(columns[best])

    def plan(
        self,
        grid: Optional[GridView],
        robot_pose: Optional[Tuple[float, float, float]],
        gate_center: Optional[Tuple[float, float]],
        forward_yaw: Optional[float],
        camera_seen: Optional[np.ndarray],
        visited_targets: Iterable[Tuple[float, float]] = (),
        sphere_hypotheses: Sequence[dict] = (),
        reviewed_hypotheses: Iterable[str] = (),
        camera_target: float = 0.80,
        navigation_grid: Optional[GridView] = None,
        minimum_forward: float = 0.0,
        topology_lock: Optional[str] = None,
        completed_topologies: Iterable[str] = (),
        confirmed_topologies: Iterable[str] = (),
        remembered_portals: Sequence[RoomPortal] = (),
        front_station_along_hint: Optional[float] = None,
        completed_front_sides: Iterable[str] = (),
        portal_prefix: str = "ROOM",
        force_laser_unknown: bool = False,
        portal_grid: Optional[GridView] = None,
        danger_guidance_level: int = 0,
        interior_only: bool = False,
    ) -> CoveragePlan:
        empty = CoverageSnapshot(0.0, 0.0, 0.0, 0, 0, 0)
        if grid is None or robot_pose is None or gate_center is None or forward_yaw is None:
            return CoveragePlan(empty, None, (), "NOT_READY")
        data = np.asarray(grid.data)
        extent = infer_task_extent(
            grid,
            gate_center,
            forward_yaw,
            self.forward_depth,
            self.back_extension,
            self.lateral_half_width,
            self.corridor_half_width,
        )
        task_mask = task_region_mask(
            grid,
            gate_center,
            forward_yaw,
            self.back_extension,
            extent.forward_limit,
            self.lateral_half_width,
            self.corridor_half_width,
            # Do not apply the gate before the fixed lobby transit has
            # completed.  This keeps the pure planner usable from a pose on
            # the gate plane (and preserves the transit connector); the live
            # node enables it with ``minimum_forward`` once the robot is
            # already on the task side.
            gate_half_width=(
                self.virtual_gate_half_width if minimum_forward > 0.0 else None
            ),
            gate_depth=self.virtual_gate_depth,
        )
        if minimum_forward > 0.0:
            # The virtual entrance gate is a one-way topological boundary for
            # the active floor.  It is not a physical obstacle in the raw
            # navigation map, but no candidate behind it may be selected.
            rows, columns = np.indices(data.shape, dtype=np.float64)
            x = grid.origin_x + (columns + 0.5) * grid.resolution
            y = grid.origin_y + (rows + 0.5) * grid.resolution
            along = (
                (x - float(gate_center[0])) * math.cos(float(forward_yaw))
                + (y - float(gate_center[1])) * math.sin(float(forward_yaw))
            )
            task_mask &= along >= float(minimum_forward)
        snapshot, eligible, clearance = coverage_snapshot(
            grid,
            task_mask,
            camera_seen,
            self.robot_radius,
            self.safety_margin,
            camera_weight=self.camera_weight,
        )
        nav_grid = navigation_grid
        if (
            nav_grid is None
            or np.asarray(nav_grid.data).shape != data.shape
            or abs(nav_grid.resolution - grid.resolution) > 1e-6
            or abs(nav_grid.origin_x - grid.origin_x) > 1e-6
            or abs(nav_grid.origin_y - grid.origin_y) > 1e-6
        ):
            nav_grid = grid
        # Connectivity is a navigation question, not a coverage question.
        # Do not use ``task_mask`` here.  The task mask is deliberately a
        # coverage/goal filter (and may contain a one-way entrance boundary),
        # while A* must be allowed to use the real free corridor and doorway
        # cells as connectors.  Applying the room mask to ``safe`` turns an
        # inferred portal line into a hard wall and is exactly the failure
        # mode seen when the lower half of both rooms becomes unreachable.
        # Doorways first: a mis-detected obstacle sitting in a confirmed doorway
        # is removed from the *map* used for planning, instead of being worked
        # around downstream (verified door bands of 0 cells were the symptom).
        completed_ids = {str(item) for item in completed_topologies}
        confirmed_ids = {str(item) for item in confirmed_topologies}
        door_portals = [
            portal
            for portal in remembered_portals
            if str(portal.topology_id) in confirmed_ids
            and str(portal.topology_id) not in completed_ids
        ]
        plan_grid = nav_grid
        freed_cells = 0
        if door_portals:
            data = np.array(np.asarray(nav_grid.data), dtype=np.int16, copy=True)
            rows, columns = np.indices(data.shape, dtype=np.float64)
            xs = nav_grid.origin_x + (columns + 0.5) * nav_grid.resolution
            ys = nav_grid.origin_y + (rows + 0.5) * nav_grid.resolution
            dx = xs - float(gate_center[0])
            dy = ys - float(gate_center[1])
            along = dx * math.cos(float(forward_yaw)) + dy * math.sin(float(forward_yaw))
            lateral = -dx * math.sin(float(forward_yaw)) + dy * math.cos(float(forward_yaw))
            mask = np.zeros(data.shape, dtype=bool)
            for portal in door_portals:
                sign = 1.0 if str(portal.side) == "L" else -1.0
                span = 0.5 * float(portal.width) + self.door_mask_margin
                mask |= (
                    (np.abs(along - float(portal.along)) <= span)
                    & (lateral * sign >= -0.20)
                    & (lateral * sign <= self.corridor_half_width + self.door_mask_depth)
                )
            freed_cells = clear_doorway_speckle(
                data, mask, self.door_speckle_max_component
            )
            if freed_cells:
                plan_grid = GridView(
                    data, nav_grid.resolution, nav_grid.origin_x, nav_grid.origin_y
                )
        safe, clearance_field = self._navigation_fields(plan_grid, None)
        # Reconnect a room whose *confirmed* doorway the inflated navigation map
        # has sealed.  reachable is derived from safe, so this has to happen
        # before the flood fill; without it the room holds unseen cells that no
        # candidate can ever reach (run27 floor 0 front-right room: 11869 task
        # cells, 260 reachable, 7654 unseen, room_reachable_camera_unseen = 0).
        verified_band_cells = 0
        verified_door_ids = []
        if door_portals:
            band = self.verified_door_band(
                plan_grid, gate_center, forward_yaw, door_portals, clearance_field
            )
            safe = safe | band
            verified_band_cells = int(np.count_nonzero(band))
            verified_door_ids = sorted(
                str(portal.topology_id) for portal in door_portals
            )
        # Reachability is rooted in the robot's own cell, with the same 3x3
        # physical-occupancy window the router uses.  There is no nearest-safe
        # seed: a robot standing where the clearance field is pessimistic must
        # still be able to reach the map around it.
        safe, seed = self._robot_local_safe(nav_grid, safe, robot_pose)
        if seed is None:
            return CoveragePlan(
                snapshot,
                None,
                (),
                "NO_SAFE_SEED",
                task_forward_limit=extent.forward_limit,
                task_extent_confident=extent.confident,
                navigation_hard_clearance=self.navigation_clearance,
            )
        distance, predecessor = self._reachable(safe, seed)
        reachable = distance >= 0
        # A new physical floor may reuse the same 2-D SLAM frame.  In that
        # case cells already known on the lower floor must not suppress the
        # upper-floor exploration frontiers; navigation still uses the real
        # map, while laser coverage is tracked by the floor-scoped camera and
        # frontier state in the live node.
        laser_unknown = eligible if force_laser_unknown else (eligible & (data < 0))
        seen = np.zeros(data.shape, dtype=bool)
        if camera_seen is not None and np.asarray(camera_seen).shape == data.shape:
            seen = np.asarray(camera_seen, dtype=bool)
        camera_unseen = eligible & ~seen
        visited = tuple((float(item[0]), float(item[1])) for item in visited_targets)

        self.last_door_crossing_offset = None
        diagnostics = {
            "assignment_portal_count": 0,
            "room_task_cells": 0,
            "room_eligible_cells": 0,
            "room_camera_unseen_cells": 0,
            "room_reachable_cells": 0,
            "room_reachable_camera_unseen_cells": 0,
            # Which ownership every generated candidate carried, and how many
            # camera viewpoints the corridor fallback had to rescue.  Without
            # this a wrong_topology count cannot be attributed to a kind or an
            # owner, which is what made the run26 deadlock take two cycles to
            # localise.
            "generated_kind_counts": {},
            "corridor_camera_fallback": 0,
            "locked_topology_family": [],
            "verified_door_band_cells": verified_band_cells,
            "verified_door_portals": verified_door_ids,
            "candidate_reject_counts": {
                "visited_or_too_near": 0,
                "wrong_topology": 0,
                "unconfirmed_topology": 0,
                "path_unreachable": 0,
                "door_crossing_offset_used": None,
            },
            "door_approach_dispatched": 0,
            "door_approach_portal": None,
            "door_approach_over_corridor": 0,
            "observation_preemptions": 0,
            "doorway_speckle_cells_freed": int(freed_cells),
            "zone_split_along": None,
            "active_zone": None,
            "zone_topologies": [],
            "zone_corridor_targets": 0,
            "corridor_wander": None,

            "last_reject_reason": None,
        }

        targets = []
        live_portals = detect_room_portals(
            portal_grid if portal_grid is not None else grid,
            gate_center,
            forward_yaw,
            extent.forward_limit,
            self.lateral_half_width,
            self.corridor_half_width,
            portal_prefix=str(portal_prefix),
        )
        confirmed = set(str(item) for item in confirmed_topologies)
        # Once a doorway has accumulated temporal confirmation, a sparse map
        # update must not erase the only route to an unvisited room.  Live
        # geometry wins when present; remembered geometry only fills a missing
        # stable topology and is still gated by ``confirmed_topologies``.
        portals_by_id = {portal.topology_id: portal for portal in live_portals}
        for portal in remembered_portals:
            if (
                portal.topology_id in confirmed
                or portal.topology_id == str(topology_lock)
            ):
                portals_by_id.setdefault(portal.topology_id, portal)
        portals = sorted(
            portals_by_id.values(), key=lambda item: (item.side, item.along)
        )
        # A geometrically valid, temporally confirmed *single* door is enough
        # to own a room.  An opposing door is a scheduling convenience, not a
        # prerequisite for entering the first room.  Longitudinal station
        # bands keep a persistent same-side false gap between the real front
        # and rear doors from being promoted into either phase.
        confirmed_portals = [
            portal
            for portal in portals
            if portal.topology_id in confirmed
            or portal.topology_id == str(topology_lock)
        ]
        if self.portal_merge_radius > 0.0 and len(confirmed_portals) > 1:
            # Collapse the bins of one physical doorway into a single candidate.
            groups = []
            for portal in sorted(confirmed_portals, key=lambda item: float(item.along)):
                group = next(
                    (
                        candidate
                        for candidate in groups
                        if candidate[0].side == portal.side
                        and abs(float(candidate[0].along) - float(portal.along))
                        <= self.portal_merge_radius
                    ),
                    None,
                )
                if group is None:
                    groups.append([portal])
                    continue
                group.append(portal)
                locked_member = next(
                    (
                        member
                        for member in group
                        if str(member.topology_id) == str(topology_lock)
                    ),
                    None,
                )
                # The room already being worked keeps its own id; otherwise the
                # widest opening represents the door.
                group[0] = locked_member or max(
                    group, key=lambda member: float(member.width)
                )
            merged = [group[0] for group in groups]
            removed = len(confirmed_portals) - len(merged)
            if removed > 0:
                diagnostics["portals_merged_same_door"] = removed
                confirmed_portals = merged
        # Guard against map seams being promoted to doorways.  The virtual
        # corridor-entrance gate is anchored a little off the corridor axis, and
        # the wall gaps around it are a recurring source of false candidates; a
        # real room leaves a large usable interior behind its doorway while a
        # seam leaves almost none.  The locked topology is always kept so an
        # in-progress room can never be filtered away mid-approach.
        if self.min_room_interior_cells > 0 and len(confirmed_portals) > 1:
            surviving_portals = []
            for portal in confirmed_portals:
                if portal.topology_id == str(topology_lock):
                    surviving_portals.append(portal)
                    continue
                region = topology_region_mask(
                    grid,
                    gate_center,
                    forward_yaw,
                    extent.forward_limit,
                    self.lateral_half_width,
                    self.corridor_half_width,
                    confirmed_portals,
                    portal.topology_id,
                )
                interior = int(np.count_nonzero(region & eligible))
                if interior >= self.min_room_interior_cells:
                    surviving_portals.append(portal)
                else:
                    diagnostics["portals_rejected_tiny_interior"] = (
                        int(diagnostics.get("portals_rejected_tiny_interior", 0)) + 1
                    )
            confirmed_portals = surviving_portals
        # Opposing door observations can shift longitudinally while the
        # robot exits and re-centres.  Keep the station bounded, but wide
        # enough to admit a directly facing door instead of forcing another
        # full sweep when its raw-map coordinate drifts by a metre or two.
        station_tolerance = 2.5
        rear_minimum_separation = 10.0
        front_candidates = [
            portal
            for portal in confirmed_portals
            if 0.5 <= float(portal.along) <= self.front_station_search_limit
        ]
        front_station_along = (
            float(front_station_along_hint)
            if front_station_along_hint is not None
            else (
                min(float(portal.along) for portal in front_candidates)
                if front_candidates
                else None
            )
        )
        front_station_portals = [
            portal
            for portal in front_candidates
            if front_station_along is not None
            and abs(float(portal.along) - front_station_along) <= station_tolerance
        ]
        completed = set(str(item) for item in completed_topologies)
        completed_portals = [
            portal
            for portal in remembered_portals
            if portal.topology_id in completed
        ]

        def portal_is_complete(portal):
            # Portal IDs are 0.5 m longitudinal bins and can change as SLAM
            # refines a wall.  Completion belongs to the physical doorway,
            # not to that transient bin label.  Opposite sides remain
            # distinct, and real stations are separated by much more than
            # this existing station tolerance.
            return bool(
                portal.topology_id in completed
                or any(
                    doorways_match(portal, old, station_tolerance)
                    for old in completed_portals
                )
            )

        front_rooms_complete = front_stations_complete(
            front_station_portals,
            completed,
            completed_portals,
            completed_front_sides,
            station_tolerance,
        )
        if topology_lock:
            assignment_portals = [
                portal
                for portal in confirmed_portals
                if portal.topology_id == str(topology_lock)
            ]
        else:
            # Corridor exploration owns the whole currently reachable area.
            # Any confirmed room doorway may receive a target; room locking
            # and the existing entry/return lifecycle keep each room isolated.
            # This removes the brittle front/rear station pairing requirement.
            assignment_portals = [
                portal for portal in confirmed_portals
                if float(portal.along) >= 0.5
                and not portal_is_complete(portal)
            ]
        diagnostics["assignment_portal_count"] = len(assignment_portals)
        if topology_lock:
            room_region = topology_region_mask(
                grid,
                gate_center,
                forward_yaw,
                extent.forward_limit,
                self.lateral_half_width,
                self.corridor_half_width,
                assignment_portals,
                str(topology_lock),
            )
            room_eligible = room_region & eligible
            diagnostics.update(
                {
                    "room_task_cells": int(np.count_nonzero(room_region & task_mask)),
                    "room_eligible_cells": int(np.count_nonzero(room_eligible)),
                    "room_camera_unseen_cells": int(
                        np.count_nonzero(room_eligible & ~seen)
                    ),
                    "room_reachable_cells": int(
                        np.count_nonzero(room_region & reachable)
                    ),
                    "room_reachable_camera_unseen_cells": int(
                        np.count_nonzero(room_region & reachable & camera_unseen)
                    ),
                }
            )
        reviewed = set(str(item) for item in reviewed_hypotheses)
        for hypothesis in sphere_hypotheses:
            hypothesis_id = str(hypothesis.get("id", ""))
            center = hypothesis.get("center", ())
            if not hypothesis_id or hypothesis_id in reviewed or len(center) < 2:
                continue
            hypothesis_cell = grid.world_to_cell(float(center[0]), float(center[1]))
            if not self._in_bounds(task_mask.shape, *hypothesis_cell) or not task_mask[hypothesis_cell]:
                continue
            cell = self._review_target(
                grid, reachable, distance, (float(center[0]), float(center[1]))
            )
            if cell is None:
                continue
            path = self._path(grid, predecessor, seed, cell)
            laser_gain, camera_gain, combined = self._local_gains(
                cell, laser_unknown, camera_unseen, grid
            )
            targets.append(
                FrontierTarget(
                    kind="SPHERE_REVIEW",
                    target=grid.cell_center(*cell),
                    path=path,
                    path_length=distance[cell] * grid.resolution,
                    laser_gain=laser_gain,
                    camera_gain=camera_gain,
                    combined_gain=combined,
                    min_clearance=min(clearance[grid.world_to_cell(*point)] for point in path),
                    look_at=(float(center[0]), float(center[1])),
                    hypothesis_id=hypothesis_id,
                )
            )

        def not_visited(cell):
            point = grid.cell_center(*cell)
            return not any(math.hypot(point[0] - old[0], point[1] - old[1]) < self.revisit_radius for old in visited)

        # Camera-unseen reachable viewpoints are preferred until the camera
        # threshold is met.  Bucket sampling prevents a dense carpet of nearly
        # identical goals and the associated target churn.  Lidar frontiers
        # are generated below only as a navigation/geometry fallback; they do
        # not compete with a usable camera viewpoint.
        if snapshot.camera < float(camera_target):
            spacing = max(0.8, self.revisit_radius)
            stride = max(1, int(math.ceil(spacing / grid.resolution)))
            buckets = {}
            for row, column in zip(*np.nonzero(reachable & camera_unseen)):
                cell = (int(row), int(column))
                if distance[cell] < max(2, int(math.ceil(0.6 / grid.resolution))) or not not_visited(cell):
                    diagnostics["candidate_reject_counts"]["visited_or_too_near"] += 1
                    continue
                bucket = (cell[0] // stride, cell[1] // stride)
                previous = buckets.get(bucket)
                if previous is None or distance[cell] < distance[previous]:
                    buckets[bucket] = cell
            all_camera_cells = sorted(
                buckets.values(), key=lambda item: distance[item]
            )
            # A global nearest-64 cutoff lets a dense near room monopolise
            # the candidate pool.  Stratify by the already confirmed
            # topology first, reserving viewpoints for every room (including
            # the far pair), then fill the remaining budget by travel
            # distance.  This makes ``far_room_first`` meaningful without
            # changing the nearest-frontier objective inside a room.
            by_topology = {}
            for candidate_cell in all_camera_cells:
                owner = topology_id_for_point(
                    grid.cell_center(*candidate_cell),
                    gate_center,
                    forward_yaw,
                    self.corridor_half_width,
                    assignment_portals,
                )
                by_topology.setdefault(owner, []).append(candidate_cell)
            reserve_per_topology = max(8, int(math.ceil(64.0 / max(1, len(by_topology)))))
            nearest_camera_cells = []
            for topology_cells in by_topology.values():
                nearest_camera_cells.extend(topology_cells[:reserve_per_topology])
            selected = set(nearest_camera_cells)
            for candidate_cell in all_camera_cells:
                if len(nearest_camera_cells) >= 128:
                    break
                if candidate_cell not in selected:
                    nearest_camera_cells.append(candidate_cell)
                    selected.add(candidate_cell)
            for cell in nearest_camera_cells:
                path = self._path(grid, predecessor, seed, cell)
                laser_gain, camera_gain, combined = self._local_gains(cell, laser_unknown, camera_unseen, grid)
                targets.append(
                    FrontierTarget(
                        kind="CAMERA_FRONTIER",
                        target=grid.cell_center(*cell),
                        path=path,
                        path_length=distance[cell] * grid.resolution,
                        laser_gain=laser_gain,
                        camera_gain=camera_gain,
                        combined_gain=combined,
                        min_clearance=min(clearance[grid.world_to_cell(*point)] for point in path),
                        look_at=self._camera_look_at(
                            cell,
                            camera_unseen,
                            grid,
                            preferred_yaw=self._room_interior_yaw(
                                cell, grid, gate_center, forward_yaw
                            ),
                        ),
                    )
                )

        laser_frontier = np.zeros(data.shape, dtype=bool)
        for row, column in zip(*np.nonzero(reachable)):
            if any(
                self._in_bounds(data.shape, row + d_row, column + d_column)
                and laser_unknown[row + d_row, column + d_column]
                for d_row, d_column in self.CARDINALS
            ):
                laser_frontier[row, column] = True
        for cluster in self._clusters(laser_frontier, grid):
            options = [cell for cell in cluster if not_visited(cell)]
            if not options:
                continue
            cell = min(options, key=lambda item: distance[item])
            path = self._path(grid, predecessor, seed, cell)
            laser_gain, camera_gain, combined = self._local_gains(cell, laser_unknown, camera_unseen, grid)
            targets.append(
                FrontierTarget(
                    kind="LASER_FRONTIER",
                    target=grid.cell_center(*cell),
                    path=path,
                    path_length=distance[cell] * grid.resolution,
                    laser_gain=laser_gain,
                    camera_gain=camera_gain,
                    combined_gain=combined,
                    min_clearance=min(clearance[grid.world_to_cell(*point)] for point in path),
                )
            )

        generated_kind_counts = {}
        for item in targets:
            generated_kind_counts[item.kind] = (
                generated_kind_counts.get(item.kind, 0) + 1
            )
        diagnostics["generated_kind_counts"] = generated_kind_counts

        # Attach an online topology owner before ranking.  In the previous
        # ordering this happened after sorting, so every candidate was still
        # labelled CORRIDOR and the far-room-first policy never took effect.
        targets = [
            replace(
                item,
                topology_id=topology_id_for_point(
                    # A sphere-review stance may lie in the corridor outside
                    # the room.  Its owner is the observed object, not the
                    # camera stance, otherwise transport-only corridor
                    # filtering silently drops a required room review.
                    item.look_at
                    if item.kind == "SPHERE_REVIEW" and item.look_at is not None
                    else item.target,
                    gate_center,
                    forward_yaw,
                    self.corridor_half_width,
                    assignment_portals,
                ),
            )
            for item in targets
        ]
        # Corridor and room topologies have deliberately different sensor
        # objectives.  In CORRIDOR, lidar frontiers discover the large shared
        # structure and may cross a newly observed narrow passage.  Once such
        # a target is dispatched the node locks its portal, after which only
        # camera frontiers in that room are eligible.  A lidar target remains
        # a room-local fallback only when no camera viewpoint exists yet.
        # Red-sphere guidance is introduced in graded steps so it never becomes
        # the dominant objective by default:
        #   level 0: detection only, detector-sourced red reviews never plan;
        #   level 1: detector red reviews are an eligible fallback and rank
        #            *behind* ordinary frontiers (chosen only if nothing else);
        #   level 2: detector red reviews rank first inside the topology that is
        #            already allowed (no cross-room-lock detour);
        #   level 3: corridor-owned detector reviews may cross a room lock.
        # Lidar sphere hypotheses keep their pre-existing (validated) priority:
        # only the colour-detector hint is graduated here.
        guidance_level = max(0, int(danger_guidance_level))
        sphere_first = guidance_level >= 2
        priority = {
            "SPHERE_REVIEW": 0 if sphere_first else 2,
            "CAMERA_FRONTIER": 1,
            "LASER_FRONTIER": 1,
        }

        def detector_review(item):
            return item.kind == "SPHERE_REVIEW" and str(
                item.hypothesis_id or ""
            ).startswith("DET_")

        def target_priority(item):
            if item.kind != "SPHERE_REVIEW":
                return 1
            if detector_review(item):
                return priority["SPHERE_REVIEW"]
            return 0
        camera_targets = [item for item in targets if item.kind == "CAMERA_FRONTIER"]
        if topology_lock and snapshot.camera < float(camera_target) and camera_targets:
            # When RGB-D still has a deficit, suppress ordinary lidar-only
            # frontiers *only in a topology that already has a usable camera
            # viewpoint*.  If a room has no camera frontier yet, retaining one
            # of its lidar frontiers is necessary to enter it and expose the
            # RGB-D sensor to new geometry.  Laser data remains fully used for
            # the navigation map, collision checks, and sphere hypotheses.
            camera_topologies = set(item.topology_id for item in camera_targets)
            targets = [
                item for item in targets
                if item.kind != "LASER_FRONTIER"
                or item.topology_id not in camera_topologies
            ]
        active_station_topologies = {
            portal.topology_id
            for portal in assignment_portals
            if not portal_is_complete(portal)
        }
        # Corridor halves: the near zone runs first, and while it does the far
        # zone's doorways are not admissible at all.  Inside a zone the robot is
        # free to move between its half-corridor and its rooms, so a locked room
        # never removes every candidate (run55 floor 0 locked ROOM_R_3, produced
        # no candidate and stood 1 m from the door until a watchdog fired).
        zone_split = zone_split_along(assignment_portals, extent.forward_limit)
        zone_now = active_zone(assignment_portals, completed, zone_split)
        zone_topology_ids = {
            portal.topology_id
            for portal in assignment_portals
            if zone_of_along(float(portal.along), zone_split) == zone_now
            and not portal_is_complete(portal)
        }
        diagnostics["zone_split_along"] = round(float(zone_split), 2)
        diagnostics["active_zone"] = zone_now
        diagnostics["zone_topologies"] = sorted(zone_topology_ids)

        def along_of(point):
            return (
                (float(point[0]) - float(gate_center[0])) * math.cos(float(forward_yaw))
                + (float(point[1]) - float(gate_center[1])) * math.sin(float(forward_yaw))
            )

        def in_active_zone(point):
            return zone_admits(along_of(point), zone_split, zone_now)

        def next_doorway_along():
            """Along the committed corridor leg should end at.

            A doorway in the ACTIVE zone is preferred, but the leg is transit,
            not a doorway approach: ANY point inside the active zone is an
            acceptable landing (user rule), so when no doorway is known yet fall
            back to the middle of the active zone's corridor, kept ahead of the
            robot and inside the task extent.

            The active-zone filter matters: door ids are 0.5 m bins
            (``ROOM_L_35`` means along ~17.5) and a drifted bin sitting on the
            zone split was chosen as the nearest doorway, so the leg stopped at
            the split (~17.5 m) instead of reaching the far zone.
            Measured on this building: corridor mouth 0.0, front doors 7.40,
            partition 14.03, rear doors 21.50, far wall 28.06.
            """
            robot_along = along_of(robot_pose)
            # "Not clearly near" instead of "== zone_now": the zone band and the
            # drifting 0.5 m door bins made the zone tag unreliable (see below).
            ahead = [
                float(portal.along)
                for portal in assignment_portals
                if str(portal.topology_id) not in completed
                and float(portal.along) > robot_along + 1.0
                and float(portal.along) >= float(zone_split) - 1.0
            ]
            limit = float(extent.forward_limit)
            if ahead:
                landing = min(ahead) - COMMITTED_LANDING_SHORTEN
            else:
                # No doorway known beyond the split: any point in the far
                # corridor is an acceptable landing.
                landing = 0.5 * (float(zone_split) + limit)
            if landing <= robot_along + 1.0 or limit <= 0.0:
                return None
            return min(landing, limit)

        def corridor_wander_target(forward_only=False, descending=False, target_along=None):
            """A guaranteed-safe transit point on the active zone's centreline.

            The corridor is free space by construction, so this is always
            dispatchable.  It is the last resort for both failure shapes seen so
            far: every doorway-derived candidate filtered out (run58 floor 0) and
            every candidate failing its route (run61 floor 0, where the pre-route
            fallback could not help because the pool only emptied during
            routing).

            ``forward_only`` and ``descending`` turn it into the P3b committed
            transit leg: longest forward centreline step first, shortened until
            one is navigable.  ``target_along`` replaces the ladder with one
            absolute along value - the next unfinished room doorway - so the
            committed leg lands at the doorway instead of at an arbitrary
            distance.  The active-zone admission test still applies, so the near
            zone is always finished before a far-zone leg is allowed.
            """
            robot_along = along_of(robot_pose)
            if target_along is not None:
                reaches = (float(target_along) - robot_along,)
            else:
                reaches = corridor_transit_reaches(forward_only, descending)
            for reach in reaches:
                candidate_along = robot_along + reach
                point = self._portal_waypoint(
                    gate_center, forward_yaw, candidate_along, 0.0
                )
                if not zone_admits(candidate_along, zone_split, zone_now):
                    continue
                # Half the normal revisit radius: a transit point is worth
                # re-driving after a failed excursion, it is not a viewpoint.
                # The committed ``target_along`` mode is a transit leg, so a
                # revisited doorway point must stay dispatchable - otherwise the
                # leg silently falls back to frontier chasing.
                if target_along is None and not door_approach_is_new(
                    point, visited, 0.5 * self.revisit_radius
                ):
                    continue
                wander_path, wander_length, wander_clearance = self.navigation_path(
                    nav_grid, robot_pose, point, None
                )
                if not wander_path:
                    continue
                diagnostics["corridor_wander"] = float(reach)
                return FrontierTarget(
                    kind="LASER_FRONTIER",
                    target=(float(point[0]), float(point[1])),
                    path=tuple(wander_path),
                    path_length=float(wander_length),
                    laser_gain=0.0,
                    camera_gain=0.0,
                    combined_gain=0.0,
                    min_clearance=float(wander_clearance),
                    topology_id="CORRIDOR",
                )
            return None
        portal_order = {
            portal.topology_id: index
            for index, portal in enumerate(
                sorted(assignment_portals, key=lambda item: (item.along, item.side))
            )
        }
        interior_only = bool(interior_only)
        if topology_lock:
            before_topology_filter = len(targets)
            # Portal ids are 0.5 m longitudinal bins of one physical doorway
            # and they drift as SLAM refines the wall, so a target for the
            # locked room can carry a neighbouring bin's id.  Matching the id
            # exactly emptied the pool and burned the node's whole station
            # budget without entering: on run26 floor 0 the explorer held
            # ROOM_R_15 with candidate_topologies == () and
            # last_plan_reason NO_FRONTIER for 59 consecutive sim seconds
            # (wrong_topology dropped 27 candidates) and released the lock
            # without entering, while that same doorway was simultaneously
            # known as ROOM_R_30/36/39/43/48/52/56.  Accept every bin of the
            # same physical door, using the tolerance the completion test
            # already applies.
            allowed_topologies = {str(topology_lock)}
            locked_portal = next(
                (
                    portal
                    for portal in confirmed_portals
                    if portal.topology_id == str(topology_lock)
                ),
                None,
            )
            if locked_portal is not None:
                allowed_topologies.update(
                    portal.topology_id
                    for portal in confirmed_portals
                    if portal.side == locked_portal.side
                    and abs(float(portal.along) - float(locked_portal.along))
                    <= station_tolerance
                )
            diagnostics["locked_topology_family"] = sorted(allowed_topologies)
            locked_targets = [
                item for item in targets
                if item.topology_id in allowed_topologies
            ]
            locked_camera = [
                item for item in locked_targets
                if item.kind in ("CAMERA_FRONTIER", "SPHERE_REVIEW")
            ]
            # A red object detected in the corridor may be reviewed even while a
            # room is locked, but only at the top guidance level: the room lock
            # exists to sequence doorway approaches, so crossing it is an
            # explicit opt-in rather than the default.
            corridor_reviews = (
                [
                    item for item in targets
                    if detector_review(item)
                    and item.topology_id == "CORRIDOR"
                    and item not in locked_targets
                ]
                if guidance_level >= 3
                else []
            )
            locked_lasers = [
                item for item in locked_targets if item.kind == "LASER_FRONTIER"
            ]
            # "Getting out is allowed": the rest of the zone - its half-corridor
            # and, through the corridor pool below, its other room - stays
            # admissible while a room is locked.  A locked room that runs out of
            # candidates therefore falls back to driving the zone instead of
            # standing still.
            zone_corridor = (
                []
                if interior_only
                else [
                    # Corridor transit is lidar-only: the corridor is a camera
                    # frontier exclusion zone (same rule as the fallback below).
                    item
                    for item in targets
                    if item.kind == "LASER_FRONTIER"
                    and item.topology_id == "CORRIDOR"
                    and in_active_zone(item.target)
                    and item not in locked_targets
                ]
            )
            targets = (
                (locked_camera or locked_lasers)
                + zone_corridor
                + corridor_reviews
            )
            diagnostics["zone_corridor_targets"] = len(zone_corridor)
            diagnostics["candidate_reject_counts"]["wrong_topology"] += (
                before_topology_filter - len(targets)
            )
        else:
            unfiltered_targets = list(targets)
            before_topology_filter = len(targets)
            # The validated baseline keeps corridor transit lidar-only.  From
            # level 1 the colour-detector reviews become an extra, lower-ranked
            # fallback; lidar sphere hypotheses stay exactly as validated.
            corridor_frontiers = [
                item for item in targets
                if item.kind == "LASER_FRONTIER"
                and (
                    (
                        item.topology_id == "CORRIDOR"
                        and in_active_zone(item.target)
                    )
                    or item.topology_id in zone_topology_ids
                )
            ]
            detector_reviews = (
                [item for item in targets if detector_review(item)]
                if guidance_level >= 1
                else []
            )
            # User rule: while the far zone is active but the robot is still in
            # the near half, a corridor CAMERA viewpoint beyond the split is the
            # natural pull toward the far zone - admit it only for that crossing
            # (the corridor stays a camera exclusion zone at all other times, so
            # ordinary near-zone corridor transit is unchanged).
            crossing = crossing_zone(zone_now, along_of(robot_pose), zone_split)
            corridor_camera = (
                [
                    item for item in targets
                    if item.kind == "CAMERA_FRONTIER"
                    and item.topology_id == "CORRIDOR"
                    and in_active_zone(item.target)
                ]
                if crossing
                else []
            )
            diagnostics["corridor_camera_crossing"] = len(corridor_camera)
            targets = corridor_frontiers + corridor_camera + detector_reviews
            # Run the camera fallback whenever no surviving target belongs to a
            # room -- not only when the list is empty.  On an upper floor the
            # laser often produces exactly one corridor frontier (the rooms are
            # already known in the shared 2-D frame, so no room laser frontier is
            # generated) while the RGB-D side still owes coverage; because that
            # single corridor frontier made the list non-empty, every one of the
            # room camera viewpoints was discarded and the explorer toured the
            # corridor for ever.  Measured live on run90 floor 1:
            # generated_kind_counts {'CAMERA_FRONTIER': 128, 'LASER_FRONTIER': 1},
            # room_eligible_cells 0, room_task_cells 0, target_kind
            # LASER_FRONTIER/CORRIDOR with no room lock and no entry for 35+ sim s.
            # The corridor target is kept as the fallback: the "room wins over
            # corridor" filter below removes it as soon as a room target exists,
            # and when the fallback is empty this is exactly the old
            # ``if not targets`` behaviour.
            # Restored to the validated baseline: the camera fallback runs only
            # when NOTHING at all is left to do.  Widening it to "no room-owned
            # target" was mine, and it is the single change behind every
            # target-selection regression this session -- the robot walked to the
            # lobby, walked back into a finished room, toured the corridor, and
            # picked far-away viewpoints while standing at a doorway.  Its
            # intended replacement is the explicit single gate
            # ``admit_candidate`` plus a declared "unexplored room must have a
            # target" invariant, not a wider pool.
            if not targets:
                # "No unknown lidar cell left" is not "floor finished".  A room
                # whose interior the laser already mapped from the corridor
                # still owes RGB-D coverage, and only a camera viewpoint puts
                # the robot inside it.  Because this branch admits lidar
                # frontiers only, that camera viewpoint used to be discarded
                # and the planner returned NO_FRONTIER for ever: measured on
                # run17_endtoend_084 (0.84, three_floor) the explorer sat with
                # actionable_portals=16, camera coverage 0.414 against a 0.84
                # target, and cmd_vel exactly 0 for 731 consecutive telemetry
                # samples while ROOM_R_55 alone still had 5665 unseen cells.
                # Ownership is deliberately not checked here.  Portal ids are
                # transient 0.5 m bins of one physical doorway and the room
                # owner they imply changes as the map refines, so gating the
                # fallback on an owner id drops exactly the viewpoints that are
                # needed (run26 floor 0: wrong_topology discarded 29 camera
                # candidates while the front-right room was still unvisited).
                # A camera viewpoint is a safe action by construction -- it is
                # generated only from reachable, still-unseen, safety-checked
                # cells -- and the confirmation and completed-portal filters
                # below still apply, so a floor whose rooms are all finished
                # keeps reporting NO_FRONTIER.
                # Room regions only: the corridor is a camera-frontier
                # exclusion zone.  A corridor viewpoint is never a task here
                # (the corridor is transit), so admitting one would put a
                # corridor camera target straight back into the pool this
                # fallback exists to fill with room entry points.
                # ... and inside the task region: the lobby sits behind the
                # entrance gate, outside the corridor band, so its cells carry a
                # room-ish id even though the lobby is not a task region.
                cosine, sine = math.cos(float(forward_yaw)), math.sin(float(forward_yaw))
                fallback = (
                    [
                        item for item in unfiltered_targets
                        if item.kind == "CAMERA_FRONTIER"
                        and item.topology_id != "CORRIDOR"
                        and (
                            (item.target[0] - float(gate_center[0])) * cosine
                            + (item.target[1] - float(gate_center[1])) * sine
                        ) >= -0.5
                        # ... and ahead of the robot along the corridor.
                        and (
                            (item.target[0] - float(robot_pose[0])) * cosine
                            + (item.target[1] - float(robot_pose[1])) * sine
                        ) >= -0.5
                    ]
                    if active_station_topologies
                    else []
                )
                targets = fallback
                diagnostics["corridor_camera_fallback"] = len(fallback)
                if not targets and active_station_topologies:
                    # P3b fallback: no frontier anywhere, so commit to one fixed
                    # forward step on the corridor centreline that lands at the
                    # next unfinished room doorway (measured ~6.5 m from the
                    # partition to the rear doors in this building).  The node
                    # runs it as the committed manoeuvre (centre, align,
                    # measured run).
                    doorway_along = next_doorway_along()
                    wander = corridor_wander_target(
                        forward_only=True,
                        descending=True,
                        target_along=doorway_along,
                    )
                    if wander is not None:
                        targets = [wander]
                        diagnostics["fixed_step_transit"] = 1
                        if doorway_along is not None:
                            diagnostics["committed_transit_along"] = float(
                                doorway_along
                            )
                if not targets and active_station_topologies:
                    # Nothing anywhere in the map: drive to the door of an
                    # unfinished room so its interior gets observed.  run64
                    # floor 1 sat at 3/4 with generated_kind_counts == {} and
                    # corridor_wander None because every transit point was
                    # already visited.
                    approach = self._door_approach_target(
                        nav_grid,
                        robot_pose,
                        gate_center,
                        forward_yaw,
                        assignment_portals,
                        active_station_topologies,
                        visited,
                        diagnostics,
                    )
                    if approach is not None:
                        targets = [approach]
                        diagnostics["door_approach_dispatched"] = 1
            else:
                diagnostics["corridor_camera_fallback"] = 0
            diagnostics["candidate_reject_counts"]["wrong_topology"] += (
                before_topology_filter - len(targets)
            )
        # The corridor is transit, not a task region.  As long as a room can be
        # worked, a corridor frontier must not win the ranking: run79 drove the
        # whole 36 m corridor (targets 14.05 -> 10.88 -> 6.79 -> 3.28 m) while
        # four rear doorways were actionable, because the corridor's laser gain
        # (~7.8) beats a room's (~2.7-4.5) in the corridor-phase ranking.  The
        # corridor stays as the fallback target only when no room candidate
        # exists (all rooms done, or nothing routable).
        if not topology_lock:
            room_targets = [item for item in targets if item.topology_id != "CORRIDOR"]
            if room_targets:
                targets = room_targets
        # Ranking.  ``nearest`` keeps the historical nearest-frontier objective.
        # ``gain_efficiency`` ranks by information gain per metre of travel, so
        # a laser-weighted gain can actually outrank a marginally nearer
        # viewpoint: with the old key the gain was only consulted when two
        # paths were exactly the same length, which made the whole gain term
        # decorative.
        def rank_key(item, gain):
            if self.target_rank_mode == "nearest":
                return (target_priority(item), item.path_length, -gain, -item.min_clearance)
            cost = max(float(item.path_length), 0.5)
            return (
                target_priority(item),
                -(float(gain) / cost),
                item.path_length,
                -item.min_clearance,
            )

        if topology_lock:
            targets.sort(key=lambda item: rank_key(item, item.combined_gain))
        else:
            # Fixed forward is the FIRST priority once the far zone is active
            # (explicit user command, 2026-09-14).  The corridor leg is a
            # committed manoeuvre: while the landing along is still ahead, the
            # pool is replaced outright.  Ranking frontiers first was what let
            # the robot stall between them (zero-gain corridor frontiers, stuck
            # target drops, then NO_FRONTIER for tens of seconds).
            #
            # The active-zone gate is what keeps this from opening the far zone
            # early: ``zone_now == "B"`` only after every near-zone doorway is
            # complete.  Once the landing is reached the ordinary targets (room
            # viewpoints) take over again, so room entry is unaffected.
            landing = next_doorway_along()
            # Gate on "no clearly-near unfinished doorway" rather than on
            # ``zone_now``: the zone band (1.5 m) classified a drifted bin at
            # along ~18 (ROOM_L_36) as NEAR, so active_zone stayed "A" for ever
            # and the fixed forward never fired (run134: zone A, committed None,
            # a zero-gain ROOM_L_36 target, then a stuck drop).
            near_unfinished = [
                portal
                for portal in assignment_portals
                if str(portal.topology_id) not in completed
                and float(portal.along) < float(zone_split) - 1.0
            ]
            if (
                not near_unfinished
                and landing is not None
                and along_of(robot_pose) < float(landing) - 0.5
            ):
                transit = corridor_wander_target(
                    forward_only=True,
                    descending=True,
                    target_along=landing,
                )
                # Only replace the pool when a dispatchable transit target
                # exists; otherwise keep the ranked pool (the corridor contract
                # test relies on corridor frontiers surviving here).
                if transit is not None:
                    diagnostics["fixed_step_transit"] = 1
                    diagnostics["committed_transit_along"] = float(landing)
                    targets = [transit]
                else:
                    diagnostics["fixed_step_transit"] = 0
                    targets.sort(
                        key=lambda item: (
                            -projected_travel(robot_pose, item.target, forward_yaw),
                            item.path_length,
                        )
                    )
            else:
                diagnostics["fixed_step_transit"] = 0
                targets.sort(
                    key=lambda item: (
                        -projected_travel(robot_pose, item.target, forward_yaw),
                        item.path_length,
                    )
                )
        # Keep the complete map-derived pool for diagnostics, even though
        # dispatch/ranking below uses only confirmed topology ownership.
        diagnostic_topologies = tuple(
            sorted(
                set(
                    topology_id_for_point(
                        item.target,
                        gate_center,
                        forward_yaw,
                        self.corridor_half_width,
                        portals,
                    )
                    for item in targets
                )
            )
        )
        candidate_topologies = diagnostic_topologies
        # An observed width/jamb gap is only a hypothesis until the live node
        # has accumulated temporal evidence.  Do not let an unconfirmed gap
        # split a room's coverage denominator or assign nearby frontiers to a
        # synthetic topology: that is what produced the horizontal cut seen
        # below both the first and second rooms.  Keep an active lock in the
        # set so a doorway already being traversed is not lost during a sparse
        # map update.
        topology_coverages = {}
        for portal in assignment_portals:
            region = topology_region_mask(
                grid,
                gate_center,
                forward_yaw,
                extent.forward_limit,
                self.lateral_half_width,
                self.corridor_half_width,
                assignment_portals,
                portal.topology_id,
            )
            local_snapshot, _local_eligible, _local_clearance = coverage_snapshot(
                grid,
                region,
                camera_seen,
                self.robot_radius,
                self.safety_margin,
                camera_weight=self.camera_weight,
            )
            topology_coverages[portal.topology_id] = local_snapshot
        # Width/jamb candidates are only dispatchable after the live node has
        # seen the same portal over several map updates.  The unfiltered
        # ``candidate_topologies`` above is retained for diagnostics and for
        # evidence accumulation; this filter prevents a one-frame, same-width
        # mapping hole from steering the robot into a false room.
        before_confirmation_filter = len(targets)
        targets = [
            item
            for item in targets
            if (
                item.kind == "SPHERE_REVIEW"
                or item.topology_id == "CORRIDOR"
                or item.topology_id in confirmed
                or item.topology_id == str(topology_lock)
                # Frontier-first (P1): a frontier point that already lies behind
                # the corridor wall is a room-interior target, so it must not
                # wait for the doorway to accumulate confirmation evidence.  The
                # doorway is only where the route crosses - not a gate the plan
                # has to pass first.  Door evidence keeps deciding the region id
                # and the coverage denominator (bookkeeping only).
                or str(item.topology_id).endswith("_UNASSIGNED")
            )
        ]
        diagnostics["candidate_reject_counts"]["unconfirmed_topology"] += (
            before_confirmation_filter - len(targets)
        )
        # A room lock prevents the nearest frontier in room 1 from stealing
        # the planner after room 2 has been selected.  If the locked room has
        # no candidate this cycle, fall through to an uncompleted topology.
        if topology_lock:
            locked = [item for item in targets if item.topology_id == str(topology_lock)]
            # A locked room is an exclusive topology.  An empty local frontier
            # pool must not fall through to another room while the robot is
            # physically inside this one; the node either waits for a local
            # map update or starts an explicit return-to-corridor route.
            targets = locked
        elif completed:
            active_ids = {
                portal.topology_id
                for portal in assignment_portals
                if not portal_is_complete(portal)
            }
            targets = [
                item
                for item in targets
                if item.topology_id == "CORRIDOR"
                or item.topology_id in active_ids
                # Frontier-first (P1b): a frontier behind the corridor wall is a
                # room-interior target even when its doorway has not yet been
                # confirmed into ``assignment_portals``.  Without this clause an
                # upper floor kept only CORRIDOR targets once one room was done
                # and drove up and down the corridor instead of entering the
                # remaining rooms (reported on run82 floor 1).
                or str(item.topology_id).endswith("_UNASSIGNED")
            ]
        # Candidate discovery/ranking remains nearest-frontier based.  Only
        # the selected executable route is replaced with orientation-aware
        # A* plus a collision-checked line-of-sight shortcut.
        if targets:
            chosen_index = None
            chosen = None
            targets_attempted = list(targets)
            failed_room_ids = []
            for index, candidate in enumerate(targets):
                path, path_length, minimum = self.navigation_path(
                    nav_grid, robot_pose, candidate.target, None
                )
                portal = next(
                    (
                        item
                        for item in assignment_portals
                        if item.topology_id == candidate.topology_id
                    ),
                    None,
                )
                robot_topology = topology_id_for_point(
                    robot_pose[:2],
                    gate_center,
                    forward_yaw,
                    self.corridor_half_width,
                    assignment_portals,
                )
                if not path and str(getattr(candidate, "topology_id", "CORRIDOR")) != "CORRIDOR":
                    failed_room_ids.append(str(candidate.topology_id))
                if portal is not None and robot_topology != candidate.topology_id:
                    path, path_length, minimum, _entry_depth = (
                        self.navigation_path_through_portal(
                        nav_grid,
                        robot_pose,
                        gate_center,
                        forward_yaw,
                        portal,
                        candidate.target,
                        None,
                        self.door_crossing_wide_search,
                    )
                    )
                if path:
                    # Reject gross detours: a target behind a wall is still
                    # "reachable" by a long route around the partition, and
                    # dispatching it burns the whole doorway-approach budget.
                    # Observed on floor 0 of the 0.84 run: a room camera
                    # frontier with a ~5 m straight line was dispatched with a
                    # 23 m (then 32 m) path, the doorway was never crossed and
                    # the room was parked as BLOCKED.
                    straight = math.hypot(
                        float(candidate.target[0]) - float(robot_pose[0]),
                        float(candidate.target[1]) - float(robot_pose[1]),
                    )
                    if path_length > 8.0 and path_length > 3.0 * max(straight, 0.5):
                        diagnostics["candidate_reject_counts"]["path_unreachable"] += 1
                        if str(getattr(candidate, "topology_id", "CORRIDOR")) != "CORRIDOR":
                            failed_room_ids.append(str(candidate.topology_id))
                        continue
                    chosen_index = index
                    chosen = replace(
                        candidate,
                        path=path,
                        path_length=path_length,
                        min_clearance=minimum,
                    )
                    break
            if chosen is not None:
                if prefer_room_approach(
                    getattr(chosen, "topology_id", "CORRIDOR"),
                    failed_room_ids,
                    getattr(chosen, "combined_gain", None),
                    len(active_station_topologies),
                    self.observation_weak_gain,
                ):
                    approach = self._door_approach_target(
                        nav_grid,
                        robot_pose,
                        gate_center,
                        forward_yaw,
                        assignment_portals,
                        set(failed_room_ids),
                        visited,
                        diagnostics,
                    )
                    if approach is not None:
                        diverting_for_observation = not failed_room_ids
                        chosen = approach
                        chosen_index = None
                        diagnostics["door_approach_dispatched"] = 1
                        diagnostics["door_approach_over_corridor"] = 1
                        if diverting_for_observation:
                            # Bought observation instead of a nearly worthless
                            # corridor move.  Counted separately so the policy
                            # can be judged on evidence: if these episodes do not
                            # unlock frontiers or new known cells, the trigger is
                            # costing time for nothing.
                            diagnostics["observation_preemptions"] = (
                                diagnostics.get("observation_preemptions", 0) + 1
                            )
                if chosen_index is None:
                    targets.insert(0, chosen)
                else:
                    targets.pop(chosen_index)
                    targets.insert(0, chosen)
            else:
                diagnostics["candidate_reject_counts"]["path_unreachable"] += len(targets)
                targets = []
                approach = self._door_approach_target(
                    nav_grid,
                    robot_pose,
                    gate_center,
                    forward_yaw,
                    assignment_portals,
                    {
                        getattr(item, "topology_id", None)
                        for item in targets_attempted
                    },
                    visited,
                    diagnostics,
                )
                if approach is not None:
                    targets = [approach]
                    diagnostics["door_approach_dispatched"] = 1
                if not targets and not topology_lock and active_station_topologies:
                    # Same P3b committed forward step as the empty-pool fallback
                    # above, after every candidate failed to route: land at the
                    # next unfinished room doorway.
                    doorway_along = next_doorway_along()
                    wander = corridor_wander_target(
                        forward_only=True,
                        descending=True,
                        target_along=doorway_along,
                    )
                    if wander is not None:
                        targets = [wander]
                        diagnostics["fixed_step_transit"] = 1
                        if doorway_along is not None:
                            diagnostics["committed_transit_along"] = float(
                                doorway_along
                            )
        if not targets and topology_lock:
            # A locked room can yield *no* candidate at all - its interior is
            # still unknown, so there is no frontier and no camera viewpoint to
            # seed from.  run55 floor 0 locked ROOM_R_3 exactly like that and
            # stood 1 m from its door for the rest of the run (tgt=None,
            # cand=1).  Send the robot to the door so the next cycle can see in.
            approach = self._door_approach_target(
                nav_grid,
                robot_pose,
                gate_center,
                forward_yaw,
                assignment_portals,
                {str(topology_lock)},
                visited,
                diagnostics,
            )
            if approach is not None:
                targets = [approach]
                diagnostics["door_approach_dispatched"] = 1
        if (
            not targets
            and not topology_lock
            and diagnostics["assignment_portal_count"] > 0
        ):
            # Last-resort corridor transit.  When nothing at all survives the
            # filters the robot used to stand still on NO_FRONTIER (run129 floor
            # 0: frozen at along 7.41 for 50+ sim seconds after the front pair
            # retired, with a zero-gain ROOM_L_31 candidate that could not be
            # routed).  This is the user's "first go to the corridor, then drive
            # straight" rule, and it deliberately does NOT depend on
            # ``active_station_topologies`` - that gate is exactly what left this
            # case with no fallback at all.
            doorway_along = next_doorway_along()
            if doorway_along is not None:
                transit = corridor_wander_target(
                    forward_only=True,
                    descending=True,
                    target_along=doorway_along,
                )
                if transit is not None:
                    targets = [transit]
                    diagnostics["fixed_step_transit"] = 1
                    diagnostics["committed_transit_along"] = float(doorway_along)
        if not targets:
            if diagnostics["assignment_portal_count"] == 0:
                diagnostics["last_reject_reason"] = "NO_ASSIGNMENT_PORTAL"
            elif not topology_lock:
                # The room_* counters below are only populated while a room is
                # locked.  Testing them in corridor ownership always matched
                # the first branch and reported EMPTY_ROOM_TASK_MASK, which
                # sent a whole debugging cycle after an empty room mask that
                # had never been computed.  Report the ownership-local reasons
                # instead.
                if diagnostics["candidate_reject_counts"]["path_unreachable"]:
                    diagnostics["last_reject_reason"] = "CANDIDATE_PATH_UNREACHABLE"
                elif diagnostics["candidate_reject_counts"]["wrong_topology"]:
                    diagnostics["last_reject_reason"] = "CANDIDATE_TOPOLOGY_MISMATCH"
                else:
                    diagnostics["last_reject_reason"] = "NO_FRONTIER_AFTER_FILTERS"
            elif diagnostics["room_task_cells"] == 0:
                diagnostics["last_reject_reason"] = "EMPTY_ROOM_TASK_MASK"
            elif diagnostics["room_eligible_cells"] == 0:
                diagnostics["last_reject_reason"] = "NO_ROOM_ELIGIBLE_CELLS"
            elif diagnostics["room_camera_unseen_cells"] == 0:
                diagnostics["last_reject_reason"] = "ROOM_CAMERA_ALREADY_SEEN"
            elif diagnostics["room_reachable_camera_unseen_cells"] == 0:
                diagnostics["last_reject_reason"] = "NO_REACHABLE_CAMERA_UNSEEN"
            elif diagnostics["candidate_reject_counts"]["path_unreachable"]:
                diagnostics["last_reject_reason"] = "CANDIDATE_PATH_UNREACHABLE"
            elif diagnostics["candidate_reject_counts"]["wrong_topology"]:
                diagnostics["last_reject_reason"] = "CANDIDATE_TOPOLOGY_MISMATCH"
            else:
                diagnostics["last_reject_reason"] = "NO_FRONTIER_AFTER_FILTERS"
        diagnostics["door_crossing_offset_used"] = self.last_door_crossing_offset
        # Surfaced so a hung-looking plan can be told apart from a planner whose
        # A* walk keeps hitting a cyclic predecessor chain.
        diagnostics["predecessor_cycle_breaks"] = int(self.predecessor_cycle_breaks)
        return CoveragePlan(
            snapshot=snapshot,
            target=targets[0] if targets else None,
            targets=tuple(targets),
            reason="TARGET" if targets else "NO_FRONTIER",
            task_forward_limit=extent.forward_limit,
            task_extent_confident=extent.confident,
            navigation_reachable_cells=int(np.count_nonzero(reachable)),
            navigation_hard_clearance=self.navigation_clearance,
            candidate_topologies=candidate_topologies,
            observed_portals=tuple(live_portals),
            actionable_portals=tuple(assignment_portals),
            topology_coverages=topology_coverages,
            front_station_portals=tuple(front_station_portals),
            front_station_along=front_station_along,
            front_rooms_complete=front_rooms_complete,
            diagnostics=diagnostics,
        )


def detect_sphere_like_clusters(
    points,
    robot_z: float,
    minimum_points: int = 8,
    cluster_radius: float = 0.09,
) -> Tuple[Tuple[float, float, float], ...]:
    """Extract small, low, roughly isotropic point clusters as review hints."""
    values = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    if len(values) == 0:
        return ()
    relative_z = values[:, 2] - float(robot_z)
    values = values[(relative_z >= -0.26) & (relative_z <= 0.06)]
    if len(values) < minimum_points:
        return ()
    tree = cKDTree(values)
    remaining = set(range(len(values)))
    clusters = []
    while remaining:
        seed = remaining.pop()
        component = [seed]
        queue = deque([seed])
        while queue:
            index = queue.popleft()
            neighbours = tree.query_ball_point(values[index], cluster_radius)
            for neighbour in neighbours:
                if neighbour in remaining:
                    remaining.remove(neighbour)
                    component.append(neighbour)
                    queue.append(neighbour)
        if len(component) < minimum_points or len(component) > 500:
            continue
        cluster = values[component]
        spans = np.ptp(cluster, axis=0)
        horizontal_min = min(spans[0], spans[1])
        horizontal_max = max(spans[0], spans[1])
        if not (0.10 <= horizontal_min <= 0.42 and horizontal_max <= 0.48):
            continue
        if not (0.08 <= spans[2] <= 0.36):
            continue
        if horizontal_max / max(horizontal_min, 1e-6) > 1.9:
            continue
        clusters.append(tuple(float(value) for value in np.mean(cluster, axis=0)))
    return tuple(clusters)
