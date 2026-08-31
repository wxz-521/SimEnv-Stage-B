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


@dataclass(frozen=True)
class PortalStation:
    """An opposing left/right doorway pair at one corridor station."""

    station_id: str
    along: float
    left: RoomPortal
    right: RoomPortal


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


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


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


def corrected_portal_heading(
    corridor_yaw: float, side: str, along_error: float
) -> float:
    """Face across a door with a small correction towards its centre."""
    sign = 1.0 if side == "L" else -1.0
    correction = max(-0.10, min(0.10, 0.35 * float(along_error)))
    return normalize_angle(corridor_yaw + sign * math.pi / 2.0 - sign * correction)


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
    laser: float, camera: float, camera_weight: float = 0.90
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


def detect_room_portals(
    grid: GridView,
    gate_center: Tuple[float, float],
    forward_yaw: float,
    forward_depth: float,
    lateral_half_width: float = 9.5,
    corridor_half_width: float = 1.1,
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
        (along >= 0.0)
        & (along <= float(forward_depth))
        & (np.abs(lateral) <= float(lateral_half_width))
    )
    bin_count = max(1, int(math.ceil(float(forward_depth) / grid.resolution)))
    portals = []
    # The wall-crossing band is kept narrow to reject ordinary room interior
    # cells.  A 0.12 m band tolerates one or two SLAM cells of wall jitter.
    # A width-only gap is not a doorway: furniture edges and mapping holes can
    # have the same apparent width.  A real room door must remain a gap in a
    # *continuous parent wall*, with occupied jamb evidence immediately before
    # and after the opening and a short free passage on the room side.
    jamb_length = 0.65
    jamb_support = 0.55
    wall_band = 0.22
    passage_depth = 0.70
    for side, sign in (("L", 1.0), ("R", -1.0)):
        signed = sign * lateral
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
            & (signed >= float(corridor_half_width) - 0.12)
            & (signed <= float(corridor_half_width) + 0.12)
        )
        indices = np.floor(along[crossing] / grid.resolution).astype(np.int64)
        indices = indices[(indices >= 0) & (indices < bin_count)]
        free_counts = np.bincount(indices, minlength=bin_count)
        wall_cells = (data >= 50) & in_search & (
            signed >= float(corridor_half_width) - wall_band
        ) & (signed <= float(corridor_half_width) + wall_band)
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
            if 0.40 <= width <= 2.4:
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
                inner_samples = np.linspace(0.20, passage_depth, 4)
                passage = []
                for depth in inner_samples:
                    point_x = float(gate_center[0]) + cosine * centre_along - sine * sign * (float(corridor_half_width) + depth)
                    point_y = float(gate_center[1]) + sine * centre_along + cosine * sign * (float(corridor_half_width) + depth)
                    row, column = grid.world_to_cell(point_x, point_y)
                    if 0 <= row < data.shape[0] and 0 <= column < data.shape[1]:
                        passage.append(int(data[row, column]) == 0)
                if len(passage) < 3 or sum(passage) < 3:
                    continue
                candidates.append((centre_along, width))
        for value, width in candidates:
            # Quantise the observed longitudinal coordinate rather than using
            # the list index.  As SLAM reveals a nearer doorway, list indices
            # can shift; a coordinate-based ID keeps an already active room
            # stable across map updates.
            stable_bin = int(round(float(value) / 0.5))
            portals.append(
                RoomPortal(
                    "ROOM_{}_{}".format(side, stable_bin),
                    side,
                    float(value),
                    sign * float(corridor_half_width),
                    float(width),
                )
            )
    return tuple(portals)


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
    side_ok = lateral >= float(corridor_half_width) + 0.35 if portal.side == "L" else lateral <= -float(corridor_half_width) - 0.35
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
        back_extension: float = 3.6,
        forward_depth: float = 25.0,
        lateral_half_width: float = 9.5,
        corridor_half_width: float = 1.1,
        navigation_clearance: float = 0.20,
        preferred_clearance: float = 0.32,
        clearance_cost_weight: float = 1.4,
        turn_cost_weight: float = 0.10,
        far_room_first: bool = False,
        minimum_room_stations: int = 2,
        front_station_search_limit: float = 10.0,
        virtual_gate_half_width: Optional[float] = None,
        virtual_gate_depth: float = 0.30,
    ):
        self.robot_radius = float(robot_radius)
        self.safety_margin = float(safety_margin)
        self.frontier_cluster_radius = max(0.15, float(frontier_cluster_radius))
        self.revisit_radius = max(0.2, float(revisit_radius))
        self.information_radius = max(0.5, float(information_radius))
        self.camera_weight = max(0.0, min(1.0, float(camera_weight)))
        self.back_extension = max(0.0, float(back_extension))
        self.forward_depth = max(1.0, float(forward_depth))
        self.lateral_half_width = max(1.0, float(lateral_half_width))
        self.corridor_half_width = max(0.2, float(corridor_half_width))
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

    def _navigation_fields(self, grid, task_mask=None):
        data = np.asarray(grid.data)
        occupied = data >= 50
        clearance = distance_transform_edt(~occupied) * grid.resolution
        safe = (
            (data >= 0)
            & (data < 50)
            & (clearance >= self.navigation_clearance)
        )
        if task_mask is not None and np.asarray(task_mask).shape == safe.shape:
            safe &= np.asarray(task_mask, dtype=bool)
        return safe, clearance

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
        states = [goal_state]
        while states[-1] != initial_state:
            states.append(predecessor[states[-1]])
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
        safe, clearance = self._navigation_fields(grid, task_mask)
        start = self._nearest_seed(grid, safe, robot_pose)
        goal = grid.world_to_cell(float(target[0]), float(target[1]))
        if start is None or not self._in_bounds(safe.shape, *goal) or not safe[goal]:
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
        minimum = min(float(clearance[grid.world_to_cell(*point)]) for point in path)
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
        for along_offset in portal_return_along_offsets(portal.width):
            shifted_along = float(portal.along) + along_offset
            corridor_stage = self._portal_waypoint(
                gate_center, forward_yaw, shifted_along, 0.0
            )
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
                    (corridor_stage, door_centre, room_entry, target),
                    task_mask,
                )
                if path:
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
        combined = (1.0 - self.camera_weight) * laser_gain + self.camera_weight * camera_gain
        return laser_gain, camera_gain, combined

    def _camera_look_at(self, cell, camera_unseen, grid):
        """Aim RGB-D toward the local unseen-area centroid at a real viewpoint."""
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
        mean_row = float(np.mean(rows[keep]))
        mean_column = float(np.mean(columns[keep]))
        return (
            grid.origin_x + (mean_column + 0.5) * grid.resolution,
            grid.origin_y + (mean_row + 0.5) * grid.resolution,
        )

    def path_is_safe(
        self,
        grid: Optional[GridView],
        path: Sequence[Tuple[float, float]],
        minimum_clearance: Optional[float] = None,
    ) -> bool:
        if grid is None or not path:
            return False
        data = np.asarray(grid.data)
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
        rear_rooms_unlocked: bool = False,
        front_station_along_hint: Optional[float] = None,
        completed_front_sides: Iterable[str] = (),
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
        safe, _ = self._navigation_fields(nav_grid, None)
        seed = self._nearest_seed(nav_grid, safe, robot_pose)
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
        laser_unknown = eligible & (data < 0)
        seen = np.zeros(data.shape, dtype=bool)
        if camera_seen is not None and np.asarray(camera_seen).shape == data.shape:
            seen = np.asarray(camera_seen, dtype=bool)
        camera_unseen = eligible & ~seen
        visited = tuple((float(item[0]), float(item[1])) for item in visited_targets)

        diagnostics = {
            "assignment_portal_count": 0,
            "room_task_cells": 0,
            "room_eligible_cells": 0,
            "room_camera_unseen_cells": 0,
            "room_reachable_cells": 0,
            "room_reachable_camera_unseen_cells": 0,
            "candidate_reject_counts": {
                "visited_or_too_near": 0,
                "wrong_topology": 0,
                "unconfirmed_topology": 0,
                "path_unreachable": 0,
            },
            "last_reject_reason": None,
        }

        targets = []
        live_portals = detect_room_portals(
            grid,
            gate_center,
            forward_yaw,
            extent.forward_limit,
            self.lateral_half_width,
            self.corridor_half_width,
        )
        confirmed = set(str(item) for item in confirmed_topologies)
        # Once a doorway has accumulated temporal confirmation, a sparse map
        # update must not erase the only route to an unvisited room.  Live
        # geometry wins when present; remembered geometry only fills a missing
        # stable topology and is still gated by ``confirmed_topologies``.
        portals_by_id = {portal.topology_id: portal for portal in live_portals}
        for portal in remembered_portals:
            if portal.topology_id in confirmed:
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
        station_tolerance = 1.25
        rear_minimum_separation = 10.0
        front_candidates = [
            portal
            for portal in confirmed_portals
            if 2.0 <= float(portal.along) <= self.front_station_search_limit
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
        front_sides = {portal.side for portal in front_station_portals}
        completed_sides = {
            str(side) for side in completed_front_sides if str(side) in ("L", "R")
        }
        completed_sides.update(
            portal.side
            for portal in front_station_portals
            if portal.topology_id in completed
        )
        front_rooms_complete = bool(
            front_sides == {"L", "R"}
            and completed_sides == {"L", "R"}
        )
        if topology_lock:
            assignment_portals = [
                portal
                for portal in confirmed_portals
                if portal.topology_id == str(topology_lock)
            ]
        elif rear_rooms_unlocked and front_station_along is not None:
            rear_candidates = [
                portal
                for portal in confirmed_portals
                if float(portal.along)
                >= front_station_along + rear_minimum_separation
            ]
            rear_station_along = (
                min(float(portal.along) for portal in rear_candidates)
                if rear_candidates
                else None
            )
            assignment_portals = [
                portal
                for portal in rear_candidates
                if rear_station_along is not None
                and abs(float(portal.along) - rear_station_along)
                <= station_tolerance
            ]
        elif not front_rooms_complete:
            assignment_portals = [
                portal
                for portal in front_station_portals
                if portal.side not in completed_sides
            ]
        else:
            # Both front rooms are complete, but the live node has not yet
            # finished the controlled corridor transit toward the rear pair.
            assignment_portals = []
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
                        look_at=self._camera_look_at(cell, camera_unseen, grid),
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
        # A stable sphere hint always gets camera review.  Other candidates use
        # nearest-frontier ordering; combined gain only breaks near ties.
        priority = {"SPHERE_REVIEW": 0, "CAMERA_FRONTIER": 1, "LASER_FRONTIER": 1}
        has_room_targets = any(item.topology_id != "CORRIDOR" for item in targets)
        camera_targets = [item for item in targets if item.kind == "CAMERA_FRONTIER"]
        if snapshot.camera < float(camera_target) and camera_targets:
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
            if portal.topology_id not in completed
        }
        portal_order = {
            portal.topology_id: index
            for index, portal in enumerate(
                sorted(assignment_portals, key=lambda item: (item.along, item.side))
            )
        }
        # The corridor is now a transport topology only.  It may be used by
        # A* as a connector, but it never owns a camera/laser frontier and
        # therefore cannot make the robot turn around to scan the corridor.
        if topology_lock:
            before_topology_filter = len(targets)
            targets = [
                item for item in targets
                if item.topology_id == str(topology_lock)
            ]
            diagnostics["candidate_reject_counts"]["wrong_topology"] += (
                before_topology_filter - len(targets)
            )
        else:
            targets = [
                item for item in targets
                if item.topology_id in active_station_topologies
            ]
        targets.sort(
            key=lambda item: (
                priority[item.kind],
                portal_order.get(item.topology_id, len(portal_order)),
                item.path_length,
                -item.combined_gain,
                -item.min_clearance,
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
            targets = [item for item in targets if item.topology_id not in completed]
        # Candidate discovery/ranking remains nearest-frontier based.  Only
        # the selected executable route is replaced with orientation-aware
        # A* plus a collision-checked line-of-sight shortcut.
        if targets:
            chosen_index = None
            chosen = None
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
                    )
                    )
                if path:
                    chosen_index = index
                    chosen = replace(
                        candidate,
                        path=path,
                        path_length=path_length,
                        min_clearance=minimum,
                    )
                    break
            if chosen is not None:
                targets.pop(chosen_index)
                targets.insert(0, chosen)
            else:
                diagnostics["candidate_reject_counts"]["path_unreachable"] += len(targets)
                targets = []
        if not targets:
            if diagnostics["assignment_portal_count"] == 0:
                diagnostics["last_reject_reason"] = "NO_ASSIGNMENT_PORTAL"
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
