#!/usr/bin/env python3
"""Pure geometry helpers for the post-exploration elevator transition."""

import math

import numpy as np


def normalize_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def door_frame_offset(point, doorway):
    """Return ``(along, lateral)`` of a world point in the doorway frame.

    ``along`` grows through the doorway into the car and ``lateral`` is the
    sideways offset from the door centre line.  Boarding must only be committed
    from the centre line: the first standalone lift test drifted to
    lateral +0.49 m, scraped the shaft wall beside the 1.70 m opening, and
    pushed there for 35 s without moving.
    """
    x, y, yaw = (float(value) for value in doorway[:3])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    dx = float(point[0]) - x
    dy = float(point[1]) - y
    return (dx * cosine + dy * sine, -dx * sine + dy * cosine)


def point_from_gate(gate, forward_offset, lateral_offset=0.0):
    """Return a world point in the entrance-gate tangent/normal frame."""
    x, y, yaw = (float(value) for value in gate[:3])
    return (
        x + forward_offset * math.cos(yaw) - lateral_offset * math.sin(yaw),
        y + forward_offset * math.sin(yaw) + lateral_offset * math.cos(yaw),
    )


def portal_staging_point(gate, portal):
    """Return the corridor-side point aligned with a mapped portal."""
    gate_x, gate_y, gate_yaw = (float(value) for value in gate[:3])
    portal_x, portal_y = float(portal[0]), float(portal[1])
    cosine, sine = math.cos(gate_yaw), math.sin(gate_yaw)
    along = (portal_x - gate_x) * cosine + (portal_y - gate_y) * sine
    return (
        gate_x + along * cosine,
        gate_y + along * sine,
    )


def planar_distance(first, second):
    return math.hypot(float(first[0]) - float(second[0]), float(first[1]) - float(second[1]))


def target_heading(pose, target):
    return math.atan2(float(target[1]) - float(pose[1]), float(target[0]) - float(pose[0]))


def choose_opening_heading(
    samples, minimum_clearance, preferred_heading=None, preferred_window=0.45
):
    """Choose a scan opening, preferring the corridor-normal direction."""
    finite = [
        (float(yaw), float(clearance))
        for yaw, clearance in samples
        if math.isfinite(clearance) and float(clearance) >= float(minimum_clearance)
    ]
    if not finite:
        return None
    if preferred_heading is not None:
        near = [
            item for item in finite
            if abs(normalize_angle(item[0] - preferred_heading)) <= float(preferred_window)
        ]
        if near:
            return min(
                near,
                key=lambda item: (
                    abs(normalize_angle(item[0] - preferred_heading)),
                    -item[1],
                ),
            )[0]
    return max(finite, key=lambda item: item[1])[0]


def height_transition_complete(start_z, current_z, minimum_rise):
    return float(current_z) - float(start_z) >= float(minimum_rise)


def entry_stall_confirms_containment(travelled, minimum_entry_progress):
    """Treat a post-threshold physical stop as a completed elevator entry."""
    return float(travelled) >= float(minimum_entry_progress)


def entry_blocked_retry_allowed(travelled, minimum_progress, retries, retry_limit):
    """Whether a short-travel front stop during elevator entry should be retried.

    run49 completed floors 0 and 1 (4 rooms each) and then failed the whole
    mission entering the car on the floor-1 return leg:
    ``ELEVATOR_PATH_BLOCKED_AFTER_0.93M`` with a 0.27 m front clearance -- 7 cm
    short of ``minimum_entry_progress``.  The scene's ``elevator_floor_1`` door
    starts closed (``initial_open: false``) and animates open, so an early
    contact is usually the door or jamb rather than a failed entry.  Retry a
    bounded number of times, with a short reverse, before declaring the entry
    blocked.
    """
    if float(travelled) >= float(minimum_progress):
        return False
    return int(retries) < int(retry_limit)


def direct_entry_applies(distance, max_distance, enabled, blocked_seconds=0.0,
                         fallback_seconds=25.0):
    """Whether a target should be driven to directly instead of by A*.

    The lift approach, the post-exit corridor move and the car entry are short
    line-of-sight manoeuvres through a doorway whose free span has just been
    measured.  Planning them tied the mission to a map that is still empty on a
    freshly entered floor: run52 floor 2 burned 654 failed A* attempts and five
    minutes crawling 2 m.  A blocked direct run still falls back to A* after
    ``fallback_seconds`` so a real obstacle in the way is not ignored.
    """
    if not enabled:
        return False
    if float(distance) > float(max_distance):
        return False
    if float(blocked_seconds) > float(fallback_seconds):
        return False
    return True


def direct_alignment_ready(heading_error, tolerance):
    """Whether the robot may drive straight yet, or must turn on the spot first.

    Driving forward while badly misaligned is what arced the robot into the wall
    it was facing during run52's floor-2 topology drive.
    """
    return abs(float(heading_error)) <= float(tolerance)


def tilt_fault_state(roll, pitch, roll_limit, pitch_limit, tilted_since, now, persist):
    """Classify a tilt sample as ``ok``, ``pending`` or ``fault``.

    A real tip-over holds the body tilted against gravity, so the test requires
    the tilt to persist.  A single-sample metric spike must not end the mission:
    run63 faulted with ROBOT_ROLLED when the world roll hit 0.45 rad for one
    sample during a hard turn while the body IMU never left 0.12 rad.
    """
    tilted = abs(float(roll)) > float(roll_limit) or abs(float(pitch)) > float(
        pitch_limit
    )
    if not tilted:
        return "ok"
    if tilted_since is None:
        return "pending"
    if float(now) - float(tilted_since) >= float(persist):
        return "fault"
    return "pending"


def establish_budget_exceeded(started_wall, now_wall, timeout):
    """Whether the floor-topology drive has used up its real-time budget.

    The corridor point after the lift is a soft hint: with one 2D map per floor
    the floor's own explorer maps the corridor anyway, so a route that cannot be
    planned must not hold the mission inside the lift lobby.  run52 floor 2 spent
    more than five minutes there with 654 failed A* attempts and a 0.45 m/s
    open-loop crawl, against 107 s on floor 1.
    """
    if started_wall is None:
        return False
    return (float(now_wall) - float(started_wall)) > float(timeout)


def elevator_door_id(floor_index, prefix="elevator_floor"):
    """Scene door id serving one elevator floor."""
    return "{}_{}".format(str(prefix), int(floor_index))


def transform_pose_between_frames(point, source_pose, target_pose):
    """Transform an x/y/yaw pose using a paired robot pose in both frames."""
    offset_yaw = normalize_angle(float(target_pose[2]) - float(source_pose[2]))
    cosine, sine = math.cos(offset_yaw), math.sin(offset_yaw)
    dx = float(point[0]) - float(source_pose[0])
    dy = float(point[1]) - float(source_pose[1])
    return (
        float(target_pose[0]) + cosine * dx - sine * dy,
        float(target_pose[1]) + sine * dx + cosine * dy,
        normalize_angle(float(point[2]) + offset_yaw),
    )


def detect_wide_lobby_openings(
    grid,
    gate_center,
    forward_yaw,
    lobby_depth=8.0,
    lateral_half_width=4.5,
    corridor_half_width=1.1,
    minimum_width=0.9,
    maximum_width=3.8,
    minimum_along=2.5,
    preferred_heading=None,
):
    """Find elevator-sized lobby openings without requiring two long jambs.

    The room detector requires strong wall support on both sides of a narrow
    gap.  Elevator fronts can be attached to only a short wall segment, so
    this variant accepts one supported edge while retaining the same
    wall-crossing and free-passage checks.  Temporal confirmation remains the
    caller's responsibility.
    """
    data = np.asarray(grid.data)
    rows, columns = np.indices(data.shape, dtype=np.float64)
    reverse_yaw = normalize_angle(float(forward_yaw) + math.pi)
    cosine, sine = math.cos(reverse_yaw), math.sin(reverse_yaw)
    x = float(grid.origin_x) + (columns + 0.5) * float(grid.resolution)
    y = float(grid.origin_y) + (rows + 0.5) * float(grid.resolution)
    dx = x - float(gate_center[0])
    dy = y - float(gate_center[1])
    along = dx * cosine + dy * sine
    lateral = -dx * sine + dy * cosine
    known_free = (data >= 0) & (data < 50)
    search = (
        (along >= float(minimum_along))
        & (along <= float(lobby_depth))
        & (np.abs(lateral) <= float(lateral_half_width))
    )
    resolution = float(grid.resolution)
    bin_count = max(1, int(math.ceil(float(lobby_depth) / resolution)))
    candidates = []
    for side, sign in (("L", 1.0), ("R", -1.0)):
        signed = sign * lateral
        crossing = (
            known_free
            & search
            & (signed >= float(corridor_half_width) - 0.14)
            & (signed <= float(corridor_half_width) + 0.14)
        )
        free_bins = np.floor(along[crossing] / resolution).astype(np.int64)
        free_bins = free_bins[(free_bins >= 0) & (free_bins < bin_count)]
        free_counts = np.bincount(free_bins, minlength=bin_count)
        wall = (
            (data >= 50)
            & search
            & (signed >= float(corridor_half_width) - 0.24)
            & (signed <= float(corridor_half_width) + 0.24)
        )
        wall_bins_array = np.floor(along[wall] / resolution).astype(np.int64)
        wall_bins_array = wall_bins_array[
            (wall_bins_array >= 0) & (wall_bins_array < bin_count)
        ]
        wall_counts = np.bincount(wall_bins_array, minlength=bin_count)
        active = np.nonzero((free_counts > 0) & (wall_counts == 0))[0]
        groups = []
        for raw_index in active:
            index = int(raw_index)
            if not groups or index - groups[-1][-1] > 2:
                groups.append([index])
            else:
                groups[-1].append(index)
        wall_bins = set(int(value) for value in wall_bins_array)
        support_bins = max(2, int(math.ceil(0.45 / resolution)))
        for group in groups:
            start, end = int(group[0]), int(group[-1])
            width = (end - start + 1) * resolution
            if not float(minimum_width) <= width <= float(maximum_width):
                continue
            before = range(start - support_bins, start)
            after = range(end + 1, end + 1 + support_bins)
            before_ratio = sum(item in wall_bins for item in before) / float(support_bins)
            after_ratio = sum(item in wall_bins for item in after) / float(support_bins)
            if max(before_ratio, after_ratio) < 0.20:
                continue
            centre_along = (start + end + 1) * 0.5 * resolution
            passage = []
            for depth in (0.25, 0.50, 0.80):
                point_x = (
                    float(gate_center[0])
                    + cosine * centre_along
                    - sine * sign * (float(corridor_half_width) + depth)
                )
                point_y = (
                    float(gate_center[1])
                    + sine * centre_along
                    + cosine * sign * (float(corridor_half_width) + depth)
                )
                row, column = grid.world_to_cell(point_x, point_y)
                if 0 <= row < data.shape[0] and 0 <= column < data.shape[1]:
                    passage.append(0 <= int(data[row, column]) < 50)
            if len(passage) < 3 or sum(passage) < 2:
                continue
            heading = normalize_angle(reverse_yaw + sign * math.pi / 2.0)
            if (
                preferred_heading is not None
                and abs(normalize_angle(heading - float(preferred_heading))) > 0.45
            ):
                continue
            centre = point_from_gate(
                (gate_center[0], gate_center[1], reverse_yaw),
                centre_along,
                sign * float(corridor_half_width),
            )
            candidates.append({
                "side": side,
                "along": centre_along,
                # Match coverage_explorer's reversed lobby-frame lateral
                # coordinate so both nodes apply the same side filter.
                "lateral": sign * float(corridor_half_width),
                "width": width,
                "support": max(before_ratio, after_ratio),
                "pose": (centre[0], centre[1], heading),
            })
    return tuple(sorted(candidates, key=lambda item: (-item["width"], -item["support"])))


def clamp_linear_speed(value, maximum):
    """Clamp a commanded linear speed into [-maximum, +maximum].

    Shared with the explorer: the mission stays at or below 0.60 m/s everywhere
    (transit, lift approach, door crossings) until the three-floor run is stable.
    """
    limit = max(0.0, float(maximum))
    return max(-limit, min(limit, float(value)))
