#!/usr/bin/env python3
"""Pure geometry helpers for the post-exploration elevator transition."""

import math

import numpy as np


def normalize_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def point_from_gate(gate, forward_offset, lateral_offset=0.0):
    """Return a world point in the entrance-gate tangent/normal frame."""
    x, y, yaw = (float(value) for value in gate[:3])
    return (
        x + forward_offset * math.cos(yaw) - lateral_offset * math.sin(yaw),
        y + forward_offset * math.sin(yaw) + lateral_offset * math.cos(yaw),
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
                "width": width,
                "support": max(before_ratio, after_ratio),
                "pose": (centre[0], centre[1], heading),
            })
    return tuple(sorted(candidates, key=lambda item: (-item["width"], -item["support"])))
