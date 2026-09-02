#!/usr/bin/env python3
"""Pure geometry helpers for the post-exploration elevator transition."""

import math


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
