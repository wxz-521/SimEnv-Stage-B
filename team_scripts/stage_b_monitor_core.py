"""Pure structural checks for Stage B dynamic acceptance metrics."""

import math


def confirmed_hypotheses(hypotheses):
    """Return actionable door hypotheses, excluding unconfirmed scan staging."""
    return [
        item
        for item in hypotheses
        if item.get("status", "CONFIRMED") == "CONFIRMED"
    ]


def doorway_station_structure(hypotheses, pairing_tolerance=2.0, minimum_station_separation=2.0):
    """Require four confirmed doors to form two opposing left/right stations.

    The check uses only the reported door centers, side labels, and normals. It
    does not use generated layout metadata or absolute room coordinates.
    """
    hypotheses = confirmed_hypotheses(hypotheses)
    if len(hypotheses) != 4:
        return {"valid": False, "reason": "door_count", "pair_errors": []}

    groups = {"-1": [], "+1": []}
    first_normal = None
    for item in hypotheses:
        try:
            side_value = float(item["side"])
            center = (float(item["center"][0]), float(item["center"][1]))
            normal = float(item["normal_yaw"])
        except (KeyError, TypeError, ValueError, IndexError):
            return {"valid": False, "reason": "malformed_hypothesis", "pair_errors": []}
        if abs(side_value) < 0.5:
            return {"valid": False, "reason": "missing_side", "pair_errors": []}
        side = "+1" if side_value > 0.0 else "-1"
        groups[side].append(center)
        if first_normal is None:
            first_normal = normal

    if any(len(values) != 2 for values in groups.values()):
        return {"valid": False, "reason": "unpaired_sides", "pair_errors": []}

    axis = (-math.sin(first_normal), math.cos(first_normal))
    stations = {
        side: sorted(axis[0] * center[0] + axis[1] * center[1] for center in values)
        for side, values in groups.items()
    }
    pair_errors = [abs(left - right) for left, right in zip(stations["-1"], stations["+1"])]
    separation = abs(stations["-1"][1] - stations["-1"][0])
    valid = max(pair_errors, default=float("inf")) <= pairing_tolerance and separation >= minimum_station_separation
    reason = "ok" if valid else "station_pairing"
    return {
        "valid": valid,
        "reason": reason,
        "pair_errors": [round(value, 4) for value in pair_errors],
        "station_separation": round(separation, 4),
        "side_counts": {side: len(values) for side, values in groups.items()},
    }
