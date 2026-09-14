#!/usr/bin/env python3
"""Height-separated 2D map bookkeeping.

The occupancy node used to accumulate every floor into a single 2D grid and to
``fill(-1)`` it whenever the robot changed floor.  Two consequences showed up in
the mission logs: a floor's topology had to be *reused* rather than measured
(the map it came from was gone), and portal ids drifted because each floor was
re-derived from a wiped map.  Keeping one 2D grid per floor - separated by the
same robot-relative height band that already excluded other floors' geometry -
lets every floor keep its own stable map, and lets the ground floor still be
navigable when the robot rides back down for the return to spawn.

This module holds the pure parts so they can be tested without ROS.
"""

def point_floor_mask(point_z, floor_z, minimum, maximum):
    """Vectorised height-band test for the points of one scan.

    ``minimum``/``maximum`` are the robot-relative band the sensor uses, so a
    point belongs to the active floor when its height above that floor's level
    falls inside the band.  Until a floor's own level has been observed
    (``floor_z is None``) every point is accepted, which preserves the original
    single-map behaviour during startup.
    """
    import numpy as np

    heights = np.asarray(point_z, dtype=float)
    if floor_z is None:
        return np.ones(heights.shape, dtype=bool)
    dz = heights - float(floor_z)
    return (dz >= float(minimum)) & (dz <= float(maximum))


def ensure_floor_grid(grids, floor_index, shape, fill_value=-1, dtype=None):
    """Return the grid for ``floor_index``, creating an empty one if needed.

    Never clears an existing floor: retaining it is the whole point of the
    height-separated representation.
    """
    import numpy as np

    key = int(floor_index)
    if key not in grids:
        grids[key] = np.full(shape, fill_value, dtype=dtype or np.int8)
        return grids[key], True
    return grids[key], False


def floor_map_report(grids, active_floor, floor_z=None, floor_height=2.6):
    """Per-floor summary used for the ``/simnav/map_floors`` metadata topic.

    Each entry carries the floor's height, so a consumer can address the maps by
    height rather than by publication order.
    """
    import numpy as np

    floor_z = dict(floor_z or {})
    levels = [float(value) for value in floor_z.values() if value is not None]
    base = min(levels) if levels else 0.0
    report = []
    for floor_index in sorted(int(key) for key in grids):
        data = np.asarray(grids[floor_index])
        known = int(np.count_nonzero(data >= 0))
        occupied = int(np.count_nonzero(data >= 50))
        level = floor_z.get(floor_index)
        if level is None:
            height = float(floor_index) * float(floor_height)
            level_known = False
        else:
            height = float(level) - base
            level_known = True
        report.append(
            {
                "floor_index": floor_index,
                "height": round(height, 3),
                "height_source": "observed" if level_known else "nominal",
                "active": bool(floor_index == int(active_floor)),
                "known_cells": known,
                "free_cells": known - occupied,
                "occupied_cells": occupied,
                "complete": bool(known > 0 and occupied > 0),
            }
        )
    return report


def observed_floor_level(robot_z_samples):
    """Lowest observed robot height on a floor, used as that floor's level.

    The robot stands on the floor, so its minimum height over a visit is the
    most robust available estimate of the floor level; averages drift with the
    metric z drift that FAST-LIO accumulates.
    """
    values = [float(value) for value in robot_z_samples if value is not None]
    if not values:
        return None
    return min(values)


def floor_index_for_height(point_z, base_z, floor_height):
    """Nearest floor index for an absolute mapped height."""
    if floor_height <= 0.0:
        return 0
    return int(round((float(point_z) - float(base_z)) / float(floor_height)))


def height_band_is_sane(minimum, maximum, floor_height):
    """Whether a band cannot leak geometry from an adjacent floor.

    The band is robot-relative, so a band that reaches ``floor_height`` would
    start absorbing the next floor's walls into this floor's grid.
    """
    return bool(
        float(maximum) < float(floor_height) and float(minimum) > -float(floor_height)
    )
