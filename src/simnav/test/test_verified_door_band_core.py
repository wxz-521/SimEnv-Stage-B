#!/usr/bin/env python3
"""Regression tests for reconnecting a room through its confirmed doorway.

run27 floor 0 stalled with the front-right room holding 11869 task cells, of
which only 260 stayed reachable, while 7654 cells were still unseen:
``room_reachable_camera_unseen_cells`` was 0, so the planner could never emit a
target that would take the robot through the door.  The cause is the
navigation map's inflation: ``safe`` demands clearance >= navigation_clearance
(0.24 m), and FAST-LIO endpoints accumulate permanently, so one stray occupied
pixel on the wall line seals the doorway and cuts the room out of ``reachable``.

``verified_door_band`` reconnects such a room, but only for a doorway the node
has already *confirmed* and only when the straight door-normal band is
known-free with clearance still above the verified-crossing threshold (0.12 m)
the accepted return path already uses.  These tests pin both halves: the narrow
confirmed door reopens, and a genuinely sealed door stays sealed.
"""

import unittest

import numpy as np

from coverage_explorer_core import (
    TaskCoveragePlanner,
    detect_room_portals,
)
from test_coverage_explorer_core import synthetic_floor


class VerifiedDoorBandTest(unittest.TestCase):
    def setUp(self):
        self.planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.24,
            back_extension=1.0,
            lateral_half_width=9.5,
        )
        self.grid = synthetic_floor(include_second_opening=False)
        portals = detect_room_portals(
            self.grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1
        )
        self.portal = next(portal for portal in portals if portal.side == "L")

    def _door_cells(self, slit):
        """Seal the left doorway, then reopen a ``slit`` metre centre opening.

        The seal spans the whole 1.0 m gap plus margin so no free path survives
        beside the slit, and the slit is sized so the residual clearance lands
        between the verified-crossing threshold (0.12 m) and the navigation
        clearance (0.24 m) -- the exact band the measured failure fell into.
        """
        grid = self.grid
        data = grid.data
        along, lateral = float(self.portal.along), float(self.portal.lateral)
        first = grid.world_to_cell(along - 0.60, lateral - 0.25)
        second = grid.world_to_cell(along + 0.60, lateral + 0.25)
        row_lo, row_hi = sorted((first[0], second[0]))
        column_lo, column_hi = sorted((first[1], second[1]))
        data[row_lo : row_hi + 1, column_lo : column_hi + 1] = 100
        if slit > 0.0:
            half = max(1, int(round(0.5 * slit / grid.resolution)))
            centre = (column_lo + column_hi) // 2
            data[
                row_lo : row_hi + 1, centre - half : centre + half + 1
            ] = 0
        return grid

    def _planner_state(self, grid):
        safe, clearance = self.planner._navigation_fields(grid, None)
        seed = self.planner._nearest_seed(grid, safe, (0.0, 0.0, 0.0))
        return safe, clearance, seed

    def test_narrow_confirmed_doorway_is_reconnected(self):
        grid = self._door_cells(slit=0.20)
        safe, clearance, seed = self._planner_state(grid)
        door = grid.world_to_cell(
            float(self.portal.along), float(self.portal.lateral)
        )
        # Precondition: the inflated navigation map cannot see through the slit,
        # which is exactly what made the room unreachable at 0.84 coverage.
        self.assertFalse(safe[door])
        distance, _predecessor = self.planner._reachable(safe, seed)
        room_cell = grid.world_to_cell(5.0, 5.0)
        self.assertLess(distance[room_cell], 0, "room must start unreachable")

        band = self.planner.verified_door_band(
            grid, (0.0, 0.0), 0.0, [self.portal], clearance
        )
        self.assertGreater(int(np.count_nonzero(band)), 0)
        distance, _predecessor = self.planner._reachable(safe | band, seed)
        self.assertGreaterEqual(
            distance[room_cell], 0, "confirmed doorway must reconnect the room"
        )

    def test_fully_sealed_doorway_is_not_opened(self):
        grid = self._door_cells(slit=0.0)
        _safe, clearance, _seed = self._planner_state(grid)
        band = self.planner.verified_door_band(
            grid, (0.0, 0.0), 0.0, [self.portal], clearance
        )
        self.assertEqual(
            int(np.count_nonzero(band)),
            0,
            "a solid wall must never be opened by the verified-door band",
        )


if __name__ == "__main__":
    unittest.main()
