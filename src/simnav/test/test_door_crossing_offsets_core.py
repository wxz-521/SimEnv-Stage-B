#!/usr/bin/env python3
"""Regression tests for the widened doorway-crossing along search.

run48 floor 0 locked the opposite room as ROOM_L_47 (4015 reachable
camera-unseen cells, campaign 3/4 rooms done) and then produced no target for
four minutes: every candidate among 128 camera and 11 laser frontiers was
rejected as ``path_unreachable``, the verified door band held 0 cells, and
``actionable_portals`` held only the locked id.  Portal ids are 0.5 m bins that
drift as SLAM refines a wall, and the crossing search only looked +/-0.30 m
inside the reported doorway, so it could never find an opening that sat a metre
or two away along the same wall.
"""

import unittest

import numpy as np

from coverage_explorer_core import (
    GridView,
    RoomPortal,
    TaskCoveragePlanner,
    door_crossing_along_offsets,
    portal_return_along_offsets,
)


class DoorCrossingOffsetsTest(unittest.TestCase):
    def test_narrow_search_is_preserved_first(self):
        narrow = portal_return_along_offsets(1.4)
        wide = door_crossing_along_offsets(1.4, 2.5, 0.5)
        self.assertEqual(wide[: len(narrow)], narrow)

    def test_wide_search_reaches_two_and_a_half_metres(self):
        wide = door_crossing_along_offsets(1.4, 2.5, 0.5)
        self.assertAlmostEqual(max(abs(value) for value in wide), 2.5)

    def test_search_is_symmetric_and_ordered_centre_out(self):
        wide = door_crossing_along_offsets(1.4, 1.0, 0.5)
        for offset in (0.5, 1.0):
            self.assertIn(-offset, wide)
            self.assertIn(offset, wide)
        self.assertLess(wide.index(-0.5), wide.index(-1.0))

    def test_zero_wide_search_keeps_the_doorway_bounded_search(self):
        narrow = portal_return_along_offsets(1.4)
        self.assertEqual(door_crossing_along_offsets(1.4, 0.0, 0.5), narrow)


class ShiftedDoorCrossingTest(unittest.TestCase):
    """The corridor wall opening sits 2 m from the portal's reported along."""

    RESOLUTION = 0.10
    OPENING_ALONG = 4.0

    def _planner(self, wide_search):
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            preferred_clearance=0.10,
        )
        planner.door_crossing_wide_search = wide_search
        planner.door_crossing_scan_step = 0.5
        return planner

    def _wall_grid(self):
        # Corridor along +x at y=0, right-side room at y < -1.1, wall on the
        # y=-1.1 line with a 0.9 m opening centred on OPENING_ALONG.
        data = np.zeros((160, 200), dtype=np.int16)
        grid = GridView(data, self.RESOLUTION, -6.0, -8.0)
        wall_row = grid.world_to_cell(0.0, -1.10)[0]
        for column in range(0, 200):
            x = grid.origin_x + (column + 0.5) * self.RESOLUTION
            if abs(x - self.OPENING_ALONG) <= 0.45:
                continue
            data[wall_row, column] = 100
            data[wall_row + 1, column] = 100
        return grid

    def _crossing(self, wide_search):
        grid = self._wall_grid()
        planner = self._planner(wide_search)
        # The portal is reported 2 m short of the real opening: an id/bin drift.
        portal = RoomPortal("ROOM_L_47", "R", 2.0, -1.10, 1.4)
        path, _length, _minimum, depth = planner.navigation_path_through_portal(
            grid,
            (0.0, 0.0, 0.0),
            (0.0, 0.0),
            0.0,
            portal,
            (self.OPENING_ALONG, -4.0),
        )
        return planner, path, depth

    def test_bounded_search_alone_cannot_cross_a_shifted_portal(self):
        _planner, path, _depth = self._crossing(0.0)
        self.assertFalse(
            path,
            "documents the run48 failure: a +/-0.30 m search cannot find a "
            "2 m off opening",
        )

    def test_wide_search_finds_the_real_opening(self):
        planner, path, depth = self._crossing(2.5)
        self.assertTrue(path, "the widened search must recover the crossing")
        # The recovered offset is well beyond the old +/-0.30 m doorway-bounded
        # search; the exact value depends on where the wall opening begins.
        self.assertIsNotNone(planner.last_door_crossing_offset)
        self.assertGreater(abs(planner.last_door_crossing_offset), 0.30)
        self.assertIsNotNone(depth)


if __name__ == "__main__":
    unittest.main()
