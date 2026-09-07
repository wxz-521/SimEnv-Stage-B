#!/usr/bin/env python3

import math
import unittest

import numpy as np

from coverage_explorer_core import GridView

from elevator_transition_core import (
    choose_opening_heading,
    detect_wide_lobby_openings,
    entry_stall_confirms_containment,
    height_transition_complete,
    point_from_gate,
    transform_pose_between_frames,
)


class ElevatorTransitionCoreTest(unittest.TestCase):
    def test_gate_offsets_follow_corridor_frame(self):
        point = point_from_gate((1.0, 2.0, math.pi / 2.0), -4.0, 1.5)
        self.assertAlmostEqual(point[0], -0.5)
        self.assertAlmostEqual(point[1], -2.0)

    def test_search_selects_deepest_confirmed_opening(self):
        selected = choose_opening_heading(
            [(-1.0, 1.2), (-0.5, 3.4), (0.0, 2.1)], 1.8
        )
        self.assertEqual(selected, -0.5)
        self.assertIsNone(choose_opening_heading([(0.0, 1.7)], 1.8))

    def test_floor_change_requires_vertical_rise(self):
        self.assertFalse(height_transition_complete(0.55, 1.9, 2.0))
        self.assertTrue(height_transition_complete(0.55, 3.15, 2.0))

    def test_entry_stall_only_confirms_after_threshold_crossing(self):
        self.assertFalse(entry_stall_confirms_containment(0.99, 1.0))
        self.assertTrue(entry_stall_confirms_containment(1.0, 1.0))

    def test_reference_pose_transfers_between_floor_frames(self):
        transformed = transform_pose_between_frames(
            (4.0, 3.0, 0.5), (1.0, 1.0, 0.0), (10.0, 20.0, math.pi / 2.0)
        )
        self.assertAlmostEqual(transformed[0], 8.0)
        self.assertAlmostEqual(transformed[1], 23.0)
        self.assertAlmostEqual(transformed[2], 0.5 + math.pi / 2.0)

    def test_wide_elevator_opening_accepts_one_short_wall_connection(self):
        resolution = 0.1
        data = np.full((160, 160), -1, dtype=np.int16)
        grid = GridView(data, resolution, -8.0, -8.0, "map")
        # Gate faces +x; the reversed lobby axis faces -x.  Place a right-side
        # lobby wall at y=+1.1 with a 1.6 m opening around x=-4.0.  Only one
        # edge has the short wall support that an elevator facade may expose.
        wall_y = int(round((1.1 - grid.origin_y) / resolution))
        for x in np.arange(-7.0, -4.8, resolution):
            column = int((x - grid.origin_x) / resolution)
            data[wall_y - 1:wall_y + 2, column] = 100
        for x in np.arange(-4.8, -3.2, resolution):
            column = int((x - grid.origin_x) / resolution)
            data[wall_y - 1:wall_y + 10, column] = 0
        candidates = detect_wide_lobby_openings(
            grid, (0.0, 0.0), 0.0, lobby_depth=7.5,
            lateral_half_width=3.0, corridor_half_width=1.1,
            minimum_along=0.25,
        )
        self.assertTrue(candidates)
        self.assertGreaterEqual(candidates[0]["width"], 0.9)


if __name__ == "__main__":
    unittest.main()
