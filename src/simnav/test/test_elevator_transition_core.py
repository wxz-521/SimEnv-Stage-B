#!/usr/bin/env python3

import math
import unittest

from elevator_transition_core import (
    choose_opening_heading,
    height_transition_complete,
    point_from_gate,
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


if __name__ == "__main__":
    unittest.main()
