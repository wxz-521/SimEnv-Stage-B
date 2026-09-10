#!/usr/bin/env python3

import math
import unittest

from coverage_explorer_core import corridor_countersteer_heading


class DoorSearchCountersteerTest(unittest.TestCase):
    def test_right_offset_turns_left(self):
        yaw = math.pi / 2.0
        self.assertGreater(
            corridor_countersteer_heading(yaw, -0.40, 1.30, 0.80), yaw
        )

    def test_left_offset_turns_right(self):
        yaw = math.pi / 2.0
        self.assertLess(
            corridor_countersteer_heading(yaw, 0.40, 0.80, 1.30), yaw
        )

    def test_total_correction_remains_bounded(self):
        yaw = math.pi / 2.0
        result = corridor_countersteer_heading(yaw, -2.0, 5.0, 0.3)
        self.assertAlmostEqual(result, yaw + 0.12)


if __name__ == "__main__":
    unittest.main()
