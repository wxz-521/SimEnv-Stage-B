#!/usr/bin/env python3
"""The mission speed ceiling is 0.60 m/s everywhere.

run70 tipped over at startup (imu_roll -> pi) while being commanded 0.90 m/s
from a standstill.  Three separate parameters fed 0.90 into the transit and the
lift approach, so the ceiling is enforced at the single point where each node
publishes a velocity command, not only in the parameters.
"""

import unittest

from coverage_explorer_core import clamp_linear_speed as explorer_clamp
from elevator_transition_core import clamp_linear_speed as elevator_clamp


class SpeedClampTest(unittest.TestCase):
    def test_above_the_ceiling_is_clamped(self):
        for clamp in (explorer_clamp, elevator_clamp):
            self.assertAlmostEqual(clamp(0.90, 0.60), 0.60)

    def test_below_the_ceiling_is_untouched(self):
        for clamp in (explorer_clamp, elevator_clamp):
            self.assertAlmostEqual(clamp(0.35, 0.60), 0.35)

    def test_reverse_is_clamped_symmetrically(self):
        for clamp in (explorer_clamp, elevator_clamp):
            self.assertAlmostEqual(clamp(-0.90, 0.60), -0.60)

    def test_zero_ceiling_stops_the_robot(self):
        for clamp in (explorer_clamp, elevator_clamp):
            self.assertAlmostEqual(clamp(0.5, 0.0), 0.0)


if __name__ == "__main__":
    unittest.main()
