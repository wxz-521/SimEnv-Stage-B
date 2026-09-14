#!/usr/bin/env python3
"""Regression tests for the collision-retreat decision rules.

The explorer had no retreat action: a collision, or a target held with no
displacement, only stopped the robot and blacklisted the target.  In a dead end
that leaves it wedged for ever, because every controller turns before it drives
and can therefore never choose a reverse action (run168 floor 1: wedged against
a wall with a room outstanding, ``cmd_vel`` pinned at (0, 0), ``NO_FRONTIER``).

These pin the pure parts: what counts as an impact, which way the escape points,
when the map itself has trapped the robot, and the self-return filter that keeps
the robot from marking its own cell occupied.
"""

import unittest

import numpy as np

from coverage_explorer_core import (
    collision_impact_detected,
    map_is_trapped,
    retreat_trail_target,
)
from map_floors_core import filter_self_returns


class CollisionImpactTest(unittest.TestCase):
    def test_acceleration_step_is_an_impact(self):
        # Walking baseline ~9.8, a leg into a wall spikes well past 5 m/s^2.
        self.assertTrue(
            collision_impact_detected(15.0, 9.8, 0.2, accel_delta=5.0)
        )

    def test_body_rate_spike_is_an_impact(self):
        # A corner strike can mostly yaw, with the force magnitude intact.
        self.assertTrue(
            collision_impact_detected(9.9, 9.8, 3.5, gyro_threshold=3.0)
        )

    def test_ordinary_walking_is_not_an_impact(self):
        self.assertFalse(
            collision_impact_detected(10.0, 9.8, 0.4, accel_delta=5.0)
        )

    def test_missing_sample_is_not_an_impact(self):
        self.assertFalse(collision_impact_detected(None, 9.8, 9.0))

    def test_constant_gravity_cannot_fire(self):
        # The baseline follows the constant part, so a steady reading is quiet.
        for value in (9.0, 9.8, 10.4):
            self.assertFalse(
                collision_impact_detected(value, value, 0.0, accel_delta=5.0)
            )


class RetreatTrailTest(unittest.TestCase):
    def test_picks_the_crumb_a_requested_distance_back(self):
        trail = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0)]
        self.assertEqual(
            retreat_trail_target(trail, (2.0, 0.0), 0.8), (1.0, 0.0)
        )

    def test_short_trail_falls_back_to_the_oldest_crumb(self):
        trail = [(0.0, 0.0), (0.5, 0.0)]
        self.assertEqual(
            retreat_trail_target(trail, (1.0, 0.0), 3.0), (0.0, 0.0)
        )

    def test_empty_trail_has_no_target(self):
        self.assertIsNone(retreat_trail_target([], (0.0, 0.0), 0.8))

    def test_accumulates_along_a_bent_trail_not_straight_line(self):
        # An L-shaped trail: the crumb is chosen by walked length, so the escape
        # retraces the corner instead of cutting across it.  Walking back from
        # the robot: 0.5 m to (1, 2), then 1.0 m to (1, 1), then 1.0 m to
        # (1, 0) -- which is the first crumb at least 1.6 m back.
        trail = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (1.0, 2.0)]
        self.assertEqual(
            retreat_trail_target(trail, (1.0, 2.5), 1.6), (1.0, 0.0)
        )


class MapTrappedTest(unittest.TestCase):
    def test_few_reachable_cells_with_work_left_is_trapped(self):
        self.assertTrue(map_is_trapped(3, 0.30, ["ROOM_R_43"]))

    def test_zero_start_clearance_with_work_left_is_trapped(self):
        self.assertTrue(map_is_trapped(500, 0.0, ["ROOM_R_43"]))

    def test_healthy_map_is_not_trapped(self):
        self.assertFalse(map_is_trapped(500, 0.32, ["ROOM_R_43"]))

    def test_no_outstanding_room_is_never_trapped(self):
        self.assertFalse(map_is_trapped(1, 0.0, []))

    def test_unknown_reachable_count_is_not_trapped(self):
        self.assertFalse(map_is_trapped(None, 0.0, ["ROOM_R_43"]))


class SelfReturnFilterTest(unittest.TestCase):
    def test_drops_points_on_the_robot_keeps_the_world(self):
        points = np.array(
            [
                [0.10, 0.00, 0.40],   # own leg
                [0.35, 0.35, 0.30],   # own body corner (0.49 m)
                [0.90, 0.00, 0.40],   # a real wall ahead
                [1.20, 0.30, 0.20],   # furniture
            ]
        )
        kept = filter_self_returns(points, (0.0, 0.0), 0.55)
        self.assertEqual(kept.shape, (2, 3))
        self.assertAlmostEqual(float(kept[0, 0]), 0.90)
        self.assertAlmostEqual(float(kept[1, 0]), 1.20)

    def test_empty_scan_stays_empty(self):
        self.assertEqual(filter_self_returns(np.zeros((0, 3)), (0.0, 0.0), 0.5).shape, (0, 3))

    def test_filter_follows_the_robot_pose(self):
        points = np.array([[5.0, 5.0, 0.3], [5.9, 5.0, 0.3]])
        kept = filter_self_returns(points, (5.0, 5.0), 0.55)
        self.assertEqual(kept.shape, (1, 3))
        self.assertAlmostEqual(float(kept[0, 0]), 5.9)


if __name__ == "__main__":
    unittest.main()
