#!/usr/bin/env python3
"""Regression test for the corridor-exhaustion deadlock.

Measured on run17_endtoend_084 (0.84 coverage, three_floor) the explorer
finished the corridor and the front room pair, then parked for ever with
``last_plan_reason=NO_FRONTIER``: ``actionable_portals`` was 16, camera
coverage was 0.414 against a 0.84 target, one room still had 5665 unseen
cells, and ``cmd_vel`` was exactly 0 for 731 consecutive telemetry samples.

The cause is structural.  In CORRIDOR ownership the planner admits lidar
frontiers only, so once the laser has mapped every reachable free cell there
is no lidar frontier left -- and every remaining executable target is a
camera viewpoint owned by a room that still owes RGB-D coverage.  Those were
discarded, the candidate pool became empty, and no later stage could put a
target back.
"""

import unittest

import numpy as np

from coverage_explorer_core import (
    TaskCoveragePlanner,
    detect_room_portals,
)
from test_coverage_explorer_core import synthetic_floor


class CorridorExhaustionTest(unittest.TestCase):
    def setUp(self):
        self.grid = synthetic_floor()
        self.portals = detect_room_portals(
            self.grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1
        )
        self.assertTrue(self.portals)
        self.confirmed = [portal.topology_id for portal in self.portals]

    def _mapped_floor(self):
        """A floor whose lidar map holds no unknown cell any more.

        ``synthetic_floor`` leaves the un-boxed surround unknown, so lidar
        frontiers would still exist and mask the deadlock.  Filling them makes
        the map fully known, which is exactly the state the measured run
        reached: the laser had mapped every reachable cell, so the lidar
        frontier pool was empty while rooms still owed camera coverage.
        """
        grid = synthetic_floor()
        grid.data[grid.data < 0] = 0
        return grid

    def _planner(self):
        return TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            forward_depth=24.0,
            back_extension=1.0,
            lateral_half_width=9.5,
        )

    def _plan(self, grid, **overrides):
        kwargs = dict(
            grid=grid,
            robot_pose=(0.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            # Nothing has been seen by the camera, so every room still owes
            # RGB-D coverage; the lidar map is already complete.
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            confirmed_topologies=self.confirmed,
            topology_lock=None,
        )
        kwargs.update(overrides)
        return self._planner().plan(**kwargs)

    def test_corridor_exhaustion_does_not_deadlock_on_unfinished_rooms(self):
        grid = self._mapped_floor()
        plan = self._plan(grid)
        self.assertIsNotNone(
            plan.target,
            "corridor lidar exhaustion must not deadlock while rooms still "
            "owe camera coverage (reason={})".format(plan.reason),
        )
        self.assertEqual(plan.target.kind, "CAMERA_FRONTIER")
        self.assertNotEqual(plan.target.topology_id, "CORRIDOR")
        self.assertTrue(plan.target.path)

    def test_completed_rooms_are_not_re_dispatched(self):
        # Every portal is already complete, so the camera fallback must not
        # march the robot back into finished rooms; NO_FRONTIER is correct.
        completed = tuple(portal.topology_id for portal in self.portals)
        plan = self._plan(self._mapped_floor(), completed_topologies=completed)
        self.assertIsNone(plan.target)


if __name__ == "__main__":
    unittest.main()
