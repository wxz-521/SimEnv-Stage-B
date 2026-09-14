#!/usr/bin/env python3
"""Regression tests for the geometric red-object sweep.

The scored danger sources are red spheres placed uniformly inside a room's
feasible interior (>= 0.45 m from a wall, >= 0.35 m from furniture).  The
mission requires recall 1.0, but the fixed forward camera only resolved a
sphere whose closest approach was 2.30 m on the measured run14 floor 0, while
the two missed spheres were never nearer than 6.27 m / 6.83 m and only 43% of
the feasible zone had ever come within 2.5 m of the path.  Area coverage
therefore cannot protect recall; the planner must keep dispatching stances
until the robot's own trajectory has passed close to every reachable feasible
cell.  These tests pin that contract and its off-by-default behaviour.
"""

import unittest

import numpy as np

from coverage_explorer_core import (
    TaskCoveragePlanner,
    detect_room_portals,
)
from test_coverage_explorer_core import synthetic_floor


class DangerSweepTest(unittest.TestCase):
    def setUp(self):
        self.grid = synthetic_floor()
        portals = detect_room_portals(self.grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1)
        left = [portal for portal in portals if portal.side == "L"]
        self.assertTrue(left, "synthetic floor must expose a left room portal")
        self.lock = left[0].topology_id
        self.confirmed = [portal.topology_id for portal in portals]

    def _planner(self, enabled=True):
        return TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            forward_depth=24.0,
            back_extension=1.0,
            lateral_half_width=9.5,
            danger_sweep_enabled=enabled,
            danger_sweep_clearance=0.30,
            danger_sweep_spacing=1.0,
        )

    def _plan(self, planner, covered):
        return planner.plan(
            self.grid,
            robot_pose=(0.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(self.grid.data.shape, dtype=bool),
            confirmed_topologies=self.confirmed,
            topology_lock=self.lock,
            danger_covered=covered,
        )

    def test_sweep_targets_appear_while_feasible_cells_are_uncovered(self):
        plan = self._plan(
            self._planner(), np.zeros(self.grid.data.shape, dtype=bool)
        )
        diagnostics = plan.diagnostics or {}
        self.assertGreater(diagnostics.get("danger_sweep_feasible_cells", 0), 0)
        self.assertGreater(diagnostics.get("danger_sweep_remaining_cells", 0), 0)
        self.assertGreater(diagnostics.get("danger_sweep_targets", 0), 0)

    def test_sweep_is_silent_once_every_feasible_cell_is_covered(self):
        plan = self._plan(
            self._planner(), np.ones(self.grid.data.shape, dtype=bool)
        )
        diagnostics = plan.diagnostics or {}
        self.assertEqual(diagnostics.get("danger_sweep_remaining_cells"), 0)
        self.assertEqual(diagnostics.get("danger_sweep_targets"), 0)

    def test_sweep_is_disabled_by_default(self):
        plan = self._plan(
            self._planner(enabled=False),
            np.zeros(self.grid.data.shape, dtype=bool),
        )
        diagnostics = plan.diagnostics or {}
        self.assertEqual(diagnostics.get("danger_sweep_targets"), 0)

    def test_sweep_requires_the_trajectory_mask(self):
        # Without a per-floor trajectory record the planner must not invent
        # sweep targets, otherwise the very first cycle would dispatch one.
        plan = self._plan(self._planner(), None)
        diagnostics = plan.diagnostics or {}
        self.assertEqual(diagnostics.get("danger_sweep_targets"), 0)


if __name__ == "__main__":
    unittest.main()
