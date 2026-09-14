#!/usr/bin/env python3
"""Regression tests for navigation speckle removal.

run29 floor 0 completed all four rooms and then faulted in RETURN_TO_ELEVATOR
with ``ROUTE_UNREACHABLE_RETURN_TO_ELEVATOR_AFTER_60S``.  Connectivity analysis
of the saved exploration map showed the corridor itself is *not* cut: with no
inflation the robot's cell reaches the elevator staging point (5.75, -0.35)
through 64597 cells, and still through 50639 cells at 0.20 m.  At the node's
own 0.30 m clearance the reachable set collapsed to 28732 cells with the staging
point unreachable -- because single-frame FAST-LIO endpoint speckle had been
inflated into a wall.

``_navigation_fields`` now drops isolated one- and two-cell occupied
components, which reconnected that pocket to 47458 cells with the staging point
reachable again.  These tests pin both halves of the contract: speckle must not
seal a passage the robot can drive through, and a real obstacle must still
block.
"""

import unittest

import numpy as np
from collections import deque

from coverage_explorer_core import GridView, TaskCoveragePlanner


class NavigationDespeckleTest(unittest.TestCase):
    ROWS = 28
    COLUMNS = 40
    CHANNEL_TOP = 10          # free channel spans rows CHANNEL_TOP..CHANNEL_TOP+6
    CENTRE_ROW = 13

    def _planner(self):
        return TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.30,
        )

    def _channel_grid(self, occupied_cells=()):
        data = np.full((self.ROWS, self.COLUMNS), 100, dtype=np.int16)
        data[
            self.CHANNEL_TOP : self.CHANNEL_TOP + 7, 1 : self.COLUMNS - 1
        ] = 0
        for row, column in occupied_cells:
            data[row, column] = 100
        return GridView(data, 0.1, 0.0, 0.0)

    @staticmethod
    def _connected(safe, start, goal):
        seen = np.zeros_like(safe)
        seen[start] = True
        queue = deque([start])
        while queue:
            row, column = queue.popleft()
            for d_row, d_column in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n_row, n_column = row + d_row, column + d_column
                if (
                    0 <= n_row < safe.shape[0]
                    and 0 <= n_column < safe.shape[1]
                    and safe[n_row, n_column]
                    and not seen[n_row, n_column]
                ):
                    seen[n_row, n_column] = True
                    queue.append((n_row, n_column))
        return bool(seen[goal])

    def test_speckle_does_not_seal_a_drivable_passage(self):
        # One isolated occupied cell in the middle of a 0.7 m channel.  Inflated
        # by 0.30 m it locally removes every safe row, which is exactly the
        # mechanism that sealed the run29 corridor.
        grid = self._channel_grid({(self.CENTRE_ROW, 20)})
        planner = self._planner()
        safe, _clearance = planner._navigation_fields(grid, None, despeckle=True)
        self.assertTrue(
            self._connected(
                safe, (self.CENTRE_ROW, 6), (self.CENTRE_ROW, self.COLUMNS - 7)
            ),
            "a single isolated occupied cell must not seal the passage",
        )
        # The speckle cell itself is still occupied: this removes noise from the
        # clearance field, it does not open mapped obstacles.
        self.assertFalse(safe[self.CENTRE_ROW, 20])

    def test_despeckle_is_opt_in_and_off_for_the_coverage_plan(self):
        # Routing opts in; the coverage plan must keep the untouched semantics.
        # Turning it on for the plan changed the explorer's behaviour (run30
        # spun at the ROOM_L_43 doorway for 297 sim s where run29 finished the
        # room in 62 s), so the default has to stay off.
        grid = self._channel_grid({(self.CENTRE_ROW, 20)})
        planner = self._planner()
        safe_default, _clearance = planner._navigation_fields(grid, None)
        self.assertFalse(
            self._connected(
                safe_default, (self.CENTRE_ROW, 6), (self.CENTRE_ROW, self.COLUMNS - 7)
            ),
            "the coverage plan's default fields must not silently despeckle",
        )

    def test_real_obstacle_still_blocks(self):
        # A three-cell-wide block is a real obstacle, not speckle.
        block = {(row, column) for row in (12, 13, 14) for column in (25, 26, 27)}
        grid = self._channel_grid(block)
        planner = self._planner()
        safe, _clearance = planner._navigation_fields(grid, None, despeckle=True)
        self.assertFalse(
            self._connected(
                safe, (self.CENTRE_ROW, 6), (self.CENTRE_ROW, self.COLUMNS - 7)
            ),
            "a real obstacle must still block the passage",
        )

    def test_open_channel_remains_connected(self):
        grid = self._channel_grid()
        planner = self._planner()
        safe, _clearance = planner._navigation_fields(grid, None, despeckle=True)
        self.assertTrue(
            self._connected(
                safe, (self.CENTRE_ROW, 6), (self.CENTRE_ROW, self.COLUMNS - 7)
            )
        )

    def _centre_path(self):
        # World path along the channel centre (row CENTRE_ROW of the grid).
        y = (self.CENTRE_ROW + 0.5) * 0.1
        return ((0.6, y), (2.0, y), (3.3, y))

    def test_speckle_shadow_does_not_invalidate_a_routed_path(self):
        # run47 floor 0 room 4: routing despeckles and therefore produced a path
        # past a speckle cell, but validation used the raw grid, where the
        # speckle's inflated shadow dropped the clearance below 0.30 m.  The
        # target was discarded every planning cycle and the robot rotated on the
        # spot at the doorway while two opposite targets alternated.
        grid = self._channel_grid({(self.CENTRE_ROW - 1, 20)})
        planner = self._planner()
        self.assertFalse(
            planner.path_is_safe(grid, self._centre_path()),
            "the raw grid keeps the speckle shadow (documents the old failure)",
        )
        self.assertTrue(
            planner.path_is_safe(grid, self._centre_path(), despeckle=True),
            "validating on the router's despeckled map must accept the path",
        )

    def test_real_obstacle_still_invalidates_a_path(self):
        block = {(row, column) for row in (12, 13, 14) for column in (25, 26, 27)}
        grid = self._channel_grid(block)
        planner = self._planner()
        self.assertFalse(
            planner.path_is_safe(grid, self._centre_path(), despeckle=True),
            "despeckling must not accept a path through a real obstacle",
        )

    def test_empty_path_is_unsafe_by_contract(self):
        grid = self._channel_grid()
        planner = self._planner()
        self.assertFalse(planner.path_is_safe(grid, ()))
        self.assertFalse(planner.path_is_safe(grid, (), despeckle=True))


if __name__ == "__main__":
    unittest.main()
