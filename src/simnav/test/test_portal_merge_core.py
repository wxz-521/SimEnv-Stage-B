#!/usr/bin/env python3
"""Regression tests for collapsing duplicate doorway bins into one candidate.

One physical doorway is reported as several 0.5 m longitudinal bins, and each
bin is a separate candidate carrying its own topology id, owner label and
approach geometry.  That ambiguity is what let the robot re-approach the same
room through a slightly different candidate (measured: front-R entered three
times in run31).

The merge must be per side and along the corridor axis.  A plain Cartesian
radius would also fold the left and right doorways of a station together, and
those are genuinely different rooms that both have to be explored, so that
mistake is pinned here as well.
"""

import unittest

import numpy as np

from coverage_explorer_core import (
    RoomPortal,
    TaskCoveragePlanner,
    detect_room_portals,
)
from test_coverage_explorer_core import synthetic_floor


class PortalMergeTest(unittest.TestCase):
    def _planner(self, merge_radius=2.5):
        return TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            forward_depth=24.0,
            back_extension=1.0,
            lateral_half_width=9.5,
            portal_merge_radius=merge_radius,
        )

    def _plan(self, portals, confirmed, lock=None, merge_radius=2.5):
        grid = synthetic_floor()
        return self._planner(merge_radius).plan(
            grid,
            robot_pose=(0.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            confirmed_topologies=confirmed,
            remembered_portals=tuple(portals),
            topology_lock=lock,
        )

    def test_duplicate_bins_of_one_door_collapse(self):
        centre = detect_room_portals(
            synthetic_floor(), (0.0, 0.0), 0.0, 35.0, 9.5, 1.1
        )[0]
        bins = [
            RoomPortal(
                topology_id="ROOM_L_15",
                side=centre.side,
                along=centre.along,
                lateral=centre.lateral,
                width=1.0,
            ),
            RoomPortal(
                topology_id="ROOM_L_16",
                side=centre.side,
                along=centre.along + 0.5,
                lateral=centre.lateral,
                width=1.2,
            ),
            RoomPortal(
                topology_id="ROOM_L_17",
                side=centre.side,
                along=centre.along + 1.0,
                lateral=centre.lateral,
                width=0.9,
            ),
        ]
        ids = [portal.topology_id for portal in bins]
        plan = self._plan(bins, ids)
        self.assertEqual(
            (plan.diagnostics or {}).get("portals_merged_same_door"), 2
        )
        self.assertEqual(len(plan.actionable_portals), 1)
        # The widest opening represents the door when no room is locked.
        self.assertEqual(plan.actionable_portals[0].topology_id, "ROOM_L_16")

    def test_locked_room_keeps_its_own_bin(self):
        centre = detect_room_portals(
            synthetic_floor(), (0.0, 0.0), 0.0, 35.0, 9.5, 1.1
        )[0]
        bins = [
            RoomPortal(
                topology_id="ROOM_L_15",
                side=centre.side,
                along=centre.along,
                lateral=centre.lateral,
                width=1.0,
            ),
            RoomPortal(
                topology_id="ROOM_L_16",
                side=centre.side,
                along=centre.along + 0.5,
                lateral=centre.lateral,
                width=1.4,
            ),
        ]
        plan = self._plan(bins, [p.topology_id for p in bins], lock="ROOM_L_15")
        self.assertEqual(len(plan.actionable_portals), 1)
        self.assertEqual(plan.actionable_portals[0].topology_id, "ROOM_L_15")

    def test_opposite_side_doorways_are_never_merged(self):
        # Left and right doorways of one station are ~2.2 m apart across the
        # corridor: inside any Cartesian merge radius, but different rooms.
        left = RoomPortal("ROOM_L_15", "L", 15.0, 1.1, 1.0)
        right = RoomPortal("ROOM_R_15", "R", 15.0, -1.1, 1.0)
        plan = self._plan([left, right], ["ROOM_L_15", "ROOM_R_15"])
        self.assertIsNone((plan.diagnostics or {}).get("portals_merged_same_door"))
        self.assertEqual(
            sorted(portal.topology_id for portal in plan.actionable_portals),
            ["ROOM_L_15", "ROOM_R_15"],
        )

    def test_merge_can_be_disabled(self):
        left = RoomPortal("ROOM_L_15", "L", 15.0, 1.1, 1.0)
        left_bin = RoomPortal("ROOM_L_16", "L", 15.5, 1.1, 1.0)
        plan = self._plan(
            [left, left_bin], ["ROOM_L_15", "ROOM_L_16"], merge_radius=0.0
        )
        self.assertIsNone((plan.diagnostics or {}).get("portals_merged_same_door"))
        self.assertEqual(len(plan.actionable_portals), 2)


if __name__ == "__main__":
    unittest.main()
