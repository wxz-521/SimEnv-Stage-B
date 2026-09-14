#!/usr/bin/env python3
"""Regression tests for the door-approach fallback.

run50 floor 0 stalled in the corridor: four actionable portals, every candidate
rejected as ``path_unreachable``, a zero-cell verified door band, and the robot
standing 8.5 m short of the door with no corridor frontier left.  Entering a
room that has never been seen cannot be planned, because the staged crossing
ends at a point inside that room which is unknown by definition -- so the robot
has to be sent to the door first.
"""

import unittest

from coverage_explorer_core import door_approach_is_new


class DoorApproachIsNewTest(unittest.TestCase):
    def test_unvisited_viewpoint_is_dispatched(self):
        self.assertTrue(door_approach_is_new((5.0, 0.0), [], 0.7))

    def test_already_visited_viewpoint_is_not_re_dispatched(self):
        # Standing in front of the door again must not become an endless
        # shuttle once the room has proved unroutable from there.
        self.assertFalse(door_approach_is_new((5.0, 0.0), [(5.1, 0.1)], 0.7))

    def test_distant_visited_viewpoints_do_not_block(self):
        self.assertTrue(door_approach_is_new((5.0, 0.0), [(9.0, 0.0)], 0.7))

    def test_radius_is_respected(self):
        visited = [(5.0, 0.5)]
        self.assertFalse(door_approach_is_new((5.0, 0.0), visited, 0.7))
        self.assertTrue(door_approach_is_new((5.0, 0.0), visited, 0.4))

    def test_missing_visited_list_is_tolerated(self):
        self.assertTrue(door_approach_is_new((5.0, 0.0), None, 0.7))


if __name__ == "__main__":
    unittest.main()


class DoorApproachTargetTest(unittest.TestCase):
    """The fallback must also fire when a locked room yields no candidate."""

    def _scene(self):
        import numpy as np

        from coverage_explorer_core import GridView, RoomPortal, TaskCoveragePlanner

        # 8 m x 8 m free space; the robot sits 1 m from the gate, and the locked
        # room's door is 3 m along the corridor wall on the right side.
        data = np.zeros((80, 80), dtype=np.int16)
        grid = GridView(data, 0.10, 0.0, 0.0)
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.10,
            preferred_clearance=0.20,
        )
        portal = RoomPortal("ROOM_R_3", "R", 3.0, -1.10, 1.40)
        return planner, grid, portal

    def test_locked_room_without_candidates_still_gets_an_approach_target(self):
        planner, grid, portal = self._scene()
        diagnostics = {"door_approach_portal": None}
        target = planner._door_approach_target(
            grid,
            (1.0, 0.0, 0.0),
            (0.0, 0.0),
            0.0,
            [portal],
            {"ROOM_R_3"},
            (),
            diagnostics,
        )
        self.assertIsNotNone(target)
        self.assertEqual(target.topology_id, "ROOM_R_3")
        self.assertEqual(diagnostics["door_approach_portal"], "ROOM_R_3")
        # It is the door-side staging point on the corridor centreline.
        self.assertAlmostEqual(target.target[1], 0.0, places=6)

    def test_unknown_owner_is_ignored(self):
        planner, grid, portal = self._scene()
        diagnostics = {"door_approach_portal": None}
        target = planner._door_approach_target(
            grid, (1.0, 0.0, 0.0), (0.0, 0.0), 0.0, [portal], {"ROOM_X"}, (), diagnostics
        )
        self.assertIsNone(target)

    def test_already_visited_door_is_not_re_dispatched(self):
        planner, grid, portal = self._scene()
        diagnostics = {"door_approach_portal": None}
        stage = planner._portal_waypoint((0.0, 0.0), 0.0, 3.0, 0.0)
        target = planner._door_approach_target(
            grid,
            (1.0, 0.0, 0.0),
            (0.0, 0.0),
            0.0,
            [portal],
            {"ROOM_R_3"},
            (stage,),
            diagnostics,
        )
        self.assertIsNone(target)


class PreferRoomApproachTest(unittest.TestCase):
    """A failed room candidate must outrank a merely reachable corridor target."""

    def test_room_approach_wins_over_corridor(self):
        from coverage_explorer_core import prefer_room_approach

        self.assertTrue(prefer_room_approach("CORRIDOR", ["ROOM_L_43"]))

    def test_corridor_target_stays_when_no_room_failed(self):
        from coverage_explorer_core import prefer_room_approach

        self.assertFalse(prefer_room_approach("CORRIDOR", []))

    def test_a_routed_room_target_is_kept(self):
        from coverage_explorer_core import prefer_room_approach

        self.assertFalse(prefer_room_approach("ROOM_L_43", ["ROOM_R_15"]))

    def test_weak_corridor_target_yields_to_a_pending_doorway(self):
        # The user's policy: buy observation when what is left is worthless.
        from coverage_explorer_core import prefer_room_approach

        self.assertTrue(
            prefer_room_approach("CORRIDOR", [], chosen_gain=0.2, unobserved_rooms=1)
        )

    def test_valuable_corridor_target_is_still_taken(self):
        from coverage_explorer_core import prefer_room_approach

        self.assertFalse(
            prefer_room_approach("CORRIDOR", [], chosen_gain=8.0, unobserved_rooms=1)
        )

    def test_no_pending_doorway_means_no_diversion(self):
        from coverage_explorer_core import prefer_room_approach

        self.assertFalse(
            prefer_room_approach("CORRIDOR", [], chosen_gain=0.1, unobserved_rooms=0)
        )

    def test_weak_gain_threshold_is_configurable(self):
        from coverage_explorer_core import prefer_room_approach

        self.assertFalse(
            prefer_room_approach(
                "CORRIDOR", [], chosen_gain=2.0, unobserved_rooms=1, weak_gain=1.0
            )
        )
        self.assertTrue(
            prefer_room_approach(
                "CORRIDOR", [], chosen_gain=2.0, unobserved_rooms=1, weak_gain=3.0
            )
        )


class InteriorOnlyTest(unittest.TestCase):
    """Inside a room the target stays in the room until retries are exhausted."""

    def test_outside_a_room_there_is_no_restriction(self):
        from coverage_explorer_core import interior_targets_only

        self.assertFalse(interior_targets_only(False, 0, 3))

    def test_inside_a_room_corridor_targets_are_blocked(self):
        from coverage_explorer_core import interior_targets_only

        self.assertTrue(interior_targets_only(True, 0, 3))
        self.assertTrue(interior_targets_only(True, 2, 3))

    def test_repeated_interior_failure_opens_the_escape(self):
        from coverage_explorer_core import interior_targets_only

        self.assertFalse(interior_targets_only(True, 3, 3))
