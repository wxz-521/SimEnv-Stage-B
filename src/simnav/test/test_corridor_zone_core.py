#!/usr/bin/env python3
"""Regression tests for corridor-half zoning.

Rooms are explored one corridor half at a time: the near half plus its opposing
room pair form one zone, the far half the next.  Inside a zone the robot may move
freely between the half-corridor and its rooms, which is what removes the
"entered the room, no candidate left, stood still" failure - run55 floor 0 locked
ROOM_R_3, produced no candidate at all and stood 1 m from its door until the
position watchdog fired.  Per-room topology and coverage accounting are
unchanged; only what may be dispatched changes.
"""

import unittest

from coverage_explorer_core import (
    RoomPortal,
    active_zone,
    corridor_transit_reaches,
    crossing_zone,
    leg_speed,
    zone_admits,
    zone_of_along,
    zone_split_along,
)


class CrossingZoneTest(unittest.TestCase):
    """Corridor camera viewpoints are admitted only while crossing zones.

    User rule: "allow the corridor to select camera frontier points, but only use
    them when crossing zones".
    """

    def test_near_zone_never_admits_corridor_cameras(self):
        self.assertFalse(crossing_zone("A", 5.0, 20.0))
        self.assertFalse(crossing_zone("A", 25.0, 20.0))

    def test_far_zone_active_but_robot_still_near_is_a_crossing(self):
        self.assertTrue(crossing_zone("B", 8.0, 20.0))

    def test_far_zone_active_and_robot_inside_is_not_a_crossing(self):
        self.assertFalse(crossing_zone("B", 20.0, 20.0))
        self.assertFalse(crossing_zone("B", 24.0, 20.0))


class LegSpeedRampTest(unittest.TestCase):
    """Ramp in from a stop and out near the goal, without crawling.

    User rule: "do not hold the maximum speed the whole time - speed up gradually
    and slow down gradually, with a clear ramp at the start and the end of a
    leg, but not so slow that it wastes efficiency".
    """

    def test_cruise_once_ramped_and_far_from_the_goal(self):
        self.assertAlmostEqual(
            leg_speed(0.60, elapsed=5.0, remaining=10.0), 0.60
        )

    def test_ramps_in_from_the_stop(self):
        # t=0 is the ramp floor, t=half is halfway to cruise.
        self.assertAlmostEqual(leg_speed(0.60, 0.0, 10.0), 0.27)
        self.assertAlmostEqual(leg_speed(0.60, 0.75, 10.0), 0.435)

    def test_ramps_out_near_the_goal_but_never_below_the_floor(self):
        near = leg_speed(0.60, 5.0, remaining=0.0)
        self.assertAlmostEqual(near, 0.27)
        middle = leg_speed(0.60, 5.0, remaining=0.5)
        self.assertAlmostEqual(middle, 0.435)
        self.assertLess(middle, 0.60)

    def test_floor_never_crawls(self):
        # A tiny cruise still respects the absolute minimum, capped by cruise.
        self.assertAlmostEqual(leg_speed(0.10, 0.0, 0.0), 0.10)

    def test_zero_cruise_is_zero_and_ramps_can_be_disabled(self):
        self.assertEqual(leg_speed(0.0, 0.0, 1.0), 0.0)
        self.assertAlmostEqual(
            leg_speed(0.60, 0.0, 0.0, accel_seconds=0.0, decel_distance=0.0), 0.60
        )


class CorridorTransitReachesTest(unittest.TestCase):
    """P3b: a far-zone transit leg is a committed forward step, not a pool rank.

    Ranking the frontier pool re-decided the corridor target on every map
    update, so the robot re-aimed instead of holding a forward leg.  The
    transit ladder must offer only forward offsets, longest first, and fall
    back to the bidirectional ladder for the ordinary wander/fallback use.
    """

    def test_default_ladder_is_bidirectional_nearest_first(self):
        self.assertEqual(
            corridor_transit_reaches(),
            (
                2.0, -2.0, 4.0, -4.0, 6.0, -6.0, 9.0, -9.0,
                14.0, -14.0, 20.0, -20.0, 26.0, -26.0, 32.0, -32.0,
            ),
        )

    def test_forward_only_drops_every_backward_step(self):
        reaches = corridor_transit_reaches(forward_only=True)
        self.assertEqual(reaches, (2.0, 4.0, 6.0, 9.0, 14.0, 20.0, 26.0, 32.0))
        self.assertTrue(all(value > 0.0 for value in reaches))

    def test_forward_reaches_span_the_whole_corridor(self):
        # A far-zone transit must be able to cross the split: the corridor is
        # ~36 m, so the longest leg has to be far larger than half of it.
        self.assertGreaterEqual(max(corridor_transit_reaches(forward_only=True)), 26.0)

    def test_descending_tries_the_longest_leg_first(self):
        reaches = corridor_transit_reaches(forward_only=True, descending=True)
        self.assertEqual(reaches, (32.0, 26.0, 20.0, 14.0, 9.0, 6.0, 4.0, 2.0))
        self.assertEqual(tuple(sorted(reaches, reverse=True)), reaches)


class ZoneSplitTest(unittest.TestCase):
    def test_split_is_the_midpoint_of_the_detected_doorways(self):
        portals = [
            RoomPortal("ROOM_L_15", "L", 15.0, 1.1, 0.9),
            RoomPortal("ROOM_L_43", "L", 43.0, 1.1, 0.9),
        ]
        self.assertAlmostEqual(zone_split_along(portals, 35.0), 29.0)

    def test_single_doorway_falls_back_to_half_the_task_extent(self):
        portals = [RoomPortal("ROOM_L_15", "L", 15.0, 1.1, 0.9)]
        self.assertAlmostEqual(zone_split_along(portals, 30.0), 15.0)

    def test_no_doorways_is_tolerated(self):
        self.assertAlmostEqual(zone_split_along([], 30.0), 15.0)


class ZoneOfAlongTest(unittest.TestCase):
    def test_near_and_far(self):
        self.assertEqual(zone_of_along(5.0, 20.0), "A")
        self.assertEqual(zone_of_along(35.0, 20.0), "B")

    def test_pair_straddling_the_midpoint_stays_near(self):
        # A doorway inside the band must not be torn into the far zone, or an
        # opposing pair would be split across two zones.
        self.assertEqual(zone_of_along(21.0, 20.0, band=1.5), "A")
        self.assertEqual(zone_of_along(22.5, 20.0, band=1.5), "B")

    def test_zone_admits_only_the_active_half(self):
        self.assertTrue(zone_admits(5.0, 20.0, "A"))
        self.assertFalse(zone_admits(35.0, 20.0, "A"))
        self.assertTrue(zone_admits(35.0, 20.0, "B"))


class ActiveZoneTest(unittest.TestCase):
    def setUp(self):
        self.portals = [
            RoomPortal("ROOM_L_15", "L", 8.0, 1.1, 0.9),
            RoomPortal("ROOM_R_15", "R", 8.5, -1.1, 0.9),
            RoomPortal("ROOM_L_43", "L", 33.0, 1.1, 0.9),
            RoomPortal("ROOM_R_43", "R", 33.5, -1.1, 0.9),
        ]
        self.split = zone_split_along(self.portals, 35.0)

    def test_near_zone_runs_first(self):
        self.assertEqual(active_zone(self.portals, (), self.split), "A")

    def test_seam_sized_portal_does_not_hold_the_near_zone(self):
        """A 0.30 m wall seam must not pin the zone.

        The corridor-mouth seams measure 0.30 m / 0.40 m against real doorways of
        0.90-1.00 m, and they can never be completed.  Counting one as an
        unfinished near-zone doorway held ``active_zone`` at "A" for ever, so
        ``zone_admits`` rejected every candidate past the split, the raw pool
        collapsed to an already-retired room and the robot stood on NO_FRONTIER
        for 40+ s (run151) while far-zone rooms were actionable.
        """
        with_seam = list(self.portals) + [RoomPortal("ROOM_L_0", "L", 0.15, 1.2, 0.30)]
        self.assertEqual(
            active_zone(with_seam, ("ROOM_L_15", "ROOM_R_15"), self.split),
            "B",
        )
        # Passing the width through as 0 keeps the pre-filter behaviour, so the
        # rule is visibly the thing that changed.
        self.assertEqual(
            active_zone(
                with_seam,
                ("ROOM_L_15", "ROOM_R_15"),
                self.split,
                minimum_width=0.0,
            ),
            "A",
        )

    def test_far_zone_waits_for_every_near_room(self):
        self.assertEqual(
            active_zone(self.portals, ("ROOM_L_15",), self.split), "A"
        )

    def test_far_zone_opens_once_the_near_half_is_complete(self):
        self.assertEqual(
            active_zone(
                self.portals, ("ROOM_L_15", "ROOM_R_15"), self.split
            ),
            "B",
        )

    def test_everything_complete_defaults_to_the_near_zone(self):
        self.assertEqual(
            active_zone(
                self.portals,
                ("ROOM_L_15", "ROOM_R_15", "ROOM_L_43", "ROOM_R_43"),
                self.split,
            ),
            "A",
        )


if __name__ == "__main__":
    unittest.main()
