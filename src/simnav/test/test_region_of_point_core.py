#!/usr/bin/env python3
"""Regression tests for the room/corridor membership test (B1 区域化).

``region_of_point`` is the single truth for "which region am I in": the
corridor band, or the nearest confirmed doorway's Voronoi band on that side.
Entry/exit and the coverage denominator both come from it, so they cannot
disagree the way the old wall-offset proxy and the radius heuristic did.
"""

import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "src", "simnav", "scripts"))

from coverage_explorer_core import (  # noqa: E402
    RoomPortal,
    distinct_room_count,
    floor_regions_complete,
    region_of_point,
    region_status,
)


class RegionOfPointTest(unittest.TestCase):
    # Gate at (10, 0) with the corridor axis along +x: along = x - 10,
    # lateral = y.  Corridor half width 1.10 m.
    GATE = (10.0, 0.0, 0.0)
    HALF = 1.10
    PORTALS = (
        RoomPortal(topology_id="ROOM_L_15", side="L", along=7.60, lateral=1.10, width=0.80),
        RoomPortal(topology_id="ROOM_L_29", side="L", along=12.00, lateral=1.10, width=0.80),
        RoomPortal(topology_id="ROOM_R_15", side="R", along=7.60, lateral=-1.15, width=0.80),
    )

    def region(self, point):
        return region_of_point(point, self.GATE, 0.0, self.HALF, self.PORTALS)

    def test_corridor_centre_is_corridor(self):
        self.assertEqual(self.region((9.0, 0.0)), "CORRIDOR")

    def test_corridor_band_edges_stay_corridor(self):
        self.assertEqual(self.region((7.6, 1.20)), "CORRIDOR")
        self.assertEqual(self.region((7.6, -1.20)), "CORRIDOR")

    def test_left_room_past_the_band_belongs_to_the_nearer_door(self):
        self.assertEqual(self.region((7.6, 1.6)), "ROOM_L_15")
        self.assertEqual(self.region((6.0, 2.2)), "ROOM_L_15")

    def test_left_room_far_along_switches_to_the_next_door(self):
        # along = x - 10, so x = 20.5 is along 10.5, nearer the 12.0 doorway.
        self.assertEqual(self.region((20.5, 2.0)), "ROOM_L_29")

    def test_right_room_is_selected_by_side(self):
        self.assertEqual(self.region((7.6, -1.6)), "ROOM_R_15")

    def test_deep_inside_a_room_still_belongs_to_it(self):
        # The room is metres wide; membership must not depend on the 0.8 m
        # doorway window, otherwise a robot working inside would fall out of
        # its own region and the entry state would flap.
        self.assertEqual(self.region((5.5, 4.0)), "ROOM_L_15")

    def test_no_portal_on_that_side_is_unknown(self):
        # Only left doorways are known, so the right side has no region.
        left_only = tuple(p for p in self.PORTALS if p.side == "L")
        self.assertEqual(
            region_of_point((7.6, -3.0), self.GATE, 0.0, self.HALF, left_only),
            "UNKNOWN",
        )

    def test_no_portals_at_all_is_unknown(self):
        self.assertEqual(
            region_of_point((5.5, 4.0), self.GATE, 0.0, self.HALF, ()), "UNKNOWN"
        )


class RegionStatusTest(unittest.TestCase):
    def test_unseen_until_the_target_is_met(self):
        self.assertEqual(region_status(0.20, 0.55), "UNSEEN")

    def test_active_while_it_is_the_region_being_worked(self):
        self.assertEqual(region_status(0.20, 0.55, active=True), "ACTIVE")

    def test_covered_once_the_target_is_reached(self):
        self.assertEqual(region_status(0.55, 0.55), "COVERED")

    def test_coverage_wins_over_active(self):
        self.assertEqual(region_status(0.90, 0.55, active=True), "COVERED")

    def test_missing_coverage_is_not_a_completion(self):
        self.assertEqual(region_status(None, 0.55), "UNSEEN")

    def test_floor_completes_only_when_every_expected_region_is_covered(self):
        self.assertTrue(
            floor_regions_complete(["COVERED"] * 4, 4)
        )
        self.assertFalse(floor_regions_complete(["COVERED"] * 3, 4))
        self.assertFalse(
            floor_regions_complete(["COVERED", "ACTIVE", "COVERED", "COVERED"], 4)
        )


class DistinctRoomCountTest(unittest.TestCase):
    """Floor completion must count physical doors, not 0.5 m id bins."""

    PORTALS = (
        RoomPortal(topology_id="ROOM_L_15", side="L", along=7.60, lateral=1.10, width=1.00),
        RoomPortal(topology_id="ROOM_R_15", side="R", along=7.55, lateral=-1.20, width=0.90),
        RoomPortal(topology_id="ROOM_R_43", side="R", along=21.20, lateral=-1.15, width=0.90),
        RoomPortal(topology_id="ROOM_R_47", side="R", along=21.80, lateral=-1.15, width=0.90),
        RoomPortal(topology_id="ROOM_L_43", side="L", along=21.10, lateral=1.15, width=0.90),
    )

    def test_the_front_pair_counts_as_two_rooms(self):
        self.assertEqual(
            distinct_room_count(["ROOM_L_15", "ROOM_R_15"], self.PORTALS), 2
        )

    def test_two_bins_of_one_door_count_once(self):
        self.assertEqual(
            distinct_room_count(["ROOM_R_43", "ROOM_R_47"], self.PORTALS), 1
        )

    def test_the_run79_trap_needs_the_rear_left_room(self):
        # front-left, front-right and two bins of the rear-right door: three
        # physical rooms, so a floor expecting four must keep exploring until
        # the rear-left doorway is covered too.
        self.assertEqual(
            distinct_room_count(
                ["ROOM_L_15", "ROOM_R_15", "ROOM_R_43", "ROOM_R_47"], self.PORTALS
            ),
            3,
        )
        self.assertEqual(
            distinct_room_count(
                ["ROOM_L_15", "ROOM_R_15", "ROOM_R_43", "ROOM_R_47", "ROOM_L_43"],
                self.PORTALS,
            ),
            4,
        )

    def test_unknown_ids_each_count_once(self):
        self.assertEqual(distinct_room_count(["ROOM_X_1"], self.PORTALS), 1)


if __name__ == "__main__":
    unittest.main()
