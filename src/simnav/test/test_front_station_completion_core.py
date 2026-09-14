#!/usr/bin/env python3

import unittest

from coverage_explorer_core import doorways_match, front_stations_complete


class _Portal:
    def __init__(self, topology_id, side, along, lateral=0.0):
        self.topology_id = topology_id
        self.side = side
        self.along = along
        self.lateral = lateral


class DoorwayIdentityTest(unittest.TestCase):
    def test_same_id_matches(self):
        first = _Portal("ROOM_L_15", "L", 15.0)
        second = _Portal("ROOM_L_15", "L", 15.2)
        self.assertTrue(doorways_match(first, second))

    def test_drifted_id_matches_at_the_same_opening(self):
        # Portal ids are transient SLAM bins: ROOM_L_10 and ROOM_L_15 are the
        # same doorway seen at two map refinement stages.
        first = _Portal("ROOM_L_10", "L", 15.1)
        second = _Portal("ROOM_L_15", "L", 15.3)
        self.assertTrue(doorways_match(first, second))

    def test_opposite_sides_never_match(self):
        left = _Portal("ROOM_L_15", "L", 15.0)
        right = _Portal("ROOM_R_15", "R", 15.0)
        self.assertFalse(doorways_match(left, right))

    def test_distant_same_side_rooms_do_not_match(self):
        first = _Portal("ROOM_L_15", "L", 15.0)
        second = _Portal("ROOM_L_22", "L", 22.0)
        self.assertFalse(doorways_match(first, second))


class FrontStationsCompleteTest(unittest.TestCase):
    def setUp(self):
        self.front = [_Portal("ROOM_L_15", "L", 15.0), _Portal("ROOM_R_15", "R", 15.0)]

    def test_complete_by_matching_ids(self):
        self.assertTrue(
            front_stations_complete(
                self.front,
                {"ROOM_L_15", "ROOM_R_15"},
                self.front,
            )
        )

    def test_complete_when_ids_drifted_the_run46_floor2_case(self):
        # Explored and completed as ROOM_L_10/ROOM_R_10 while the front station
        # identity drifted to ROOM_L_15/ROOM_R_15.  Matching ids only kept
        # front_rooms_complete False for ever and dead-locked the corridor.
        completed_portals = [
            _Portal("ROOM_L_10", "L", 15.2),
            _Portal("ROOM_R_10", "R", 14.8),
        ]
        self.assertTrue(
            front_stations_complete(
                self.front,
                {"ROOM_L_10", "ROOM_R_10"},
                completed_portals,
            )
        )

    def test_incomplete_when_only_one_side_is_done(self):
        completed_portals = [_Portal("ROOM_L_10", "L", 15.2)]
        self.assertFalse(
            front_stations_complete(
                self.front,
                {"ROOM_L_10"},
                completed_portals,
            )
        )

    def test_a_distant_completed_room_does_not_complete_the_front_pair(self):
        completed_portals = [
            _Portal("ROOM_L_43", "L", 43.0),
            _Portal("ROOM_R_43", "R", 43.0),
        ]
        self.assertFalse(
            front_stations_complete(
                self.front,
                {"ROOM_L_43", "ROOM_R_43"},
                completed_portals,
            )
        )

    def test_explicit_front_sides_still_count(self):
        self.assertTrue(
            front_stations_complete(self.front, set(), [], ("L", "R"))
        )

    def test_single_sided_front_pair_is_never_complete(self):
        only_left = [_Portal("ROOM_L_15", "L", 15.0)]
        self.assertFalse(
            front_stations_complete(only_left, {"ROOM_L_15"}, only_left)
        )


if __name__ == "__main__":
    unittest.main()
