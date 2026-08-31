#!/usr/bin/env python3

import unittest

from stage_b_monitor_core import confirmed_hypotheses, doorway_station_structure


def hypothesis(x, y, side, normal):
    return {"center": [x, y], "side": side, "normal_yaw": normal}


class DoorwayStationStructureTest(unittest.TestCase):
    def test_pending_scan_staging_is_not_an_actionable_door(self):
        pending = hypothesis(17.0, -1.2, -1, -1.5708)
        pending["status"] = "PENDING"
        confirmed = hypothesis(17.0, 1.1, 1, 1.5708)
        self.assertEqual(confirmed_hypotheses([pending, confirmed]), [confirmed])

    def test_accepts_two_opposing_door_stations(self):
        hypotheses = [
            hypothesis(17.0, -1.2, -1, -1.5708),
            hypothesis(17.0, 1.1, 1, 1.5708),
            hypothesis(30.9, -1.3, -1, -1.5708),
            hypothesis(31.0, 0.9, 1, 1.5708),
        ]
        self.assertTrue(doorway_station_structure(hypotheses)["valid"])

    def test_rejects_four_doors_without_opposing_stations(self):
        hypotheses = [
            hypothesis(17.0, -1.2, -1, -1.5708),
            hypothesis(17.0, 1.1, 1, 1.5708),
            hypothesis(19.0, -1.3, -1, -1.5708),
            hypothesis(25.0, -1.0, -1, -1.5708),
        ]
        result = doorway_station_structure(hypotheses)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "unpaired_sides")

    def test_rejects_missing_real_pair_replaced_by_distant_false_door(self):
        hypotheses = [
            hypothesis(17.0, -1.2, -1, -1.5708),
            hypothesis(17.0, 1.1, 1, 1.5708),
            hypothesis(30.9, -1.3, -1, -1.5708),
            hypothesis(44.0, 1.0, 1, 1.5708),
        ]
        result = doorway_station_structure(hypotheses)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "station_pairing")


if __name__ == "__main__":
    unittest.main()
