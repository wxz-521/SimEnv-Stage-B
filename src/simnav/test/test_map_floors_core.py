#!/usr/bin/env python3
"""Regression tests for the height-separated 2D maps.

The occupancy node used to keep a single 2D grid and ``fill(-1)`` it on every
floor change.  The mission logs show the two consequences: a floor's topology
had to be reused rather than measured, and portal ids drifted because each floor
was re-derived from a wiped map.  These tests pin the replacement contract: one
grid per floor, retained across switches, and a band that cannot absorb the
adjacent floor's walls.
"""

import unittest

import numpy as np

from map_floors_core import (
    ensure_floor_grid,
    floor_index_for_height,
    floor_map_report,
    height_band_is_sane,
    observed_floor_level,
    point_floor_mask,
)


class FloorBandTest(unittest.TestCase):
    MINIMUM = -0.10
    MAXIMUM = 1.30

    def _mask(self, heights, floor_z=0.60):
        return point_floor_mask(heights, floor_z, self.MINIMUM, self.MAXIMUM)

    def test_same_floor_wall_is_accepted(self):
        # Robot stands 0.6 m above the floor slab; a 1.2 m wall is +1.2 m.
        self.assertTrue(bool(self._mask([1.80])[0]))

    def test_next_floor_wall_is_rejected(self):
        # floor_height is 2.6 m, so the floor above starts at +2.6 m.
        self.assertFalse(bool(self._mask([3.20])[0]))

    def test_floor_slab_below_is_rejected(self):
        self.assertFalse(bool(self._mask([0.20])[0]))

    def test_unknown_floor_level_accepts_everything(self):
        # Startup: no floor level observed yet, so behaviour matches the
        # original single-map node.
        self.assertTrue(bool(self._mask([99.0], None)[0]))

    def test_mixed_scan_splits_by_height(self):
        mask = self._mask([1.80, 3.20, 0.20])
        self.assertEqual([bool(value) for value in mask], [True, False, False])

    def test_band_sanity_guard(self):
        self.assertTrue(height_band_is_sane(-0.10, 1.30, 2.6))
        self.assertFalse(height_band_is_sane(-0.10, 2.60, 2.6))
        self.assertFalse(height_band_is_sane(-3.00, 1.30, 2.6))


class FloorRetentionTest(unittest.TestCase):
    SHAPE = (4, 5)

    def test_new_floor_starts_unknown_and_is_reported_as_new(self):
        grids = {}
        grid, created = ensure_floor_grid(grids, 0, self.SHAPE)
        self.assertTrue(created)
        self.assertTrue(np.all(grid == -1))

    def test_switching_floors_retains_the_previous_map(self):
        grids = {}
        ensure_floor_grid(grids, 0, self.SHAPE)
        grids[0][1, 1] = 100
        grids[0][2, 2] = 0
        second, created = ensure_floor_grid(grids, 1, self.SHAPE)
        self.assertTrue(created)
        # The floor-0 map must survive the switch: that is what the old
        # fill(-1) destroyed and why the ground floor was not navigable on the
        # way back down.
        self.assertEqual(int(grids[0][1, 1]), 100)
        self.assertEqual(int(grids[0][2, 2]), 0)
        self.assertTrue(np.all(second == -1))

    def test_returning_to_a_floor_restores_its_map(self):
        grids = {}
        ensure_floor_grid(grids, 0, self.SHAPE)
        grids[0][0, 0] = 100
        ensure_floor_grid(grids, 1, self.SHAPE)
        restored, created = ensure_floor_grid(grids, 0, self.SHAPE)
        self.assertFalse(created)
        self.assertEqual(int(restored[0, 0]), 100)


class FloorReportTest(unittest.TestCase):
    SHAPE = (4, 5)

    def _grids(self):
        grids = {}
        ensure_floor_grid(grids, 0, self.SHAPE)
        ensure_floor_grid(grids, 1, self.SHAPE)
        grids[0][0, 0] = 100
        grids[0][0, 1] = 0
        grids[0][0, 2] = 0
        return grids

    def test_report_lists_every_retained_floor_with_counts(self):
        report = floor_map_report(self._grids(), 0, {0: 0.6, 1: 3.2}, 2.6)
        self.assertEqual([item["floor_index"] for item in report], [0, 1])
        first, second = report
        self.assertTrue(first["active"])
        self.assertFalse(second["active"])
        self.assertEqual(first["known_cells"], 3)
        self.assertEqual(first["occupied_cells"], 1)
        self.assertEqual(first["free_cells"], 2)
        self.assertEqual(second["known_cells"], 0)

    def test_height_is_measured_from_the_lowest_floor(self):
        report = floor_map_report(self._grids(), 1, {0: 0.6, 1: 3.2}, 2.6)
        self.assertAlmostEqual(report[0]["height"], 0.0)
        self.assertAlmostEqual(report[1]["height"], 2.6)
        self.assertEqual(report[1]["height_source"], "observed")

    def test_unknown_level_falls_back_to_the_nominal_floor_height(self):
        report = floor_map_report(self._grids(), 0, {}, 2.6)
        self.assertEqual(report[0]["height"], 0.0)
        self.assertEqual(report[1]["height"], 2.6)
        self.assertEqual(report[1]["height_source"], "nominal")

    def test_empty_floor_is_not_reported_complete(self):
        report = floor_map_report(self._grids(), 0, {}, 2.6)
        self.assertFalse(report[1]["complete"])


class FloorLevelTest(unittest.TestCase):
    def test_lowest_sample_is_the_level_estimate(self):
        self.assertAlmostEqual(observed_floor_level([0.62, 0.60, 0.75]), 0.60)

    def test_no_samples_means_unknown(self):
        self.assertIsNone(observed_floor_level([]))
        self.assertIsNone(observed_floor_level([None, None]))

    def test_floor_index_from_height(self):
        self.assertEqual(floor_index_for_height(0.6, 0.6, 2.6), 0)
        self.assertEqual(floor_index_for_height(3.2, 0.6, 2.6), 1)
        self.assertEqual(floor_index_for_height(5.8, 0.6, 2.6), 2)

    def test_zero_floor_height_is_tolerated(self):
        self.assertEqual(floor_index_for_height(5.8, 0.6, 0.0), 0)


if __name__ == "__main__":
    unittest.main()
