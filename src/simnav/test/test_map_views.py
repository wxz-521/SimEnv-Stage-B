#!/usr/bin/env python3

import unittest

import numpy as np

from map_views_node import close_small_gaps


class MapViewsTest(unittest.TestCase):
    def test_closing_fills_small_gap_without_changing_source(self):
        source = np.zeros((20, 20), dtype=np.int16)
        source[4:16, 8] = 100
        source[9:11, 8] = 0
        original = source.copy()
        result = close_small_gaps(source, 5)
        self.assertTrue(np.array_equal(source, original))
        self.assertTrue(np.all(result[9:11, 8] == 100))

    def test_unknown_cells_do_not_become_occupied_away_from_gap(self):
        source = np.full((20, 20), -1, dtype=np.int16)
        source[8:12, 8:12] = 0
        result = close_small_gaps(source, 5)
        self.assertEqual(int(result[0, 0]), -1)
        self.assertEqual(int(result[10, 10]), 0)


if __name__ == "__main__":
    unittest.main()
