#!/usr/bin/env python3

import unittest

import numpy as np
from lio_occupancy_node import trace_ray_free


class LioOccupancyRayTest(unittest.TestCase):
    def test_ray_tracing_does_not_clear_occupied_wall(self):
        grid = np.full((20, 20), -1, dtype=np.int8)
        grid[10, 5] = 100
        trace_ray_free(grid, (0, 10), (15, 10))

        self.assertEqual(int(grid[10, 5]), 100)
        self.assertEqual(int(grid[10, 3]), 0)


if __name__ == "__main__":
    unittest.main()
