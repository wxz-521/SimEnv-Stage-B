#!/usr/bin/env python3
"""Regression tests for doorway-scoped removal of mis-detected obstacles.

The occupancy map never converts an occupied cell back to free, so one stray
endpoint on a door line seals a real doorway for the whole run.  Measured
symptom: verified door bands of 0 cells while the opening was physically clear.
Inside a doorway whose span the portal detector has already measured, a one- or
two-cell blob cannot be a wall.
"""

import unittest

import numpy as np

from coverage_explorer_core import clear_doorway_speckle


class DoorwaySpeckleTest(unittest.TestCase):
    def _grid(self):
        # 40x40 free map with a wall row at 20 except a 4-cell opening.
        data = np.zeros((40, 40), dtype=np.int16)
        data[20, :] = 100
        data[20, 18:22] = 0
        return data

    def _mask(self):
        mask = np.zeros((40, 40), dtype=bool)
        mask[18:23, 17:23] = True
        return mask

    def test_isolated_blob_in_the_doorway_is_removed(self):
        data = self._grid()
        data[20, 19] = 100  # phantom endpoint inside the opening
        freed = clear_doorway_speckle(data, self._mask(), 2)
        self.assertEqual(freed, 1)
        self.assertEqual(int(data[20, 19]), 0)

    def test_two_cell_blob_is_removed(self):
        data = self._grid()
        data[20, 19] = 100
        data[20, 20] = 100
        freed = clear_doorway_speckle(data, self._mask(), 2)
        self.assertEqual(freed, 2)

    def test_real_wall_is_never_touched(self):
        data = self._grid()
        freed = clear_doorway_speckle(data, self._mask(), 2)
        self.assertEqual(freed, 0)
        self.assertEqual(int(data[20, 5]), 100)
        self.assertEqual(int(data[20, 30]), 100)

    def test_larger_blob_in_the_doorway_is_kept(self):
        # A 3-cell obstacle is not speckle; treat it as a real obstacle.
        data = self._grid()
        data[20, 18] = 100
        data[20, 19] = 100
        data[20, 20] = 100
        freed = clear_doorway_speckle(data, self._mask(), 2)
        self.assertEqual(freed, 0)
        self.assertEqual(int(data[20, 19]), 100)

    def test_blob_outside_the_doorway_span_is_kept(self):
        # Outside the mask this function must do nothing: the caller scopes it.
        data = self._grid()
        data[5, 5] = 100
        data[5, 6] = 100
        freed = clear_doorway_speckle(data, self._mask(), 2)
        self.assertEqual(freed, 0)
        self.assertEqual(int(data[5, 5]), 100)

    def test_empty_mask_is_a_no_op(self):
        data = self._grid()
        data[20, 19] = 100
        self.assertEqual(
            clear_doorway_speckle(data, np.zeros_like(data, dtype=bool), 2), 0
        )
        self.assertEqual(int(data[20, 19]), 100)


if __name__ == "__main__":
    unittest.main()
