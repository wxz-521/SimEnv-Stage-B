#!/usr/bin/env python3

import unittest

from coverage_explorer_core import unsafe_path_replan_needed


class UnsafePathReplanTest(unittest.TestCase):
    def test_no_active_target_never_replans_on_safety(self):
        self.assertFalse(
            unsafe_path_replan_needed(False, False, 99, 3)
        )

    def test_safe_path_never_replans(self):
        self.assertFalse(unsafe_path_replan_needed(True, True, 99, 3))

    def test_single_unsafe_reading_is_debounced(self):
        # run47: one unsafe reading per rotation re-dispatched the room's other
        # target every ~5.5 s and the robot never crossed the doorway.
        self.assertFalse(unsafe_path_replan_needed(True, False, 1, 3))
        self.assertFalse(unsafe_path_replan_needed(True, False, 2, 3))

    def test_persistent_unsafe_path_eventually_replans(self):
        self.assertTrue(unsafe_path_replan_needed(True, False, 3, 3))
        self.assertTrue(unsafe_path_replan_needed(True, False, 4, 3))

    def test_cycle_floor_is_clamped(self):
        self.assertTrue(unsafe_path_replan_needed(True, False, 1, 0))
        self.assertFalse(unsafe_path_replan_needed(True, False, 0, 1))


if __name__ == "__main__":
    unittest.main()
