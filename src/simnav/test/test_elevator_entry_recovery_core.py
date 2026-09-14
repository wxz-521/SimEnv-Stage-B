#!/usr/bin/env python3
"""Regression tests for elevator car-door handling.

The standalone lift test (team_scripts/elevator_only_test.sh) drove the whole
boarding sequence by coordinates and showed that the failed entries were caused
by entering off the doorway centre line, not by the 6 cm car threshold or by
the gait policy.  DoorFrameOffsetTest pins the geometry that discipline uses.

run49 completed floors 0 and 1 with 4 rooms each and then failed the whole
mission on the floor-1 return leg:
``ELEVATOR_PATH_BLOCKED_AFTER_0.93M`` with a 0.27 m front clearance, 7 cm short
of ``minimum_entry_progress``.  The layout marks ``elevator_floor_0`` as
initially open and ``elevator_floor_1`` as initially closed, and the node only
ever called ``/set_door_state`` for the main entrance.
"""

import math
import unittest

from elevator_transition_core import (
    door_frame_offset,
    elevator_door_id,
    entry_blocked_retry_allowed,
)


class DoorFrameOffsetTest(unittest.TestCase):
    """Boarding is only committed from the doorway centre line.

    The scene's car doorway sits at (1.65, 2.60) facing +x with a 1.70 m clear
    opening.  The first standalone lift test drifted to lateral +0.49 m, scraped
    the shaft wall beside the opening, and pushed there for 35 s.
    """

    DOOR = (1.65, 2.60, 0.0, 1.40)

    def test_point_behind_the_door_is_negative_along(self):
        along, lateral = door_frame_offset((-0.55, 2.60), self.DOOR)
        self.assertAlmostEqual(along, -2.20)
        self.assertAlmostEqual(lateral, 0.0)

    def test_point_inside_the_car_is_positive_along(self):
        along, lateral = door_frame_offset((2.85, 2.60), self.DOOR)
        self.assertAlmostEqual(along, 1.20)
        self.assertAlmostEqual(lateral, 0.0)

    def test_sideways_offset_is_lateral(self):
        along, lateral = door_frame_offset((1.65, 3.09), self.DOOR)
        self.assertAlmostEqual(along, 0.0)
        self.assertAlmostEqual(lateral, 0.49)


class EntryBlockedRetryTest(unittest.TestCase):
    def test_short_travel_block_is_retried_while_budget_remains(self):
        # The exact run49 numbers: 0.93 m travelled against a 1.0 m threshold.
        self.assertTrue(
            entry_blocked_retry_allowed(0.93, 1.0, retries=0, retry_limit=4)
        )

    def test_post_threshold_block_is_containment_not_a_retry(self):
        # At or beyond the threshold the caller treats the stop as a completed
        # entry, so no retry may be requested.
        self.assertFalse(
            entry_blocked_retry_allowed(1.0, 1.0, retries=0, retry_limit=4)
        )
        self.assertFalse(
            entry_blocked_retry_allowed(2.4, 1.0, retries=3, retry_limit=4)
        )

    def test_retry_budget_is_finite(self):
        self.assertTrue(
            entry_blocked_retry_allowed(0.4, 1.0, retries=3, retry_limit=4)
        )
        self.assertFalse(
            entry_blocked_retry_allowed(0.4, 1.0, retries=4, retry_limit=4)
        )

    def test_zero_budget_preserves_the_old_fail_fast_behaviour(self):
        self.assertFalse(
            entry_blocked_retry_allowed(0.93, 1.0, retries=0, retry_limit=0)
        )


class ElevatorDoorIdTest(unittest.TestCase):
    def test_floor_ids_match_the_scene_layout(self):
        self.assertEqual(elevator_door_id(0), "elevator_floor_0")
        self.assertEqual(elevator_door_id(1), "elevator_floor_1")
        self.assertEqual(elevator_door_id(2), "elevator_floor_2")

    def test_float_floor_index_is_accepted(self):
        self.assertEqual(elevator_door_id(1.0), "elevator_floor_1")

    def test_prefix_can_be_overridden(self):
        self.assertEqual(
            elevator_door_id(2, "lift_door"), "lift_door_2"
        )


if __name__ == "__main__":
    unittest.main()


class EstablishBudgetTest(unittest.TestCase):
    """The floor-topology drive must not hold the mission in the lift lobby."""

    def test_within_budget_keeps_driving(self):
        from elevator_transition_core import establish_budget_exceeded

        self.assertFalse(establish_budget_exceeded(100.0, 150.0, 150.0))

    def test_budget_exhaustion_releases_the_mission(self):
        from elevator_transition_core import establish_budget_exceeded

        # run52 floor 2 sat here for more than five minutes.
        self.assertTrue(establish_budget_exceeded(100.0, 401.0, 150.0))

    def test_no_start_means_no_budget(self):
        from elevator_transition_core import establish_budget_exceeded

        self.assertFalse(establish_budget_exceeded(None, 9999.0, 150.0))


class DirectEntryTest(unittest.TestCase):
    """Brute-force straight entry: short line-of-sight targets skip A*."""

    def test_short_line_of_sight_target_is_driven_directly(self):
        from elevator_transition_core import direct_entry_applies

        self.assertTrue(direct_entry_applies(1.8, 4.0, True))

    def test_long_target_still_uses_planning(self):
        from elevator_transition_core import direct_entry_applies

        # The return to spawn crosses the whole building; it must stay planned.
        self.assertFalse(direct_entry_applies(28.0, 4.0, True))

    def test_direct_entry_can_be_disabled(self):
        from elevator_transition_core import direct_entry_applies

        self.assertFalse(direct_entry_applies(1.8, 4.0, False))

    def test_blocked_direct_run_falls_back_to_planning(self):
        from elevator_transition_core import direct_entry_applies

        self.assertTrue(direct_entry_applies(1.8, 4.0, True, 10.0, 25.0))
        self.assertFalse(direct_entry_applies(1.8, 4.0, True, 30.0, 25.0))

    def test_must_align_before_driving(self):
        from elevator_transition_core import direct_alignment_ready

        # run52 drove forward while facing ~pi away and arced into the wall.
        self.assertFalse(direct_alignment_ready(3.0, 0.25))
        self.assertTrue(direct_alignment_ready(0.05, 0.25))
        self.assertTrue(direct_alignment_ready(-0.24, 0.25))


class TiltFaultTest(unittest.TestCase):
    """ROBOT_ROLLED must need a sustained tilt, not a single metric spike."""

    def test_level_attitude_is_ok(self):
        from elevator_transition_core import tilt_fault_state

        self.assertEqual(
            tilt_fault_state(0.1, 0.05, 0.40, 0.40, None, 100.0, 0.4), "ok"
        )

    def test_single_sample_spike_does_not_fault(self):
        from elevator_transition_core import tilt_fault_state

        # run63: world roll 0.45 rad for one sample, body IMU 0.12 rad.
        self.assertEqual(
            tilt_fault_state(0.45, 0.31, 0.40, 0.40, None, 100.0, 0.4),
            "pending",
        )
        self.assertEqual(
            tilt_fault_state(0.45, 0.31, 0.40, 0.40, 100.0, 100.2, 0.4),
            "pending",
        )

    def test_sustained_tilt_faults(self):
        from elevator_transition_core import tilt_fault_state

        self.assertEqual(
            tilt_fault_state(0.45, 0.31, 0.40, 0.40, 100.0, 100.5, 0.4),
            "fault",
        )

    def test_recovery_clears_the_window(self):
        from elevator_transition_core import tilt_fault_state

        self.assertEqual(
            tilt_fault_state(0.1, 0.1, 0.40, 0.40, 100.0, 100.5, 0.4), "ok"
        )
