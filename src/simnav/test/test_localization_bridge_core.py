#!/usr/bin/env python3

import math
import unittest

from pose_continuity import (
    CommandPoseIntegrator,
    PoseContinuityFilter,
    constrain_to_corridor,
)


class PoseContinuityFilterTest(unittest.TestCase):
    def test_corridor_constraint_removes_lateral_drift_only(self):
        x, y = constrain_to_corridor(
            (-1.8, 15.0),
            (0.0, -3.2),
            math.pi / 2.0,
            lateral_error=0.1,
        )
        self.assertAlmostEqual(x, -0.1)
        self.assertAlmostEqual(y, 15.0)

    def test_command_pose_integrator_uses_metric_body_velocity_and_lio_yaw(self):
        integrator = CommandPoseIntegrator(0.0, -3.2, response_scale=0.95)
        integrator.update(1.0, math.pi / 2.0)
        integrator.set_command(1.0, 0.6)
        x, y = integrator.update(2.0, math.pi / 2.0)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, -3.2, places=6)
        integrator.set_command(2.0, 0.6)
        x, y = integrator.update(2.3, math.pi / 2.0)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, -3.029, places=6)

    def test_command_pose_integrator_rejects_stale_command(self):
        integrator = CommandPoseIntegrator(1.0, 2.0)
        integrator.update(0.0, 0.0)
        integrator.set_command(0.0, 0.5)
        self.assertEqual(integrator.update(0.5, 0.0), (1.0, 2.0))

    def test_command_pose_integrator_rejects_motion_below_command_fraction(self):
        integrator = CommandPoseIntegrator(1.0, 2.0)
        integrator.update(0.0, 0.0)
        integrator.set_command(0.0, 0.5)
        self.assertEqual(
            integrator.update(0.1, 0.0, observed_translation=0.005), (1.0, 2.0)
        )
        integrator.set_command(0.1, 0.5)
        x, y = integrator.update(0.2, 0.0, observed_translation=0.02)
        self.assertGreater(x, 1.0)
        self.assertAlmostEqual(y, 2.0)

    def test_command_pose_integrator_trusts_bounded_room_motion(self):
        integrator = CommandPoseIntegrator(1.0, 2.0)
        integrator.update(0.0, 0.0)
        integrator.set_command(0.0, 0.5)
        x, y = integrator.update(
            0.1,
            0.0,
            observed_translation=0.0,
            trust_command_motion=True,
        )
        self.assertGreater(x, 1.0)
        self.assertAlmostEqual(y, 2.0)

    def test_command_pose_integrator_synchronizes_to_trusted_lio_pose(self):
        integrator = CommandPoseIntegrator(0.0, -3.2)
        integrator.update(0.0, math.pi / 2.0)
        integrator.set_command(0.0, 0.6)
        integrator.update(0.1, math.pi / 2.0, trust_command_motion=True)
        self.assertNotEqual((integrator.x, integrator.y), (0.2, 5.0))
        self.assertEqual(
            integrator.synchronize(0.2, 5.0, 0.2, math.pi / 2.0),
            (0.2, 5.0),
        )
        # The next trusted room increment starts from the LIO-aligned anchor.
        integrator.set_command(0.2, 0.6)
        x, y = integrator.update(0.3, math.pi / 2.0, trust_command_motion=True)
        self.assertAlmostEqual(x, 0.2, places=6)
        self.assertGreater(y, 5.0)

    def test_preserves_normal_scan_matcher_motion(self):
        continuity = PoseContinuityFilter()
        self.assertEqual(continuity.update(0.0, 0.0, 0.0, 0.0, 0.0), (0.0, 0.0, False))
        x, y, jumped = continuity.update(0.1, 0.05, 0.01, 0.01, 0.01)
        self.assertFalse(jumped)
        self.assertAlmostEqual(x, 0.05)
        self.assertAlmostEqual(y, 0.01)

    def test_absorbs_translation_jump_and_keeps_later_increments(self):
        continuity = PoseContinuityFilter()
        continuity.update(0.0, 0.0, 0.0, 0.0, 0.0)
        continuity.update(0.1, 0.05, 0.0, 0.0, 0.0)
        x, y, jumped = continuity.update(0.2, 2.35, -1.0, 0.0, 0.0)
        self.assertTrue(jumped)
        self.assertAlmostEqual(x, 0.05)
        self.assertAlmostEqual(y, 0.0)
        x, y, jumped = continuity.update(0.3, 2.40, -1.0, 0.0, 0.0)
        self.assertFalse(jumped)
        self.assertAlmostEqual(x, 0.10)
        self.assertAlmostEqual(y, 0.0)
        self.assertEqual(continuity.absorbed_jumps, 1)

    def test_rotates_raw_increment_into_imu_heading_frame(self):
        continuity = PoseContinuityFilter()
        continuity.update(0.0, 0.0, 0.0, 0.0, math.pi / 2.0)
        x, y, jumped = continuity.update(0.1, 0.1, 0.0, 0.0, math.pi / 2.0)
        self.assertFalse(jumped)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.1, places=6)

    def test_absorbs_scan_matcher_rotation_jump(self):
        continuity = PoseContinuityFilter()
        continuity.update(0.0, 0.0, 0.0, 0.0, 0.0)
        x, y, jumped = continuity.update(0.1, 0.0, 0.0, 1.0, 0.05)
        self.assertTrue(jumped)
        self.assertEqual((x, y), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
