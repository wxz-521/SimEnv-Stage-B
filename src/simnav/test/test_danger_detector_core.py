#!/usr/bin/env python3

import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from danger_detector_core import (
    CameraIntrinsics,
    DangerTracker,
    RedBallDetector,
    ResultWriter,
    position_on_floor,
    project_pose_from_anchor,
    rasterize_planar_ray,
)


class RedBallDetectorTest(unittest.TestCase):
    def setUp(self):
        self.detector = RedBallDetector()
        self.intrinsics = CameraIntrinsics(400.0, 400.0, 160.0, 120.0)

    def test_red_ball_is_detected_with_metric_camera_position(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        depth = np.full((240, 320), 4.0, dtype=np.float32)
        cv2.circle(image, (190, 105), 24, (0, 0, 255), -1)
        cv2.circle(depth, (190, 105), 24, 2.0, -1)
        observations = self.detector.detect(image, depth, self.intrinsics)
        self.assertEqual(len(observations), 1)
        x, y, z = observations[0].position_camera
        self.assertAlmostEqual(x, 0.15, places=2)
        self.assertAlmostEqual(y, -0.075, places=2)
        self.assertAlmostEqual(z, 2.0, places=2)

    def test_red_square_and_green_ball_are_rejected(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        depth = np.full((240, 320), 2.0, dtype=np.float32)
        cv2.rectangle(image, (35, 75), (95, 135), (0, 0, 255), -1)
        cv2.circle(image, (220, 110), 28, (0, 255, 0), -1)
        self.assertEqual(self.detector.detect(image, depth, self.intrinsics), [])

    def test_invalid_depth_does_not_create_target(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        depth = np.zeros((240, 320), dtype=np.float32)
        cv2.circle(image, (160, 120), 24, (0, 0, 255), -1)
        self.assertEqual(self.detector.detect(image, depth, self.intrinsics), [])


class DangerTrackerTest(unittest.TestCase):
    def test_camera_ray_rasterization_keeps_horizontal_cells_connected(self):
        cells = rasterize_planar_ray((0.0, 0.0), (1.0, 0.75), 0.25)
        self.assertEqual(cells[0], (0, 0))
        self.assertEqual(cells[-1], (4, 3))
        self.assertTrue(
            all(
                max(abs(next_cell[0] - cell[0]), abs(next_cell[1] - cell[1])) <= 1
                for cell, next_cell in zip(cells, cells[1:])
            )
        )

    def test_camera_ray_rasterization_handles_negative_coordinates(self):
        self.assertEqual(
            rasterize_planar_ray((-0.01, -0.01), (-0.51, -0.26), 0.25)[-1],
            (-3, -2),
        )

    def test_room_entry_projection_uses_relative_source_motion(self):
        projected = project_pose_from_anchor(
            source_pose=(10.0 + 3.0 * np.cos(0.2), 20.0 + 3.0 * np.sin(0.2), 0.2),
            source_anchor=(10.0, 20.0, 0.2),
            metric_anchor=(4.0, 10.0, 0.5 * np.pi),
        )
        self.assertTrue(np.allclose(projected[:2], [4.0, 13.0]))
        self.assertAlmostEqual(projected[2], 0.5 * np.pi)

    def test_room_entry_projection_rotates_source_relative_motion(self):
        projected = project_pose_from_anchor(
            source_pose=(2.0, 0.0, 1.5 * np.pi),
            source_anchor=(0.0, 0.0, 0.5 * np.pi),
            metric_anchor=(7.0, -2.0, 0.0),
        )
        self.assertTrue(np.allclose(projected[:2], [7.0, -4.0]))
        self.assertAlmostEqual(projected[2], np.pi)

    def test_loop_closure_translation_keeps_tracks_in_corrected_world_frame(self):
        tracker = DangerTracker(confirmation_frames=1)
        tracker.update([(1.0, 2.0, 0.2)], 1.0)
        tracker.translate((0.5, -0.25, 0.0))
        self.assertTrue(
            np.allclose(tracker.confirmed()[0].position_world, [1.5, 1.75, 0.2])
        )

    def test_anchor_loop_correction_preserves_reliable_tracks_only(self):
        tracker = DangerTracker(confirmation_frames=1, cluster_radius=0.5)
        tracker.update([(1.0, 2.0, 0.2)], 1.0)
        tracker.update([(4.0, 5.0, 0.2)], 2.0)
        transform = np.asarray(
            [[0.0, -1.0, 0.5], [1.0, 0.0, -0.25], [0.0, 0.0, 1.0]]
        )
        tracker.transform_except(transform, preserved_track_ids=[0])
        self.assertTrue(np.allclose(tracker.tracks[0].position_world, [1.0, 2.0, 0.2]))
        self.assertTrue(np.allclose(tracker.tracks[1].position_world, [-4.5, 3.75, 0.2]))

    def test_post_loop_duplicate_merges_into_preserved_track(self):
        tracker = DangerTracker(confirmation_frames=1, cluster_radius=0.75)
        tracker.update([(5.6, 15.3, 0.15)], 1.0)
        tracker.update([(4.7, 15.3, 0.15)], 2.0)
        self.assertEqual(len(tracker.tracks), 2)
        tracker.merge_nearby_tracks([0], max_distance=1.0)
        self.assertEqual(len(tracker.tracks), 1)
        self.assertEqual(tracker.tracks[0].track_id, 0)
        self.assertEqual(tracker.tracks[0].observations, 2)

    def test_floor_gate_rejects_upper_floor_observation(self):
        self.assertTrue(position_on_floor((1.0, 2.0, 0.15), floor_z=0.0))
        self.assertFalse(position_on_floor((1.0, 2.0, 2.48), floor_z=0.0))

    def test_multiframe_confirmation_and_spatial_deduplication(self):
        tracker = DangerTracker(confirmation_frames=3, cluster_radius=0.6)
        self.assertEqual(tracker.update([(1.0, 2.0, 0.3)], 1.0), [])
        self.assertEqual(tracker.update([(1.1, 1.9, 0.3)], 2.0), [])
        confirmed = tracker.update([(0.95, 2.05, 0.3)], 3.0)
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(confirmed[0].observations, 3)
        confirmed = tracker.update([(1.05, 2.02, 0.32)], 4.0)
        self.assertEqual(len(confirmed), 1)
        self.assertEqual(len(tracker.tracks), 1)

    def test_spatially_separate_balls_create_separate_tracks(self):
        tracker = DangerTracker(confirmation_frames=1, cluster_radius=0.5)
        confirmed = tracker.update([(0.0, 0.0, 0.3), (2.0, 0.0, 0.3)], 1.0)
        self.assertEqual(len(confirmed), 2)

    def test_one_to_one_frame_assignment_preserves_nearby_balls(self):
        tracker = DangerTracker(confirmation_frames=2, cluster_radius=0.75)
        tracker.update([(0.0, 0.0, 0.3), (0.70, 0.0, 0.3)], 1.0)
        confirmed = tracker.update([(0.02, 0.0, 0.3), (0.72, 0.0, 0.3)], 2.0)
        self.assertEqual(len(confirmed), 2)

    def test_reassociates_same_ball_across_bounded_viewpoint_drift(self):
        tracker = DangerTracker(confirmation_frames=1, cluster_radius=0.75)
        tracker.update([(5.36, 15.62, 0.17)], 1.0)
        confirmed = tracker.update([(4.90, 16.06, 0.17)], 2.0)
        self.assertEqual(len(confirmed), 1)

    def test_track_records_position_variance_and_localization_health(self):
        tracker = DangerTracker(confirmation_frames=2, cluster_radius=2.0)
        tracker.update([(1.0, 2.0, 0.3)], 1.0, localization_health="GOOD")
        confirmed = tracker.update(
            [(2.0, 2.0, 0.3)], 2.0, localization_health="BAD"
        )
        self.assertEqual(len(confirmed), 1)
        self.assertAlmostEqual(confirmed[0].position_variance[0], 0.5)
        self.assertEqual(
            confirmed[0].localization_health_counts, {"GOOD": 1, "BAD": 1}
        )

    def test_result_writer_separates_official_and_debug_schema(self):
        tracker = DangerTracker(confirmation_frames=2, cluster_radius=0.6)
        tracker.update([(1.0, 2.0, 0.3)], 1.0)
        confirmed = tracker.update([(1.1, 2.0, 0.3)], 2.0)
        with tempfile.TemporaryDirectory() as directory:
            formal_path = Path(directory) / "detected_danger.json"
            debug_path = Path(directory) / "detected_danger_debug.json"
            writer = ResultWriter(formal_path, debug_path)
            writer.write(tracker.tracks, confirmed, 12.5, floor_complete=True)
            formal = json.loads(formal_path.read_text(encoding="utf-8"))
            debug = json.loads(debug_path.read_text(encoding="utf-8"))
        self.assertEqual(
            set(formal), {"exploration_time", "detected_danger_sources"}
        )
        self.assertEqual(formal["exploration_time"], 12.5)
        self.assertEqual(len(formal["detected_danger_sources"]), 1)
        self.assertIn("position_variance", debug["dangers"][0])
        self.assertTrue(debug["floor_complete"])


if __name__ == "__main__":
    unittest.main()
