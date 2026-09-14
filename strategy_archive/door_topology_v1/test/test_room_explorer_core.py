#!/usr/bin/env python3

import math
import unittest
from dataclasses import replace

import numpy as np

from room_explorer_core import (
    ALIGN_TO_DOOR_NORMAL,
    BAD,
    DEGRADED,
    DISCOVERED,
    DOOR_CROSSING,
    DoorCandidateManager,
    DoorEvidence,
    DoorFusion,
    DoorVerifier,
    EXIT_ROOM,
    GOOD,
    GO_TO_PRE_DOOR,
    GridView,
    INCONCLUSIVE,
    LocalizationHealthMonitor,
    MissionFaultMonitor,
    MISSION_FAULT,
    MAP,
    OpeningDetector,
    OpeningCandidate,
    RecoveryManager,
    RoomFrontierPlanner,
    ROOM_SCAN,
    REJECTED,
    RoomMission,
    SCAN,
    STALE,
    UNREACHABLE,
    VERIFIED,
    VISITED,
    CorridorEstimator,
    corridor_entry_is_wide,
    corridor_is_elongated,
    corridor_sweep_needed,
    corridor_timeout_requires_reversal,
    door_discovery_allowed,
    door_corridor_side_reached,
    door_crossing_confirmed,
    door_crossing_errors,
    door_interior_reached,
    opposite_door_at_station,
    point_beyond_topology_gate,
    finite_median_clearance,
    finite_percentile_clearance,
    corridor_steering_correction,
    corridor_side_walls_present,
    select_observed_corridor_turn,
    closed_opening_measurement,
    corridor_approach_motion,
    projected_travel,
    normalize_angle,
    new_room_danger_ids,
    local_loop_icp,
    metric_axis_progress,
    transform_planar_point,
    transform_planar_yaw,
)


def synthetic_floor(seed):
    rng = np.random.RandomState(seed)
    resolution = 0.05
    data = np.zeros((480, 280), dtype=np.int8)
    origin_x, origin_y = -7.0, -2.0

    def wall_x(x, gaps):
        column = int(round((x - origin_x) / resolution))
        data[:, column - 1 : column + 2] = 100
        for center, width in gaps:
            row_min = int((center - width / 2.0 - origin_y) / resolution)
            row_max = int((center + width / 2.0 - origin_y) / resolution)
            data[row_min:row_max, column - 1 : column + 2] = 0

    jitter = rng.uniform(-0.03, 0.03, 4)
    left_gaps = [(4.0 + jitter[0], 1.2), (12.0 + jitter[1], 1.1)]
    right_gaps = [(7.0 + jitter[2], 1.25), (16.0 + jitter[3], 1.15)]
    wall_x(-1.2, left_gaps)
    wall_x(1.2, right_gaps)
    return GridView(data, resolution, origin_x, origin_y)


class OpeningDetectorTest(unittest.TestCase):
    def test_mission_fault_requires_sustained_sensor_failure(self):
        monitor = MissionFaultMonitor(bad_localization_duration=1.0)
        self.assertIsNone(monitor.evaluate(0.0, True, GOOD, "RL", "GOOD", 20))
        self.assertIsNone(monitor.evaluate(0.0, True, BAD, "RL", "GOOD", 20))
        self.assertEqual(
            monitor.evaluate(1.1, True, BAD, "RL", "GOOD", 20),
            "localization_bad_sustained",
        )

    def test_mission_fault_controller_fall_is_immediate(self):
        monitor = MissionFaultMonitor()
        self.assertEqual(
            monitor.evaluate(0.0, True, GOOD, "FALL", "GOOD", 20),
            "controller_fall",
        )

    def test_mission_fault_lidar_stall_is_bounded(self):
        monitor = MissionFaultMonitor(low_points_duration=0.5)
        self.assertIsNone(monitor.evaluate(0.0, True, GOOD, "RL", "GOOD", 0))
        self.assertEqual(
            monitor.evaluate(0.6, True, GOOD, "RL", "NO_EFFECTIVE_POINTS", 0),
            "lio_no_effective_points",
        )

    def test_door_verifier_accepts_opening_with_two_vertical_jambs(self):
        verifier = DoorVerifier()
        points = []
        for lateral in (-0.55, 0.55):
            for height in np.linspace(0.15, 2.0, 12):
                points.append((0.0, lateral, height))
        self.assertEqual(
            verifier.verify(points, (0.0, 0.0), 0.0, 1.1), VERIFIED
        )

    def test_door_verifier_rejects_blocked_gap_without_frame_support(self):
        verifier = DoorVerifier()
        points = [
            (0.0, lateral, height)
            for lateral in np.linspace(-0.35, 0.35, 12)
            for height in np.linspace(0.15, 1.2, 6)
        ]
        self.assertEqual(
            verifier.verify(points, (0.0, 0.0), 0.0, 1.1), REJECTED
        )

    def test_door_verifier_is_inconclusive_for_sparse_cloud(self):
        self.assertEqual(
            DoorVerifier().verify([(0.0, 0.0, 0.5)], (0.0, 0.0), 0.0, 1.1),
            INCONCLUSIVE,
        )

    def test_room_danger_baseline_includes_tracks_seen_while_crossing(self):
        baseline = {3, 7}
        self.assertEqual(new_room_danger_ids({3, 7}, baseline), set())
        self.assertEqual(new_room_danger_ids({3, 7, 9}, baseline), {9})

    def test_room_entry_metrics_measure_bounded_travel(self):
        self.assertAlmostEqual(
            projected_travel((1.0, 2.0), (1.0, 4.5), math.pi / 2.0),
            2.5,
        )
        self.assertAlmostEqual(
            metric_axis_progress(
                (0.0, -3.2),
                (0.0, 11.8),
                source_axis_yaw=0.0,
                source_pose_yaw=0.0,
                metric_pose_yaw=math.pi / 2.0,
            ),
            15.0,
        )
        self.assertFalse(door_interior_reached((1.0, 2.0), (1.59, 2.0), 0.0, 0.65))
        self.assertTrue(door_interior_reached((1.0, 2.0), (1.66, 2.0), 0.0, 0.65))
        self.assertFalse(
            door_corridor_side_reached((1.0, 2.0), (0.56, 2.0), 0.0, 0.45)
        )
        self.assertTrue(
            door_corridor_side_reached((1.0, 2.0), (0.54, 2.0), 0.0, 0.45)
        )

    def test_door_crossing_requires_centering_and_current_attempt_travel(self):
        depth, lateral = door_crossing_errors(
            (1.0, 2.0), (1.80, 2.18), 0.0
        )
        self.assertAlmostEqual(depth, 0.80)
        self.assertAlmostEqual(lateral, 0.18)
        self.assertTrue(
            door_crossing_confirmed(
                (1.0, 2.0),
                (1.80, 2.18),
                0.0,
                (0.90, 2.0),
                0.65,
                0.30,
                0.75,
            )
        )

        self.assertFalse(
            door_crossing_confirmed(
                (1.0, 2.0),
                (1.80, 2.36),
                0.0,
                (0.90, 2.0),
                0.65,
                0.30,
                0.75,
            )
        )
        self.assertFalse(
            door_crossing_confirmed(
                (1.0, 2.0),
                (1.80, 2.18),
                0.0,
                (1.20, 2.0),
                0.65,
                0.30,
                0.75,
            )
        )

    def test_directed_topology_gate_separates_lobby_and_corridor(self):
        gate = (0.0, 11.3)
        heading = math.pi / 2.0
        self.assertFalse(
            point_beyond_topology_gate(gate, (-1.8, 8.0), heading, 0.25)
        )
        self.assertFalse(
            point_beyond_topology_gate(gate, (0.0, 11.4), heading, 0.25)
        )
        self.assertTrue(
            point_beyond_topology_gate(gate, (1.1, 14.8), heading, 0.25)
        )

    def test_frontier_planner_respects_robot_body_clearance(self):
        resolution = 0.1
        data = np.zeros((30, 30), dtype=np.int8)
        data[:, 14:16] = 100
        grid = GridView(data, resolution, 0.0, 0.0)
        plan = RoomFrontierPlanner(robot_radius=0.42, safety_margin=0.18).plan(
            grid, (1.0, 1.0, 0.0), (1.0, 0.5), math.pi / 2.0
        )
        self.assertIn(plan.reason, ("NO_FRONTIER", "FRONTIER"))
        if plan.frontier is not None:
            self.assertGreaterEqual(plan.frontier.min_clearance, 0.60)

    def test_frontier_planner_keeps_laser_complete_room_open_for_camera(self):
        data = np.zeros((100, 100), dtype=np.int8)
        grid = GridView(data, 0.1, 0.0, 0.0)
        visual_seen = np.zeros(data.shape, dtype=bool)
        plan = RoomFrontierPlanner(
            robot_radius=0.20,
            safety_margin=0.0,
            frontier_min_cluster_cells=1,
            topology_entry_margin=0.2,
            topology_max_depth=7.0,
        ).plan(
            grid,
            (3.0, 5.0, 0.0),
            (2.0, 5.0),
            0.0,
            visual_seen=visual_seen,
            visual_coverage_target=0.70,
        )
        self.assertLess(plan.visual_coverage, 0.70)
        self.assertGreater(plan.visual_unseen_cells, 0)
        self.assertGreater(plan.visual_frontier_cells, 0)
        self.assertIsNotNone(plan.frontier)

    def test_frontier_planner_accepts_camera_complete_room(self):
        data = np.zeros((100, 100), dtype=np.int8)
        grid = GridView(data, 0.1, 0.0, 0.0)
        visual_seen = np.ones(data.shape, dtype=bool)
        plan = RoomFrontierPlanner(
            robot_radius=0.20,
            safety_margin=0.0,
            frontier_min_cluster_cells=1,
            topology_entry_margin=0.2,
            topology_max_depth=7.0,
        ).plan(
            grid,
            (3.0, 5.0, 0.0),
            (2.0, 5.0),
            0.0,
            visual_seen=visual_seen,
            visual_coverage_target=0.70,
        )
        self.assertAlmostEqual(plan.visual_coverage, 1.0)
        self.assertTrue(plan.visual_complete)
        self.assertEqual(plan.visual_unseen_cells, 0)

    def test_camera_viewpoints_remain_independent_after_one_is_visited(self):
        data = np.zeros((100, 100), dtype=np.int8)
        grid = GridView(data, 0.1, 0.0, 0.0)
        visual_seen = np.zeros(data.shape, dtype=bool)
        planner = RoomFrontierPlanner(
            robot_radius=0.20,
            safety_margin=0.0,
            frontier_min_cluster_cells=3,
            topology_entry_margin=0.2,
            topology_max_depth=7.0,
            target_revisit_radius=0.75,
        )
        initial = planner.plan(
            grid,
            (3.0, 5.0, 0.0),
            (2.0, 5.0),
            0.0,
            visual_seen=visual_seen,
            visual_coverage_target=0.70,
        )
        self.assertGreater(len(initial.frontiers), 1)
        revisited = planner.plan(
            grid,
            (3.0, 5.0, 0.0),
            (2.0, 5.0),
            0.0,
            visited_targets=[initial.frontier.target],
            visual_seen=visual_seen,
            visual_coverage_target=0.70,
        )
        self.assertIsNotNone(revisited.frontier)
        self.assertGreaterEqual(
            math.hypot(
                revisited.frontier.target[0] - initial.frontier.target[0],
                revisited.frontier.target[1] - initial.frontier.target[1],
            ),
            0.75,
        )

    def test_camera_ring_prefers_short_clockwise_steps_around_central_obstacle(self):
        data = np.zeros((120, 120), dtype=np.int8)
        data[48:72, 48:72] = 100
        grid = GridView(data, 0.1, 0.0, 0.0)
        visual_seen = np.zeros(data.shape, dtype=bool)
        plan = RoomFrontierPlanner(
            robot_radius=0.20,
            safety_margin=0.0,
            frontier_min_cluster_cells=3,
            topology_entry_margin=0.2,
            topology_max_depth=10.0,
            topology_lateral_limit=5.0,
            target_revisit_radius=0.75,
            visual_ring_enabled=True,
            visual_ring_clockwise=True,
            visual_ring_max_step=3.0,
            visual_ring_min_obstacle_cells=8,
        ).plan(
            grid,
            (2.5, 6.0, 0.0),
            (1.0, 6.0),
            0.0,
            visual_seen=visual_seen,
            visual_coverage_target=0.70,
        )
        self.assertTrue(plan.ring_detected)
        self.assertEqual(plan.ring_direction, "CLOCKWISE")
        self.assertGreater(plan.ring_candidate_count, 1)
        self.assertIsNotNone(plan.frontier)
        self.assertTrue(plan.frontier.visual)
        self.assertLessEqual(plan.frontier.path_length, 3.0)

    def test_frontier_planner_uses_four_connected_paths(self):
        # The only unknown frontier is at the end of an L-shaped free route;
        # every emitted waypoint must retain the route's cardinal turns.
        data = np.full((50, 50), 100, dtype=np.int8)
        data[5:40, 5:15] = 0
        data[30:40, 5:40] = 0
        data[4, 5:15] = -1
        grid = GridView(data, 0.1, 0.0, 0.0)
        planner = RoomFrontierPlanner(
            robot_radius=0.1,
            safety_margin=0.0,
            topology_entry_margin=0.2,
            minimum_escape_cells=1,
        )
        plan = planner.plan(grid, (3.0, 3.2, 0.0), (3.0, 0.2), math.pi / 2.0)
        self.assertIsNotNone(plan.frontier)
        for first, second in zip(plan.frontier.path, plan.frontier.path[1:]):
            self.assertTrue(
                math.isclose(first[0], second[0])
                or math.isclose(first[1], second[1]),
                msg="frontier path skipped a cardinal turn: %r -> %r" % (first, second),
            )

    def test_frontier_planner_rejects_wall_gap_narrower_than_robot(self):
        data = np.full((24, 24), -1, dtype=np.int8)
        data[4:20, 9:13] = 0
        data[4:20, 8] = 100
        data[4:20, 13] = 100
        grid = GridView(data, 0.1, 0.0, 0.0)
        plan = RoomFrontierPlanner(robot_radius=0.42, safety_margin=0.18).plan(
            grid, (1.05, 0.65, math.pi / 2.0), (1.05, 0.25), math.pi / 2.0
        )
        self.assertEqual(plan.reason, "NO_SAFE_ENTRY")

    def test_frontier_planner_rejects_unknown_wall_seam_narrower_than_robot(self):
        data = np.full((45, 45), 100, dtype=np.int8)
        data[5:35, 5:40] = 0
        # The wall return has a 0.4 m unknown seam.  Unknown is a frontier
        # candidate, but the adjacent free cells remain too close to the
        # observed jambs for the robot body to enter safely.
        data[18:22, 40] = -1
        grid = GridView(data, 0.1, 0.0, 0.0)
        planner = RoomFrontierPlanner(
            robot_radius=0.42,
            safety_margin=0.18,
            topology_entry_margin=0.2,
            frontier_min_cluster_cells=1,
        )
        plan = planner.plan(grid, (1.0, 1.0, 0.0), (1.0, 0.5), math.pi / 2.0)
        self.assertEqual(plan.reason, "NO_FRONTIER")

    def test_drifted_map_candidates_share_one_metric_door_position(self):
        metric_pose = (0.0, 15.0, math.pi / 2.0)
        first = transform_planar_point(
            (17.0, 1.0), (17.0, 0.0, 0.0), metric_pose
        )
        drifted = transform_planar_point(
            (22.0, 1.0), (22.0, 0.0, 0.0), metric_pose
        )
        self.assertAlmostEqual(first[0], drifted[0])
        self.assertAlmostEqual(first[1], drifted[1])
        self.assertAlmostEqual(
            transform_planar_yaw(math.pi / 2.0, 0.0, math.pi / 2.0),
            math.pi,
        )

    def test_validated_loop_closure_rebases_health_without_pose_jump(self):
        monitor = LocalizationHealthMonitor()
        monitor.update(1.0, 0.0, 0.0, 0.0)
        monitor.rebase(2.0, 1.5, -0.2, 0.1)
        snapshot = monitor.snapshot()
        self.assertEqual(snapshot["state"], GOOD)
        self.assertEqual(snapshot["reason"], "local_loop_closure")
        self.assertEqual(snapshot["max_translation_jump"], 0.0)
        self.assertEqual(monitor.update(2.1, 1.51, -0.2, 0.1), GOOD)

    def test_local_loop_icp_recovers_repeated_viewpoint_correction(self):
        x_axis = np.column_stack((np.linspace(-3.0, 3.0, 100), np.zeros(100)))
        y_axis = np.column_stack((np.zeros(80), np.linspace(-1.0, 4.0, 80)))
        diagonal = np.column_stack(
            (np.linspace(0.0, 2.0, 60), np.linspace(0.0, 1.0, 60))
        )
        reference = np.vstack((x_axis, y_axis, diagonal))
        angle = 0.04
        rotation = np.array(
            [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]]
        )
        current = np.matmul(rotation, reference.T).T + np.array([0.25, -0.18])
        result = local_loop_icp(reference, current)
        self.assertIsNotNone(result)
        corrected = np.matmul(
            result["transform"],
            np.column_stack((current, np.ones(len(current)))).T,
        ).T[:, :2]
        self.assertLess(np.mean(np.linalg.norm(corrected - reference, axis=1)), 0.05)
        self.assertLess(result["rmse"], 0.05)
        self.assertGreater(result["overlap"], 0.9)

    def test_corridor_discovery_is_disjoint_from_room_frontier_phase(self):
        self.assertTrue(door_discovery_allowed(None, "CORRIDOR_PROGRESSION"))
        self.assertFalse(door_discovery_allowed("opening_1", "ROOM_SCAN"))
        self.assertFalse(door_discovery_allowed(None, "ROOM_FRONTIER_EXPLORE"))
        self.assertFalse(door_discovery_allowed(None, "EXIT_ROOM"))
        self.assertFalse(door_discovery_allowed(None, None))

    def test_room_frontier_planner_requires_confirmed_room_topology(self):
        grid = GridView(np.zeros((20, 20), dtype=np.int8), 0.1, 0.0, 0.0)
        plan = RoomFrontierPlanner().plan(
            grid,
            (1.0, 1.0, 0.0),
            door_center=None,
            inward_yaw=None,
        )
        self.assertEqual(plan.reason, "NO_ROOM_TOPOLOGY")

    def test_frontier_coverage_uses_safe_traversable_space(self):
        # Wall-adjacent cells are grid-free but intentionally excluded by
        # body-radius inflation, so they must not lower achievable coverage.
        data = np.zeros((30, 30), dtype=np.int8)
        data[:, :3] = 100
        grid = GridView(data, 0.1, 0.0, 0.0)
        plan = RoomFrontierPlanner(
            robot_radius=0.2,
            safety_margin=0.0,
            topology_entry_margin=0.2,
            frontier_min_cluster_cells=1,
        ).plan(grid, (1.0, 1.0, 0.0), (1.0, 0.2), math.pi / 2.0)
        self.assertGreaterEqual(plan.coverage, plan.raw_coverage)
        self.assertEqual(plan.coverage, 1.0)

    def test_frontier_information_gain_is_reported_for_bounded_unknown_area(self):
        data = np.zeros((50, 50), dtype=np.int8)
        data[8:14, 8:14] = -1
        data[25:45, 25:45] = -1
        grid = GridView(data, 0.1, 0.0, 0.0)
        plan = RoomFrontierPlanner(
            robot_radius=0.1,
            safety_margin=0.0,
            topology_entry_margin=0.2,
            frontier_min_cluster_cells=1,
            information_radius=2.5,
            minimum_escape_cells=1,
        ).plan(grid, (0.8, 0.8, 0.0), (0.8, 0.2), math.pi / 2.0)
        self.assertIsNotNone(plan.frontier)
        self.assertGreater(plan.frontier.information_gain, 0.0)
        self.assertGreater(plan.frontier.target[0], 2.0)
        self.assertGreater(plan.frontier.target[1], 2.0)

    def test_committed_path_is_rejected_after_new_occupied_cell(self):
        data = np.zeros((20, 20), dtype=np.int8)
        grid = GridView(data, 0.1, 0.0, 0.0)
        planner = RoomFrontierPlanner(robot_radius=0.1, safety_margin=0.0)
        path = ((0.15, 0.15), (0.25, 0.15), (0.35, 0.15))
        self.assertTrue(planner.path_is_safe(grid, path))
        data[1, 2] = 100
        self.assertFalse(planner.path_is_safe(grid, path))

    def test_committed_path_can_use_revalidation_hysteresis(self):
        data = np.zeros((20, 20), dtype=np.int8)
        data[5, :] = 100
        grid = GridView(data, 0.1, 0.0, 0.0)
        planner = RoomFrontierPlanner(robot_radius=0.38, safety_margin=0.04)
        path = ((1.0, 0.9),)
        self.assertFalse(planner.path_is_safe(grid, path))
        self.assertTrue(planner.path_is_safe(grid, path, minimum_clearance=0.35))

    def test_front_clearance_ignores_missing_livox_returns(self):
        clearance = finite_percentile_clearance(
            [float("inf")] * 20 + [0.32, 0.35, 0.38]
        )
        self.assertAlmostEqual(clearance, 0.332, places=3)
        self.assertTrue(math.isinf(finite_percentile_clearance([float("inf")] * 20)))

    def test_side_measurement_ignores_missing_livox_returns(self):
        self.assertAlmostEqual(
            finite_median_clearance([float("inf"), 0.9, 1.1, float("inf")]),
            1.0,
        )
        self.assertTrue(math.isnan(finite_median_clearance([float("inf")] * 20)))

    def test_corridor_steering_uses_close_wall_as_emergency_evidence(self):
        correction, close_wall = corridor_steering_correction(
            0.0, 12.0, 12.0, 0.20, 12.0
        )
        self.assertTrue(close_wall)
        self.assertLess(correction, -0.25)

        correction, close_wall = corridor_steering_correction(
            0.0, 12.0, 12.0, 12.0, 0.20
        )
        self.assertTrue(close_wall)
        self.assertGreater(correction, 0.25)

    def test_corridor_steering_does_not_follow_an_open_door(self):
        correction, close_wall = corridor_steering_correction(
            0.0, 1.1, 12.0, 1.1, 12.0
        )
        self.assertFalse(close_wall)
        self.assertEqual(correction, 0.0)

    def test_corridor_steering_uses_sparse_side_nearest_point(self):
        correction, close_wall = corridor_steering_correction(
            0.0, 12.0, 4.1, 0.51, 1.95, wall_avoid_distance=0.75
        )
        self.assertTrue(close_wall)
        self.assertLess(correction, -0.10)

    def test_sparse_finite_returns_confirm_both_corridor_walls(self):
        self.assertTrue(corridor_side_walls_present(0.85, 1.15))
        self.assertTrue(corridor_side_walls_present(0.55, 1.65))
        self.assertFalse(corridor_side_walls_present(1.65, 1.65))
        self.assertFalse(corridor_side_walls_present(12.0, 1.15))
        self.assertFalse(corridor_side_walls_present(0.2, 1.15))

    def test_perpendicular_turn_requires_mapped_corridor_geometry(self):
        self.assertIsNone(
            select_observed_corridor_turn(0.0, 12.0, 5.0, 1.2, False, False)
        )
        self.assertAlmostEqual(
            select_observed_corridor_turn(0.0, 3.0, 5.0, 1.2, True, False),
            math.pi / 2.0,
        )
        self.assertAlmostEqual(
            select_observed_corridor_turn(0.0, 3.0, 5.0, 1.2, True, True),
            -math.pi / 2.0,
        )

    def test_opening_is_accepted_only_after_full_width_is_known(self):
        center, width = closed_opening_measurement(4.0, 5.2, 0.8, 1.6)
        self.assertAlmostEqual(center, 4.6)
        self.assertAlmostEqual(width, 1.2)
        self.assertIsNone(closed_opening_measurement(4.0, 6.4, 0.8, 1.6))

    def test_opening_requires_wall_support_before_its_leading_edge(self):
        self.assertIsNone(
            closed_opening_measurement(
                4.0, 5.2, 0.8, 1.6,
                leading_wall_start=None,
                min_leading_wall_length=0.5,
            )
        )
        self.assertIsNone(
            closed_opening_measurement(
                4.0, 5.2, 0.8, 1.6,
                leading_wall_start=3.7,
                min_leading_wall_length=0.5,
            )
        )
        center, width = closed_opening_measurement(
            4.0, 5.2, 0.8, 1.6,
            leading_wall_start=3.4,
            min_leading_wall_length=0.5,
        )
        self.assertAlmostEqual(center, 4.6)
        self.assertAlmostEqual(width, 1.2)

    def test_reverse_opening_accepts_leading_wall_in_reverse_direction(self):
        center, width = closed_opening_measurement(
            5.2, 4.0, 0.8, 1.6,
            leading_wall_start=5.8,
            min_leading_wall_length=0.5,
        )
        self.assertAlmostEqual(center, 4.6)
        self.assertAlmostEqual(width, 1.2)

    def test_door_detection_requires_an_elongated_corridor(self):
        grid = synthetic_floor(19)
        self.assertTrue(corridor_is_elongated(grid, (0.0, 8.0), math.pi / 2.0))

        lobby = GridView(np.zeros_like(grid.data), grid.resolution, grid.origin_x, grid.origin_y)
        self.assertFalse(corridor_is_elongated(lobby, (0.0, 8.0), math.pi / 2.0))

    def test_corridor_branch_rejects_door_width_bottleneck(self):
        grid = synthetic_floor(31)
        self.assertTrue(corridor_entry_is_wide(grid, (0.0, 0.0), math.pi / 2.0))
        self.assertFalse(corridor_entry_is_wide(grid, (0.0, 4.0), math.pi))

    def test_detects_multiple_bounded_room_openings_for_random_seeds(self):
        detector = OpeningDetector()
        for seed in (3, 19, 77, 20260822):
            candidates = detector.detect(synthetic_floor(seed), (0.0, 0.0), math.pi / 2.0)
            self.assertEqual(len(candidates), 4, seed)
            self.assertTrue(all(0.8 <= item.width <= 1.6 for item in candidates))
            self.assertTrue(all(item.pre_pose != item.post_pose for item in candidates))

    def test_rejects_gap_without_both_continuous_corridor_wall_jambs(self):
        resolution = 0.05
        data = np.zeros((120, 120), dtype=np.int8)
        origin_x, origin_y = -3.0, -2.0
        column = int(round((-1.2 - origin_x) / resolution))
        row_start = int(round((0.0 - origin_y) / resolution))
        row_end = int(round((2.0 - origin_y) / resolution))
        data[row_start:row_end, column - 1 : column + 2] = 100
        gap_start = int(round((0.6 - origin_y) / resolution))
        gap_end = int(round((1.6 - origin_y) / resolution))
        data[gap_start:gap_end, column - 1 : column + 2] = 0
        grid = GridView(data, resolution, origin_x, origin_y)

        # This bounded 1 m gap ends in only 0.4 m of wall, like the false
        # opening at the lobby/corridor junction seen in the Gazebo run.
        self.assertEqual(
            OpeningDetector().detect(grid, (0.0, 1.0), math.pi / 2.0), []
        )

    def test_local_observation_window_only_uses_nearby_wall_geometry(self):
        candidates = OpeningDetector(observation_range=3.0).detect(
            synthetic_floor(19), (0.0, 8.0), math.pi / 2.0
        )
        self.assertEqual(len(candidates), 1)
        self.assertAlmostEqual(candidates[0].center[1], 7.0, delta=0.1)

    def test_defer_zone_excludes_vertical_transition_opening(self):
        grid = synthetic_floor(19)
        defer_zone = [(-1.8, 11.2), (-0.7, 11.2), (-0.7, 12.8), (-1.8, 12.8)]
        candidates = OpeningDetector().detect(
            grid, (0.0, 0.0), math.pi / 2.0, defer_polygons=[defer_zone]
        )
        self.assertEqual(len(candidates), 3)
        self.assertTrue(all(not (11.2 < item.center[1] < 12.8) for item in candidates))


class RoomMissionTest(unittest.TestCase):
    def test_detected_door_can_be_approached_after_robot_passes_it(self):
        self.assertEqual(corridor_approach_motion(0.8, 0.0, 0.45), (0.0, 0.45))
        yaw, speed = corridor_approach_motion(-0.8, 0.0, 0.45)
        self.assertAlmostEqual(abs(yaw), math.pi)
        self.assertEqual(speed, 0.45)
        self.assertEqual(corridor_approach_motion(0.2, 0.0, 0.45), (0.0, 0.0))

    def test_corridor_timeout_uses_clearance_before_reversing(self):
        self.assertFalse(corridor_timeout_requires_reversal(2.0, 0.5, 0, 3))
        self.assertTrue(corridor_timeout_requires_reversal(0.3, 0.5, 0, 3))
        self.assertTrue(corridor_timeout_requires_reversal(2.0, 0.5, 3, 3))

    def test_corridor_rescan_is_bounded_and_requires_all_rooms(self):
        self.assertTrue(corridor_sweep_needed([VISITED, VISITED], 4, 0, 2))
        self.assertFalse(corridor_sweep_needed([VISITED] * 4, 4, 0, 2))
        self.assertFalse(corridor_sweep_needed([VISITED, VISITED], 4, 2, 2))

    def test_crossing_sequence_visits_each_room_once(self):
        candidates = OpeningDetector().detect(synthetic_floor(77), (0.0, 0.0), math.pi / 2.0)
        mission = RoomMission(max_attempts=2, quiet_period=5.0)
        self.assertEqual(mission.merge_candidates(candidates, sim_time=1.0), 4)
        self.assertEqual(mission.merge_candidates(candidates, sim_time=2.0), 0)

        expected = [
            GO_TO_PRE_DOOR,
            ALIGN_TO_DOOR_NORMAL,
            DOOR_CROSSING,
            ROOM_SCAN,
            EXIT_ROOM,
            VISITED,
        ]
        for _ in range(4):
            active = mission.next_candidate((0.0, 0.0))
            self.assertEqual(active.status, expected[0])
            actual = [active.status]
            while mission.active_id is not None:
                actual.append(mission.advance(True))
            self.assertEqual(actual, expected)
        self.assertTrue(all(item.status == VISITED for item in mission.candidates.values()))
        self.assertFalse(mission.complete(5.9))
        self.assertTrue(mission.complete(6.0))

    def test_failed_door_has_bounded_retries(self):
        candidate = OpeningDetector().detect(
            synthetic_floor(3), (0.0, 0.0), math.pi / 2.0
        )[0]
        mission = RoomMission(max_attempts=2)
        mission.merge_candidates([candidate], sim_time=0.0)
        mission.next_candidate((0.0, 0.0))
        self.assertEqual(mission.advance(False), DISCOVERED)
        mission.next_candidate((0.0, 0.0))
        self.assertEqual(mission.advance(False), UNREACHABLE)
        self.assertIsNone(mission.next_candidate((0.0, 0.0)))

    def test_failed_exit_retries_exit_without_reentering_room(self):
        candidate = OpeningDetector().detect(
            synthetic_floor(3), (0.0, 0.0), math.pi / 2.0
        )[0]
        mission = RoomMission(max_attempts=2)
        mission.merge_candidates([candidate], sim_time=0.0)
        candidate = mission.next_candidate((0.0, 0.0))
        for _ in range(4):
            mission.advance(True)
        self.assertEqual(candidate.status, EXIT_ROOM)
        self.assertEqual(mission.advance(False), EXIT_ROOM)
        self.assertEqual(candidate.attempts, 1)
        self.assertEqual(candidate.exit_attempts, 2)
        self.assertEqual(mission.active_id, candidate.candidate_id)
        self.assertEqual(mission.advance(True), VISITED)
        self.assertIsNone(mission.active_id)

    def test_small_map_quantization_shift_does_not_duplicate_room(self):
        candidates = OpeningDetector().detect(
            synthetic_floor(19), (0.0, 0.0), math.pi / 2.0
        )
        shifted = OpeningDetector().detect(
            synthetic_floor(19), (0.03, 0.02), math.pi / 2.0
        )
        mission = RoomMission()
        self.assertEqual(mission.merge_candidates(candidates, sim_time=1.0), 4)
        self.assertEqual(mission.merge_candidates(shifted, sim_time=2.0), 0)

    def test_visited_door_revisit_with_new_detector_id_is_rejected(self):
        candidate = OpeningDetector().detect(
            synthetic_floor(23), (0.0, 0.0), math.pi / 2.0
        )[0]
        mission = RoomMission(
            visited_revisit_radius=3.0, visited_revisit_normal_gate=0.70
        )
        self.assertEqual(mission.merge_candidates([candidate], sim_time=0.0), 1)
        active = mission.next_candidate((0.0, 0.0))
        for _ in range(5):
            mission.advance(True)
        self.assertEqual(active.status, VISITED)
        shifted = replace(
            candidate,
            candidate_id="reobserved_with_new_id",
            center=(candidate.center[0] + 2.4, candidate.center[1] + 0.3),
        )
        self.assertEqual(mission.merge_candidates([shifted], sim_time=10.0), 0)
        self.assertEqual(mission.revisit_rejections, 1)

    def test_exit_observed_opposite_door_is_prioritized_once(self):
        candidates = [
            OpeningCandidate(
                candidate_id="right_door",
                center=(1.1, 10.0),
                width=1.2,
                normal_yaw=0.0,
                pre_pose=(1.1, 9.35, 0.0),
                post_pose=(1.1, 11.0, 0.0),
            ),
            OpeningCandidate(
                candidate_id="opposite_left_door",
                center=(-1.1, 10.4),
                width=1.2,
                normal_yaw=math.pi,
                pre_pose=(-1.1, 9.75, math.pi),
                post_pose=(-1.1, 11.4, math.pi),
            ),
            OpeningCandidate(
                candidate_id="forward_left_door",
                center=(-1.1, 13.0),
                width=1.2,
                normal_yaw=math.pi,
                pre_pose=(-1.1, 12.35, math.pi),
                post_pose=(-1.1, 14.0, math.pi),
            ),
        ]
        mission = RoomMission(opposite_pair_station_tolerance=1.0)
        mission.merge_candidates(candidates, sim_time=0.0)
        active = mission.next_candidate((1.1, 9.0), corridor_yaw=math.pi / 2.0)
        self.assertEqual(active.candidate_id, "right_door")
        for _ in range(5):
            mission.advance(True)
        rejected_observation = replace(
            DoorEvidence(
                source=MAP,
                timestamp=1.0,
                side=-1.0,
                center=(-1.1, 10.4),
                width=1.2,
                normal_yaw=math.pi,
                pre_pose=(-1.1, 9.75, math.pi),
                post_pose=(-1.1, 11.4, math.pi),
                corridor_confidence=0.9,
                source_confidence=0.9,
                opening_complete=True,
            ),
            verification_status=REJECTED,
        )
        self.assertIsNone(
            mission.mark_exit_opposite_from_evidence(
                [rejected_observation],
                (1.1, 10.0),
                0.0,
                math.pi / 2.0,
                1.0,
            )
        )
        observed = DoorEvidence(
            source=MAP,
            timestamp=1.0,
            side=-1.0,
            center=(-1.1, 10.4),
            width=1.2,
            normal_yaw=math.pi,
            pre_pose=(-1.1, 9.75, math.pi),
            post_pose=(-1.1, 11.4, math.pi),
            corridor_confidence=0.9,
            source_confidence=0.9,
            opening_complete=True,
        )
        self.assertEqual(
            mission.mark_exit_opposite_from_evidence(
                [observed],
                (1.1, 10.0),
                0.0,
                math.pi / 2.0,
                1.0,
            ),
            "opposite_left_door",
        )
        paired = mission.next_candidate(
            (1.1, 10.0), corridor_yaw=math.pi / 2.0, prefer_opposite=True
        )
        self.assertEqual(paired.candidate_id, "opposite_left_door")
        self.assertEqual(mission.opposite_pair_selections, 1)
        self.assertEqual(mission.last_selection_reason, "EXIT_OPPOSITE")

    def test_unmarked_opposite_door_does_not_change_normal_order(self):
        candidates = [
            OpeningCandidate(
                candidate_id="near_forward",
                center=(1.1, 10.0),
                width=1.2,
                normal_yaw=0.0,
                pre_pose=(1.1, 9.35, 0.0),
                post_pose=(1.1, 11.0, 0.0),
            ),
            OpeningCandidate(
                candidate_id="far_opposite",
                center=(-1.1, 14.0),
                width=1.2,
                normal_yaw=math.pi,
                pre_pose=(-1.1, 13.35, math.pi),
                post_pose=(-1.1, 15.0, math.pi),
            ),
        ]
        mission = RoomMission()
        mission.merge_candidates(candidates, sim_time=0.0)
        selected = mission.next_candidate(
            (1.1, 9.0), corridor_yaw=math.pi / 2.0, prefer_opposite=True
        )
        self.assertEqual(selected.candidate_id, "near_forward")

    def test_paired_station_precedes_isolated_fragment(self):
        candidates = [
            OpeningCandidate(
                candidate_id="isolated_fragment",
                center=(1.1, 18.0),
                width=0.9,
                normal_yaw=0.0,
                pre_pose=(1.1, 17.35, 0.0),
                post_pose=(1.1, 19.0, 0.0),
            ),
            OpeningCandidate(
                candidate_id="paired_right",
                center=(1.1, 28.0),
                width=1.1,
                normal_yaw=0.0,
                pre_pose=(1.1, 27.35, 0.0),
                post_pose=(1.1, 29.0, 0.0),
            ),
            OpeningCandidate(
                candidate_id="paired_left",
                center=(-1.1, 28.0),
                width=1.1,
                normal_yaw=math.pi,
                pre_pose=(-1.1, 27.35, math.pi),
                post_pose=(-1.1, 29.0, math.pi),
            ),
        ]
        mission = RoomMission()
        mission.merge_candidates(candidates, sim_time=0.0)
        selected = mission.next_candidate(
            (0.0, 10.0),
            corridor_yaw=math.pi / 2.0,
            opposite_pair_station_tolerance=1.5,
        )
        self.assertEqual(selected.candidate_id, "paired_right")

    def test_pair_filter_can_be_disabled_for_immediate_near_door_entry(self):
        candidates = [
            OpeningCandidate(
                candidate_id="near_single",
                center=(1.1, 14.0),
                width=1.1,
                normal_yaw=0.0,
                pre_pose=(0.45, 14.0, 0.0),
                post_pose=(2.1, 14.0, 0.0),
            ),
            OpeningCandidate(
                candidate_id="far_right",
                center=(1.1, 28.0),
                width=1.1,
                normal_yaw=0.0,
                pre_pose=(0.45, 28.0, 0.0),
                post_pose=(2.1, 28.0, 0.0),
            ),
            OpeningCandidate(
                candidate_id="far_left",
                center=(-1.1, 28.0),
                width=1.1,
                normal_yaw=math.pi,
                pre_pose=(-0.45, 28.0, math.pi),
                post_pose=(-2.1, 28.0, math.pi),
            ),
        ]
        mission = RoomMission()
        mission.merge_candidates(candidates, sim_time=0.0)
        selected = mission.next_candidate(
            (0.0, 10.0),
            corridor_yaw=math.pi / 2.0,
            opposite_pair_station_tolerance=1.0,
            prefer_paired_station=False,
        )
        self.assertEqual(selected.candidate_id, "near_single")

    def test_later_candidate_observation_refreshes_discovered_waypoint(self):
        mission = DoorCandidateManager(dedup_radius=2.0)
        first = OpeningCandidate(
            candidate_id="door",
            center=(1.0, 14.4),
            width=0.9,
            normal_yaw=0.0,
            pre_pose=(0.35, 14.4, 0.0),
            post_pose=(2.0, 14.4, 0.0),
            scan_support=True,
            confidence=0.75,
        )
        refined = OpeningCandidate(
            candidate_id="door",
            center=(1.1, 14.9),
            width=1.2,
            normal_yaw=0.0,
            pre_pose=(0.45, 14.9, 0.0),
            post_pose=(2.1, 14.9, 0.0),
            map_support=True,
            confidence=0.9,
        )
        self.assertEqual(mission.merge_candidates([first], sim_time=0.0), 1)
        self.assertEqual(mission.merge_candidates([refined], sim_time=1.0), 0)
        candidate = mission.candidates["door"]
        self.assertEqual(candidate.center, refined.center)
        self.assertEqual(candidate.pre_pose, refined.pre_pose)
        self.assertTrue(candidate.scan_support)
        self.assertTrue(candidate.map_support)

    def test_opposite_door_at_station_requires_cross_corridor_same_station(self):
        self.assertTrue(
            opposite_door_at_station(
                (1.1, 10.0), 0.0, (-1.1, 10.4), math.pi, math.pi / 2.0, 1.0
            )
        )
        self.assertFalse(
            opposite_door_at_station(
                (1.1, 10.0), 0.0, (-1.1, 13.0), math.pi, math.pi / 2.0, 1.0
            )
        )
        self.assertFalse(
            opposite_door_at_station(
                (1.1, 10.0), 0.0, (1.1, 10.4), 0.0, math.pi / 2.0, 1.0
            )
        )

    def test_completion_allows_extra_unvisited_candidate_and_waits_for_danger(self):
        candidates = OpeningDetector().detect(
            synthetic_floor(19), (0.0, 0.0), math.pi / 2.0
        )
        false_candidate = OpeningDetector().detect(
            synthetic_floor(19), (0.03, 0.02), math.pi / 2.0
        )[0]
        false_candidate.candidate_id = "false_extra"
        false_candidate.center = (4.0, 20.0)
        mission = DoorCandidateManager(quiet_period=3.0)
        mission.merge_candidates(candidates + [false_candidate], sim_time=1.0)
        for candidate in list(mission.candidates.values())[:4]:
            candidate.status = VISITED
        mission.completion_ready_time = 5.0
        self.assertIsNone(mission.next_candidate((0.0, 0.0), expected_room_count=4))
        self.assertFalse(
            mission.complete(8.0, expected_room_count=4, danger_confirmation_active=True)
        )
        self.assertTrue(mission.complete(8.0, expected_room_count=4))

    def test_visited_same_side_station_rejects_nearby_map_fragment(self):
        mission = DoorCandidateManager(visited_revisit_radius=4.0)
        first = OpeningCandidate(
            candidate_id="visited_door",
            center=(1.1, 14.4),
            width=1.1,
            normal_yaw=0.0,
            pre_pose=(1.1, 13.75, 0.0),
            post_pose=(1.1, 15.4, 0.0),
            status=VISITED,
        )
        mission.candidates[first.candidate_id] = first
        fragment = OpeningCandidate(
            candidate_id="map_fragment",
            center=(1.1, 18.0),
            width=0.9,
            normal_yaw=0.0,
            pre_pose=(1.1, 17.35, 0.0),
            post_pose=(1.1, 19.0, 0.0),
        )
        self.assertEqual(mission.merge_candidates([fragment], sim_time=2.0), 0)
        self.assertEqual(mission.revisit_rejections, 1)


def door_evidence(
    source, health=GOOD, confidence=0.8, x=1.0, verification=INCONCLUSIVE
):
    return DoorEvidence(
        source=source,
        timestamp=1.0 if source == SCAN else 2.0,
        side=1.0,
        center=(x, 2.0),
        width=1.1,
        normal_yaw=math.pi / 2.0,
        pre_pose=(x, 1.4, math.pi / 2.0),
        post_pose=(x, 3.0, math.pi / 2.0),
        corridor_confidence=confidence,
        source_confidence=confidence,
        opening_complete=True,
        localization_health=health,
        verification_status=verification,
    )


class StageBArchitectureTest(unittest.TestCase):
    def test_corridor_estimator_is_single_geometry_contract(self):
        estimator = CorridorEstimator(min_width=1.7, max_width=2.7)
        estimate = estimator.estimate(0.1, 0.9, 1.1, 4.0, center_error=0.1)
        self.assertTrue(estimate.valid)
        self.assertEqual(estimate.confidence, 1.0)
        self.assertAlmostEqual(estimate.corridor_width, 2.0)
        self.assertFalse(estimator.estimate(0.1, 0.9, 2.5, 4.0).valid)

    def test_localization_health_detects_jump_and_stale_pose(self):
        monitor = LocalizationHealthMonitor(stale_timeout=1.0, recovery_samples=2)
        monitor.set_command(0.4, 0.0)
        self.assertEqual(monitor.update(1.0, 0.0, 0.0, 0.0), GOOD)
        self.assertEqual(monitor.update(1.1, 0.04, 0.0, 0.0), GOOD)
        self.assertEqual(monitor.update(1.2, 1.5, 0.0, 0.0), BAD)
        self.assertGreater(monitor.snapshot()["max_translation_jump"], 1.0)
        self.assertEqual(monitor.evaluate(2.3), STALE)

    def test_strong_scan_or_good_map_can_confirm_independently(self):
        scan_fusion = DoorFusion()
        self.assertEqual(len(scan_fusion.ingest(door_evidence(SCAN))), 1)
        map_fusion = DoorFusion()
        self.assertEqual(len(map_fusion.ingest(door_evidence(MAP, GOOD))), 1)

    def test_scan_only_confirmation_requires_verified_frame_support(self):
        fusion = DoorFusion(require_scan_verification=True)
        self.assertEqual(fusion.ingest(door_evidence(SCAN)), [])
        self.assertEqual(
            len(fusion.ingest(door_evidence(SCAN, verification=VERIFIED))), 1
        )

    def test_map_strong_can_confirm_when_scan_verification_is_inconclusive(self):
        fusion = DoorFusion(require_scan_verification=True)
        self.assertEqual(len(fusion.ingest(door_evidence(MAP))), 1)

    def test_rejected_verifier_evidence_never_creates_hypothesis(self):
        fusion = DoorFusion(require_scan_verification=True)
        self.assertEqual(
            fusion.ingest(door_evidence(SCAN, verification=REJECTED)), []
        )
        self.assertEqual(fusion.hypotheses, {})

    def test_degraded_map_cannot_create_but_can_support_scan(self):
        fusion = DoorFusion()
        self.assertEqual(fusion.ingest(door_evidence(MAP, DEGRADED)), [])
        self.assertEqual(fusion.hypotheses, {})
        self.assertEqual(
            fusion.ingest(door_evidence(SCAN, GOOD, confidence=0.55)), []
        )
        confirmed = fusion.ingest(door_evidence(MAP, DEGRADED, confidence=0.55))
        self.assertEqual(len(confirmed), 1)
        self.assertTrue(confirmed[0].scan_support)
        self.assertTrue(confirmed[0].map_support)

    def test_confirmed_hypothesis_is_not_emitted_twice(self):
        fusion = DoorFusion()
        self.assertEqual(len(fusion.ingest(door_evidence(SCAN))), 1)
        self.assertEqual(fusion.ingest(door_evidence(SCAN)), [])
        self.assertEqual(len(fusion.hypotheses), 1)

    def test_incremental_map_fragments_keep_one_door_owner(self):
        fusion = DoorFusion(spatial_gate=2.0)
        self.assertEqual(len(fusion.ingest(door_evidence(MAP, x=1.0))), 1)
        self.assertEqual(fusion.ingest(door_evidence(MAP, x=2.65)), [])
        self.assertEqual(len(fusion.hypotheses), 1)

    def test_fused_waypoints_remain_tied_to_fused_center(self):
        fusion = DoorFusion(spatial_gate=2.0)
        self.assertEqual(len(fusion.ingest(door_evidence(MAP, x=1.0))), 1)
        fusion.ingest(door_evidence(MAP, x=1.4))
        hypothesis = next(iter(fusion.hypotheses.values()))
        self.assertAlmostEqual(
            hypothesis.center[1] - hypothesis.pre_pose[1], 0.6, places=6
        )
        self.assertAlmostEqual(
            hypothesis.post_pose[1] - hypothesis.center[1], 1.0, places=6
        )

    def test_recovery_budgets_are_independent_and_bounded(self):
        recovery = RecoveryManager(max_door_attempts=2, max_exit_attempts=2, max_sweeps=1)
        self.assertTrue(recovery.begin_door_attempt("door"))
        self.assertTrue(recovery.begin_door_attempt("door"))
        self.assertFalse(recovery.begin_door_attempt("door"))
        self.assertTrue(recovery.begin_exit_attempt("door"))
        self.assertTrue(recovery.begin_exit_attempt("door"))
        self.assertFalse(recovery.begin_exit_attempt("door"))
        self.assertTrue(recovery.request_reverse_sweep())
        self.assertFalse(recovery.request_reverse_sweep())


if __name__ == "__main__":
    unittest.main()
