#!/usr/bin/env python3

import math
import unittest

import numpy as np

from coverage_explorer_core import (
    GridView,
    RoomPortal,
    TaskCoveragePlanner,
    corrected_portal_heading,
    coverage_classification,
    detect_lobby_portals,
    coverage_snapshot,
    detect_room_portals,
    infer_task_extent,
    pair_room_portals,
    portal_return_along_offsets,
    task_region_mask,
    topology_region_mask,
    topology_id_for_point,
    topology_state_for_new_target,
    target_kind_allowed_for_topology_state,
    topology_completion_ready,
    weighted_linear_coverage,
    weighted_harmonic_coverage,
)


def synthetic_floor(include_second_opening=True, include_end_wall=True):
    resolution = 0.10
    grid = GridView(
        data=np.full((220, 400), -1, dtype=np.int16),
        resolution=resolution,
        origin_x=-4.0,
        origin_y=-11.0,
    )

    def box(x0, x1, y0, y1, value):
        row0, column0 = grid.world_to_cell(x0, y0)
        row1, column1 = grid.world_to_cell(x1, y1)
        grid.data[min(row0, row1) : max(row0, row1) + 1,
                  min(column0, column1) : max(column0, column1) + 1] = value

    # Main corridor and four-room interior are known free.  The central narrow
    # corridor deliberately continues beyond the rooms.
    box(-3.5, 34.5, -1.0, 1.0, 0)
    box(-3.5, 24.5, 1.2, 9.3, 0)
    box(-3.5, 24.5, -9.3, -1.2, 0)
    # Corridor side walls, with one or two paired door gaps.
    box(-3.5, 24.5, 1.05, 1.15, 100)
    box(-3.5, 24.5, -1.15, -1.05, 100)
    for centre in ([3.5, 17.5] if include_second_opening else [3.5]):
        box(centre - 0.50, centre + 0.50, 1.00, 1.20, 0)
        box(centre - 0.50, centre + 0.50, -1.20, -1.00, 0)
    if include_end_wall:
        box(24.45, 24.65, 1.2, 9.5, 100)
        box(24.45, 24.65, -9.5, -1.2, 100)
    return grid


class CoverageMathTest(unittest.TestCase):
    def test_lobby_portal_detector_uses_separate_prefix(self):
        grid = synthetic_floor()
        portals = detect_lobby_portals(grid, (0.0, 7.0), math.pi / 2.0)
        self.assertTrue(all(item.topology_id.startswith("LOBBY_") for item in portals))
    def test_room_combined_coverage_uses_five_percent_lidar_by_default(self):
        self.assertAlmostEqual(weighted_linear_coverage(1.0, 0.84, 0.90), 0.856)
        self.assertAlmostEqual(weighted_linear_coverage(1.0, 0.70, 0.90), 0.73)
        self.assertAlmostEqual(weighted_linear_coverage(1.0, 0.80), 0.81)

    def test_floor_completion_requires_four_returned_rooms(self):
        rooms = ("ROOM_L_15", "ROOM_R_15", "ROOM_L_29", "ROOM_R_29")
        self.assertFalse(topology_completion_ready(rooms[:3], 4, 0))
        self.assertFalse(topology_completion_ready(rooms, 4, 1))
        self.assertTrue(topology_completion_ready(rooms, 4, 0))

    def test_portal_heading_faces_room_with_small_centre_correction(self):
        self.assertAlmostEqual(corrected_portal_heading(0.0, "L", 0.0), math.pi / 2.0)
        self.assertAlmostEqual(corrected_portal_heading(0.0, "R", 0.0), -math.pi / 2.0)
        self.assertGreater(corrected_portal_heading(0.0, "R", 0.2), -math.pi / 2.0)
        self.assertLess(corrected_portal_heading(0.0, "L", 0.2), math.pi / 2.0)

    def test_new_room_target_preserves_crossing_and_return_states(self):
        self.assertEqual(topology_state_for_new_target(None), "APPROACHING")
        self.assertEqual(
            topology_state_for_new_target("APPROACHING"), "APPROACHING"
        )
        self.assertEqual(
            topology_state_for_new_target("EXPLORING"), "EXPLORING"
        )
        self.assertEqual(
            topology_state_for_new_target("RETURNING"), "RETURNING"
        )

    def test_returning_state_accepts_only_corridor_return(self):
        self.assertTrue(
            target_kind_allowed_for_topology_state(
                "RETURNING", "RETURN_TO_CORRIDOR"
            )
        )
        self.assertFalse(
            target_kind_allowed_for_topology_state("RETURNING", "CAMERA_FRONTIER")
        )
        self.assertFalse(
            target_kind_allowed_for_topology_state("RETURNING", "SPHERE_REVIEW")
        )
        self.assertFalse(target_kind_allowed_for_topology_state("RETURNING", None))
        self.assertTrue(
            target_kind_allowed_for_topology_state("EXPLORING", "CAMERA_FRONTIER")
        )

    def test_return_portal_search_stays_inside_confirmed_width(self):
        self.assertEqual(portal_return_along_offsets(0.40), (0.0,))
        offsets = portal_return_along_offsets(0.80)
        self.assertEqual(offsets, (0.0, -0.1, 0.1, -0.2, 0.2))
        self.assertLessEqual(max(abs(value) for value in offsets), 0.28)
        self.assertLessEqual(
            max(abs(value) for value in portal_return_along_offsets(2.0)), 0.35
        )

    def test_coverage_classification_keeps_sensor_layers_separate(self):
        data = np.zeros((4, 4), dtype=np.int16)
        data[0, 0] = -1
        data[0, 1] = 100
        data[1, 2] = -1
        camera = np.zeros((4, 4), dtype=bool)
        camera[1, 1] = True
        camera[1, 2] = True
        layers = coverage_classification(
            GridView(data, 0.1, 0.0, 0.0),
            np.ones((4, 4), dtype=bool),
            camera,
            robot_radius=0.0,
            safety_margin=0.0,
        )
        self.assertEqual(int(layers[0, 0]), 0)
        self.assertEqual(int(layers[0, 1]), 25)
        self.assertEqual(int(layers[1, 1]), 100)
        self.assertEqual(int(layers[1, 2]), 75)
        self.assertEqual(int(layers[2, 2]), 50)

    def test_weighted_harmonic_does_not_hide_camera_gap(self):
        combined = weighted_harmonic_coverage(0.95, 0.80, 0.65)
        self.assertAlmostEqual(combined, 0.8468, places=4)
        self.assertLess(weighted_harmonic_coverage(1.0, 0.50, 0.65), 0.65)

    def test_snapshot_uses_same_task_denominator_for_both_sensors(self):
        grid = GridView(np.zeros((20, 20), dtype=np.int16), 0.1, 0.0, 0.0)
        task = np.ones((20, 20), dtype=bool)
        camera = np.zeros((20, 20), dtype=bool)
        camera[:16, :] = True
        snapshot, _, _ = coverage_snapshot(
            grid, task, camera, robot_radius=0.0, safety_margin=0.0
        )
        self.assertEqual(snapshot.task_cells, 400)
        self.assertEqual(snapshot.laser, 1.0)
        self.assertEqual(snapshot.camera, 0.8)


class TaskExtentTest(unittest.TestCase):
    def test_incomplete_room_evidence_keeps_conservative_extent(self):
        grid = synthetic_floor(include_second_opening=False)
        extent = infer_task_extent(grid, (0.0, 0.0), 0.0, 35.0, back_extension=3.5)
        self.assertFalse(extent.confident)
        self.assertEqual(extent.forward_limit, 35.0)
        self.assertIsNone(extent.terminal_corridor_start)

    def test_two_room_pairs_and_end_wall_trim_only_terminal_centre(self):
        grid = synthetic_floor()
        extent = infer_task_extent(grid, (0.0, 0.0), 0.0, 35.0, back_extension=3.5)
        self.assertTrue(extent.confident)
        self.assertAlmostEqual(extent.terminal_corridor_start, 17.95, delta=0.8)
        self.assertAlmostEqual(extent.forward_limit, 24.55, delta=0.4)
        mask = task_region_mask(
            grid,
            (0.0, 0.0),
            0.0,
            3.5,
            extent.forward_limit,
            9.5,
            1.1,
            extent.terminal_corridor_start,
        )
        self.assertTrue(mask[grid.world_to_cell(22.0, 0.0)])
        self.assertTrue(mask[grid.world_to_cell(22.0, 5.0)])
        self.assertTrue(mask[grid.world_to_cell(-2.0, 0.0)])
        self.assertFalse(mask[grid.world_to_cell(26.0, 0.0)])

    def test_no_far_end_wall_never_allows_completion_extent(self):
        grid = synthetic_floor(include_end_wall=False)
        extent = infer_task_extent(grid, (0.0, 0.0), 0.0, 35.0, back_extension=3.5)
        self.assertFalse(extent.confident)


class PlannerTest(unittest.TestCase):
    def test_wall_inner_edge_free_space_does_not_merge_real_door_into_wall_length_gap(self):
        resolution = 0.05
        grid = GridView(
            np.full((400, 800), -1, dtype=np.int16),
            resolution,
            -5.0,
            -10.0,
        )

        def box(x0, x1, y0, y1, value):
            row0, column0 = grid.world_to_cell(x0, y0)
            row1, column1 = grid.world_to_cell(x1, y1)
            grid.data[
                min(row0, row1) : max(row0, row1) + 1,
                min(column0, column1) : max(column0, column1) + 1,
            ] = value

        # Match the generated competition geometry: free corridor reaches the
        # configured 1.10 m half-width while the physical wall spans roughly
        # 1.10--1.28 m.  Only the 0.90 m longitudinal wall gap is a door.
        box(-2.0, 30.0, -1.10, 1.10, 0)
        box(-2.0, 30.0, 1.10, 1.28, 100)
        box(-2.0, 30.0, -1.28, -1.10, 100)
        box(-2.0, 30.0, 1.28, 8.0, 0)
        box(-2.0, 30.0, -8.0, -1.28, 0)
        box(7.10, 8.00, 1.05, 1.35, 0)
        box(7.10, 8.00, -1.35, -1.05, 0)

        portals = detect_room_portals(
            grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1
        )
        self.assertEqual({item.side for item in portals}, {"L", "R"})
        self.assertTrue(all(abs(item.along - 7.55) < 0.20 for item in portals))

    def test_room_portals_have_stable_coordinate_ids_and_side_assignment(self):
        grid = synthetic_floor()
        portals = detect_room_portals(grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1)
        self.assertEqual(
            [item.topology_id for item in portals],
            ["ROOM_L_7", "ROOM_L_35", "ROOM_R_7", "ROOM_R_35"],
        )
        self.assertEqual(topology_id_for_point((4.0, 3.5), (0.0, 0.0), 0.0, 1.1, portals), "ROOM_L_7")
        self.assertEqual(topology_id_for_point((4.0, -3.5), (0.0, 0.0), 0.0, 1.1, portals), "ROOM_R_7")
        self.assertEqual(topology_id_for_point((0.0, 0.9), (0.0, 0.0), 0.0, 1.1, portals), "CORRIDOR")

    def test_only_opposing_portals_form_actionable_stations(self):
        grid = synthetic_floor()
        # Add a persistent, doorway-sized left-only wall seam between the two
        # real stations.  It remains diagnostic but must never become a room.
        row0, column0 = grid.world_to_cell(9.5, 1.00)
        row1, column1 = grid.world_to_cell(10.5, 1.20)
        grid.data[
            min(row0, row1) : max(row0, row1) + 1,
            min(column0, column1) : max(column0, column1) + 1,
        ] = 0
        portals = detect_room_portals(grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1)
        stations = pair_room_portals(portals)
        self.assertEqual(len(stations), 2)
        self.assertAlmostEqual(stations[0].along, 3.5, delta=0.2)
        self.assertAlmostEqual(stations[1].along, 17.5, delta=0.2)
        actionable_ids = {
            portal.topology_id
            for station in stations
            for portal in (station.left, station.right)
        }
        self.assertNotIn("ROOM_L_20", actionable_ids)

    def test_topology_lock_filters_candidates_without_changing_global_pool(self):
        grid = synthetic_floor()
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            forward_depth=24.0,
            lateral_half_width=9.5,
        )
        kwargs = dict(
            grid=grid,
            robot_pose=(0.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            camera_target=1.0,
            confirmed_topologies=[item.topology_id for item in detect_room_portals(
                grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1
            )],
        )
        all_targets = planner.plan(**kwargs)
        locked = planner.plan(**kwargs, topology_lock="ROOM_L_7")
        self.assertTrue(all_targets.targets)
        self.assertTrue(locked.targets)
        self.assertTrue(all(item.topology_id == "ROOM_L_7" for item in locked.targets))
        self.assertIn("ROOM_R_7", all_targets.candidate_topologies)
        self.assertEqual(locked.diagnostics["assignment_portal_count"], 1)
        self.assertGreater(locked.diagnostics["room_task_cells"], 0)
        self.assertGreater(locked.diagnostics["room_eligible_cells"], 0)
        self.assertGreater(
            locked.diagnostics["room_reachable_camera_unseen_cells"], 0
        )
        self.assertIn("candidate_reject_counts", locked.diagnostics)

    def test_near_station_is_completed_before_far_pair(self):
        grid = synthetic_floor()
        portals = detect_room_portals(grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1)
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            forward_depth=24.0,
            lateral_half_width=9.5,
            far_room_first=False,
        )
        plan = planner.plan(
            grid,
            robot_pose=(0.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            camera_target=1.0,
            confirmed_topologies=[item.topology_id for item in portals],
        )
        self.assertIsNotNone(plan.target)
        self.assertIn(plan.target.topology_id, ("ROOM_L_7", "ROOM_R_7"))
        self.assertFalse(any(item.topology_id == "ROOM_L_35" for item in plan.targets))
        self.assertFalse(any(item.topology_id == "ROOM_R_35" for item in plan.targets))

        left_complete = planner.plan(
            grid,
            robot_pose=(0.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            camera_target=1.0,
            confirmed_topologies=[item.topology_id for item in portals],
            completed_topologies=("ROOM_L_7",),
        )
        self.assertEqual(left_complete.target.topology_id, "ROOM_R_7")

        near_complete = planner.plan(
            grid,
            robot_pose=(0.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            camera_target=1.0,
            confirmed_topologies=[item.topology_id for item in portals],
            completed_topologies=("ROOM_L_7", "ROOM_R_7"),
        )
        self.assertIsNone(near_complete.target)
        self.assertTrue(near_complete.front_rooms_complete)

        rear_unlocked = planner.plan(
            grid,
            robot_pose=(14.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            camera_target=1.0,
            confirmed_topologies=[item.topology_id for item in portals],
            completed_topologies=("ROOM_L_7", "ROOM_R_7"),
            rear_rooms_unlocked=True,
        )
        self.assertIn(
            rear_unlocked.target.topology_id, ("ROOM_L_35", "ROOM_R_35")
        )

    def test_single_confirmed_door_is_immediately_actionable(self):
        grid = synthetic_floor(include_second_opening=False)
        portals = detect_room_portals(grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1)
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            forward_depth=24.0,
            lateral_half_width=9.5,
            minimum_room_stations=1,
        )
        plan = planner.plan(
            grid,
            robot_pose=(0.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            camera_target=1.0,
            confirmed_topologies=("ROOM_L_7",),
        )
        self.assertTrue(plan.targets)
        self.assertTrue(all(item.topology_id == "ROOM_L_7" for item in plan.targets))
        self.assertEqual(
            [item.topology_id for item in plan.actionable_portals],
            ["ROOM_L_7"],
        )

    def test_confirmed_remembered_portal_survives_sparse_map_dropout(self):
        grid = synthetic_floor(include_second_opening=False)
        remembered = RoomPortal("ROOM_L_7", "L", 3.5, 1.1, 1.0)
        # Close the live left opening while leaving the room and navigable
        # connector represented in the navigation map.  The task planner must
        # retain the confirmed topology owner instead of idling in corridor.
        row0, column0 = grid.world_to_cell(3.0, 1.05)
        row1, column1 = grid.world_to_cell(4.0, 1.15)
        grid.data[
            min(row0, row1) : max(row0, row1) + 1,
            min(column0, column1) : max(column0, column1) + 1,
        ] = 100
        navigation = synthetic_floor(include_second_opening=False)
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            forward_depth=24.0,
            lateral_half_width=9.5,
            minimum_room_stations=1,
        )
        plan = planner.plan(
            grid,
            robot_pose=(0.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            camera_target=1.0,
            navigation_grid=navigation,
            confirmed_topologies=(remembered.topology_id,),
            remembered_portals=(remembered,),
        )
        self.assertEqual(plan.diagnostics["assignment_portal_count"], 1)
        self.assertTrue(plan.targets)
        self.assertTrue(
            all(item.topology_id == remembered.topology_id for item in plan.targets)
        )

    def test_room_lock_never_falls_through_to_another_topology(self):
        grid = synthetic_floor()
        portals = detect_room_portals(grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1)
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            forward_depth=24.0,
            lateral_half_width=9.5,
        )
        plan = planner.plan(
            grid,
            robot_pose=(0.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            camera_target=1.0,
            confirmed_topologies=[item.topology_id for item in portals],
            topology_lock="ROOM_L_7",
        )
        self.assertTrue(plan.targets)
        self.assertTrue(all(item.topology_id == "ROOM_L_7" for item in plan.targets))

    def test_single_confirmed_station_does_not_own_the_far_room(self):
        grid = synthetic_floor()
        portals = detect_room_portals(grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1)
        near_left = next(item for item in portals if item.topology_id == "ROOM_L_7")
        region = topology_region_mask(
            grid,
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            forward_limit=24.5,
            lateral_half_width=9.5,
            corridor_half_width=1.1,
            portals=(near_left,),
            topology_id=near_left.topology_id,
        )
        self.assertTrue(region[grid.world_to_cell(3.5, 4.0)])
        self.assertFalse(region[grid.world_to_cell(17.5, 4.0)])

    def test_door_entry_route_contains_corridor_and_room_staging(self):
        grid = synthetic_floor()
        portals = detect_room_portals(grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1)
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            forward_depth=24.0,
            lateral_half_width=9.5,
        )
        plan = planner.plan(
            grid,
            robot_pose=(0.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            camera_target=1.0,
            confirmed_topologies=[item.topology_id for item in portals],
        )
        self.assertIsNotNone(plan.target)
        self.assertNotEqual(plan.target.topology_id, "CORRIDOR")
        portal = next(
            item for item in portals if item.topology_id == plan.target.topology_id
        )
        # The executable path must visit the corridor centre at the portal and
        # then cross the side wall approximately perpendicular to it.
        self.assertTrue(
            any(
                abs(point[0] - portal.along) < 0.25 and abs(point[1]) < 0.25
                for point in plan.target.path
            )
        )
        self.assertTrue(
            any(
                abs(point[0] - portal.along) < 0.35
                and abs(point[1]) > 1.6
                for point in plan.target.path
            )
        )

    def test_navigation_clearance_is_independent_of_coverage_wall_band(self):
        data = np.zeros((80, 100), dtype=np.int16)
        data[:, 48:51] = 100
        # A 1.2 m opening through a wall.  A 0.42 m circular hard inflation
        # leaves too little robust grid width, while the A1's forward-facing
        # 0.20 m lateral half-footprint crosses with generous clearance.
        data[34:46, 48:51] = 0
        grid = GridView(data, 0.1, -5.0, -4.0)
        planner = TaskCoveragePlanner(
            robot_radius=0.38,
            safety_margin=0.04,
            navigation_clearance=0.20,
            preferred_clearance=0.32,
        )
        path, length, clearance = planner.navigation_path(
            grid,
            robot_pose=(-2.0, 0.0, 0.0),
            target=(2.0, 0.0),
        )
        self.assertTrue(path)
        self.assertGreater(length, 3.8)
        self.assertGreaterEqual(clearance, 0.20)
        self.assertTrue(any(point[0] > 0.5 for point in path))

    def test_astar_shortcuts_unobstructed_area_to_straight_path(self):
        grid = GridView(np.zeros((100, 100), dtype=np.int16), 0.1, -5.0, -5.0)
        planner = TaskCoveragePlanner(navigation_clearance=0.05)
        path, length, _ = planner.navigation_path(
            grid,
            robot_pose=(-3.0, -2.0, math.atan2(3.0, 6.0)),
            target=(3.0, 1.0),
        )
        self.assertTrue(path)
        self.assertAlmostEqual(length, math.hypot(6.0, 3.0), delta=0.20)
        slopes = [
            (point[1] - path[0][1]) / (point[0] - path[0][0])
            for point in path[1:]
            if abs(point[0] - path[0][0]) > 1e-6
        ]
        self.assertTrue(slopes)
        self.assertLess(max(slopes) - min(slopes), 0.03)

    def test_corridor_transport_generates_no_exploration_frontier(self):
        data = np.zeros((60, 120), dtype=np.int16)
        data[:, 85:] = -1
        grid = GridView(data, 0.1, -6.0, -3.0)
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            back_extension=3.0,
            forward_depth=8.0,
        )
        plan = planner.plan(
            grid,
            robot_pose=(0.2, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(data.shape, dtype=bool),
            camera_target=0.0,
            minimum_forward=0.45,
        )
        self.assertFalse(plan.targets)
        self.assertEqual(plan.reason, "NO_FRONTIER")

    def test_virtual_gate_closes_only_the_entrance_slab(self):
        data = np.zeros((40, 40), dtype=np.int16)
        grid = GridView(data, 0.5, -5.0, -5.0)
        mask = task_region_mask(
            grid,
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            back_extension=2.0,
            forward_depth=5.0,
            lateral_half_width=4.0,
            corridor_half_width=1.1,
            gate_half_width=1.1,
            gate_depth=0.30,
        )
        # At the gate plane, a corridor cell is closed while a point outside
        # the corridor-width gate remains valid.  Past the short slab, the
        # corridor cell is valid task space again.
        corridor_at_gate = grid.world_to_cell(0.25, 0.75)
        outside_gate = grid.world_to_cell(0.25, 1.75)
        corridor_forward = grid.world_to_cell(1.25, 0.75)
        self.assertFalse(mask[corridor_at_gate])
        self.assertTrue(mask[outside_gate])
        self.assertTrue(mask[corridor_forward])

    def test_stable_sphere_review_preempts_frontiers(self):
        grid = synthetic_floor()
        portals = detect_room_portals(grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1)
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            forward_depth=24.0,
            back_extension=1.0,
            lateral_half_width=9.5,
        )
        plan = planner.plan(
            grid,
            robot_pose=(0.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            sphere_hypotheses=({"id": "sphere_1", "center": (3.5, 3.0, 0.15)},),
            confirmed_topologies=[item.topology_id for item in portals],
        )
        self.assertIsNotNone(plan.target)
        self.assertEqual(plan.target.kind, "SPHERE_REVIEW")
        self.assertEqual(plan.target.hypothesis_id, "sphere_1")

    def test_path_safety_checks_between_sparse_waypoints(self):
        data = np.zeros((20, 30), dtype=np.int16)
        data[:, 15] = 100
        grid = GridView(data, 0.1, 0.0, 0.0)
        planner = TaskCoveragePlanner(robot_radius=0.0, safety_margin=0.0)
        self.assertFalse(planner.path_is_safe(grid, ((0.5, 1.0), (2.5, 1.0))))

    def test_cached_portal_return_accepts_only_known_narrow_crossing(self):
        data = np.zeros((80, 100), dtype=np.int16)
        grid = GridView(data, 0.10, -5.0, -4.0)
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.20,
            preferred_clearance=0.30,
        )
        # A wall with a 0.3 m mapped opening: below general navigation
        # clearance, but above the verified-return portal clearance.
        wall_column = grid.world_to_cell(0.0, 0.0)[1]
        data[:, wall_column] = 100
        centre_row = grid.world_to_cell(0.0, 0.0)[0]
        data[centre_row, wall_column] = 0
        path, _length, minimum = planner.navigation_path_from_room_through_portal(
            grid,
            robot_pose=(-2.0, 0.0, 0.0),
            room_stage=(-0.4, 0.0),
            corridor_stage=(0.4, 0.0),
            portal_clearance=0.10,
        )
        self.assertTrue(path)
        self.assertLess(minimum, planner.navigation_clearance)
        data[centre_row, wall_column] = 100
        blocked, _length, _minimum = planner.navigation_path_from_room_through_portal(
            grid,
            robot_pose=(-2.0, 0.0, 0.0),
            room_stage=(-0.4, 0.0),
            corridor_stage=(0.4, 0.0),
            portal_clearance=0.10,
        )
        self.assertFalse(blocked)

    def test_cached_portal_return_ignores_only_isolated_endpoint_noise(self):
        data = np.zeros((80, 100), dtype=np.int16)
        grid = GridView(data, 0.10, -5.0, -4.0)
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.20,
            preferred_clearance=0.30,
        )
        noise = grid.world_to_cell(0.0, 0.0)
        data[noise] = 100
        path, _length, _minimum = planner.navigation_path_from_room_through_portal(
            grid,
            robot_pose=(-2.0, 0.0, 0.0),
            room_stage=(-0.4, 0.0),
            corridor_stage=(0.4, 0.0),
            portal_clearance=0.10,
        )
        self.assertTrue(path)
        # Three connected cells are not endpoint speckle and must still block.
        data[noise[0] - 1, noise[1]] = 100
        data[noise[0] + 1, noise[1]] = 100
        blocked, _length, _minimum = planner.navigation_path_from_room_through_portal(
            grid,
            robot_pose=(-2.0, 0.0, 0.0),
            room_stage=(-0.4, 0.0),
            corridor_stage=(0.4, 0.0),
            portal_clearance=0.10,
        )
        self.assertFalse(blocked)

    def test_portal_path_falls_back_from_unknown_fixed_entry_cell(self):
        data = np.zeros((100, 100), dtype=np.int16)
        grid = GridView(data, 0.10, -5.0, -5.0)
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            preferred_clearance=0.10,
        )
        portal = detect_room_portals(
            synthetic_floor(), (0.0, 0.0), 0.0, 35.0
        )[2]
        portal = type(portal)(portal.topology_id, "R", 2.0, -1.1, 1.0)
        # Make the exact former fixed 0.80 m entry cell unknown while the
        # 0.70 m entry and deeper target remain known.
        fixed_entry = planner._portal_waypoint(
            (0.0, 0.0), 0.0, 2.0, -(1.1 + 0.80)
        )
        data[grid.world_to_cell(*fixed_entry)] = -1
        path, _length, _minimum, depth = planner.navigation_path_through_portal(
            grid,
            (0.0, 0.0, 0.0),
            (0.0, 0.0),
            0.0,
            portal,
            (2.0, -3.0),
        )
        self.assertTrue(path)
        self.assertEqual(depth, 0.70)


if __name__ == "__main__":
    unittest.main()
