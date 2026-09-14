#!/usr/bin/env python3

import math
import unittest

import numpy as np

from coverage_explorer_core import (
    FrontierTarget,
    admit_candidate,
    GridView,
    RoomPortal,
    TaskCoveragePlanner,
    coverage_classification,
    detect_lobby_portals,
    coverage_snapshot,
    detect_room_portals,
    infer_task_extent,
    opposite_room_portal,
    pair_room_portals,
    portal_return_along_offsets,
    room_lock_for_target,
    target_switch_allowed,
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
    def test_opposite_room_uses_nearby_observation_or_mirrors_source(self):
        source = RoomPortal("F1_ROOM_R_15", "R", 7.6, -1.1, 0.9)
        observed = RoomPortal("F1_ROOM_L_16", "L", 7.9, 1.1, 1.0)
        self.assertEqual(
            opposite_room_portal(source, (observed,)).topology_id,
            observed.topology_id,
        )
        mirrored = opposite_room_portal(source, ())
        self.assertEqual(mirrored.topology_id, "F1_ROOM_L_15")
        self.assertEqual(mirrored.side, "L")
        self.assertAlmostEqual(mirrored.along, source.along)
        self.assertAlmostEqual(mirrored.lateral, -source.lateral)
        self.assertAlmostEqual(mirrored.width, source.width)

    def test_lobby_portal_detector_uses_separate_prefix(self):
        grid = synthetic_floor()
        portals = detect_lobby_portals(grid, (0.0, 7.0), math.pi / 2.0)
        self.assertTrue(all(item.topology_id.startswith("LOBBY_") for item in portals))

    def test_room_portals_follow_measured_walls_when_gate_axis_is_offset(self):
        grid = synthetic_floor(include_second_opening=False)
        portals = detect_room_portals(
            grid,
            gate_center=(0.0, 0.4),
            forward_yaw=0.0,
            forward_depth=12.0,
            lateral_half_width=9.5,
            corridor_half_width=1.1,
        )
        near = [item for item in portals if abs(item.along - 3.5) < 1.0]
        self.assertEqual({item.side for item in near}, {"L", "R"})
        left = next(item for item in near if item.side == "L")
        right = next(item for item in near if item.side == "R")
        self.assertLess(left.lateral, 1.1)
        self.assertLess(right.lateral, -1.1)

    def test_room_combined_coverage_uses_five_percent_lidar_by_default(self):
        self.assertAlmostEqual(weighted_linear_coverage(1.0, 0.84, 0.90), 0.856)
        self.assertAlmostEqual(weighted_linear_coverage(1.0, 0.70, 0.90), 0.73)
        self.assertAlmostEqual(weighted_linear_coverage(1.0, 0.80), 0.81)

    def test_floor_completion_requires_four_returned_rooms(self):
        rooms = ("ROOM_L_15", "ROOM_R_15", "ROOM_L_29", "ROOM_R_29")
        self.assertFalse(topology_completion_ready(rooms[:3], 4, 0))
        self.assertFalse(topology_completion_ready(rooms, 4, 1))
        self.assertTrue(topology_completion_ready(rooms, 4, 0))

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


class CandidateAdmissionTest(unittest.TestCase):
    """One gate for every candidate.

    The lobby, the finished front room and the corridor-touring regressions were
    three different filters that a newly admitted candidate pool slipped past.
    These tests pin the gate itself so a fourth one cannot leak in silently.
    """

    GATE = (0.0, 0.0)
    YAW = 0.0            # corridor forward is +x

    @staticmethod
    def _target(x, y, topology="ROOM_L_15", kind="CAMERA_FRONTIER"):
        return FrontierTarget(
            kind=kind,
            target=(x, y),
            path=((0.0, 0.0), (x, y)),
            path_length=5.0,
            laser_gain=0.0,
            camera_gain=1.0,
            combined_gain=1.0,
            min_clearance=0.4,
            topology_id=topology,
        )

    def _admit(self, item, **kwargs):
        kwargs.setdefault("active_topologies", ("ROOM_L_15",))
        return admit_candidate(item, self.GATE, self.YAW, (2.0, 0.0), **kwargs)

    def test_a_room_viewpoint_ahead_and_in_zone_is_admitted(self):
        self.assertEqual(self._admit(self._target(6.0, 1.5)), (True, "ADMITTED"))

    def test_the_lobby_is_never_a_task_region(self):
        self.assertEqual(self._admit(self._target(-3.0, 1.5))[1], "LOBBY")

    def test_a_corridor_camera_viewpoint_is_refused(self):
        self.assertEqual(
            self._admit(self._target(6.0, 0.2, topology="CORRIDOR"))[1],
            "CORRIDOR_CAMERA",
        )

    def test_a_point_behind_the_robot_is_refused(self):
        self.assertEqual(self._admit(self._target(0.5, 1.5))[1], "BEHIND")

    def test_a_room_outside_the_active_zone_is_refused(self):
        self.assertEqual(
            self._admit(self._target(6.0, 1.5, topology="ROOM_R_99"))[1], "ZONE"
        )

    def test_a_covered_room_is_refused(self):
        self.assertEqual(
            self._admit(
                self._target(6.0, 1.5),
                room_status={"ROOM_L_15": "COVERED"},
            )[1],
            "ROOM_COVERED",
        )

    def test_a_corridor_laser_frontier_needs_the_active_zone(self):
        item = self._target(6.0, 0.2, topology="CORRIDOR", kind="LASER_FRONTIER")
        self.assertEqual(self._admit(item)[1], "ZONE")
        self.assertEqual(
            self._admit(item, corridor_in_active_zone=True), (True, "ADMITTED")
        )


class RoomLockForTargetTest(unittest.TestCase):
    """Both adoption paths must arm the room lock from the same rule.

    The mid-route handover forgot this, so ``locked_topology`` stayed None and
    the explorer ping-ponged between the two front rooms (run107 floor 0: 13
    handovers in 196 sim seconds, never entering either).
    """

    def test_a_room_target_locks_its_room(self):
        self.assertEqual(room_lock_for_target("ROOM_L_15"), "ROOM_L_15")

    def test_corridor_and_unassigned_targets_never_lock(self):
        self.assertIsNone(room_lock_for_target("CORRIDOR"))
        self.assertIsNone(room_lock_for_target("ROOM_L_15_UNASSIGNED"))
        self.assertIsNone(room_lock_for_target(None))

    def test_a_retired_room_is_not_re_locked(self):
        self.assertIsNone(
            room_lock_for_target("ROOM_L_15", retired={"ROOM_L_15"})
        )
        self.assertEqual(
            room_lock_for_target("ROOM_R_15", retired={"ROOM_L_15"}), "ROOM_R_15"
        )


class TargetSwitchTest(unittest.TestCase):
    """A target is handed over only when it has itself been observed.

    User requirement: switching is triggered by the current viewpoint's own
    remaining information collapsing while walking -- NOT by comparing two
    candidates' gains.  The earlier score-comparison form is what oscillated.
    """

    @staticmethod
    def _target(x, y, path_length=5.0, camera_gain=1.0, topology="ROOM_L_15"):
        return FrontierTarget(
            kind="CAMERA_FRONTIER",
            target=(x, y),
            path=((0.0, 0.0), (x, y)),
            path_length=path_length,
            laser_gain=0.0,
            camera_gain=camera_gain,
            combined_gain=camera_gain,
            min_clearance=0.4,
            topology_id=topology,
        )

    def test_handover_requires_the_active_viewpoint_to_be_observed(self):
        active, candidate = self._target(2.0, 1.0, 5.0, 1.0), self._target(3.0, 1.0, 5.0, 2.0)
        # Still plenty left to see -> keep it.
        self.assertFalse(
            target_switch_allowed(active, candidate, 30.0, active_remaining=1.0)
        )
        self.assertFalse(
            target_switch_allowed(active, candidate, 30.0, active_remaining=0.9)
        )
        # Clearly observed -> the planner may hand over.
        self.assertTrue(
            target_switch_allowed(active, candidate, 30.0, active_remaining=0.1)
        )

    def test_unknown_remaining_information_refuses_the_switch(self):
        self.assertFalse(
            target_switch_allowed(
                self._target(2.0, 1.0), self._target(3.0, 1.0), 30.0,
                active_remaining=None,
            )
        )

    def test_a_longer_detour_is_still_refused_while_unobserved(self):
        # The path guard is gone, but with the target still unobserved there is
        # no reason to take a detour at all.
        self.assertFalse(
            target_switch_allowed(
                self._target(2.0, 1.0, 5.0), self._target(20.0, 1.0, 40.0, 9.0),
                30.0, active_remaining=1.0,
            )
        )

    def test_dwell_blocks_an_immediate_switch(self):
        self.assertFalse(
            target_switch_allowed(
                self._target(2.0, 1.0), self._target(3.0, 1.0), 1.0,
                active_remaining=0.0,
            )
        )

    def test_corridor_candidates_are_not_adopted(self):
        self.assertFalse(
            target_switch_allowed(
                self._target(2.0, 1.0), self._target(3.0, 1.0, 5.0, 9.0, "CORRIDOR"),
                30.0, active_remaining=0.0,
            )
        )

    def test_a_room_lock_is_never_switched_out_of(self):
        self.assertFalse(
            target_switch_allowed(
                self._target(2.0, 1.0), self._target(3.0, 1.0, 5.0, 9.0, "ROOM_R_15"),
                30.0, locked_topology="ROOM_L_15", active_remaining=0.0,
            )
        )

    def test_the_same_point_is_not_a_switch(self):
        self.assertFalse(
            target_switch_allowed(
                self._target(2.0, 1.0), self._target(2.0, 1.0, 5.0, 9.0),
                30.0, active_remaining=0.0,
            )
        )


class CameraLookAtTest(unittest.TestCase):
    """The RGB-D viewpoint heading must be an information-gain bearing.

    measured on run90 floor 1: all 61 door-area viewpoints came back with
    ``look_at == target``, because the centroid of a symmetric unseen region
    lands on the viewpoint itself.  ``atan2(0, 0)`` then collapsed every heading
    to 0 rad -- a fixed world axis -- which is exactly the "sideways at the
    doorway / facing out of the door" orientation the user reported.
    """

    def _planner_and_grid(self):
        grid = synthetic_floor()
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            forward_depth=24.0,
            lateral_half_width=9.5,
        )
        return planner, grid

    @staticmethod
    def _yaw(point, centre):
        return math.atan2(point[1] - centre[1], point[0] - centre[0])

    def test_symmetric_unseen_is_not_degenerate_and_follows_the_preference(self):
        planner, grid = self._planner_and_grid()
        unseen = np.ones(grid.data.shape, dtype=bool)
        centre = (6.0, 1.85)
        cell = grid.world_to_cell(*centre)
        look_at = planner._camera_look_at(
            cell, unseen, grid, preferred_yaw=math.pi / 2.0
        )
        self.assertIsNotNone(look_at)
        # Never the viewpoint itself: that is the degeneracy being fixed.
        self.assertGreater(math.hypot(look_at[0] - centre[0], look_at[1] - centre[1]), 0.1)
        # A fully symmetric unseen region must resolve toward the room interior.
        deviation = abs(
            math.degrees(
                math.atan2(
                    math.sin(self._yaw(look_at, centre) - math.pi / 2.0),
                    math.cos(self._yaw(look_at, centre) - math.pi / 2.0),
                )
            )
        )
        self.assertLess(deviation, 30.0)

    def test_a_real_asymmetry_beats_the_preference(self):
        planner, grid = self._planner_and_grid()
        unseen = np.zeros(grid.data.shape, dtype=bool)
        centre = (6.0, 1.85)
        row, column = grid.world_to_cell(*centre)
        radius_cells = int(math.ceil(planner.information_radius / grid.resolution))
        rows, columns = np.indices(unseen.shape)
        distance = np.hypot(
            (rows - row) * grid.resolution, (columns - column) * grid.resolution
        )
        annulus = (distance >= 1.0) & (distance <= planner.information_radius)
        # A narrow wedge centred on -x, so the heaviest bearing bin is
        # unambiguous and the preference cannot win by accident.
        wedge = np.abs(rows - row) * grid.resolution <= 0.35 * (
            np.abs(columns - column) * grid.resolution
        )
        unseen[annulus & wedge & (columns < column - 1)] = True
        look_at = planner._camera_look_at(
            cell=(row, column), camera_unseen=unseen, grid=grid,
            preferred_yaw=math.pi / 2.0,
        )
        self.assertIsNotNone(look_at)
        yaw = self._yaw(look_at, centre)
        deviation = abs(
            math.degrees(math.atan2(math.sin(yaw - math.pi), math.cos(yaw - math.pi)))
        )
        self.assertLess(deviation, 30.0)


class RobotStartSeedTest(unittest.TestCase):
    """The route must start in the cell the robot is standing in.

    Seeding A* at the nearest cell above the clearance threshold turned the
    first leg into a detour to a cell the robot was not standing in whenever it
    stood near a wall (run107 floor 0: ``path_start_offset`` 1.737 and 2.455 m).
    The robot is physically in its own cell, so that cell is traversable.
    """

    @staticmethod
    def _wall_grid(wall_columns):
        data = np.zeros((60, 120), dtype=np.int16)
        data[:, :wall_columns] = 100
        return GridView(data, 0.1, 0.0, 0.0)

    def test_the_route_starts_in_the_robot_cell_even_below_clearance(self):
        # Cell column 2 (centre x=0.25) is 0.20 m from the wall, below the
        # 0.30 m threshold; the old seed was the nearest safe cell at x=0.35.
        planner = TaskCoveragePlanner(
            robot_radius=0.0, safety_margin=0.0, navigation_clearance=0.30
        )
        path, _length, _clearance = planner.navigation_path(
            self._wall_grid(1), robot_pose=(0.25, 3.0, 0.0), target=(6.0, 3.0)
        )
        self.assertTrue(path)
        self.assertAlmostEqual(path[0][0], 0.25, delta=0.06)
        self.assertAlmostEqual(path[0][1], 3.05, delta=0.06)
        self.assertNotAlmostEqual(path[0][0], 0.35, delta=0.04)

    def test_a_wide_low_clearance_band_is_escaped_from_the_robot_cell(self):
        # Four wall columns put the robot (centre x=0.45) inside a band that is
        # entirely below the threshold.  There is no nearest-safe fallback any
        # more: the 3x3 physical-occupancy window must let the route leave the
        # robot's own cell without moving the start.
        planner = TaskCoveragePlanner(
            robot_radius=0.0, safety_margin=0.0, navigation_clearance=0.30
        )
        path, _length, _clearance = planner.navigation_path(
            self._wall_grid(4), robot_pose=(0.45, 3.0, 0.0), target=(6.0, 3.0)
        )
        self.assertTrue(path)
        self.assertAlmostEqual(path[0][0], 0.45, delta=0.06)


class PlannerTest(unittest.TestCase):
    def test_astar_backwards_walk_breaks_on_cyclic_predecessor(self):
        """A cyclic predecessor chain must not spin for ever.

        The A* search re-opens states, so ``predecessor`` can end up holding
        X -> Y -> X.  The unguarded walk never reached the start state and burned
        a core inside the planning thread; run146 sat still at the ROOM_*_49
        doorway for the rest of the mission because no plan was ever published
        again.  A normal chain must still reconstruct exactly as before.
        """
        planner = TaskCoveragePlanner()
        initial = (1, 1, 0)
        middle = (2, 2, 1)
        goal = (3, 3, 2)
        self.assertEqual(
            planner._reconstruct_path({goal: middle, middle: initial}, initial, goal),
            ((1, 1), (2, 2), (3, 3)),
        )
        self.assertEqual(planner.predecessor_cycle_breaks, 0)
        # Closed cycle: abandoned, counted, and returned as unroutable.
        self.assertEqual(
            planner._reconstruct_path({goal: middle, middle: goal}, initial, goal),
            (),
        )
        self.assertEqual(planner.predecessor_cycle_breaks, 1)
        # A missing predecessor link is the same defect class.
        self.assertEqual(planner._reconstruct_path({}, initial, goal), ())
        self.assertEqual(planner.predecessor_cycle_breaks, 2)

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

    def test_corridor_uses_lidar_and_room_lock_uses_camera(self):
        grid = synthetic_floor()
        row0, column0 = grid.world_to_cell(-3.4, 5.0)
        row1, column1 = grid.world_to_cell(10.5, 9.2)
        grid.data[
            min(row0, row1) : max(row0, row1) + 1,
            min(column0, column1) : max(column0, column1) + 1,
        ] = -1
        planner = TaskCoveragePlanner(
            robot_radius=0.0,
            safety_margin=0.0,
            navigation_clearance=0.05,
            forward_depth=24.0,
            lateral_half_width=9.5,
        )
        kwargs = dict(
            grid=grid,
            robot_pose=(0.5, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            camera_target=1.0,
            minimum_forward=0.45,
            confirmed_topologies=[item.topology_id for item in detect_room_portals(
                grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1
            )],
        )
        all_targets = planner.plan(**kwargs)
        locked = planner.plan(**kwargs, topology_lock="ROOM_L_7")
        self.assertTrue(all_targets.targets)
        self.assertTrue(locked.targets)
        self.assertTrue(all(item.kind == "LASER_FRONTIER" for item in all_targets.targets))
        self.assertEqual(all_targets.target.topology_id, "ROOM_L_7")
        self.assertTrue(all(item.topology_id == "ROOM_L_7" for item in locked.targets))
        self.assertTrue(all(item.kind == "CAMERA_FRONTIER" for item in locked.targets))
        self.assertEqual(locked.diagnostics["assignment_portal_count"], 1)
        self.assertGreater(locked.diagnostics["room_task_cells"], 0)
        self.assertGreater(locked.diagnostics["room_eligible_cells"], 0)
        self.assertGreater(
            locked.diagnostics["room_reachable_camera_unseen_cells"], 0
        )
        self.assertIn("candidate_reject_counts", locked.diagnostics)

    def test_corridor_has_no_hidden_front_rear_station_schedule(self):
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
        self.assertEqual(len(plan.actionable_portals), 4)

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
        self.assertEqual(len(left_complete.actionable_portals), 3)

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
        self.assertEqual(len(near_complete.actionable_portals), 2)
        self.assertTrue(near_complete.front_rooms_complete)

    def test_single_confirmed_door_is_immediately_actionable(self):
        grid = synthetic_floor(include_second_opening=False)
        row0, column0 = grid.world_to_cell(-3.4, 5.0)
        row1, column1 = grid.world_to_cell(10.5, 9.2)
        grid.data[
            min(row0, row1) : max(row0, row1) + 1,
            min(column0, column1) : max(column0, column1) + 1,
        ] = -1
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
            robot_pose=(0.5, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            camera_target=1.0,
            minimum_forward=0.45,
            confirmed_topologies=("ROOM_L_7",),
        )
        self.assertTrue(plan.targets)
        self.assertTrue(all(item.kind == "LASER_FRONTIER" for item in plan.targets))
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
        # Restored to the validated baseline contract: with nothing else left to
        # do the planner keeps returning lidar frontiers here.  The wider camera
        # fallback that replaced this was mine and caused every target-selection
        # regression of this session, so the original contract is pinned again.
        self.assertTrue(all(item.kind == "LASER_FRONTIER" for item in plan.targets))

    def test_completed_physical_door_is_not_redispatched_after_id_drift(self):
        grid = synthetic_floor()
        live_portals = detect_room_portals(
            grid, (0.0, 0.0), 0.0, 35.0, 9.5, 1.1
        )
        live_left = next(
            item for item in live_portals if item.topology_id == "ROOM_L_7"
        )
        completed = RoomPortal(
            "ROOM_L_9", "L", live_left.along + 1.15, live_left.lateral, live_left.width
        )
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
            confirmed_topologies=[item.topology_id for item in live_portals],
            completed_topologies=(completed.topology_id,),
            remembered_portals=(completed,),
        )
        self.assertNotIn(
            live_left.topology_id,
            {item.topology_id for item in plan.actionable_portals},
        )

    def test_locked_remembered_portal_survives_confirmation_dropout(self):
        grid = synthetic_floor(include_second_opening=False)
        remembered = RoomPortal("ROOM_L_7", "L", 3.5, 1.1, 1.0)
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
            topology_lock=remembered.topology_id,
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

    def test_door_entry_route_crosses_without_corridor_staging(self):
        grid = synthetic_floor()
        row0, column0 = grid.world_to_cell(-3.4, 5.0)
        row1, column1 = grid.world_to_cell(10.5, 9.2)
        grid.data[
            min(row0, row1) : max(row0, row1) + 1,
            min(column0, column1) : max(column0, column1) + 1,
        ] = -1
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
            robot_pose=(0.5, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            camera_target=1.0,
            minimum_forward=0.45,
            confirmed_topologies=[item.topology_id for item in portals],
        )
        self.assertIsNotNone(plan.target)
        self.assertNotEqual(plan.target.topology_id, "CORRIDOR")
        portal = next(
            item for item in portals if item.topology_id == plan.target.topology_id
        )
        # The crossing must reach the room side of the doorway, and it must NOT
        # stage on the corridor centreline first: that staging waypoint turned
        # every replan into "enter the room, drive back out to the axis, enter
        # again" because a robot already inside the room was re-routed through
        # the corridor centre before the door.
        room_side = [
            index
            for index, point in enumerate(plan.target.path)
            if abs(point[1]) > 1.6 and abs(point[0] - portal.along) < 0.6
        ]
        self.assertTrue(
            room_side, "the crossing route must reach the room side of the door"
        )
        crossed_at = room_side[0]
        self.assertFalse(
            any(abs(point[1]) < 0.25 for point in plan.target.path[crossed_at:]),
            "the route must not turn back to the corridor centreline after crossing",
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

    def test_corridor_generates_only_lidar_frontiers(self):
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
        self.assertTrue(plan.targets)
        self.assertTrue(all(item.kind == "LASER_FRONTIER" for item in plan.targets))
        self.assertEqual(plan.target.topology_id, "CORRIDOR")

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
            topology_lock="ROOM_L_7",
        )
        self.assertIsNotNone(plan.target)
        self.assertEqual(plan.target.kind, "SPHERE_REVIEW")
        self.assertEqual(plan.target.hypothesis_id, "sphere_1")

    def test_detector_review_guidance_is_graded(self):
        """A colour-detector hint must only outrank frontiers from level 2."""
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
        common = dict(
            grid=grid,
            robot_pose=(0.0, 0.0, 0.0),
            gate_center=(0.0, 0.0),
            forward_yaw=0.0,
            camera_seen=np.zeros(grid.data.shape, dtype=bool),
            sphere_hypotheses=({"id": "DET_0", "center": (3.5, 3.0, 0.15)},),
            confirmed_topologies=[item.topology_id for item in portals],
            topology_lock="ROOM_L_7",
        )
        level1 = planner.plan(danger_guidance_level=1, **common)
        level2 = planner.plan(danger_guidance_level=2, **common)
        # Level 1: the hint is only a fallback, so an ordinary frontier wins.
        self.assertIsNotNone(level1.target)
        self.assertEqual(level1.target.kind, "CAMERA_FRONTIER")
        # Level 2: the detector hint takes the top rank.
        self.assertIsNotNone(level2.target)
        self.assertEqual(level2.target.kind, "SPHERE_REVIEW")
        self.assertEqual(level2.target.hypothesis_id, "DET_0")

    def test_lidar_sphere_review_keeps_validated_priority(self):
        """A lidar hypothesis (non DET_ id) is unaffected by the graded knob."""
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
        for level in (0, 1, 2, 3):
            plan = planner.plan(
                grid,
                robot_pose=(0.0, 0.0, 0.0),
                gate_center=(0.0, 0.0),
                forward_yaw=0.0,
                camera_seen=np.zeros(grid.data.shape, dtype=bool),
                sphere_hypotheses=({"id": "sphere_1", "center": (3.5, 3.0, 0.15)},),
                confirmed_topologies=[item.topology_id for item in portals],
                topology_lock="ROOM_L_7",
                danger_guidance_level=level,
            )
            self.assertIsNotNone(plan.target)
            self.assertEqual(plan.target.kind, "SPHERE_REVIEW")

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
