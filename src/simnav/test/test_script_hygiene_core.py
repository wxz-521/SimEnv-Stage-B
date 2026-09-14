#!/usr/bin/env python3
"""Guard the run44 defect class: a local read before its first assignment.

The coverage explorer's rospy.Timer thread died with an UnboundLocalError
(heading_tolerance read inside the doorway_entry diagnostic before it was
assigned), so cmd_vel stayed (0, 0) for the rest of the mission and the run
looked like a planner stall.  An exception in a timer callback is terminal for
the loop, so this is worth a permanent offline gate.
"""

import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "team_scripts"))

import check_read_before_assign as checker  # noqa: E402
import check_undefined_attrs as attr_checker  # noqa: E402

SCRIPTS = (
    "coverage_explorer_node.py",
    "coverage_explorer_core.py",
    "elevator_transition_node.py",
    "elevator_transition_core.py",
    "danger_detector_node.py",
    "danger_detector_core.py",
)


class ScriptHygieneTest(unittest.TestCase):
    def test_scripts_have_no_read_before_assign(self):
        problems = []
        for name in SCRIPTS:
            path = os.path.join(ROOT, "src", "simnav", "scripts", name)
            problems.extend(checker.scan(path))
        self.assertEqual(problems, [], "read-before-assign local: %r" % problems)

    def test_scripts_have_no_undefined_self_attribute(self):
        # run60's creep used self.stop_distance, an elevator-node parameter; the
        # control guard turned it into 4274 logged faults instead of a silent
        # freeze, but it still burned a full run.
        problems = []
        for name in SCRIPTS:
            path = os.path.join(ROOT, "src", "simnav", "scripts", name)
            problems.extend(attr_checker.scan(path))
        self.assertEqual(problems, [], "undefined self attribute: %r" % problems)

    def test_attr_checker_detects_a_missing_attribute(self):
        source = "class A:\n    def __init__(self):\n        self.real = 1\n\n    def f(self):\n        return self.missing\n"
        with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as handle:
            handle.write(source)
            path = handle.name
        try:
            self.assertEqual(attr_checker.scan(path), [("A", "missing", 6)])
        finally:
            os.unlink(path)

    def test_checker_detects_the_run44_shape(self):
        source = (
            "def f(flag):\n"
            "    if flag:\n"
            "        print(tolerance)\n"
            "    tolerance = 0.10\n"
            "    return tolerance\n"
        )
        with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as handle:
            handle.write(source)
            path = handle.name
        try:
            self.assertEqual(
                checker.scan(path),
                [("f", "tolerance", 3, 4)],
            )
        finally:
            os.unlink(path)

    def test_checker_accepts_a_rebound_parameter(self):
        source = (
            "def f(limit):\n"
            "    if limit is not None:\n"
            "        limit = min(1.0, limit)\n"
            "    return limit\n"
        )
        with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as handle:
            handle.write(source)
            path = handle.name
        try:
            self.assertEqual(checker.scan(path), [])
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()


class PlannerApiTest(unittest.TestCase):
    """Guard against a helper inserted inside the class body.

    A module-level ``def`` placed inside a class silently *ends* the class: every
    later method becomes a nested function of that helper.  The file still
    compiles and imports, but ``TaskCoveragePlanner.plan`` disappears - which is
    exactly what happened while adding doorway speckle removal.
    """

    API = (
        "plan",
        "navigation_path",
        "navigation_path_via",
        "navigation_path_through_portal",
        "navigation_path_from_room_through_portal",
        "path_is_safe",
        "verified_door_band",
        "_navigation_fields",
        "_portal_waypoint",
        "_door_approach_target",
    )

    def test_planner_exposes_its_public_api(self):
        from coverage_explorer_core import TaskCoveragePlanner

        missing = [name for name in self.API if not hasattr(TaskCoveragePlanner, name)]
        self.assertEqual(missing, [], "methods lost from TaskCoveragePlanner: %r" % missing)
