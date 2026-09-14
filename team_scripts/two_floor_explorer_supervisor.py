#!/usr/bin/env python3
"""Restart the frozen floor explorer at the corresponding upper-floor start."""

import json
import os
import subprocess
import threading

import rospy
from std_msgs.msg import Bool, String


class Supervisor:
    def __init__(self):
        self.lock = threading.RLock()
        # Restore the validated Stage B room criterion: the room gate is the
        # 5/95 combined metric (0.05*laser + 0.95*camera) and 0.84 was the
        # lowest threshold that still gave 3/3 red-sphere recall on the frozen
        # seed (SimEnv_two_floor_exploration_codex_spec.md section 17).
        # Red-sphere recall at lower area coverage is protected by the
        # dedicated SPHERE_REVIEW policy, not by lowering this number.
        self.room_coverage_target = float(
            os.environ.get("STAGE_B_ROOM_COMBINED_COVERAGE_TARGET", "0.84")
        )
        self.motion_speed = float(os.environ.get("STAGE_B_MOTION_SPEED", "0.60"))
        # The floor-wide gates stay at the historically validated values
        # (laser 0.95 / camera 0.85 / combined 0.84).  camera_coverage_target is
        # functional: it keeps generating camera-unseen viewpoints until the
        # floor reaches 0.85, which is what pushes the fixed forward camera deep
        # enough into each room to see the red spheres (3/3 floor-0 recall in
        # coverage_speed_sweep_95cam_20260901/speed_060).
        self.camera_coverage_target = float(
            os.environ.get(
                "STAGE_B_CAMERA_COVERAGE_TARGET",
                "0.85",
            )
        )
        self.combined_coverage_target = float(
            os.environ.get("STAGE_B_COMBINED_COVERAGE_TARGET", "0.84")
        )
        self.initial_test_yaw_bias = float(
            os.environ.get("STAGE_B_INITIAL_TEST_YAW_BIAS", "0.0")
        )
        self.initial_test_yaw_bias_distance = float(
            os.environ.get("STAGE_B_INITIAL_TEST_YAW_BIAS_DISTANCE", "0.0")
        )
        self.initial_test_post_entry_turn = float(
            os.environ.get("STAGE_B_INITIAL_TEST_POST_ENTRY_TURN", "0.0")
        )
        self.initial_test_post_entry_turn_progress = float(
            os.environ.get("STAGE_B_INITIAL_TEST_POST_ENTRY_TURN_PROGRESS", "2.0")
        )
        self.process = None
        self.floor_index = 0
        self.completed_floors = set()
        self.last_ready_floor = 0
        self.start_explorer(0)
        rospy.Subscriber("/simnav/floor_complete", Bool, self.complete_callback, queue_size=2)
        rospy.Subscriber("/simnav/elevator_status", String, self.elevator_callback, queue_size=5)
        rospy.on_shutdown(self.stop_explorer)

    def _kill_leftover_explorers(self):
        """Make sure no other coverage_explorer_node.py survives a handover.

        ``stop_explorer`` terminates the ``roslaunch`` that owns the node, and
        roslaunch needs a real shutdown to reap its children.  When that takes
        longer than the 8 s budget it is SIGKILLed, which leaves the
        ``coverage_explorer`` child ORPHANED and still publishing /cmd_vel and
        /simnav/explorer_status - nobody's child any more, so nothing notices.
        run152 ended up with two explorers alive at once (one of them still on
        the previous floor's context) and the floor-2 explorer gone, which reads
        as "the third floor never really boarded".

        The pattern is written so it cannot match this command's own argv.
        """
        try:
            subprocess.run(
                ["pkill", "-9", "-f", "coverage_explorer_node.p[y]"],
                check=False,
            )
        except OSError:
            pass

    def start_explorer(self, floor_index):
        with self.lock:
            if self.process is not None and self.process.poll() is None:
                if int(floor_index) == int(self.floor_index):
                    # Same floor asked for twice (both the floor_complete and the
                    # elevator callback can fire for one transition): keep the
                    # explorer that is already running instead of starting a
                    # second one.
                    return
            self.stop_explorer()
            self._kill_leftover_explorers()
            self._spawn_explorer(floor_index)

    def _spawn_explorer(self, floor_index):
        command = [
            "roslaunch", "simnav", "stage_b_floor_explorer.launch",
            "node_name:=coverage_explorer",
            "room_combined_coverage_target:={:.3f}".format(
                self.room_coverage_target
            ),
            # One source for all three thresholds.  They used to differ (the
            # room target was overridden to 0.55 for tests while the camera and
            # combined targets stayed at the 0.85/0.84 launch defaults), so a
            # room could satisfy "leave the room" yet still fail "room complete"
            # and the planner immediately re-dispatched a viewpoint inside it --
            # the observed "it exits, then plans inside the room again".
            "camera_coverage_target:={:.3f}".format(
                self.room_coverage_target
            ),
            "combined_coverage_target:={:.3f}".format(
                self.room_coverage_target
            ),
            "motion_speed:={:.3f}".format(self.motion_speed),
            "initial_test_yaw_bias:={:.4f}".format(self.initial_test_yaw_bias),
            "initial_test_yaw_bias_distance:={:.2f}".format(
                self.initial_test_yaw_bias_distance
            ),
            "initial_test_post_entry_turn:={:.4f}".format(
                self.initial_test_post_entry_turn
            ),
            "initial_test_post_entry_turn_progress:={:.2f}".format(
                self.initial_test_post_entry_turn_progress
            ),
        ]
        if int(floor_index) > 0:
            # Floors 2/3 only.  The lift node now hands the robot to the explorer
            # standing ON the corridor start (it drives there after leaving the
            # car -- see elevator_transition_node.py ESTABLISH_FLOOR_1_TOPOLOGY),
            # so this fixed forward starts at the corridor start and must end at
            # the same physical node as floor 0's 14.5 m entrance transit.
            #
            # Floor 0's transit is measured from the spawn; its corridor start
            # (the virtual gate, ~virtual_gate_forward_distance = 10.5 m) is
            # 4.0 m before the 14.5 m node.  Hence 4.00 m here, not the old 6.00 m
            # (which was measured from a lift-mouth anchor 1.8 m PAST the gate and
            # therefore landed ~3.4 m long, near the front doorways).
            command.append(
                "initial_forward_distance:={:.2f}".format(
                    float(os.environ.get("STAGE_B_UPPER_FLOOR_FORWARD", "4.00"))
                )
            )
        self.process = subprocess.Popen(command)
        self.floor_index = int(floor_index)
        rospy.loginfo(
            "Started frozen floor explorer for floor %d (roslaunch pid %d)",
            self.floor_index,
            int(self.process.pid),
        )

    def stop_explorer(self):
        process = self.process
        self.process = None
        if process is None or process.poll() is not None:
            self._kill_leftover_explorers()
            return
        process.terminate()
        try:
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)
        # Whatever roslaunch managed to reap, the node must not survive it: an
        # orphaned explorer keeps driving the robot (see _kill_leftover_explorers).
        self._kill_leftover_explorers()

    def complete_callback(self, message):
        if not message.data:
            return
        with self.lock:
            if self.floor_index == 0:
                self.completed_floors.add(0)
            else:
                self.completed_floors.add(self.floor_index)

    def elevator_callback(self, message):
        try:
            payload = json.loads(message.data)
            state = payload.get("state")
        except (TypeError, ValueError):
            return
        with self.lock:
            if (
                0 in self.completed_floors
                and self.floor_index == 0
                and state == "RETURN_TO_ELEVATOR"
            ):
                self.stop_explorer()
            if (
                self.floor_index in self.completed_floors
                and self.floor_index > 0
                and state in ("RETURN_TO_FLOOR_1_GATE", "TOP_FLOOR_COMPLETE")
            ):
                self.stop_explorer()
            try:
                ready_floor = int(payload.get("floor_index", 0))
            except (TypeError, ValueError):
                ready_floor = 0
            if state == "FLOOR_1_READY" and ready_floor > self.last_ready_floor:
                self.last_ready_floor = ready_floor
                self.start_explorer(ready_floor)


def main():
    rospy.init_node("two_floor_explorer_supervisor")
    Supervisor()
    rospy.spin()


if __name__ == "__main__":
    main()
