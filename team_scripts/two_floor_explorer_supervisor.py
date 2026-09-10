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
        # Match run_stage_b_seed.sh: the room metric is near-saturated at 0.84
        # while the separate combined gate sits at the validated 0.40.  A 0.70
        # room default here silently started multi-floor runs at a different
        # threshold than the single-floor baseline whenever the caller did not
        # export the environment explicitly.
        self.room_coverage_target = float(
            os.environ.get("STAGE_B_ROOM_COMBINED_COVERAGE_TARGET", "0.84")
        )
        self.motion_speed = float(os.environ.get("STAGE_B_MOTION_SPEED", "0.60"))
        self.camera_coverage_target = float(
            os.environ.get(
                "STAGE_B_CAMERA_COVERAGE_TARGET",
                "0.40",
            )
        )
        self.combined_coverage_target = float(
            os.environ.get("STAGE_B_COMBINED_COVERAGE_TARGET", "0.40")
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

    def start_explorer(self, floor_index):
        self.stop_explorer()
        command = [
            "roslaunch", "simnav", "stage_b_floor_explorer.launch",
            "node_name:=coverage_explorer",
            "room_combined_coverage_target:={:.3f}".format(
                self.room_coverage_target
            ),
            "camera_coverage_target:={:.3f}".format(
                self.camera_coverage_target
            ),
            "combined_coverage_target:={:.3f}".format(
                self.combined_coverage_target
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
            # The transition node has already used A* to place the robot just
            # inside the mapped corridor.  Skip only the outdoor/lobby transit;
            # all doorway and room exploration behavior remains identical.
            command.append("initial_forward_distance:=0.0")
        self.process = subprocess.Popen(command)
        self.floor_index = int(floor_index)
        rospy.loginfo("Started frozen floor explorer for floor %d", self.floor_index)

    def stop_explorer(self):
        process = self.process
        self.process = None
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)

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
