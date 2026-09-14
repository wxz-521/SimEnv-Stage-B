#!/usr/bin/env python3
"""Drive the elevator state machine through every floor and back down.

``STAGE_B_TRANSITION_ONLY=1`` disables the explorer, so ``floor1_complete``
can never be set by the coverage node and the state machine stops at the first
``FLOOR_1_READY`` instead of walking the chain.  This probe supplies the one
missing input: it publishes ``/simnav/floor_complete`` once per floor, after
that floor's topology has been established, so the mission proceeds through

    0 -> 1 -> 2 -> RIDE_TO_GROUND_FLOOR -> OPEN_MAIN_ENTRANCE -> RETURN_TO_SPAWN

and the descent can be validated in minutes instead of after a multi-hour
exploration.

It changes no robot code: it only publishes a topic the explorer would normally
publish, and it records every state change with wall and sim timestamps.
"""

import argparse
import json
import os
import time

import rospy
from std_msgs.msg import Bool, String

READY_STATES = ("FLOOR_1_READY",)
TERMINAL_FLAGS = ("returned_to_spawn", "two_floor_mission_complete")


class TransitionProbe:
    def __init__(self, out_dir, hold_seconds, timeout_seconds):
        self.out_dir = out_dir
        self.hold_seconds = max(0.5, float(hold_seconds))
        self.timeout_seconds = max(30.0, float(timeout_seconds))
        os.makedirs(self.out_dir, exist_ok=True)
        self.log_path = os.path.join(self.out_dir, "transition_probe.log")
        self.verdict_path = os.path.join(self.out_dir, "transition_probe_verdict.json")
        self.log = open(self.log_path, "a", buffering=1)

        self.started_wall = time.time()
        self.status = {}
        self.timeline = []
        self.last_state = None
        self.armed_floor = None
        self.arm_wall = None
        self.completed_floors = set()
        self.verdict = None

        self.publisher = rospy.Publisher(
            "/simnav/floor_complete", Bool, queue_size=1
        )
        rospy.Subscriber(
            "/simnav/elevator_status", String, self.status_callback, queue_size=5
        )

    def status_callback(self, message):
        try:
            self.status = json.loads(message.data)
        except (TypeError, ValueError):
            return

    def _sim_now(self):
        try:
            return rospy.Time.now().to_sec()
        except Exception:
            return 0.0

    def tick(self):
        data = self.status
        if not data:
            return
        state = str(data.get("state") or "")
        floor = data.get("floor_index")
        sim = self._sim_now()
        if state != self.last_state:
            self.last_state = state
            entry = {
                "wall": time.strftime("%H:%M:%S"),
                "sim": round(sim, 1),
                "state": state,
                "floor_index": floor,
                "completed_floor_indices": data.get("completed_floor_indices"),
                "returned_to_spawn": data.get("returned_to_spawn"),
                "main_entrance_opened": data.get("main_entrance_opened"),
            }
            self.timeline.append(entry)
            self.log.write(json.dumps(entry, sort_keys=True) + "\n")

        if data.get("fault"):
            self._finish("MISSION_FAULT: {}".format(data.get("fault")))
            return
        if data.get("returned_to_spawn") and data.get("two_floor_mission_complete"):
            self._finish("CHAIN_COMPLETE")
            return
        if time.time() - self.started_wall > self.timeout_seconds:
            self._finish("TIMEOUT")
            return

        # Supply the explorer's completion signal once per floor, after that
        # floor's topology is up.  Hold it for a few seconds so a state race
        # cannot swallow the single message.
        if state in READY_STATES:
            key = (floor, state)
            if self.armed_floor != key:
                self.armed_floor = key
                self.arm_wall = time.time()
                self.log.write(
                    "  arming completion for floor {} in state {}\n".format(floor, state)
                )
            elif time.time() - self.arm_wall <= self.hold_seconds:
                self.publisher.publish(Bool(data=True))
            elif floor not in self.completed_floors:
                self.completed_floors.add(floor)
                self.log.write(
                    "  published floor_complete for floor {} ({} published so far)\n".format(
                        floor, sorted(self.completed_floors)
                    )
                )

    def _finish(self, reason):
        if self.verdict is not None:
            return
        data = self.status or {}
        self.verdict = {
            "verdict": reason,
            "wall_elapsed_seconds": round(time.time() - self.started_wall, 1),
            "sim_elapsed_seconds": round(self._sim_now(), 1),
            "floors_signalled_complete": sorted(self.completed_floors),
            "final_state": data.get("state"),
            "floor_index": data.get("floor_index"),
            "completed_floor_indices": data.get("completed_floor_indices"),
            "returned_to_spawn": data.get("returned_to_spawn"),
            "main_entrance_opened": data.get("main_entrance_opened"),
            "two_floor_mission_complete": data.get("two_floor_mission_complete"),
            "fault": data.get("fault"),
            "state_timeline": self.timeline,
        }
        with open(self.verdict_path, "w") as handle:
            json.dump(self.verdict, handle, indent=2, sort_keys=True, default=str)
        self.log.write("VERDICT {} -> {}\n".format(reason, self.verdict_path))
        rospy.signal_shutdown(reason)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--hold-seconds", type=float, default=4.0)
    parser.add_argument("--timeout-seconds", type=float, default=1500.0)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("stage_b_transition_probe", anonymous=True)
    probe = TransitionProbe(args.out_dir, args.hold_seconds, args.timeout_seconds)
    rate = rospy.Rate(5.0)
    while not rospy.is_shutdown():
        try:
            probe.tick()
        except Exception as error:
            probe.log.write("probe error: {}\n".format(error))
        rate.sleep()


if __name__ == "__main__":
    main()
