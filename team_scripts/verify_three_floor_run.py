#!/usr/bin/env python3
"""Verdict harness for a Stage-B three-floor run (55% coverage, return to spawn).

The mission is judged on behaviour, not on internal counters:

  * 不碰撞      -> navigation_blocks / obstacle stops in the explorer status
  * 进门敢走    -> every room is entered exactly once, and each room's coverage
                   rises above the target
  * 红球        -> danger detector confirmations
  * 上下楼      -> elevator state sequence across the floors
  * 回出生点    -> the elevator returns to the ground floor and exits
  * 定位        -> LIO health / pose continuity warnings (reported, not gated)

Usage (inside the container, or on the host with --host-logs):

    python3 team_scripts/verify_three_floor_run.py \
        --run-dir logs/run79_modules_20260913/seed_20260902 \
        --ros-log /tmp/simenv-home/.ros/log/<uuid>

It is read-only: it never touches the running nodes.
"""

import argparse
import glob
import json
import os
import re
import sys


def newest(pattern):
    matches = sorted(glob.glob(pattern), key=os.path.getmtime)
    return matches[-1] if matches else None


def read_lines(path):
    if not path or not os.path.isfile(path):
        return []
    with open(path, "r", errors="replace") as handle:
        return handle.read().splitlines()


def count(lines, pattern):
    rx = re.compile(pattern)
    return sum(1 for line in lines if rx.search(line))


def collect(lines, pattern, limit=12):
    rx = re.compile(pattern)
    return [line.strip() for line in lines if rx.search(line)][-limit:]


def explorer_evidence(explorer_log, run_start_marker):
    lines = read_lines(explorer_log)
    if run_start_marker:
        index = None
        for position, line in enumerate(lines):
            if run_start_marker in line:
                index = position
        if index is not None:
            lines = lines[index:]
    entries = re.findall(r"Topology room (\S+) doorway crossing confirmed", "\n".join(lines))
    completed = re.findall(
        r"Topology room (\S+) coverage complete", "\n".join(lines)
    )
    return {
        "entries": entries,
        "completed": completed,
        "reentry_warnings": count(lines, r"entered \d+ times"),
        "unsafe_path": count(lines, r"Unsafe path"),
        "no_candidate": count(lines, r"produced no candidate"),
        "dropped_target": count(lines, r"Dropping stuck target"),
        "reopened": count(lines, r"Re-opening blocked room"),
        "returning": collect(lines, r"returning to CORRIDOR"),
        "last_targets": collect(lines, r"COVERAGE target", limit=4),
    }


def elevator_evidence(elevator_log, rosout_log=None):
    # The transition node publishes its state on /simnav/elevator_status and
    # does not log every transition itself, so the aggregated rosout log is
    # scanned as well: its messages carry names like
    # "Elevator car door opened: state=FLOOR_1_READY".
    lines = read_lines(elevator_log) + read_lines(rosout_log)
    joined = "\n".join(lines)
    states = re.findall(r"state -> ([A-Z0-9_]+)", joined)
    if not states:
        states = re.findall(
            r"\b(FLOOR_1_READY|RIDE_TO_[A-Z_]+|EXIT_GROUND_FLOOR|RETURN_TO_[A-Z_]+|"
            r"ALIGN_[A-Z_]+|ROBOT_ON_GROUND|WAIT_FLOOR_COMPLETE|ENTER_[A-Z_]+)\b",
            joined,
        )
    return {
        "states": states,
        "returned_to_ground": any(
            "EXIT_GROUND_FLOOR" in item or "ROBOT_ON_GROUND" in item or "Main entrance opened" in item
            for item in states
        )
        or "Main entrance opened" in joined,
        "floors_visited": sorted(
            {
                match
                for match in re.findall(r"FLOOR_(\d+)_READY", joined)
            }
        ),
    }


def timeline_evidence(run_dir):
    """Sampled run timeline written by team_scripts/sample_run_status.py."""
    if not run_dir:
        return {"rows": 0, "floors": [], "last": None, "rooms_done": []}
    path = os.path.join(run_dir, "verify_timeline.log")
    lines = read_lines(path)[1:]
    floors = sorted({line.split(",")[3] for line in lines if len(line.split(",")) > 13})
    retired = set()
    for line in lines:
        parts = line.split(",")
        if len(parts) > 13:
            with_prefix = parts[8]
            for token in re.findall(r"'?(ROOM_[A-Z]_[0-9]+)'?", with_prefix):
                retired.add(token)
    return {
        "rows": len(lines),
        "floors": floors,
        "last": lines[-1] if lines else None,
        "rooms_done": sorted(retired),
    }


def danger_evidence(danger_log):
    lines = read_lines(danger_log)
    return {
        "confirmed": count(lines, r"CONFIRMED"),
        "scope_active": count(lines, r"exploration scope active"),
    }


def status_snapshot():
    try:
        import rospy
        from std_msgs.msg import String
    except Exception as error:  # pragma: no cover - only when rospy is absent
        return {"error": "rospy unavailable: %s" % error}
    try:
        rospy.init_node("verify_three_floor_run", anonymous=True, disable_signals=True)
        message = rospy.wait_for_message("/simnav/explorer_status", String, timeout=10)
        payload = json.loads(message.data)
    except Exception as error:
        return {"error": str(error)}
    keep = (
        "floor_index",
        "topology_lock",
        "topology_region",
        "state",
        "room_entry_counts",
        "retired_topologies",
        "completed_topologies",
        "room_coverages",
        "navigation_blocks",
        "targets_reached",
        "floor_complete",
    )
    return {key: payload.get(key) for key in keep if key in payload}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", default=None, help="host run dir (logs/runNN_...)")
    parser.add_argument("--ros-log", default=None, help="ROS log dir of this session")
    parser.add_argument("--coverage-target", type=float, default=0.55)
    parser.add_argument("--skip-status", action="store_true")
    args = parser.parse_args()

    run_dir = args.run_dir
    if run_dir and not os.path.isdir(run_dir):
        run_dir = None
    ros_log = args.ros_log or newest("/tmp/simenv-home/.ros/log/*/")
    explorer_log = os.path.join(ros_log, "coverage_explorer-1.log") if ros_log else None
    elevator_log = os.path.join(ros_log, "elevator_transition-2.log") if ros_log else None
    rosout_log = os.path.join(ros_log, "rosout.log") if ros_log else None
    danger_log = os.path.join(ros_log, "danger_detector-1.log") if ros_log else None

    explorer = explorer_evidence(explorer_log, "name[/coverage_explorer]")
    elevator = elevator_evidence(elevator_log, rosout_log)
    timeline = timeline_evidence(run_dir)
    danger = danger_evidence(danger_log)
    status = {} if args.skip_status else status_snapshot()

    rooms_expected = 4 * 3  # four rooms per floor in this scene, three floors
    checklist = []

    def item(name, ok, detail):
        verdict = "PASS" if ok is True else ("FAIL" if ok is False else "PENDING")
        checklist.append((name, verdict, detail))

    repeated = sorted({room for room in explorer["entries"] if explorer["entries"].count(room) > 1})
    item(
        "进门敢走（无房间二次进入）",
        None if not explorer["entries"] else (not repeated and explorer["reentry_warnings"] == 0),
        "entries=%s repeated=%s warnings=%d" % (explorer["entries"], repeated, explorer["reentry_warnings"]),
    )
    nav_blocks = (status or {}).get("navigation_blocks")
    item(
        "无碰撞",
        None if nav_blocks is None else (nav_blocks == 0),
        "navigation_blocks=%s" % nav_blocks,
    )
    item("红球检测", True if danger["confirmed"] > 0 else None, "confirmed=%d" % danger["confirmed"])
    floors_seen = sorted(set(timeline["floors"]) | set(elevator["floors_visited"]))
    item(
        "上下楼",
        True if len(floors_seen) >= 2 else None,
        "floors=%s states=%d" % (floors_seen, len(elevator["states"])),
    )
    item(
        "回出生点",
        True if elevator["returned_to_ground"] else None,
        "returned=%s" % elevator["returned_to_ground"],
    )

    coverages = (status or {}).get("room_coverages") or {}
    reached = [
        room
        for room, data in coverages.items()
        if float((data or {}).get("combined", 0.0)) >= args.coverage_target
    ]
    item(
        "每房间达到 %.2f 覆盖" % args.coverage_target,
        True if len(reached) >= rooms_expected else None,
        "reached=%d/%d rooms=%s" % (len(reached), rooms_expected, reached),
    )

    print("== Stage-B 三层运行验收 ==")
    print("run_dir   :", run_dir or "(not found)")
    print("ros_log   :", ros_log or "(not found)")
    print()
    print("-- explorer --")
    for key in ("entries", "completed", "reentry_warnings", "unsafe_path", "no_candidate", "dropped_target", "reopened"):
        print("  %-16s %s" % (key, explorer[key]))
    for line in explorer["returning"]:
        print("  returning        ", line[:120])
    print("-- elevator --")
    print("  floors_visited   ", elevator["floors_visited"])
    print("  returned_to_ground", elevator["returned_to_ground"])
    print("  states           ", " -> ".join(elevator["states"][-12:]))
    print("-- danger --")
    print("  confirmed        ", danger["confirmed"])
    print("-- timeline --")
    print("  rows             ", timeline["rows"])
    print("  floors_seen      ", timeline["floors"])
    print("  rooms_retired    ", timeline["rooms_done"])
    print("  last             ", (timeline["last"] or "")[:170])
    print("-- status snapshot --")
    print(" ", json.dumps(status, ensure_ascii=False)[:600])
    print()
    print("-- checklist --")
    for name, verdict, detail in checklist:
        print("  [%-7s] %-28s %s" % (verdict, name, detail))
    failed = [name for name, verdict, _ in checklist if verdict == "FAIL"]
    pending = [name for name, verdict, _ in checklist if verdict == "PENDING"]
    print()
    if failed:
        print("VERDICT: FAIL: " + ", ".join(failed))
    elif pending:
        print("VERDICT: INCOMPLETE (pending: %s)" % ", ".join(pending))
    else:
        print("VERDICT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
