#!/usr/bin/env python3
"""Evaluate one Stage B floor with the official one-to-one 3D matching rule."""

import argparse
import json
import math
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--truth", required=True)
    parser.add_argument("--detected", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--summary")
    parser.add_argument("--floor-index", type=int, default=0)
    parser.add_argument("--threshold", type=float, default=1.0)
    # Stage B functional closeout requires every truth source to be matched.
    parser.add_argument("--minimum-recall", type=float, default=1.0)
    parser.add_argument("--maximum-false-alarm-rate", type=float, default=0.1)
    return parser.parse_args()


def distance(left, right):
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)))


def main():
    args = parse_args()
    truth_payload = json.loads(Path(args.truth).read_text(encoding="utf-8"))
    detected_payload = json.loads(Path(args.detected).read_text(encoding="utf-8"))
    truth = [
        item["position"]
        for item in truth_payload.get("danger_sources", [])
        if int(item.get("floor_index", -1)) == args.floor_index
        and item.get("is_danger", True)
    ]
    detected = [
        item["position"]
        for item in detected_payload.get("detected_danger_sources", [])
    ]
    pairs = sorted(
        (distance(expected, actual), truth_index, detected_index)
        for truth_index, expected in enumerate(truth)
        for detected_index, actual in enumerate(detected)
        if distance(expected, actual) <= args.threshold
    )
    matched_truth = set()
    matched_detected = set()
    for _distance, truth_index, detected_index in pairs:
        if truth_index in matched_truth or detected_index in matched_detected:
            continue
        matched_truth.add(truth_index)
        matched_detected.add(detected_index)
    correct = len(matched_truth)
    false_alarms = len(detected) - len(matched_detected)
    recall = correct / float(len(truth)) if truth else 1.0
    false_alarm_rate = false_alarms / float(len(detected)) if detected else 0.0
    passed = (
        recall >= args.minimum_recall
        and false_alarm_rate <= args.maximum_false_alarm_rate
    )
    result = {
        "floor_index": args.floor_index,
        "threshold": args.threshold,
        "truth_count": len(truth),
        "detected_count": len(detected),
        "correct": correct,
        "missed": len(truth) - correct,
        "false_alarms": false_alarms,
        "recall": round(recall, 4),
        "false_alarm_rate": round(false_alarm_rate, 4),
        "passed": passed,
    }
    output_path = Path(args.output)
    output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.summary:
        summary_path = Path(args.summary)
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        summary["danger_evaluation"] = result
        summary["passed"] = bool(summary.get("passed", False) and passed)
        summary_path.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(json.dumps(result, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
