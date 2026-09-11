#!/usr/bin/env python3
"""Measure how much area the robot actually reached, from the telemetry log.

Coverage percentage counts static cells and can be traded down; missing a red
sphere is decided by whether the robot ever got close enough to look.  This
tool answers that second question from the recorded 10 Hz pose stream: every
pose is stamped with a camera-reach disk, and the union of those disks is the
area the sensors could actually see.

Usage:
  analyse_danger_reach.py <telemetry.csv> [--reach 2.5] [--cell 0.25]
"""

import argparse
import csv
import math

import numpy as np


def load_poses(path):
    xs, ys = [], []
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                x = float(row["src_x"])
                y = float(row["src_y"])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                xs.append(x)
                ys.append(y)
    return np.asarray(xs), np.asarray(ys)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("telemetry")
    parser.add_argument("--reach", type=float, default=2.5,
                        help="sensor reach around each pose in metres")
    parser.add_argument("--cell", type=float, default=0.25,
                        help="raster resolution for the swept-area union")
    args = parser.parse_args()

    xs, ys = load_poses(args.telemetry)
    if xs.size == 0:
        print("no poses found")
        return
    origin_x = float(xs.min() - args.reach - args.cell)
    origin_y = float(ys.min() - args.reach - args.cell)
    width = int(math.ceil((xs.max() - origin_x + args.reach) / args.cell)) + 1
    height = int(math.ceil((ys.max() - origin_y + args.reach) / args.cell)) + 1
    grid = np.zeros((height, width), dtype=bool)
    radius = int(math.ceil(args.reach / args.cell))
    offsets = [
        (dr, dc)
        for dr in range(-radius, radius + 1)
        for dc in range(-radius, radius + 1)
        if (dr * dr + dc * dc) <= radius * radius
    ]
    columns = ((xs - origin_x) / args.cell).astype(np.int64)
    rows = ((ys - origin_y) / args.cell).astype(np.int64)
    for row, column in zip(rows, columns):
        for dr, dc in offsets:
            r, c = row + dr, column + dc
            if 0 <= r < height and 0 <= c < width:
                grid[r, c] = True

    swept = float(np.count_nonzero(grid)) * args.cell * args.cell
    path = float(np.sum(np.hypot(np.diff(xs), np.diff(ys))))
    span_x = float(xs.max() - xs.min())
    span_y = float(ys.max() - ys.min())
    print("poses                : {}".format(xs.size))
    print("path length          : {:.1f} m".format(path))
    print("pose extent          : {:.1f} m x {:.1f} m".format(span_x, span_y))
    print("sensor reach         : {:.2f} m".format(args.reach))
    print("swept area (union)   : {:.1f} m^2".format(swept))
    print("swept bounding box   : {:.1f} m^2".format(
        (span_x + 2 * args.reach) * (span_y + 2 * args.reach)))
    if swept > 0:
        print("implied uniform fill : {:.3f}".format(
            swept / ((span_x + 2 * args.reach) * (span_y + 2 * args.reach))))


if __name__ == "__main__":
    main()
