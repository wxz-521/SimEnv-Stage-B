#!/usr/bin/env python3
"""Lightweight RGB-D red-sphere detection and spatial track confirmation."""

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class BallObservation:
    pixel: Tuple[float, float]
    radius_pixels: float
    circularity: float
    position_camera: Tuple[float, float, float]


@dataclass
class DangerTrack:
    track_id: int
    position_world: np.ndarray
    observations: int
    confidence: float
    last_seen: float
    m2_world: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    localization_health_counts: dict = field(default_factory=dict)

    @property
    def position_variance(self):
        if self.observations < 2:
            return np.zeros(3, dtype=np.float64)
        return self.m2_world / float(self.observations - 1)


class RedBallDetector:
    def __init__(self, min_area=80.0, min_circularity=0.82, min_radius=5.0, max_radius=180.0):
        self.min_area = min_area
        self.min_circularity = min_circularity
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.last_stats = {
            "red_mask_found": 0,
            "contour_pass": 0,
            "depth_pass": 0,
            "observations": 0,
        }

    def detect(self, image_bgr, depth_meters, intrinsics):
        self.last_stats = {
            "red_mask_found": 0,
            "contour_pass": 0,
            "depth_pass": 0,
            "observations": 0,
        }
        if image_bgr is None or depth_meters is None or image_bgr.shape[:2] != depth_meters.shape[:2]:
            return []
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        lower_red = cv2.inRange(hsv, np.array([0, 90, 60]), np.array([10, 255, 255]))
        upper_red = cv2.inRange(hsv, np.array([170, 90, 60]), np.array([179, 255, 255]))
        mask = cv2.bitwise_or(lower_red, upper_red)
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        self.last_stats["red_mask_found"] = len(contours)
        observations = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            perimeter = float(cv2.arcLength(contour, True))
            if area < self.min_area or perimeter <= 1e-6:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            _, _, width, height = cv2.boundingRect(contour)
            aspect = width / float(max(height, 1))
            extent = area / float(max(width * height, 1))
            (center_x, center_y), radius = cv2.minEnclosingCircle(contour)
            if circularity < self.min_circularity or not 0.78 <= aspect <= 1.28:
                continue
            if extent > 0.88 or not self.min_radius <= radius <= self.max_radius:
                continue
            self.last_stats["contour_pass"] += 1
            depth = self._center_depth(depth_meters, center_x, center_y, radius)
            if depth is None:
                continue
            self.last_stats["depth_pass"] += 1
            camera_x = (center_x - intrinsics.cx) * depth / intrinsics.fx
            camera_y = (center_y - intrinsics.cy) * depth / intrinsics.fy
            observations.append(
                BallObservation(
                    pixel=(float(center_x), float(center_y)),
                    radius_pixels=float(radius),
                    circularity=float(circularity),
                    position_camera=(float(camera_x), float(camera_y), float(depth)),
                )
            )
        self.last_stats["observations"] = len(observations)
        return observations

    @staticmethod
    def _center_depth(depth, center_x, center_y, radius):
        height, width = depth.shape
        radius_inner = max(2, int(round(radius * 0.35)))
        x0 = max(0, int(round(center_x)) - radius_inner)
        x1 = min(width, int(round(center_x)) + radius_inner + 1)
        y0 = max(0, int(round(center_y)) - radius_inner)
        y1 = min(height, int(round(center_y)) + radius_inner + 1)
        values = depth[y0:y1, x0:x1]
        with np.errstate(invalid="ignore"):
            valid_mask = np.isfinite(values) & (values > 0.15) & (values < 12.0)
        valid = values[valid_mask]
        if valid.size < 6:
            return None
        return float(np.median(valid))

def rasterize_planar_ray(origin_xy, endpoint_xy, resolution):
    """Return horizontal grid cells crossed by one camera observation ray."""
    cell_size = max(float(resolution), 1e-6)
    origin = (
        int(math.floor(float(origin_xy[0]) / cell_size)),
        int(math.floor(float(origin_xy[1]) / cell_size)),
    )
    endpoint = (
        int(math.floor(float(endpoint_xy[0]) / cell_size)),
        int(math.floor(float(endpoint_xy[1]) / cell_size)),
    )
    delta_x = endpoint[0] - origin[0]
    delta_y = endpoint[1] - origin[1]
    steps = max(abs(delta_x), abs(delta_y))
    if steps == 0:
        return (origin,)
    cells = []
    for index in range(steps + 1):
        fraction = float(index) / float(steps)
        cell = (
            int(round(origin[0] + fraction * delta_x)),
            int(round(origin[1] + fraction * delta_y)),
        )
        if not cells or cells[-1] != cell:
            cells.append(cell)
    return tuple(cells)


class DangerTracker:
    def __init__(self, confirmation_frames=3, cluster_radius=0.75):
        self.confirmation_frames = confirmation_frames
        self.cluster_radius = cluster_radius
        self.tracks = []
        self.next_id = 0

    def update(self, positions_world, sim_time, localization_health="GOOD"):
        matched_track_ids = set()
        for values in positions_world:
            position = np.asarray(values, dtype=np.float64)
            track = self._nearest(position, excluded_track_ids=matched_track_ids)
            if track is None:
                track = DangerTrack(
                    track_id=self.next_id,
                    position_world=position.copy(),
                    observations=1,
                    confidence=1.0 / self.confirmation_frames,
                    last_seen=sim_time,
                    localization_health_counts={localization_health: 1},
                )
                self.tracks.append(track)
                self.next_id += 1
            else:
                previous_mean = track.position_world.copy()
                count = track.observations + 1
                track.position_world = previous_mean + (position - previous_mean) / count
                track.m2_world += (position - previous_mean) * (position - track.position_world)
                track.observations = count
                track.confidence = min(1.0, count / float(self.confirmation_frames))
                track.last_seen = sim_time
                track.localization_health_counts[localization_health] = (
                    track.localization_health_counts.get(localization_health, 0) + 1
                )
            matched_track_ids.add(track.track_id)
        return self.confirmed()

    def _nearest(self, position, excluded_track_ids=None):
        if not self.tracks:
            return None
        excluded = set(excluded_track_ids or [])
        candidates = [item for item in self.tracks if item.track_id not in excluded]
        if not candidates:
            return None
        distances = [
            float(np.linalg.norm(item.position_world - position)) for item in candidates
        ]
        index = int(np.argmin(distances))
        return candidates[index] if distances[index] <= self.cluster_radius else None

    def confirmed(self):
        return [item for item in self.tracks if item.observations >= self.confirmation_frames]

    def translate(self, delta_world):
        """Keep existing tracks in the corrected world frame after loop closure."""
        delta = np.asarray(delta_world, dtype=np.float64)
        for track in self.tracks:
            track.position_world += delta

    def transform_except(self, transform, preserved_track_ids):
        """Correct tracks created after a reliable local observation anchor."""
        matrix = np.asarray(transform, dtype=np.float64).reshape((3, 3))
        preserved = {int(track_id) for track_id in preserved_track_ids}
        for track in self.tracks:
            if track.track_id in preserved:
                continue
            planar = np.asarray(
                [track.position_world[0], track.position_world[1], 1.0]
            )
            corrected = np.matmul(matrix, planar)
            track.position_world[0] = corrected[0]
            track.position_world[1] = corrected[1]

    def merge_nearby_tracks(self, anchor_track_ids, max_distance):
        """Merge short-lived post-loop duplicates into protected tracks."""
        anchor_ids = {int(value) for value in anchor_track_ids}
        anchors = {
            track.track_id: track
            for track in self.tracks
            if track.track_id in anchor_ids
        }
        if not anchors:
            return
        retained = []
        for track in self.tracks:
            if track.track_id in anchors:
                retained.append(track)
                continue
            nearest = min(
                anchors.values(),
                key=lambda anchor: float(
                    np.linalg.norm(anchor.position_world - track.position_world)
                ),
            )
            distance = float(
                np.linalg.norm(nearest.position_world - track.position_world)
            )
            if distance > float(max_distance):
                retained.append(track)
                continue
            total = nearest.observations + track.observations
            nearest.position_world = (
                nearest.position_world * nearest.observations
                + track.position_world * track.observations
            ) / float(total)
            nearest.observations = total
            nearest.confidence = min(1.0, total / float(self.confirmation_frames))
            nearest.last_seen = max(nearest.last_seen, track.last_seen)
            for state, count in track.localization_health_counts.items():
                nearest.localization_health_counts[state] = (
                    nearest.localization_health_counts.get(state, 0) + count
                )
        self.tracks = retained


def position_on_floor(position_world, floor_z=0.0, minimum_offset=-0.2, maximum_offset=1.2):
    return minimum_offset <= float(position_world[2]) - floor_z <= maximum_offset


def project_pose_from_anchor(source_pose, source_anchor, metric_anchor):
    """Express the current source pose in the metric frame at room entry.

    The source odometry supplies only short-term relative motion.  The room
    entry reference supplies the globally meaningful position and heading;
    this affects detection coordinates without changing navigation pose.
    """
    source_x, source_y, source_yaw = (float(value) for value in source_pose[:3])
    anchor_source_x, anchor_source_y, anchor_source_yaw = (
        float(value) for value in source_anchor[:3]
    )
    anchor_metric_x, anchor_metric_y, anchor_metric_yaw = (
        float(value) for value in metric_anchor[:3]
    )
    dx = source_x - anchor_source_x
    dy = source_y - anchor_source_y
    cosine = math.cos(anchor_source_yaw)
    sine = math.sin(anchor_source_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    metric_cosine = math.cos(anchor_metric_yaw)
    metric_sine = math.sin(anchor_metric_yaw)
    projected_x = anchor_metric_x + metric_cosine * local_x - metric_sine * local_y
    projected_y = anchor_metric_y + metric_sine * local_x + metric_cosine * local_y
    yaw_delta = math.atan2(
        math.sin(source_yaw - anchor_source_yaw),
        math.cos(source_yaw - anchor_source_yaw),
    )
    projected_yaw = math.atan2(
        math.sin(anchor_metric_yaw + yaw_delta),
        math.cos(anchor_metric_yaw + yaw_delta),
    )
    return projected_x, projected_y, projected_yaw


class ResultWriter:
    """Atomically write separate debug and official evaluator payloads."""

    def __init__(self, formal_path, debug_path):
        self.formal_path = Path(formal_path)
        self.debug_path = Path(debug_path)

    def write(
        self,
        all_tracks,
        confirmed_tracks,
        exploration_time,
        frame_id="world",
        floor_complete=False,
    ):
        debug_payload = {
            "frame_id": frame_id,
            "floor_complete": bool(floor_complete),
            "exploration_time": max(0.0, float(exploration_time)),
            "dangers": [self._debug_track(track) for track in all_tracks],
        }
        formal_payload = {
            "exploration_time": max(0.0, float(exploration_time)),
            "detected_danger_sources": [
                {
                    "position": [
                        round(float(value), 4) for value in track.position_world
                    ]
                }
                for track in confirmed_tracks
            ],
        }
        self._atomic_write(self.debug_path, debug_payload)
        self._atomic_write(self.formal_path, formal_payload)
        return formal_payload

    @staticmethod
    def _debug_track(track):
        return {
            "id": track.track_id,
            "position_world": [round(float(value), 4) for value in track.position_world],
            "position_variance": [
                round(float(value), 6) for value in track.position_variance
            ],
            "observations": track.observations,
            "confidence": round(float(track.confidence), 4),
            "last_seen": float(track.last_seen),
            "localization_health_counts": dict(track.localization_health_counts),
        }

    @staticmethod
    def _atomic_write(path, payload):
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
