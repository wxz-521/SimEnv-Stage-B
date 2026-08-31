#!/usr/bin/env python3
"""Publish separate navigation and task-filtered exploration maps."""

import copy
import math
import threading

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid


def binary_dilate(mask, radius):
    result = np.zeros_like(mask, dtype=bool)
    height, width = mask.shape
    for dy in range(-radius, radius + 1):
        sy0, sy1 = max(0, -dy), min(height, height - dy)
        ty0, ty1 = max(0, dy), min(height, height + dy)
        for dx in range(-radius, radius + 1):
            sx0, sx1 = max(0, -dx), min(width, width - dx)
            tx0, tx1 = max(0, dx), min(width, width + dx)
            result[ty0:ty1, tx0:tx1] |= mask[sy0:sy1, sx0:sx1]
    return result


def binary_erode(mask, radius):
    return ~binary_dilate(~mask, radius)


def close_small_gaps(occupancy, kernel_size):
    if kernel_size <= 1:
        return occupancy.copy()
    occupied = (occupancy >= 50).astype(np.uint8)
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    closed = cv2.morphologyEx(
        occupied,
        cv2.MORPH_CLOSE,
        kernel,
        borderType=cv2.BORDER_CONSTANT,
        borderValue=0,
    ).astype(bool)
    result = occupancy.copy()
    result[closed] = 100
    return result


class MapViewsNode:
    def __init__(self):
        self.frequency = max(0.1, float(rospy.get_param("~mapping_frequency", 2.0)))
        self.kernel = max(1, int(rospy.get_param("~small_gap_kernel", 5)))
        if self.kernel % 2 == 0:
            self.kernel += 1
        self.lock = threading.Lock()
        self.latest_map = None
        self.gate = None
        self.defer_zones = []
        self.navigation_pub = rospy.Publisher("/navigation_map", OccupancyGrid, queue_size=1, latch=True)
        self.exploration_pub = rospy.Publisher("/exploration_map", OccupancyGrid, queue_size=1, latch=True)
        rospy.Subscriber("/map", OccupancyGrid, self._map_callback, queue_size=1)
        rospy.Subscriber("/simnav/entrance_gate", PolygonStamped, self._gate_callback, queue_size=1)
        rospy.Subscriber("/simnav/defer_zone", PolygonStamped, self._defer_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.frequency), self._publish)
        rospy.on_shutdown(self.timer.shutdown)

    def _map_callback(self, message):
        with self.lock:
            self.latest_map = message

    def _gate_callback(self, message):
        with self.lock:
            self.gate = message

    def _defer_callback(self, message):
        with self.lock:
            self.defer_zones.append(message)

    def _publish(self, _event):
        if rospy.is_shutdown():
            return
        with self.lock:
            if self.latest_map is None:
                return
            source = self.latest_map
            gate = self.gate
            zones = list(self.defer_zones)
        try:
            self.navigation_pub.publish(source)
        except rospy.ROSException:
            if rospy.is_shutdown():
                return
            raise
        data = np.asarray(source.data, dtype=np.int16).reshape(source.info.height, source.info.width)
        filtered = close_small_gaps(data, self.kernel)
        if gate is not None:
            self._rasterize_polygon(filtered, source, gate, line_only=True)
        for zone in zones:
            self._rasterize_polygon(filtered, source, zone, line_only=False)
        exploration = copy.deepcopy(source)
        exploration.data = filtered.astype(np.int8).reshape(-1).tolist()
        try:
            self.exploration_pub.publish(exploration)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    @staticmethod
    def _world_to_cell(grid, x, y):
        column = int(math.floor((x - grid.info.origin.position.x) / grid.info.resolution))
        row = int(math.floor((y - grid.info.origin.position.y) / grid.info.resolution))
        return column, row

    def _rasterize_polygon(self, data, grid, polygon, line_only):
        points = [self._world_to_cell(grid, point.x, point.y) for point in polygon.polygon.points]
        if len(points) < 2:
            return
        if line_only or len(points) == 2:
            pairs = list(zip(points, points[1:]))
            if not line_only:
                pairs.append((points[-1], points[0]))
            for start, end in pairs:
                self._draw_line(data, start, end)
            return
        columns = [point[0] for point in points]
        rows = [point[1] for point in points]
        min_column, max_column = max(0, min(columns)), min(data.shape[1] - 1, max(columns))
        min_row, max_row = max(0, min(rows)), min(data.shape[0] - 1, max(rows))
        for row in range(min_row, max_row + 1):
            for column in range(min_column, max_column + 1):
                if self._inside_polygon(column + 0.5, row + 0.5, points):
                    data[row, column] = 100

    @staticmethod
    def _draw_line(data, start, end):
        x0, y0 = start
        x1, y1 = end
        steps = max(abs(x1 - x0), abs(y1 - y0), 1)
        for index in range(steps + 1):
            ratio = index / float(steps)
            x = int(round(x0 + ratio * (x1 - x0)))
            y = int(round(y0 + ratio * (y1 - y0)))
            if 0 <= y < data.shape[0] and 0 <= x < data.shape[1]:
                data[y, x] = 100

    @staticmethod
    def _inside_polygon(x, y, points):
        inside = False
        previous = points[-1]
        for current in points:
            x1, y1 = previous
            x2, y2 = current
            if (y1 > y) != (y2 > y):
                crossing_x = (x2 - x1) * (y - y1) / float(y2 - y1) + x1
                if x < crossing_x:
                    inside = not inside
            previous = current
        return inside


if __name__ == "__main__":
    rospy.init_node("map_views")
    MapViewsNode()
    rospy.spin()
