#!/usr/bin/env python3
"""Build a lightweight 2D occupancy map from FAST-LIO registered scans."""

import json
import threading

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf.transformations as transformations
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


from map_floors_core import (
    ensure_floor_grid,
    floor_map_report,
    height_band_is_sane,
    observed_floor_level,
    point_floor_mask,
)


def trace_ray_free(grid, start, end):
    """Mark unknown cells along a sensor ray as free without erasing walls."""
    ray = np.zeros_like(grid, dtype=np.uint8)
    cv2.line(ray, start, end, 1, 1)
    free_cells = (ray != 0) & (grid < 0)
    grid[free_cells] = 0


class LioOccupancyNode:
    def __init__(self):
        self.resolution = float(rospy.get_param("~resolution", 0.10))
        self.width = int(rospy.get_param("~width", 800))
        self.height = int(rospy.get_param("~height", 600))
        self.origin_x = float(rospy.get_param("~origin_x", -30.0))
        self.origin_y = float(rospy.get_param("~origin_y", -15.0))
        self.min_height = float(rospy.get_param("~min_height", -0.10))
        self.max_height = float(rospy.get_param("~max_height", 1.30))
        self.point_stride = max(1, int(rospy.get_param("~point_stride", 4)))
        self.frame_id = rospy.get_param("~frame_id", "simnav_map")
        self.lock = threading.Lock()
        # One 2D grid per floor, separated by height, all retained.  The old
        # node kept a single grid and cleared it on every floor change, so a
        # floor's map (and therefore its topology) only existed while the robot
        # was standing on it.
        self.grids = {}
        self.active_floor = 0
        self.grid, _created = ensure_floor_grid(
            self.grids, 0, (self.height, self.width), -1, np.int8
        )
        self.robot_z_samples = {0: []}
        self.floor_height = float(rospy.get_param("~floor_height", 2.6))
        self.floor_publish_period = max(
            0.5, float(rospy.get_param("~floor_publish_period", 2.0))
        )
        if not height_band_is_sane(
            self.min_height, self.max_height, self.floor_height
        ):
            rospy.logwarn(
                "Height band [%.2f, %.2f] m reaches an adjacent floor level "
                "(floor_height=%.2f m); geometry could leak between floor maps.",
                self.min_height,
                self.max_height,
                self.floor_height,
            )
        self.robot = None
        self.alignment = None
        self.stamp = rospy.Time()
        self.publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
        # Per-floor maps, so a consumer can address a floor by height instead of
        # relying on publication order.
        self.floor_publishers = {}
        self.floors_publisher = rospy.Publisher(
            "/simnav/map_floors", String, queue_size=1, latch=True
        )
        self.last_floor_publish = rospy.Time(0)
        rospy.Subscriber("/simnav/odom", Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber(
            "/simnav/lio_map_transform",
            TransformStamped,
            self._alignment_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~cloud_topic", "/cloud_registered"),
            PointCloud2,
            self._cloud_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/simnav/floor_exploration_context", String,
            self._floor_context_callback, queue_size=1,
        )
        self.timer = rospy.Timer(rospy.Duration(0.5), self._publish)

    def _floor_context_callback(self, message):
        try:
            floor_index = int(json.loads(message.data)["floor_index"])
        except (KeyError, TypeError, ValueError):
            return
        with self.lock:
            if floor_index == self.active_floor:
                return
            self.grid, created = ensure_floor_grid(
                self.grids, floor_index, (self.height, self.width), -1, np.int8
            )
            self.active_floor = int(floor_index)
            self.robot_z_samples.setdefault(int(floor_index), [])
            self.stamp = rospy.Time.now()
            retained = sorted(self.grids)
        rospy.loginfo(
            "Occupancy map switched to floor %d (%s); retained floors: %s",
            int(floor_index),
            "new" if created else "restored",
            retained,
        )

    def _cell(self, x, y):
        return (
            int((x - self.origin_x) / self.resolution),
            int((y - self.origin_y) / self.resolution),
        )

    def _odom_callback(self, message):
        with self.lock:
            self.robot = (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            )
            samples = self.robot_z_samples.setdefault(int(self.active_floor), [])
            # Bounded history: the level estimate only needs the visit's lowest
            # height, and an unbounded list would grow for the whole run.
            if len(samples) < 2000:
                samples.append(float(self.robot[2]))

    def _alignment_callback(self, message):
        quaternion = message.transform.rotation
        matrix = transformations.quaternion_matrix(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
        matrix[:3, 3] = [
            message.transform.translation.x,
            message.transform.translation.y,
            message.transform.translation.z,
        ]
        with self.lock:
            self.alignment = matrix

    def _cloud_callback(self, message):
        with self.lock:
            robot = self.robot
            alignment = self.alignment
        if robot is None or alignment is None:
            return
        raw_points = []
        for index, point in enumerate(
            point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if index % self.point_stride:
                continue
            raw_points.append(point)
        if not raw_points:
            return
        raw = np.asarray(raw_points, dtype=float)
        homogeneous = np.ones((raw.shape[0], 4), dtype=float)
        homogeneous[:, :3] = raw
        transformed = np.matmul(alignment, homogeneous.T).T[:, :3]
        with self.lock:
            floor_level = observed_floor_level(
                self.robot_z_samples.get(int(self.active_floor), ())
            )
        height_mask = point_floor_mask(
            transformed[:, 2], floor_level, self.min_height, self.max_height
        )
        points = []
        for point in transformed[height_mask]:
            cell = self._cell(point[0], point[1])
            if 0 <= cell[0] < self.width and 0 <= cell[1] < self.height:
                points.append(cell)
        if not points:
            return
        robot_cell = self._cell(robot[0], robot[1])
        with self.lock:
            for endpoint in points:
                # A farther return can send a ray through a previously mapped
                # wall. Only unknown cells may become free; confirmed
                # occupied cells must remain occupied.
                trace_ray_free(self.grid, robot_cell, endpoint)
            columns, rows = zip(*points)
            self.grid[np.asarray(rows), np.asarray(columns)] = 100
            self.stamp = message.header.stamp

    def _grid_message(self, grid, stamp):
        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.info.resolution = self.resolution
        message.info.width = self.width
        message.info.height = self.height
        message.info.origin.position.x = self.origin_x
        message.info.origin.position.y = self.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = grid.reshape(-1).tolist()
        return message

    def _publish(self, _event):
        now = rospy.Time.now()
        with self.lock:
            grids = {key: value.copy() for key, value in self.grids.items()}
            active = int(self.active_floor)
            samples = {
                key: tuple(value) for key, value in self.robot_z_samples.items()
            }
            stamp = self.stamp if self.stamp != rospy.Time() else now
            publish_all = (now - self.last_floor_publish).to_sec() >= (
                self.floor_publish_period
            )
            if publish_all:
                self.last_floor_publish = now
        # ``/map`` keeps its original meaning for existing consumers: the map of
        # the floor the robot is on.
        self.publisher.publish(self._grid_message(grids[active], stamp))
        if publish_all:
            for floor_index, grid in grids.items():
                publisher = self.floor_publishers.get(floor_index)
                if publisher is None:
                    publisher = rospy.Publisher(
                        "/map/floor_{}".format(int(floor_index)),
                        OccupancyGrid,
                        queue_size=1,
                        latch=True,
                    )
                    self.floor_publishers[floor_index] = publisher
                publisher.publish(self._grid_message(grid, stamp))
            level = {
                key: observed_floor_level(value) for key, value in samples.items()
            }
            report = floor_map_report(
                grids, active, level, self.floor_height
            )
            self.floors_publisher.publish(
                String(
                    data=json.dumps(
                        {
                            "active_floor": active,
                            "floor_height": self.floor_height,
                            "height_band": [self.min_height, self.max_height],
                            "floors": report,
                        },
                        sort_keys=True,
                    )
                )
            )


if __name__ == "__main__":
    rospy.init_node("lio_occupancy")
    LioOccupancyNode()
    rospy.spin()
