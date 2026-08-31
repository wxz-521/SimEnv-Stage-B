#!/usr/bin/env python3
"""Build a lightweight 2D occupancy map from FAST-LIO registered scans."""

import threading

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
import tf.transformations as transformations
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2


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
        self.grid = np.full((self.height, self.width), -1, dtype=np.int8)
        self.robot = None
        self.alignment = None
        self.stamp = rospy.Time()
        self.publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
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
        self.timer = rospy.Timer(rospy.Duration(0.5), self._publish)

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
        height_mask = np.logical_and(
            transformed[:, 2] - robot[2] >= self.min_height,
            transformed[:, 2] - robot[2] <= self.max_height,
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

    def _publish(self, _event):
        with self.lock:
            grid = self.grid.copy()
            stamp = self.stamp if self.stamp != rospy.Time() else rospy.Time.now()
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
        self.publisher.publish(message)


if __name__ == "__main__":
    rospy.init_node("lio_occupancy")
    LioOccupancyNode()
    rospy.spin()
