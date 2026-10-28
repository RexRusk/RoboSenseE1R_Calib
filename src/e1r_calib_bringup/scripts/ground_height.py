#!/usr/bin/env python3
"""Publish the lidar origin's height above the ground.

The mechanism encoder is not connected. With the gimbal pivot taken as the
lidar origin, that height is the installation height. Pitch comes from the
same ground normal. Each frame is solved on its own, so the number does not
drift while the gimbal rotates.
"""

import os
import sys

for _ros_py in (
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/opt/ros/humble/lib/python3.10/site-packages",
):
    if _ros_py not in sys.path:
        sys.path.insert(0, _ros_py)

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker


def fit_ground(xyz):
    """Return height, pitch, roll, residual std, and inlier count.

    Height is the perpendicular distance from the lidar origin to the ground
    plane. The plane is n·p + d = 0 with n pointing up, so the distance is d.
    """
    range_m = np.linalg.norm(xyz, axis=1)
    # Ground is below the housing, but once the lidar looks down it sits at
    # positive z in the lidar frame. Do not require negative elevation.
    keep = (
        np.isfinite(range_m)
        & (range_m > 0.4)
        & (range_m < 8.0)
        & (xyz[:, 0] > 0.3)
        & (xyz[:, 2] > -3.0)
        & (xyz[:, 2] < 4.0)
    )
    pts = xyz[keep]
    if pts.shape[0] < 80:
        return None
    if pts.shape[0] > 4000:
        pick = np.random.default_rng().choice(pts.shape[0], 4000, replace=False)
        pts = pts[pick]
    rng = np.random.default_rng()
    best_count = 0
    best_inliers = None
    for _ in range(60):
        sample = pts[rng.choice(pts.shape[0], 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal = normal / length
        if normal[2] < 0.0:
            normal = -normal
        if normal[2] < 0.5:
            continue
        offset = -float(np.dot(normal, sample[0]))
        inliers = np.abs(pts @ normal + offset) < 0.04
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers
    if best_inliers is None or best_count < 80:
        return None
    chosen = pts[best_inliers]
    center = chosen.mean(axis=0)
    _values, axes = np.linalg.eigh(np.cov(chosen.T))
    normal = axes[:, 0]
    if normal[2] < 0.0:
        normal = -normal
    if normal[2] < 0.5:
        return None
    offset = -float(np.dot(normal, center))
    if offset < 0.25 or offset > 2.2:
        return None
    height = offset
    residual = np.abs(chosen @ normal + offset)
    pitch = float(np.arctan2(-normal[0], np.hypot(normal[1], normal[2])))
    roll = float(np.arctan2(normal[1], normal[2]))
    return height, pitch, roll, float(residual.std()), int(chosen.shape[0])


class GroundHeight(Node):
    def __init__(self):
        super().__init__("ground_height")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, "/rslidar_points", self._on_cloud, qos)
        self.height_pub = self.create_publisher(Float64, "/lidar_height", qos)
        self.marker_pub = self.create_publisher(Marker, "/lidar_height_marker", qos)
        self.get_logger().info("fitting ground height from /rslidar_points")

    def _on_cloud(self, msg):
        count = msg.width * msg.height
        if count == 0 or msg.point_step < 12:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < count * msg.point_step:
            return
        xyz = raw.reshape(count, msg.point_step)[:, :12].copy().view(np.float32).reshape(count, 3)
        fitted = fit_ground(xyz.astype(np.float64))
        text = "h --"
        if fitted is not None:
            height, pitch, _roll, sigma, inliers = fitted
            self.height_pub.publish(Float64(data=float(height)))
            text = f"h {height:.3f} m\npitch {np.degrees(pitch):+.1f} deg\nsigma {sigma * 100:.1f} cm"
        marker = Marker()
        marker.header = msg.header
        marker.ns = "height"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 1.5
        marker.pose.position.y = -1.0
        marker.pose.position.z = 0.8
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.22
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = text
        self.marker_pub.publish(marker)


def main():
    if not os.path.isdir(os.environ.get("HOME", "")):
        os.environ["HOME"] = "/root"
    rclpy.init()
    node = GroundHeight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
