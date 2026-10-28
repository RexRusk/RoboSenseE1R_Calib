#!/usr/bin/env python3
"""Color /rslidar_points with the latest camera image and the locked extrinsic.

The lidar clock and the camera clock are not the same, so each cloud uses the
newest image. That matches a locked gimbal and a scene that is not moving.
"""

import os
import re
import sys

for _ros_py in (
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/opt/ros/humble/lib/python3.10/site-packages",
):
    if _ros_py not in sys.path:
        sys.path.insert(0, _ros_py)

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField


def _scalar(text, key):
    match = re.search(rf"^[ \t]*{key}:\s*([-+0-9.eE]+)", text, re.M)
    if not match:
        raise SystemExit(f"yaml has no {key}")
    return float(match.group(1))


def _data(text, parent):
    match = re.search(parent + r":.*?data:\s*\[([^\]]*)\]", text, re.S)
    if not match:
        raise SystemExit(f"yaml has no {parent}.data")
    return [float(part.strip()) for part in match.group(1).split(",") if part.strip()]


def resolve_url(url):
    if url.startswith("file://"):
        return url[len("file://"):]
    return url


class Colorizer(Node):
    def __init__(self):
        super().__init__("cloud_colorizer")
        self.declare_parameter("extrinsic_yaml", "")
        self.declare_parameter("camera_yaml", "")
        extrinsic = resolve_url(self.get_parameter("extrinsic_yaml").get_parameter_value().string_value)
        camera = resolve_url(self.get_parameter("camera_yaml").get_parameter_value().string_value)
        with open(extrinsic, "r", encoding="utf-8") as handle:
            text = handle.read()
        self.rotation = np.array(_data(text, "rotation_matrix"), dtype=np.float64).reshape(3, 3)
        self.translation = np.array([
            _scalar(text, "x"), _scalar(text, "y"), _scalar(text, "z")
        ], dtype=np.float64)
        with open(camera, "r", encoding="utf-8") as handle:
            text = handle.read()
        self.k = np.array(_data(text, "camera_matrix"), dtype=np.float64).reshape(3, 3)
        self.dist = np.array(_data(text, "distortion_coefficients"), dtype=np.float64)
        self.width = int(_scalar(text, "image_width"))
        self.height = int(_scalar(text, "image_height"))
        self.rvec, _ = cv2.Rodrigues(self.rotation)
        self.image = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, "/camera/image_raw", self._on_image, image_qos)
        self.create_subscription(PointCloud2, "/rslidar_points", self._on_cloud, qos)
        self.publisher = self.create_publisher(PointCloud2, "/rslidar_points_color", qos)
        self.get_logger().info(
            f"coloring /rslidar_points with {self.width}x{self.height} images"
        )

    def _on_image(self, msg):
        if msg.encoding != "bgr8" or msg.width != self.width or msg.height != self.height:
            self.get_logger().error(
                f"image is {msg.width}x{msg.height} {msg.encoding}, "
                f"calibration is {self.width}x{self.height} bgr8"
            )
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.step != msg.width * 3:
            frame = frame.reshape(msg.height, msg.step)[:, : msg.width * 3]
        self.image = frame.reshape(msg.height, msg.width, 3).copy()

    def _on_cloud(self, msg):
        if self.image is None or msg.point_step < 12 or msg.width == 0:
            return
        count = msg.width * msg.height
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < count * msg.point_step:
            return
        xyz = raw.reshape(count, msg.point_step)[:, :12].copy().view(np.float32).reshape(count, 3)
        finite = np.isfinite(xyz).all(axis=1)
        colors = np.full((count, 3), 70, dtype=np.uint8)
        if finite.any():
            pts = xyz[finite].astype(np.float64)
            projected, _ = cv2.projectPoints(pts, self.rvec, self.translation, self.k, self.dist)
            projected = projected.reshape(-1, 2)
            cam = (self.rotation @ pts.T).T + self.translation
            ui = np.rint(projected[:, 0]).astype(np.int32)
            vi = np.rint(projected[:, 1]).astype(np.int32)
            inside = (
                (cam[:, 2] > 0.1)
                & (ui >= 0) & (ui < self.width)
                & (vi >= 0) & (vi < self.height)
            )
            if inside.any():
                sampled = self.image[vi[inside], ui[inside]]
                colored = np.full((pts.shape[0], 3), 70, dtype=np.uint8)
                colored[inside, 0] = sampled[:, 2]
                colored[inside, 1] = sampled[:, 1]
                colored[inside, 2] = sampled[:, 0]
                colors[finite] = colored
        packed = (
            np.uint32(255) << 24
            | colors[:, 0].astype(np.uint32) << 16
            | colors[:, 1].astype(np.uint32) << 8
            | colors[:, 2].astype(np.uint32)
        ).view(np.float32)
        out = np.empty(count, dtype=[
            ("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.float32)
        ])
        out["x"] = xyz[:, 0]
        out["y"] = xyz[:, 1]
        out["z"] = xyz[:, 2]
        out["rgb"] = packed
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = count
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = 16 * count
        cloud.is_dense = False
        cloud.data = out.tobytes()
        self.publisher.publish(cloud)


def main():
    if not os.path.isdir(os.environ.get("HOME", "")):
        os.environ["HOME"] = "/root"
    rclpy.init()
    node = Colorizer()
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
