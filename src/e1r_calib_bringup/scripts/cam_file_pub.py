#!/usr/bin/env python3
"""Publish a JPEG plus the CameraInfo yaml that was solved at that resolution."""

import json
import os
import re
import sys
import time

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
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


def _scalar(text, key):
    match = re.search(rf"^{key}:\s*(\S+)", text, re.M)
    if not match:
        raise SystemExit(f"camera yaml has no {key}")
    return match.group(1)


def _data(text, parent):
    match = re.search(parent + r":.*?data:\s*\[([^\]]*)\]", text, re.S)
    if not match:
        raise SystemExit(f"camera yaml has no {parent}.data")
    return [float(part.strip()) for part in match.group(1).split(",") if part.strip()]


def load_camera_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        text = handle.read()
    width = int(_scalar(text, "image_width"))
    height = int(_scalar(text, "image_height"))
    model = _scalar(text, "distortion_model")
    return {
        "width": width,
        "height": height,
        "model": model,
        "k": _data(text, "camera_matrix"),
        "d": _data(text, "distortion_coefficients"),
        "r": _data(text, "rectification_matrix"),
        "p": _data(text, "projection_matrix"),
    }


def resolve_url(url):
    if url.startswith("file://"):
        return url[len("file://"):]
    return url


class FileCamera(Node):
    def __init__(self):
        super().__init__("camera_file_publisher")
        self.declare_parameter("jpeg_path", "/tmp/e1r_live.jpg")
        self.declare_parameter("camera_info_url", "")
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("rate", 10.0)
        self.jpeg_path = self.get_parameter("jpeg_path").get_parameter_value().string_value
        url = self.get_parameter("camera_info_url").get_parameter_value().string_value.strip()
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.calib = load_camera_yaml(resolve_url(url))
        self.qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_pub = self.create_publisher(Image, "/camera/image_raw", self.qos)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", self.qos)
        self.manifest_path = os.path.join(os.path.dirname(self.jpeg_path), "cameras.json")
        self.manifest_mtime = None
        self.cameras = []
        self.streams = {}
        period = 1.0 / max(self.get_parameter("rate").get_parameter_value().double_value, 0.1)
        self.seen_mtime = None
        self.cached = None
        self.last_wait_log = 0.0
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(f"USB camera publishes /camera/image_raw from {self.jpeg_path}")

    def _load_manifest(self):
        if not os.path.isfile(self.manifest_path):
            return
        mtime = os.path.getmtime(self.manifest_path)
        if mtime == self.manifest_mtime:
            return
        with open(self.manifest_path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        self.cameras = payload.get("cameras", [])
        self.manifest_mtime = mtime
        folder = os.path.dirname(self.jpeg_path)
        for entry in self.cameras:
            slug = entry["slug"]
            if slug in self.streams:
                continue
            topic = f"/camera/{slug}/image_raw"
            self.streams[slug] = {
                "path": os.path.join(folder, entry["file"]),
                "pub": self.create_publisher(Image, topic, self.qos),
                "mtime": None,
                "cached": None,
            }
            self.get_logger().info(f"{entry['name']} -> {topic}")

    def _publish_image(self, publisher, frame, frame_id):
        height, width = frame.shape[:2]
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        image = Image()
        image.header = header
        image.height = height
        image.width = width
        image.encoding = "bgr8"
        image.is_bigendian = 0
        image.step = width * 3
        image.data = frame.tobytes()
        publisher.publish(image)
        return header

    def _tick(self):
        self._load_manifest()
        for entry in self.cameras:
            if entry.get("calibrated"):
                continue
            stream = self.streams.get(entry["slug"])
            if stream is None or not os.path.isfile(stream["path"]):
                continue
            mtime = os.path.getmtime(stream["path"])
            if mtime == stream["mtime"]:
                continue
            raw = np.fromfile(stream["path"], dtype=np.uint8)
            frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            if frame is None:
                continue
            stream["cached"] = frame
            stream["mtime"] = mtime
            self._publish_image(stream["pub"], frame, "camera_" + entry["slug"])
        if not any(entry.get("calibrated") for entry in self.cameras):
            return
        if not os.path.isfile(self.jpeg_path):
            now = time.monotonic()
            if now - self.last_wait_log > 2.0:
                self.get_logger().warn(f"waiting for {self.jpeg_path}")
                self.last_wait_log = now
            return
        mtime = os.path.getmtime(self.jpeg_path)
        if mtime != self.seen_mtime:
            raw = np.fromfile(self.jpeg_path, dtype=np.uint8)
            frame = cv2.imdecode(raw, cv2.IMREAD_COLOR)
            if frame is None:
                return
            height, width = frame.shape[:2]
            if (width, height) != (self.calib["width"], self.calib["height"]):
                self.get_logger().error(
                    f"jpeg is {width}x{height}, camera yaml is "
                    f"{self.calib['width']}x{self.calib['height']}"
                )
                self.seen_mtime = mtime
                return
            self.cached = frame
            self.seen_mtime = mtime
        else:
            return
        frame = self.cached
        height, width = frame.shape[:2]
        stamp = self.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = self.frame_id
        image = Image()
        image.header = header
        image.height = height
        image.width = width
        image.encoding = "bgr8"
        image.is_bigendian = 0
        image.step = width * 3
        image.data = frame.tobytes()
        info = CameraInfo()
        info.header = header
        info.width = width
        info.height = height
        info.distortion_model = self.calib["model"]
        info.d = self.calib["d"]
        info.k = self.calib["k"]
        info.r = self.calib["r"]
        info.p = self.calib["p"]
        self.image_pub.publish(image)
        self.info_pub.publish(info)


def main():
    if not os.path.isdir(os.environ.get("HOME", "")):
        os.environ["HOME"] = "/root"
    rclpy.init()
    node = FileCamera()
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
