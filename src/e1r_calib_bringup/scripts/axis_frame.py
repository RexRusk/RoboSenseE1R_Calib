#!/usr/bin/env python3
"""Place the current scan in a fixed world with the axis-center formula.

p_b = Rz(yaw) Ry(pitch) Rx(roll) (p_l - c) + [0, 0, h]

Pitch and roll come from the ground, so ground points stay on z = 0.
Yaw is the horizontal motion between frames. c is the pivot in the lidar
frame. Each message is the lidar's current view, written into that world.
"""

import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)

for _ros_py in (
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/opt/ros/humble/lib/python3.10/site-packages",
):
    if _ros_py not in sys.path:
        sys.path.insert(0, _ros_py)

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster

from ground_height import fit_ground

CELL = 0.25


def rot_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def quat_from_R(rotation):
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    return float(x), float(y), float(z), float(w)


def fit_wall(points, ground_n, ground_d):
    """Return a vertical plane n·p + d = 0, with d the distance from the origin."""
    dist = np.abs(points @ ground_n + ground_d)
    rest = points[(dist > 0.12) & (np.linalg.norm(points, axis=1) < 8.0) & (np.linalg.norm(points, axis=1) > 0.4)]
    if rest.shape[0] < 80:
        return None
    rng = np.random.default_rng()
    if rest.shape[0] > 2500:
        rest = rest[rng.choice(rest.shape[0], 2500, replace=False)]
    best = None
    best_count = 0
    for _ in range(40):
        choice = rest[rng.choice(rest.shape[0], 3, replace=False)]
        normal = np.cross(choice[1] - choice[0], choice[2] - choice[0])
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal = normal / length
        if abs(float(normal[2])) > 0.35:
            continue
        offset = -float(normal @ choice[0])
        if offset < 0.0:
            normal = -normal
            offset = -offset
        count = int(np.sum(np.abs(rest @ normal + offset) < 0.04))
        if count > best_count:
            best_count = count
            best = (normal.copy(), offset)
    if best is None or best_count < 80:
        return None
    normal, offset = best
    chosen = rest[np.abs(rest @ normal + offset) < 0.04]
    center = chosen.mean(axis=0)
    _values, axes = np.linalg.eigh(np.cov(chosen.T))
    normal = axes[:, 0]
    if float(normal @ center) > 0.0:
        normal = -normal
    offset = -float(normal @ center)
    if offset < 0.0:
        normal = -normal
        offset = -offset
    if abs(float(normal[2])) > 0.35:
        return None
    return normal, offset


def _shift_stamp(stamp, extra_ns):
    total = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec) + int(extra_ns)
    shifted = Time()
    shifted.sec = total // 1_000_000_000
    shifted.nanosec = total % 1_000_000_000
    return shifted


def _cells(xy):
    ix = np.floor(xy[:, 0] / CELL).astype(np.int32)
    iy = np.floor(xy[:, 1] / CELL).astype(np.int32)
    return set(zip(ix.tolist(), iy.tolist()))


class AxisFrame(Node):
    def __init__(self):
        super().__init__("axis_frame")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, "/rslidar_points", self._on_cloud, qos)
        self.cloud_pub = self.create_publisher(PointCloud2, "/rslidar_points_world", qos)
        self.tf_pub = TransformBroadcaster(self)
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.height = 0.0
        self.distance = 0.0
        self.pivot = np.zeros(3)
        self.ready = False
        self.prev = None
        self.ground_obs = []
        self.wall_obs = []
        self.get_logger().info("world cloud /rslidar_points_world uses the axis-center formula")

    def _on_cloud(self, msg):
        count = msg.width * msg.height
        if count == 0 or msg.point_step < 16:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < count * msg.point_step:
            return
        cols = raw.reshape(count, msg.point_step)[:, :16].copy().view(np.float32).reshape(count, 4)
        finite = np.isfinite(cols[:, :3]).all(axis=1)
        cols = cols[finite]
        if cols.shape[0] < 80:
            return
        xyz = cols[:, :3].astype(np.float64)
        intensity = cols[:, 3].astype(np.float32)
        fitted = fit_ground(xyz)
        if fitted is not None:
            distance, pitch, roll, _sigma, _count = fitted
            self.distance = float(distance)
            self.pitch = float(pitch)
            self.roll = float(roll)
            self.ready = True
            normal = np.array([
                -np.sin(self.pitch),
                np.cos(self.pitch) * np.sin(self.roll),
                np.cos(self.pitch) * np.cos(self.roll),
            ])
            level = rot_y(self.pitch) @ rot_x(self.roll)
            leveled = (level @ (xyz - self.pivot).T).T
            flat = np.hypot(leveled[:, 0], leveled[:, 1])
            xy = leveled[(flat > 1.0) & (flat < 8.0)][:, :2]
            if xy.shape[0] > 1500:
                pick = np.random.default_rng().choice(xy.shape[0], 1500, replace=False)
                xy = xy[pick]
            self.yaw += self._delta_yaw(xy)
            pose = rot_z(self.yaw) @ level
            self._remember_plane(self.ground_obs, normal, self.distance)
            wall = fit_wall(xyz, normal, self.distance)
            if wall is not None:
                self._remember_plane(self.wall_obs, wall[0], wall[1], self._wall_key(pose, wall[0], wall[1]))
            self._solve_pivot()
            self.height = float(self.distance + normal @ self.pivot)
        if not self.ready:
            return
        pose = rot_z(self.yaw) @ rot_y(self.pitch) @ rot_x(self.roll)
        world = (pose @ (xyz - self.pivot).T).T
        world[:, 2] += self.height
        self._publish_cloud(msg.header.stamp, world, intensity)
        self._publish_tf(msg.header.stamp, pose)

    def _delta_yaw(self, xy):
        cells = _cells(xy) if xy.shape[0] >= 40 else set()
        if self.prev is None or len(self.prev) < 30 or xy.shape[0] < 40:
            self.prev = cells
            return 0.0
        best_angle = 0.0
        best_score = -1
        step = max(1, xy.shape[0] // 400)
        samples = xy[::step]
        for angle in np.linspace(np.deg2rad(-15.0), np.deg2rad(15.0), 21):
            c, s = np.cos(angle), np.sin(angle)
            rx = c * samples[:, 0] - s * samples[:, 1]
            ry = s * samples[:, 0] + c * samples[:, 1]
            ix = np.floor(rx / CELL).astype(np.int32)
            iy = np.floor(ry / CELL).astype(np.int32)
            score = sum((int(a), int(b)) in self.prev for a, b in zip(ix.tolist(), iy.tolist()))
            if score > best_score:
                best_score = score
                best_angle = float(angle)
        self.prev = cells
        if best_score < 0.12 * samples.shape[0]:
            return 0.0
        return best_angle

    def _remember_plane(self, store, normal, offset, key=None):
        if store:
            cosine = abs(float(np.clip(normal @ store[-1][0], -1.0, 1.0)))
            if cosine > np.cos(np.deg2rad(1.5)) and abs(offset - store[-1][1]) < 0.01:
                return
        if key is None:
            store.append((normal.copy(), float(offset)))
        else:
            store.append((normal.copy(), float(offset), key))
        if len(store) > 36:
            del store[0]

    def _wall_key(self, pose, normal, offset):
        world = pose @ normal
        signed = offset + float(normal @ self.pivot)
        if world[0] < 0.0:
            world = -world
            signed = -signed
        bearing = int(np.round(np.degrees(np.arctan2(world[1], world[0])) / 30.0))
        distance = int(np.round(signed / 0.5))
        return bearing, distance

    def _solve_pivot(self):
        if len(self.ground_obs) < 3:
            return
        groups = {}
        for normal, offset, key in self.wall_obs:
            groups.setdefault(key, []).append((normal, offset))
        keys = [key for key, obs in groups.items() if len(obs) >= 3]
        if not keys:
            return
        width = 4 + len(keys)
        rows = []
        rhs = []
        for normal, offset in self.ground_obs:
            row = np.zeros(width)
            row[:3] = normal
            row[3] = -1.0
            rows.append(row)
            rhs.append(-offset)
        for index, key in enumerate(keys):
            for normal, offset in groups[key]:
                row = np.zeros(width)
                row[:3] = normal
                row[4 + index] = -1.0
                rows.append(row)
                rhs.append(-offset)
        design = np.vstack(rows)
        target = np.array(rhs)
        normal_eq = design.T @ design
        normal_eq[:3, :3] += 1e-3 * np.eye(3)
        solved = np.linalg.solve(normal_eq, design.T @ target)
        pivot = solved[:3]
        if np.linalg.norm(pivot) > 0.4:
            return
        if np.linalg.norm(self.pivot) > 0.0 and np.linalg.norm(pivot - self.pivot) > 0.15:
            return
        residual = design @ solved - target
        if np.median(np.abs(residual)) > 0.03:
            return
        if np.linalg.norm(pivot - self.pivot) > 0.01:
            self.get_logger().info(
                f"pivot {pivot[0]:+.3f} {pivot[1]:+.3f} {pivot[2]:+.3f} m"
            )
        self.pivot = pivot

    def _publish_cloud(self, stamp, world, intensity):
        count = world.shape[0]
        out = np.empty(count, dtype=[
            ("x", np.float32), ("y", np.float32), ("z", np.float32), ("intensity", np.float32),
        ])
        out["x"] = world[:, 0]
        out["y"] = world[:, 1]
        out["z"] = world[:, 2]
        out["intensity"] = intensity
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = "base"
        cloud.height = 1
        cloud.width = count
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = 16 * count
        cloud.is_dense = True
        cloud.data = out.tobytes()
        self.cloud_pub.publish(cloud)

    def _publish_tf(self, stamp, pose):
        shift = np.array([0.0, 0.0, self.height]) - pose @ self.pivot
        qx, qy, qz, qw = quat_from_R(pose)
        for extra_ns in (0, 200_000_000):
            msg = TransformStamped()
            msg.header.stamp = _shift_stamp(stamp, extra_ns)
            msg.header.frame_id = "base"
            msg.child_frame_id = "rslidar"
            msg.transform.translation.x = float(shift[0])
            msg.transform.translation.y = float(shift[1])
            msg.transform.translation.z = float(shift[2])
            msg.transform.rotation.x = qx
            msg.transform.rotation.y = qy
            msg.transform.rotation.z = qz
            msg.transform.rotation.w = qw
            self.tf_pub.sendTransform(msg)


def main():
    if not os.path.isdir(os.environ.get("HOME", "")):
        os.environ["HOME"] = "/root"
    rclpy.init()
    node = AxisFrame()
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
