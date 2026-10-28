"""Zhang camera intrinsics. Square size does not change K.

A later RGB camera is a new photo set and a new yaml, not a change to this file.
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np


def parse_corners(text):
    parts = text.lower().replace("*", "x").split("x")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("use COLSxROWS, for example 4x6")
    return int(parts[0]), int(parts[1])


def collect_images(spec):
    if any(ch in spec for ch in "*?["):
        paths = glob.glob(spec)
    elif os.path.isdir(spec):
        paths = []
        for name in sorted(os.listdir(spec)):
            if name.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
                paths.append(os.path.join(spec, name))
    else:
        paths = [spec]
    return sorted(paths)


def write_yaml(path, width, height, camera_name, k, dist, rms, corners, views):
    fx, fy = float(k[0, 0]), float(k[1, 1])
    cx, cy = float(k[0, 2]), float(k[1, 2])
    d = [float(v) for v in np.ravel(dist)[:5]]
    while len(d) < 5:
        d.append(0.0)
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    text = f"""# plumb_bob intrinsics for {width}x{height}
# inner corners {corners[0]}x{corners[1]}, views {views}, rms_px {rms:.4f}
# k3 is fixed at 0: the board does not reach the image edge, so k3 is not observable
image_width: {width}
image_height: {height}
camera_name: {camera_name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{fx:.8f}, 0.0, {cx:.8f}, 0.0, {fy:.8f}, {cy:.8f}, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [{d[0]:.8f}, {d[1]:.8f}, {d[2]:.8f}, {d[3]:.8f}, {d[4]:.8f}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [{fx:.8f}, 0.0, {cx:.8f}, 0.0, 0.0, {fy:.8f}, {cy:.8f}, 0.0, 0.0, 0.0, 1.0, 0.0]
"""
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", required=True, help="directory or glob of board photos")
    parser.add_argument("--contains", default="", help="keep only filenames containing this text")
    parser.add_argument("--corners", type=parse_corners, default=(4, 6), help="inner corners COLSxROWS")
    parser.add_argument(
        "--square",
        type=float,
        default=1.0,
        help="square size in metres; scales the board pose only, K is unchanged",
    )
    parser.add_argument("--camera-name", default="usb_camera")
    parser.add_argument("--output", required=True, help="camera_info yaml to write")
    parser.set_defaults(fix_k3=True)
    parser.add_argument("--fix-k3", dest="fix_k3", action="store_true")
    parser.add_argument("--no-fix-k3", dest="fix_k3", action="store_false")
    args = parser.parse_args(argv)

    photos = collect_images(args.images)
    if args.contains:
        photos = [path for path in photos if args.contains in os.path.basename(path)]
    if not photos:
        print("no images matched", args.images, file=sys.stderr)
        return 1

    cols, rows = args.corners
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * float(args.square)
    obj_points, img_points = [], []
    image_size = None
    for path in photos:
        image = cv2.imread(path)
        if image is None:
            print("unreadable", path)
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        size = (gray.shape[1], gray.shape[0])
        if image_size is None:
            image_size = size
        elif size != image_size:
            print("skip size", path, size, "expected", image_size)
            continue
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
        if not found:
            print("no board", os.path.basename(path))
            continue
        corners = cv2.cornerSubPix(
            gray,
            corners,
            (5, 5),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001),
        )
        obj_points.append(objp)
        img_points.append(corners)
        print("board", os.path.basename(path), f"{size[0]}x{size[1]}")

    if len(obj_points) < 3:
        print("need at least 3 views, got", len(obj_points), file=sys.stderr)
        return 1

    calib_flags = cv2.CALIB_FIX_K3 if args.fix_k3 else 0
    rms, camera_matrix, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None, flags=calib_flags
    )
    for index, (obj, img, rvec, tvec) in enumerate(zip(obj_points, img_points, rvecs, tvecs)):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist)
        err = np.linalg.norm(img.reshape(-1, 2) - projected.reshape(-1, 2), axis=1)
        print(f"view {index} mean_px {float(err.mean()):.3f}")
    print(f"rms_px {float(rms):.4f} fix_k3 {args.fix_k3}")
    print("K")
    print(np.array2string(camera_matrix, precision=4, suppress_small=True))
    print("dist", np.array2string(np.ravel(dist), precision=6, suppress_small=True))
    write_yaml(
        args.output,
        image_size[0],
        image_size[1],
        args.camera_name,
        camera_matrix,
        dist,
        float(rms),
        (cols, rows),
        len(obj_points),
    )
    print("wrote", args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
