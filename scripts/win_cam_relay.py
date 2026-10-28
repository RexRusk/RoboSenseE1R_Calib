"""Capture a clean frame on Windows and write the resolution the intrinsics belong to.

This Etron camera's 16:9 mode is 1280x720. The Zhang photos are 1024x576, which is
that frame scaled by 0.8. USB/IP delivers torn JPEG above 320x240, so the file
written here is what WSL publishes.
"""

import argparse
import os
import sys
import time

import cv2


def uniform(src_w, src_h, dst_w, dst_h):
    if (src_w, src_h) == (dst_w, dst_h):
        return True
    if dst_w <= 0 or dst_h <= 0:
        return False
    return abs((src_w / dst_w) - (src_h / dst_h)) < 1e-6


def torn(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if gray.shape[0] < 48:
        return False
    body = gray[:-24]
    bottom = gray[-24:]
    return float(body.std()) > 5.0 and float(bottom.std()) < 0.5


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", type=int, default=2)
    parser.add_argument("--capture-width", type=int, default=1280)
    parser.add_argument("--capture-height", type=int, default=720)
    parser.add_argument("--width", type=int, default=1024)
    parser.add_argument("--height", type=int, default=576)
    parser.add_argument("--out", default=r"C:\Users\nonew\cam_frames\live.jpg")
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--count", type=int, default=0, help="stop after this many good frames; 0 keeps going")
    args = parser.parse_args(argv)

    cap = cv2.VideoCapture(args.device, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(
            "camera did not open. Leave it on Windows; usbipd attach hides it from this process.",
            flush=True,
        )
        return 1
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.capture_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.capture_height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    period = 1.0 / args.fps if args.fps > 0 else 0.0
    written = 0
    misses = 0
    try:
        while args.count <= 0 or written < args.count:
            frame = None
            for _ in range(4):
                ok, grabbed = cap.read()
                if ok and grabbed is not None:
                    frame = grabbed
            if frame is None:
                misses += 1
                print("no frame", misses, flush=True)
                if misses > 30:
                    return 1
                time.sleep(0.1)
                continue
            got_w, got_h = frame.shape[1], frame.shape[0]
            if (got_w, got_h) != (args.capture_width, args.capture_height):
                print(
                    f"camera returned {got_w}x{got_h}, asked {args.capture_width}x{args.capture_height}",
                    flush=True,
                )
                return 1
            if not uniform(got_w, got_h, args.width, args.height):
                print("refusing a non-uniform scale; it would invalidate K", flush=True)
                return 1
            if float(frame.mean()) < 8.0 or torn(frame):
                misses += 1
                continue
            if (got_w, got_h) != (args.width, args.height):
                frame = cv2.resize(frame, (args.width, args.height), interpolation=cv2.INTER_AREA)
            tmp = os.path.splitext(args.out)[0] + ".tmp.jpg"
            if not cv2.imwrite(tmp, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95]):
                print("imwrite failed", flush=True)
                return 1
            os.replace(tmp, args.out)
            written += 1
            misses = 0
            if written == 1 or written % 20 == 0:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                print(
                    f"wrote {args.width}x{args.height} mean={float(frame.mean()):.1f} "
                    f"bottom_std={float(gray[-24:].std()):.2f} -> {args.out}",
                    flush=True,
                )
            if period:
                time.sleep(period)
    except KeyboardInterrupt:
        return 0
    finally:
        cap.release()
    print(f"frames {written}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
