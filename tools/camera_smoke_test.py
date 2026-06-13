#!/usr/bin/env python3
"""
camera_smoke_test.py - standalone, no ROS.

Run on the Ninja Pi BEFORE the ROS chain. If this prints OK and saves
JPEGs, pi_camera_node will work too. If FAIL, fix the OS side first.

Usage:
    python3 camera_smoke_test.py
    python3 camera_smoke_test.py --backend gstreamer --width 1280 --height 720
    python3 camera_smoke_test.py --backend opencv --device /dev/video0

Required on Ubuntu 24.04:
    sudo apt install -y libcamera-tools \\
        gstreamer1.0-libcamera \\
        gstreamer1.0-plugins-base gstreamer1.0-plugins-good \\
        python3-opencv
"""

import argparse
import os
import sys
import time

try:
    import cv2
except Exception as exc:
    print(f"FAIL: opencv not installed: {exc}")
    sys.exit(1)

try:
    from picamera2 import Picamera2
    HAVE_PICAMERA2 = True
except Exception:
    HAVE_PICAMERA2 = False

CV_HAS_GSTREAMER = "GStreamer: YES" in cv2.getBuildInformation()


def gst_pipeline(width, height, fps):
    return (
        f"libcamerasrc ! "
        f"video/x-raw,width={width},height={height},framerate={int(fps)}/1 ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )


def try_picamera2(width, height, frames, period):
    if not HAVE_PICAMERA2:
        print("picamera2: not installed (expected on Ubuntu 24.04), skip")
        return False
    try:
        cam = Picamera2()
        cfg = cam.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        cam.configure(cfg)
        cam.start()
        time.sleep(0.5)
        n = 0
        for i in range(frames):
            arr = cam.capture_array()
            if arr is None or arr.size == 0:
                continue
            n += 1
            cv2.imwrite(f"/tmp/ninja_cam_picamera2_{i:02d}.jpg", arr)
            print(f"picamera2 frame {i}: shape={arr.shape}")
            time.sleep(period)
        cam.stop()
        cam.close()
        if n > 0:
            print(f"picamera2 OK ({n}/{frames})")
            return True
        return False
    except Exception as exc:
        print(f"picamera2 error: {exc}")
        return False


def try_gstreamer(width, height, fps, frames, period):
    if not CV_HAS_GSTREAMER:
        print("gstreamer: cv2 was built without GStreamer support")
        print("  fix: sudo apt install python3-opencv (apt build has gst)")
        print("       (and uninstall pip cv2 if present)")
        return False
    pipe = gst_pipeline(width, height, fps)
    print(f"gstreamer pipeline: {pipe}")
    try:
        cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("gstreamer: VideoCapture did not open")
            print("  check: libcamera-hello --list-cameras")
            print("  check: gst-inspect-1.0 libcamerasrc")
            cap.release()
            return False
        n = 0
        for i in range(frames):
            ok, frame = cap.read()
            if not ok or frame is None:
                print(f"gstreamer frame {i}: grab failed")
                continue
            n += 1
            cv2.imwrite(f"/tmp/ninja_cam_gstreamer_{i:02d}.jpg", frame)
            print(f"gstreamer frame {i}: shape={frame.shape}")
            time.sleep(period)
        cap.release()
        if n > 0:
            print(f"gstreamer OK ({n}/{frames})")
            return True
        return False
    except Exception as exc:
        print(f"gstreamer error: {exc}")
        return False


def try_opencv(device, width, height, frames, period):
    candidates = [device, 0, 1]
    seen = set()
    for cand in candidates:
        if cand in seen:
            continue
        seen.add(cand)
        try:
            cap = cv2.VideoCapture(cand)
            if not cap.isOpened():
                print(f"opencv {cand}: open failed")
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            n = 0
            for i in range(frames):
                ok, frame = cap.read()
                if not ok or frame is None:
                    continue
                n += 1
                cv2.imwrite(f"/tmp/ninja_cam_opencv_{i:02d}.jpg", frame)
                print(f"opencv {cand} frame {i}: shape={frame.shape}")
                time.sleep(period)
            cap.release()
            if n > 0:
                print(f"opencv {cand} OK ({n}/{frames})")
                return True
        except Exception as exc:
            print(f"opencv {cand} error: {exc}")
    return False


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--backend",
        choices=["auto", "picamera2", "gstreamer", "opencv"],
        default="auto",
    )
    p.add_argument("--device", default="/dev/video0")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--frames", type=int, default=5)
    p.add_argument("--rate", type=float, default=10.0)
    args = p.parse_args()

    period = 1.0 / max(1.0, args.rate)
    print(
        f"smoke test: backend={args.backend} {args.width}x{args.height} "
        f"frames={args.frames} rate={args.rate}"
    )
    print(f"  cv2 GStreamer: {CV_HAS_GSTREAMER}")
    print(f"  picamera2:     {HAVE_PICAMERA2}")

    ok = False
    if args.backend in ("auto", "picamera2"):
        ok = try_picamera2(args.width, args.height, args.frames, period)
    if not ok and args.backend in ("auto", "gstreamer"):
        ok = try_gstreamer(args.width, args.height, args.rate, args.frames, period)
    if not ok and args.backend in ("auto", "opencv"):
        ok = try_opencv(args.device, args.width, args.height, args.frames, period)

    if ok:
        outs = sorted(f for f in os.listdir("/tmp") if f.startswith("ninja_cam_"))
        if outs:
            print("Saved files:")
            for f in outs[-args.frames:]:
                print(f"  /tmp/{f}")
        print("OK")
        return 0
    print("FAIL: no working camera backend")
    return 2


if __name__ == "__main__":
    sys.exit(main())
