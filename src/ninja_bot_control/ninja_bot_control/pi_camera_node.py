#!/usr/bin/env python3
"""
pi_camera_node.py  (v2 - Ubuntu 24.04 / Pi 5 / CSI Pi Camera)

Publishes /camera/image_raw. Three backends:
  1. picamera2  - SKIPPED on Ubuntu 24.04, not in apt
  2. gstreamer  - libcamerasrc -> appsink (Ubuntu 24.04 + CSI Pi Camera)
  3. opencv     - cv2.VideoCapture for USB webcams or legacy v4l2

REQUIRED packages on Ubuntu 24.04 Pi 5:
    sudo apt install -y \\
        libcamera-tools \\
        gstreamer1.0-libcamera \\
        gstreamer1.0-tools \\
        gstreamer1.0-plugins-base \\
        gstreamer1.0-plugins-good \\
        python3-opencv

Sanity-check before running:
    libcamera-hello --list-cameras
    gst-inspect-1.0 libcamerasrc | head
    python3 -c 'import cv2; print(cv2.getBuildInformation())' | grep -i gstreamer

Outputs:
    /camera/image_raw                  sensor_msgs/Image (bgr8)
    /camera/image_raw/compressed       sensor_msgs/CompressedImage (jpeg)
    /ninja/camera/status               std_msgs/String (JSON, ~1 Hz)
"""

import json
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

try:
    from picamera2 import Picamera2
    _HAVE_PICAMERA2 = True
except Exception:
    _HAVE_PICAMERA2 = False

try:
    import cv2
    _HAVE_CV2 = True
    _CV_BUILD_HAS_GSTREAMER = "GStreamer: YES" in cv2.getBuildInformation()
except Exception:
    _HAVE_CV2 = False
    _CV_BUILD_HAS_GSTREAMER = False

try:
    from cv_bridge import CvBridge
    _HAVE_BRIDGE = True
except Exception:
    _HAVE_BRIDGE = False


def _libcamerasrc_pipeline(width: int, height: int, fps: float) -> str:
    fps_int = max(1, int(round(fps)))
    return (
        f"libcamerasrc ! "
        f"video/x-raw,width={width},height={height},framerate={fps_int}/1 ! "
        f"videoconvert ! "
        f"video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )


class PiCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("pi_camera_node")

        self.declare_parameter("backend", "auto")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("rotate_deg", 0)
        self.declare_parameter("hflip", False)
        self.declare_parameter("vflip", False)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("frame_id", "ninja_camera")
        self.declare_parameter("publish_compressed", True)
        self.declare_parameter("jpeg_quality", 70)

        self.backend_choice = str(self.get_parameter("backend").value).lower()
        self.device = str(self.get_parameter("device").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.rotate_deg = int(self.get_parameter("rotate_deg").value)
        self.hflip = bool(self.get_parameter("hflip").value)
        self.vflip = bool(self.get_parameter("vflip").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_compressed = bool(
            self.get_parameter("publish_compressed").value
        )
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        if not _HAVE_BRIDGE:
            raise RuntimeError("cv_bridge import failed")
        if not _HAVE_CV2:
            raise RuntimeError("opencv-python not available")
        if not _CV_BUILD_HAS_GSTREAMER:
            self.get_logger().warn(
                "OpenCV without GStreamer support. The 'gstreamer' backend "
                "will not work. Likely a pip-installed cv2 is shadowing "
                "the apt one."
            )

        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=2,
        )
        self.image_pub = self.create_publisher(Image, self.image_topic, qos)
        self.compressed_pub = (
            self.create_publisher(
                CompressedImage, self.image_topic + "/compressed", qos
            )
            if self.publish_compressed else None
        )
        self.status_pub = self.create_publisher(
            String, "/ninja/camera/status", 10
        )

        self.backend_name: str = ""
        self.picam: Optional["Picamera2"] = None
        self.cv_cap: Optional["cv2.VideoCapture"] = None
        self._select_and_open_backend()

        self.frames_total = 0
        self.frames_failed = 0
        self.last_frame_t = 0.0

        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f"camera up: backend={self.backend_name} {self.width}x{self.height} "
            f"-> {self.image_topic} @ {self.rate_hz:.0f} Hz"
        )

    def _select_and_open_backend(self) -> None:
        if self.backend_choice == "picamera2":
            order = ["picamera2"]
        elif self.backend_choice == "gstreamer":
            order = ["gstreamer"]
        elif self.backend_choice == "opencv":
            order = ["opencv"]
        else:
            order = ["picamera2", "gstreamer", "opencv"]

        last_err: Optional[Exception] = None
        for backend in order:
            try:
                if backend == "picamera2":
                    if not _HAVE_PICAMERA2:
                        raise RuntimeError("python3-picamera2 not available")
                    self._open_picamera2()
                    self.backend_name = "picamera2"
                    return
                elif backend == "gstreamer":
                    if not _CV_BUILD_HAS_GSTREAMER:
                        raise RuntimeError("OpenCV has no GStreamer support")
                    self._open_gstreamer()
                    self.backend_name = "gstreamer"
                    return
                elif backend == "opencv":
                    self._open_opencv()
                    self.backend_name = "opencv"
                    return
            except Exception as exc:
                self.get_logger().warn(f"backend '{backend}' failed: {exc}")
                last_err = exc

        raise RuntimeError(
            f"No camera backend opened. Last error: {last_err}\n"
            "For CSI Pi Camera on Ubuntu 24.04:\n"
            "  sudo apt install libcamera-tools gstreamer1.0-libcamera "
            "gstreamer1.0-plugins-base gstreamer1.0-plugins-good\n"
            "Then verify: libcamera-hello --list-cameras"
        )

    def _open_picamera2(self) -> None:
        self.picam = Picamera2()
        config = self.picam.create_video_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"}
        )
        self.picam.configure(config)
        self.picam.start()
        for _ in range(2):
            try:
                self.picam.capture_array()
            except Exception:
                pass

    def _open_gstreamer(self) -> None:
        pipeline = _libcamerasrc_pipeline(self.width, self.height, self.rate_hz)
        self.get_logger().info(f"gstreamer pipeline: {pipeline}")
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(
                "cv2.VideoCapture failed to open libcamerasrc pipeline. "
                "Verify with: libcamera-hello --list-cameras"
            )
        for _ in range(3):
            ok, _ = cap.read()
            if not ok:
                cap.release()
                raise RuntimeError("gstreamer capture grabbed no frames")
        self.cv_cap = cap

    def _open_opencv(self) -> None:
        candidates = [self.device, 0]
        for cand in candidates:
            try:
                cap = cv2.VideoCapture(cand)
                if not cap.isOpened():
                    cap.release()
                    continue
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
                cap.set(cv2.CAP_PROP_FPS, float(self.rate_hz))
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                ok, _ = cap.read()
                if not ok:
                    cap.release()
                    continue
                self.cv_cap = cap
                return
            except Exception:
                continue
        raise RuntimeError("opencv: no working /dev/video* device")

    def _capture(self) -> Optional[np.ndarray]:
        if self.backend_name == "picamera2" and self.picam is not None:
            try:
                arr = self.picam.capture_array()
                if arr is None:
                    return None
                if arr.ndim == 3 and arr.shape[2] == 3:
                    return arr
                return None
            except Exception as exc:
                self.get_logger().warn(f"picamera2 capture: {exc}")
                return None
        if self.backend_name in ("gstreamer", "opencv") and self.cv_cap is not None:
            ok, frame = self.cv_cap.read()
            if not ok or frame is None:
                return None
            return frame
        return None

    def _post(self, frame: np.ndarray) -> np.ndarray:
        if self.rotate_deg == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotate_deg == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotate_deg == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if self.hflip:
            frame = cv2.flip(frame, 1)
        if self.vflip:
            frame = cv2.flip(frame, 0)
        return frame

    def _tick(self) -> None:
        frame = self._capture()
        if frame is None:
            self.frames_failed += 1
            return

        frame = self._post(frame)
        stamp = self.get_clock().now().to_msg()

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)
        except Exception as exc:
            self.get_logger().warn(f"image publish failed: {exc}")
            return

        if self.compressed_pub is not None:
            try:
                ok, jpeg = cv2.imencode(
                    ".jpg", frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
                )
                if ok:
                    cmsg = CompressedImage()
                    cmsg.header.stamp = stamp
                    cmsg.header.frame_id = self.frame_id
                    cmsg.format = "jpeg"
                    cmsg.data = jpeg.tobytes()
                    self.compressed_pub.publish(cmsg)
            except Exception as exc:
                self.get_logger().warn(f"compressed publish failed: {exc}")

        self.frames_total += 1
        self.last_frame_t = time.time()

    def _publish_status(self) -> None:
        age = time.time() - self.last_frame_t if self.last_frame_t > 0 else None
        msg = String()
        msg.data = json.dumps({
            "backend": self.backend_name,
            "frames_total": self.frames_total,
            "frames_failed": self.frames_failed,
            "last_frame_age_s": round(age, 2) if age is not None else None,
            "width": self.width, "height": self.height,
            "rate_hz": self.rate_hz,
            "cv_has_gstreamer": _CV_BUILD_HAS_GSTREAMER,
        }, separators=(",", ":"))
        self.status_pub.publish(msg)

    def destroy_node(self) -> None:
        try:
            if self.picam is not None:
                self.picam.stop()
                self.picam.close()
        except Exception:
            pass
        try:
            if self.cv_cap is not None:
                self.cv_cap.release()
        except Exception:
            pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
