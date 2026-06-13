#!/usr/bin/env python3
"""
Low-bandwidth Ninja crate detector for ROS 2.

Subscribes:
  /ninja/camera/image_raw           sensor_msgs/Image

Publishes:
  /ninja/vision/crate               std_msgs/String JSON, lightweight, every processed frame
  /ninja/vision/debug_image          sensor_msgs/Image, low FPS, resized, best-effort QoS

Design choices for Tailscale/NoMachine stability:
  - DICT_4X4_50 only
  - optional frame skipping
  - debug image is throttled
  - debug image is resized before publishing
  - debug image is only created/published if there is a subscriber
  - debug image publisher uses BEST_EFFORT + KEEP_LAST(1)
"""

from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String

BLUE_CRATE_ID = 36
YELLOW_CRATE_ID = 47
EMPTY_CRATE_ID = 41
VALID_CRATE_IDS = {BLUE_CRATE_ID, YELLOW_CRATE_ID, EMPTY_CRATE_ID}


def make_aruco_parameters():
    """Compatible ArUco parameter creation for different OpenCV versions."""
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    return cv2.aruco.DetectorParameters()


class CrateDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("crate_detector_node")

        self.declare_parameter("image_topic", "/ninja/camera/image_raw")
        self.declare_parameter("show_window", False)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/ninja/vision/debug_image")
        self.declare_parameter("debug_image_rate_hz", 5.0)
        self.declare_parameter("debug_image_max_width", 1280)
        self.declare_parameter("process_every_n_frames", 2)
        self.declare_parameter("print_every_n_frames", 30)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.debug_image_rate_hz = float(self.get_parameter("debug_image_rate_hz").value)
        self.debug_image_max_width = int(self.get_parameter("debug_image_max_width").value)
        self.process_every_n_frames = max(1, int(self.get_parameter("process_every_n_frames").value))
        self.print_every_n_frames = max(1, int(self.get_parameter("print_every_n_frames").value))

        self.bridge = CvBridge()
        self.frame_count = 0
        self.processed_count = 0
        self.last_debug_publish_t = 0.0
        self.last_image_t: Optional[float] = None
        self._logged_first_image = False

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = make_aruco_parameters()

        # Camera image topics are usually sensor-data QoS. This avoids reliable/backlog issues.
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.crate_pub = self.create_publisher(String, "/ninja/vision/crate", 10)

        debug_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.debug_pub = (
            self.create_publisher(Image, self.debug_image_topic, debug_qos)
            if self.publish_debug_image
            else None
        )

        self.get_logger().info(
            "CRATE_DETECTOR (safe low-bandwidth) "
            f"image_topic={self.image_topic} "
            f"show_window={self.show_window} "
            f"debug_image={'on' if self.publish_debug_image else 'off'} "
            f"debug_topic={self.debug_image_topic} "
            f"debug_rate={self.debug_image_rate_hz:.2f}Hz "
            f"debug_max_width={self.debug_image_max_width}px "
            f"process_every={self.process_every_n_frames}"
        )
        self.get_logger().info("Dictionary: DICT_4X4_50 only")
        self.get_logger().info("Publishing JSON to /ninja/vision/crate")

    def image_msg_to_bgr(self, msg: Image) -> np.ndarray:
        """Convert ROS Image to BGR while respecting row stride/padding.

        camera_ros on the Pi publishes RGB888/rgb8. Some versions include a
        row step that must be handled exactly; reshaping the raw buffer without
        using msg.step can create the shifted/grey-looking debug image.
        """
        enc = (msg.encoding or "").lower()
        h = int(msg.height)
        w = int(msg.width)
        step = int(msg.step)

        if enc in ("rgb8", "bgr8", "8uc3"):
            needed = h * step
            raw = np.frombuffer(msg.data, dtype=np.uint8, count=needed)
            rows = raw.reshape((h, step))
            packed = rows[:, : w * 3].reshape((h, w, 3)).copy()

            # The camera format in the launch is RGB888. Treat generic 8UC3 as RGB
            # because that is what this camera node normally provides here.
            if enc in ("rgb8", "8uc3"):
                return cv2.cvtColor(packed, cv2.COLOR_RGB2BGR)
            return packed

        if enc == "mono8":
            raw = np.frombuffer(msg.data, dtype=np.uint8, count=h * step)
            rows = raw.reshape((h, step))
            mono = rows[:, :w].copy()
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

        if enc in ("rgba8", "bgra8"):
            raw = np.frombuffer(msg.data, dtype=np.uint8, count=h * step)
            rows = raw.reshape((h, step))
            packed = rows[:, : w * 4].reshape((h, w, 4)).copy()
            code = cv2.COLOR_RGBA2BGR if enc == "rgba8" else cv2.COLOR_BGRA2BGR
            return cv2.cvtColor(packed, code)

        # Fallback for encodings handled by cv_bridge.
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    @staticmethod
    def marker_size_px(corners: np.ndarray) -> float:
        pts = corners.reshape((4, 2))
        sizes = [float(np.linalg.norm(pts[(i + 1) % 4] - pts[i])) for i in range(4)]
        return float(np.mean(sizes))

    @staticmethod
    def marker_angle_deg(corners: np.ndarray) -> float:
        pts = corners.reshape((4, 2))
        dx = pts[1][0] - pts[0][0]
        dy = pts[1][1] - pts[0][1]
        return float(math.degrees(math.atan2(dy, dx)))

    def image_callback(self, msg: Image) -> None:
        self.frame_count += 1
        self.last_image_t = time.monotonic()

        if self.frame_count % self.process_every_n_frames != 0:
            return

        if not self._logged_first_image:
            self._logged_first_image = True
            self.get_logger().info(
                "First camera image: "
                f"encoding={msg.encoding} width={msg.width} height={msg.height} "
                f"step={msg.step} data_len={len(msg.data)}"
            )

        try:
            frame = self.image_msg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f"image conversion failed: {exc}")
            return

        self.processed_count += 1
        self.process_frame(frame)

    def detect(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params,
        )

        detections: List[Dict[str, Any]] = []
        h, w = frame.shape[:2]
        cx_img = w / 2.0

        if ids is None:
            return detections

        for i, marker_id in enumerate(ids.flatten()):
            marker_id = int(marker_id)
            c = corners[i]
            pts = c.reshape((4, 2))
            cx = float(np.mean(pts[:, 0]))
            cy = float(np.mean(pts[:, 1]))
            detections.append(
                {
                    "id": marker_id,
                    "is_crate": marker_id in VALID_CRATE_IDS,
                    "center_x": cx,
                    "center_y": cy,
                    "center_error_px": cx - cx_img,
                    "marker_size_px": self.marker_size_px(c),
                    "raw_angle_deg": self.marker_angle_deg(c),
                    "corners": c,
                }
            )
        return detections

    def make_payload(self, detections: List[Dict[str, Any]], frame_width: int) -> Dict[str, Any]:
        crates = [d for d in detections if d["is_crate"]]

        payload: Dict[str, Any] = {
            "seen": bool(crates),
            "markers": [],
            "all_marker_ids": [int(d["id"]) for d in detections],
            "best": None,
            "pair": None,
            "id": None,
            "center_x": None,
            "center_y": None,
            "center_error_px": None,
            "marker_size_px": None,
            "raw_angle_deg": None,
            "stamp_monotonic": time.monotonic(),
        }

        for d in crates:
            payload["markers"].append(
                {
                    "id": int(d["id"]),
                    "center_x": float(d["center_x"]),
                    "center_y": float(d["center_y"]),
                    "center_error_px": float(d["center_error_px"]),
                    "marker_size_px": float(d["marker_size_px"]),
                    "raw_angle_deg": float(d["raw_angle_deg"]),
                }
            )

        if crates:
            best = max(crates, key=lambda d: d["marker_size_px"])
            best_payload = {
                "id": int(best["id"]),
                "center_x": float(best["center_x"]),
                "center_y": float(best["center_y"]),
                "center_error_px": float(best["center_error_px"]),
                "marker_size_px": float(best["marker_size_px"]),
                "raw_angle_deg": float(best["raw_angle_deg"]),
            }
            payload["best"] = best_payload
            payload.update(best_payload)

        by_id = {int(d["id"]): d for d in crates}
        if BLUE_CRATE_ID in by_id and YELLOW_CRATE_ID in by_id:
            d36 = by_id[BLUE_CRATE_ID]
            d47 = by_id[YELLOW_CRATE_ID]
            mid_x = 0.5 * (float(d36["center_x"]) + float(d47["center_x"]))
            mid_y = 0.5 * (float(d36["center_y"]) + float(d47["center_y"]))
            sep = abs(float(d36["center_x"]) - float(d47["center_x"]))
            avg_size = 0.5 * (float(d36["marker_size_px"]) + float(d47["marker_size_px"]))
            payload["pair"] = {
                "ids": [BLUE_CRATE_ID, YELLOW_CRATE_ID],
                "id36_x": float(d36["center_x"]),
                "id47_x": float(d47["center_x"]),
                "center_x": float(mid_x),
                "center_y": float(mid_y),
                "center_error_px": float(mid_x - frame_width / 2.0),
                "separation_px": float(sep),
                "marker_size_px": float(avg_size),
                "orientation": "forward" if float(d47["center_x"]) < float(d36["center_x"]) else "opposite",
                "seen": True,
            }

        return payload

    def should_publish_debug(self) -> bool:
        if self.debug_pub is None:
            return False
        if self.debug_image_rate_hz <= 0.0:
            return False
        if self.debug_pub.get_subscription_count() <= 0:
            return False
        now = time.monotonic()
        period = 1.0 / self.debug_image_rate_hz
        if now - self.last_debug_publish_t < period:
            return False
        self.last_debug_publish_t = now
        return True

    def draw_debug(self, frame: np.ndarray, detections: List[Dict[str, Any]], payload: Dict[str, Any]) -> np.ndarray:
        img = frame.copy()
        h, w = img.shape[:2]

        img_cx = w // 2
        img_cy = h // 2

        # Old tuning view centre reference, softened so it does not dominate the image.
        # Vertical camera centreline: thin cyan over a subtle dark grey shadow.
        cv2.line(img, (img_cx, 0), (img_cx, h), (70, 70, 70), 2)
        cv2.line(img, (img_cx, 0), (img_cx, h), (180, 220, 220), 1)

        # Keep horizontal image centre very subtle; the vertical line is the important one.
        cv2.line(img, (0, img_cy), (w, img_cy), (45, 45, 45), 1)

        for d in detections:
            color = (0, 255, 0) if d["is_crate"] else (0, 255, 255)
            pts = d["corners"].reshape((4, 2)).astype(int)
            for i in range(4):
                cv2.line(img, tuple(pts[i]), tuple(pts[(i + 1) % 4]), color, 2)

            cx = int(round(d["center_x"]))
            cy = int(round(d["center_y"]))
            err = float(d["center_error_px"])
            size = float(d["marker_size_px"])
            angle = float(d["raw_angle_deg"])

            # Old-style visual centre offset: blue line from tag centre to the camera centreline.
            # The horizontal segment is the actual pixel error used by alignment.
            cv2.line(img, (img_cx, cy), (cx, cy), (255, 0, 0), 2)
            cv2.circle(img, (cx, cy), 6, (0, 0, 255), -1)
            cv2.circle(img, (cx, cy), 8, (255, 0, 0), 1)

            cv2.putText(
                img,
                f"id={d['id']} dx={err:+.1f}px sz={size:.1f}px ang={angle:+.1f}deg",
                (cx + 8, max(20, cy - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                color,
                2,
            )
            cv2.putText(
                img,
                f"tag_center=({d['center_x']:.1f},{d['center_y']:.1f})",
                (cx + 8, min(h - 10, cy + 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                color,
                1,
            )

        if payload.get("best") is not None:
            b = payload["best"]
            label1 = (
                f"BEST id={b['id']}  dx={b['center_error_px']:+.1f}px  "
                f"size={b['marker_size_px']:.1f}px  angle={b['raw_angle_deg']:+.1f}deg"
            )
            label2 = f"image_center=({img_cx},{img_cy})  tag_center=({b['center_x']:.1f},{b['center_y']:.1f})"
            cv2.putText(img, label1, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 0, 0), 2)
            cv2.putText(img, label2, (8, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 0, 0), 2)
        else:
            label = f"NO CRATE | ids={payload.get('all_marker_ids', [])}"
            cv2.putText(img, label, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

        if self.debug_image_max_width > 0 and img.shape[1] > self.debug_image_max_width:
            scale = self.debug_image_max_width / float(img.shape[1])
            new_w = int(round(img.shape[1] * scale))
            new_h = int(round(img.shape[0] * scale))
            img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

        return img

    def publish_debug(self, frame: np.ndarray, detections: List[Dict[str, Any]], payload: Dict[str, Any]) -> None:
        if not self.should_publish_debug() and not self.show_window:
            return

        debug_img = self.draw_debug(frame, detections, payload)

        if self.debug_pub is not None and self.debug_pub.get_subscription_count() > 0:
            try:
                out_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
                self.debug_pub.publish(out_msg)
            except Exception as exc:
                self.get_logger().warn(f"debug publish failed: {exc}")

        if self.show_window:
            cv2.imshow("Ninja Crate Detector", debug_img)
            cv2.waitKey(1)

    def process_frame(self, frame: np.ndarray) -> None:
        h, w = frame.shape[:2]
        detections = self.detect(frame)
        payload = self.make_payload(detections, w)

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.crate_pub.publish(msg)

        self.publish_debug(frame, detections, payload)

        if self.processed_count % self.print_every_n_frames == 0:
            best = payload.get("best")
            debug_subs = self.debug_pub.get_subscription_count() if self.debug_pub is not None else 0
            if best:
                self.get_logger().info(
                    f"processed={self.processed_count} best id={best['id']} "
                    f"err={best['center_error_px']:+.1f} size={best['marker_size_px']:.1f} "
                    f"debug_subs={debug_subs}"
                )
            else:
                self.get_logger().info(
                    f"processed={self.processed_count} no crate "
                    f"all_ids={payload.get('all_marker_ids', [])} debug_subs={debug_subs}"
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CrateDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
