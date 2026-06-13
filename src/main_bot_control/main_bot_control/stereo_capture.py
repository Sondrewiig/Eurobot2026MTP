#!/usr/bin/env python3

import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class StereoCapture(Node):
    def __init__(self):
        super().__init__("stereo_capture")

        self.declare_parameter("image_topic", "/zed/image_raw")
        self.declare_parameter("output_dir", str(Path.home() / "Eurobot2026MTP" / "stereo_pairs"))
        self.declare_parameter("board_cols", 7)   # inner corners
        self.declare_parameter("board_rows", 10)   # inner corners
        self.declare_parameter("preview_scale", 0.8)

        self.image_topic = self.get_parameter("image_topic").value
        self.output_dir = Path(self.get_parameter("output_dir").value).expanduser()
        self.board_cols = int(self.get_parameter("board_cols").value)
        self.board_rows = int(self.get_parameter("board_rows").value)
        self.preview_scale = float(self.get_parameter("preview_scale").value)

        self.pattern_size = (self.board_cols, self.board_rows)

        self.left_dir = self.output_dir / "left"
        self.right_dir = self.output_dir / "right"
        self.left_dir.mkdir(parents=True, exist_ok=True)
        self.right_dir.mkdir(parents=True, exist_ok=True)

        self.latest_left = None
        self.latest_right = None
        self.latest_left_vis = None
        self.latest_right_vis = None
        self.latest_left_found = False
        self.latest_right_found = False
        self.latest_stamp_ns = None

        self.save_index = self.find_next_index()

        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.write_manifest()

        self.get_logger().info(f"Stereo capture listening on {self.image_topic}")
        self.get_logger().info(f"Saving pairs to {self.output_dir}")
        self.get_logger().info(
            f"Checkerboard pattern: {self.board_cols}x{self.board_rows} inner corners"
        )
        self.get_logger().info("Controls: s=save pair, q=quit")

    def write_manifest(self):
        manifest = {
            "image_topic": self.image_topic,
            "board_cols_inner": self.board_cols,
            "board_rows_inner": self.board_rows,
        }
        with open(self.output_dir / "manifest.json", "w") as f:
            json.dump(manifest, f, indent=2)

    def find_next_index(self):
        existing = sorted(self.left_dir.glob("left_*.png"))
        if not existing:
            return 0
        last = existing[-1].stem.split("_")[-1]
        try:
            return int(last) + 1
        except ValueError:
            return 0

    def decode_bgr(self, msg: Image):
        h = msg.height
        w = msg.width
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        enc = msg.encoding.lower()

        if enc == "rgb8":
            row_pixels = msg.step // 3
            rgb = buf.reshape((h, row_pixels, 3))[:, :w, :].copy()
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if enc == "bgr8":
            row_pixels = msg.step // 3
            return buf.reshape((h, row_pixels, 3))[:, :w, :].copy()

        if enc in ("yuv422", "yuv422_yuy2", "yuyv"):
            row_pixels = msg.step // 2
            yuyv = buf.reshape((h, row_pixels, 2))[:, :w, :].copy()
            return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)

        self.get_logger().warning(f"Unsupported encoding: {msg.encoding}")
        return None

    def split_side_by_side(self, bgr):
        h, w = bgr.shape[:2]
        if w % 2 != 0:
            self.get_logger().warning(f"Expected even width for side-by-side image, got {w}")
            return None, None
        half = w // 2
        left = bgr[:, :half, :].copy()
        right = bgr[:, half:, :].copy()
        return left, right

    def detect_checkerboard(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        found = False
        corners = None

        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(gray, self.pattern_size, None)
        else:
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            found, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags)
            if found:
                term = (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001,
                )
                cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)

        vis = bgr.copy()
        cv2.drawChessboardCorners(vis, self.pattern_size, corners, found)
        return found, corners, vis

    def image_callback(self, msg: Image):
        bgr = self.decode_bgr(msg)
        if bgr is None:
            return

        left, right = self.split_side_by_side(bgr)
        if left is None or right is None:
            return

        left_found, _, left_vis = self.detect_checkerboard(left)
        right_found, _, right_vis = self.detect_checkerboard(right)

        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

        self.latest_left = left
        self.latest_right = right
        self.latest_left_vis = left_vis
        self.latest_right_vis = right_vis
        self.latest_left_found = left_found
        self.latest_right_found = right_found
        self.latest_stamp_ns = stamp_ns

    def save_current_pair(self):
        if self.latest_left is None or self.latest_right is None:
            self.get_logger().warning("No image received yet")
            return

        if not (self.latest_left_found and self.latest_right_found):
            self.get_logger().warning("Checkerboard not found in both cameras, not saving")
            return

        idx = self.save_index
        left_path = self.left_dir / f"left_{idx:04d}.png"
        right_path = self.right_dir / f"right_{idx:04d}.png"

        cv2.imwrite(str(left_path), self.latest_left)
        cv2.imwrite(str(right_path), self.latest_right)

        self.get_logger().info(f"Saved stereo pair {idx:04d}")
        self.save_index += 1

    def make_preview(self):
        if self.latest_left_vis is None or self.latest_right_vis is None:
            blank = np.zeros((360, 960, 3), dtype=np.uint8)
            cv2.putText(
                blank,
                "Waiting for /zed/image_raw ...",
                (30, 180),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            return blank

        left = self.latest_left_vis.copy()
        right = self.latest_right_vis.copy()

        left_status = "FOUND" if self.latest_left_found else "MISS"
        right_status = "FOUND" if self.latest_right_found else "MISS"
        can_save = self.latest_left_found and self.latest_right_found

        cv2.putText(left, f"LEFT: {left_status}", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 255, 0) if self.latest_left_found else (0, 0, 255), 2, cv2.LINE_AA)

        cv2.putText(right, f"RIGHT: {right_status}", (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (0, 255, 0) if self.latest_right_found else (0, 0, 255), 2, cv2.LINE_AA)

        preview = np.hstack([left, right])

        status_text = f"s=save | q=quit | next={self.save_index:04d} | save_ready={'YES' if can_save else 'NO'}"
        cv2.putText(preview, status_text, (20, preview.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 0), 2, cv2.LINE_AA)

        if self.preview_scale != 1.0:
            preview = cv2.resize(
                preview,
                None,
                fx=self.preview_scale,
                fy=self.preview_scale,
                interpolation=cv2.INTER_AREA,
            )

        return preview

    def run(self):
        cv2.namedWindow("stereo_capture", cv2.WINDOW_NORMAL)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            preview = self.make_preview()
            cv2.imshow("stereo_capture", preview)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                self.save_current_pair()

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = StereoCapture()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()