#!/usr/bin/env python3
"""
Local Ninja Pi debug image viewer.

Subscribes to /ninja/vision/debug_image using BEST_EFFORT QoS and opens
an enlarged OpenCV window. Designed for local viewing on the Pi, not for
streaming over Tailscale.

Examples:
  python3 tools/view_ninja_debug.py
  python3 tools/view_ninja_debug.py --scale 2.5
  python3 tools/view_ninja_debug.py --width 1100
  python3 tools/view_ninja_debug.py --rotate cw --scale 2
  python3 tools/view_ninja_debug.py --fullscreen
"""

import argparse
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image


class NinjaDebugViewer(Node):
    def __init__(self, args):
        super().__init__("ninja_debug_local_viewer")
        self.args = args
        self.bridge = CvBridge()
        self.window_name = args.window_name
        self.frame_count = 0

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.sub = self.create_subscription(
            Image,
            args.topic,
            self._on_image,
            qos,
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        if args.fullscreen:
            cv2.setWindowProperty(
                self.window_name,
                cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN,
            )
        else:
            cv2.resizeWindow(self.window_name, args.initial_window_width, args.initial_window_height)

        self.get_logger().info(f"Viewing {args.topic} with BEST_EFFORT QoS")
        self.get_logger().info("Press q or ESC in the image window, or Ctrl+C in terminal, to quit")

    def _rotate(self, frame):
        if self.args.rotate == "cw":
            return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if self.args.rotate == "ccw":
            return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if self.args.rotate == "180":
            return cv2.rotate(frame, cv2.ROTATE_180)
        return frame

    def _scale(self, frame):
        h, w = frame.shape[:2]

        if self.args.width is not None:
            new_w = max(1, int(self.args.width))
            new_h = max(1, int(round(h * (new_w / float(w)))))
            return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        if self.args.height is not None:
            new_h = max(1, int(self.args.height))
            new_w = max(1, int(round(w * (new_h / float(h)))))
            return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        scale = max(0.1, float(self.args.scale))
        if abs(scale - 1.0) < 1e-6:
            return frame
        return cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            return

        frame = self._rotate(frame)
        frame = self._scale(frame)

        if self.args.show_info:
            self.frame_count += 1
            label = f"{msg.width}x{msg.height} -> {frame.shape[1]}x{frame.shape[0]}"
            cv2.putText(
                frame,
                label,
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            rclpy.shutdown()


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Large local viewer for Ninja debug camera image")
    parser.add_argument("--topic", default="/ninja/vision/debug_image")
    parser.add_argument("--scale", type=float, default=2.0, help="Image scale factor if --width/--height are not set")
    parser.add_argument("--width", type=int, default=None, help="Displayed image width, preserving aspect ratio")
    parser.add_argument("--height", type=int, default=None, help="Displayed image height, preserving aspect ratio")
    parser.add_argument("--rotate", choices=["none", "cw", "ccw", "180"], default="none")
    parser.add_argument("--fullscreen", action="store_true")
    parser.add_argument("--window-name", default="Ninja Pi debug camera")
    parser.add_argument("--initial-window-width", type=int, default=1100)
    parser.add_argument("--initial-window-height", type=int, default=800)
    parser.add_argument("--no-info", dest="show_info", action="store_false", help="Hide size overlay text")
    parser.set_defaults(show_info=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = NinjaDebugViewer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
