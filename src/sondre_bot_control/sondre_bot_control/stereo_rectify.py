#!/usr/bin/env python3

from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image


class StereoRectify(Node):
    def __init__(self):
        super().__init__("stereo_rectify")

        self.declare_parameter("left_image_topic", "/camera/left/image_raw")
        self.declare_parameter("right_image_topic", "/camera/right/image_raw")
        self.declare_parameter("left_rect_topic", "/camera/left/image_rect")
        self.declare_parameter("right_rect_topic", "/camera/right/image_rect")
        self.declare_parameter(
            "stereo_yaml",
            str(Path.home() / "Eurobot2026MTP" / "config" / "zed_stereo.yaml"),
        )
        self.declare_parameter("queue_size", 5)

        self.left_image_topic = self.get_parameter("left_image_topic").value
        self.right_image_topic = self.get_parameter("right_image_topic").value
        self.left_rect_topic = self.get_parameter("left_rect_topic").value
        self.right_rect_topic = self.get_parameter("right_rect_topic").value
        self.stereo_yaml = self.get_parameter("stereo_yaml").value
        self.queue_size = int(self.get_parameter("queue_size").value)

        self.left_pub = self.create_publisher(Image, self.left_rect_topic, 10)
        self.right_pub = self.create_publisher(Image, self.right_rect_topic, 10)

        self.left_map1 = None
        self.left_map2 = None
        self.right_map1 = None
        self.right_map2 = None
        self.image_width = None
        self.image_height = None

        self.latest_left = None
        self.latest_right = None
        self.latest_left_stamp_ns = None
        self.latest_right_stamp_ns = None
        self.last_processed_left_stamp_ns = None
        self.last_processed_right_stamp_ns = None

        self.load_stereo_calibration(self.stereo_yaml)

        self.create_subscription(
            Image,
            self.left_image_topic,
            self.left_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.right_image_topic,
            self.right_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info(f"stereo_rectify started")
        self.get_logger().info(f"left raw:  {self.left_image_topic}")
        self.get_logger().info(f"right raw: {self.right_image_topic}")
        self.get_logger().info(f"left rect: {self.left_rect_topic}")
        self.get_logger().info(f"right rect:{self.right_rect_topic}")

    def load_stereo_calibration(self, yaml_path: str):
        fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise RuntimeError(f"Failed to open stereo calibration file: {yaml_path}")

        self.image_width = int(fs.getNode("image_width").real())
        self.image_height = int(fs.getNode("image_height").real())

        K1 = fs.getNode("K1").mat()
        D1 = fs.getNode("D1").mat()
        K2 = fs.getNode("K2").mat()
        D2 = fs.getNode("D2").mat()
        R1 = fs.getNode("R1").mat()
        R2 = fs.getNode("R2").mat()
        P1 = fs.getNode("P1").mat()
        P2 = fs.getNode("P2").mat()
        fs.release()

        if any(x is None for x in [K1, D1, K2, D2, R1, R2, P1, P2]):
            raise RuntimeError(f"Missing required matrices in stereo calibration file: {yaml_path}")

        image_size = (self.image_width, self.image_height)

        self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
            K1, D1, R1, P1, image_size, cv2.CV_16SC2
        )
        self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
            K2, D2, R2, P2, image_size, cv2.CV_16SC2
        )

        self.get_logger().info(
            f"Loaded stereo calibration from {yaml_path} for image size {image_size}"
        )

    def stamp_to_ns(self, msg: Image):
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def decode_image(self, msg: Image):
        h = msg.height
        w = msg.width
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        enc = msg.encoding.lower()

        if enc == "rgb8":
            row_pixels = msg.step // 3
            rgb = buf.reshape((h, row_pixels, 3))[:, :w, :].copy()
            return rgb, "rgb8"

        if enc == "bgr8":
            row_pixels = msg.step // 3
            bgr = buf.reshape((h, row_pixels, 3))[:, :w, :].copy()
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            return rgb, "rgb8"

        if enc in ("yuv422", "yuv422_yuy2", "yuyv"):
            row_pixels = msg.step // 2
            yuyv = buf.reshape((h, row_pixels, 2))[:, :w, :].copy()
            rgb = cv2.cvtColor(yuyv, cv2.COLOR_YUV2RGB_YUY2)
            return rgb, "rgb8"

        self.get_logger().warning(f"Unsupported encoding: {msg.encoding}")
        return None, None

    def encode_rgb8(self, rgb: np.ndarray, header):
        out = Image()
        out.header = header
        out.height = rgb.shape[0]
        out.width = rgb.shape[1]
        out.encoding = "rgb8"
        out.is_bigendian = False
        out.step = rgb.shape[1] * 3
        out.data = rgb.tobytes()
        return out

    def rectify_rgb(self, rgb: np.ndarray, side: str):
        if side == "left":
            return cv2.remap(
                rgb,
                self.left_map1,
                self.left_map2,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )
        return cv2.remap(
            rgb,
            self.right_map1,
            self.right_map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

    def try_process_pair(self):
        if self.latest_left is None or self.latest_right is None:
            return

        if self.latest_left_stamp_ns == self.last_processed_left_stamp_ns and \
           self.latest_right_stamp_ns == self.last_processed_right_stamp_ns:
            return

        left_rgb, _ = self.decode_image(self.latest_left)
        right_rgb, _ = self.decode_image(self.latest_right)

        if left_rgb is None or right_rgb is None:
            return

        if left_rgb.shape[1] != self.image_width or left_rgb.shape[0] != self.image_height:
            self.get_logger().warning(
                f"Left image size {left_rgb.shape[1]}x{left_rgb.shape[0]} "
                f"does not match calibration size {self.image_width}x{self.image_height}"
            )
            return

        if right_rgb.shape[1] != self.image_width or right_rgb.shape[0] != self.image_height:
            self.get_logger().warning(
                f"Right image size {right_rgb.shape[1]}x{right_rgb.shape[0]} "
                f"does not match calibration size {self.image_width}x{self.image_height}"
            )
            return

        left_rect = self.rectify_rgb(left_rgb, "left")
        right_rect = self.rectify_rgb(right_rgb, "right")

        left_msg = self.encode_rgb8(left_rect, self.latest_left.header)
        right_msg = self.encode_rgb8(right_rect, self.latest_right.header)

        left_msg.header.frame_id = "left_camera_rect"
        right_msg.header.frame_id = "right_camera_rect"

        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

        self.last_processed_left_stamp_ns = self.latest_left_stamp_ns
        self.last_processed_right_stamp_ns = self.latest_right_stamp_ns

    def left_cb(self, msg: Image):
        self.latest_left = msg
        self.latest_left_stamp_ns = self.stamp_to_ns(msg)
        self.try_process_pair()

    def right_cb(self, msg: Image):
        self.latest_right = msg
        self.latest_right_stamp_ns = self.stamp_to_ns(msg)
        self.try_process_pair()


def main(args=None):
    rclpy.init(args=args)
    node = StereoRectify()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()