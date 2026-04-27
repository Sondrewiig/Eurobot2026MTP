import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo


class ZedStereoSplitter(Node):
    def __init__(self):
        super().__init__("zed_stereo_splitter")

        self.declare_parameter("image_in", "/zed/image_raw")
        self.declare_parameter("camera_info_in", "/zed/camera_info")

        self.declare_parameter("left_image_out", "/camera/left/image_raw")
        self.declare_parameter("right_image_out", "/camera/right/image_raw")
        self.declare_parameter("left_camera_info_out", "/camera/left/camera_info")
        self.declare_parameter("right_camera_info_out", "/camera/right/camera_info")

        self.declare_parameter("left_frame_id", "left_camera")
        self.declare_parameter("right_frame_id", "right_camera")

        self.declare_parameter(
            "left_calibration_yaml",
            os.path.expanduser("~/Eurobot2026MTP/config/zed_left_camera.yaml"),
        )
        self.declare_parameter(
            "right_calibration_yaml",
            os.path.expanduser("~/Eurobot2026MTP/config/zed_right_camera.yaml"),
        )

        self.image_in = self.get_parameter("image_in").value
        self.camera_info_in = self.get_parameter("camera_info_in").value

        self.left_image_out = self.get_parameter("left_image_out").value
        self.right_image_out = self.get_parameter("right_image_out").value
        self.left_camera_info_out = self.get_parameter("left_camera_info_out").value
        self.right_camera_info_out = self.get_parameter("right_camera_info_out").value

        self.left_frame_id = self.get_parameter("left_frame_id").value
        self.right_frame_id = self.get_parameter("right_frame_id").value

        self.left_calibration_yaml = self.get_parameter("left_calibration_yaml").value
        self.right_calibration_yaml = self.get_parameter("right_calibration_yaml").value

        self.left_image_pub = self.create_publisher(Image, self.left_image_out, 10)
        self.right_image_pub = self.create_publisher(Image, self.right_image_out, 10)
        self.left_info_pub = self.create_publisher(CameraInfo, self.left_camera_info_out, 10)
        self.right_info_pub = self.create_publisher(CameraInfo, self.right_camera_info_out, 10)

        self.left_calib = self.load_calibration(self.left_calibration_yaml)
        self.right_calib = self.load_calibration(self.right_calibration_yaml)

        if self.left_calib is not None:
            self.get_logger().info(f"Loaded left calibration YAML: {self.left_calibration_yaml}")
        else:
            self.get_logger().warning(f"No left calibration YAML loaded from {self.left_calibration_yaml}")

        if self.right_calib is not None:
            self.get_logger().info(f"Loaded right calibration YAML: {self.right_calibration_yaml}")
        else:
            self.get_logger().warning(f"No right calibration YAML loaded from {self.right_calibration_yaml}")

        self.create_subscription(
            CameraInfo,
            self.camera_info_in,
            self.camera_info_cb,
            10,
        )
        self.create_subscription(
            Image,
            self.image_in,
            self.image_cb,
            qos_profile_sensor_data,
        )

    def load_calibration(self, path):
        if not os.path.exists(path):
            return None

        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            self.get_logger().warning(f"Failed to open calibration file: {path}")
            return None

        try:
            width_node = fs.getNode("image_width")
            height_node = fs.getNode("image_height")
            distortion_model_node = fs.getNode("distortion_model")
            camera_matrix_node = fs.getNode("camera_matrix")
            distortion_coeffs_node = fs.getNode("distortion_coefficients")
            rectification_matrix_node = fs.getNode("rectification_matrix")
            projection_matrix_node = fs.getNode("projection_matrix")

            width = int(width_node.real()) if not width_node.empty() else 0
            height = int(height_node.real()) if not height_node.empty() else 0
            distortion_model = (
                distortion_model_node.string()
                if not distortion_model_node.empty()
                else "plumb_bob"
            )

            k = camera_matrix_node.mat()
            d = distortion_coeffs_node.mat()
            r = rectification_matrix_node.mat()
            p = projection_matrix_node.mat()

            if k is None or d is None or r is None or p is None:
                self.get_logger().warning(f"Incomplete calibration data in {path}")
                return None

            calib = {
                "width": width,
                "height": height,
                "distortion_model": distortion_model,
                "d": d.reshape(-1).astype(float).tolist(),
                "k": k.reshape(-1).astype(float).tolist(),
                "r": r.reshape(-1).astype(float).tolist(),
                "p": p.reshape(-1).astype(float).tolist(),
            }
            return calib
        finally:
            fs.release()

    def build_camera_info(self, header, width, height, frame_id, calib):
        info = CameraInfo()
        info.header = header
        info.header.frame_id = frame_id
        info.width = width
        info.height = height

        if calib is not None:
            info.width = calib["width"]
            info.height = calib["height"]
            info.distortion_model = calib["distortion_model"]
            info.d = calib["d"]
            info.k = calib["k"]
            info.r = calib["r"]
            info.p = calib["p"]
        else:
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
            info.k = [0.0] * 9
            info.r = [0.0] * 9
            info.p = [0.0] * 12

        return info

    def camera_info_cb(self, _msg: CameraInfo):
        pass

    def image_cb(self, msg: Image):
        if msg.encoding.lower() not in ("rgb8", "bgr8"):
            self.get_logger().warning(f"Unsupported encoding for splitter: {msg.encoding}")
            return

        channels = 3
        row_pixels = msg.step // channels
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, row_pixels, channels))[:, :msg.width, :]

        if msg.width % 2 != 0:
            self.get_logger().warning(f"Expected even image width, got {msg.width}")
            return

        half_w = msg.width // 2
        left = img[:, :half_w, :].copy()
        right = img[:, half_w:, :].copy()

        left_msg = Image()
        left_msg.header = msg.header
        left_msg.header.frame_id = self.left_frame_id
        left_msg.height = left.shape[0]
        left_msg.width = left.shape[1]
        left_msg.encoding = msg.encoding
        left_msg.is_bigendian = msg.is_bigendian
        left_msg.step = left.shape[1] * channels
        left_msg.data = left.tobytes()
        self.left_image_pub.publish(left_msg)

        right_msg = Image()
        right_msg.header = msg.header
        right_msg.header.frame_id = self.right_frame_id
        right_msg.height = right.shape[0]
        right_msg.width = right.shape[1]
        right_msg.encoding = msg.encoding
        right_msg.is_bigendian = msg.is_bigendian
        right_msg.step = right.shape[1] * channels
        right_msg.data = right.tobytes()
        self.right_image_pub.publish(right_msg)

        left_info = self.build_camera_info(
            left_msg.header, left_msg.width, left_msg.height, self.left_frame_id, self.left_calib
        )
        right_info = self.build_camera_info(
            right_msg.header, right_msg.width, right_msg.height, self.right_frame_id, self.right_calib
        )

        self.left_info_pub.publish(left_info)
        self.right_info_pub.publish(right_info)


def main(args=None):
    rclpy.init(args=args)
    node = ZedStereoSplitter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()