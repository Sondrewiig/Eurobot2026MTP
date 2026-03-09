import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32MultiArray


ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
}


class ArucoDetect(Node):
    def __init__(self):
        super().__init__("aruco_detect")

        self.declare_parameter("image_topic", "/camera/left/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/left/camera_info")
        self.declare_parameter("dictionary", "DICT_4X4_100")
        self.declare_parameter("tag_size", 0.10)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        dict_name = self.get_parameter("dictionary").value
        self.tag_size = float(self.get_parameter("tag_size").value)

        if dict_name not in ARUCO_DICTS:
            raise ValueError(f"Unsupported dictionary: {dict_name}")

        self.dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])

        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.detector_params = cv2.aruco.DetectorParameters_create()
        else:
            self.detector_params = cv2.aruco.DetectorParameters()

        self.camera_matrix = None
        self.dist_coeffs = None

        self.ids_pub = self.create_publisher(Int32MultiArray, "/aruco_ids", 10)

        self.last_ids = None
        self.last_log_time_ns = 0

        self.info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )

        self.get_logger().info(
            f"aruco_detect started | dict={dict_name} | tag_size={self.tag_size:.3f} m"
        )

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))

            d = np.array(msg.d, dtype=np.float64)
            if d.size == 0:
                d = np.zeros((5, 1), dtype=np.float64)
            self.dist_coeffs = d.reshape(-1, 1)

            self.get_logger().info("Camera intrinsics received")

    def image_callback(self, msg: Image):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.detector_params
            )

            if ids is None or len(ids) == 0:
                return

            ids_flat = ids.flatten().astype(int).tolist()

            out = Int32MultiArray()
            out.data = ids_flat
            self.ids_pub.publish(out)

            now_ns = self.get_clock().now().nanoseconds

            # marker object points in marker frame
            # OpenCV returns corners in order:
            # top-left, top-right, bottom-right, bottom-left
            s = self.tag_size / 2.0
            obj_pts = np.array([
                [-s,  s, 0.0],   # top-left
                [ s,  s, 0.0],   # top-right
                [ s, -s, 0.0],   # bottom-right
                [-s, -s, 0.0],   # bottom-left
            ], dtype=np.float32)

            if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
                pnp_flag = cv2.SOLVEPNP_IPPE_SQUARE
            else:
                pnp_flag = cv2.SOLVEPNP_ITERATIVE

            for marker_corners, marker_id in zip(corners, ids_flat):
                img_pts = marker_corners[0].astype(np.float32)

                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts,
                    img_pts,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=pnp_flag
                )

                if not ok:
                    continue

                tvec = tvec.reshape(3)

                # Distance from camera to marker center
                dist = float(np.linalg.norm(tvec))

                # Horizontal bearing in camera frame
                # +bearing = marker is to the right of camera centerline
                bearing_deg = math.degrees(math.atan2(float(tvec[0]), float(tvec[2])))

                # Log at most ~3 times per second unless IDs changed
                if (ids_flat != self.last_ids) or (now_ns - self.last_log_time_ns > 300_000_000):
                    self.get_logger().info(
                        f"id={marker_id} | dist={dist:.3f} m | bearing={bearing_deg:.1f} deg"
                    )

            if (ids_flat != self.last_ids) or (now_ns - self.last_log_time_ns > 300_000_000):
                self.last_ids = ids_flat
                self.last_log_time_ns = now_ns

        except Exception as e:
            self.get_logger().error(f"Aruco detection failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetect()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()