import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D


ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
}


def rotz(yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)


def roty(pitch: float) -> np.ndarray:
    c = math.cos(pitch)
    s = math.sin(pitch)
    return np.array([
        [ c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c]
    ], dtype=np.float64)


def rotx(roll: float) -> np.ndarray:
    c = math.cos(roll)
    s = math.sin(roll)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0,  c, -s],
        [0.0,  s,  c]
    ], dtype=np.float64)


def make_T(x: float, y: float, z: float,
           roll: float, pitch: float, yaw: float) -> np.ndarray:
    R = rotz(yaw) @ roty(pitch) @ rotx(roll)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def invert_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


class TagLocalization(Node):
    def __init__(self):
        super().__init__("tag_localization")

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

        # Known field markers in official field frame
        # origin = bottom-left on yellow side
        # x,y in meters
        # yaw is tag orientation in field frame
        self.marker_map = {
            20: {"x": 0.600, "y": 1.400, "yaw": 0},
        }

        # base -> camera_link
        T_base_camera_link = make_T(
            x=0.22,
            y=0.00,
            z=0.28,
            roll=0.0,
            pitch=-math.pi / 4.0,   # 45 deg downward tilt
            yaw=0.0,
        )

        # camera_link -> left_camera sensor
        T_camera_link_left_camera = make_T(
            x=0.0,
            y=0.06,
            z=0.0,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )

        # left_camera -> optical frame
        # matches the optical-frame rotation commonly used for image geometry
        T_left_camera_optical = make_T(
            x=0.0,
            y=0.0,
            z=0.0,
            roll=-math.pi / 2.0,
            pitch=0.0,
            yaw=-math.pi / 2.0,
        )

        self.T_base_camera = (
            T_base_camera_link
            @ T_camera_link_left_camera
            @ T_left_camera_optical
        )

        self.pose_pub = self.create_publisher(Pose2D, "/bot_pose_estimate", 10)

        self.info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10
        )
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )

        self.last_log_time_ns = 0

        self.get_logger().info(
            f"tag_localization started | dict={dict_name} | tag_size={self.tag_size:.3f} m"
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
                if marker_id not in self.marker_map:
                    continue

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

                R_camera_marker, _ = cv2.Rodrigues(rvec)

                T_camera_marker = np.eye(4, dtype=np.float64)
                T_camera_marker[:3, :3] = R_camera_marker
                T_camera_marker[:3, 3] = tvec.reshape(3)

                marker = self.marker_map[marker_id]
                T_field_marker = make_T(
                    x=marker["x"],
                    y=marker["y"],
                    z=0.0,
                    roll=0.0,
                    pitch=0.0,
                    yaw=marker["yaw"],
                )

                # field->base = field->marker * inverse(camera->marker) * inverse(base->camera)
                T_field_base = T_field_marker @ invert_T(T_camera_marker) @ invert_T(self.T_base_camera)

                x = float(T_field_base[0, 3])
                y = float(T_field_base[1, 3])
                yaw = math.atan2(T_field_base[1, 0], T_field_base[0, 0])

                pose_msg = Pose2D()
                pose_msg.x = x
                pose_msg.y = y
                pose_msg.theta = yaw
                self.pose_pub.publish(pose_msg)

                now_ns = self.get_clock().now().nanoseconds
                if now_ns - self.last_log_time_ns > 300_000_000:
                    self.get_logger().info(
                        f"id={marker_id} | bot_pose: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg"
                    )
                    self.last_log_time_ns = now_ns

                break

        except Exception as e:
            self.get_logger().error(f"Localization failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TagLocalization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()