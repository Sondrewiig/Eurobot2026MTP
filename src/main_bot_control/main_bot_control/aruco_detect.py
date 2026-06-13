import json

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32MultiArray, String

from main_bot_control.tag_registry import default_tags_yaml, load_tag_registry


ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
}


class ArucoDetect(Node):
    def __init__(self):
        super().__init__("aruco_detect")

        self.declare_parameter("image_topic", "/zed/image_raw")
        self.declare_parameter("camera_info_topic", "/zed/camera_info")
        self.declare_parameter("dictionary", "DICT_4X4_100")
        self.declare_parameter("tag_size", 0.10)
        self.declare_parameter("debug_image_topic", "/aruco/debug_image")
        self.declare_parameter("detections_topic", "/aruco/detections_json")
        self.declare_parameter("use_left_half_only", False)
        self.declare_parameter("publish_debug_every_n", 3)
        self.declare_parameter("debug_max_width", 640)
        self.declare_parameter("log_period_sec", 0.5)
        self.declare_parameter("tags_yaml", default_tags_yaml())
        self.declare_parameter("detect_scale", 0.5)
        self.detect_scale = float(self.get_parameter("detect_scale").value)

        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.dict_name = self.get_parameter("dictionary").value
        self.tag_size = float(self.get_parameter("tag_size").value)
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.use_left_half_only = bool(self.get_parameter("use_left_half_only").value)
        self.publish_debug_every_n = int(self.get_parameter("publish_debug_every_n").value)
        self.debug_max_width = int(self.get_parameter("debug_max_width").value)
        self.log_period_ns = int(float(self.get_parameter("log_period_sec").value) * 1e9)
        tags_yaml = self.get_parameter("tags_yaml").value

        if self.dict_name not in ARUCO_DICTS:
            raise ValueError(f"Unsupported dictionary: {self.dict_name}")

        self.dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[self.dict_name])

        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.detector_params = cv2.aruco.DetectorParameters_create()
        else:
            self.detector_params = cv2.aruco.DetectorParameters()

        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.detector_params)
        else:
            self.detector = None

        self.camera_matrix = None
        self.dist_coeffs = None
        self.have_valid_intrinsics = False

        self.tag_registry = {}
        self.brick_tag_to_color = {}
        self.marker_names = {}

        try:
            self.tag_registry = load_tag_registry(tags_yaml)
            for tag_id, info in self.tag_registry.items():
                tag_id = int(tag_id)
                self.marker_names[tag_id] = info.get("name", f"id_{tag_id}")

                if info.get("role") == "brick":
                    color = str(info.get("brick_color", "")).upper()
                    if color:
                        self.brick_tag_to_color[tag_id] = color

            self.get_logger().info(f"Loaded {len(self.tag_registry)} tags from {tags_yaml}")
        except Exception as e:
            self.get_logger().warning(f"Failed to load tags.yaml from {tags_yaml}: {e}")

        self.ids_pub = self.create_publisher(Int32MultiArray, "/aruco_ids", 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)

        self.frame_counter = 0
        self.last_log_time_ns = 0
        self.last_ids = None

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )
        self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"aruco_detect started | dict={self.dict_name} | tag_size={self.tag_size:.3f} m"
        )
        self.get_logger().info(f"image_topic={self.image_topic}")
        self.get_logger().info(f"camera_info_topic={self.camera_info_topic}")
        self.get_logger().info(f"detections_topic={self.detections_topic}")

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is not None:
            return

        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))

        d = np.array(msg.d, dtype=np.float64)
        if d.size == 0:
            d = np.zeros((5, 1), dtype=np.float64)
        self.dist_coeffs = d.reshape(-1, 1)

        self.have_valid_intrinsics = not np.allclose(self.camera_matrix, 0.0)

        if self.have_valid_intrinsics:
            self.get_logger().info("Valid camera intrinsics received")
        else:
            self.get_logger().warning(
                "Camera intrinsics are all zeros. IDs/debug will work, but absolute pose will not."
            )

    def detect_markers(self, gray):
        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.detector_params,
            )
        return corners, ids, rejected

    def decode_rgb_and_gray(self, msg: Image):
        h = msg.height
        w = msg.width
        buf = np.frombuffer(msg.data, dtype=np.uint8)

        enc = msg.encoding.lower()

        if enc == "rgb8":
            row_pixels = msg.step // 3
            rgb = buf.reshape((h, row_pixels, 3))[:, :w, :].copy()
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            return rgb, gray

        if enc == "bgr8":
            row_pixels = msg.step // 3
            bgr = buf.reshape((h, row_pixels, 3))[:, :w, :].copy()
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            return rgb, gray

        if enc in ("yuv422", "yuv422_yuy2", "yuyv"):
            row_pixels = msg.step // 2
            yuyv = buf.reshape((h, row_pixels, 2))[:, :w, :].copy()
            gray = yuyv[:, :, 0].copy()
            rgb = cv2.cvtColor(yuyv, cv2.COLOR_YUV2RGB_YUY2)
            return rgb, gray

        self.get_logger().warning(f"Unsupported image encoding: {msg.encoding}")
        return None, None

    def assign_brick_indices_left_to_right(self, detections):
        """
        Only assign B1..B4 when exactly 4 brick tags are visible.
        Otherwise leave them unnumbered to avoid wrong labels.
        """
        brick_dets = []

        for d in detections:
            tag_id = int(d.get("id", -1))
            if tag_id in self.brick_tag_to_color:
                brick_dets.append(d)

        brick_dets.sort(key=lambda d: float(d.get("cx", 0.0)))

        # Do NOT assign indices unless all 4 bricks are visible
        if len(brick_dets) != 4:
            for d in brick_dets:
                d.pop("brick_index", None)
            return []

        for i, d in enumerate(brick_dets, start=1):
            d["brick_index"] = i
            d["brick_color"] = self.brick_tag_to_color.get(int(d["id"]), "")

        return brick_dets

    def publish_debug_image(self, rgb_debug: np.ndarray, corners, ids, detections, header):
        dbg = cv2.cvtColor(rgb_debug, cv2.COLOR_RGB2BGR)

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(dbg, corners, ids)

        for d in detections:
            cx = int(round(float(d.get("cx", 0.0))))
            cy = int(round(float(d.get("cy", 0.0))))
            marker_id = int(d.get("id", -1))

            label = f"ID {marker_id}"

            if "brick_index" in d:
                label += f"  B{d['brick_index']}"
                color = (0, 255, 255)  # yellow-ish in BGR
            else:
                color = (0, 255, 0)

            cv2.putText(
                dbg,
                label,
                (cx + 8, cy - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

        h, w = dbg.shape[:2]
        if w > self.debug_max_width:
            scale = self.debug_max_width / float(w)
            dbg = cv2.resize(
                dbg,
                (int(w * scale), int(h * scale)),
                interpolation=cv2.INTER_AREA,
            )

        out = Image()
        out.header = header
        out.height = dbg.shape[0]
        out.width = dbg.shape[1]
        out.encoding = "bgr8"
        out.is_bigendian = False
        out.step = dbg.shape[1] * 3
        out.data = dbg.tobytes()
        self.debug_pub.publish(out)

    def image_callback(self, msg: Image):
        try:
            rgb, gray = self.decode_rgb_and_gray(msg)
            if rgb is None or gray is None:
                return

            if self.use_left_half_only and msg.header.frame_id != "left_camera":
                rgb = rgb[:, : rgb.shape[1] // 2, :].copy()
                gray = gray[:, : gray.shape[1] // 2].copy()

            detect_gray = gray
            scale = self.detect_scale

            if 0.1 < scale < 0.999:
                detect_gray = cv2.resize(
                    gray,
                    None,
                    fx=scale,
                    fy=scale,
                    interpolation=cv2.INTER_AREA,
                )

            corners, ids, _ = self.detect_markers(detect_gray)

            if ids is not None and len(ids) > 0 and detect_gray is not gray:
                corners = [c.astype(np.float32) / scale for c in corners]

            self.frame_counter += 1
            now_ns = self.get_clock().now().nanoseconds

            ids_flat = []
            detections = []

            if ids is not None and len(ids) > 0:
                ids_flat = ids.flatten().astype(int).tolist()

                if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
                    pnp_flag = cv2.SOLVEPNP_IPPE_SQUARE
                else:
                    pnp_flag = cv2.SOLVEPNP_ITERATIVE

                s = self.tag_size / 2.0
                obj_pts = np.array([
                    [-s,  s, 0.0],
                    [ s,  s, 0.0],
                    [ s, -s, 0.0],
                    [-s, -s, 0.0],
                ], dtype=np.float32)

                for marker_corners, marker_id in zip(corners, ids_flat):
                    img_pts = np.asarray(marker_corners[0], dtype=np.float32)
                    center = np.mean(img_pts, axis=0)

                    det = {
                        "id": int(marker_id),
                        "cx": float(center[0]),
                        "cy": float(center[1]),
                        "name": self.marker_names.get(int(marker_id), f"id_{marker_id}"),
                    }

                    if int(marker_id) in self.brick_tag_to_color:
                        det["brick_color"] = self.brick_tag_to_color[int(marker_id)]

                    if self.have_valid_intrinsics:
                        ok, rvec, tvec = cv2.solvePnP(
                            obj_pts,
                            img_pts,
                            self.camera_matrix,
                            self.dist_coeffs,
                            flags=pnp_flag,
                        )

                        if ok:
                            det["rvec"] = [float(v) for v in rvec.reshape(3)]
                            det["tvec"] = [float(v) for v in tvec.reshape(3)]

                    detections.append(det)

                self.assign_brick_indices_left_to_right(detections)

            ids_msg = Int32MultiArray()
            ids_msg.data = ids_flat
            self.ids_pub.publish(ids_msg)

            detections_msg = String()
            detections_msg.data = json.dumps({
                "stamp_ns": int(now_ns),
                "detections": detections,
            })
            self.detections_pub.publish(detections_msg)

            if self.publish_debug_every_n > 0 and (self.frame_counter % self.publish_debug_every_n == 0):
                self.publish_debug_image(rgb, corners, ids, detections, msg.header)

            if (ids_flat != self.last_ids) or (now_ns - self.last_log_time_ns > self.log_period_ns):
                self.get_logger().info(f"ids={ids_flat}")
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