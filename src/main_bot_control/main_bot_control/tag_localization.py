import json
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import Bool, String


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

        self.declare_parameter("detections_topic", "/aruco/detections_json")
        self.detections_topic = self.get_parameter("detections_topic").value

        self.marker_map = {
            20: {"x": 0.600, "y": 1.400, "yaw": 0.0},
            21: {"x": 2.400, "y": 1.400, "yaw": 0.0},
            22: {"x": 0.600, "y": 0.600, "yaw": 0.0},
            23: {"x": 2.400, "y": 0.600, "yaw": 0.0},
        }

        self.marker_names = {
            20: "Yellow_close",
            21: "Blue_close",
            22: "Yellow_far",
            23: "Blue_far",
        }

        self.selected_tag_pub = self.create_publisher(String, "/aruco_selected_tag", 10)
        self.pose_pub = self.create_publisher(Pose2D, "/bot_pose_estimate", 10)
        # NEW: publish whether the pose estimate is currently valid (tag visible)
        self.pose_valid_pub = self.create_publisher(Bool, "/bot_pose_estimate_valid", 10)

        T_base_camera_link = make_T(
            x=0.22,
            y=0.00,
            z=0.28,
            roll=0.0,
            pitch=math.pi / 4.0,
            yaw=0.0,
        )

        T_camera_link_left_camera = make_T(
            x=0.0,
            y=0.06,
            z=0.0,
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
        )

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

        self.last_log_time_ns = 0

        self.create_subscription(
            String,
            self.detections_topic,
            self.detections_callback,
            10,
        )

        self.get_logger().info("tag_localization started")
        self.get_logger().info(f"detections_topic={self.detections_topic}")

    def publish_selected_tag(self, text: str):
        msg = String()
        msg.data = text
        self.selected_tag_pub.publish(msg)

    def publish_pose_valid(self, valid: bool):
        msg = Bool()
        msg.data = valid
        self.pose_valid_pub.publish(msg)

    def detections_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            detections = payload.get("detections", [])

            candidates = []

            for det in detections:
                marker_id = int(det["id"])
                if marker_id not in self.marker_map:
                    continue

                # Need rvec/tvec for pose estimation
                if "rvec" not in det or "tvec" not in det:
                    continue

                rvec = np.array(det["rvec"], dtype=np.float64).reshape(3, 1)
                tvec = np.array(det["tvec"], dtype=np.float64).reshape(3, 1)

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

                T_field_base = (
                    T_field_marker
                    @ invert_T(T_camera_marker)
                    @ invert_T(self.T_base_camera)
                )

                pose_msg = Pose2D()
                pose_msg.x = float(T_field_base[0, 3])
                pose_msg.y = float(T_field_base[1, 3])
                pose_msg.theta = math.atan2(T_field_base[1, 0], T_field_base[0, 0])

                dist = float(np.linalg.norm(tvec.reshape(3)))

                candidates.append({
                    "id": marker_id,
                    "pose": pose_msg,
                    "dist": dist,
                })

            if len(candidates) == 0:
                self.publish_selected_tag("NONE")
                # NEW: signal that no valid pose is available
                self.publish_pose_valid(False)
                return

            best = min(candidates, key=lambda c: c["dist"])
            best_id = best["id"]
            best_pose = best["pose"]

            self.pose_pub.publish(best_pose)
            # NEW: signal that the pose is valid (actively seeing a tag)
            self.publish_pose_valid(True)

            best_name = self.marker_names.get(best_id, f"id_{best_id}")
            visible_names = [self.marker_names.get(c["id"], f"id_{c['id']}") for c in candidates]

            if len(candidates) == 1:
                selected_text = f"{best_name} (ID {best_id})"
            else:
                selected_text = f"{best_name} (ID {best_id}) | multiple visible: {visible_names}"

            self.publish_selected_tag(selected_text)

            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_log_time_ns > 300_000_000:
                self.get_logger().info(
                    f"selected={best_name} | visible={visible_names} | "
                    f"bot_pose: x={best_pose.x:.3f}, y={best_pose.y:.3f}, "
                    f"yaw={math.degrees(best_pose.theta):.1f} deg"
                )
                self.last_log_time_ns = now_ns

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