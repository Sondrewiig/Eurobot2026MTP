#!/usr/bin/env python3

import math
import os
import json

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from cv_bridge import CvBridge


class OverheadRectifierNode(Node):
    def __init__(self):
        super().__init__('overhead_rectifier_node')

        self.declare_parameter('board_width_mm', 3000)
        self.declare_parameter('board_height_mm', 2000)
        self.declare_parameter('tag_size_mm', 100)

        self.declare_parameter('image_topic', '/overhead_cam/image')
        self.declare_parameter('camera_info_topic', '/overhead_cam/camera_info')
        self.declare_parameter('warped_topic', '/vision/board_warped')
        self.declare_parameter('debug_topic', '/vision/board_debug')
        self.declare_parameter('robot_pose_topic', '/vision/robot_pose')
        self.declare_parameter('enemy_pose_topic', '/vision/enemy_pose')
        self.declare_parameter('enemy_tag_ids', [0])
        self.declare_parameter('highway_path_topic', '/opencr/highway_path')

        self.declare_parameter('cache_h_seconds', 2.0)

        self.declare_parameter('fixed_ids', [20, 21, 22, 23])
        self.declare_parameter('fixed_x_mm', [600.0, 2400.0, 600.0, 2400.0])
        self.declare_parameter('fixed_y_mm', [1400.0, 1400.0, 600.0, 600.0])
        self.declare_parameter('fixed_yaw_deg', [0.0, 0.0, 0.0, 0.0])

        self.declare_parameter('robot_tag_id', 71)
        self.declare_parameter('robot_tag_size_m', 0.10)
        self.declare_parameter('robot_tag_yaw_offset_deg', 0.0)

        # ---------- Team selection ----------
        # 'yellow' or 'blue' — determines which overhead camera position is used
        # for parallax correction.
        self.declare_parameter('team', 'yellow')
        self.declare_parameter('control_config_path', '')

        # ---------- Empirical correction parameters ----------
        # If true, use camera pose to estimate the point on the board directly under the
        # optical axis. This is better than assuming the board center.
        self.declare_parameter('use_dynamic_camera_center', True)

        # Fallback optical-axis hit point on the board, in mm.
        # Used if dynamic center cannot be computed.
        self.declare_parameter('camera_center_x_mm', 1400.0)
        self.declare_parameter('camera_center_y_mm', 1000.0)

        # Camera height above board, meters. Used as fallback if dynamic camera height
        # cannot be computed.
        self.declare_parameter('camera_height_m', 2.20)

        # Height of robot tag plane above board, meters.
        self.declare_parameter('robot_tag_height_m', 0.35)

        # Enable simple inward parallax correction:
        # P_corr = C + ((H-h)/H) * (P_meas - C)
        self.declare_parameter('apply_parallax_correction', True)

        # EMA smoothing factor (0 = no smoothing, 1 = fully trust new measurement)
        self.declare_parameter('pose_ema_alpha', 0.4)

        # Empirical yaw bias as a function of x:
        # yaw_bias_deg = k1*dx + k3*dx^3, where dx is in meters
        self.declare_parameter('apply_yaw_bias_correction', False)
        self.declare_parameter('yaw_bias_center_x_mm', 1400.0)
        self.declare_parameter('yaw_bias_linear_deg_per_m', 0.0)
        self.declare_parameter('yaw_bias_cubic_deg_per_m3', 0.0)

        # Offset from tag center to robot center in robot frame.
        # Convention:
        #   forward = + along robot heading
        #   right   = + to robot's right side
        self.declare_parameter('tag_to_robot_forward_m', 0.0)
        self.declare_parameter('tag_to_robot_right_m', 0.0)

        self.board_width_mm = int(self.get_parameter('board_width_mm').value)
        self.board_height_mm = int(self.get_parameter('board_height_mm').value)
        self.tag_size_mm = float(self.get_parameter('tag_size_mm').value)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.control_config_path = str(self.get_parameter('control_config_path').value)
        self.warped_topic = str(self.get_parameter('warped_topic').value)
        self.debug_topic = str(self.get_parameter('debug_topic').value)
        self.robot_pose_topic = str(self.get_parameter('robot_pose_topic').value)
        self.enemy_pose_topic = str(self.get_parameter('enemy_pose_topic').value)
        self.highway_path_topic = str(self.get_parameter('highway_path_topic').value)

        self.cache_h_seconds = float(self.get_parameter('cache_h_seconds').value)

        self.robot_tag_id = int(self.get_parameter('robot_tag_id').value)
        self.robot_tag_size_m = float(self.get_parameter('robot_tag_size_m').value)
        self.robot_tag_yaw_offset_rad = math.radians(
            float(self.get_parameter('robot_tag_yaw_offset_deg').value)
        )

        self.use_dynamic_camera_center = bool(
            self.get_parameter('use_dynamic_camera_center').value
        )
        self.camera_center_x_m = float(
            self.get_parameter('camera_center_x_mm').value
        ) / 1000.0
        self.camera_center_y_m = float(
            self.get_parameter('camera_center_y_mm').value
        ) / 1000.0
        self.camera_height_m = float(self.get_parameter('camera_height_m').value)
        self.robot_tag_height_m = float(self.get_parameter('robot_tag_height_m').value)
        self.apply_parallax_correction = bool(
            self.get_parameter('apply_parallax_correction').value
        )

        # Camera positions per team (mm -> m)
        self.team = str(self.get_parameter('team').value).lower()
        control_team = self._load_team_from_control_config(self.control_config_path)
        if control_team is not None:
            self.team = control_team
        self._select_overhead_topics_for_team()

        camera_positions = {
            'yellow': (1.275, 2.100, 1.670),
            'blue':   (1.725, 2.100, 1.670),
        }
        if self.team not in camera_positions:
            self.get_logger().error(f"Unknown team '{self.team}', defaulting to yellow")
            self.team = 'yellow'
        self.overhead_cam_x, self.overhead_cam_y, self.overhead_cam_z = camera_positions[self.team]

        self.enemy_tag_ids = self._load_enemy_tag_ids()

        self.pose_ema_alpha = float(self.get_parameter('pose_ema_alpha').value)

        self.apply_yaw_bias_correction = bool(
            self.get_parameter('apply_yaw_bias_correction').value
        )
        self.yaw_bias_center_x_m = float(
            self.get_parameter('yaw_bias_center_x_mm').value
        ) / 1000.0
        self.yaw_bias_linear_deg_per_m = float(
            self.get_parameter('yaw_bias_linear_deg_per_m').value
        )
        self.yaw_bias_cubic_deg_per_m3 = float(
            self.get_parameter('yaw_bias_cubic_deg_per_m3').value
        )

        self.tag_to_robot_forward_m = float(
            self.get_parameter('tag_to_robot_forward_m').value
        )
        self.tag_to_robot_right_m = float(
            self.get_parameter('tag_to_robot_right_m').value
        )

        fixed_ids = list(self.get_parameter('fixed_ids').value)
        fixed_x = list(self.get_parameter('fixed_x_mm').value)
        fixed_y = list(self.get_parameter('fixed_y_mm').value)
        fixed_yaw = list(self.get_parameter('fixed_yaw_deg').value)

        if not (len(fixed_ids) == len(fixed_x) == len(fixed_y) == len(fixed_yaw)):
            raise ValueError('fixed_ids, fixed_x_mm, fixed_y_mm, fixed_yaw_deg must have same length')

        self.fixed_tags = {}
        for i, tag_id in enumerate(fixed_ids):
            self.fixed_tags[int(tag_id)] = {
                'x_mm': float(fixed_x[i]),
                'y_mm': float(fixed_y[i]),
                'yaw_deg': float(fixed_yaw[i]),
            }

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, qos_profile_sensor_data
        )
        self.highway_path_sub = self.create_subscription(
            String, self.highway_path_topic, self.highway_path_callback, 10
        )

        self.warped_pub = self.create_publisher(Image, self.warped_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)
        self.robot_pose_pub = self.create_publisher(Pose2D, self.robot_pose_topic, 10)
        self.enemy_pose_pub = self.create_publisher(Pose2D, self.enemy_pose_topic, 10)

        self.last_h = None
        self.last_h_time = None

        self.camera_matrix = None
        self.dist_coeffs = None

        self.last_robot_pose = None
        self.last_enemy_pose = None
        self.last_enemy_tag_id = None
        self.last_cam_rvec = None
        self.last_cam_tvec = None
        self.last_cam_pose_time = None

        self.last_camera_center_world = None
        self.last_camera_height_world = None

        self.current_highway_path = []
        self.current_highway_path_index = 0
        self.current_highway_path_active = False
        self.current_highway_path_label = ''
        self.current_highway_current_pose = None

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.get_logger().info(f'Listening on {self.image_topic}')
        self.get_logger().info(f'Listening on {self.camera_info_topic}')
        self.get_logger().info(f'Publishing warped image on {self.warped_topic}')
        self.get_logger().info(f'Publishing debug image on {self.debug_topic}')
        self.get_logger().info(f'Publishing robot pose on {self.robot_pose_topic}')
        self.get_logger().info(f'Publishing enemy pose on {self.enemy_pose_topic}')
        self.get_logger().info(f'Listening for highway path on {self.highway_path_topic}')
        self.get_logger().info(f'Fixed tags: {sorted(self.fixed_tags.keys())}')
        self.get_logger().info(f'Robot tag ID: {self.robot_tag_id}')
        self.get_logger().info(f'Enemy tag IDs: {self.enemy_tag_ids}')
        self.get_logger().info(
            f'Team: {self.team} | overhead cam pos: '
            f'({self.overhead_cam_x:.3f}, {self.overhead_cam_y:.3f}, {self.overhead_cam_z:.3f}) m'
        )

        # ── Load highway/avoidance overlay values from control_tuning.yaml ──
        self.hw_outer = (375.0, 2600.0, 350.0, 1300.0)  # x_min, x_max, y_min, y_max
        self.hw_inner = (700.0, 2300.0, 550.0, 1100.0)
        self.bot_front_mm = 336.0
        self.bot_rear_mm = 59.0
        self.bot_half_width_mm = 150.0
        self.avoidance_enabled = True
        self.avoidance_front_margin_mm = 500.0
        self.avoidance_rear_margin_mm = 300.0
        self.avoidance_side_margin_mm = 300.0
        self._load_highway_corridor_config()


    def _load_team_from_control_config(self, path: str):
        if not path:
            return None
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            cfg = data.get('opencr_bridge_sim', data)
            team_cfg = cfg.get('team', {}) if isinstance(cfg, dict) else {}
            team = str(team_cfg.get('color', '')).strip().lower()
            if team in ('yellow', 'blue'):
                return team
        except Exception as exc:
            self.get_logger().warn(f"Could not read team from control_tuning.yaml '{path}': {exc}")
        return None

    def _load_enemy_tag_ids(self):
        """Return all possible opponent top-tag IDs for the active team.

        Blue robots use ArUco IDs 1-5. Yellow robots use IDs 6-10.
        The overhead camera should therefore look for the opposite range.
        A launch override may pass enemy_tag_ids explicitly.
        """
        raw = list(self.get_parameter('enemy_tag_ids').value)
        ids = []
        for value in raw:
            try:
                tag_id = int(value)
            except Exception:
                continue
            if 1 <= tag_id <= 10 and tag_id not in ids:
                ids.append(tag_id)

        if ids:
            return ids

        if self.team == 'blue':
            return [6, 7, 8, 9, 10]
        return [1, 2, 3, 4, 5]

    def _select_overhead_topics_for_team(self):
        # If the YAML still uses the old neutral /overhead_cam topics, select
        # the actual camera topic from the active team. If a custom topic is set,
        # leave it alone.
        default_image_topics = {'/overhead_cam/image', '/overhead_cam_yellow/image', '/overhead_cam_blue/image'}
        default_info_topics = {'/overhead_cam/camera_info', '/overhead_cam_yellow/camera_info', '/overhead_cam_blue/camera_info'}
        if self.image_topic in default_image_topics:
            self.image_topic = f'/overhead_cam_{self.team}/image'
        if self.camera_info_topic in default_info_topics:
            self.camera_info_topic = f'/overhead_cam_{self.team}/camera_info'

    def highway_path_callback(self, msg: String):
        try:
            data = json.loads(msg.data) if msg.data else {}
        except Exception as exc:
            self.get_logger().warn(f"Ignoring invalid /opencr/highway_path payload: {exc}")
            return

        waypoints = []
        for item in data.get("waypoints_mm", []):
            if not isinstance(item, (list, tuple)) or len(item) < 2:
                continue
            try:
                waypoints.append((float(item[0]), float(item[1]), float(item[2]) if len(item) > 2 else 0.0))
            except Exception:
                continue

        cp = data.get("current_pose_mm")
        current_pose = None
        if isinstance(cp, (list, tuple)) and len(cp) >= 2:
            try:
                current_pose = (float(cp[0]), float(cp[1]), float(cp[2]) if len(cp) > 2 else 0.0)
            except Exception:
                current_pose = None

        self.current_highway_path = waypoints
        self.current_highway_path_index = max(0, int(data.get("path_index", 0)))
        self.current_highway_path_active = bool(data.get("active", False))
        self.current_highway_path_label = str(data.get("label", ""))
        self.current_highway_current_pose = current_pose

    def _robot_rect_points_mm(self, x_mm: float, y_mm: float, yaw_rad: float,
                              front_mm: float, rear_mm: float, half_width_mm: float):
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)
        pts = []
        for fwd, lat in (
            (front_mm, half_width_mm),
            (front_mm, -half_width_mm),
            (-rear_mm, -half_width_mm),
            (-rear_mm, half_width_mm),
        ):
            pts.append((x_mm + fwd * c - lat * s,
                        y_mm + fwd * s + lat * c))
        return pts

    def _enemy_avoidance_rect_points_mm(self, pose: Pose2D):
        return self._robot_rect_points_mm(
            pose.x * 1000.0,
            pose.y * 1000.0,
            pose.theta,
            self.bot_front_mm + self.avoidance_front_margin_mm,
            self.bot_rear_mm + self.avoidance_rear_margin_mm,
            self.bot_half_width_mm + self.avoidance_side_margin_mm,
        )

    def _load_highway_corridor_config(self):
        """Load highway/avoidance overlay values from control_tuning.yaml."""
        try:
            from ament_index_python.packages import get_package_share_directory
            default_path = os.path.join(
                get_package_share_directory("sondre_bot_control"),
                "config",
                "control_tuning.yaml",
            )
        except Exception:
            default_path = None

        # Also check the source tree as a fallback
        candidates = []
        if default_path:
            candidates.append(default_path)

        # Common source-tree locations
        for base in [os.path.expanduser("~/sondre_bot_gz/src/sondre_bot_control/sondre_bot_control/config"),
                     os.path.expanduser("~/sondre_bot_gz/install/sondre_bot_control/share/sondre_bot_control/config")]:
            p = os.path.join(base, "control_tuning.yaml")
            if p not in candidates:
                candidates.append(p)

        actual_path = next((p for p in candidates if p and os.path.exists(p)), None)
        if actual_path is None:
            self.get_logger().warn("control_tuning.yaml not found — using default corridor bounds")
            return

        try:
            with open(actual_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(f"Could not read {actual_path}: {exc}")
            return

        cfg = data.get("opencr_bridge_sim", data)
        if not isinstance(cfg, dict):
            return

        rg = cfg.get("robot_geometry", {})
        if isinstance(rg, dict):
            self.bot_front_mm = float(rg.get("bot_front_mm", self.bot_front_mm))
            self.bot_rear_mm = float(rg.get("bot_rear_mm", self.bot_rear_mm))
            self.bot_half_width_mm = float(rg.get("bot_half_width_mm", self.bot_half_width_mm))

        av = cfg.get("avoidance", {})
        if isinstance(av, dict):
            self.avoidance_enabled = bool(av.get("enabled", self.avoidance_enabled))
            self.avoidance_front_margin_mm = float(av.get("opponent_front_margin_mm", self.avoidance_front_margin_mm))
            self.avoidance_rear_margin_mm = float(av.get("opponent_rear_margin_mm", self.avoidance_rear_margin_mm))
            self.avoidance_side_margin_mm = float(av.get("opponent_side_margin_mm", self.avoidance_side_margin_mm))

        hw = cfg.get("highway", {})
        if not isinstance(hw, dict):
            return

        def g(key, default):
            v = hw.get(key)
            return float(v) if v is not None else default

        self.hw_outer = (
            g("outer_x_min_mm", self.hw_outer[0]),
            g("outer_x_max_mm", self.hw_outer[1]),
            g("outer_y_min_mm", self.hw_outer[2]),
            g("outer_y_max_mm", self.hw_outer[3]),
        )
        self.hw_inner = (
            g("inner_x_min_mm", self.hw_inner[0]),
            g("inner_x_max_mm", self.hw_inner[1]),
            g("inner_y_min_mm", self.hw_inner[2]),
            g("inner_y_max_mm", self.hw_inner[3]),
        )

        self.get_logger().info(
            f"Highway corridor from {actual_path}: "
            f"outer=({self.hw_outer[0]:.0f},{self.hw_outer[1]:.0f},{self.hw_outer[2]:.0f},{self.hw_outer[3]:.0f}) "
            f"inner=({self.hw_inner[0]:.0f},{self.hw_inner[1]:.0f},{self.hw_inner[2]:.0f},{self.hw_inner[3]:.0f})"
        )
        self.get_logger().info(
            f"Avoidance overlay: enabled={self.avoidance_enabled} "
            f"front_margin={self.avoidance_front_margin_mm:.0f}mm "
            f"rear_margin={self.avoidance_rear_margin_mm:.0f}mm "
            f"side_margin={self.avoidance_side_margin_mm:.0f}mm"
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    @staticmethod
    def wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def homography_still_valid(self) -> bool:
        if self.last_h is None or self.last_h_time is None:
            return False
        age = (self.get_clock().now() - self.last_h_time).nanoseconds / 1e9
        return age <= self.cache_h_seconds

    def camera_pose_still_valid(self) -> bool:
        if self.last_cam_rvec is None or self.last_cam_tvec is None or self.last_cam_pose_time is None:
            return False
        age = (self.get_clock().now() - self.last_cam_pose_time).nanoseconds / 1e9
        return age <= self.cache_h_seconds

    def dst_tag_corners_px(self, cx_mm: float, cy_mm: float) -> np.ndarray:
        px = self.board_width_mm - cx_mm
        py = cy_mm
        h = self.tag_size_mm / 2.0

        return np.array([
            [px + h, py - h],
            [px - h, py - h],
            [px - h, py + h],
            [px + h, py + h],
        ], dtype=np.float32)

    def fixed_tag_corners_world_3d(self, cx_mm: float, cy_mm: float) -> np.ndarray:
        cx = cx_mm / 1000.0
        cy = cy_mm / 1000.0
        h = self.tag_size_mm / 2000.0

        return np.array([
            [cx - h, cy + h, 0.0],
            [cx + h, cy + h, 0.0],
            [cx + h, cy - h, 0.0],
            [cx - h, cy - h, 0.0],
        ], dtype=np.float32)

    def robot_tag_corners_local_3d(self) -> np.ndarray:
        h = self.robot_tag_size_m / 2.0
        return np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float32)

    def solve_camera_pose_from_fixed_tags(self, image_pts: np.ndarray, world_pts: np.ndarray):
        if self.camera_matrix is None:
            return None, None

        dist = self.dist_coeffs if self.dist_coeffs is not None else np.zeros((5, 1), dtype=np.float64)

        ok, rvec, tvec = cv2.solvePnP(
            world_pts.astype(np.float32),
            image_pts.astype(np.float32),
            self.camera_matrix,
            dist,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        if not ok:
            return None, None

        return rvec, tvec

    def get_camera_world_geometry(self, cam_rvec, cam_tvec):
        """
        Returns:
          cam_pos_world: np.array shape (3,)
          optical_axis_hit_xy: (cx, cy) on z=0 plane
          cam_height: camera z above board
        """
        if cam_rvec is None or cam_tvec is None:
            return None, (self.camera_center_x_m, self.camera_center_y_m), self.camera_height_m

        R_cw, _ = cv2.Rodrigues(cam_rvec)
        t_cw = cam_tvec.reshape(3, 1)

        R_world_optical = R_cw.T
        cam_pos_world = (-R_world_optical @ t_cw).reshape(3)

        optical_axis_world = R_world_optical @ np.array([0.0, 0.0, 1.0], dtype=np.float64)

        cx = self.camera_center_x_m
        cy = self.camera_center_y_m

        if self.use_dynamic_camera_center and abs(optical_axis_world[2]) > 1e-9:
            lam = -cam_pos_world[2] / optical_axis_world[2]
            hit = cam_pos_world + lam * optical_axis_world
            cx = float(hit[0])
            cy = float(hit[1])

        cam_height = float(cam_pos_world[2])

        return cam_pos_world, (cx, cy), cam_height

    def correct_tag_xy_for_parallax(self, x_tag: float, y_tag: float, camera_x: float, camera_y: float, camera_z: float):
        if not self.apply_parallax_correction:
            return x_tag, y_tag

        # Homography maps the detected top tag center to the BOARD plane (z=0).
        # The tag is actually at bot_z above the board, so recover the point
        # along the camera ray at z=bot_z.
        #
        # A_x_corr = (A_x_board * overhead_z - A_x_board * bot_z + bot_z * overhead_x) / overhead_z
        # A_y_corr = (A_y_board * overhead_z - A_y_board * bot_z + bot_z * overhead_y) / overhead_z
        #
        # Equivalent form:
        # A_corr = A_board * (1 - bot_z / overhead_z) + overhead_xy * (bot_z / overhead_z)
        bot_z = max(0.0, float(self.robot_tag_height_m))
        overhead_z = float(camera_z) if camera_z is not None else float(self.overhead_cam_z)
        if overhead_z <= bot_z + 0.05:
            overhead_z = float(self.overhead_cam_z)
        if overhead_z <= bot_z + 0.05:
            return x_tag, y_tag

        overhead_x = float(camera_x) if camera_x is not None else float(self.overhead_cam_x)
        overhead_y = float(camera_y) if camera_y is not None else float(self.overhead_cam_y)
        height_ratio = bot_z / overhead_z

        x_corr = x_tag * (1.0 - height_ratio) + overhead_x * height_ratio
        y_corr = y_tag * (1.0 - height_ratio) + overhead_y * height_ratio
        return x_corr, y_corr

    def correct_yaw_bias_from_x(self, theta: float, x_world: float) -> float:
        if not self.apply_yaw_bias_correction:
            return theta

        dx = x_world - self.yaw_bias_center_x_m
        bias_deg = (
            self.yaw_bias_linear_deg_per_m * dx
            + self.yaw_bias_cubic_deg_per_m3 * (dx ** 3)
        )
        return self.wrap_angle(theta - math.radians(bias_deg))

    def tag_pose_to_robot_pose(self, x_tag: float, y_tag: float, theta: float):
        """
        theta is robot heading.
        tag_to_robot_forward_m: positive in heading direction
        tag_to_robot_right_m:   positive to robot's right side
        """
        forward = self.tag_to_robot_forward_m
        right = self.tag_to_robot_right_m

        x_robot = x_tag + forward * math.cos(theta) + right * math.sin(theta)
        y_robot = y_tag + forward * math.sin(theta) - right * math.cos(theta)
        return x_robot, y_robot

    def choose_best_robot_pose(self, rvecs, tvecs, reprojection_errors, R_world_optical, cam_pos_world):
        """Take only the first IPPE solution (lowest reprojection error).
        Validate it falls within the board bounds."""

        if len(rvecs) == 0:
            return None

        # IPPE_SQUARE returns solutions sorted by reprojection error (best first)
        rvec = rvecs[0]
        tvec = tvecs[0]

        R_optical_tag, _ = cv2.Rodrigues(rvec)
        t_optical_tag = tvec.reshape(3, 1)

        tag_center_world = cam_pos_world.reshape(3, 1) + R_world_optical @ t_optical_tag
        x = float(tag_center_world[0, 0])
        y = float(tag_center_world[1, 0])
        z = float(tag_center_world[2, 0])

        R_world_tag = R_world_optical @ R_optical_tag
        forward_world = R_world_tag @ np.array([0.0, 1.0, 0.0], dtype=np.float64)
        theta = self.wrap_angle(
            math.atan2(forward_world[1], forward_world[0]) + self.robot_tag_yaw_offset_rad
        )

        # Basic sanity check
        if not (0.0 <= x <= 3.0 and 0.0 <= y <= 2.0):
            return None
        if not (0.10 <= z <= 1.0):
            return None

        return (x, y, theta)

    def pose_from_homography(self, H, tag_corners, cam_rvec=None, cam_tvec=None):
        """Compute a robot-center Pose2D from a top tag using the homography.

        The same geometry is used for the active robot and the opponent:
        the ArUco tag is on top of the robot, and tag_to_robot_* converts
        from tag center to wheel-axis/robot pose.
        """
        rc = tag_corners.reshape(4, 2).astype(np.float32)

        # Tag center in image pixels
        center_img = np.mean(rc, axis=0).reshape(1, 1, 2)

        # Transform to board-mm space
        center_board = cv2.perspectiveTransform(center_img, H).reshape(2)

        # Convert board-mm to world coords (x is flipped: world_x = board_width - px)
        x_world = (self.board_width_mm - center_board[0]) / 1000.0
        y_world = center_board[1] / 1000.0

        # Compute yaw from the tag's "forward" direction
        # ArUco corner order: [top-left, top-right, bottom-right, bottom-left]
        # "Forward" = from bottom-center to top-center
        top_center = (rc[0] + rc[1]) / 2.0
        bottom_center = (rc[2] + rc[3]) / 2.0

        top_board = cv2.perspectiveTransform(
            top_center.reshape(1, 1, 2), H
        ).reshape(2)
        bottom_board = cv2.perspectiveTransform(
            bottom_center.reshape(1, 1, 2), H
        ).reshape(2)

        # Forward direction in board-mm (flipped x)
        dx = -(top_board[0] - bottom_board[0])  # negate because x is flipped
        dy = top_board[1] - bottom_board[1]
        theta = self.wrap_angle(math.atan2(dy, dx) + self.robot_tag_yaw_offset_rad)

        # Apply height correction using the actual overhead camera position.
        # Do not use the optical-axis hit point here; the ray correction must
        # interpolate toward the camera's x/y position above the board.
        cam_pos_world, _, cam_height = self.get_camera_world_geometry(cam_rvec, cam_tvec)
        if cam_pos_world is not None:
            camera_x = float(cam_pos_world[0])
            camera_y = float(cam_pos_world[1])
        else:
            camera_x = self.overhead_cam_x
            camera_y = self.overhead_cam_y
        x_corr, y_corr = self.correct_tag_xy_for_parallax(
            x_world, y_world, camera_x, camera_y, cam_height
        )

        # Apply tag-to-robot offset
        x_robot, y_robot = self.tag_pose_to_robot_pose(x_corr, y_corr, theta)

        pose = Pose2D()
        pose.x = x_robot
        pose.y = y_robot
        pose.theta = theta
        return pose

    def publish_robot_pose_from_homography(self, H, robot_corners, cam_rvec=None, cam_tvec=None):
        """Compute and publish active robot pose using the homography."""
        pose = self.pose_from_homography(H, robot_corners, cam_rvec, cam_tvec)
        self.last_robot_pose = pose
        self.robot_pose_pub.publish(pose)

    def publish_enemy_pose_from_homography(self, H, enemy_corners, enemy_tag_id=None, cam_rvec=None, cam_tvec=None):
        """Compute and publish opponent pose using the homography."""
        pose = self.pose_from_homography(H, enemy_corners, cam_rvec, cam_tvec)
        self.last_enemy_pose = pose
        self.last_enemy_tag_id = int(enemy_tag_id) if enemy_tag_id is not None else None
        self.enemy_pose_pub.publish(pose)

    def publish_robot_pose_from_raw(self, cam_rvec, cam_tvec, robot_corners):
        if self.camera_matrix is None:
            return
        if cam_rvec is None or cam_tvec is None:
            return
        if robot_corners is None:
            return

        dist = self.dist_coeffs if self.dist_coeffs is not None else np.zeros((5, 1), dtype=np.float64)

        robot_obj_pts = self.robot_tag_corners_local_3d()
        robot_img_pts = robot_corners.reshape(4, 2).astype(np.float32)

        ok, rvecs, tvecs, reprojection_errors = cv2.solvePnPGeneric(
            robot_obj_pts,
            robot_img_pts,
            self.camera_matrix,
            dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not ok or len(rvecs) == 0:
            return

        R_cw, _ = cv2.Rodrigues(cam_rvec)
        t_cw = cam_tvec.reshape(3, 1)

        R_world_optical = R_cw.T
        cam_pos_world = (-R_world_optical @ t_cw).reshape(3)

        best = self.choose_best_robot_pose(
            rvecs, tvecs, reprojection_errors,
            R_world_optical, cam_pos_world
        )

        if best is None:
            return

        x_tag_raw, y_tag_raw, theta_raw = best

        _, (center_x, center_y), cam_height = self.get_camera_world_geometry(cam_rvec, cam_tvec)
        self.last_camera_center_world = (center_x, center_y)
        self.last_camera_height_world = cam_height

        # 1) Correct x/y for systematic parallax-like bias
        x_tag_corr, y_tag_corr = self.correct_tag_xy_for_parallax(
            x_tag_raw, y_tag_raw, center_x, center_y, cam_height
        )

        # 2) Correct yaw bias as a function of x
        theta_corr = self.correct_yaw_bias_from_x(theta_raw, x_tag_corr)

        # 3) Convert from tag center pose to robot center pose if needed
        x_robot, y_robot = self.tag_pose_to_robot_pose(
            x_tag_corr, y_tag_corr, theta_corr
        )

        pose = Pose2D()
        pose.x = x_robot
        pose.y = y_robot
        pose.theta = theta_corr

        # EMA smoothing to reduce jitter
        if self.last_robot_pose is not None:
            a = self.pose_ema_alpha
            pose.x = (1 - a) * self.last_robot_pose.x + a * pose.x
            pose.y = (1 - a) * self.last_robot_pose.y + a * pose.y
            # Smooth angle carefully
            dtheta = math.atan2(
                math.sin(pose.theta - self.last_robot_pose.theta),
                math.cos(pose.theta - self.last_robot_pose.theta),
            )
            pose.theta = self.wrap_angle(self.last_robot_pose.theta + a * dtheta)

        self.last_robot_pose = pose
        self.robot_pose_pub.publish(pose)

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        debug = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.detector_params
        )

        H_use = None
        cam_rvec_use = None
        cam_tvec_use = None
        status_text = 'NO TAGS'
        found_fixed_ids = []
        robot_corners = None
        enemy_corners = None
        enemy_tag_id = None

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()
            cv2.aruco.drawDetectedMarkers(debug, corners, ids.reshape(-1, 1))

            image_pts_2d = []
            board_pts_2d = []

            image_pts_world = []
            world_pts = []

            for marker_corners, marker_id in zip(corners, ids_flat):
                marker_id = int(marker_id)

                if marker_id == self.robot_tag_id:
                    robot_corners = marker_corners.copy()
                elif marker_id in self.enemy_tag_ids and enemy_corners is None:
                    enemy_corners = marker_corners.copy()
                    enemy_tag_id = marker_id

                if marker_id not in self.fixed_tags:
                    continue

                found_fixed_ids.append(marker_id)

                img4 = marker_corners.reshape(4, 2).astype(np.float32)
                tag_cfg = self.fixed_tags[marker_id]

                dst4 = self.dst_tag_corners_px(tag_cfg['x_mm'], tag_cfg['y_mm'])
                obj4_world = self.fixed_tag_corners_world_3d(tag_cfg['x_mm'], tag_cfg['y_mm'])

                image_pts_2d.append(img4)
                board_pts_2d.append(dst4)

                image_pts_world.append(img4)
                world_pts.append(obj4_world)

            if len(image_pts_2d) >= 2 and len(found_fixed_ids) == len(self.fixed_tags):
                image_pts_2d = np.vstack(image_pts_2d)
                board_pts_2d = np.vstack(board_pts_2d)

                H, _ = cv2.findHomography(image_pts_2d, board_pts_2d, 0)

                if H is not None:
                    self.last_h = H
                    self.last_h_time = self.get_clock().now()
                    H_use = H
                    status_text = f'LIVE H  fixed={sorted(found_fixed_ids)}'
                else:
                    status_text = f'FAILED H  fixed={sorted(found_fixed_ids)}'
            else:
                status_text = f'NOT ENOUGH FIXED TAGS  fixed={sorted(found_fixed_ids)}'

            if len(image_pts_world) >= 2:
                image_pts_world = np.vstack(image_pts_world)
                world_pts = np.vstack(world_pts)

                cam_rvec, cam_tvec = self.solve_camera_pose_from_fixed_tags(image_pts_world, world_pts)
                if cam_rvec is not None:
                    self.last_cam_rvec = cam_rvec
                    self.last_cam_tvec = cam_tvec
                    self.last_cam_pose_time = self.get_clock().now()
                    cam_rvec_use = cam_rvec
                    cam_tvec_use = cam_tvec

        if H_use is None and self.homography_still_valid():
            H_use = self.last_h
            if found_fixed_ids:
                status_text = f'CACHED H  fixed={sorted(found_fixed_ids)}'
            else:
                status_text = 'CACHED H  no fixed tags'
        elif H_use is None and self.last_h is not None:
            # Even if cache is "expired", keep using it — better than nothing
            H_use = self.last_h
            status_text = 'STALE H (reusing last good)'

        if cam_rvec_use is None and self.camera_pose_still_valid():
            cam_rvec_use = self.last_cam_rvec
            cam_tvec_use = self.last_cam_tvec

        if H_use is not None:
            warped = cv2.warpPerspective(
                frame,
                H_use,
                (self.board_width_mm, self.board_height_mm)
            )
            warped_msg = self.bridge.cv2_to_imgmsg(warped, encoding='bgr8')
            warped_msg.header = msg.header
            self.warped_pub.publish(warped_msg)

        if robot_corners is not None and H_use is not None:
            self.publish_robot_pose_from_homography(H_use, robot_corners, cam_rvec_use, cam_tvec_use)

        if enemy_corners is not None and H_use is not None:
            self.publish_enemy_pose_from_homography(H_use, enemy_corners, enemy_tag_id, cam_rvec_use, cam_tvec_use)

        # ---- Debug image: draw overlay on warped top-down view ----
        # Drawing on the warped image avoids all H_inv instability issues.
        # Board-mm coord: px = board_width - world_x, py = world_y
        if H_use is not None:
            overlay = warped.copy()
            bw = self.board_width_mm

            def w2b(x_mm, y_mm):
                return (int(bw - x_mm), int(y_mm))

            # Arena perimeter
            cv2.rectangle(overlay, w2b(0, 0), w2b(3000, 2000), (0, 255, 0), 6)

            # ── Highway corridor zone (hollow red rectangle) ──
            # Outer boundary = drivable corridor limits
            # Inner boundary = forbidden zone the bot must go around
            # Drawn as a semi-transparent red fill between the two rectangles.
            hw_color = (0, 0, 255)  # red in BGR

            # Draw with semi-transparent fill
            corridor_overlay = overlay.copy()

            # Fill outer rectangle
            cv2.rectangle(corridor_overlay,
                          w2b(self.hw_outer[0], self.hw_outer[2]),
                          w2b(self.hw_outer[1], self.hw_outer[3]),
                          hw_color, -1)

            # Cut out inner rectangle (restore original pixels)
            inner_tl = w2b(self.hw_inner[1], self.hw_inner[2])
            inner_br = w2b(self.hw_inner[0], self.hw_inner[3])
            x1 = min(inner_tl[0], inner_br[0])
            y1 = min(inner_tl[1], inner_br[1])
            x2 = max(inner_tl[0], inner_br[0])
            y2 = max(inner_tl[1], inner_br[1])
            corridor_overlay[y1:y2, x1:x2] = overlay[y1:y2, x1:x2]

            # Blend at 25% opacity
            cv2.addWeighted(corridor_overlay, 0.25, overlay, 0.75, 0, overlay)

            # Draw outlines on top (solid, not transparent)
            cv2.rectangle(overlay,
                          w2b(self.hw_outer[0], self.hw_outer[2]),
                          w2b(self.hw_outer[1], self.hw_outer[3]),
                          hw_color, 4, cv2.LINE_AA)
            cv2.rectangle(overlay,
                          w2b(self.hw_inner[0], self.hw_inner[2]),
                          w2b(self.hw_inner[1], self.hw_inner[3]),
                          hw_color, 4, cv2.LINE_AA)

            # ── Current optimal highway path ──
            # Draw the active planned route from the controller.
            if self.current_highway_path_active and self.current_highway_path:
                full_pts = [w2b(x, y) for (x, y, _yaw) in self.current_highway_path]
                if len(full_pts) >= 2:
                    cv2.polylines(overlay, [np.array(full_pts, dtype=np.int32)], False, (180, 180, 180), 4, cv2.LINE_AA)

                rem_pts_world = []
                if self.current_highway_current_pose is not None:
                    rem_pts_world.append((self.current_highway_current_pose[0], self.current_highway_current_pose[1]))

                start_idx = min(max(self.current_highway_path_index, 0), len(self.current_highway_path))
                rem_pts_world.extend((x, y) for (x, y, _yaw) in self.current_highway_path[start_idx:])
                rem_pts = [w2b(x, y) for (x, y) in rem_pts_world]

                if len(rem_pts) >= 2:
                    cv2.polylines(overlay, [np.array(rem_pts, dtype=np.int32)], False, (0, 255, 255), 8, cv2.LINE_AA)

                for i, pt in enumerate(full_pts):
                    color = (0, 255, 255) if i >= self.current_highway_path_index else (180, 180, 180)
                    radius = 18 if i == self.current_highway_path_index else 12
                    cv2.circle(overlay, pt, radius, color, -1, cv2.LINE_AA)
                    cv2.circle(overlay, pt, radius + 4, (0, 0, 0), 2, cv2.LINE_AA)

                if rem_pts:
                    cv2.putText(overlay,
                                f"PATH: {self.current_highway_path_label} [{self.current_highway_path_index + 1}/{len(self.current_highway_path)}]",
                                (50, 180),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3, cv2.LINE_AA)

            # Corner labels
            for (x, y), label in [((0,0),"(0,0)"), ((3000,0),"(3,0)"), ((3000,2000),"(3,2)"), ((0,2000),"(0,2)")]:
                pt = w2b(x, y)
                cv2.putText(overlay, label, (pt[0]+15, pt[1]+50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.8, (0, 255, 0), 3, cv2.LINE_AA)

            # Axis arrows
            cv2.arrowedLine(overlay, w2b(0, 0), w2b(600, 0), (0, 0, 255), 10, cv2.LINE_AA, tipLength=0.12)
            cv2.arrowedLine(overlay, w2b(0, 0), w2b(0, 600), (0, 255, 0), 10, cv2.LINE_AA, tipLength=0.12)
            cv2.putText(overlay, "X", (w2b(600, 0)[0]+15, w2b(600, 0)[1]-15),
                        cv2.FONT_HERSHEY_SIMPLEX, 2.5, (0, 0, 255), 5)
            cv2.putText(overlay, "Y", (w2b(0, 600)[0]+15, w2b(0, 600)[1]+60),
                        cv2.FONT_HERSHEY_SIMPLEX, 2.5, (0, 255, 0), 5)

            # Fixed tags
            for tag_id, tag_cfg in self.fixed_tags.items():
                pt = w2b(tag_cfg['x_mm'], tag_cfg['y_mm'])
                cv2.circle(overlay, pt, 25, (255, 0, 255), -1, cv2.LINE_AA)
                cv2.putText(overlay, f"ID{tag_id}", (pt[0]+30, pt[1]+15),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 0, 255), 3)

            # ── Jenga pickup zones (blue, 200x150mm) ──
            # Zones 1,2,7,8 are rotated: 150w x 200h (long side vertical)
            # Zones 3-6 are normal: 200w x 150h (long side horizontal)
            jenga_zones = {
                1: (175, 1200),  2: (175, 400),
                3: (1150, 800),  4: (1100, 175),
                5: (1850, 800),  6: (1900, 175),
                7: (2825, 1200), 8: (2825, 400),
            }
            rotated_zones = {1, 2, 7, 8}
            for zid, (zx, zy) in jenga_zones.items():
                if zid in rotated_zones:
                    hw, hh = 75, 100   # 150w x 200h
                else:
                    hw, hh = 100, 75   # 200w x 150h
                tl = w2b(zx - hw, zy - hh)
                br = w2b(zx + hw, zy + hh)
                cv2.rectangle(overlay, tl, br, (255, 150, 0), 4, cv2.LINE_AA)
                ct = w2b(zx, zy)
                cv2.putText(overlay, f"Z{zid}", (ct[0]-30, ct[1]+15),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 150, 0), 3, cv2.LINE_AA)

            # ── Pantry placement zones (green, 200x200mm) ──
            pantry_locs = {
                1: (1250, 1450), 2: (1750, 1450),
                3: (100, 800),   4: (800, 800),
                5: (1500, 800),  6: (2200, 800),
                7: (2900, 800),  8: (700, 100),
                9: (1500, 100),  10: (2300, 100),
            }
            pw = 100  # half-width: 200/2
            for pid, (px, py) in pantry_locs.items():
                tl = w2b(px - pw, py - pw)
                br = w2b(px + pw, py + pw)
                cv2.rectangle(overlay, tl, br, (50, 205, 50), 4, cv2.LINE_AA)
                ct = w2b(px, py)
                cv2.putText(overlay, f"P{pid}", (ct[0]-30, ct[1]+15),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (50, 205, 50), 3, cv2.LINE_AA)

            # Robot pose
            if self.last_robot_pose is not None:
                rp = self.last_robot_pose
                rob = w2b(rp.x * 1000, rp.y * 1000)
                head = w2b((rp.x + 0.35 * math.cos(rp.theta)) * 1000,
                           (rp.y + 0.35 * math.sin(rp.theta)) * 1000)
                cv2.circle(overlay, rob, 30, (0, 255, 255), -1, cv2.LINE_AA)
                cv2.arrowedLine(overlay, rob, head, (0, 255, 255), 8, cv2.LINE_AA, tipLength=0.25)
                cv2.putText(overlay,
                            f"BOT ({rp.x:.2f}, {rp.y:.2f}, {math.degrees(rp.theta):.0f}deg)",
                            (rob[0]+35, rob[1]-20),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)

            # Enemy pose and inflated avoidance stop-zone
            if self.last_enemy_pose is not None:
                ep = self.last_enemy_pose
                if self.avoidance_enabled:
                    avoid_pts = [w2b(x, y) for x, y in self._enemy_avoidance_rect_points_mm(ep)]
                    avoid_np = np.array(avoid_pts, dtype=np.int32)
                    avoid_overlay = overlay.copy()
                    cv2.fillPoly(avoid_overlay, [avoid_np], (0, 0, 255))
                    cv2.addWeighted(avoid_overlay, 0.18, overlay, 0.82, 0, overlay)
                    cv2.polylines(overlay, [avoid_np], True, (0, 0, 255), 5, cv2.LINE_AA)
                    front_mid = w2b((ep.x * 1000.0 + (self.bot_front_mm + self.avoidance_front_margin_mm) * math.cos(ep.theta)),
                                    (ep.y * 1000.0 + (self.bot_front_mm + self.avoidance_front_margin_mm) * math.sin(ep.theta)))
                    cv2.putText(overlay, "ENEMY STOP-ZONE",
                                (front_mid[0] + 15, front_mid[1] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3, cv2.LINE_AA)

                enemy = w2b(ep.x * 1000, ep.y * 1000)
                head = w2b((ep.x + 0.35 * math.cos(ep.theta)) * 1000,
                           (ep.y + 0.35 * math.sin(ep.theta)) * 1000)
                cv2.circle(overlay, enemy, 30, (0, 0, 255), -1, cv2.LINE_AA)
                cv2.arrowedLine(overlay, enemy, head, (0, 0, 255), 8, cv2.LINE_AA, tipLength=0.25)
                tag_txt = f"ID{self.last_enemy_tag_id} " if self.last_enemy_tag_id is not None else ""
                cv2.putText(overlay,
                            f"ENEMY {tag_txt}({ep.x:.2f}, {ep.y:.2f}, {math.degrees(ep.theta):.0f}deg)",
                            (enemy[0]+35, enemy[1]+45),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

            # Status
            cv2.putText(overlay, status_text, (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 2.5, (0, 255, 0), 5, cv2.LINE_AA)

            # Downscale 3000x2000 -> 640x427
            debug_small = cv2.resize(overlay, (640, int(self.board_height_mm * 640 / self.board_width_mm)),
                                      interpolation=cv2.INTER_AREA)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_small, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
        else:
            cv2.putText(debug, status_text, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OverheadRectifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()