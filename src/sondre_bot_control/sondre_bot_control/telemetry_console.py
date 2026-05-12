#!/usr/bin/env python3

import json
import math
import os
import threading
import time
import tkinter as tk
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Bool, Empty, Float32, Int32, Int32MultiArray, String

from sondre_bot_control.tag_registry import default_tags_yaml, load_tag_registry


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class TelemetryConsole(Node):
    def __init__(self):
        super().__init__("telemetry_console")

        # ---------------- Parameters ----------------
        self.declare_parameter("tags_yaml", default_tags_yaml())
        self.declare_parameter("left_debug_topic", "/aruco/debug_image")
        self.declare_parameter("right_debug_topic", "/aruco_right/debug_image")
        self.declare_parameter("beacon_pose_topic", "/bot_pose_beacon")
        self.declare_parameter("dustpan_config_yaml", "")
        self.declare_parameter("default_image_width", 960)
        self.declare_parameter("default_image_height", 540)

        tags_yaml = self.get_parameter("tags_yaml").value
        self.left_debug_topic = self.get_parameter("left_debug_topic").value
        self.right_debug_topic = self.get_parameter("right_debug_topic").value
        self.beacon_pose_topic = self.get_parameter("beacon_pose_topic").value
        self.latest_image_width = int(self.get_parameter("default_image_width").value)
        self.latest_image_height = int(self.get_parameter("default_image_height").value)

        dustpan_cfg_path = self.get_parameter("dustpan_config_yaml").value
        self.load_dustpan_config(dustpan_cfg_path)

        try:
            self.tag_registry = load_tag_registry(tags_yaml)
            self.get_logger().info(f"Loaded {len(self.tag_registry)} tags from {tags_yaml}")
        except Exception as e:
            self.get_logger().warning(f"Failed to load tags.yaml from {tags_yaml}: {e}")
            self.tag_registry = {}

        self.brick_tag_to_color = {}
        for tag_id, info in self.tag_registry.items():
            if info.get("role") == "brick":
                color = str(info.get("brick_color", "")).upper()
                if color:
                    self.brick_tag_to_color[int(tag_id)] = color

        self.flip_uses_exclusion_logic = True

        # ---------------- ROS state ----------------
        self.state = None
        self.aruco_ids = None
        self.right_aruco_ids = None
        self.est_pose = None
        self.beacon_pose = None
        self.cmd_vel = None
        self.drive_mode = "UNKNOWN"
        self.imu_msg = None
        self.overhead_pose = None
        self.fused_pose = None
        self.localization_status = None
        self.selected_tag = None

        # Raspberry Pi health metrics from rbpi_metrics.py on /rbpi/metrics
        self.rbpi_metrics = None
        self.rbpi_metrics_rx_wall_time = None

        self.latest_aruco_detections = []
        self.latest_aruco_detections_right = []
        self.latest_camera_bricks_text = "---"
        self.latest_dustpan_text = "---"
        self.latest_dustpan_states = None
        self.latest_bricks_left_text = "---"
        self.latest_bricks_right_text = "---"
        self.latest_image_width_right = self.latest_image_width
        self.latest_image_height_right = self.latest_image_height

        self.latched_brick_colors = None
        self.latched_brick_time_ns = 0
        self.latched_brick_ttl_s = 15.0
        self.latched_dustpan_per_slot = {1: None, 2: None, 3: None, 4: None}

        # OpenCR state
        self.opencr_connected = False
        self.opencr_status = None
        self.opencr_brick_state = None
        self.opencr_ack = None
        self.opencr_done = None
        self.opencr_error = None
        self.opencr_event = None
        self.opencr_odom_pose = None
        self.opencr_goal_pose = None
        self.opencr_imu_yaw_deg = None
        self.opencr_gyro_z = None
        self.opencr_gains = None

        # Debug image handling
        self.latest_left_debug_msg = None
        self.latest_right_debug_msg = None
        self.latest_debug_lock = threading.Lock()
        self.left_debug_image_tk = None
        self.right_debug_image_tk = None
        self.last_left_debug_render_ns = 0
        self.last_right_debug_render_ns = 0
        self.debug_render_period_ns = 250_000_000

        # Command UI state
        self.last_command_text = "---"
        self.command_feedback = "Ready"

        self.marker_names = {}
        for tag_id, info in self.tag_registry.items():
            self.marker_names[int(tag_id)] = info.get("name", f"id_{tag_id}")

        # ---------------- ROS subscriptions ----------------
        self.create_subscription(String, "/bot_state", self.state_cb, 10)
        self.create_subscription(Int32MultiArray, "/aruco_ids", self.aruco_cb, 10)
        self.create_subscription(Int32MultiArray, "/aruco_right/ids", self.right_aruco_cb, 10)

        self.create_subscription(Pose2D, "/bot_pose_estimate", self.est_cb, 10)
        self.create_subscription(Pose2D, "/bot_pose_fused", self.fused_cb, 10)
        self.create_subscription(Pose2D, "/vision/robot_pose", self.overhead_pose_cb, 10)
        self.create_subscription(Pose2D, self.beacon_pose_topic, self.beacon_pose_cb, 10)

        self.create_subscription(String, "/localization_status", self.loc_status_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(String, "/drive_mode", self.drive_mode_cb, 10)
        self.create_subscription(Imu, "/imu", self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(String, "/aruco_selected_tag", self.selected_tag_cb, 10)

        # Pi metrics subscriber
        self.create_subscription(String, "/rbpi/metrics", self.rbpi_metrics_cb, 10)

        self.create_subscription(Image, self.left_debug_topic, self.left_debug_image_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.right_debug_topic, self.right_debug_image_cb, qos_profile_sensor_data)
        self.create_subscription(String, "/aruco/detections_json", self.aruco_detections_cb, 10)
        self.create_subscription(String, "/aruco_right/detections_json", self.aruco_right_detections_cb, 10)

        self.create_subscription(Bool, "/opencr/connected", self.opencr_connected_cb, 10)
        self.create_subscription(String, "/opencr/status", self.opencr_status_cb, 10)
        self.create_subscription(String, "/opencr/brick_state", self.opencr_brick_state_cb, 10)
        self.create_subscription(String, "/opencr/ack", self.opencr_ack_cb, 10)
        self.create_subscription(String, "/opencr/done", self.opencr_done_cb, 10)
        self.create_subscription(String, "/opencr/error", self.opencr_error_cb, 10)
        self.create_subscription(String, "/opencr/event", self.opencr_event_cb, 10)
        self.create_subscription(Pose2D, "/opencr/odom_pose", self.opencr_odom_pose_cb, 10)
        self.create_subscription(Pose2D, "/opencr/goal_pose", self.opencr_goal_pose_cb, 10)
        self.create_subscription(Float32, "/opencr/imu_yaw_deg", self.opencr_imu_yaw_deg_cb, 10)
        self.create_subscription(Float32, "/opencr/gyro_z", self.opencr_gyro_z_cb, 10)
        self.create_subscription(String, "/opencr/gains", self.opencr_gains_cb, 10)

        # ---------------- ROS publishers ----------------
        self.go_pub = self.create_publisher(Pose2D, "/opencr/cmd/go", 10)
        self.go_center_pub = self.create_publisher(Pose2D, "/opencr/cmd/go_center", 10)
        self.go_dustpan_pub = self.create_publisher(Pose2D, "/opencr/cmd/go_dustpan", 10)
        self.go_home_pub = self.create_publisher(Empty, "/opencr/cmd/go_home", 10)
        self.stop_pub = self.create_publisher(Empty, "/opencr/cmd/stop", 10)
        self.estop_pub = self.create_publisher(Empty, "/opencr/cmd/estop", 10)
        self.flip_pub = self.create_publisher(Int32, "/opencr/cmd/flip", 10)
        self.flip_seq_pub = self.create_publisher(Int32MultiArray, "/opencr/cmd/flip_seq", 10)
        self.set_pattern_pub = self.create_publisher(String, "/opencr/cmd/set_pattern", 10)
        self.set_bricks_pub = self.create_publisher(String, "/opencr/cmd/set_bricks", 10)
        self.reset_odom_pub = self.create_publisher(Pose2D, "/opencr/cmd/reset_odom", 10)
        self.set_home_pub = self.create_publisher(Pose2D, "/opencr/cmd/set_home", 10)
        self.get_state_pub = self.create_publisher(Empty, "/opencr/cmd/get_state", 10)
        self.telem_hz_pub = self.create_publisher(Int32, "/opencr/cmd/telemetry_hz", 10)
        self.raw_command_pub = self.create_publisher(String, "/opencr/cmd/raw", 10)

        # ---------------- GUI ----------------
        self.root = tk.Tk()
        self.root.title("Eurobot MTP Telemetry")

        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        win_w = min(1700, max(1000, int(screen_w * 0.90)))
        win_h = min(920, max(600, int(screen_h * 0.90)))
        pos_x = max(0, (screen_w - win_w) // 2)
        pos_y = max(0, (screen_h - win_h) // 2)
        self.root.geometry(f"{win_w}x{win_h}+{pos_x}+{pos_y}")

        self.root.minsize(900, 560)
        self.root.resizable(True, True)
        self.root.configure(padx=10, pady=10)

        self._is_fullscreen = False
        self.root.bind("<F11>", self._toggle_fullscreen)
        self.root.bind("<Escape>", self._exit_fullscreen)

        top_frame = tk.Frame(self.root)
        top_frame.pack(side="top", fill="x", pady=(0, 8))

        self.mode_label = tk.Label(
            top_frame,
            text="Mode: UNKNOWN",
            font=("Arial", 16, "bold"),
            width=24,
            anchor="w",
        )
        self.mode_label.pack(side="left")

        self.show_slot_overlay = True
        self.overlay_button = tk.Button(
            top_frame,
            text="Slot Overlay: ON",
            width=18,
            command=self._toggle_slot_overlay,
        )
        self.overlay_button.pack(side="left", padx=(16, 0))

        self.opencr_label = tk.Label(
            top_frame,
            text="OpenCR: DISCONNECTED",
            font=("Arial", 14, "bold"),
            width=24,
            anchor="e",
        )
        self.opencr_label.pack(side="right")

        cmd_frame = tk.LabelFrame(self.root, text="Command Console", padx=8, pady=8)
        cmd_frame.pack(side="bottom", fill="x", pady=(8, 0))

        self.command_help_label = tk.Label(
            cmd_frame,
            text=(
                "Examples: go 0.3 1.8 -90 | go_mm 300 1800 -90 | "
                "go_center 0.3 1.8 -90 | go_center_mm 300 1800 -90 | "
                "go_dustpan 0.3 1.8 -90 | go_dustpan_mm 300 1800 -90 | "
                "go home | stop | estop | "
                "flip blue | flip yellow | flip 1 | flip 1,2,3 | "
                "set_pattern ALL_BLUE | set_pattern ALL_YELLOW | "
                "reset_odom 0.3 1.8 -90 | "
                "get_gains | set_k_yaw 0.8 | set_yaw_min 0.03 | "
                "set_yaw_max 0.20 | set_yaw_tol_deg 3.0 | set_pos_tol 15 | "
                "push_out | pull_in | carwash_arm 90 | "
                "carwash_spin_positive | carwash_spin_negative | carwash_spin_stop"
            ),
            justify="left",
            anchor="w",
        )
        self.command_help_label.pack(fill="x", pady=(0, 6))

        entry_row = tk.Frame(cmd_frame)
        entry_row.pack(fill="x", pady=(0, 6))

        self.command_entry = tk.Entry(entry_row, font=("Courier New", 13))
        self.command_entry.pack(side="left", fill="x", expand=True)
        self.command_entry.bind("<Return>", self.on_command_enter)

        self.send_button = tk.Button(
            entry_row,
            text="Send",
            width=10,
            command=self.send_command_from_entry,
        )
        self.send_button.pack(side="left", padx=(12, 0))

        self.command_status_label = tk.Label(
            cmd_frame,
            text="Ready",
            justify="left",
            anchor="w",
            font=("Arial", 10),
        )
        self.command_status_label.pack(fill="x")

        def _update_wrap(event):
            w = max(200, event.width - 40)
            self.command_help_label.config(wraplength=w)
            self.command_status_label.config(wraplength=w)

        cmd_frame.bind("<Configure>", _update_wrap)

        main_frame = tk.Frame(self.root)
        main_frame.pack(side="top", fill="both", expand=True)

        image_row = tk.Frame(main_frame)
        image_row.pack(side="top", fill="x", anchor="n")

        left_frame = tk.LabelFrame(image_row, text="Left Detection Overlay", padx=4, pady=4)
        left_frame.pack(side="left", fill="x", expand=True, padx=(0, 4))

        self.left_debug_title = tk.Label(
            left_frame,
            text=self.left_debug_topic,
            font=("Arial", 10, "bold"),
        )
        self.left_debug_title.pack(anchor="w")

        self.left_debug_label = tk.Label(
            left_frame,
            text="No left debug image yet",
            bg="black",
            fg="white",
            anchor="center",
        )
        self.left_debug_label.pack(anchor="n", pady=(2, 0))

        right_frame = tk.LabelFrame(image_row, text="Right Detection Overlay", padx=4, pady=4)
        right_frame.pack(side="left", fill="x", expand=True, padx=(4, 0))

        self.right_debug_title = tk.Label(
            right_frame,
            text=self.right_debug_topic,
            font=("Arial", 10, "bold"),
        )
        self.right_debug_title.pack(anchor="w")

        self.right_debug_label = tk.Label(
            right_frame,
            text="No right debug image yet",
            bg="black",
            fg="white",
            anchor="center",
        )
        self.right_debug_label.pack(anchor="n", pady=(2, 0))

        self.left_debug_container = left_frame
        self.right_debug_container = right_frame
        self.image_row = image_row

        telemetry_outer = tk.LabelFrame(main_frame, text="Telemetry", padx=8, pady=8)
        telemetry_outer.pack(side="top", fill="both", expand=True, pady=(8, 0))

        telemetry_columns = tk.Frame(telemetry_outer)
        telemetry_columns.pack(fill="both", expand=True)
        telemetry_columns.columnconfigure(0, weight=1, uniform="telcol")
        telemetry_columns.columnconfigure(1, weight=1, uniform="telcol")
        telemetry_columns.columnconfigure(2, weight=1, uniform="telcol")
        telemetry_columns.rowconfigure(0, weight=1)

        # Left column: general / localization / goal
        left_col = tk.Frame(telemetry_columns)
        left_col.grid(row=0, column=0, sticky="nsew", padx=(0, 4))

        left_scroll = tk.Scrollbar(left_col)
        left_scroll.pack(side="right", fill="y")

        self.text_left = tk.Text(
            left_col,
            font=("Courier New", 10),
            state="disabled",
            wrap="word",
            yscrollcommand=left_scroll.set,
        )
        self.text_left.pack(side="left", fill="both", expand=True)
        left_scroll.config(command=self.text_left.yview)

        # Middle column: Raspberry Pi health
        pi_col = tk.Frame(telemetry_columns)
        pi_col.grid(row=0, column=1, sticky="nsew", padx=(4, 4))

        pi_scroll = tk.Scrollbar(pi_col)
        pi_scroll.pack(side="right", fill="y")

        self.text_pi = tk.Text(
            pi_col,
            font=("Courier New", 10),
            state="disabled",
            wrap="word",
            yscrollcommand=pi_scroll.set,
        )
        self.text_pi.pack(side="left", fill="both", expand=True)
        pi_scroll.config(command=self.text_pi.yview)

        # Right column: OpenCR / IMU / topic health
        right_col = tk.Frame(telemetry_columns)
        right_col.grid(row=0, column=2, sticky="nsew", padx=(4, 0))

        right_scroll = tk.Scrollbar(right_col)
        right_scroll.pack(side="right", fill="y")

        self.text_right = tk.Text(
            right_col,
            font=("Courier New", 10),
            state="disabled",
            wrap="word",
            yscrollcommand=right_scroll.set,
        )
        self.text_right.pack(side="left", fill="both", expand=True)
        right_scroll.config(command=self.text_right.yview)

        self.command_entry.focus_set()

        self.root.after(200, self.refresh_gui)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.get_logger().info("Telemetry GUI started")

    # ---------------- Dustpan / brick-slot config ----------------
    def load_dustpan_config(self, path: str):
        default_cfg = {
            "y_threshold": 0.75,
            "brick_slot_ranges": {
                1: (0.00, 0.25),
                2: (0.25, 0.50),
                3: (0.50, 0.75),
                4: (0.75, 1.00),
            },
        }

        self.dustpan_cfg = {
            "left": {
                "y_threshold": default_cfg["y_threshold"],
                "brick_slot_ranges": dict(default_cfg["brick_slot_ranges"]),
            },
            "right": {
                "y_threshold": default_cfg["y_threshold"],
                "brick_slot_ranges": dict(default_cfg["brick_slot_ranges"]),
            },
        }

        if not path:
            self.get_logger().info(
                "No dustpan_config_yaml parameter set; using defaults for both lenses"
            )
            return

        if not os.path.isfile(path):
            self.get_logger().warning(
                f"Dustpan config file not found: {path} -- using defaults"
            )
            return

        try:
            with open(path, "r") as f:
                cfg = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().warning(
                f"Failed to parse dustpan config {path}: {e} -- using defaults"
            )
            return

        for lens in ("left", "right"):
            lens_cfg = cfg.get(lens, {}) or {}

            y_thr = lens_cfg.get("y_threshold")
            if y_thr is not None:
                try:
                    self.dustpan_cfg[lens]["y_threshold"] = float(y_thr)
                except (TypeError, ValueError):
                    self.get_logger().warning(
                        f"Invalid {lens}.y_threshold in {path}; keeping default"
                    )

            slot_cfg = lens_cfg.get("brick_slots", {}) or {}
            for slot in (1, 2, 3, 4):
                raw = slot_cfg.get(slot, slot_cfg.get(str(slot)))
                if raw is None:
                    continue
                try:
                    lo, hi = float(raw[0]), float(raw[1])
                    if lo >= hi:
                        raise ValueError(f"lo ({lo}) >= hi ({hi})")
                    self.dustpan_cfg[lens]["brick_slot_ranges"][slot] = (lo, hi)
                except Exception as e:
                    self.get_logger().warning(
                        f"Invalid {lens}.brick_slots.{slot} in {path}: {e}; keeping default"
                    )

        self.get_logger().info(
            f"Loaded dustpan config from {path}: "
            f"left={self.dustpan_cfg['left']}, right={self.dustpan_cfg['right']}"
        )

    def assign_brick_to_slot(self, cx: float, lens: str) -> Optional[int]:
        if lens == "right":
            w = max(1, self.latest_image_width_right)
        else:
            w = max(1, self.latest_image_width)

        cx_frac = cx / w

        for slot, (lo, hi) in self.dustpan_cfg[lens]["brick_slot_ranges"].items():
            if lo <= cx_frac < hi:
                return slot

        return None

    # ---------------- ROS callbacks ----------------
    def state_cb(self, msg: String):
        self.state = msg.data

    def aruco_cb(self, msg: Int32MultiArray):
        self.aruco_ids = list(msg.data)

    def right_aruco_cb(self, msg: Int32MultiArray):
        self.right_aruco_ids = list(msg.data)

    def est_cb(self, msg: Pose2D):
        self.est_pose = msg

    def fused_cb(self, msg: Pose2D):
        self.fused_pose = msg

    def overhead_pose_cb(self, msg: Pose2D):
        self.overhead_pose = msg

    def beacon_pose_cb(self, msg: Pose2D):
        self.beacon_pose = msg

    def loc_status_cb(self, msg: String):
        self.localization_status = msg.data

    def cmd_cb(self, msg: Twist):
        self.cmd_vel = msg

    def drive_mode_cb(self, msg: String):
        self.drive_mode = msg.data
        self.root.after(0, self.update_labels)

    def imu_cb(self, msg: Imu):
        self.imu_msg = msg

    def selected_tag_cb(self, msg: String):
        self.selected_tag = msg.data

    def rbpi_metrics_cb(self, msg: String):
        try:
            self.rbpi_metrics = json.loads(msg.data)
            self.rbpi_metrics_rx_wall_time = time.time()
        except Exception as e:
            self.rbpi_metrics = {
                "status": "ERR",
                "warnings": [f"Failed to parse /rbpi/metrics: {e}"],
            }
            self.rbpi_metrics_rx_wall_time = time.time()

    def left_debug_image_cb(self, msg: Image):
        with self.latest_debug_lock:
            self.latest_left_debug_msg = msg

        if msg.height > 0:
            self.latest_image_height = int(msg.height)
        if msg.width > 0:
            self.latest_image_width = int(msg.width)

    def right_debug_image_cb(self, msg: Image):
        with self.latest_debug_lock:
            self.latest_right_debug_msg = msg

        if msg.height > 0:
            self.latest_image_height_right = int(msg.height)
        if msg.width > 0:
            self.latest_image_width_right = int(msg.width)

    def aruco_detections_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.latest_aruco_detections = payload.get("detections", [])
        except Exception:
            self.latest_aruco_detections = []

    def aruco_right_detections_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.latest_aruco_detections_right = payload.get("detections", [])
        except Exception:
            self.latest_aruco_detections_right = []

    def opencr_connected_cb(self, msg: Bool):
        self.opencr_connected = bool(msg.data)
        self.root.after(0, self.update_labels)

    def opencr_status_cb(self, msg: String):
        self.opencr_status = msg.data

    def opencr_brick_state_cb(self, msg: String):
        self.opencr_brick_state = msg.data

    def opencr_ack_cb(self, msg: String):
        self.opencr_ack = msg.data
        self.command_feedback = f"ACK: {msg.data}"

    def opencr_done_cb(self, msg: String):
        self.opencr_done = msg.data
        self.command_feedback = f"DONE: {msg.data}"

    def opencr_error_cb(self, msg: String):
        self.opencr_error = msg.data
        self.command_feedback = f"ERR: {msg.data}"

    def opencr_event_cb(self, msg: String):
        self.opencr_event = msg.data
        self.command_feedback = f"EVENT: {msg.data}"

    def opencr_odom_pose_cb(self, msg: Pose2D):
        self.opencr_odom_pose = msg

    def opencr_goal_pose_cb(self, msg: Pose2D):
        self.opencr_goal_pose = msg

    def opencr_imu_yaw_deg_cb(self, msg: Float32):
        self.opencr_imu_yaw_deg = float(msg.data)

    def opencr_gyro_z_cb(self, msg: Float32):
        self.opencr_gyro_z = float(msg.data)

    def opencr_gains_cb(self, msg: String):
        gains = {}
        for token in msg.data.split():
            if "=" in token:
                key, val = token.split("=", 1)
                gains[key] = val
        if gains:
            self.opencr_gains = gains

    # ---------------- Image conversion ----------------
    def _draw_slot_overlay(self, img, lens: str):
        h, w = img.shape[:2]
        cfg = self.dustpan_cfg.get(lens)
        if not cfg:
            return

        slot_color_rgb = (0, 220, 0)

        for _, (lo, hi) in cfg["brick_slot_ranges"].items():
            x1 = int(round(lo * w))
            x2 = int(round(hi * w))
            cv2.line(img, (x1, 0), (x1, h - 1), slot_color_rgb, thickness=1)
            cv2.line(img, (x2, 0), (x2, h - 1), slot_color_rgb, thickness=1)

        for slot, (lo, hi) in cfg["brick_slot_ranges"].items():
            cx = int(round((lo + hi) / 2.0 * w))
            cv2.putText(
                img,
                str(slot),
                (max(0, cx - 6), 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                slot_color_rgb,
                2,
                cv2.LINE_AA,
            )

        y_line = int(round(cfg["y_threshold"] * h))
        y_line = max(0, min(h - 1, y_line))
        cv2.line(img, (0, y_line), (w - 1, y_line), slot_color_rgb, thickness=2)

    def rosimg_to_tk(self, msg: Image, max_w: int, max_h: int, lens: str = "left"):
        try:
            enc = msg.encoding.lower()
            buf = np.frombuffer(msg.data, dtype=np.uint8)

            if enc == "rgb8":
                row_pixels = msg.step // 3
                img = buf.reshape((msg.height, row_pixels, 3))[:, :msg.width, :]

            elif enc == "bgr8":
                row_pixels = msg.step // 3
                img = buf.reshape((msg.height, row_pixels, 3))[:, :msg.width, :]
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            elif enc == "rgba8":
                row_pixels = msg.step // 4
                img = buf.reshape((msg.height, row_pixels, 4))[:, :msg.width, :]
                img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)

            elif enc == "bgra8":
                row_pixels = msg.step // 4
                img = buf.reshape((msg.height, row_pixels, 4))[:, :msg.width, :]
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)

            elif enc == "mono8":
                img = buf.reshape((msg.height, msg.step))[:, :msg.width]
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

            else:
                self.get_logger().warning(f"Unsupported debug image encoding: {msg.encoding}")
                return None

            img = np.ascontiguousarray(img)

            if self.show_slot_overlay:
                self._draw_slot_overlay(img, lens)

            h, w = img.shape[:2]
            scale = min(max_w / w, max_h / h)
            new_w = max(1, int(w * scale))
            new_h = max(1, int(h * scale))

            if new_w != w or new_h != h:
                interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
                img = cv2.resize(img, (new_w, new_h), interpolation=interp)

            header = f"P6 {new_w} {new_h} 255 ".encode("ascii")
            ppm_data = header + img.tobytes()
            return tk.PhotoImage(data=ppm_data, format="PPM")

        except Exception as e:
            self.get_logger().warning(f"Failed to convert debug image for GUI: {e}")
            return None

    # ---------------- Brick inference ----------------
    def _assign_detections_to_slots(self, detections, lens: str):
        per_slot = {}

        for d in detections:
            tag_id = int(d.get("id", -1))
            if tag_id not in self.brick_tag_to_color:
                continue

            slot = self.assign_brick_to_slot(float(d.get("cx", 0.0)), lens)
            if slot is None:
                continue

            prev = per_slot.get(slot)
            if prev is None or float(d.get("cy", 0.0)) > float(prev.get("cy", 0.0)):
                per_slot[slot] = d

        return per_slot

    def _merge_per_slot(self, left_slots, right_slots):
        merged = {}

        for slot in (1, 2, 3, 4):
            left_d = left_slots.get(slot)
            right_d = right_slots.get(slot)

            if left_d is not None:
                merged[slot] = (left_d, "left")
            elif right_d is not None:
                merged[slot] = (right_d, "right")

        return merged

    def _slots_to_colors_text(self, per_slot_detections):
        parts = []

        for slot in (1, 2, 3, 4):
            d = per_slot_detections.get(slot)
            if d is None:
                parts.append("?")
            else:
                parts.append(self.brick_tag_to_color[int(d["id"])])

        return " ".join(parts)

    def update_dustpan_states(self):
        left_slots = self._assign_detections_to_slots(
            self.latest_aruco_detections, "left"
        )
        right_slots = self._assign_detections_to_slots(
            self.latest_aruco_detections_right, "right"
        )

        self.latest_bricks_left_text = self._slots_to_colors_text(left_slots) if left_slots else "---"
        self.latest_bricks_right_text = self._slots_to_colors_text(right_slots) if right_slots else "---"

        merged = self._merge_per_slot(left_slots, right_slots)

        if not merged:
            self.latest_dustpan_states = None
            self.latest_dustpan_text = "---"
            return None

        now_ns = self.get_clock().now().nanoseconds

        states = []
        for slot in (1, 2, 3, 4):
            entry = merged.get(slot)

            if entry is None:
                states.append("?")
                continue

            d, lens = entry

            if lens == "right":
                img_h = max(1, self.latest_image_height_right)
            else:
                img_h = max(1, self.latest_image_height)

            threshold_px = self.dustpan_cfg[lens]["y_threshold"] * img_h
            cy = float(d.get("cy", 0.0))
            state = "ON" if cy >= threshold_px else "OFF"

            states.append(state)
            self.latched_dustpan_per_slot[slot] = (state, now_ns)

        self.latest_dustpan_states = states
        self.latest_dustpan_text = " ".join(states)

        return states

    def get_effective_dustpan_states(self):
        now_ns = self.get_clock().now().nanoseconds
        live = self.latest_dustpan_states or ["?"] * 4

        effective = []
        sources = []

        for i, slot in enumerate((1, 2, 3, 4)):
            live_state = live[i] if i < len(live) else "?"

            if live_state in ("ON", "OFF"):
                effective.append(live_state)
                sources.append("live")
                continue

            latched = self.latched_dustpan_per_slot.get(slot)
            if latched is None:
                effective.append("?")
                sources.append("unknown")
                continue

            state, t_ns = latched
            age_s = (now_ns - t_ns) / 1e9

            if age_s > self.latched_brick_ttl_s:
                effective.append("?")
                sources.append(f"stale {age_s:.1f}s")
            else:
                effective.append(state)
                sources.append(f"latched {age_s:.1f}s")

        return effective, sources

    def infer_camera_brick_states(self):
        self.update_dustpan_states()

        left_slots = self._assign_detections_to_slots(
            self.latest_aruco_detections, "left"
        )
        right_slots = self._assign_detections_to_slots(
            self.latest_aruco_detections_right, "right"
        )

        merged = self._merge_per_slot(left_slots, right_slots)
        merged_detections = {slot: entry[0] for slot, entry in merged.items()}

        if any(merged_detections.get(slot) is None for slot in (1, 2, 3, 4)):
            partial = self._slots_to_colors_text(merged_detections)
            self.latest_camera_bricks_text = partial if "?" in partial and any(
                p != "?" for p in partial.split()
            ) else ("---" if partial == "? ? ? ?" else partial)
            return None

        colors = [
            self.brick_tag_to_color[int(merged_detections[slot]["id"])]
            for slot in (1, 2, 3, 4)
        ]

        self.latest_camera_bricks_text = " ".join(colors)

        self.latched_brick_colors = list(colors)
        self.latched_brick_time_ns = self.get_clock().now().nanoseconds

        return colors

    def get_brick_colors_for_flip(self):
        live = self.infer_camera_brick_states()
        if live is not None:
            return live, "live"

        if self.latched_brick_colors is None:
            return None, "none"

        age_s = (self.get_clock().now().nanoseconds - self.latched_brick_time_ns) / 1e9
        if age_s > self.latched_brick_ttl_s:
            return None, "stale"

        return list(self.latched_brick_colors), f"latched {age_s:.1f}s"

    def desired_bricks_to_low_level_indices(self, desired_bricks):
        desired_set = sorted(set(int(v) for v in desired_bricks if 1 <= int(v) <= 4))

        if not desired_set:
            return []

        if not self.flip_uses_exclusion_logic:
            return desired_set

        return [i for i in [1, 2, 3, 4] if i not in desired_set]

    def _filter_by_dustpan(self, desired_bricks):
        effective, _ = self.get_effective_dustpan_states()

        kept = []
        skipped_off = []

        for b in desired_bricks:
            idx = int(b)
            state = effective[idx - 1] if 1 <= idx <= 4 else "?"

            if state == "OFF":
                skipped_off.append(idx)
            else:
                kept.append(idx)

        return kept, skipped_off

    def publish_flip_bricks(self, desired_bricks):
        kept, skipped_off = self._filter_by_dustpan(desired_bricks)

        if not kept:
            if skipped_off:
                self.command_feedback = (
                    f"No bricks to flip (all targets OFF dustpan: {skipped_off})"
                )
            else:
                self.command_feedback = "No bricks need flipping"
            return

        low_level = self.desired_bricks_to_low_level_indices(kept)

        if not low_level:
            self.command_feedback = "No bricks need flipping"
            return

        msg = Int32MultiArray()
        msg.data = low_level
        self.flip_seq_pub.publish(msg)

        feedback = f"SENT flip desired={kept} low_level={low_level}"
        if skipped_off:
            feedback += f" SKIPPED_OFF={skipped_off}"

        self.command_feedback = feedback

    def parse_brick_index_csv(self, text):
        items = [s.strip() for s in text.replace(" ", "").split(",") if s.strip()]

        if not items:
            raise ValueError("need comma-separated brick indices")

        values = [int(v) for v in items]

        for v in values:
            if v < 1 or v > 4:
                raise ValueError("brick indices must be 1..4")

        return sorted(set(values))

    # ---------------- Formatting ----------------
    def fmt_pose(self, pose):
        if pose is None:
            return "x=---   y=---   yaw=---"
        return f"x={pose.x: .3f}   y={pose.y: .3f}   yaw={math.degrees(pose.theta): .1f} deg"

    def fmt5(self, value):
        return f"{value:.5f}"

    def fmt_bytes(self, value):
        if value is None:
            return "---"

        try:
            n = float(value)
        except Exception:
            return "---"

        units = ["B", "KB", "MB", "GB", "TB"]
        for unit in units:
            if abs(n) < 1024.0:
                if unit == "B":
                    return f"{n:.0f} {unit}"
                return f"{n:.1f} {unit}"
            n /= 1024.0

        return f"{n:.1f} PB"

    def fmt_percent(self, value):
        if value is None:
            return "---"

        try:
            return f"{float(value):.1f}%"
        except Exception:
            return "---"

    def fmt_rate(self, value):
        if value is None:
            return "---"

        return f"{self.fmt_bytes(value)}/s"

    def fmt_uptime(self, seconds):
        if seconds is None:
            return "---"

        try:
            total = int(float(seconds))
        except Exception:
            return "---"

        days = total // 86400
        total %= 86400
        hours = total // 3600
        total %= 3600
        minutes = total // 60

        if days > 0:
            return f"{days}d {hours}h {minutes}m"
        if hours > 0:
            return f"{hours}h {minutes}m"
        return f"{minutes}m"

    def imu_status(self):
        if self.imu_msg is None:
            return "---"

        gz = abs(self.imu_msg.angular_velocity.z)
        ax = abs(self.imu_msg.linear_acceleration.x)
        ay = abs(self.imu_msg.linear_acceleration.y)

        if gz > 0.15:
            return "TURNING"
        if ax > 0.20 or ay > 0.20:
            return "ACCELERATING"
        return "STEADY"

    def fmt_aruco_ids(self):
        if self.aruco_ids is None:
            return "---"
        if len(self.aruco_ids) == 0:
            return "none"

        parts = []
        for marker_id in self.aruco_ids:
            name = self.marker_names.get(marker_id, f"id_{marker_id}")
            parts.append(f"{marker_id} ({name})")

        return ", ".join(parts)

    def build_rbpi_metrics_text(self):
        lines = []
        lines.append("Raspberry Pi Health:")
        lines.append("")

        m = self.rbpi_metrics

        if m is None:
            lines.append("  Waiting for /rbpi/metrics...")
            lines.append("")
            lines.append("Expected publisher:")
            lines.append("  ros2 run sondre_bot_control rbpi_metrics")
            return "\n".join(lines)

        age_s = None
        if self.rbpi_metrics_rx_wall_time is not None:
            age_s = time.time() - self.rbpi_metrics_rx_wall_time

        status = str(m.get("status", "---"))
        if age_s is not None and age_s > 3.0:
            status += " / STALE"

        lines.append(f"Status:       {status}")
        if age_s is not None:
            lines.append(f"Age:          {age_s:.1f}s")

        warnings = m.get("warnings") or []
        if warnings:
            lines.append("")
            lines.append("Warnings:")
            for warning in warnings[:8]:
                lines.append(f"  ! {warning}")
        else:
            lines.append("Warnings:     none")

        lines.append("")
        lines.append("Power / throttle:")

        throttle = m.get("throttle") or {}
        lines.append(f"  Throttled:  {throttle.get('hex', '---')}")

        now_flags = []
        if throttle.get("undervoltage_now"):
            now_flags.append("UNDERVOLTAGE")
        if throttle.get("throttled_now"):
            now_flags.append("THROTTLED")
        if throttle.get("freq_capped_now"):
            now_flags.append("FREQ CAPPED")
        if throttle.get("soft_temp_limit_now"):
            now_flags.append("TEMP LIMIT")

        seen_flags = []
        if throttle.get("undervoltage_seen"):
            seen_flags.append("undervoltage")
        if throttle.get("throttled_seen"):
            seen_flags.append("throttled")
        if throttle.get("freq_capped_seen"):
            seen_flags.append("freq capped")
        if throttle.get("soft_temp_limit_seen"):
            seen_flags.append("temp limit")

        lines.append(f"  Now:        {', '.join(now_flags) if now_flags else 'OK'}")
        lines.append(f"  Since boot: {', '.join(seen_flags) if seen_flags else 'OK'}")

        input_voltage = m.get("input_voltage_v")
        if input_voltage is not None:
            lines.append(f"  Input 5V:   {input_voltage:.3f} V")
        else:
            lines.append("  Input 5V:   ---")

        core_volts = m.get("core_volts")
        if core_volts is not None:
            lines.append(f"  Core volts: {core_volts:.3f} V")
        else:
            lines.append("  Core volts: ---")

        lines.append("")
        lines.append("CPU:")

        temp = m.get("cpu_temp_c")
        clock = m.get("cpu_clock_mhz")
        cpu = m.get("cpu") or {}

        lines.append(f"  Temp:       {temp:.1f} C" if temp is not None else "  Temp:       ---")
        lines.append(f"  Clock:      {clock:.0f} MHz" if clock is not None else "  Clock:      ---")
        lines.append(f"  Usage:      {self.fmt_percent(cpu.get('total_percent'))}")

        per_core = cpu.get("per_core_percent") or {}
        if per_core:
            parts = []
            for name in sorted(per_core.keys(), key=lambda x: int(x.replace("cpu", ""))):
                parts.append(f"{name}:{per_core[name]:.0f}%")
            lines.append("  Per-core:   " + "  ".join(parts))
        else:
            lines.append("  Per-core:   ---")

        load = m.get("load") or {}
        lines.append(
            "  Load avg:   "
            f"{load.get('load_1m', 0.0):.2f}, "
            f"{load.get('load_5m', 0.0):.2f}, "
            f"{load.get('load_15m', 0.0):.2f}"
        )

        lines.append("")
        lines.append("Memory:")

        mem = m.get("memory") or {}
        lines.append(
            f"  RAM:        {self.fmt_bytes(mem.get('ram_used_bytes'))} / "
            f"{self.fmt_bytes(mem.get('ram_total_bytes'))} "
            f"({self.fmt_percent(mem.get('ram_used_percent'))})"
        )
        lines.append(f"  Available:  {self.fmt_bytes(mem.get('ram_available_bytes'))}")
        lines.append(
            f"  Swap:       {self.fmt_bytes(mem.get('swap_used_bytes'))} / "
            f"{self.fmt_bytes(mem.get('swap_total_bytes'))} "
            f"({self.fmt_percent(mem.get('swap_used_percent'))})"
        )

        lines.append("")
        lines.append("Disk:")

        disk = m.get("disk") or {}
        lines.append(
            f"  Root:       {self.fmt_bytes(disk.get('root_used_bytes'))} / "
            f"{self.fmt_bytes(disk.get('root_total_bytes'))} "
            f"({self.fmt_percent(disk.get('root_used_percent'))})"
        )
        lines.append(f"  Free:       {self.fmt_bytes(disk.get('root_free_bytes'))}")

        lines.append("")
        lines.append("Uptime:")

        uptime = m.get("uptime") or {}
        lines.append(f"  {self.fmt_uptime(uptime.get('uptime_seconds'))}")

        lines.append("")
        lines.append("Network:")

        net = m.get("network") or {}
        if net:
            for ifname, values in sorted(net.items()):
                lines.append(
                    f"  {ifname}: RX {self.fmt_rate(values.get('rx_rate_bytes_s'))}  "
                    f"TX {self.fmt_rate(values.get('tx_rate_bytes_s'))}"
                )

                err = (
                    int(values.get("rx_errors", 0))
                    + int(values.get("tx_errors", 0))
                    + int(values.get("rx_dropped", 0))
                    + int(values.get("tx_dropped", 0))
                )

                if err:
                    lines.append(
                        f"    errors/drop: rx_err={values.get('rx_errors', 0)} "
                        f"tx_err={values.get('tx_errors', 0)} "
                        f"rx_drop={values.get('rx_dropped', 0)} "
                        f"tx_drop={values.get('tx_dropped', 0)}"
                    )
        else:
            lines.append("  ---")

        lines.append("")
        lines.append("Kernel warnings:")

        kernel = m.get("kernel") or {}
        if kernel.get("available"):
            lines.append(f"  Storage:    {kernel.get('storage_error_count', 0)}")
            lines.append(f"  OOM:        {kernel.get('oom_error_count', 0)}")
            lines.append(f"  Power:      {kernel.get('power_error_count', 0)}")
        else:
            lines.append("  dmesg unavailable")

        lines.append("")
        lines.append("Top CPU processes:")

        top = m.get("top_processes") or []
        if top:
            for proc in top[:6]:
                lines.append(
                    f"  {proc.get('command', '---')[:14]:14s} "
                    f"CPU {proc.get('cpu_percent', 0.0):5.1f}%  "
                    f"MEM {proc.get('mem_percent', 0.0):4.1f}%"
                )
        else:
            lines.append("  ---")

        return "\n".join(lines)

    def build_telemetry_text_left(self):
        lines = []

        lines.append("=== EUROBOT MTP TELEMETRY ===")
        lines.append("")
        lines.append(f"Mode:         {self.drive_mode}")
        lines.append(f"Bot state:    {self.state if self.state is not None else '---'}")
        lines.append(f"ArUco IDs:    {self.fmt_aruco_ids()}")
        lines.append(f"Selected tag: {self.selected_tag if self.selected_tag is not None else '---'}")
        lines.append("")

        lines.append("Localization estimates:")
        lines.append(f"  ArUco estimate:   {self.fmt_pose(self.est_pose)}")
        lines.append(f"  PID estimate:     {self.fmt_pose(self.opencr_odom_pose)}")
        lines.append(f"  Overhead estimate:{self.fmt_pose(self.overhead_pose)}")
        lines.append(f"  Beacon estimate:  {self.fmt_pose(self.beacon_pose)}")
        lines.append(f"  Loc status:       {self.localization_status if self.localization_status is not None else '---'}")

        lines.append("")
        lines.append("Final pose estimate:")
        lines.append(f"  Combined pose:    {self.fmt_pose(self.fused_pose)}")

        lines.append("")
        lines.append("Goal position:")
        lines.append(f"  Goal pose:        {self.fmt_pose(self.opencr_goal_pose)}")

        lines.append("")
        lines.append("Control gains:")

        if self.opencr_gains is not None:
            g = self.opencr_gains
            lines.append(f"  K_rho:     {g.get('K_RHO', '---')}")
            lines.append(f"  K_alpha:   {g.get('K_ALPHA', '---')}")
            lines.append(f"  K_alpha_i: {g.get('K_ALPHA_I', '---')}")
            lines.append(f"  K_yaw:     {g.get('K_YAW', '---')}")
            lines.append(f"  Pos tol:   {g.get('POS_TOL_MM', '---')} mm")
            lines.append(f"  Yaw tol:   {g.get('YAW_TOL_DEG', '---')} deg")
            lines.append(f"  Yaw min:   {g.get('YAW_MIN', '---')} rad/s")
            lines.append(f"  Yaw max:   {g.get('YAW_MAX', '---')} rad/s")
        else:
            lines.append("  (type 'get_gains' to fetch)")

        return "\n".join(lines)

    def build_telemetry_text_right(self):
        lines = []

        lines.append("OpenCR:")
        lines.append(f"  Connected:   {'YES' if self.opencr_connected else 'NO'}")
        lines.append(f"  Status:      {self.opencr_status if self.opencr_status is not None else '---'}")
        lines.append(f"  Brick state: {self.opencr_brick_state if self.opencr_brick_state is not None else '---'}")

        self.infer_camera_brick_states()

        lines.append(f"  Bricks L:      {self.latest_bricks_left_text}")
        lines.append(f"  Bricks R:      {self.latest_bricks_right_text}")
        lines.append(f"  Camera bricks: {self.latest_camera_bricks_text}")
        lines.append(f"  Dustpan:       {self.latest_dustpan_text}")

        effective, sources = self.get_effective_dustpan_states()

        marked = []
        for st, src in zip(effective, sources):
            if src.startswith("latched"):
                marked.append(f"{st}*")
            else:
                marked.append(st)

        lines.append(f"  Dustpan eff:   {' '.join(marked)}  (* = latched)")

        if self.latched_brick_colors is not None:
            age_s = (self.get_clock().now().nanoseconds - self.latched_brick_time_ns) / 1e9
            latched_str = " ".join(self.latched_brick_colors)

            if age_s <= self.latched_brick_ttl_s:
                lines.append(f"  Latched:      {latched_str}  ({age_s:.1f}s ago, FRESH)")
            else:
                lines.append(f"  Latched:      {latched_str}  ({age_s:.1f}s ago, STALE)")
        else:
            lines.append("  Latched:      ---")

        if self.opencr_imu_yaw_deg is not None:
            lines.append(f"  IMU yaw:     {self.opencr_imu_yaw_deg: .1f} deg")
        else:
            lines.append("  IMU yaw:     ---")

        if self.opencr_gyro_z is not None:
            lines.append(f"  Gyro z:      {self.opencr_gyro_z: .5f}")
        else:
            lines.append("  Gyro z:      ---")

        lines.append(f"  ACK:         {self.opencr_ack if self.opencr_ack is not None else '---'}")
        lines.append(f"  DONE:        {self.opencr_done if self.opencr_done is not None else '---'}")
        lines.append(f"  ERROR:       {self.opencr_error if self.opencr_error is not None else '---'}")
        lines.append(f"  EVENT:       {self.opencr_event if self.opencr_event is not None else '---'}")

        if self.cmd_vel is not None:
            lines.append("")
            lines.append(f"Cmd Vel:      vx={self.cmd_vel.linear.x: .3f}   wz={self.cmd_vel.angular.z: .3f}")

        lines.append("")
        lines.append("IMU:")

        if self.imu_msg is not None:
            lines.append(
                f"  Accel:       ax={self.fmt5(self.imu_msg.linear_acceleration.x)}   "
                f"ay={self.fmt5(self.imu_msg.linear_acceleration.y)}   "
                f"az={self.fmt5(self.imu_msg.linear_acceleration.z)}"
            )
            lines.append(
                f"  Gyro:        gx={self.fmt5(self.imu_msg.angular_velocity.x)}   "
                f"gy={self.fmt5(self.imu_msg.angular_velocity.y)}   "
                f"gz={self.fmt5(self.imu_msg.angular_velocity.z)}"
            )
            lines.append(f"  Status:      {self.imu_status()}")
        else:
            lines.append("  ---")

        lines.append("")
        lines.append("Topic health:")
        lines.append(f"  left debug image    {'OK' if self.left_debug_image_tk is not None else '---'}")
        lines.append(f"  right debug image   {'OK' if self.right_debug_image_tk is not None else '---'}")
        lines.append(f"  /aruco_ids          {'OK' if self.aruco_ids is not None else '---'}")
        lines.append(f"  /bot_pose_estimate  {'OK' if self.est_pose is not None else '---'}")
        lines.append(f"  /opencr/connected   {'OK' if self.opencr_connected else '---'}")
        lines.append(f"  /opencr/brick_state {'OK' if self.opencr_brick_state is not None else '---'}")
        lines.append(f"  /rbpi/metrics       {'OK' if self.rbpi_metrics is not None else '---'}")

        lines.append("")
        lines.append(f"Last command:  {self.last_command_text}")
        lines.append(f"Last feedback: {self.command_feedback}")

        return "\n".join(lines)

    # ---------------- Command parsing / sending ----------------
    def on_command_enter(self, _event):
        self.send_command_from_entry()

    def parse_pose_m(self, tokens):
        if len(tokens) != 3:
            raise ValueError("expected x y yaw_deg")

        x_m = float(tokens[0])
        y_m = float(tokens[1])
        yaw_deg = float(tokens[2])

        msg = Pose2D()
        msg.x = x_m
        msg.y = y_m
        msg.theta = math.radians(yaw_deg)

        return msg

    def parse_pose_mm(self, tokens):
        if len(tokens) != 3:
            raise ValueError("expected x_mm y_mm yaw_deg")

        x_mm = float(tokens[0])
        y_mm = float(tokens[1])
        yaw_deg = float(tokens[2])

        msg = Pose2D()
        msg.x = x_mm / 1000.0
        msg.y = y_mm / 1000.0
        msg.theta = math.radians(yaw_deg)

        return msg

    def send_command_from_entry(self):
        cmd = self.command_entry.get().strip()

        if not cmd:
            self.command_feedback = "PARSE ERROR: empty command"
            self.command_status_label.config(text=self.command_feedback)
            return

        try:
            self.handle_command_text(cmd)
            self.last_command_text = cmd

            if not self.command_feedback.startswith(("ACK:", "DONE:", "ERR:", "EVENT:", "SENT:")):
                self.command_feedback = f"SENT: {cmd}"

            self.command_entry.delete(0, tk.END)

        except Exception as e:
            self.command_feedback = f"PARSE ERROR: {e}"

        self.command_status_label.config(text=self.command_feedback)

    def publish_raw_opencr_command(self, text: str):
        msg = String()
        msg.data = text.strip()
        self.raw_command_pub.publish(msg)

    def parse_single_float(self, text: str) -> float:
        text = text.strip()

        if not text:
            raise ValueError("expected numeric value")

        return float(text)

    def handle_command_text(self, cmd: str):
        cmd = cmd.strip()
        cmd_lower = cmd.lower()

        if cmd_lower in ("go home", "go_home", "home"):
            self.go_home_pub.publish(Empty())
            self.command_feedback = "SENT: GO_HOME"
            return

        parts = cmd.split(maxsplit=1)
        head = parts[0].lower()
        tail = parts[1].strip() if len(parts) > 1 else ""

        if head == "go":
            if tail.lower() in ("home", "go_home"):
                self.go_home_pub.publish(Empty())
                self.command_feedback = "SENT: GO_HOME"
                return

            pose = self.parse_pose_m(tail.split())
            self.go_pub.publish(pose)
            self.command_feedback = (
                f"SENT: GO {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            )
            return

        if head == "go_mm":
            pose = self.parse_pose_mm(tail.split())
            self.go_pub.publish(pose)
            self.command_feedback = (
                f"SENT: GO {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            )
            return

        if head == "go_center":
            pose = self.parse_pose_m(tail.split())
            self.go_center_pub.publish(pose)
            self.command_feedback = (
                f"SENT: GO_CENTER {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            )
            return

        if head == "go_center_mm":
            pose = self.parse_pose_mm(tail.split())
            self.go_center_pub.publish(pose)
            self.command_feedback = (
                f"SENT: GO_CENTER {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            )
            return

        if head == "go_dustpan":
            pose = self.parse_pose_m(tail.split())
            self.go_dustpan_pub.publish(pose)
            self.command_feedback = (
                f"SENT: GO_DUSTPAN {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            )
            return

        if head == "go_dustpan_mm":
            pose = self.parse_pose_mm(tail.split())
            self.go_dustpan_pub.publish(pose)
            self.command_feedback = (
                f"SENT: GO_DUSTPAN {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            )
            return

        if head == "stop":
            self.stop_pub.publish(Empty())
            self.command_feedback = "SENT: STOP"
            return

        if head == "estop":
            self.estop_pub.publish(Empty())
            self.command_feedback = "SENT: ESTOP"
            return

        if head == "get_state":
            self.get_state_pub.publish(Empty())
            self.command_feedback = "SENT: GET_STATE"
            return

        if head == "telem":
            hz = int(tail)
            msg = Int32()
            msg.data = hz
            self.telem_hz_pub.publish(msg)
            self.command_feedback = f"SENT: TELEM {hz}"
            return

        if head == "get_gains":
            self.publish_raw_opencr_command("GET_GAINS")
            self.command_feedback = "SENT: GET_GAINS"
            return

        if head == "set_k_rho":
            v = self.parse_single_float(tail)
            self.publish_raw_opencr_command(f"SET_K_RHO {v}")
            self.command_feedback = f"SENT: SET_K_RHO {v}"
            return

        if head == "set_k_alpha":
            v = self.parse_single_float(tail)
            self.publish_raw_opencr_command(f"SET_K_ALPHA {v}")
            self.command_feedback = f"SENT: SET_K_ALPHA {v}"
            return

        if head == "set_k_alpha_i":
            v = self.parse_single_float(tail)
            self.publish_raw_opencr_command(f"SET_K_ALPHA_I {v}")
            self.command_feedback = f"SENT: SET_K_ALPHA_I {v}"
            return

        if head == "set_k_yaw":
            v = self.parse_single_float(tail)
            self.publish_raw_opencr_command(f"SET_K_YAW {v}")
            self.command_feedback = f"SENT: SET_K_YAW {v}"
            return

        if head == "set_pos_tol":
            v = self.parse_single_float(tail)
            self.publish_raw_opencr_command(f"SET_POS_TOL {v}")
            self.command_feedback = f"SENT: SET_POS_TOL {v}"
            return

        if head == "set_yaw_tol_deg":
            v = self.parse_single_float(tail)
            self.publish_raw_opencr_command(f"SET_YAW_TOL_DEG {v}")
            self.command_feedback = f"SENT: SET_YAW_TOL_DEG {v}"
            return

        if head == "set_yaw_min":
            v = self.parse_single_float(tail)
            self.publish_raw_opencr_command(f"SET_YAW_MIN {v}")
            self.command_feedback = f"SENT: SET_YAW_MIN {v}"
            return

        if head == "set_yaw_max":
            v = self.parse_single_float(tail)
            self.publish_raw_opencr_command(f"SET_YAW_MAX {v}")
            self.command_feedback = f"SENT: SET_YAW_MAX {v}"
            return

        if head == "push_out":
            self.publish_raw_opencr_command("PUSH_OUT")
            self.command_feedback = "SENT: PUSH_OUT"
            return

        if head == "pull_in":
            self.publish_raw_opencr_command("PULL_IN")
            self.command_feedback = "SENT: PULL_IN"
            return

        if head == "carwash_arm":
            v = int(self.parse_single_float(tail))
            self.publish_raw_opencr_command(f"CARWASH_ARM {v}")
            self.command_feedback = f"SENT: CARWASH_ARM {v}"
            return

        if head == "carwash_spin_positive":
            self.publish_raw_opencr_command("CARWASH_SPIN_POSITIVE")
            self.command_feedback = "SENT: CARWASH_SPIN_POSITIVE"
            return

        if head == "carwash_spin_negative":
            self.publish_raw_opencr_command("CARWASH_SPIN_NEGATIVE")
            self.command_feedback = "SENT: CARWASH_SPIN_NEGATIVE"
            return

        if head == "carwash_spin_stop":
            self.publish_raw_opencr_command("CARWASH_SPIN_STOP")
            self.command_feedback = "SENT: CARWASH_SPIN_STOP"
            return

        if head == "flip":
            tail_lower = tail.lower().strip()

            if tail_lower == "blue":
                colors, source = self.get_brick_colors_for_flip()
                if colors is None:
                    raise ValueError("need all 4 brick tags visible for 'flip blue' (no recent detection)")
                desired_bricks = [i + 1 for i, c in enumerate(colors) if c != "BLUE"]
                self.publish_flip_bricks(desired_bricks)
                self.command_feedback += f" [{source}]"
                return

            if tail_lower == "yellow":
                colors, source = self.get_brick_colors_for_flip()
                if colors is None:
                    raise ValueError("need all 4 brick tags visible for 'flip yellow' (no recent detection)")
                desired_bricks = [i + 1 for i, c in enumerate(colors) if c != "YELLOW"]
                self.publish_flip_bricks(desired_bricks)
                self.command_feedback += f" [{source}]"
                return

            if "," in tail:
                desired_bricks = self.parse_brick_index_csv(tail)
                self.publish_flip_bricks(desired_bricks)
                return

            n = int(tail)
            if n < 1 or n > 4:
                raise ValueError("brick index must be 1..4")

            self.publish_flip_bricks([n])
            return

        if head == "flip_seq":
            desired_bricks = self.parse_brick_index_csv(tail)
            self.publish_flip_bricks(desired_bricks)
            return

        if head == "set_pattern":
            tail_upper = tail.upper().replace(" ", "")

            if tail_upper in ("ALL_BLUE", "ALLBLUE"):
                colors, source = self.get_brick_colors_for_flip()
                if colors is None:
                    raise ValueError("need all 4 brick tags visible for ALL_BLUE (no recent detection)")
                desired_bricks = [i + 1 for i, c in enumerate(colors) if c != "BLUE"]
                self.publish_flip_bricks(desired_bricks)
                self.command_feedback += f" [{source}]"
                return

            if tail_upper in ("ALL_YELLOW", "ALLYELLOW"):
                colors, source = self.get_brick_colors_for_flip()
                if colors is None:
                    raise ValueError("need all 4 brick tags visible for ALL_YELLOW (no recent detection)")
                desired_bricks = [i + 1 for i, c in enumerate(colors) if c != "YELLOW"]
                self.publish_flip_bricks(desired_bricks)
                self.command_feedback += f" [{source}]"
                return

            msg = String()
            msg.data = tail
            self.set_pattern_pub.publish(msg)
            self.command_feedback = f"SENT: SET_PATTERN {tail}"
            return

        if head == "set_bricks":
            msg = String()
            msg.data = tail
            self.set_bricks_pub.publish(msg)
            self.command_feedback = f"SENT: SET_BRICKS {tail}"
            return

        if head == "reset_odom":
            pose = self.parse_pose_m(tail.split())
            self.reset_odom_pub.publish(pose)
            self.command_feedback = (
                f"SENT: RESET_ODOM {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            )
            return

        if head == "reset_odom_mm":
            pose = self.parse_pose_mm(tail.split())
            self.reset_odom_pub.publish(pose)
            self.command_feedback = (
                f"SENT: RESET_ODOM {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            )
            return

        if head == "set_home":
            pose = self.parse_pose_m(tail.split())
            self.set_home_pub.publish(pose)
            self.command_feedback = (
                f"SENT: SET_HOME {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            )
            return

        if head == "set_home_mm":
            pose = self.parse_pose_mm(tail.split())
            self.set_home_pub.publish(pose)
            self.command_feedback = (
                f"SENT: SET_HOME {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            )
            return

        raise ValueError("unknown command")

    # ---------------- GUI updates ----------------
    def update_labels(self):
        self.mode_label.config(text=f"Mode: {self.drive_mode}")
        self.opencr_label.config(
            text=f"OpenCR: {'CONNECTED' if self.opencr_connected else 'DISCONNECTED'}"
        )
        self.command_status_label.config(text=self.command_feedback)

    def update_text_box(self, widget: tk.Text, new_text: str):
        """Update a disabled Text widget while preserving the user's scroll position."""
        try:
            top_index = widget.index("@0,0")
            at_top = widget.yview()[0] <= 0.001
            at_bottom = widget.yview()[1] >= 0.999
        except tk.TclError:
            top_index = "1.0"
            at_top = True
            at_bottom = False

        widget.config(state="normal")
        widget.delete("1.0", tk.END)
        widget.insert(tk.END, new_text)
        widget.config(state="disabled")

        try:
            if at_top:
                widget.yview_moveto(0.0)
            elif at_bottom:
                widget.yview_moveto(1.0)
            else:
                widget.yview(top_index)
        except tk.TclError:
            widget.yview_moveto(0.0)

    def refresh_gui(self):
        self.update_labels()

        now_ns = self.get_clock().now().nanoseconds

        left_msg_to_render: Optional[Image] = None
        right_msg_to_render: Optional[Image] = None

        with self.latest_debug_lock:
            if (
                self.latest_left_debug_msg is not None
                and now_ns - self.last_left_debug_render_ns >= self.debug_render_period_ns
            ):
                left_msg_to_render = self.latest_left_debug_msg

            if (
                self.latest_right_debug_msg is not None
                and now_ns - self.last_right_debug_render_ns >= self.debug_render_period_ns
            ):
                right_msg_to_render = self.latest_right_debug_msg

        row_w = max(240, self.image_row.winfo_width())
        root_h = max(400, self.root.winfo_height())
        max_w_per_image = max(120, (row_w // 2) - 20)
        max_img_h = max(180, min(380, int(root_h * 0.35)))

        if left_msg_to_render is not None:
            tk_img = self.rosimg_to_tk(left_msg_to_render, max_w_per_image, max_img_h, lens="left")
            if tk_img is not None:
                self.left_debug_image_tk = tk_img
                self.left_debug_label.configure(image=self.left_debug_image_tk, text="")
                self.left_debug_label.image = self.left_debug_image_tk
            self.last_left_debug_render_ns = now_ns

        if right_msg_to_render is not None:
            tk_img = self.rosimg_to_tk(right_msg_to_render, max_w_per_image, max_img_h, lens="right")
            if tk_img is not None:
                self.right_debug_image_tk = tk_img
                self.right_debug_label.configure(image=self.right_debug_image_tk, text="")
                self.right_debug_label.image = self.right_debug_image_tk
            self.last_right_debug_render_ns = now_ns

        # Left telemetry
        self.update_text_box(self.text_left, self.build_telemetry_text_left())
        self.update_text_box(self.text_pi, self.build_rbpi_metrics_text())
        self.update_text_box(self.text_right, self.build_telemetry_text_right())

        self.root.after(200, self.refresh_gui)

    # ---------------- Run / shutdown ----------------
    def _toggle_fullscreen(self, _event=None):
        self._is_fullscreen = not self._is_fullscreen
        self.root.attributes("-fullscreen", self._is_fullscreen)

    def _exit_fullscreen(self, _event=None):
        if self._is_fullscreen:
            self._is_fullscreen = False
            self.root.attributes("-fullscreen", False)

    def _toggle_slot_overlay(self):
        self.show_slot_overlay = not self.show_slot_overlay
        state_str = "ON" if self.show_slot_overlay else "OFF"
        self.overlay_button.config(text=f"Slot Overlay: {state_str}")

        self.last_left_debug_render_ns = 0
        self.last_right_debug_render_ns = 0

    def on_close(self):
        self.root.quit()
        self.root.destroy()

    def run(self):
        ros_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        ros_thread.start()
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryConsole()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()