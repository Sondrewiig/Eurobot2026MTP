#!/usr/bin/env python3

import json
import math
import threading
import tkinter as tk
from typing import Optional

import cv2
import numpy as np
import rclpy
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

        tags_yaml = self.get_parameter("tags_yaml").value
        self.left_debug_topic = self.get_parameter("left_debug_topic").value
        self.right_debug_topic = self.get_parameter("right_debug_topic").value
        self.beacon_pose_topic = self.get_parameter("beacon_pose_topic").value

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
        self.est_pose = None                 # ArUco estimate
        self.beacon_pose = None             # Beacon estimate
        self.cmd_vel = None
        self.drive_mode = "UNKNOWN"
        self.imu_msg = None
        self.overhead_pose = None
        self.fused_pose = None              # final combined estimate
        self.localization_status = None
        self.selected_tag = None

        self.latest_aruco_detections = []
        self.latest_camera_bricks_text = "---"

        # Latched last-good brick color sequence. ArUco detection of all 4
        # tags in a single frame is flaky (glare/blur/occlusion), so we cache
        # the most recent frame where all 4 were seen and use it as a fallback
        # for the flip commands. Expires after `latched_brick_ttl_s` seconds.
        self.latched_brick_colors = None
        self.latched_brick_time_ns = 0
        self.latched_brick_ttl_s = 15.0

        # OpenCR state
        self.opencr_connected = False
        self.opencr_status = None
        self.opencr_brick_state = None
        self.opencr_ack = None
        self.opencr_done = None
        self.opencr_error = None
        self.opencr_event = None
        self.opencr_odom_pose = None        # shown as PID estimate
        self.opencr_goal_pose = None
        self.opencr_imu_yaw_deg = None
        self.opencr_gyro_z = None

        # Debug image handling
        self.latest_left_debug_msg = None
        self.latest_right_debug_msg = None
        self.latest_debug_lock = threading.Lock()
        self.left_debug_image_tk = None
        self.right_debug_image_tk = None
        self.last_left_debug_render_ns = 0
        self.last_right_debug_render_ns = 0
        self.debug_render_period_ns = 250_000_000  # 4 Hz

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

        self.create_subscription(Image, self.left_debug_topic, self.left_debug_image_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.right_debug_topic, self.right_debug_image_cb, qos_profile_sensor_data)
        self.create_subscription(String, "/aruco/detections_json", self.aruco_detections_cb, 10)

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

        # ---------------- ROS publishers ----------------
        self.go_pub = self.create_publisher(Pose2D, "/opencr/cmd/go", 10)
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

        # ---------------- GUI ----------------
        self.root = tk.Tk()
        self.root.title("Eurobot MTP Telemetry")

        # Size the window as a fraction of the actual screen size so it fits,
        # and center it. This avoids the window being created wider than the
        # display (which caused cropping on narrower screens).
        screen_w = self.root.winfo_screenwidth()
        screen_h = self.root.winfo_screenheight()
        win_w = min(1700, max(1000, int(screen_w * 0.90)))
        win_h = min(920, max(600, int(screen_h * 0.90)))
        pos_x = max(0, (screen_w - win_w) // 2)
        pos_y = max(0, (screen_h - win_h) // 2)
        self.root.geometry(f"{win_w}x{win_h}+{pos_x}+{pos_y}")

        # Keep minsize well below typical screen dimensions so fullscreen and
        # resizing work properly on smaller displays.
        self.root.minsize(900, 560)
        self.root.resizable(True, True)
        self.root.configure(padx=10, pady=10)

        # F11 toggles fullscreen, Esc exits fullscreen
        self._is_fullscreen = False
        self.root.bind("<F11>", self._toggle_fullscreen)
        self.root.bind("<Escape>", self._exit_fullscreen)

        # Top bar: mode + opencr status
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

        self.opencr_label = tk.Label(
            top_frame,
            text="OpenCR: DISCONNECTED",
            font=("Arial", 14, "bold"),
            width=24,
            anchor="e",
        )
        self.opencr_label.pack(side="right")

        # Bottom: command area (pack before main so it stays at bottom)
        cmd_frame = tk.LabelFrame(self.root, text="Command Console", padx=8, pady=8)
        cmd_frame.pack(side="bottom", fill="x", pady=(8, 0))

        self.command_help_label = tk.Label(
            cmd_frame,
            text=(
                "Examples: go 0.3 1.8 -90 | go_mm 300 1800 -90 | go home | stop | estop | "
                "flip blue | flip yellow | flip 1 | flip 1,2,3 | "
                "set_pattern ALL_BLUE | set_pattern ALL_YELLOW | "
                "reset_odom 0.3 1.8 -90"
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

        # Dynamically update wraplength on labels as the window resizes so
        # long help/status text wraps instead of forcing the window wider.
        def _update_wrap(event):
            w = max(200, event.width - 40)
            self.command_help_label.config(wraplength=w)
            self.command_status_label.config(wraplength=w)
        cmd_frame.bind("<Configure>", _update_wrap)

        # Main content area: vertical stack
        # Row 0 = images (full width, at top, no vertical expansion)
        # Row 1 = telemetry split into 2 columns (expands to fill remaining space)
        main_frame = tk.Frame(self.root)
        main_frame.pack(side="top", fill="both", expand=True)

        # ---- Top: image row (full width, side-by-side cameras) ----
        image_row = tk.Frame(main_frame)
        image_row.pack(side="top", fill="x", anchor="n")

        # Left camera
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

        # Right camera
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

        # Keep references for container sizing
        self.left_debug_container = left_frame
        self.right_debug_container = right_frame
        self.image_row = image_row

        # ---- Middle: telemetry in two columns ----
        telemetry_outer = tk.LabelFrame(main_frame, text="Telemetry", padx=8, pady=8)
        telemetry_outer.pack(side="top", fill="both", expand=True, pady=(8, 0))

        telemetry_columns = tk.Frame(telemetry_outer)
        telemetry_columns.pack(fill="both", expand=True)
        telemetry_columns.columnconfigure(0, weight=1, uniform="telcol")
        telemetry_columns.columnconfigure(1, weight=1, uniform="telcol")
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

        # Right column: opencr / imu / topic health
        right_col = tk.Frame(telemetry_columns)
        right_col.grid(row=0, column=1, sticky="nsew", padx=(4, 0))

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

    def left_debug_image_cb(self, msg: Image):
        with self.latest_debug_lock:
            self.latest_left_debug_msg = msg

    def right_debug_image_cb(self, msg: Image):
        with self.latest_debug_lock:
            self.latest_right_debug_msg = msg

    def aruco_detections_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.latest_aruco_detections = payload.get("detections", [])
        except Exception:
            self.latest_aruco_detections = []

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

    # ---------------- Image conversion ----------------
    def rosimg_to_tk(self, msg: Image, max_w: int, max_h: int):
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
    def infer_camera_brick_states(self):
        """Return live [B/Y, B/Y, B/Y, B/Y] only if all 4 tags are seen right
        now. Also latches the result for fallback use by get_brick_colors_for_flip()."""
        brick_dets = []
        for d in self.latest_aruco_detections:
            tag_id = int(d.get("id", -1))
            if tag_id in self.brick_tag_to_color:
                brick_dets.append(d)

        if len(brick_dets) < 4:
            self.latest_camera_bricks_text = "---"
            return None

        brick_dets = sorted(brick_dets, key=lambda d: float(d.get("cx", 0.0)))
        brick_dets = brick_dets[:4]

        colors = []
        for d in brick_dets:
            tag_id = int(d["id"])
            colors.append(self.brick_tag_to_color[tag_id])

        self.latest_camera_bricks_text = " ".join(colors)

        # Latch this good reading for fallback in flip commands
        self.latched_brick_colors = list(colors)
        self.latched_brick_time_ns = self.get_clock().now().nanoseconds

        return colors

    def get_brick_colors_for_flip(self):
        """Return brick colors for use in flip/set_pattern commands.
        Prefers the live reading; falls back to the latched reading if it is
        still fresh (<= latched_brick_ttl_s seconds old)."""
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

    def publish_flip_bricks(self, desired_bricks):
        low_level = self.desired_bricks_to_low_level_indices(desired_bricks)
        if not low_level:
            self.command_feedback = "No bricks need flipping"
            return

        msg = Int32MultiArray()
        msg.data = low_level
        self.flip_seq_pub.publish(msg)
        self.command_feedback = f"SENT flip desired={desired_bricks} low_level={low_level}"

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

    def build_telemetry_text_left(self):
        """Left column: header, mode/state, localization, final pose, goal."""
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

        return "\n".join(lines)

    def build_telemetry_text_right(self):
        """Right column: OpenCR, IMU, topic health, last command."""
        lines = []
        lines.append("OpenCR:")
        lines.append(f"  Connected:   {'YES' if self.opencr_connected else 'NO'}")
        lines.append(f"  Status:      {self.opencr_status if self.opencr_status is not None else '---'}")
        lines.append(f"  Brick state: {self.opencr_brick_state if self.opencr_brick_state is not None else '---'}")
        self.infer_camera_brick_states()
        lines.append(f"  Camera bricks: {self.latest_camera_bricks_text}")

        # Show the latched (last-good) brick reading and its age, which is
        # what the flip commands will fall back on when the current frame
        # doesn't have all 4 tags.
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
            self.command_feedback = f"SENT: GO {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            return

        if head == "go_mm":
            pose = self.parse_pose_mm(tail.split())
            self.go_pub.publish(pose)
            self.command_feedback = f"SENT: GO {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            return

        if head == "stop":
            self.stop_pub.publish(Empty())
            self.command_feedback = "SENT: STOP"
            return

        if head == "estop":
            self.estop_pub.publish(Empty())
            self.command_feedback = "SENT: ESTOP"
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
            self.command_feedback = f"SENT: RESET_ODOM {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            return

        if head == "reset_odom_mm":
            pose = self.parse_pose_mm(tail.split())
            self.reset_odom_pub.publish(pose)
            self.command_feedback = f"SENT: RESET_ODOM {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            return

        if head == "set_home":
            pose = self.parse_pose_m(tail.split())
            self.set_home_pub.publish(pose)
            self.command_feedback = f"SENT: SET_HOME {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
            return

        if head == "set_home_mm":
            pose = self.parse_pose_mm(tail.split())
            self.set_home_pub.publish(pose)
            self.command_feedback = f"SENT: SET_HOME {pose.x:.3f} {pose.y:.3f} {math.degrees(pose.theta):.1f}"
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

        raise ValueError("unknown command")

    # ---------------- GUI updates ----------------
    def update_labels(self):
        self.mode_label.config(text=f"Mode: {self.drive_mode}")
        self.opencr_label.config(
            text=f"OpenCR: {'CONNECTED' if self.opencr_connected else 'DISCONNECTED'}"
        )
        self.command_status_label.config(text=self.command_feedback)

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

        # Calculate max image dimensions based on available space
        # Images span full width of main area, split in half
        row_w = max(240, self.image_row.winfo_width())
        root_h = max(400, self.root.winfo_height())
        max_w_per_image = max(120, (row_w // 2) - 20)
        # Cap image height so telemetry + command console still have room below.
        # ~35% of window height (min 180, max 380) gives larger images while
        # leaving enough space for the two-column telemetry text.
        max_img_h = max(180, min(380, int(root_h * 0.35)))

        if left_msg_to_render is not None:
            tk_img = self.rosimg_to_tk(left_msg_to_render, max_w_per_image, max_img_h)
            if tk_img is not None:
                self.left_debug_image_tk = tk_img
                self.left_debug_label.configure(image=self.left_debug_image_tk, text="")
                self.left_debug_label.image = self.left_debug_image_tk
            self.last_left_debug_render_ns = now_ns

        if right_msg_to_render is not None:
            tk_img = self.rosimg_to_tk(right_msg_to_render, max_w_per_image, max_img_h)
            if tk_img is not None:
                self.right_debug_image_tk = tk_img
                self.right_debug_label.configure(image=self.right_debug_image_tk, text="")
                self.right_debug_label.image = self.right_debug_image_tk
            self.last_right_debug_render_ns = now_ns

        # Update the two telemetry columns.
        # Preserve scroll position across refresh so the user can scroll freely
        # without being yanked back to the top every 200 ms.
        left_text = self.build_telemetry_text_left()
        left_yview = self.text_left.yview()
        self.text_left.config(state="normal")
        self.text_left.delete("1.0", tk.END)
        self.text_left.insert(tk.END, left_text)
        self.text_left.config(state="disabled")
        self.text_left.yview_moveto(left_yview[0])

        right_text = self.build_telemetry_text_right()
        right_yview = self.text_right.yview()
        self.text_right.config(state="normal")
        self.text_right.delete("1.0", tk.END)
        self.text_right.insert(tk.END, right_text)
        self.text_right.config(state="disabled")
        self.text_right.yview_moveto(right_yview[0])

        self.root.after(200, self.refresh_gui)

    # ---------------- Run / shutdown ----------------
    def _toggle_fullscreen(self, _event=None):
        self._is_fullscreen = not self._is_fullscreen
        self.root.attributes("-fullscreen", self._is_fullscreen)

    def _exit_fullscreen(self, _event=None):
        if self._is_fullscreen:
            self._is_fullscreen = False
            self.root.attributes("-fullscreen", False)

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