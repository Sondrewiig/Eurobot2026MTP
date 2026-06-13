#!/usr/bin/env python3
"""
operator_gui.py - Eurobot Ninja operator station.

This version keeps ONE original GUI and separates commands into:

Simple operator commands:
  x 1900       axis move along X using defaults
  y 1700       axis move along Y using defaults
  diag X Y     special diagonal step move
  turnto -90   rotate in place to absolute heading
  align        enable onboard crate-align node
  pickup       tiltdown -> twocrates -> tiltup

Advanced tuning commands:
  pulsex TARGET_X [Y_LINE] [HEADING_DEG] [STEP_MM] [TOL_MM]
  pulsey TARGET_Y [X_LINE] [HEADING_DEG] [STEP_MM] [TOL_MM]

The simple x/y commands are aliases around the same internal step/pulse
movement, but with safe defaults and current line/heading.
"""

import json
import math
import threading
import time
import tkinter as tk
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool, String

try:
    import cv2
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from PIL import Image as PILImage, ImageTk
    _HAVE_IMAGES = True
except Exception:
    _HAVE_IMAGES = False


BASIC_DRIVE_HELP = (
    "BASIC DRIVE\n"
    "  enable             ARM drive. Required before any driving command\n"
    "  goto X Y [H]       rough overhead goto; H is heading_deg if given\n"
    "  forward MM         rough forward test from current heading\n"
    "  x X                simple accurate X move; keeps current y and heading\n"
    "  y Y                simple accurate Y move; keeps current x and heading\n"
    "  pulsex X [Y H STEP TOL]  advanced X move with optional tuning\n"
    "  pulsey Y [X H STEP TOL]  advanced Y move with optional tuning\n"
    "  diag X Y           special diagonal step move; use only in open space\n"
    "  turn DEG           relative turn in place\n"
    "  turnto DEG         absolute turn in place\n"
    "  cancel             cancel x/y/pulsex/pulsey/diag movement\n"
    "  stop               disable drive and send ESP stop\n"
)

OTHER_HELP = (
    "VISION / GRIPPER\n"
    "  align              enable onboard crate alignment (/ninja/align/enable)\n"
    "  alignstop          disable onboard crate alignment and stop\n"
    "  pickup             tiltdown -> twocrates -> tiltup\n"
    "  tiltup | tiltdown | twocrates | onecrate | release\n"
    "\n"
    "VLX / TOF\n"
    "  vlx                toggle VLX stream on ESP32\n"
    "  vlxstatus          one VLX status print from ESP32\n"
    "  tof on/off         use side VLX correction during x/y moves\n"
    "  tofzero            set side-wall target from current VLX5/VLX6\n"
    "  tofstatus          show VLX values and current ToF mode\n"
    "  tofside left/right select side sensor; default auto\n"
    "  tofsign 1/-1       reverse correction if it bends wrong\n"
    "\n"
    "MISC\n"
    "  esp <raw>          pass raw command to /ninja/esp32_cmd\n"
    "  pose               print current pose\n"
    "  img both/debug/topdown\n"
    "  mission <cmd>      send mission command\n"
    "  help               reprint this list\n"
)

# Axis step defaults. Keep conservative for granary/fridge approach.
AXIS_STEP_MM = 150.0
AXIS_TOL_MM = 30.0
DIAG_STEP_MM = 120.0
DIAG_TOL_MM = 35.0
STEP_TIMEOUT_S = 5.0
SETTLE_S = 0.20

# ToF side wall correction. Only active after: tofzero + tof on.
TOF_CORR_GAIN = 0.45
TOF_CORR_LIMIT_MM = 25.0


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def wrap_rad(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def wrap_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


# ----------------------------------------------------------------------
# ROS BACKEND
# ----------------------------------------------------------------------
class GuiBackend(Node):
    def __init__(self) -> None:
        super().__init__("ninja_operator_gui")

        self.lock = threading.Lock()
        self.pose: Optional[Pose2D] = None
        self.status_json: Dict[str, Any] = {}
        self.cmd_vel: Optional[Twist] = None
        self.connected: Optional[bool] = None
        self.range_json: Dict[str, Any] = {}
        self.align_status_json: Dict[str, Any] = {}
        self.debug_image_bgr = None
        self.topdown_image_bgr = None

        self.bridge = CvBridge() if _HAVE_IMAGES else None

        # Subscribers.
        self.create_subscription(Pose2D, "/ninja/pose", self._on_pose, 10)
        self.create_subscription(String, "/ninja/go_to_point_status", self._on_status, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Bool, "/ninja/connected", self._on_connected, 10)
        self.create_subscription(String, "/ninja/range_array_json", self._on_range, 10)
        self.create_subscription(String, "/ninja/vision/align_status", self._on_align_status, 10)

        if _HAVE_IMAGES:
            self.create_subscription(Image, "/overhead/debug_image", self._on_debug_image, 5)
            self.create_subscription(Image, "/overhead/topdown_image", self._on_topdown_image, 5)

        # Publishers.
        self.goal_pub = self.create_publisher(Pose2D, "/ninja/goal_pose", 10)
        self.enable_pub = self.create_publisher(Bool, "/ninja/enable_drive", 10)
        self.esp_pub = self.create_publisher(String, "/ninja/esp32_cmd", 10)
        self.mission_pub = self.create_publisher(String, "/ninja/mission", 10)
        self.align_enable_pub = self.create_publisher(Bool, "/ninja/align/enable", 10)

    # --- subscriber callbacks ---
    def _on_pose(self, msg: Pose2D) -> None:
        with self.lock:
            self.pose = msg

    def _on_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"raw": msg.data}
        with self.lock:
            self.status_json = data

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self.lock:
            self.cmd_vel = msg

    def _on_connected(self, msg: Bool) -> None:
        with self.lock:
            self.connected = bool(msg.data)

    def _on_range(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"raw": msg.data}
        with self.lock:
            self.range_json = data

    def _on_align_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"raw": msg.data}
        with self.lock:
            self.align_status_json = data

    def _on_debug_image(self, msg) -> None:
        if not _HAVE_IMAGES:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return
        with self.lock:
            self.debug_image_bgr = frame

    def _on_topdown_image(self, msg) -> None:
        if not _HAVE_IMAGES:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return
        with self.lock:
            self.topdown_image_bgr = frame

    # --- publishers ---
    def send_goal(self, x: float, y: float, theta: float) -> None:
        msg = Pose2D()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(theta)
        self.goal_pub.publish(msg)

    def send_enable(self, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        self.enable_pub.publish(msg)

    def send_align_enable(self, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        self.align_enable_pub.publish(msg)

    def send_esp(self, text: str) -> None:
        msg = String()
        msg.data = str(text)
        self.esp_pub.publish(msg)

    def send_mission(self, cmd: str) -> None:
        msg = String()
        msg.data = str(cmd)
        self.mission_pub.publish(msg)


# ----------------------------------------------------------------------
# GUI
# ----------------------------------------------------------------------
class OperatorGUI:
    BG = "#1e1e1e"
    FG = "#e6e6e6"
    DIM = "#8a8a8a"
    OK = "#5fd35f"
    WARN = "#e0a020"
    ERR = "#ff5a5a"

    def __init__(self, backend: GuiBackend) -> None:
        self.backend = backend

        self.root = tk.Tk()
        self.root.title("Ninja Operator")
        self.root.geometry("1600x900")
        self.root.minsize(1320, 760)
        self.root.configure(bg=self.BG)

        self.image_mode = "both"

        # Safety arming: operator must type `enable` before drive commands.
        # stop/cancel/disable/align and completed axis moves disarm it again.
        self.drive_armed = False

        # Main layout: full-width image band on top, controls below.
        self.main_frame = tk.Frame(self.root, bg=self.BG)
        self.main_frame.pack(side="top", fill="both", expand=True)

        self.top_frame = tk.Frame(self.main_frame, bg="#000", height=540)
        self.top_frame.pack(side="top", fill="x", padx=4, pady=(4, 2))
        self.top_frame.pack_propagate(False)

        self.bottom_frame = tk.Frame(self.main_frame, bg=self.BG)
        self.bottom_frame.pack(side="top", fill="both", expand=True)

        # Axis step state.
        self.axis_mode: Optional[str] = None
        self.axis_params: Dict[str, Any] = {}
        self.axis_waiting = False
        self.axis_step_start_s = 0.0
        self.axis_next_time_s = 0.0

        # ToF correction state.
        self.tof_enabled = False
        self.tof_side = "auto"  # auto | left | right
        self.tof_target_mm: Optional[float] = None
        self.tof_sign = 1.0

        self._build_image_area()
        self._build_status_line()
        self._build_help_panel()
        self._build_input_line()

        self._echo("ready. type 'help' for commands.", "info")
        self.root.after(100, self._tick)

    # --- layout ---
    def _build_image_area(self) -> None:
        self.image_frame = tk.Frame(self.top_frame, bg="#000")
        self.image_frame.pack(side="top", fill="both", expand=True)

        self.left_img = tk.Label(self.image_frame, bg="#000", fg="#9af", text="(no /overhead/debug_image yet)")
        self.right_img = tk.Label(self.image_frame, bg="#000", fg="#9af", text="(no /overhead/topdown_image yet)")
        self._repack_image_labels()

        self._tk_left_img = None
        self._tk_right_img = None

    def _repack_image_labels(self) -> None:
        for w in self.image_frame.winfo_children():
            w.pack_forget()
        if self.image_mode == "both":
            self.left_img.pack(side="left", fill="both", expand=True, padx=2, pady=2)
            self.right_img.pack(side="left", fill="both", expand=True, padx=2, pady=2)
        elif self.image_mode == "debug":
            self.left_img.pack(side="left", fill="both", expand=True, padx=2, pady=2)
        elif self.image_mode == "topdown":
            self.right_img.pack(side="left", fill="both", expand=True, padx=2, pady=2)

    def _build_status_line(self) -> None:
        f = tk.Frame(self.bottom_frame, bg=self.BG)
        f.pack(side="top", fill="x", padx=6, pady=(2, 0))
        self.status_label = tk.Label(f, anchor="w", justify="left", bg=self.BG, fg=self.FG, font=("Courier New", 10), text="pose: --   goal: --   cmd_vel: --   esp32: --")
        self.status_label.pack(side="top", fill="x")
        self.range_label = tk.Label(f, anchor="w", justify="left", bg=self.BG, fg=self.DIM, font=("Courier New", 10), text="range: --")
        self.range_label.pack(side="top", fill="x", pady=(2, 0))
        self.echo_label = tk.Label(f, anchor="w", justify="left", bg=self.BG, fg=self.DIM, font=("Courier New", 10), text="")
        self.echo_label.pack(side="top", fill="x", pady=(2, 0))

    def _build_help_panel(self) -> None:
        f = tk.LabelFrame(self.bottom_frame, text=" commands ", bg=self.BG, fg=self.DIM, font=("Helvetica", 9, "italic"))
        f.pack(side="top", fill="both", expand=True, padx=6, pady=(6, 0))

        body = tk.Frame(f, bg=self.BG)
        body.pack(side="top", fill="both", expand=True, padx=6, pady=4)

        left = tk.Frame(body, bg=self.BG)
        left.pack(side="left", fill="both", expand=True, padx=(0, 6))
        right = tk.Frame(body, bg=self.BG)
        right.pack(side="left", fill="both", expand=True, padx=(6, 0))

        tk.Label(left, text=BASIC_DRIVE_HELP, justify="left", anchor="nw", bg=self.BG, fg=self.FG, font=("Courier New", 9)).pack(side="top", fill="both", expand=True)
        tk.Label(right, text=OTHER_HELP, justify="left", anchor="nw", bg=self.BG, fg=self.FG, font=("Courier New", 9)).pack(side="top", fill="both", expand=True)

    def _build_input_line(self) -> None:
        f = tk.Frame(self.bottom_frame, bg=self.BG)
        f.pack(side="top", fill="x", padx=6, pady=6)
        tk.Label(f, text=">", bg=self.BG, fg=self.OK, font=("Courier New", 12, "bold")).pack(side="left")
        self.input_var = tk.StringVar()
        entry = tk.Entry(f, textvariable=self.input_var, bg="#111", fg=self.FG, insertbackground=self.FG, font=("Courier New", 11), relief="flat")
        entry.pack(side="left", fill="x", expand=True, padx=6)
        entry.bind("<Return>", lambda _e: self._on_enter())
        entry.focus_set()

    # --- echo helper ---
    def _echo(self, text: str, level: str = "info") -> None:
        color = {"info": self.DIM, "ok": self.OK, "warn": self.WARN, "err": self.ERR}.get(level, self.DIM)
        self.echo_label.configure(text=text, fg=color)

    # --- command handling ---
    def _require_drive_armed(self, command_name: str) -> bool:
        if self.drive_armed:
            return True
        self._echo(f"{command_name}: type 'enable' first, then send the drive command", "warn")
        return False

    def _on_enter(self) -> None:
        raw = self.input_var.get().strip()
        self.input_var.set("")
        if raw:
            self._handle_command(raw)

    def _handle_command(self, raw: str) -> None:
        parts = raw.split()
        cmd = parts[0].lower()
        args = parts[1:]

        try:
            if cmd == "help":
                self._echo("see commands panel below. type a command and press Enter.", "info")
                return

            if cmd == "pose":
                p = self._get_pose_copy()
                if p is None:
                    self._echo("pose: -- (no /ninja/pose yet)", "warn")
                else:
                    self._echo(f"pose: x={p.x:.1f} mm  y={p.y:.1f} mm  h={math.degrees(p.theta):.1f} deg", "ok")
                return

            if cmd == "cancel":
                self._cancel_axis(send_stop=True)
                self._echo("axis move cancelled; drive DISARMED", "ok")
                return

            if cmd == "enable":
                self.drive_armed = True
                self.backend.send_align_enable(False)
                self.backend.send_enable(True)
                self._echo("DRIVE ARMED. Next driving command may move the robot.", "ok")
                return

            if cmd == "disable":
                self.drive_armed = False
                self.backend.send_enable(False)
                self._echo("drive DISARMED", "ok")
                return

            if cmd == "stop":
                self._cancel_axis(send_stop=False)
                self.drive_armed = False
                self.backend.send_enable(False)
                self.backend.send_align_enable(False)
                self.backend.send_esp("stop")
                self._echo("→ STOP (drive=false, align=false, esp32=stop). Type enable before next drive.", "ok")
                return

            if cmd == "esp":
                if not args:
                    self._echo("usage: esp <raw command>", "err")
                    return
                payload = " ".join(args)
                self.backend.send_esp(payload)
                self._echo(f"→ /ninja/esp32_cmd: {payload}", "ok")
                return

            if cmd in ("tiltup", "tiltdown", "twocrates", "onecrate", "release", "vlx", "vlxstatus", "startposition", "neutralposition"):
                self.backend.send_esp(cmd)
                self._echo(f"→ esp {cmd}", "ok")
                return

            if cmd == "pickup":
                self._start_pickup_macro()
                return

            if cmd == "align":
                self._cancel_axis(send_stop=True)
                self.drive_armed = False
                self.backend.send_enable(False)
                self.backend.send_esp("stop")
                self.backend.send_align_enable(True)
                self._echo("→ align enabled; normal drive DISARMED", "ok")
                return

            if cmd in ("alignstop", "stopalign"):
                self.backend.send_align_enable(False)
                self.backend.send_esp("stop")
                self._echo("→ align disabled and ESP stop sent", "ok")
                return

            if cmd == "tof":
                self._handle_tof(args)
                return
            if cmd in ("tofzero", "tofstatus", "tofside", "tofsign"):
                # Short aliases so command box is quick.
                self._handle_tof([cmd[3:]] + args if cmd.startswith("tof") else args)
                return

            if cmd == "img":
                if not args or args[0] not in ("both", "debug", "topdown"):
                    self._echo("usage: img both | debug | topdown", "err")
                    return
                self.image_mode = args[0]
                self._repack_image_labels()
                self._echo(f"→ image mode: {self.image_mode}", "ok")
                return

            if cmd == "goto":
                if not self._require_drive_armed("goto"):
                    return
                if len(args) not in (2, 3):
                    self._echo("usage: goto X Y [H]", "err")
                    return
                self._cancel_axis(send_stop=False)
                x = float(args[0])
                y = float(args[1])
                if len(args) == 3:
                    theta = math.radians(float(args[2]))
                else:
                    p = self._get_pose_copy()
                    if p is None:
                        self._echo("no pose yet — cannot keep current heading. give H explicitly.", "err")
                        return
                    theta = float(p.theta)
                self.backend.send_goal(x, y, theta)
                self.backend.send_enable(True)
                self._echo(f"→ goal: x={x:.0f} y={y:.0f} h={math.degrees(theta):.1f} deg", "ok")
                return

            if cmd == "forward":
                if not self._require_drive_armed("forward"):
                    return
                if len(args) != 1:
                    self._echo("usage: forward MM", "err")
                    return
                self._cancel_axis(send_stop=False)
                mm = float(args[0])
                p = self._get_pose_copy()
                if p is None:
                    self._echo("no pose yet — cannot compute forward goal.", "err")
                    return
                gx = p.x + mm * math.cos(p.theta)
                gy = p.y + mm * math.sin(p.theta)
                self.backend.send_goal(gx, gy, float(p.theta))
                self.backend.send_enable(True)
                self._echo(f"→ forward {mm:.0f} mm → goal x={gx:.0f} y={gy:.0f} h={math.degrees(p.theta):.1f} deg", "ok")
                return

            if cmd == "x":
                if not self._require_drive_armed("x"):
                    return
                if len(args) != 1:
                    self._echo("usage: x TARGET_X", "err")
                    return
                # Simple command: use current y, current heading, default step/tolerance.
                self._start_axis_x(float(args[0]))
                return

            if cmd == "y":
                if not self._require_drive_armed("y"):
                    return
                if len(args) != 1:
                    self._echo("usage: y TARGET_Y", "err")
                    return
                # Simple command: use current x, current heading, default step/tolerance.
                self._start_axis_y(float(args[0]))
                return

            if cmd == "pulsex":
                if not self._require_drive_armed("pulsex"):
                    return
                if len(args) not in (1, 2, 3, 4, 5):
                    self._echo("usage: pulsex TARGET_X [Y_LINE] [HEADING_DEG] [STEP_MM] [TOL_MM]", "err")
                    return
                target_x = float(args[0])
                line_y = float(args[1]) if len(args) >= 2 else None
                heading_deg = float(args[2]) if len(args) >= 3 else None
                step_mm = float(args[3]) if len(args) >= 4 else None
                tol_mm = float(args[4]) if len(args) >= 5 else None
                self._start_axis_x(target_x, line_y=line_y, heading_deg=heading_deg, step_mm=step_mm, tol_mm=tol_mm, advanced=True)
                return

            if cmd == "pulsey":
                if not self._require_drive_armed("pulsey"):
                    return
                if len(args) not in (1, 2, 3, 4, 5):
                    self._echo("usage: pulsey TARGET_Y [X_LINE] [HEADING_DEG] [STEP_MM] [TOL_MM]", "err")
                    return
                target_y = float(args[0])
                line_x = float(args[1]) if len(args) >= 2 else None
                heading_deg = float(args[2]) if len(args) >= 3 else None
                step_mm = float(args[3]) if len(args) >= 4 else None
                tol_mm = float(args[4]) if len(args) >= 5 else None
                self._start_axis_y(target_y, line_x=line_x, heading_deg=heading_deg, step_mm=step_mm, tol_mm=tol_mm, advanced=True)
                return

            if cmd == "diag":
                if not self._require_drive_armed("diag"):
                    return
                if len(args) != 2:
                    self._echo("usage: diag TARGET_X TARGET_Y", "err")
                    return
                self._start_diag(float(args[0]), float(args[1]))
                return

            if cmd == "turn":
                if not self._require_drive_armed("turn"):
                    return
                if len(args) != 1:
                    self._echo("usage: turn DEG (relative)", "err")
                    return
                self._cancel_axis(send_stop=False)
                d = float(args[0])
                p = self._get_pose_copy()
                if p is None:
                    self._echo("no pose yet — cannot compute relative turn.", "err")
                    return
                new_theta = wrap_rad(p.theta + math.radians(d))
                self.backend.send_goal(p.x, p.y, new_theta)
                self.backend.send_enable(True)
                self._echo(f"→ turn {d:+.1f} deg → heading {math.degrees(new_theta):+.1f} deg", "ok")
                return

            if cmd == "turnto":
                if not self._require_drive_armed("turnto"):
                    return
                if len(args) != 1:
                    self._echo("usage: turnto DEG (absolute)", "err")
                    return
                self._cancel_axis(send_stop=False)
                d = float(args[0])
                p = self._get_pose_copy()
                if p is None:
                    self._echo("no pose yet — cannot turn to absolute heading.", "err")
                    return
                theta = math.radians(d)
                self.backend.send_goal(p.x, p.y, theta)
                self.backend.send_enable(True)
                self._echo(f"→ turnto {d:+.1f} deg", "ok")
                return

            if cmd == "mission":
                if not args:
                    self._echo("usage: mission fridge1 | fridge2 | full | abort", "err")
                    return
                payload = args[0].lower()
                if payload not in ("abort", "stop") and not self._require_drive_armed("mission"):
                    return
                self.backend.send_mission(payload)
                self._echo(f"→ /ninja/mission: {payload}", "ok")
                return

            self._echo(f"unknown command: {cmd!r}. type 'help'.", "err")

        except ValueError as e:
            self._echo(f"bad number in arguments: {e}", "err")
        except Exception as e:
            self._echo(f"error: {e}", "err")

    # ------------------------------------------------------------------
    # Clean axis commands: x, y, diag
    # ------------------------------------------------------------------
    def _start_axis_x(
        self,
        target_x: float,
        line_y: Optional[float] = None,
        heading_deg: Optional[float] = None,
        step_mm: Optional[float] = None,
        tol_mm: Optional[float] = None,
        advanced: bool = False,
    ) -> None:
        p = self._get_pose_copy()
        if p is None:
            self._echo("no pose yet — cannot start x/pulsex command", "err")
            return

        line_y = float(p.y) if line_y is None else float(line_y)
        heading = float(p.theta) if heading_deg is None else math.radians(float(heading_deg))
        step = AXIS_STEP_MM if step_mm is None else max(2.0, float(step_mm))
        tol = AXIS_TOL_MM if tol_mm is None else max(2.0, float(tol_mm))

        self._cancel_axis(send_stop=False)
        self.axis_mode = "pulsex" if advanced else "x"
        self.axis_params = {
            "target_x": float(target_x),
            "line_y": line_y,
            "heading": heading,
            "base_line": line_y,
            "tol": tol,
            "step": step,
        }
        self.axis_waiting = False
        self.axis_next_time_s = time.monotonic() + 0.05
        name = "pulsex" if advanced else "x"
        self._echo(f"{name} started: target_x={target_x:.0f}, y_line={line_y:.0f}, h={math.degrees(heading):.1f}, step={step:.0f}, tol={tol:.0f}", "ok")

    def _start_axis_y(
        self,
        target_y: float,
        line_x: Optional[float] = None,
        heading_deg: Optional[float] = None,
        step_mm: Optional[float] = None,
        tol_mm: Optional[float] = None,
        advanced: bool = False,
    ) -> None:
        p = self._get_pose_copy()
        if p is None:
            self._echo("no pose yet — cannot start y/pulsey command", "err")
            return

        line_x = float(p.x) if line_x is None else float(line_x)
        heading = float(p.theta) if heading_deg is None else math.radians(float(heading_deg))
        step = AXIS_STEP_MM if step_mm is None else max(2.0, float(step_mm))
        tol = AXIS_TOL_MM if tol_mm is None else max(2.0, float(tol_mm))

        self._cancel_axis(send_stop=False)
        self.axis_mode = "pulsey" if advanced else "y"
        self.axis_params = {
            "target_y": float(target_y),
            "line_x": line_x,
            "heading": heading,
            "base_line": line_x,
            "tol": tol,
            "step": step,
        }
        self.axis_waiting = False
        self.axis_next_time_s = time.monotonic() + 0.05
        name = "pulsey" if advanced else "y"
        self._echo(f"{name} started: target_y={target_y:.0f}, x_line={line_x:.0f}, h={math.degrees(heading):.1f}, step={step:.0f}, tol={tol:.0f}", "ok")

    def _start_diag(self, target_x: float, target_y: float) -> None:
        p = self._get_pose_copy()
        if p is None:
            self._echo("no pose yet — cannot start diag command", "err")
            return
        self._cancel_axis(send_stop=False)
        self.axis_mode = "diag"
        self.axis_params = {
            "target_x": float(target_x),
            "target_y": float(target_y),
            "heading": float(p.theta),
            "tol": DIAG_TOL_MM,
            "step": DIAG_STEP_MM,
        }
        self.axis_waiting = False
        self.axis_next_time_s = time.monotonic() + 0.05
        self._echo(f"diag move started: target=({target_x:.0f},{target_y:.0f}), h={math.degrees(p.theta):.1f}", "ok")

    def _axis_tick(self) -> None:
        if self.axis_mode is None:
            return
        now = time.monotonic()
        if now < self.axis_next_time_s:
            return
        p = self._get_pose_copy()
        if p is None:
            self._echo("axis waiting for /ninja/pose", "warn")
            return

        # If a previous short goto is still running, wait until reached or timeout.
        if self.axis_waiting:
            reason = self._status_reason()
            if reason in ("goal_reached", "no_goal_or_reached") or (now - self.axis_step_start_s) > STEP_TIMEOUT_S:
                # Keep /ninja/enable_drive armed between pulse steps.
                # Disabling here left the controller in drive_disabled with a valid goal.
                self.backend.send_esp("stop")
                self.axis_waiting = False
                self.axis_next_time_s = now + SETTLE_S
            return

        try:
            if self.axis_mode in ("x", "pulsex"):
                done = self._axis_step_x(p)
            elif self.axis_mode in ("y", "pulsey"):
                done = self._axis_step_y(p)
            elif self.axis_mode == "diag":
                done = self._axis_step_diag(p)
            else:
                done = True
        except Exception as exc:
            self._echo(f"axis error: {exc}", "err")
            done = True

        if done:
            self._cancel_axis(send_stop=True)
            self._echo("axis move done; drive DISARMED", "ok")

    def _axis_step_x(self, p: Pose2D) -> bool:
        target_x = float(self.axis_params["target_x"])
        tol = float(self.axis_params["tol"])
        if abs(target_x - p.x) <= tol:
            return True
        direction = 1.0 if target_x > p.x else -1.0
        step = min(float(self.axis_params["step"]), max(0.0, abs(target_x - p.x) - tol))
        next_x = p.x + direction * step
        line_y = float(self.axis_params["base_line"])
        line_y += self._tof_line_correction()
        theta = float(self.axis_params["heading"])
        self._start_axis_goto(next_x, line_y, theta)
        self._echo(f"x step → x={next_x:.0f}, y_line={line_y:.0f}", "info")
        return False

    def _axis_step_y(self, p: Pose2D) -> bool:
        target_y = float(self.axis_params["target_y"])
        tol = float(self.axis_params["tol"])
        if abs(target_y - p.y) <= tol:
            return True
        direction = 1.0 if target_y > p.y else -1.0
        step = min(float(self.axis_params["step"]), max(0.0, abs(target_y - p.y) - tol))
        next_y = p.y + direction * step
        line_x = float(self.axis_params["base_line"])
        line_x += self._tof_line_correction()
        theta = float(self.axis_params["heading"])
        self._start_axis_goto(line_x, next_y, theta)
        self._echo(f"y step → y={next_y:.0f}, x_line={line_x:.0f}", "info")
        return False

    def _axis_step_diag(self, p: Pose2D) -> bool:
        tx = float(self.axis_params["target_x"])
        ty = float(self.axis_params["target_y"])
        tol = float(self.axis_params["tol"])
        d = math.hypot(tx - p.x, ty - p.y)
        if d <= tol:
            return True
        step = min(float(self.axis_params["step"]), max(0.0, d - tol))
        nx = p.x + (tx - p.x) / d * step
        ny = p.y + (ty - p.y) / d * step
        theta = float(self.axis_params["heading"])
        self._start_axis_goto(nx, ny, theta)
        self._echo(f"diag step → x={nx:.0f}, y={ny:.0f}", "info")
        return False

    def _start_axis_goto(self, x: float, y: float, theta: float) -> None:
        self.backend.send_goal(float(x), float(y), float(theta))
        self.backend.send_enable(True)
        self.axis_waiting = True
        self.axis_step_start_s = time.monotonic()

    def _cancel_axis(self, send_stop: bool = True) -> None:
        self.axis_mode = None
        self.axis_params = {}
        self.axis_waiting = False
        self.axis_step_start_s = 0.0
        self.axis_next_time_s = 0.0
        if send_stop:
            self.drive_armed = False
            self.backend.send_enable(False)
            self.backend.send_esp("stop")

    def _status_reason(self) -> str:
        with self.backend.lock:
            return str(self.backend.status_json.get("reason", ""))

    # ------------------------------------------------------------------
    # ToF helpers
    # ------------------------------------------------------------------
    def _handle_tof(self, args) -> None:
        if not args:
            self._echo("usage: tof on|off|zero|status|side left/right|sign 1/-1", "err")
            return
        sub = args[0].lower()
        if sub == "on":
            self.tof_enabled = True
            self._echo("ToF correction ON. Use tofzero first if target is not set.", "ok")
            return
        if sub == "off":
            self.tof_enabled = False
            self._echo("ToF correction OFF", "ok")
            return
        if sub in ("zero", "zer", "z"):
            value = self._selected_side_distance_mm()
            if value is None:
                self._echo("no valid side VLX distance yet. run 'vlx' and wait for /ninja/range_array_json", "err")
                return
            self.tof_target_mm = float(value)
            self._echo(f"ToF target set to {value:.0f} mm from {self._selected_side_name()}", "ok")
            return
        if sub == "status":
            r = self._get_range_copy()
            self._echo(f"ToF {'ON' if self.tof_enabled else 'OFF'} side={self.tof_side} target={self.tof_target_mm} sign={self.tof_sign:+.0f} range={r}", "info")
            return
        if sub == "side":
            if len(args) < 2 or args[1].lower() not in ("auto", "left", "right"):
                self._echo("usage: tof side auto|left|right", "err")
                return
            self.tof_side = args[1].lower()
            self._echo(f"ToF side = {self.tof_side}", "ok")
            return
        if sub == "sign":
            if len(args) < 2 or args[1] not in ("1", "+1", "-1"):
                self._echo("usage: tof sign 1|-1", "err")
                return
            self.tof_sign = -1.0 if args[1] == "-1" else 1.0
            self._echo(f"ToF correction sign = {self.tof_sign:+.0f}", "ok")
            return
        self._echo("usage: tof on|off|zero|status|side|sign", "err")

    def _selected_side_name(self) -> str:
        if self.tof_side != "auto":
            return self.tof_side
        r = self._get_range_copy()
        left = self._num_or_none(r.get("left_mm"))
        right = self._num_or_none(r.get("right_mm"))
        if left is not None:
            return "left"
        if right is not None:
            return "right"
        return "left"

    def _selected_side_distance_mm(self) -> Optional[float]:
        r = self._get_range_copy()
        side = self._selected_side_name()
        if side == "right":
            return self._num_or_none(r.get("right_mm"))
        return self._num_or_none(r.get("left_mm"))

    def _tof_line_correction(self) -> float:
        if not self.tof_enabled or self.tof_target_mm is None:
            return 0.0
        current = self._selected_side_distance_mm()
        if current is None:
            return 0.0
        error = current - self.tof_target_mm
        return self.tof_sign * clamp(error * TOF_CORR_GAIN, -TOF_CORR_LIMIT_MM, TOF_CORR_LIMIT_MM)

    @staticmethod
    def _num_or_none(value) -> Optional[float]:
        try:
            v = float(value)
        except Exception:
            return None
        if v <= 0:
            return None
        return v

    # ------------------------------------------------------------------
    # Gripper / align helpers
    # ------------------------------------------------------------------
    def _start_pickup_macro(self) -> None:
        def worker():
            self.backend.send_enable(False)
            self.backend.send_align_enable(False)
            self.backend.send_esp("stop")
            time.sleep(0.20)
            self.backend.send_esp("tiltdown")
            time.sleep(1.20)
            self.backend.send_esp("twocrates")
            time.sleep(1.00)
            self.backend.send_esp("tiltup")
        threading.Thread(target=worker, daemon=True).start()
        self._echo("pickup macro started: tiltdown -> twocrates -> tiltup", "ok")

    # --- helpers ---
    def _get_pose_copy(self) -> Optional[Pose2D]:
        with self.backend.lock:
            p = self.backend.pose
            if p is None:
                return None
            copy = Pose2D()
            copy.x = p.x
            copy.y = p.y
            copy.theta = p.theta
            return copy

    def _get_range_copy(self) -> Dict[str, Any]:
        with self.backend.lock:
            return dict(self.backend.range_json)

    # --- periodic UI update ---
    def _tick(self) -> None:
        try:
            self._axis_tick()
            self._render_status()
            self._render_images()
        finally:
            self.root.after(100, self._tick)

    def _render_status(self) -> None:
        with self.backend.lock:
            pose = self.backend.pose
            status = dict(self.backend.status_json)
            cmd_vel = self.backend.cmd_vel
            connected = self.backend.connected
            rng = dict(self.backend.range_json)
            align_status = dict(self.backend.align_status_json)

        if pose is None:
            pose_s = "pose: --"
        else:
            pose_s = f"pose: x={pose.x:7.1f}  y={pose.y:7.1f}  h={math.degrees(pose.theta):6.1f}"

        goal = status.get("goal") if isinstance(status, dict) else None
        if isinstance(goal, dict):
            goal_s = f"goal: x={goal.get('x_mm', 0):7.1f}  y={goal.get('y_mm', 0):7.1f}  h={goal.get('heading_deg', 0):6.1f}"
        else:
            goal_s = "goal: --"

        if cmd_vel is None:
            cv_s = "cmd_vel: --"
        else:
            cv_s = f"cmd_vel: lin={cmd_vel.linear.x:+.3f}  ang={cmd_vel.angular.z:+.3f}"

        esp_s = "esp32: --" if connected is None else f"esp32: {'connected' if connected else 'DISCONNECTED'}"
        reason = status.get("reason", "") if isinstance(status, dict) else ""
        reason_s = f"  [{reason}]" if reason else ""
        axis_s = f"  axis={self.axis_mode}" if self.axis_mode else ""
        arm_s = "  ARMED" if self.drive_armed else "  disarmed"

        self.status_label.configure(text=f"{pose_s}   {goal_s}   {cv_s}   {esp_s}{reason_s}{axis_s}{arm_s}")

        front = rng.get("front") or rng.get("front_mm") or []
        left = rng.get("left_mm", "--")
        right = rng.get("right_mm", "--")
        fmin = rng.get("front_min_mm", "--")
        crates = rng.get("crates_in_position", False)
        align_action = align_status.get("action", "--") if isinstance(align_status, dict) else "--"
        tof_s = f"ToF={'ON' if self.tof_enabled else 'off'} target={self.tof_target_mm if self.tof_target_mm is not None else '--'} side={self.tof_side}"
        self.range_label.configure(text=f"range: front={front} fmin={fmin} left={left} right={right} crates={crates} align={align_action}   {tof_s}")


    def _render_images(self) -> None:
        if not _HAVE_IMAGES:
            return

        with self.backend.lock:
            debug = self.backend.debug_image_bgr
            topdown = self.backend.topdown_image_bgr

        if self.image_mode == "both":
            self._draw_into(self.left_img, debug, "_tk_left_img", placeholder="(no /overhead/debug_image yet)")
            self._draw_into(self.right_img, topdown, "_tk_right_img", placeholder="(no /overhead/topdown_image yet)")
        elif self.image_mode == "debug":
            self._draw_into(self.left_img, debug, "_tk_left_img", placeholder="(no /overhead/debug_image yet)")
        elif self.image_mode == "topdown":
            self._draw_into(self.right_img, topdown, "_tk_right_img", placeholder="(no /overhead/topdown_image yet)")

    def _draw_into(self, label: tk.Label, frame_bgr, attr: str, placeholder: str) -> None:
        if frame_bgr is None:
            label.configure(image="", text=placeholder)
            setattr(self, attr, None)
            return

        max_w = max(160, label.winfo_width() - 4)
        max_h = max(120, label.winfo_height() - 4)
        h, w = frame_bgr.shape[:2]
        scale = min(max_w / w, max_h / h, 1.0)
        nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
        small = cv2.resize(frame_bgr, (nw, nh))
        rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        pil = PILImage.fromarray(rgb)
        tkimg = ImageTk.PhotoImage(pil)
        setattr(self, attr, tkimg)
        label.configure(image=tkimg, text="")


def main() -> None:
    rclpy.init()
    backend = GuiBackend()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(backend), daemon=True)
    spin_thread.start()

    gui = OperatorGUI(backend)
    try:
        gui.root.mainloop()
    finally:
        try:
            backend.send_enable(False)
            backend.send_align_enable(False)
            backend.send_esp("stop")
        except Exception:
            pass
        backend.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
