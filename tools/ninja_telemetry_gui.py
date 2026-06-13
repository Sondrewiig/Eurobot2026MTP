#!/usr/bin/env python3
"""
Step 33 - Ninja telemetry/control GUI for Eurobot overhead + Ninja Pi tests.

Run on the overhead laptop or on the Ninja Pi after sourcing ROS 2.
It replaces several helper terminals by showing pose/status/cmd/ESP telemetry and
publishing safe test commands/goals.

Topics used:
  SUB /ninja/pose                      geometry_msgs/Pose2D
  SUB /ninja/go_to_point_status         std_msgs/String JSON
  SUB /cmd_vel                          geometry_msgs/Twist
  SUB /ninja/telemetry                  std_msgs/String
  SUB /ninja/connected                  std_msgs/Bool
  PUB /ninja/esp32_cmd                  std_msgs/String
  PUB /ninja/goal_pose                  geometry_msgs/Pose2D

Important: this GUI does not replace the Fast DDS discovery server, overhead
camera node, ninja_pose_from_overhead.py, esp32_bridge.py, or go_to_point_node.py.
It only makes monitoring and sending test commands easier.
"""

import json
import math
import threading
import time
import tkinter as tk
from tkinter import ttk
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Pose2D, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


def now_s() -> float:
    return time.monotonic()


def deg(rad: float) -> float:
    return math.degrees(rad)


def fmt_age(stamp: Optional[float]) -> str:
    if stamp is None:
        return "never"
    return f"{now_s() - stamp:.2f}s"


def parse_float(text: str, fallback: float) -> float:
    try:
        return float(text.strip())
    except Exception:
        return fallback


class NinjaTelemetryGui(Node):
    def __init__(self):
        super().__init__("ninja_telemetry_gui")

        self.lock = threading.Lock()

        self.pose: Optional[Pose2D] = None
        self.pose_time: Optional[float] = None

        self.status_raw = "---"
        self.status_json: Dict[str, Any] = {}
        self.status_time: Optional[float] = None

        self.cmd_vel: Optional[Twist] = None
        self.cmd_time: Optional[float] = None

        self.telemetry_raw = "---"
        self.telemetry_lines = []
        self.telemetry_time: Optional[float] = None

        self.connected: Optional[bool] = None
        self.connected_time: Optional[float] = None

        self.last_action = "Ready"

        self.create_subscription(Pose2D, "/ninja/pose", self.pose_cb, 10)
        self.create_subscription(String, "/ninja/go_to_point_status", self.status_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.create_subscription(String, "/ninja/telemetry", self.telemetry_cb, 10)
        self.create_subscription(Bool, "/ninja/connected", self.connected_cb, 10)

        self.raw_pub = self.create_publisher(String, "/ninja/esp32_cmd", 10)
        self.goal_pub = self.create_publisher(Pose2D, "/ninja/goal_pose", 10)

        self.root = tk.Tk()
        self.root.title("Ninja Telemetry + Safe Test GUI")
        self.root.geometry("1080x760")
        self.root.minsize(950, 640)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.colors = {
            "ok": "#0a7d25",
            "warn": "#b07800",
            "bad": "#b00020",
            "idle": "#404040",
        }

        self._build_gui()
        self.root.after(150, self.refresh_gui)

    # ---------------- ROS callbacks ----------------
    def pose_cb(self, msg: Pose2D):
        with self.lock:
            self.pose = msg
            self.pose_time = now_s()

    def status_cb(self, msg: String):
        with self.lock:
            self.status_raw = msg.data
            self.status_time = now_s()
            try:
                parsed = json.loads(msg.data)
                self.status_json = parsed if isinstance(parsed, dict) else {}
            except Exception:
                self.status_json = {}

    def cmd_cb(self, msg: Twist):
        with self.lock:
            self.cmd_vel = msg
            self.cmd_time = now_s()

    def telemetry_cb(self, msg: String):
        with self.lock:
            self.telemetry_raw = msg.data
            self.telemetry_time = now_s()
            self.telemetry_lines.append(msg.data)
            self.telemetry_lines = self.telemetry_lines[-12:]

    def connected_cb(self, msg: Bool):
        with self.lock:
            self.connected = bool(msg.data)
            self.connected_time = now_s()

    # ---------------- GUI layout ----------------
    def _build_gui(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(3, weight=1)

        title = ttk.Label(
            self.root,
            text="Ninja Telemetry + Safe Test GUI",
            font=("Arial", 18, "bold"),
        )
        title.grid(row=0, column=0, sticky="w", padx=12, pady=(10, 4))

        subtitle = ttk.Label(
            self.root,
            text="Use this instead of many echo/pub terminals. Keep discovery server, overhead camera, pose publisher, bridge, and go-to-point running.",
        )
        subtitle.grid(row=1, column=0, sticky="w", padx=12, pady=(0, 8))

        top = ttk.Frame(self.root)
        top.grid(row=2, column=0, sticky="nsew", padx=12)
        top.columnconfigure(0, weight=1)
        top.columnconfigure(1, weight=1)
        top.columnconfigure(2, weight=1)

        self._build_connection_panel(top).grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=4)
        self._build_pose_status_panel(top).grid(row=0, column=1, sticky="nsew", padx=6, pady=4)
        self._build_cmd_panel(top).grid(row=0, column=2, sticky="nsew", padx=(6, 0), pady=4)

        middle = ttk.Frame(self.root)
        middle.grid(row=3, column=0, sticky="nsew", padx=12, pady=6)
        middle.columnconfigure(0, weight=1)
        middle.columnconfigure(1, weight=1)
        middle.rowconfigure(0, weight=1)

        self._build_safe_test_panel(middle).grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self._build_goal_panel(middle).grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        bottom = ttk.LabelFrame(self.root, text="ESP32 / Bridge telemetry")
        bottom.grid(row=4, column=0, sticky="nsew", padx=12, pady=(4, 10))
        bottom.columnconfigure(0, weight=1)
        bottom.rowconfigure(0, weight=1)

        self.telemetry_text = tk.Text(bottom, height=8, wrap="word")
        self.telemetry_text.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)
        self.telemetry_text.configure(state="disabled")

        self.action_var = tk.StringVar(value="Ready")
        action = ttk.Label(self.root, textvariable=self.action_var, font=("Arial", 11, "bold"))
        action.grid(row=5, column=0, sticky="w", padx=12, pady=(0, 10))

    def _build_connection_panel(self, parent):
        box = ttk.LabelFrame(parent, text="Node/topic health")
        box.columnconfigure(1, weight=1)

        self.health_vars = {}
        labels = [
            ("pose_pub", "/ninja/pose publishers"),
            ("goal_sub", "/ninja/goal_pose subscribers"),
            ("esp_sub", "/ninja/esp32_cmd subscribers"),
            ("cmd_pub", "/cmd_vel publishers"),
            ("cmd_sub", "/cmd_vel subscribers"),
            ("status_pub", "/ninja/status publishers"),
            ("connected", "ESP connected"),
        ]
        for r, (key, label) in enumerate(labels):
            ttk.Label(box, text=label + ":").grid(row=r, column=0, sticky="w", padx=8, pady=3)
            var = tk.StringVar(value="---")
            self.health_vars[key] = var
            tk.Label(box, textvariable=var, width=20, anchor="w").grid(row=r, column=1, sticky="ew", padx=8, pady=3)

        self.big_stop_btn = tk.Button(
            box,
            text="EMERGENCY STOP",
            bg="#cc0000",
            fg="white",
            activebackground="#aa0000",
            activeforeground="white",
            font=("Arial", 15, "bold"),
            command=self.send_stop,
            height=2,
        )
        self.big_stop_btn.grid(row=len(labels), column=0, columnspan=2, sticky="ew", padx=8, pady=(10, 8))
        return box

    def _build_pose_status_panel(self, parent):
        box = ttk.LabelFrame(parent, text="Pose + go-to-point status")
        box.columnconfigure(1, weight=1)
        self.pose_vars = {}
        rows = [
            ("pose", "Pose"),
            ("pose_age", "Pose age"),
            ("active", "Active"),
            ("reason", "Reason"),
            ("distance", "Distance"),
            ("heading_err", "Heading error"),
            ("goal", "Goal"),
        ]
        for r, (key, label) in enumerate(rows):
            ttk.Label(box, text=label + ":").grid(row=r, column=0, sticky="w", padx=8, pady=3)
            var = tk.StringVar(value="---")
            self.pose_vars[key] = var
            tk.Label(box, textvariable=var, anchor="w").grid(row=r, column=1, sticky="ew", padx=8, pady=3)
        return box

    def _build_cmd_panel(self, parent):
        box = ttk.LabelFrame(parent, text="Command output")
        box.columnconfigure(1, weight=1)
        self.cmd_vars = {}
        rows = [
            ("cmd", "/cmd_vel"),
            ("cmd_age", "cmd age"),
            ("last_telemetry", "Last telemetry"),
            ("telemetry_age", "Telemetry age"),
        ]
        for r, (key, label) in enumerate(rows):
            ttk.Label(box, text=label + ":").grid(row=r, column=0, sticky="w", padx=8, pady=3)
            var = tk.StringVar(value="---")
            self.cmd_vars[key] = var
            tk.Label(box, textvariable=var, anchor="w", wraplength=260, justify="left").grid(row=r, column=1, sticky="ew", padx=8, pady=3)

        ttk.Button(box, text="Ping ESP32", command=lambda: self.send_raw("ping")).grid(row=4, column=0, columnspan=2, sticky="ew", padx=8, pady=(12, 3))
        ttk.Button(box, text="Send STOP once", command=self.send_stop).grid(row=5, column=0, columnspan=2, sticky="ew", padx=8, pady=3)
        return box

    def _build_safe_test_panel(self, parent):
        box = ttk.LabelFrame(parent, text="Safe raw motor tests")
        for c in range(4):
            box.columnconfigure(c, weight=1)

        ttk.Label(box, text="PWM:").grid(row=0, column=0, sticky="e", padx=8, pady=6)
        self.pwm_entry = ttk.Entry(box, width=8)
        self.pwm_entry.insert(0, "150")
        self.pwm_entry.grid(row=0, column=1, sticky="w", padx=4, pady=6)

        ttk.Label(box, text="Duration s:").grid(row=0, column=2, sticky="e", padx=8, pady=6)
        self.duration_entry = ttk.Entry(box, width=8)
        self.duration_entry.insert(0, "0.50")
        self.duration_entry.grid(row=0, column=3, sticky="w", padx=4, pady=6)

        ttk.Label(
            box,
            text="These buttons auto-stop after the duration. Start with wheels lifted.",
            foreground="#555555",
        ).grid(row=1, column=0, columnspan=4, sticky="w", padx=8, pady=(0, 8))

        tests = [
            ("Forward? motors + +", 1, 1),
            ("Backward? motors - -", -1, -1),
            ("Rotate L? motors - +", -1, 1),
            ("Rotate R? motors + -", 1, -1),
        ]
        for i, (label, ls, rs) in enumerate(tests):
            ttk.Button(
                box,
                text=label,
                command=lambda a=ls, b=rs: self.send_timed_motors(a, b),
            ).grid(row=2 + i, column=0, columnspan=4, sticky="ew", padx=8, pady=4)

        ttk.Separator(box, orient="horizontal").grid(row=6, column=0, columnspan=4, sticky="ew", padx=8, pady=10)

        ttk.Label(box, text="Manual raw command:").grid(row=7, column=0, sticky="w", padx=8, pady=4)
        self.raw_entry = ttk.Entry(box)
        self.raw_entry.insert(0, "motors 150 150")
        self.raw_entry.grid(row=7, column=1, columnspan=2, sticky="ew", padx=4, pady=4)
        ttk.Button(box, text="Send", command=self.send_raw_from_entry).grid(row=7, column=3, sticky="ew", padx=8, pady=4)

        ttk.Button(box, text="STOP", command=self.send_stop).grid(row=8, column=0, columnspan=4, sticky="ew", padx=8, pady=(8, 4))
        return box

    def _build_goal_panel(self, parent):
        box = ttk.LabelFrame(parent, text="Goal test")
        for c in range(4):
            box.columnconfigure(c, weight=1)

        ttk.Label(box, text="Goal X mm:").grid(row=0, column=0, sticky="e", padx=8, pady=6)
        self.goal_x_entry = ttk.Entry(box, width=10)
        self.goal_x_entry.grid(row=0, column=1, sticky="w", padx=4, pady=6)

        ttk.Label(box, text="Goal Y mm:").grid(row=0, column=2, sticky="e", padx=8, pady=6)
        self.goal_y_entry = ttk.Entry(box, width=10)
        self.goal_y_entry.grid(row=0, column=3, sticky="w", padx=4, pady=6)

        ttk.Label(box, text="Theta rad:").grid(row=1, column=0, sticky="e", padx=8, pady=6)
        self.goal_theta_entry = ttk.Entry(box, width=10)
        self.goal_theta_entry.insert(0, "0.0")
        self.goal_theta_entry.grid(row=1, column=1, sticky="w", padx=4, pady=6)

        ttk.Label(box, text="Step mm:").grid(row=1, column=2, sticky="e", padx=8, pady=6)
        self.step_entry = ttk.Entry(box, width=10)
        self.step_entry.insert(0, "50")
        self.step_entry.grid(row=1, column=3, sticky="w", padx=4, pady=6)

        ttk.Button(box, text="Use current pose", command=self.use_current_pose_as_goal).grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=4)
        ttk.Button(box, text="Publish goal", command=self.publish_goal_from_entries).grid(row=2, column=2, columnspan=2, sticky="ew", padx=8, pady=4)

        ttk.Separator(box, orient="horizontal").grid(row=3, column=0, columnspan=4, sticky="ew", padx=8, pady=10)

        ttk.Button(box, text="World +Y", command=lambda: self.publish_relative_world_goal(0, 1)).grid(row=4, column=0, sticky="ew", padx=6, pady=4)
        ttk.Button(box, text="World -Y", command=lambda: self.publish_relative_world_goal(0, -1)).grid(row=4, column=1, sticky="ew", padx=6, pady=4)
        ttk.Button(box, text="World +X", command=lambda: self.publish_relative_world_goal(1, 0)).grid(row=4, column=2, sticky="ew", padx=6, pady=4)
        ttk.Button(box, text="World -X", command=lambda: self.publish_relative_world_goal(-1, 0)).grid(row=4, column=3, sticky="ew", padx=6, pady=4)

        ttk.Button(box, text="Robot forward", command=lambda: self.publish_relative_robot_goal(1)).grid(row=5, column=0, columnspan=2, sticky="ew", padx=8, pady=4)
        ttk.Button(box, text="Robot backward", command=lambda: self.publish_relative_robot_goal(-1)).grid(row=5, column=2, columnspan=2, sticky="ew", padx=8, pady=4)

        ttk.Label(
            box,
            text="Only send tiny goals after pose is fresh and raw motor directions are known. Watch distance_mm go down.",
            foreground="#555555",
            wraplength=430,
            justify="left",
        ).grid(row=6, column=0, columnspan=4, sticky="w", padx=8, pady=(12, 4))

        ttk.Button(box, text="STOP", command=self.send_stop).grid(row=7, column=0, columnspan=4, sticky="ew", padx=8, pady=(10, 4))
        return box

    # ---------------- commands ----------------
    def set_action(self, text: str):
        with self.lock:
            self.last_action = text

    def send_raw(self, command: str):
        msg = String()
        msg.data = command.strip()
        self.raw_pub.publish(msg)
        self.set_action(f"Sent /ninja/esp32_cmd: {msg.data}")

    def send_raw_from_entry(self):
        self.send_raw(self.raw_entry.get())

    def send_stop(self):
        self.send_raw("stop")

    def send_timed_motors(self, left_sign: int, right_sign: int):
        pwm = int(abs(parse_float(self.pwm_entry.get(), 150.0)))
        pwm = max(0, min(255, pwm))
        duration = max(0.05, min(3.0, parse_float(self.duration_entry.get(), 0.5)))
        left = left_sign * pwm
        right = right_sign * pwm
        self.send_raw(f"motors {left} {right}")
        self.root.after(int(duration * 1000), self.send_stop)
        self.set_action(f"Timed motor test: motors {left} {right} for {duration:.2f}s, then stop")

    def use_current_pose_as_goal(self):
        with self.lock:
            pose = self.pose
        if pose is None:
            self.set_action("No /ninja/pose yet")
            return
        self.goal_x_entry.delete(0, tk.END)
        self.goal_x_entry.insert(0, f"{pose.x:.1f}")
        self.goal_y_entry.delete(0, tk.END)
        self.goal_y_entry.insert(0, f"{pose.y:.1f}")
        self.goal_theta_entry.delete(0, tk.END)
        self.goal_theta_entry.insert(0, f"{pose.theta:.4f}")
        self.set_action("Copied current pose into goal fields")

    def publish_goal(self, x: float, y: float, theta: float):
        msg = Pose2D()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(theta)
        self.goal_pub.publish(msg)
        self.set_action(f"Published /ninja/goal_pose: x={x:.1f}, y={y:.1f}, theta={theta:.3f}")

    def publish_goal_from_entries(self):
        with self.lock:
            pose = self.pose
        fallback_theta = pose.theta if pose is not None else 0.0
        x = parse_float(self.goal_x_entry.get(), pose.x if pose is not None else 0.0)
        y = parse_float(self.goal_y_entry.get(), pose.y if pose is not None else 0.0)
        theta = parse_float(self.goal_theta_entry.get(), fallback_theta)
        self.publish_goal(x, y, theta)

    def publish_relative_world_goal(self, sx: int, sy: int):
        with self.lock:
            pose = self.pose
        if pose is None:
            self.set_action("No /ninja/pose yet")
            return
        step = max(1.0, min(300.0, parse_float(self.step_entry.get(), 50.0)))
        self.publish_goal(pose.x + sx * step, pose.y + sy * step, pose.theta)

    def publish_relative_robot_goal(self, sign: int):
        with self.lock:
            pose = self.pose
        if pose is None:
            self.set_action("No /ninja/pose yet")
            return
        step = max(1.0, min(300.0, parse_float(self.step_entry.get(), 50.0))) * sign
        self.publish_goal(pose.x + math.cos(pose.theta) * step, pose.y + math.sin(pose.theta) * step, pose.theta)

    # ---------------- refresh ----------------
    def set_health_label(self, key: str, text: str, state: str):
        var = self.health_vars[key]
        var.set(text)
        # Label widgets are in grid; easiest is find by textvariable name.
        # We keep it simple: color is less important than text.

    def refresh_gui(self):
        with self.lock:
            pose = self.pose
            pose_time = self.pose_time
            status = dict(self.status_json)
            status_raw = self.status_raw
            status_time = self.status_time
            cmd = self.cmd_vel
            cmd_time = self.cmd_time
            telemetry_raw = self.telemetry_raw
            telemetry_lines = list(self.telemetry_lines)
            telemetry_time = self.telemetry_time
            connected = self.connected
            connected_time = self.connected_time
            last_action = self.last_action

        # Topic health counts. These can be called from GUI thread.
        try:
            pose_pub = self.count_publishers("/ninja/pose")
            goal_sub = self.count_subscribers("/ninja/goal_pose")
            esp_sub = self.count_subscribers("/ninja/esp32_cmd")
            cmd_pub = self.count_publishers("/cmd_vel")
            cmd_sub = self.count_subscribers("/cmd_vel")
            status_pub = self.count_publishers("/ninja/go_to_point_status")
        except Exception:
            pose_pub = goal_sub = esp_sub = cmd_pub = cmd_sub = status_pub = -1

        self.health_vars["pose_pub"].set(f"{pose_pub}  {'OK' if pose_pub > 0 else 'MISSING'}")
        self.health_vars["goal_sub"].set(f"{goal_sub}  {'OK' if goal_sub > 0 else 'NO go_to_point'}")
        self.health_vars["esp_sub"].set(f"{esp_sub}  {'OK' if esp_sub > 0 else 'NO bridge'}")
        self.health_vars["cmd_pub"].set(f"{cmd_pub}  {'OK' if cmd_pub > 0 else 'NO cmd publisher'}")
        self.health_vars["cmd_sub"].set(f"{cmd_sub}  {'OK' if cmd_sub > 0 else 'NO bridge sub'}")
        self.health_vars["status_pub"].set(f"{status_pub}  {'OK' if status_pub > 0 else 'NO status'}")

        if connected is None:
            conn_text = "unknown"
        else:
            conn_text = f"{connected} age {fmt_age(connected_time)}"
        self.health_vars["connected"].set(conn_text)

        if pose is None:
            self.pose_vars["pose"].set("---")
        else:
            self.pose_vars["pose"].set(f"x={pose.x:.1f}  y={pose.y:.1f}  θ={deg(pose.theta):.1f}°")
        self.pose_vars["pose_age"].set(fmt_age(pose_time))

        if status:
            self.pose_vars["active"].set(str(status.get("active", "---")))
            self.pose_vars["reason"].set(str(status.get("reason", "---")))
            err = status.get("error", {}) if isinstance(status.get("error", {}), dict) else {}
            dist = err.get("distance_mm", status.get("distance_mm", "---"))
            head = err.get("heading_error_deg", status.get("heading_error_deg", "---"))
            self.pose_vars["distance"].set(f"{float(dist):.1f} mm" if isinstance(dist, (int, float)) else str(dist))
            self.pose_vars["heading_err"].set(f"{float(head):.1f}°" if isinstance(head, (int, float)) else str(head))
            goal = status.get("goal", {}) if isinstance(status.get("goal", {}), dict) else {}
            if goal:
                self.pose_vars["goal"].set(f"x={goal.get('x_mm', '---')} y={goal.get('y_mm', '---')}")
            else:
                self.pose_vars["goal"].set("---")
        else:
            self.pose_vars["active"].set("---")
            self.pose_vars["reason"].set(status_raw[:80] if status_raw else "---")
            self.pose_vars["distance"].set("---")
            self.pose_vars["heading_err"].set("---")
            self.pose_vars["goal"].set("---")

        if cmd is None:
            self.cmd_vars["cmd"].set("---")
        else:
            self.cmd_vars["cmd"].set(f"linear.x={cmd.linear.x:.4f} m/s, angular.z={cmd.angular.z:.4f} rad/s")
        self.cmd_vars["cmd_age"].set(fmt_age(cmd_time))
        self.cmd_vars["last_telemetry"].set(telemetry_raw[:120] if telemetry_raw else "---")
        self.cmd_vars["telemetry_age"].set(fmt_age(telemetry_time))

        self.telemetry_text.configure(state="normal")
        self.telemetry_text.delete("1.0", tk.END)
        self.telemetry_text.insert(tk.END, "\n".join(telemetry_lines) if telemetry_lines else "---")
        self.telemetry_text.configure(state="disabled")
        self.telemetry_text.see(tk.END)

        # Give the user one direct warning when bridge is missing.
        if esp_sub == 0:
            self.action_var.set("NO /ninja/esp32_cmd subscriber: esp32_bridge is not running or ROS discovery is broken.")
        elif goal_sub == 0:
            self.action_var.set("NO /ninja/goal_pose subscriber: go_to_point_node.py is not running or ROS discovery is broken.")
        elif pose_pub == 0:
            self.action_var.set("NO /ninja/pose publisher: start ninja_pose_from_overhead.py on the overhead laptop.")
        else:
            self.action_var.set(last_action)

        self.root.after(200, self.refresh_gui)

    def run(self):
        spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        spin_thread.start()
        self.root.mainloop()

    def close(self):
        try:
            self.send_stop()
        except Exception:
            pass
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = NinjaTelemetryGui()
    try:
        node.run()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
