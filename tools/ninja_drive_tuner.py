#!/usr/bin/env python3
"""
ninja_drive_tuner.py - standalone, no ROS package install needed.

Tk window with sliders for the parameters that affect driving feel.
Drag, click apply, the change goes live to the running esp32_bridge and
ninja_go_to_point nodes via set_parameters_atomically.

Run from anywhere on the network after sourcing your workspace:
    python3 ninja_drive_tuner.py

NOTE: the Tune tab in operator_gui.py duplicates this functionality and
adds the drive_calibrator panel. If you're using the GUI, you don't need
this script. It exists so you can tune driving without bringing up the
full GUI.
"""

import math
import threading
import tkinter as tk
from typing import Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from geometry_msgs.msg import Pose2D
from std_msgs.msg import String


SLIDERS = [
    ("esp32_bridge", "command_rate_hz", "bridge rate (Hz)", 5.0, 30.0, 1.0, 20.0, "double"),
    ("esp32_bridge", "max_pwm", "max_pwm", 150, 255, 5, 240, "int"),
    ("esp32_bridge", "turn_scale", "turn_scale", 0.05, 1.0, 0.05, 0.45, "double"),
    ("esp32_bridge", "drive_guard_norm", "drive_guard_norm", 0.0, 0.5, 0.02, 0.20, "double"),
    ("esp32_bridge", "max_pwm_step_per_tick", "pwm_step/tick", 1, 60, 1, 30, "int"),
    ("esp32_bridge", "linear_full_scale_mps", "lin_full_scale", 0.05, 0.5, 0.01, 0.20, "double"),
    ("esp32_bridge", "angular_full_scale_radps", "ang_full_scale", 0.2, 3.0, 0.1, 1.20, "double"),
    ("ninja_go_to_point", "max_linear_mps", "gtp max_lin", 0.02, 0.35, 0.01, 0.18, "double"),
    ("ninja_go_to_point", "max_angular_radps", "gtp max_ang", 0.1, 1.5, 0.05, 0.6, "double"),
    ("ninja_go_to_point", "k_linear", "gtp k_lin", 0.1, 2.5, 0.1, 0.8, "double"),
    ("ninja_go_to_point", "k_angular", "gtp k_ang", 0.5, 4.0, 0.1, 1.8, "double"),
    ("ninja_go_to_point", "turn_in_place_threshold_deg", "turn_in_place_deg", 5, 60, 1, 18, "double"),
    ("ninja_go_to_point", "xy_tolerance_mm", "xy_tol_mm", 10, 100, 5, 40, "double"),
    ("ninja_go_to_point", "slowdown_radius_mm", "slowdown_r_mm", 100, 600, 20, 250, "double"),
]


def make_param(name: str, value, ptype: str) -> Parameter:
    p = Parameter()
    p.name = name
    pv = ParameterValue()
    if ptype == "int":
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = int(value)
    elif ptype == "double":
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(value)
    elif ptype == "bool":
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = bool(value)
    else:
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = str(value)
    p.value = pv
    return p


class DriveTuner(Node):
    def __init__(self) -> None:
        super().__init__("ninja_drive_tuner")
        self.clients = {}
        self.pose: Optional[Pose2D] = None

        self.create_subscription(Pose2D, "/ninja/pose", self._on_pose, 10)
        self.goal_pub = self.create_publisher(Pose2D, "/ninja/goal_pose", 10)
        self.cmd_pub = self.create_publisher(String, "/ninja/esp32_cmd", 10)

    def _on_pose(self, msg: Pose2D) -> None:
        self.pose = msg

    def _get_client(self, node_name: str):
        if node_name not in self.clients:
            self.clients[node_name] = self.create_client(
                SetParametersAtomically,
                f"/{node_name}/set_parameters_atomically",
            )
        return self.clients[node_name]

    def set_param(self, node_name: str, name: str, value, ptype: str) -> None:
        client = self._get_client(node_name)
        if not client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(
                f"{node_name}: param service not available"
            )
            return
        req = SetParametersAtomically.Request()
        req.parameters = [make_param(name, value, ptype)]
        client.call_async(req)

    def send_tiny_goal(self) -> None:
        if self.pose is None:
            self.get_logger().warn("no pose yet, cannot send tiny goal")
            return
        gx = self.pose.x + 200.0 * math.cos(self.pose.theta)
        gy = self.pose.y + 200.0 * math.sin(self.pose.theta)
        msg = Pose2D()
        msg.x = gx
        msg.y = gy
        msg.theta = self.pose.theta
        self.goal_pub.publish(msg)

    def send_stop(self) -> None:
        m = String()
        m.data = "stop"
        self.cmd_pub.publish(m)


class TunerGUI:
    def __init__(self, node: DriveTuner) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("Ninja Drive Tuner")
        self.root.configure(bg="#1e1e1e")

        tk.Label(
            self.root,
            text="Move sliders. Changes go live to the running nodes.",
            bg="#1e1e1e", fg="#cde", font=("Helvetica", 10, "italic"),
        ).pack(side="top", padx=8, pady=6)

        body = tk.Frame(self.root, bg="#1e1e1e")
        body.pack(side="top", fill="both", expand=True, padx=8, pady=4)

        self.vars = {}
        for i, (nn, pn, label, lo, hi, step, default, ptype) in enumerate(SLIDERS):
            row = tk.Frame(body, bg="#1e1e1e")
            row.grid(row=i, column=0, sticky="ew", pady=2)
            body.grid_columnconfigure(0, weight=1)

            tk.Label(
                row, text=f"{nn}.{pn}", bg="#1e1e1e", fg="#9af",
                width=34, anchor="w", font=("Courier New", 9),
            ).pack(side="left")

            v = tk.DoubleVar(value=float(default))
            self.vars[(nn, pn)] = (v, ptype)

            tk.Scale(
                row, variable=v, from_=lo, to=hi, resolution=step,
                orient="horizontal", bg="#1e1e1e", fg="#fff",
                troughcolor="#444", highlightthickness=0, length=320,
            ).pack(side="left", fill="x", expand=True, padx=4)

            tk.Button(
                row, text="apply", bg="#048", fg="#fff",
                command=lambda nn=nn, pn=pn, var=v, pt=ptype:
                    self.node.set_param(nn, pn, var.get(), pt),
            ).pack(side="left", padx=4)

        ctl = tk.Frame(self.root, bg="#1e1e1e")
        ctl.pack(side="bottom", fill="x", padx=8, pady=8)

        tk.Button(
            ctl, text="Apply ALL", bg="#063", fg="#fff",
            font=("Helvetica", 11, "bold"),
            command=self._apply_all,
        ).pack(side="left", expand=True, fill="x", padx=2)

        tk.Button(
            ctl, text="Tiny Goal +200mm", bg="#048", fg="#fff",
            font=("Helvetica", 11),
            command=self.node.send_tiny_goal,
        ).pack(side="left", expand=True, fill="x", padx=2)

        tk.Button(
            ctl, text="STOP", bg="#a00", fg="#fff",
            font=("Helvetica", 11, "bold"),
            command=self.node.send_stop,
        ).pack(side="left", expand=True, fill="x", padx=2)

        self.root.bind("<space>", lambda e: self.node.send_stop())

    def _apply_all(self) -> None:
        for (nn, pn), (var, pt) in self.vars.items():
            self.node.set_param(nn, pn, var.get(), pt)

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    rclpy.init()
    node = DriveTuner()
    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True
    )
    spin_thread.start()

    gui = TunerGUI(node)
    try:
        gui.run()
    finally:
        try:
            node.send_stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
