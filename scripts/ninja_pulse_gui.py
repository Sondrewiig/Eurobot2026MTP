#!/usr/bin/env python3
"""
Small companion GUI for overhead pulse driving.
Uses the same go_to_point path as the normal operator GUI: publishes a short Pose goal
and toggles /ninja/enable_drive, then stops after each pulse.

Run from repo after source ~/eurobot_net.sh:
  python3 scripts/ninja_pulse_gui.py
"""
import math
import json
import threading
import time
import tkinter as tk
from tkinter import ttk
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from turtlesim.msg import Pose


def wrap_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


class PulseRos(Node):
    def __init__(self):
        super().__init__('ninja_pulse_gui')
        self.pose: Optional[Pose] = None
        self.pose_time: Optional[float] = None
        self.status_text = ''
        self.running = False
        self.stop_requested = False
        self.goal_topic = '/ninja/goal'
        self.goal_pub = None

        self.create_subscription(Pose, '/ninja/pose', self._pose_cb, 10)
        self.create_subscription(String, '/ninja/go_to_point_status', self._status_cb, 10)
        self.enable_pub = self.create_publisher(Bool, '/ninja/enable_drive', 10)
        self.esp_pub = self.create_publisher(String, '/ninja/esp32_cmd', 10)
        self.cmdvel_stop_count = 0

    def _pose_cb(self, msg: Pose):
        self.pose = msg
        self.pose_time = time.time()

    def _status_cb(self, msg: String):
        self.status_text = msg.data

    def detect_goal_topic(self) -> str:
        """Try to find the Pose goal topic subscribed by /ninja_go_to_point."""
        candidates = []
        try:
            # Common for a node named /ninja_go_to_point
            infos = self.get_subscriber_names_and_types_by_node('ninja_go_to_point', '/')
            for topic, types in infos:
                if topic == '/ninja/pose':
                    continue
                if any(t in ('turtlesim/msg/Pose', 'geometry_msgs/msg/Pose2D') for t in types):
                    candidates.append(topic)
        except Exception:
            pass

        # Fallback common names. The existing GUI normally knows this topic, but this panel
        # can test likely names without changing the robot until a subscriber exists.
        fallback = [
            '/ninja/goal',
            '/ninja/goal_pose',
            '/ninja/target_pose',
            '/ninja/go_to_point_goal',
        ]
        for t in fallback:
            if t not in candidates:
                candidates.append(t)

        # Pick the first topic with subscribers if possible.
        for topic in candidates:
            try:
                pubs = self.count_subscribers(topic)
                if pubs > 0:
                    self.set_goal_topic(topic)
                    return topic
            except Exception:
                pass

        self.set_goal_topic(candidates[0])
        return candidates[0]

    def set_goal_topic(self, topic: str):
        if topic != self.goal_topic or self.goal_pub is None:
            self.goal_topic = topic
            # Goal topic used by current project is expected to be turtlesim/Pose.
            self.goal_pub = self.create_publisher(Pose, topic, 10)
            self.get_logger().info(f'Using goal topic: {topic}')

    def publish_goal(self, x_mm: float, y_mm: float, heading_deg: float):
        if self.goal_pub is None:
            self.set_goal_topic(self.goal_topic)
        msg = Pose()
        msg.x = float(x_mm)
        msg.y = float(y_mm)
        msg.theta = math.radians(float(heading_deg))
        msg.linear_velocity = 0.0
        msg.angular_velocity = 0.0
        self.goal_pub.publish(msg)

    def enable_drive(self, enabled: bool):
        msg = Bool()
        msg.data = bool(enabled)
        self.enable_pub.publish(msg)

    def esp(self, text: str):
        msg = String()
        msg.data = text
        self.esp_pub.publish(msg)

    def hard_stop(self):
        # Disable go_to_point and also send ESP stop, because your setup already uses both.
        for _ in range(3):
            self.enable_drive(False)
            self.esp('stop')
            time.sleep(0.03)

    def wait_for_pose(self, timeout_s: float = 3.0) -> bool:
        start = time.time()
        while time.time() - start < timeout_s and rclpy.ok():
            if self.pose is not None:
                return True
            time.sleep(0.02)
        return False

    def wait_for_fresh_pose(self, old_time: Optional[float], timeout_s: float = 2.0) -> bool:
        start = time.time()
        while time.time() - start < timeout_s and rclpy.ok():
            if self.pose_time is not None and self.pose_time != old_time:
                return True
            time.sleep(0.02)
        return False

    def parse_status_reached(self) -> bool:
        s = self.status_text or ''
        if 'goal_reached' in s or 'no_goal_or_reached' in s:
            return True
        try:
            d = json.loads(s)
            return bool(d.get('reached', False)) or d.get('reason') in ('goal_reached', 'no_goal_or_reached')
        except Exception:
            return False

    def pulse_to_x(self, target_x: float, y_line: float, heading_deg: float, pulse_mm: float,
                   tol_x: float, max_y_error: float, max_heading_error: float,
                   settle_s: float, step_timeout_s: float, log_cb):
        self.running = True
        self.stop_requested = False
        try:
            if not self.wait_for_pose(3.0):
                log_cb('No /ninja/pose. Check overhead pose first.')
                return
            self.detect_goal_topic()
            self.hard_stop()
            time.sleep(0.15)

            log_cb(f'Pulse-to-X start. target_x={target_x:.1f}, y_line={y_line:.1f}, goal_topic={self.goal_topic}')

            while rclpy.ok() and not self.stop_requested:
                p = self.pose
                if p is None:
                    log_cb('No pose yet')
                    time.sleep(0.1)
                    continue

                x = float(p.x)
                y = float(p.y)
                h = math.degrees(float(p.theta))
                x_err = target_x - x
                y_err = y_line - y
                h_err = wrap_deg(heading_deg - h)
                log_cb(f'pose x={x:.1f} y={y:.1f} h={h:.1f} | x_err={x_err:.1f} y_err={y_err:.1f} h_err={h_err:.1f}')

                if abs(x_err) <= tol_x:
                    log_cb('Reached target x tolerance.')
                    return
                if abs(y_err) > max_y_error:
                    log_cb('STOP: y drift too large.')
                    return
                if abs(h_err) > max_heading_error:
                    log_cb('STOP: heading error too large.')
                    return

                # Decide direction along x. For your fridge approach from rear, x normally decreases.
                direction = 1.0 if target_x > x else -1.0
                remaining = abs(target_x - x)
                step = min(pulse_mm, max(5.0, remaining - tol_x))
                next_x = x + direction * step

                old_pose_t = self.pose_time
                self.publish_goal(next_x, y_line, heading_deg)
                time.sleep(0.05)
                self.enable_drive(True)
                log_cb(f'go short: x={next_x:.1f}, y={y_line:.1f}, h={heading_deg:.1f}')

                t0 = time.time()
                while rclpy.ok() and not self.stop_requested and time.time() - t0 < step_timeout_s:
                    # Stop early when status reports reached or pose reached the mini target.
                    if self.parse_status_reached():
                        break
                    p2 = self.pose
                    if p2 is not None:
                        if abs(float(p2.x) - next_x) <= max(tol_x, 15.0):
                            break
                    time.sleep(0.02)

                self.hard_stop()
                time.sleep(settle_s)
                self.wait_for_fresh_pose(old_pose_t, timeout_s=2.0)

        finally:
            self.hard_stop()
            self.running = False
            self.stop_requested = False
            log_cb('Pulse stopped.')

    def one_forward_pulse(self, pulse_mm: float, settle_s: float, heading_override: Optional[float], log_cb):
        self.running = True
        self.stop_requested = False
        try:
            if not self.wait_for_pose(3.0):
                log_cb('No /ninja/pose. Check overhead pose first.')
                return
            self.detect_goal_topic()
            p = self.pose
            if p is None:
                return
            h_deg = math.degrees(float(p.theta)) if heading_override is None else heading_override
            h_rad = math.radians(h_deg)
            # Field convention: theta 180 means forward toward lower x.
            target_x = float(p.x) + math.cos(h_rad) * pulse_mm
            target_y = float(p.y) + math.sin(h_rad) * pulse_mm
            self.hard_stop()
            time.sleep(0.1)
            self.publish_goal(target_x, target_y, h_deg)
            time.sleep(0.05)
            self.enable_drive(True)
            log_cb(f'one pulse goal x={target_x:.1f} y={target_y:.1f} h={h_deg:.1f}')
            time.sleep(0.45)
            self.hard_stop()
            time.sleep(settle_s)
        finally:
            self.hard_stop()
            self.running = False
            self.stop_requested = False
            log_cb('One pulse stopped.')


class PulseGui:
    def __init__(self, root: tk.Tk, node: PulseRos):
        self.root = root
        self.node = node
        root.title('Ninja Pulse GUI')
        root.geometry('720x520')

        frm = ttk.Frame(root, padding=10)
        frm.pack(fill='both', expand=True)

        self.vars = {}
        defaults = {
            'goal_topic': '/ninja/goal',
            'target_x': '2247',
            'y_line': '1938',
            'heading_deg': '180',
            'pulse_mm': '20',
            'tol_x': '15',
            'max_y_error': '30',
            'max_heading_error': '12',
            'settle_s': '0.75',
            'step_timeout_s': '2.5',
        }
        for k, v in defaults.items():
            self.vars[k] = tk.StringVar(value=v)

        rows = [
            ('Goal topic', 'goal_topic'),
            ('Target X mm', 'target_x'),
            ('Y line mm', 'y_line'),
            ('Heading deg', 'heading_deg'),
            ('Pulse mm', 'pulse_mm'),
            ('Tolerance X mm', 'tol_x'),
            ('Max Y error mm', 'max_y_error'),
            ('Max heading error deg', 'max_heading_error'),
            ('Settle seconds', 'settle_s'),
            ('Step timeout seconds', 'step_timeout_s'),
        ]
        for i, (label, key) in enumerate(rows):
            ttk.Label(frm, text=label).grid(row=i, column=0, sticky='w', pady=2)
            ttk.Entry(frm, textvariable=self.vars[key], width=18).grid(row=i, column=1, sticky='w', pady=2)

        ttk.Button(frm, text='Detect goal topic', command=self.detect_goal_topic).grid(row=0, column=2, padx=6, pady=2, sticky='w')
        ttk.Button(frm, text='PULSE TO X', command=self.start_pulse_to_x).grid(row=2, column=2, padx=6, pady=2, sticky='ew')
        ttk.Button(frm, text='ONE FORWARD PULSE', command=self.start_one_pulse).grid(row=3, column=2, padx=6, pady=2, sticky='ew')
        ttk.Button(frm, text='STOP', command=self.stop, style='Danger.TButton').grid(row=4, column=2, padx=6, pady=2, sticky='ew')

        self.pose_label = ttk.Label(frm, text='pose: waiting')
        self.pose_label.grid(row=10, column=0, columnspan=3, sticky='w', pady=(10,2))

        self.status_label = ttk.Label(frm, text='status: waiting')
        self.status_label.grid(row=11, column=0, columnspan=3, sticky='w', pady=2)

        self.log = tk.Text(frm, height=13, width=85)
        self.log.grid(row=12, column=0, columnspan=3, sticky='nsew', pady=(8,0))
        frm.rowconfigure(12, weight=1)
        frm.columnconfigure(2, weight=1)

        self.root.after(100, self.update_labels)

    def log_msg(self, msg: str):
        def _append():
            self.log.insert('end', time.strftime('%H:%M:%S ') + msg + '\n')
            self.log.see('end')
        self.root.after(0, _append)

    def f(self, key: str) -> float:
        return float(self.vars[key].get())

    def detect_goal_topic(self):
        topic = self.node.detect_goal_topic()
        self.vars['goal_topic'].set(topic)
        self.log_msg(f'Detected/using goal topic: {topic}')

    def start_pulse_to_x(self):
        if self.node.running:
            self.log_msg('Already running. Press STOP first.')
            return
        self.node.set_goal_topic(self.vars['goal_topic'].get().strip())
        th = threading.Thread(
            target=self.node.pulse_to_x,
            args=(
                self.f('target_x'), self.f('y_line'), self.f('heading_deg'), self.f('pulse_mm'),
                self.f('tol_x'), self.f('max_y_error'), self.f('max_heading_error'),
                self.f('settle_s'), self.f('step_timeout_s'), self.log_msg,
            ),
            daemon=True,
        )
        th.start()

    def start_one_pulse(self):
        if self.node.running:
            self.log_msg('Already running. Press STOP first.')
            return
        self.node.set_goal_topic(self.vars['goal_topic'].get().strip())
        heading = self.f('heading_deg')
        th = threading.Thread(
            target=self.node.one_forward_pulse,
            args=(self.f('pulse_mm'), self.f('settle_s'), heading, self.log_msg),
            daemon=True,
        )
        th.start()

    def stop(self):
        self.node.stop_requested = True
        self.node.hard_stop()
        self.log_msg('Manual STOP sent.')

    def update_labels(self):
        p = self.node.pose
        if p is not None:
            age = time.time() - self.node.pose_time if self.node.pose_time else 999.0
            self.pose_label.config(text=f'pose: x={p.x:.1f} y={p.y:.1f} theta={math.degrees(p.theta):.1f} deg age={age:.2f}s')
        s = self.node.status_text
        if len(s) > 140:
            s = s[:140] + '...'
        self.status_label.config(text=f'status: {s}')
        self.root.after(100, self.update_labels)


def ros_spin_thread(node: PulseRos):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)


def main():
    rclpy.init()
    node = PulseRos()
    spin = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin.start()
    root = tk.Tk()
    app = PulseGui(root, node)
    try:
        root.mainloop()
    finally:
        node.hard_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
