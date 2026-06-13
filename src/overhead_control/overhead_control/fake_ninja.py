#!/usr/bin/env python3
"""
fake_ninja.py - dummy Ninja for testing operator_gui without hardware.

What it does:
  - Publishes a fake /ninja/pose that drifts toward whatever goal it receives
    on /ninja/goal_pose. ~5 cm/s linear, ~30 deg/s angular when enabled.
  - Publishes /ninja/go_to_point_status JSON so the GUI's goal/reason fields
    light up. Reason follows the obvious states: idle, no_pose (never here,
    we always have pose), driving_to_goal, goal_reached.
  - Publishes /cmd_vel that loosely matches what the controller would emit.
  - Publishes /ninja/connected = true so the GUI shows "connected".
  - Subscribes to /ninja/enable_drive (gates motion) and /ninja/esp32_cmd
    (just logs it).

Run on the same machine as the GUI:
    ros2 run overhead_control fake_ninja

Optional params:
    start_x_mm, start_y_mm, start_heading_deg
    max_linear_mps, max_angular_radps
"""

import json
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool, String


def wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FakeNinja(Node):
    def __init__(self) -> None:
        super().__init__("fake_ninja")

        self.declare_parameter("start_x_mm", 500.0)
        self.declare_parameter("start_y_mm", 1000.0)
        self.declare_parameter("start_heading_deg", 0.0)
        self.declare_parameter("max_linear_mps", 0.05)
        self.declare_parameter("max_angular_radps", math.radians(30.0))
        self.declare_parameter("xy_tolerance_mm", 40.0)
        self.declare_parameter("heading_tolerance_deg", 8.0)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.x = float(self.get_parameter("start_x_mm").value)
        self.y = float(self.get_parameter("start_y_mm").value)
        self.theta = math.radians(
            float(self.get_parameter("start_heading_deg").value)
        )

        self.max_lin = float(self.get_parameter("max_linear_mps").value)
        self.max_ang = float(self.get_parameter("max_angular_radps").value)
        self.xy_tol = float(self.get_parameter("xy_tolerance_mm").value)
        self.heading_tol = math.radians(
            float(self.get_parameter("heading_tolerance_deg").value)
        )
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.dt = 1.0 / rate

        self.enabled = False
        self.goal = None       # Pose2D or None
        self.reason = "idle"

        # Publishers.
        self.pose_pub = self.create_publisher(Pose2D, "/ninja/pose", 10)
        self.status_pub = self.create_publisher(
            String, "/ninja/go_to_point_status", 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.connected_pub = self.create_publisher(
            Bool, "/ninja/connected", 10
        )

        # Subscribers.
        self.create_subscription(
            Pose2D, "/ninja/goal_pose", self._on_goal, 10
        )
        self.create_subscription(
            Bool, "/ninja/enable_drive", self._on_enable, 10
        )
        self.create_subscription(
            String, "/ninja/esp32_cmd", self._on_esp_cmd, 10
        )

        # Heartbeat: pose + status every dt; connected once a second.
        self.create_timer(self.dt, self._step)
        self.create_timer(1.0, self._publish_connected)

        self.get_logger().info(
            f"fake_ninja up at x={self.x:.0f} y={self.y:.0f} "
            f"h={math.degrees(self.theta):.1f} deg"
        )

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _on_goal(self, msg: Pose2D) -> None:
        self.goal = msg
        self.reason = "driving_to_goal" if self.enabled else "disabled"
        self.get_logger().info(
            f"goal received: x={msg.x:.0f} y={msg.y:.0f} "
            f"h={math.degrees(msg.theta):.1f}"
        )

    def _on_enable(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.reason = "disabled"
        elif self.goal is not None:
            self.reason = "driving_to_goal"
        else:
            self.reason = "idle"
        self.get_logger().info(f"enable_drive = {self.enabled}")

    def _on_esp_cmd(self, msg: String) -> None:
        self.get_logger().info(f"esp32_cmd: {msg.data!r}")
        if msg.data.strip().lower() == "stop":
            self.enabled = False
            self.reason = "stopped"

    # ------------------------------------------------------------------
    # main step: integrate fake motion, publish everything
    # ------------------------------------------------------------------
    def _step(self) -> None:
        linear = 0.0
        angular = 0.0

        if self.enabled and self.goal is not None:
            dx = self.goal.x - self.x
            dy = self.goal.y - self.y
            dist = math.hypot(dx, dy)

            if dist > self.xy_tol:
                bearing = math.atan2(dy, dx)
                heading_err = wrap(bearing - self.theta)

                # If heading is way off, turn in place first.
                if abs(heading_err) > math.radians(18.0):
                    angular = math.copysign(self.max_ang, heading_err)
                    linear = 0.0
                else:
                    angular = max(-self.max_ang,
                                  min(self.max_ang, 2.0 * heading_err))
                    linear = self.max_lin
                self.reason = "driving_to_goal"
            else:
                # In position. Align heading if it's far off.
                final_err = wrap(self.goal.theta - self.theta)
                if abs(final_err) > self.heading_tol:
                    angular = math.copysign(
                        min(self.max_ang, 2.0 * abs(final_err)), final_err
                    )
                    linear = 0.0
                    self.reason = "final_align"
                else:
                    linear = 0.0
                    angular = 0.0
                    self.reason = "goal_reached"

        # Integrate. linear is m/s; pose is in mm.
        self.theta = wrap(self.theta + angular * self.dt)
        self.x += linear * 1000.0 * self.dt * math.cos(self.theta)
        self.y += linear * 1000.0 * self.dt * math.sin(self.theta)

        # /ninja/pose
        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta
        self.pose_pub.publish(pose_msg)

        # /cmd_vel
        cv = Twist()
        cv.linear.x = linear
        cv.angular.z = angular
        self.cmd_vel_pub.publish(cv)

        # /ninja/go_to_point_status (matches what go_to_point_node would emit)
        status = {
            "reason": self.reason,
            "enabled": self.enabled,
            "have_pose": True,
            "have_goal": self.goal is not None,
        }
        if self.goal is not None:
            status["goal"] = {
                "x_mm": float(self.goal.x),
                "y_mm": float(self.goal.y),
                "heading_deg": math.degrees(self.goal.theta),
            }
            status["error_mm"] = float(
                math.hypot(self.goal.x - self.x, self.goal.y - self.y)
            )
        s = String()
        s.data = json.dumps(status)
        self.status_pub.publish(s)

    def _publish_connected(self) -> None:
        b = Bool()
        b.data = True
        self.connected_pub.publish(b)


def main() -> None:
    rclpy.init()
    node = FakeNinja()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
