#!/usr/bin/env python3
"""
drive_calibrator_node.py

Measures what your robot ACTUALLY does for a given /cmd_vel command, using
the overhead camera as ground truth. Use this to fill in:
  esp32_bridge.linear_full_scale_mps   <- from a "linear" run
  esp32_bridge.angular_full_scale_radps <- from an "angular" run

USAGE
Run alongside esp32_bridge and the overhead pose chain. Send commands on
/ninja/calib/cmd:

  linear  <speed_mps> <duration_s>     drive forward at this m/s for N s
  angular <speed_radps> <duration_s>   spin in place
  stop                                 abort current run
  report                               re-publish last result

Result published on /ninja/calib/result (JSON) with measured speed, ratio
(measured/commanded), and a recommendation.

The operator GUI's Tune tab has a panel for this, so once you have it
running you can use the GUI buttons instead of typing commands.

Topics:
  in:   /ninja/calib/cmd      std_msgs/String
        /ninja/pose           geometry_msgs/Pose2D  (overhead ground truth)
  out:  /cmd_vel              geometry_msgs/Twist
        /ninja/calib/result   std_msgs/String  JSON
        /ninja/calib/status   std_msgs/String  short text
"""

import json
import math
import time
from typing import Optional, List, Dict, Any

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import String


def norm_angle_rad(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class DriveCalibrator(Node):
    def __init__(self) -> None:
        super().__init__("drive_calibrator")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter("max_command_duration_s", 8.0)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.result_pub = self.create_publisher(String, "/ninja/calib/result", 10)
        self.status_pub = self.create_publisher(String, "/ninja/calib/status", 10)

        self.create_subscription(String, "/ninja/calib/cmd", self._on_calib_cmd, 10)
        self.create_subscription(Pose2D, "/ninja/pose", self._on_pose, 10)

        self.pose: Optional[Pose2D] = None
        self.pose_t: float = 0.0

        self.run_active: bool = False
        self.run_kind: str = ""
        self.run_cmd_value: float = 0.0
        self.run_duration_s: float = 0.0
        self.run_start_t: float = 0.0
        self.run_start_pose: Optional[Pose2D] = None
        self.samples: List[Dict[str, Any]] = []

        self.last_result: Dict[str, Any] = {}

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            "drive_calibrator ready. /ninja/calib/cmd accepts:\n"
            "  'linear 0.15 2.0'   forward 0.15 m/s for 2 s\n"
            "  'angular 0.5 4.0'   spin in place 0.5 rad/s for 4 s\n"
            "  'stop' / 'report'"
        )

    def _on_pose(self, msg: Pose2D) -> None:
        self.pose = msg
        self.pose_t = time.time()

    def _on_calib_cmd(self, msg: String) -> None:
        text = msg.data.strip().lower()
        if not text:
            return
        parts = text.split()
        if parts[0] == "stop":
            self._abort("user_stop")
            return
        if parts[0] == "report":
            self._publish_result(self.last_result, mark="report")
            return
        if parts[0] in ("linear", "angular"):
            try:
                value = float(parts[1])
                duration = float(parts[2])
            except (IndexError, ValueError):
                self._status(
                    f"bad command '{text}', need: <kind> <value> <seconds>"
                )
                return
            self._start_run(parts[0], value, duration)
            return
        self._status(f"unknown command '{text}'")

    def _start_run(self, kind: str, value: float, duration: float) -> None:
        if self.run_active:
            self._abort("preempted_by_new_command")

        max_dur = float(self.get_parameter("max_command_duration_s").value)
        duration = max(0.1, min(max_dur, duration))

        if self.pose is None:
            self._status("no pose yet, refusing to start run")
            return

        self.run_active = True
        self.run_kind = kind
        self.run_cmd_value = value
        self.run_duration_s = duration
        self.run_start_t = time.time()
        self.run_start_pose = self.pose
        self.samples = []
        self._status(f"START {kind} value={value:.3f} duration={duration:.2f}s")

    def _abort(self, reason: str) -> None:
        if self.run_active:
            self.cmd_pub.publish(Twist())
        self.run_active = False
        self._status(f"ABORTED: {reason}")

    def _finish(self) -> None:
        if not self.run_active:
            return
        self.cmd_pub.publish(Twist())
        self.run_active = False

        if self.run_start_pose is None or self.pose is None:
            self._status("FINISHED but no pose, no result")
            return

        end_pose = self.pose
        dx_mm = end_pose.x - self.run_start_pose.x
        dy_mm = end_pose.y - self.run_start_pose.y
        dist_mm = math.hypot(dx_mm, dy_mm)
        dtheta = norm_angle_rad(end_pose.theta - self.run_start_pose.theta)
        dtheta_deg = math.degrees(dtheta)
        actual_duration = time.time() - self.run_start_t

        result: Dict[str, Any] = {
            "kind": self.run_kind,
            "commanded_value": self.run_cmd_value,
            "commanded_duration_s": round(self.run_duration_s, 3),
            "measured_duration_s": round(actual_duration, 3),
            "start_pose": {
                "x_mm": round(self.run_start_pose.x, 1),
                "y_mm": round(self.run_start_pose.y, 1),
                "heading_deg": round(
                    math.degrees(self.run_start_pose.theta), 1
                ),
            },
            "end_pose": {
                "x_mm": round(end_pose.x, 1),
                "y_mm": round(end_pose.y, 1),
                "heading_deg": round(math.degrees(end_pose.theta), 1),
            },
            "delta": {
                "dx_mm": round(dx_mm, 1),
                "dy_mm": round(dy_mm, 1),
                "distance_mm": round(dist_mm, 1),
                "heading_change_deg": round(dtheta_deg, 1),
            },
        }

        if self.run_kind == "linear":
            mean_speed = (
                dist_mm / 1000.0 / actual_duration if actual_duration > 0 else 0.0
            )
            ratio = (
                mean_speed / self.run_cmd_value if self.run_cmd_value != 0 else 0.0
            )
            result["mean_speed_mps"] = round(mean_speed, 4)
            result["ratio_measured_over_commanded"] = round(ratio, 3)
            result["recommendation"] = (
                "If ratio < 1, robot is slower than commanded. To make "
                "commanded speed match measured, multiply esp32_bridge "
                "linear_full_scale_mps by this ratio. E.g. ratio 0.7 + "
                "current full_scale 0.20 -> set 0.14."
            )
        elif self.run_kind == "angular":
            mean_rate = (
                dtheta / actual_duration if actual_duration > 0 else 0.0
            )
            ratio = (
                mean_rate / self.run_cmd_value if self.run_cmd_value != 0 else 0.0
            )
            result["mean_rate_radps"] = round(mean_rate, 4)
            result["mean_rate_deg_per_s"] = round(math.degrees(mean_rate), 2)
            result["ratio_measured_over_commanded"] = round(ratio, 3)
            result["recommendation"] = (
                "Angular ratio meaningful only after linear is calibrated. "
                "If ratio < 1, multiply angular_full_scale_radps by this "
                "ratio."
            )

        self.last_result = result
        self._publish_result(result, mark="finished")

    def _status(self, text: str) -> None:
        self.get_logger().info(text)
        m = String()
        m.data = text
        self.status_pub.publish(m)

    def _publish_result(self, result: Dict[str, Any], mark: str) -> None:
        if not result:
            self._status(f"no result yet ({mark})")
            return
        m = String()
        m.data = json.dumps(result, separators=(",", ":"))
        self.result_pub.publish(m)
        kind = result.get("kind", "?")
        delta = result.get("delta", {})
        if kind == "linear":
            self._status(
                f"RESULT linear cmd={result.get('commanded_value')} "
                f"dist={delta.get('distance_mm')}mm "
                f"speed={result.get('mean_speed_mps')}m/s "
                f"ratio={result.get('ratio_measured_over_commanded')}"
            )
        elif kind == "angular":
            self._status(
                f"RESULT angular cmd={result.get('commanded_value')} "
                f"hd_change={delta.get('heading_change_deg')}deg "
                f"rate={result.get('mean_rate_deg_per_s')}deg/s "
                f"ratio={result.get('ratio_measured_over_commanded')}"
            )

    def _tick(self) -> None:
        if self.run_active:
            now = time.time()
            pose_age = (
                now - self.pose_t if self.pose_t > 0 else float("inf")
            )
            if pose_age > float(self.get_parameter("pose_timeout_s").value):
                self._abort(f"pose_timeout (age={pose_age:.2f}s)")
                return

            if now - self.run_start_t >= self.run_duration_s:
                self._finish()
                return

            t = Twist()
            if self.run_kind == "linear":
                t.linear.x = float(self.run_cmd_value)
            elif self.run_kind == "angular":
                t.angular.z = float(self.run_cmd_value)
            self.cmd_pub.publish(t)


def main() -> None:
    rclpy.init()
    node = DriveCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._abort("keyboard_interrupt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
