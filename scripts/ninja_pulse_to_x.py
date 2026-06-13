#!/usr/bin/env python3
"""
Ninja pulse-to-X helper for Eurobot 2026 overhead-guided final approach.

Run from the Eurobot2026MTP workspace after sourcing ~/eurobot_net.sh.
This is intended for short, slow, straight granary/fridge approach moves.
It repeatedly sends a small ESP32 forward command, sends stop, waits for a
fresh overhead pose, then decides whether another pulse is needed.

Default /ninja/pose type is geometry_msgs/msg/Pose2D. If your topic type is
something else but still has x, y, theta fields, pass --pose-msg-type.
Example:
  ros2 topic type /ninja/pose
"""

import argparse
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rosidl_runtime_py.utilities import get_message


def wrap_deg(angle: float) -> float:
    """Wrap angle to [-180, 180]."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def heading_close_error_deg(target_deg: float, theta_rad: float) -> float:
    return wrap_deg(target_deg - math.degrees(theta_rad))


class NinjaPulseToX(Node):
    def __init__(self, args: argparse.Namespace, pose_msg_cls):
        super().__init__("ninja_pulse_to_x")
        self.args = args
        self.pose = None
        self.pose_time: Optional[float] = None

        self.create_subscription(pose_msg_cls, args.pose_topic, self.pose_cb, 10)
        self.esp_pub = self.create_publisher(String, args.esp_topic, 10)
        self.enable_pub = self.create_publisher(Bool, args.enable_topic, 10)

    def pose_cb(self, msg):
        self.pose = msg
        self.pose_time = time.time()

    def send_esp(self, text: str):
        msg = String()
        msg.data = text
        self.esp_pub.publish(msg)
        self.get_logger().info(f"ESP <= {text}")

    def set_drive_enabled(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.enable_pub.publish(msg)
        self.get_logger().info(f"/ninja/enable_drive <= {enabled}")

    def wait_for_pose(self, timeout_s: float) -> bool:
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is not None:
                return True
        return False

    def wait_for_fresh_pose_after(self, old_pose_time: Optional[float], timeout_s: float) -> bool:
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose_time is not None and self.pose_time != old_pose_time:
                return True
        return False

    def current_pose_values(self):
        # Works for geometry_msgs/msg/Pose2D and any compatible message with x, y, theta.
        return float(self.pose.x), float(self.pose.y), float(self.pose.theta)

    def stop_all(self):
        self.send_esp("stop")
        time.sleep(0.05)
        self.send_esp("stop")

    def run(self):
        self.get_logger().info(f"Waiting for pose on {self.args.pose_topic} ...")
        if not self.wait_for_pose(self.args.start_timeout_s):
            self.get_logger().error("No pose received. Check topic name, message type, and overhead pose node.")
            return 2

        # Disable normal go_to_point so it does not fight the direct ESP pulses.
        if not self.args.keep_drive_enabled:
            self.set_drive_enabled(False)
            time.sleep(0.1)

        if self.args.watchdog_off:
            self.send_esp("watchdog off")
            time.sleep(0.1)

        self.stop_all()
        time.sleep(self.args.settle_time_s)

        self.get_logger().info(
            "Starting pulse approach: "
            f"target_x={self.args.target_x:.1f}, y_line={self.args.y_line:.1f}, "
            f"heading={self.args.heading_deg:.1f}, pulse={self.args.pulse_mm:.1f} mm"
        )

        pulses = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pose is None:
                continue

            x, y, theta = self.current_pose_values()
            h_deg = math.degrees(theta)
            x_err = self.args.target_x - x
            y_err = self.args.y_line - y
            h_err = heading_close_error_deg(self.args.heading_deg, theta)

            self.get_logger().info(
                f"pose x={x:.1f} y={y:.1f} h={h_deg:.1f} | "
                f"x_err={x_err:.1f} y_err={y_err:.1f} h_err={h_err:.1f}"
            )

            if abs(x_err) <= self.args.tol_x:
                self.stop_all()
                self.get_logger().info("DONE: reached target x tolerance.")
                return 0

            if abs(y_err) > self.args.max_y_error:
                self.stop_all()
                self.get_logger().error("ABORT: y drift too large; stopping to avoid crash.")
                return 3

            if abs(h_err) > self.args.max_heading_error:
                self.stop_all()
                self.get_logger().error("ABORT: heading error too large; stopping to avoid bad turn.")
                return 4

            # This helper assumes the robot faces roughly 180 deg and forward reduces x.
            # If the target has already gone behind the robot, stop instead of turning/chasing.
            if x <= self.args.target_x - self.args.tol_x:
                self.stop_all()
                self.get_logger().info("DONE: target passed/behind robot; stopping instead of chasing.")
                return 0

            if pulses >= self.args.max_pulses:
                self.stop_all()
                self.get_logger().error("ABORT: max pulses reached.")
                return 5

            remaining = abs(x_err)
            pulse_mm = min(self.args.pulse_mm, max(self.args.min_pulse_mm, remaining - self.args.tol_x))

            old_pose_time = self.pose_time
            pulses += 1

            self.send_esp(f"forward {int(round(pulse_mm))}")
            time.sleep(self.args.pulse_time_s)
            self.stop_all()
            time.sleep(self.args.settle_time_s)

            got_fresh = self.wait_for_fresh_pose_after(old_pose_time, self.args.fresh_pose_timeout_s)
            if not got_fresh:
                self.get_logger().warning("No fresh pose after pulse; continuing carefully with latest pose.")

        self.stop_all()
        return 1


def parse_args():
    parser = argparse.ArgumentParser(
        description="Pulse Ninja forward toward a target X using overhead pose feedback."
    )
    parser.add_argument("--target-x", type=float, required=True, help="Target x in mm.")
    parser.add_argument("--y-line", type=float, required=True, help="Expected y line in mm.")
    parser.add_argument("--heading-deg", type=float, default=180.0, help="Expected heading in degrees.")

    parser.add_argument("--pulse-mm", type=float, default=25.0, help="Nominal forward pulse command in mm.")
    parser.add_argument("--min-pulse-mm", type=float, default=10.0, help="Minimum pulse command in mm.")
    parser.add_argument("--pulse-time-s", type=float, default=0.25, help="Time before stop after sending forward command.")
    parser.add_argument("--settle-time-s", type=float, default=0.75, help="Wait time after stop for pose/motion to settle.")

    parser.add_argument("--tol-x", type=float, default=20.0, help="Stop when abs(x error) <= this mm.")
    parser.add_argument("--max-y-error", type=float, default=35.0, help="Abort if abs(y error) exceeds this mm.")
    parser.add_argument("--max-heading-error", type=float, default=12.0, help="Abort if heading error exceeds this deg.")
    parser.add_argument("--max-pulses", type=int, default=20, help="Safety limit on number of pulses.")

    parser.add_argument("--pose-topic", default="/ninja/pose")
    parser.add_argument("--pose-msg-type", default="geometry_msgs/msg/Pose2D")
    parser.add_argument("--esp-topic", default="/ninja/esp32_cmd")
    parser.add_argument("--enable-topic", default="/ninja/enable_drive")

    parser.add_argument("--watchdog-off", action=argparse.BooleanOptionalAction, default=True,
                        help="Send 'watchdog off' before pulsing. Default: true.")
    parser.add_argument("--keep-drive-enabled", action="store_true",
                        help="Do not publish /ninja/enable_drive=false before pulsing.")

    parser.add_argument("--start-timeout-s", type=float, default=3.0)
    parser.add_argument("--fresh-pose-timeout-s", type=float, default=2.0)
    return parser.parse_args()


def main():
    args = parse_args()

    try:
        pose_msg_cls = get_message(args.pose_msg_type)
    except Exception as exc:
        print(f"ERROR: could not load pose message type {args.pose_msg_type}: {exc}")
        print("Check with: ros2 topic type /ninja/pose")
        return 2

    rclpy.init()
    node = NinjaPulseToX(args, pose_msg_cls)
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        node.get_logger().warning("KeyboardInterrupt: stopping robot.")
        node.stop_all()
        code = 130
    finally:
        try:
            node.stop_all()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
