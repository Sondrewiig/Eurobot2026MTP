#!/usr/bin/env python3
"""
esp32_bridge.py

Serial bridge between ROS 2 and the ESP32 motor controller. Runs on the Ninja Pi.

Subscribes:
  /cmd_vel           geometry_msgs/Twist  — velocity commands from go_to_point or crate_align
  /ninja/esp32_cmd   std_msgs/String      — raw ESP32 commands: ping, stop, motors L R

Publishes:
  /ninja/telemetry   std_msgs/String  — raw serial output from ESP32
  /ninja/connected   std_msgs/Bool    — serial connection status

Converts Twist commands to differential PWM motor commands. The drivetrain has
a high practical minimum PWM, so a deadband is applied to make small commands
usable. A reverse guard prevents unintended backward wheel spin during forward
drive while still allowing true in-place turns when linear.x is near zero.
"""

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import String, Bool


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def norm_angle_rad(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def apply_min_abs(value: float, min_abs: float) -> float:
    """Apply simple deadband compensation so tiny commands can move the drivetrain."""
    if abs(value) < 1e-9 or min_abs <= 0.0:
        return value
    if abs(value) < min_abs:
        return math.copysign(min_abs, value)
    return value


class NinjaGoToPointTest(Node):
    def __init__(self) -> None:
        super().__init__('ninja_go_to_point_test_node')

        self.declare_parameter('pose_topic', '/ninja/pose')
        self.declare_parameter('goal_topic', '/ninja/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('status_topic', '/ninja/go_to_point_status')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('pose_timeout_s', 0.7)

        self.declare_parameter('xy_tolerance_mm', 30.0)
        self.declare_parameter('heading_tolerance_deg', 8.0)
        self.declare_parameter('use_goal_heading', False)

        self.declare_parameter('max_linear_mps', 0.08)
        self.declare_parameter('max_angular_radps', 0.25)
        self.declare_parameter('min_linear_mps', 0.025)
        self.declare_parameter('min_angular_radps', 0.08)
        self.declare_parameter('k_linear', 1.2)
        self.declare_parameter('k_angular', 1.0)
        self.declare_parameter('turn_in_place_threshold_deg', 90.0)
        self.declare_parameter('slowdown_radius_mm', 150.0)

        # Flip turn_sign to -1.0 if the drivetrain turn direction is inverted.
        self.declare_parameter('turn_sign', 1.0)
        # Flip drive_sign to -1.0 if the drivetrain drive direction is inverted.
        self.declare_parameter('drive_sign', 1.0)

        # Master arming. When False, /cmd_vel stays zero even if a goal arrives.
        # Toggle live with: ros2 topic pub /ninja/enable_drive std_msgs/Bool "{data: true}"
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('enable_topic', '/ninja/enable_drive')

        self.pose: Optional[Pose2D] = None
        self.pose_time = 0.0
        self.goal: Optional[Pose2D] = None
        self.goal_time = 0.0
        self.active = False
        self.reached = False
        self.last_status_pub = 0.0
        self.drive_enabled = bool(self.get_parameter('start_enabled').value)

        pose_topic = self.get_parameter('pose_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        status_topic = self.get_parameter('status_topic').value

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.pose_sub = self.create_subscription(Pose2D, pose_topic, self.on_pose, 10)
        self.goal_sub = self.create_subscription(Pose2D, goal_topic, self.on_goal, 10)

        enable_topic = self.get_parameter('enable_topic').value
        self.enable_sub = self.create_subscription(Bool, enable_topic, self.on_enable, 10)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(1.0, rate), self.on_timer)

        self.get_logger().info(
            f'Go-to-point node: pose={pose_topic}, goal={goal_topic}, cmd_vel={cmd_vel_topic}, '
            f'enable={enable_topic}, start_enabled={self.drive_enabled}. '
            'Start with wheels lifted or ESP disconnected.'
        )

    def on_pose(self, msg: Pose2D) -> None:
        self.pose = msg
        self.pose_time = time.time()

    def on_goal(self, msg: Pose2D) -> None:
        self.goal = msg
        self.goal_time = time.time()
        self.active = True
        self.reached = False
        self.get_logger().info(f'New goal: x={msg.x:.1f} y={msg.y:.1f} theta={msg.theta:.3f}')

    def on_enable(self, msg: Bool) -> None:
        new_state = bool(msg.data)
        if new_state != self.drive_enabled:
            self.get_logger().info(f'Drive enable toggled: {self.drive_enabled} -> {new_state}')
            if not new_state:
                # Disarm: stop immediately. Goal remains so re-arm resumes.
                self.stop()
        self.drive_enabled = new_state

    def stop(self) -> Twist:
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        return cmd

    def publish_status(self, data: dict, force: bool = False) -> None:
        now = time.time()
        if force or now - self.last_status_pub >= 0.5:
            self.last_status_pub = now
            msg = String()
            msg.data = json.dumps(data, separators=(',', ':'))
            self.status_pub.publish(msg)

    def on_timer(self) -> None:
        now = time.time()

        if not self.drive_enabled:
            # One-shot stop is published in on_enable callback.
            # Don't repeat here so other nodes can publish cmd_vel while we're disarmed.
            self.publish_status({'active': False, 'reason': 'drive_disabled'})
            return

        if self.pose is None:
            self.stop()
            self.publish_status({'active': False, 'reason': 'no_pose'})
            return

        pose_age = now - self.pose_time
        if pose_age > float(self.get_parameter('pose_timeout_s').value):
            self.stop()
            self.publish_status({'active': False, 'reason': 'pose_timeout', 'pose_age_s': round(pose_age, 3)})
            return

        if self.goal is None or not self.active:
            self.stop()
            self.publish_status({'active': False, 'reason': 'no_goal_or_reached'})
            return

        dx_mm = self.goal.x - self.pose.x
        dy_mm = self.goal.y - self.pose.y
        dist_mm = math.hypot(dx_mm, dy_mm)
        target_angle = math.atan2(dy_mm, dx_mm)
        heading_error = norm_angle_rad(target_angle - self.pose.theta)

        xy_tol = float(self.get_parameter('xy_tolerance_mm').value)
        heading_tol = math.radians(float(self.get_parameter('heading_tolerance_deg').value))
        use_goal_heading = bool(self.get_parameter('use_goal_heading').value)

        cmd = Twist()
        reason = 'driving_to_goal'

        if dist_mm <= xy_tol:
            if use_goal_heading:
                final_error = norm_angle_rad(self.goal.theta - self.pose.theta)
                if abs(final_error) <= heading_tol:
                    self.active = False
                    self.reached = True
                    reason = 'goal_reached'
                    cmd = self.stop()
                else:
                    reason = 'rotating_to_goal_heading'
                    ang = float(self.get_parameter('k_angular').value) * final_error
                    max_ang = float(self.get_parameter('max_angular_radps').value)
                    min_ang = float(self.get_parameter('min_angular_radps').value)
                    ang = clamp(ang, -max_ang, max_ang)
                    ang = apply_min_abs(ang, min_ang)
                    ang = clamp(ang, -max_ang, max_ang)
                    cmd.angular.z = float(self.get_parameter('turn_sign').value) * ang
                    self.cmd_pub.publish(cmd)
            else:
                self.active = False
                self.reached = True
                reason = 'goal_reached'
                cmd = self.stop()
        else:
            max_lin = float(self.get_parameter('max_linear_mps').value)
            max_ang = float(self.get_parameter('max_angular_radps').value)
            min_lin = float(self.get_parameter('min_linear_mps').value)
            min_ang = float(self.get_parameter('min_angular_radps').value)
            k_lin = float(self.get_parameter('k_linear').value)
            k_ang = float(self.get_parameter('k_angular').value)
            turn_threshold = math.radians(float(self.get_parameter('turn_in_place_threshold_deg').value))
            slowdown_radius = max(1.0, float(self.get_parameter('slowdown_radius_mm').value))

            ang = clamp(k_ang * heading_error, -max_ang, max_ang)
            if abs(heading_error) > math.radians(2.0):
                ang = apply_min_abs(ang, min_ang)
                ang = clamp(ang, -max_ang, max_ang)
            cmd.angular.z = float(self.get_parameter('turn_sign').value) * ang

            if abs(heading_error) > turn_threshold:
                cmd.linear.x = 0.0
                reason = 'turning_to_face_goal'
            else:
                # Proportional linear speed with slowdown near the target.
                dist_m = dist_mm / 1000.0
                lin = min(max_lin, k_lin * dist_m)
                lin *= clamp(dist_mm / slowdown_radius, 0.15, 1.0)
                lin *= max(0.0, math.cos(heading_error))
                lin = apply_min_abs(lin, min_lin)
                lin = clamp(lin, 0.0, max_lin)
                cmd.linear.x = float(self.get_parameter('drive_sign').value) * lin
                reason = 'driving_to_goal'

            self.cmd_pub.publish(cmd)

        self.publish_status({
            'active': self.active,
            'reached': self.reached,
            'reason': reason,
            'pose': {
                'x_mm': round(self.pose.x, 1),
                'y_mm': round(self.pose.y, 1),
                'heading_deg': round(math.degrees(self.pose.theta), 1),
                'age_s': round(pose_age, 3),
            },
            'goal': {
                'x_mm': round(self.goal.x, 1),
                'y_mm': round(self.goal.y, 1),
                'heading_deg': round(math.degrees(self.goal.theta), 1),
            },
            'error': {
                'distance_mm': round(dist_mm, 1),
                'heading_error_deg': round(math.degrees(heading_error), 1),
            },
            'cmd_vel': {
                'linear_x_mps': round(cmd.linear.x, 4),
                'angular_z_radps': round(cmd.angular.z, 4),
            },
        })


def main() -> None:
    rclpy.init()
    node = NinjaGoToPointTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
