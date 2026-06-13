#!/usr/bin/env python3
"""
Step 32 - ESP32 bridge with quiet logging and safer differential mixing.

Runs on the Ninja Pi.

Input:
  /cmd_vel           geometry_msgs/Twist
  /ninja/esp32_cmd   std_msgs/String raw commands: ping, stop, motors L R

Output:
  serial to ESP32
  /ninja/telemetry   std_msgs/String
  /ninja/connected   std_msgs/Bool

Why this version exists:
  - Step 30 could spam terminal logs at command_rate_hz.
  - Deadband PWM makes tiny commands usable, but differential mixing can easily
    become motors +PWM -PWM, causing spin-in-place.
  - This version can prevent reverse wheel commands while driving forward, while
    still allowing true in-place turns when linear.x is near zero.
"""

import json
import threading
import time
from typing import Optional

import serial
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__("esp32_bridge")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        # Motor output limits. For your current ninja, min_pwm is around 150.
        self.declare_parameter("min_pwm", 150)
        self.declare_parameter("max_pwm", 155)

        # cmd_vel full-scale values used to normalize cmd_vel into -1..+1.
        self.declare_parameter("linear_full_scale_mps", 0.10)
        self.declare_parameter("angular_full_scale_radps", 0.80)

        # Reduce turn_scale when the robot wiggles or spins instead of driving.
        self.declare_parameter("turn_scale", 0.10)

        # Serial command output rate. 5 Hz is enough for first floor tests.
        self.declare_parameter("command_rate_hz", 5.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.4)

        # Ignore very small normalized wheel requests.
        self.declare_parameter("wheel_deadband_norm", 0.03)

        # Safety mixer: if driving forward, do not allow one wheel to reverse.
        # This prevents small heading corrections from becoming spin-in-place.
        self.declare_parameter("prevent_reverse_while_driving", True)
        self.declare_parameter("drive_guard_norm", 0.08)

        # Optional acceleration limiting in PWM per command tick. 0 disables.
        self.declare_parameter("max_pwm_step_per_tick", 8)

        # Reduce terminal spam.
        self.declare_parameter("send_only_on_change", True)
        self.declare_parameter("pwm_change_threshold", 2)
        self.declare_parameter("resend_interval_s", 0.8)
        self.declare_parameter("log_tx", True)
        self.declare_parameter("log_rx", False)
        self.declare_parameter("log_unchanged_tx", False)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.min_pwm = int(self.get_parameter("min_pwm").value)
        self.max_pwm = int(self.get_parameter("max_pwm").value)
        self.linear_full_scale_mps = max(0.001, float(self.get_parameter("linear_full_scale_mps").value))
        self.angular_full_scale_radps = max(0.001, float(self.get_parameter("angular_full_scale_radps").value))
        self.turn_scale = float(self.get_parameter("turn_scale").value)
        self.command_rate_hz = max(0.5, float(self.get_parameter("command_rate_hz").value))
        self.cmd_vel_timeout_sec = float(self.get_parameter("cmd_vel_timeout_sec").value)
        self.wheel_deadband_norm = clamp(float(self.get_parameter("wheel_deadband_norm").value), 0.0, 0.95)
        self.prevent_reverse_while_driving = bool(self.get_parameter("prevent_reverse_while_driving").value)
        self.drive_guard_norm = clamp(float(self.get_parameter("drive_guard_norm").value), 0.0, 1.0)
        self.max_pwm_step_per_tick = max(0, int(self.get_parameter("max_pwm_step_per_tick").value))
        self.send_only_on_change = bool(self.get_parameter("send_only_on_change").value)
        self.pwm_change_threshold = max(0, int(self.get_parameter("pwm_change_threshold").value))
        self.resend_interval_s = max(0.05, float(self.get_parameter("resend_interval_s").value))
        self.log_tx = bool(self.get_parameter("log_tx").value)
        self.log_rx = bool(self.get_parameter("log_rx").value)
        self.log_unchanged_tx = bool(self.get_parameter("log_unchanged_tx").value)

        if self.max_pwm < self.min_pwm:
            self.get_logger().warn(
                f"max_pwm {self.max_pwm} is below min_pwm {self.min_pwm}; swapping them"
            )
            self.min_pwm, self.max_pwm = self.max_pwm, self.min_pwm

        self.ser: Optional[serial.Serial] = None
        self.running = True
        self.connected = False

        self.serial_lock = threading.Lock()
        self.cmd_lock = threading.Lock()

        self.target_left_pwm = 0
        self.target_right_pwm = 0
        self.output_left_pwm = 0
        self.output_right_pwm = 0
        self.last_cmd_vel_time = 0.0
        self.drive_active = False
        self.stop_sent_after_idle = False
        self.last_sent_left: Optional[int] = None
        self.last_sent_right: Optional[int] = None
        self.last_sent_time = 0.0
        self.last_sent_command = ""

        self.raw_sub = self.create_subscription(String, "/ninja/esp32_cmd", self.raw_cmd_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.telemetry_pub = self.create_publisher(String, "/ninja/telemetry", 10)
        self.range_pub = self.create_publisher(String, "/ninja/range_array_json", 10)
        self.connected_pub = self.create_publisher(Bool, "/ninja/connected", 10)

        self.connect_serial()

        self.reader_thread = threading.Thread(target=self.read_from_esp32, daemon=True)
        self.reader_thread.start()

        self.drive_timer = self.create_timer(1.0 / self.command_rate_hz, self.drive_loop)
        self.status_timer = self.create_timer(1.0, self.status_loop)
        self.add_on_set_parameters_callback(self.on_parameter_update)

        self.send_to_esp32("ping", force_log=True)

        self.get_logger().info("ESP32 bridge started")
        self.get_logger().info(f"Port: {self.port}")
        self.get_logger().info(f"Baudrate: {self.baudrate}")
        self.get_logger().info(f"PWM mapping: min_pwm={self.min_pwm}, max_pwm={self.max_pwm}")
        self.get_logger().info(
            f"cmd_vel full scale: linear={self.linear_full_scale_mps:.3f} m/s, "
            f"angular={self.angular_full_scale_radps:.3f} rad/s, turn_scale={self.turn_scale:.3f}"
        )
        self.get_logger().info(
            f"safe mixing: prevent_reverse_while_driving={self.prevent_reverse_while_driving}, "
            f"drive_guard_norm={self.drive_guard_norm:.3f}"
        )
        self.get_logger().info(
            f"quiet send: rate={self.command_rate_hz:.1f} Hz, send_only_on_change={self.send_only_on_change}, "
            f"resend_interval_s={self.resend_interval_s:.2f}"
        )


    def on_parameter_update(self, params):
        """Apply ROS parameter changes to cached bridge fields immediately."""
        try:
            recreate_drive_timer = False
            for param in params:
                name = param.name
                value = param.value

                if name == "min_pwm":
                    self.min_pwm = int(value)
                elif name == "max_pwm":
                    self.max_pwm = int(value)
                elif name == "linear_full_scale_mps":
                    self.linear_full_scale_mps = max(0.001, float(value))
                elif name == "angular_full_scale_radps":
                    self.angular_full_scale_radps = max(0.001, float(value))
                elif name == "turn_scale":
                    self.turn_scale = float(value)
                elif name == "command_rate_hz":
                    self.command_rate_hz = max(0.5, float(value))
                    recreate_drive_timer = True
                elif name == "cmd_vel_timeout_sec":
                    self.cmd_vel_timeout_sec = float(value)
                elif name == "wheel_deadband_norm":
                    self.wheel_deadband_norm = clamp(float(value), 0.0, 0.95)
                elif name == "prevent_reverse_while_driving":
                    self.prevent_reverse_while_driving = bool(value)
                elif name == "drive_guard_norm":
                    self.drive_guard_norm = clamp(float(value), 0.0, 1.0)
                elif name == "max_pwm_step_per_tick":
                    self.max_pwm_step_per_tick = max(0, int(value))
                elif name == "send_only_on_change":
                    self.send_only_on_change = bool(value)
                elif name == "pwm_change_threshold":
                    self.pwm_change_threshold = max(0, int(value))
                elif name == "resend_interval_s":
                    self.resend_interval_s = max(0.05, float(value))
                elif name == "log_tx":
                    self.log_tx = bool(value)
                elif name == "log_rx":
                    self.log_rx = bool(value)
                elif name == "log_unchanged_tx":
                    self.log_unchanged_tx = bool(value)

            if self.max_pwm < self.min_pwm:
                self.min_pwm, self.max_pwm = self.max_pwm, self.min_pwm

            if recreate_drive_timer:
                try:
                    self.drive_timer.cancel()
                    self.destroy_timer(self.drive_timer)
                except Exception:
                    pass
                self.drive_timer = self.create_timer(1.0 / self.command_rate_hz, self.drive_loop)

            self.get_logger().info(
                "Updated bridge params: "
                f"min_pwm={self.min_pwm}, max_pwm={self.max_pwm}, "
                f"lin_full_scale={self.linear_full_scale_mps:.3f}, "
                f"ang_full_scale={self.angular_full_scale_radps:.3f}, "
                f"turn_scale={self.turn_scale:.3f}, "
                f"rate={self.command_rate_hz:.1f}Hz, "
                f"send_only_on_change={self.send_only_on_change}, "
                f"resend_interval_s={self.resend_interval_s:.2f}"
            )
            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def connect_serial(self):
        try:
            self.get_logger().info(f"Opening serial port {self.port} at {self.baudrate}")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05, write_timeout=0.2)

            # Many ESP32 boards reset when serial opens.
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self.connected = True
            self.publish_telemetry(f"BRIDGE CONNECTED {self.port}")
            self.get_logger().info("Serial connected")
        except Exception as error:
            self.ser = None
            self.connected = False
            self.publish_telemetry(f"BRIDGE DISCONNECTED {error}")
            self.get_logger().error(f"Could not open serial port: {error}")

    def raw_cmd_callback(self, msg: String):
        command = msg.data.strip()
        if not command:
            return

        # Raw commands are exact: ping, settings, stop, motors 150 150,
        # and gripper/tilt commands such as release, twocrates, tiltup, tiltdown.
        # Keep these responsive even when another node is publishing zero /cmd_vel.
        self.send_to_esp32(command, force_log=True, force_send=True)

        is_actuator_command = (
            command == "stop"
            or command.startswith("motors")
            or command in {"release", "twocrates", "onecrate", "tiltup", "tiltdown"}
            or command.startswith("tilt ")
        )

        if is_actuator_command:
            # Raw commands should override autonomous drive until a new non-zero cmd_vel arrives.
            # This prevents crate_align zero-velocity output from spamming serial "stop"
            # immediately after tilt/grip commands.
            with self.cmd_lock:
                self.drive_active = False
                self.target_left_pwm = 0
                self.target_right_pwm = 0
                self.output_left_pwm = 0
                self.output_right_pwm = 0
                self.stop_sent_after_idle = command == "stop"

    def normalized_to_pwm(self, value: float) -> int:
        value = clamp(float(value), -1.0, 1.0)
        if abs(value) < self.wheel_deadband_norm:
            return 0
        sign = 1 if value > 0.0 else -1
        magnitude = abs(value)
        pwm = self.min_pwm + magnitude * (self.max_pwm - self.min_pwm)
        return int(sign * round(pwm))

    def cmd_vel_callback(self, msg: Twist):
        forward_norm = clamp(float(msg.linear.x) / self.linear_full_scale_mps, -1.0, 1.0)
        turn_norm = clamp((float(msg.angular.z) / self.angular_full_scale_radps) * self.turn_scale, -1.0, 1.0)

        left_norm = forward_norm - turn_norm
        right_norm = forward_norm + turn_norm

        # If the command is mostly a forward/backward drive, avoid reversing one wheel.
        # Large turns with linear.x near zero are still allowed to spin in place.
        if self.prevent_reverse_while_driving and abs(forward_norm) >= self.drive_guard_norm:
            if forward_norm > 0.0:
                left_norm = max(0.0, left_norm)
                right_norm = max(0.0, right_norm)
            else:
                left_norm = min(0.0, left_norm)
                right_norm = min(0.0, right_norm)

        peak = max(1.0, abs(left_norm), abs(right_norm))
        left_norm /= peak
        right_norm /= peak

        left_pwm = self.normalized_to_pwm(left_norm)
        right_pwm = self.normalized_to_pwm(right_norm)

        # A zero Twist means "stop". Do not keep drive_active true, otherwise
        # drive_loop sends repeated serial "stop" messages and can starve
        # gripper/tilt commands while align is idle. Send one stop only.
        if left_pwm == 0 and right_pwm == 0:
            need_stop = False
            with self.cmd_lock:
                self.target_left_pwm = 0
                self.target_right_pwm = 0
                self.output_left_pwm = 0
                self.output_right_pwm = 0
                self.last_cmd_vel_time = time.time()
                self.drive_active = False
                if not self.stop_sent_after_idle:
                    self.stop_sent_after_idle = True
                    need_stop = True
            if need_stop:
                self.send_stop()
            return

        with self.cmd_lock:
            self.target_left_pwm = left_pwm
            self.target_right_pwm = right_pwm
            self.last_cmd_vel_time = time.time()
            self.drive_active = True
            self.stop_sent_after_idle = False

    def ramp_towards(self, current: int, target: int) -> int:
        step = self.max_pwm_step_per_tick
        if step <= 0:
            return target
        if target > current:
            return min(target, current + step)
        if target < current:
            return max(target, current - step)
        return current

    def should_send_pwm(self, left: int, right: int) -> bool:
        now = time.time()
        if not self.send_only_on_change:
            return True
        if self.last_sent_left is None or self.last_sent_right is None:
            return True
        if abs(left - self.last_sent_left) >= self.pwm_change_threshold:
            return True
        if abs(right - self.last_sent_right) >= self.pwm_change_threshold:
            return True
        if now - self.last_sent_time >= self.resend_interval_s:
            return True
        return False

    def drive_loop(self):
        with self.cmd_lock:
            drive_active = self.drive_active
            target_left = self.target_left_pwm
            target_right = self.target_right_pwm
            age = time.time() - self.last_cmd_vel_time
            stop_already_sent = self.stop_sent_after_idle

        if not drive_active:
            return

        if age > self.cmd_vel_timeout_sec:
            if not stop_already_sent:
                self.send_stop()
                with self.cmd_lock:
                    self.target_left_pwm = 0
                    self.target_right_pwm = 0
                    self.output_left_pwm = 0
                    self.output_right_pwm = 0
                    self.drive_active = False
                    self.stop_sent_after_idle = True
            return

        self.output_left_pwm = self.ramp_towards(self.output_left_pwm, target_left)
        self.output_right_pwm = self.ramp_towards(self.output_right_pwm, target_right)

        left = self.output_left_pwm
        right = self.output_right_pwm

        if left == 0 and right == 0:
            # Safety fallback: one stop, then idle. Continuous stop spam is avoided.
            if not stop_already_sent:
                self.send_stop()
            with self.cmd_lock:
                self.target_left_pwm = 0
                self.target_right_pwm = 0
                self.output_left_pwm = 0
                self.output_right_pwm = 0
                self.drive_active = False
                self.stop_sent_after_idle = True
            return

        if self.should_send_pwm(left, right):
            ok = self.send_to_esp32(f"motors {left} {right}")
            if ok:
                self.last_sent_left = left
                self.last_sent_right = right
                self.last_sent_time = time.time()

    def send_stop(self):
        ok = self.send_to_esp32("stop", force_log=True)
        if ok:
            self.last_sent_left = 0
            self.last_sent_right = 0
            self.last_sent_time = time.time()

    def send_to_esp32(self, command: str, force_log: bool = False, force_send: bool = False) -> bool:
        command = command.strip()
        if not command:
            return False

        if not force_send and command == self.last_sent_command and self.send_only_on_change:
            # Do not suppress motor resends here; PWM sends are handled by should_send_pwm.
            if not command.startswith("motors") and command != "stop":
                return True

        with self.serial_lock:
            if self.ser is None or not self.ser.is_open:
                self.connected = False
                self.get_logger().warn("No serial connection")
                return False
            try:
                self.ser.write((command + "\n").encode("utf-8"))
                self.last_sent_command = command
                if self.log_tx and (force_log or self.log_unchanged_tx or command.startswith("motors") or command == "stop"):
                    self.get_logger().info(f"Pi -> ESP32: {command}")
                return True
            except Exception as error:
                self.connected = False
                self.get_logger().error(f"Write failed: {error}")
                self.close_serial()
                return False

    def read_from_esp32(self):
        while self.running and rclpy.ok():
            if self.ser is None or not self.ser.is_open:
                self.connected = False
                time.sleep(0.2)
                continue
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self.publish_telemetry(line)
                    self.try_publish_vlx_range(line)
                    if self.log_rx:
                        self.get_logger().info(f"ESP32 -> Pi: {line}")
            except Exception as error:
                if rclpy.ok():
                    self.connected = False
                    self.get_logger().error(f"Read failed: {error}")
                self.close_serial()
                time.sleep(0.2)

    def status_loop(self):
        msg = Bool()
        msg.data = self.connected
        self.connected_pub.publish(msg)

    def try_publish_vlx_range(self, line: str):
        """Convert ESP32 'TEL VLX d1 d2 d3 d4 d5 d6' into JSON.

        Sensor mapping used by the Ninja Arduino code:
          VLX1..VLX4 = front/gripper sensors
          VLX5       = left side sensor
          VLX6       = right side sensor

        Invalid/out-of-range values from firmware are <= 0 and are preserved in
        the raw array but omitted from min/side values.
        """
        parts = line.strip().split()
        if len(parts) < 8 or parts[0] != "TEL" or parts[1] != "VLX":
            return
        try:
            values = [int(v) for v in parts[2:8]]
        except Exception:
            return

        def valid(v: int):
            return v if v > 0 else None

        front = values[0:4]
        front_valid = [v for v in front if v > 0]
        left = valid(values[4])
        right = valid(values[5])
        data = {
            "stamp_s": time.time(),
            "vlx": values,
            "front": front,
            "front_min_mm": min(front_valid) if front_valid else None,
            "front_all_valid": len(front_valid) == 4,
            "left_mm": left,
            "right_mm": right,
            "crates_in_position": bool(len(front_valid) == 4 and all(v < 80 for v in front_valid)),
            "mapping": {
                "vlx1": "front_1",
                "vlx2": "front_2",
                "vlx3": "front_3",
                "vlx4": "front_4",
                "vlx5": "left_side",
                "vlx6": "right_side",
            },
            "raw": line,
        }
        msg = String()
        msg.data = json.dumps(data)
        self.range_pub.publish(msg)

    def publish_telemetry(self, text: str):
        msg = String()
        msg.data = text
        self.telemetry_pub.publish(msg)

    def close_serial(self):
        with self.serial_lock:
            try:
                if self.ser is not None:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.connected = False

    def destroy_node(self):
        self.running = False
        self.send_to_esp32("stop", force_log=True, force_send=True)
        time.sleep(0.05)
        self.send_to_esp32("stop", force_log=True, force_send=True)
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send_to_esp32("stop", force_log=True, force_send=True)
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
