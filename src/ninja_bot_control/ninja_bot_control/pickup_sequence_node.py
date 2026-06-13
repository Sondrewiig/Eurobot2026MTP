#!/usr/bin/env python3
"""
pickup_sequence_node.py

Small state-machine for the Ninja crate pickup sequence.
It does not do vision or path planning. It only coordinates gripper/tilt
commands with the existing crate_align_node.

Manual triggers:
  ros2 topic pub --once /ninja/pickup/start std_msgs/msg/Bool "{data: true}"
      -> tilt down, then enable onboard alignment

  ros2 topic pub --once /ninja/pickup/drop std_msgs/msg/Bool "{data: true}"
      -> tilt halfway down and release crates at nest

  ros2 topic pub --once /ninja/pickup/cancel std_msgs/msg/Bool "{data: true}"
      -> disable align and stop motors

Automatic pickup:
  While aligning, when /ninja/vision/align_status reports PAIR_PICKUP_READY
  for ready_required_count consecutive samples:
      disable align, stop, twocrates, tiltup
"""

import json
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class PickupSequenceNode(Node):
    def __init__(self) -> None:
        super().__init__("ninja_pickup_sequence")

        self.declare_parameter("align_status_topic", "/ninja/vision/align_status")
        self.declare_parameter("align_enable_topic", "/ninja/align/enable")
        self.declare_parameter("esp32_cmd_topic", "/ninja/esp32_cmd")
        self.declare_parameter("pickup_start_topic", "/ninja/pickup/start")
        self.declare_parameter("pickup_drop_topic", "/ninja/pickup/drop")
        self.declare_parameter("pickup_cancel_topic", "/ninja/pickup/cancel")

        # Start/travel behavior. The gripper is already open/released mechanically,
        # so only tilt up is sent on startup.
        self.declare_parameter("tilt_up_on_start", True)
        self.declare_parameter("startup_delay_s", 1.0)

        # Commands understood by the ESP32 firmware.
        self.declare_parameter("tilt_up_cmd", "tiltup")
        self.declare_parameter("tilt_down_cmd", "tiltdown")
        self.declare_parameter("half_tilt_cmd", "tilt 40")
        self.declare_parameter("grip_two_cmd", "twocrates")
        self.declare_parameter("release_cmd", "release")
        self.declare_parameter("stop_cmd", "stop")

        # Timing. Keep conservative because servo movement is slow and blocking on ESP32.
        self.declare_parameter("tilt_down_wait_s", 1.2)
        self.declare_parameter("grip_wait_s", 1.0)
        self.declare_parameter("tilt_up_wait_s", 1.0)
        self.declare_parameter("half_tilt_wait_s", 0.8)
        self.declare_parameter("release_wait_s", 0.5)

        # Ready filtering. Avoid triggering on a single noisy READY message.
        self.declare_parameter("ready_required_count", 2)
        self.declare_parameter("ready_action", "PAIR_PICKUP_READY")
        # Keep publishing align enable while the sequence is in the aligning state.
        # A single Bool message can occasionally be missed when ROS discovery/QoS is busy,
        # and manual tests showed that repeated true messages make the gate reliable.
        self.declare_parameter("align_enable_keepalive_hz", 5.0)
        self.declare_parameter("release_before_align", False)
        self.declare_parameter("tilt_up_after_drop", False)

        self.align_pub = self.create_publisher(
            Bool, self.get_parameter("align_enable_topic").value, 10
        )
        self.cmd_pub = self.create_publisher(
            String, self.get_parameter("esp32_cmd_topic").value, 10
        )

        self.create_subscription(
            String,
            self.get_parameter("align_status_topic").value,
            self._on_align_status,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("pickup_start_topic").value,
            self._on_start,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("pickup_drop_topic").value,
            self._on_drop,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("pickup_cancel_topic").value,
            self._on_cancel,
            10,
        )

        self._lock = threading.Lock()

        keepalive_hz = max(0.5, float(self.get_parameter("align_enable_keepalive_hz").value))
        self.create_timer(1.0 / keepalive_hz, self._align_keepalive_tick)
        self._state = "idle"  # idle, preparing_align, aligning, picking, dropping
        self._ready_count = 0
        self._worker: Optional[threading.Thread] = None

        if bool(self.get_parameter("tilt_up_on_start").value):
            delay = float(self.get_parameter("startup_delay_s").value)
            self.create_timer(delay, self._startup_tilt_once)
            self._startup_done = False
        else:
            self._startup_done = True

        self.get_logger().info("Pickup sequence ready. Use /ninja/pickup/start and /ninja/pickup/drop.")

    def _startup_tilt_once(self) -> None:
        if getattr(self, "_startup_done", True):
            return
        self._startup_done = True
        self._send_align(False)
        self._send_cmd(str(self.get_parameter("stop_cmd").value))
        self._send_cmd(str(self.get_parameter("tilt_up_cmd").value))
        self.get_logger().info("Startup: tiltup sent for safe/travel pose")

    def _align_keepalive_tick(self) -> None:
        # While pickup is waiting for PAIR_PICKUP_READY, keep the live align gate open.
        # This replaces fragile one-shot behaviour and matches the reliable manual test
        # using `ros2 topic pub -r 5 /ninja/align/enable ... true`.
        with self._lock:
            should_keep_enabled = self._state == "aligning"
        if should_keep_enabled:
            self._send_align(True, log=False)

    def _send_cmd(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"ESP32 cmd: {command}")

    def _send_align(self, enabled: bool, log: bool = True) -> None:
        msg = Bool()
        msg.data = enabled
        self.align_pub.publish(msg)
        if log:
            self.get_logger().info(f"align enable: {enabled}")

    def _start_worker(self, target) -> bool:
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                self.get_logger().warn(f"Busy in state={self._state}; command ignored")
                return False
            self._worker = threading.Thread(target=target, daemon=True)
            self._worker.start()
            return True

    def _on_start(self, msg: Bool) -> None:
        if not msg.data:
            return
        self._start_worker(self._prepare_align_sequence)

    def _on_drop(self, msg: Bool) -> None:
        if not msg.data:
            return
        self._start_worker(self._drop_sequence)

    def _on_cancel(self, msg: Bool) -> None:
        if not msg.data:
            return
        with self._lock:
            self._state = "idle"
            self._ready_count = 0
        self._send_align(False)
        self._send_cmd(str(self.get_parameter("stop_cmd").value))
        self.get_logger().info("Cancelled pickup/alignment")

    def _prepare_align_sequence(self) -> None:
        with self._lock:
            self._state = "preparing_align"
            self._ready_count = 0

        self._send_align(False)
        self._send_cmd(str(self.get_parameter("stop_cmd").value))

        if bool(self.get_parameter("release_before_align").value):
            self._send_cmd(str(self.get_parameter("release_cmd").value))
            time.sleep(float(self.get_parameter("release_wait_s").value))

        # Lower gripper before onboard alignment. This is needed for the pickup pose.
        self._send_cmd(str(self.get_parameter("tilt_down_cmd").value))
        time.sleep(float(self.get_parameter("tilt_down_wait_s").value))

        with self._lock:
            self._state = "aligning"
            self._ready_count = 0
        self._send_align(True)
        self.get_logger().info("Pickup start: tiltdown done, onboard alignment enabled")

    def _on_align_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        action = data.get("action", "")
        ready_action = str(self.get_parameter("ready_action").value)
        required = int(self.get_parameter("ready_required_count").value)

        trigger_pickup = False
        with self._lock:
            if self._state != "aligning":
                return
            if action == ready_action:
                self._ready_count += 1
            else:
                self._ready_count = 0
            if self._ready_count >= required:
                self._state = "picking"
                self._ready_count = 0
                trigger_pickup = True

        if trigger_pickup:
            self.get_logger().info(f"{ready_action} stable for {required} samples; starting pickup")
            self._start_worker(self._pickup_sequence)

    def _pickup_sequence(self) -> None:
        self._send_align(False)
        self._send_cmd(str(self.get_parameter("stop_cmd").value))
        time.sleep(0.1)

        self._send_cmd(str(self.get_parameter("grip_two_cmd").value))
        time.sleep(float(self.get_parameter("grip_wait_s").value))

        self._send_cmd(str(self.get_parameter("tilt_up_cmd").value))
        time.sleep(float(self.get_parameter("tilt_up_wait_s").value))

        with self._lock:
            self._state = "idle"
        self.get_logger().info("Pickup complete: gripped two crates and tilted up")

    def _drop_sequence(self) -> None:
        with self._lock:
            self._state = "dropping"
            self._ready_count = 0

        self._send_align(False)
        self._send_cmd(str(self.get_parameter("stop_cmd").value))
        time.sleep(0.1)

        # At nest: tilt halfway down, then release crates.
        self._send_cmd(str(self.get_parameter("half_tilt_cmd").value))
        time.sleep(float(self.get_parameter("half_tilt_wait_s").value))

        self._send_cmd(str(self.get_parameter("release_cmd").value))
        time.sleep(float(self.get_parameter("release_wait_s").value))

        if bool(self.get_parameter("tilt_up_after_drop").value):
            self._send_cmd(str(self.get_parameter("tilt_up_cmd").value))
            time.sleep(float(self.get_parameter("tilt_up_wait_s").value))

        with self._lock:
            self._state = "idle"
        self.get_logger().info("Drop complete: half tilt and release sent")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickupSequenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
