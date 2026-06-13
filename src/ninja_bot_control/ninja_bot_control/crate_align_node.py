#!/usr/bin/env python3
"""
crate_align_node.py

Vision based crate/Jenga alignment for the Ninja robot.

Important design note:
The drivetrain has a high practical minimum PWM. Small continuous angular
commands either do nothing or suddenly become a real spin when they cross the
bridge deadband. Therefore this node defaults to pulse control:
  - send a short turn/forward pulse
  - stop
  - wait for a fresh camera update
  - decide again
This avoids the left-right oscillation caused by delayed camera feedback.
"""

import json
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


# === CALIBRATED PROFILES =====================================================
SINGLE_PROFILES: Dict[int, Dict[str, Dict[str, float]]] = {
    36: {
        "forward": {"center": -2.0, "size": 141.0, "angle": -98.7},
        "opposite": {"center": -2.0, "size": 141.0, "angle": 97.0},
    },
    47: {
        "forward": {"center": -2.0, "size": 141.0, "angle": -98.7},
        "opposite": {"center": -2.0, "size": 141.0, "angle": 97.0},
    },
    41: {
        "forward": {"center": -2.0, "size": 141.0, "angle": -98.7},
        "opposite": {"center": -2.0, "size": 141.0, "angle": 97.0},
    },
}

PAIR_PROFILES: Dict[str, Dict[str, Any]] = {
    "forward": {
        "expected_left_id": 47,
        "expected_right_id": 36,
        "center": -15.8,
        "separation": 396.5,
        "size": 185.5,
    },
    "opposite": {
        "expected_left_id": 36,
        "expected_right_id": 47,
        "center": -15.8,
        "separation": 396.5,
        "size": 185.5,
    },
}
# ============================================================================


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def angle_diff_deg(raw: float, target: float) -> float:
    return (raw - target + 180.0) % 360.0 - 180.0


class CrateAlignNode(Node):
    def __init__(self) -> None:
        super().__init__("crate_align_node")

        self.declare_parameter("detection_topic", "/ninja/vision/crate")
        self.declare_parameter("mode", "single")
        self.declare_parameter("target_id", 36)
        self.declare_parameter("orientation", "auto")
        self.declare_parameter("enable_motion", False)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        # Separate single and pair tolerances. Old aliases remain valid.
        self.declare_parameter("center_tolerance_px", 50.0)
        self.declare_parameter("size_tolerance_px", 8.0)
        self.declare_parameter("single_center_tolerance_px", 50.0)
        self.declare_parameter("pair_center_tolerance_px", 35.0)
        self.declare_parameter("single_size_tolerance_px", 8.0)
        self.declare_parameter("pair_size_tolerance_px", 8.0)
        self.declare_parameter("angle_tolerance_deg", 18.0)
        self.declare_parameter("require_angle", False)
        self.declare_parameter("separation_tolerance_px", 40.0)

        # Desired command calculation. max_turn must stay above the bridge
        # deadband or it becomes stop; pulse control makes it safe.
        self.declare_parameter("center_gain", 0.0010)
        self.declare_parameter("max_turn", 0.16)
        self.declare_parameter("approach_speed", 0.030)
        self.declare_parameter("turn_sign", 1.0)
        self.declare_parameter("print_hz", 20.0)
        self.declare_parameter("stale_timeout_s", 1.2)

        # Pulse controller. This is the main fix for the oscillation.
        self.declare_parameter("pulse_control", True)
        self.declare_parameter("turn_pulse_s", 0.18)
        self.declare_parameter("drive_pulse_s", 0.25)
        self.declare_parameter("pulse_wait_s", 0.45)
        self.declare_parameter("min_turn_cmd_radps", 0.15)
        self.declare_parameter("min_approach_speed", 0.025)

        # Kept so older command snippets do not fail; not used for holding lost targets.
        self.declare_parameter("command_hold_s", 0.0)

        self.declare_parameter("vlx_confirm_threshold_mm", 0.0)

        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.print_hz = max(0.5, float(self.get_parameter("print_hz").value))
        self._refresh_runtime_params()

        self.enabled = False
        self.latest_payload: Optional[Dict[str, Any]] = None
        self.latest_time = self.get_clock().now()
        self.latest_range: Dict[str, Any] = {}

        self.pulse_cmd: Optional[Tuple[float, float]] = None
        self.pulse_until_s = 0.0
        self.cooldown_until_s = 0.0
        self.last_pulse_kind = "none"

        self.create_subscription(String, self.detection_topic, self.detection_cb, 10)
        self.create_subscription(String, "/ninja/range_array_json", self.range_cb, 10)
        self.create_subscription(Bool, "/ninja/align/enable", self.enable_cb, 10)

        self.status_pub = self.create_publisher(String, "/ninja/vision/align_status", 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / self.print_hz, self.update)

        self.get_logger().info("CRATE_ALIGN_NODE_PULSE_V2")
        self.get_logger().info(
            f"mode={self.mode} target_id={self.target_id} "
            f"orientation={self.orientation} enable_motion={self.enable_motion}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _refresh_runtime_params(self) -> None:
        self.mode = str(self.get_parameter("mode").value).lower()
        self.target_id = int(self.get_parameter("target_id").value)
        self.orientation = str(self.get_parameter("orientation").value).lower()
        self.enable_motion = bool(self.get_parameter("enable_motion").value)

        center_alias = float(self.get_parameter("center_tolerance_px").value)
        size_alias = float(self.get_parameter("size_tolerance_px").value)
        self.single_center_tol = float(self.get_parameter("single_center_tolerance_px").value)
        self.pair_center_tol = float(self.get_parameter("pair_center_tolerance_px").value)
        self.single_size_tol = float(self.get_parameter("single_size_tolerance_px").value)
        self.pair_size_tol = float(self.get_parameter("pair_size_tolerance_px").value)
        if self.single_center_tol == 50.0:
            self.single_center_tol = center_alias
        if self.single_size_tol == 8.0:
            self.single_size_tol = size_alias

        self.angle_tol = float(self.get_parameter("angle_tolerance_deg").value)
        self.require_angle = bool(self.get_parameter("require_angle").value)
        self.sep_tol = float(self.get_parameter("separation_tolerance_px").value)

        self.center_gain = float(self.get_parameter("center_gain").value)
        self.max_turn = abs(float(self.get_parameter("max_turn").value))
        self.approach_speed = max(0.0, float(self.get_parameter("approach_speed").value))
        self.turn_sign = float(self.get_parameter("turn_sign").value)
        self.stale_timeout_s = max(0.1, float(self.get_parameter("stale_timeout_s").value))

        self.pulse_control = bool(self.get_parameter("pulse_control").value)
        self.turn_pulse_s = max(0.03, float(self.get_parameter("turn_pulse_s").value))
        self.drive_pulse_s = max(0.03, float(self.get_parameter("drive_pulse_s").value))
        self.pulse_wait_s = max(0.0, float(self.get_parameter("pulse_wait_s").value))
        self.min_turn_cmd = abs(float(self.get_parameter("min_turn_cmd_radps").value))
        self.min_approach_speed = max(0.0, float(self.get_parameter("min_approach_speed").value))

    def detection_cb(self, msg: String) -> None:
        try:
            self.latest_payload = json.loads(msg.data)
            self.latest_time = self.get_clock().now()
        except Exception as exc:
            self.get_logger().warn(f"Bad detection JSON: {exc}")

    def range_cb(self, msg: String) -> None:
        try:
            self.latest_range = json.loads(msg.data)
        except Exception:
            self.latest_range = {}

    def enable_cb(self, msg: Bool) -> None:
        new = bool(msg.data)
        if new != self.enabled:
            self.get_logger().info(f"align gate -> {new}")
        self.enabled = new
        if not new:
            self.stop_robot()

    def _can_drive(self) -> bool:
        return bool(self.enable_motion) and bool(self.enabled)

    def _publish_direct(self, linear_x: float, angular_z: float) -> None:
        if not self._can_drive():
            return
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def stop_robot(self) -> None:
        self.pulse_cmd = None
        self.pulse_until_s = 0.0
        self.cooldown_until_s = 0.0
        self.last_pulse_kind = "stop"
        if self.enable_motion:
            self.cmd_pub.publish(Twist())

    def _shape_motion_cmd(self, linear_x: float, angular_z: float) -> Tuple[float, float, str, float]:
        linear_x = float(linear_x)
        angular_z = float(angular_z)
        if abs(angular_z) > 1e-6:
            sign = 1.0 if angular_z > 0.0 else -1.0
            mag = min(max(abs(angular_z), min(self.min_turn_cmd, self.max_turn)), self.max_turn)
            return 0.0, sign * mag, "turn", self.turn_pulse_s
        if abs(linear_x) > 1e-6:
            sign = 1.0 if linear_x > 0.0 else -1.0
            mag = max(abs(linear_x), self.min_approach_speed)
            return sign * mag, 0.0, "drive", self.drive_pulse_s
        return 0.0, 0.0, "stop", 0.0

    def publish_cmd(self, linear_x: float, angular_z: float) -> None:
        if not self._can_drive():
            return

        if not self.pulse_control:
            self._publish_direct(linear_x, angular_z)
            return

        now = self._now_s()

        # Continue current pulse until it expires. Do not update direction mid-pulse.
        if self.pulse_cmd is not None and now < self.pulse_until_s:
            self._publish_direct(self.pulse_cmd[0], self.pulse_cmd[1])
            return

        # End the pulse and force a stop before any new decision.
        if self.pulse_cmd is not None and now >= self.pulse_until_s:
            self.pulse_cmd = None
            self._publish_direct(0.0, 0.0)
            return

        # Wait for the camera to catch up before starting the next pulse.
        if now < self.cooldown_until_s:
            self._publish_direct(0.0, 0.0)
            return

        shaped_linear, shaped_angular, kind, duration = self._shape_motion_cmd(linear_x, angular_z)
        if kind == "stop":
            self._publish_direct(0.0, 0.0)
            return

        self.pulse_cmd = (shaped_linear, shaped_angular)
        self.pulse_until_s = now + duration
        self.cooldown_until_s = self.pulse_until_s + self.pulse_wait_s
        self.last_pulse_kind = kind
        self._publish_direct(shaped_linear, shaped_angular)

    def _pulse_status(self) -> Dict[str, Any]:
        now = self._now_s()
        return {
            "pulse_control": self.pulse_control,
            "state": "active" if self.pulse_cmd is not None and now < self.pulse_until_s else (
                "cooldown" if now < self.cooldown_until_s else "idle"
            ),
            "kind": self.last_pulse_kind,
            "remaining_s": max(0.0, self.pulse_until_s - now) if self.pulse_cmd else 0.0,
            "cooldown_remaining_s": max(0.0, self.cooldown_until_s - now),
        }

    def stop_status(self, action: str, reason: str = "", **extra: Any) -> Dict[str, Any]:
        # Lost/no/stale vision must stop immediately. Do not hold turns after losing target.
        self.stop_robot()
        status = {"ok": False, "action": action}
        if reason:
            status["reason"] = reason
        status.update(extra)
        status["pulse"] = self._pulse_status()
        return status

    def _vlx_ok_for_pickup(self) -> bool:
        threshold = float(self.get_parameter("vlx_confirm_threshold_mm").value)
        if threshold <= 0.0:
            return True
        front = self.latest_range.get("front_min_mm")
        try:
            return float(front) <= threshold
        except Exception:
            return False

    def find_marker(self, payload: Dict[str, Any], target_id: int) -> Optional[Dict[str, Any]]:
        for marker in payload.get("markers") or []:
            try:
                marker_id = int(marker.get("id"))
            except (TypeError, ValueError):
                continue
            if marker_id == target_id:
                return marker
        try:
            if int(payload.get("id")) == target_id:
                return payload
        except (TypeError, ValueError):
            return None
        return None

    def choose_single_profile(self, marker: Dict[str, Any]) -> Tuple[str, Dict[str, float], float]:
        profiles = SINGLE_PROFILES.get(self.target_id, SINGLE_PROFILES[36])
        raw_angle = float(marker.get("raw_angle_deg", 0.0))
        if self.orientation in ("forward", "opposite"):
            profile = profiles[self.orientation]
            return self.orientation, profile, angle_diff_deg(raw_angle, profile["angle"])
        candidates = []
        for name, profile in profiles.items():
            err = angle_diff_deg(raw_angle, profile["angle"])
            candidates.append((abs(err), name, profile, err))
        _, name, profile, err = min(candidates, key=lambda x: x[0])
        return name, profile, err

    def compute_single(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        marker = self.find_marker(payload, self.target_id)
        if marker is None:
            return self.stop_status(
                "NO_TARGET",
                reason=f"id {self.target_id} not visible",
                mode="single",
                target_id=self.target_id,
            )

        profile_name, profile, angle_error = self.choose_single_profile(marker)
        center_now = float(marker.get("center_error_px", 0.0))
        size_now = float(marker.get("marker_size_px", marker.get("size_px", 0.0)))
        raw_angle = float(marker.get("raw_angle_deg", 0.0))

        center_error = center_now - float(profile["center"])
        size_error = float(profile["size"]) - size_now

        center_ready = abs(center_error) <= self.single_center_tol
        size_ready = abs(size_error) <= self.single_size_tol
        angle_within_tolerance = abs(angle_error) <= self.angle_tol
        angle_ready = True if not self.require_angle else angle_within_tolerance

        linear_x = 0.0
        angular_z = 0.0
        if not center_ready:
            action = "CENTER"
            angular_z = self.turn_sign * clamp(self.center_gain * center_error, -self.max_turn, self.max_turn)
        elif not angle_ready:
            action = "ANGLE_BAD_STOP"
        elif not size_ready:
            if size_error > 0.0:
                action = "APPROACH"
                linear_x = self.approach_speed
            else:
                action = "TOO_CLOSE_STOP"
        else:
            action = "PICKUP_READY" if self._vlx_ok_for_pickup() else "PICKUP_WAIT_VLX"

        if action in ("CENTER", "APPROACH"):
            self.publish_cmd(linear_x, angular_z)
        else:
            self.stop_robot()

        shaped_linear, shaped_angular, _, _ = self._shape_motion_cmd(linear_x, angular_z)
        return {
            "ok": True,
            "mode": "single",
            "target_id": self.target_id,
            "profile": profile_name,
            "action": action,
            "center_error_corrected_px": center_error,
            "size_error_px": size_error,
            "angle_error_deg": angle_error,
            "center_ready": center_ready,
            "size_ready": size_ready,
            "angle_ready": angle_ready,
            "angle_within_tolerance": angle_within_tolerance,
            "require_angle": self.require_angle,
            "current": {
                "center_error_px": center_now,
                "marker_size_px": size_now,
                "raw_angle_deg": raw_angle,
            },
            "target": profile,
            "tolerances": {
                "center_px": self.single_center_tol,
                "size_px": self.single_size_tol,
                "angle_deg": self.angle_tol,
            },
            "cmd": {"linear_x": linear_x, "angular_z": angular_z},
            "shaped_cmd": {"linear_x": shaped_linear, "angular_z": shaped_angular},
            "pulse": self._pulse_status(),
            "gate": {
                "enable_motion": self.enable_motion,
                "runtime_enabled": self.enabled,
                "would_drive": self._can_drive(),
            },
        }

    def pair_orientation_from_payload(self, payload: Dict[str, Any]) -> Optional[str]:
        pair = payload.get("pair") or {}
        if pair.get("orientation") in ("forward", "opposite"):
            return str(pair["orientation"])
        x36 = None
        x47 = None
        for marker in payload.get("markers") or []:
            try:
                marker_id = int(marker.get("id"))
            except (TypeError, ValueError):
                continue
            if marker_id == 36:
                x36 = float(marker.get("center_x", 0.0))
            elif marker_id == 47:
                x47 = float(marker.get("center_x", 0.0))
        if x36 is None and "id36_x" in pair:
            x36 = float(pair["id36_x"])
        if x47 is None and "id47_x" in pair:
            x47 = float(pair["id47_x"])
        if x36 is None or x47 is None:
            return None
        return "forward" if x47 < x36 else "opposite"

    def compute_pair(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        pair = payload.get("pair")
        if not pair or not bool(pair.get("seen", True)):
            return self.stop_status(
                "NO_PAIR",
                reason="need both id36 and id47 visible",
                mode="pair",
            )

        detected_orientation = self.pair_orientation_from_payload(payload)
        if detected_orientation is None:
            return self.stop_status(
                "NO_PAIR_ORDER",
                reason="could not determine id36/id47 order",
                mode="pair",
            )

        if self.orientation in ("forward", "opposite"):
            profile_name = self.orientation
            if detected_orientation != profile_name:
                self.stop_robot()
                return {
                    "ok": False,
                    "action": "WRONG_PAIR_ORIENTATION",
                    "detected_orientation": detected_orientation,
                    "requested_orientation": profile_name,
                    "pulse": self._pulse_status(),
                }
        else:
            profile_name = detected_orientation

        profile = PAIR_PROFILES[profile_name]
        center_now = float(pair.get("center_error_px", 0.0))
        size_now = float(pair.get("marker_size_px", 0.0))
        sep_now = float(pair.get("separation_px", 0.0))

        center_error = center_now - float(profile["center"])
        size_error = float(profile["size"]) - size_now
        sep_error = sep_now - float(profile["separation"])

        center_ready = abs(center_error) <= self.pair_center_tol
        size_ready = abs(size_error) <= self.pair_size_tol
        sep_ready = abs(sep_error) <= self.sep_tol

        linear_x = 0.0
        angular_z = 0.0
        if not center_ready:
            action = "CENTER_PAIR"
            angular_z = self.turn_sign * clamp(self.center_gain * center_error, -self.max_turn, self.max_turn)
        elif not size_ready:
            if size_error > 0.0:
                action = "APPROACH_PAIR"
                linear_x = self.approach_speed
            else:
                action = "PAIR_TOO_CLOSE_STOP"
        elif not sep_ready:
            action = "PAIR_SEPARATION_BAD_STOP"
        else:
            action = "PAIR_PICKUP_READY" if self._vlx_ok_for_pickup() else "PAIR_PICKUP_WAIT_VLX"

        if action in ("CENTER_PAIR", "APPROACH_PAIR"):
            self.publish_cmd(linear_x, angular_z)
        else:
            self.stop_robot()

        shaped_linear, shaped_angular, _, _ = self._shape_motion_cmd(linear_x, angular_z)
        return {
            "ok": True,
            "mode": "pair",
            "profile": profile_name,
            "detected_orientation": detected_orientation,
            "action": action,
            "center_error_corrected_px": center_error,
            "size_error_px": size_error,
            "separation_error_px": sep_error,
            "center_ready": center_ready,
            "size_ready": size_ready,
            "separation_ready": sep_ready,
            "current": {
                "pair_center_error_px": center_now,
                "pair_size_px": size_now,
                "pair_separation_px": sep_now,
            },
            "target": profile,
            "tolerances": {
                "center_px": self.pair_center_tol,
                "size_px": self.pair_size_tol,
                "separation_px": self.sep_tol,
            },
            "cmd": {"linear_x": linear_x, "angular_z": angular_z},
            "shaped_cmd": {"linear_x": shaped_linear, "angular_z": shaped_angular},
            "pulse": self._pulse_status(),
            "gate": {
                "enable_motion": self.enable_motion,
                "runtime_enabled": self.enabled,
                "would_drive": self._can_drive(),
            },
        }

    def update(self) -> None:
        self._refresh_runtime_params()
        if self.latest_payload is None:
            status = self.stop_status("NO_DETECTION_MESSAGES")
        else:
            age = (self.get_clock().now() - self.latest_time).nanoseconds / 1e9
            if age > self.stale_timeout_s:
                status = self.stop_status(
                    "STALE_DETECTION",
                    age_sec=age,
                    stale_timeout_s=self.stale_timeout_s,
                )
            elif self.mode == "pair":
                status = self.compute_pair(self.latest_payload)
            else:
                status = self.compute_single(self.latest_payload)
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CrateAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
