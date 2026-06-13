#!/usr/bin/env python3
"""
ninja_mission_node.py

Mission orchestrator for Ninja SIMA. Runs on the Ninja Pi.

Subscribes:
  /ninja/mission              std_msgs/String   — fridge1 | fridge2 | full |
                                                   test_nav_fridge1 | test_pickup | abort
  /ninja/go_to_point_status   std_msgs/String   — JSON from go_to_point_node
  /ninja/pose                 geometry_msgs/Pose2D

Publishes:
  /ninja/goal_pose            geometry_msgs/Pose2D
  /ninja/enable_drive         std_msgs/Bool
  /ninja/esp32_cmd            std_msgs/String
  /ninja/mission_status       std_msgs/String   — JSON phase updates
  /cmd_vel                    geometry_msgs/Twist  (NAV_APPROACH only)

Gripper reference (from ninja_moves.py):
  tilt 0   = tilted down (pickup position)
  tilt 65  = tilted up   (travel/safe position)
  tilt 9   = drop position
  grip 58  = closed on 2 crates
  grip 165 = fully open
  release  = release command

Navigation strategy:
  NAV_APPROACH  — Mission node publishes /cmd_vel directly (bypasses go_to_point).
                  Drives at fixed heading=180° along the rear wall.
                  y drift is corrected only via small heading adjustments.
  NAV_TO_FRIDGE — go_to_point turns robot and nudges to fridge pre-position at 270°.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool, String


# ── Mission phases ────────────────────────────────────────────────────────────
IDLE            = "IDLE"
NAV_APPROACH    = "NAV_APPROACH"       # straight drive along rear wall to fridge x
NAV_TO_FRIDGE   = "NAV_TO_FRIDGE"     # go_to_point: turn + nudge to face fridge
PUSH_FORWARD    = "PUSH_FORWARD"       # fridge 2 only: push crates away from wall
PUSH_BACK       = "PUSH_BACK"          # fridge 2 only: reverse after push
PICKUP          = "PICKUP"             # tilt down → grip → tilt up
NAV_TO_NEST     = "NAV_TO_NEST"
DROP            = "DROP"
DONE            = "DONE"
ABORTED         = "ABORTED"
ERROR           = "ERROR"

# Gripper timing.
GRIPPER_SETTLE_S = 0.8
TILT_SETTLE_S    = 0.5

# Straight-line approach controller gains.
K_APPROACH_HEADING   = 0.8     # rad/s per rad heading error
K_APPROACH_Y_MM      = 0.004   # rad/s per mm lateral drift from approach_y
MAX_APPROACH_ANGULAR = 0.20    # rad/s cap during approach
APPROACH_LINEAR_MPS  = 0.03    # fixed forward speed
APPROACH_SLOWDOWN_MM = 150.0   # start slowing down this far before target x


class NinjaMissionNode(Node):

    def __init__(self):
        super().__init__("ninja_mission_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("nest_pose_x",         2300.0)
        self.declare_parameter("nest_pose_y",         1940.0)
        self.declare_parameter("nest_pose_theta_deg",  180.0)

        self.declare_parameter("fridge1_pre_x",          1900.0)
        self.declare_parameter("fridge1_pre_y",          1925.0)
        self.declare_parameter("fridge1_pre_theta_deg",   270.0)

        self.declare_parameter("fridge2_pre_x",          1650.0)
        self.declare_parameter("fridge2_pre_y",          1925.0)
        self.declare_parameter("fridge2_pre_theta_deg",   270.0)

        self.declare_parameter("fridge_approach_y",         1940.0)
        self.declare_parameter("fridge_approach_theta_deg",  180.0)

        self.declare_parameter("fridge2_push_mm",   60.0)
        self.declare_parameter("nav_timeout_s",     30.0)
        self.declare_parameter("pickup_timeout_s",   8.0)

        # ── Publishers ──────────────────────────────────────────────────────
        self.goal_pub    = self.create_publisher(Pose2D,  "/ninja/goal_pose",      10)
        self.enable_pub  = self.create_publisher(Bool,    "/ninja/enable_drive",   10)
        self.esp_pub     = self.create_publisher(String,  "/ninja/esp32_cmd",      10)
        self.status_pub  = self.create_publisher(String,  "/ninja/mission_status", 10)
        self.cmd_vel_pub = self.create_publisher(Twist,   "/cmd_vel",              10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(String, "/ninja/mission",            self._on_mission,      10)
        self.create_subscription(String, "/ninja/go_to_point_status", self._on_nav_status,   10)
        self.create_subscription(Pose2D, "/ninja/pose",               self._on_pose,         10)
        self.create_subscription(Bool,   "/ninja/enable_drive",       self._on_ext_enable,   10)

        # ── State ────────────────────────────────────────────────────────────
        self.phase              = IDLE
        self.mission            = None
        self.phase_start        = 0.0
        self.nav_reached        = False
        self.nav_active         = False
        self.pose               = None
        self._pose_time         = 0.0
        self._pending_fridge    = None
        self._push_origin       = None
        self._current_fridge    = 1
        self._approach_target_x = 0.0
        self._approach_target_y = 1940.0
        self._approach_armed    = False
        self._last_drive_enable = False

        # ── Timers ───────────────────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self._tick)
        self._init_timer = self.create_timer(3.0, self._startup_tilt)

        self.get_logger().info(
            "[NinjaMission] Ready. Send a command on /ninja/mission: "
            "fridge1 | fridge2 | full | test_nav_fridge1 | test_pickup | abort"
        )

    # ── Startup ───────────────────────────────────────────────────────────────

    def _startup_tilt(self):
        self._esp("tilt 65")
        self._init_timer.cancel()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _esp(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.esp_pub.publish(msg)
        self.get_logger().info(f"[NinjaMission] ESP32 → {cmd}")

    def _drive(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.enable_pub.publish(msg)

    def _stop_direct(self):
        self.cmd_vel_pub.publish(Twist())

    def _send_goal(self, x_mm: float, y_mm: float, theta_deg: float, enable_drive: bool = False):
        goal = Pose2D()
        goal.x     = float(x_mm)
        goal.y     = float(y_mm)
        goal.theta = math.radians(float(theta_deg))
        self.goal_pub.publish(goal)
        if enable_drive:
            self._drive(True)
        self.nav_reached = False
        self.nav_active  = True
        self.get_logger().info(
            f"[NinjaMission] Goal → x={x_mm:.0f} y={y_mm:.0f} θ={theta_deg:.0f}°"
        )

    def _set_phase(self, phase: str):
        self.phase       = phase
        self.phase_start = time.time()
        self._publish_status()
        self.get_logger().info(f"[NinjaMission] Phase: {phase}")

    def _phase_elapsed(self) -> float:
        return time.time() - self.phase_start

    def _publish_status(self, extra: dict | None = None):
        data = {
            "phase":   self.phase,
            "mission": self.mission,
            "t":       round(time.time(), 2),
        }
        if extra:
            data.update(extra)
        msg = String()
        msg.data = json.dumps(data)
        self.status_pub.publish(msg)

    def _abort(self, reason: str = ""):
        self.get_logger().warn(f"[NinjaMission] ABORT — {reason}")
        self._stop_direct()
        self._drive(False)
        self._esp("stop")
        self._set_phase(ABORTED)
        self.mission = None

    @staticmethod
    def _wrap_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # ── Subscribers ──────────────────────────────────────────────────────────

    def _on_mission(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"[NinjaMission] Received mission command: {cmd}")

        if cmd == "abort":
            self._abort("operator abort")
            return

        if self.phase not in (IDLE, DONE, ABORTED, ERROR):
            self.get_logger().warn(
                f"[NinjaMission] Ignoring '{cmd}' — mission '{self.mission}' still running (phase={self.phase})"
            )
            return

        self.mission     = cmd
        self.nav_reached = False
        self.nav_active  = False

        if cmd == "test_nav_fridge1":
            self._start_approach_to_fridge(1)
        elif cmd == "test_nav_fridge2":
            self._start_approach_to_fridge(2)
        elif cmd == "test_pickup":
            self._set_phase(PICKUP)
        elif cmd == "fridge1":
            self._start_approach_to_fridge(1)
        elif cmd == "fridge2":
            self._start_approach_to_fridge(2)
        elif cmd == "full":
            self._pending_fridge = 2
            self._start_approach_to_fridge(1)
        else:
            self.get_logger().warn(f"[NinjaMission] Unknown command: {cmd}")

    def _on_nav_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.nav_reached = bool(data.get("reached", False))
            self.nav_active  = bool(data.get("active",  True))
        except Exception:
            pass

    def _on_pose(self, msg: Pose2D):
        self.pose      = msg
        self._pose_time = time.time()

    def _on_ext_enable(self, msg: Bool):
        self._last_drive_enable = bool(msg.data)
        # Only arm the approach from an external True — never disarm it here,
        # because _drive(False) publishes to this same topic and must not
        # cancel the arm that the operator set before the mission was sent.
        if msg.data and self.phase == NAV_APPROACH:
            self._approach_armed = True

    # ── Navigation helpers ────────────────────────────────────────────────────

    def _start_approach_to_fridge(self, fridge_num: int, auto_enable: bool = False):
        """Step 1: direct cmd_vel straight-line drive along rear wall to fridge x.
        go_to_point is disabled here so it cannot fight cmd_vel.
        Operator must type 'enable' (or auto_enable=True) to arm movement.
        """
        if fridge_num == 1:
            x = float(self.get_parameter("fridge1_pre_x").value)
        else:
            x = float(self.get_parameter("fridge2_pre_x").value)
        y = float(self.get_parameter("fridge_approach_y").value)
        self._current_fridge    = fridge_num
        self._approach_target_x = x
        self._approach_target_y = y
        self.nav_reached        = False
        self.nav_active         = False
        if auto_enable or self._last_drive_enable:
            self._approach_armed = True
        self._drive(False)   # disable go_to_point so it won't fight cmd_vel
        self._set_phase(NAV_APPROACH)
        self.get_logger().info(
            f"[NinjaMission] Approach fridge {fridge_num}: "
            f"target x={x:.0f} y={y:.0f} armed={self._approach_armed}"
        )

    def _start_nav_to_fridge(self, fridge_num: int, auto_enable: bool = False):
        """Step 2: go_to_point turns and nudges to fridge pre-position at 270°."""
        if fridge_num == 1:
            x   = float(self.get_parameter("fridge1_pre_x").value)
            y   = float(self.get_parameter("fridge1_pre_y").value)
            deg = float(self.get_parameter("fridge1_pre_theta_deg").value)
        else:
            x   = float(self.get_parameter("fridge2_pre_x").value)
            y   = float(self.get_parameter("fridge2_pre_y").value)
            deg = float(self.get_parameter("fridge2_pre_theta_deg").value)
        self._current_fridge = fridge_num
        self._send_goal(x, y, deg, enable_drive=auto_enable)
        self._set_phase(NAV_TO_FRIDGE)

    def _start_nav_to_nest(self):
        x   = float(self.get_parameter("nest_pose_x").value)
        y   = float(self.get_parameter("nest_pose_y").value)
        deg = float(self.get_parameter("nest_pose_theta_deg").value)
        self._send_goal(x, y, deg, enable_drive=True)
        self._set_phase(NAV_TO_NEST)

    # ── Main tick ────────────────────────────────────────────────────────────

    def _tick(self):
        if self.phase == IDLE:
            return

        # ── NAV_APPROACH: direct cmd_vel straight-line to fridge x ──────────
        if self.phase == NAV_APPROACH:
            timeout = float(self.get_parameter("nav_timeout_s").value)
            if self._phase_elapsed() > timeout:
                self._stop_direct()
                self._abort(f"approach timed out after {timeout:.0f}s")
                return

            if not self._approach_armed:
                return  # waiting for operator to type 'enable'

            if self.pose is None or (time.time() - self._pose_time) > 0.5:
                self._stop_direct()
                return

            dx = self.pose.x - self._approach_target_x
            if dx <= 30.0:
                self._stop_direct()
                self._start_nav_to_fridge(self._current_fridge, auto_enable=True)
                return

            error_heading = self._wrap_angle(math.pi - self.pose.theta)
            error_y       = self.pose.y - self._approach_target_y
            angular       = (K_APPROACH_HEADING * error_heading
                             + K_APPROACH_Y_MM   * error_y)
            angular       = max(-MAX_APPROACH_ANGULAR, min(MAX_APPROACH_ANGULAR, angular))
            slowdown      = min(1.0, max(0.15, dx / APPROACH_SLOWDOWN_MM))
            linear        = APPROACH_LINEAR_MPS * slowdown

            cmd = Twist()
            cmd.linear.x  = linear
            cmd.angular.z = angular
            self.cmd_vel_pub.publish(cmd)

        # ── NAV_TO_FRIDGE ──────────────────────────────────────────────────
        elif self.phase == NAV_TO_FRIDGE:
            timeout = float(self.get_parameter("nav_timeout_s").value)
            if self.nav_reached and self._phase_elapsed() > 0.5:
                self._drive(False)
                if self.mission in ("test_nav_fridge1", "test_nav_fridge2"):
                    self.get_logger().info("[NinjaMission] test_nav done — at fridge position.")
                    self._set_phase(DONE)
                elif self._current_fridge == 2:
                    if self.pose:
                        self._push_origin = (self.pose.x, self.pose.y)
                    push_mm = float(self.get_parameter("fridge2_push_mm").value)
                    goal_y  = float(self.get_parameter("fridge2_pre_y").value) - push_mm
                    goal_x  = float(self.get_parameter("fridge2_pre_x").value)
                    deg     = float(self.get_parameter("fridge2_pre_theta_deg").value)
                    self._send_goal(goal_x, goal_y, deg, enable_drive=True)
                    self._set_phase(PUSH_FORWARD)
                else:
                    self._set_phase(PICKUP)
            elif self._phase_elapsed() > timeout:
                self._abort(f"nav to fridge timed out after {timeout:.0f}s")

        # ── PUSH_FORWARD (fridge 2) ────────────────────────────────────────
        elif self.phase == PUSH_FORWARD:
            if self.nav_reached and self._phase_elapsed() > 0.5:
                self._drive(False)
                x   = float(self.get_parameter("fridge2_pre_x").value)
                y   = float(self.get_parameter("fridge2_pre_y").value)
                deg = float(self.get_parameter("fridge2_pre_theta_deg").value)
                self._send_goal(x, y, deg, enable_drive=True)
                self._set_phase(PUSH_BACK)
            elif self._phase_elapsed() > 10.0:
                self._abort("push forward timed out")

        # ── PUSH_BACK (fridge 2) ───────────────────────────────────────────
        elif self.phase == PUSH_BACK:
            if self.nav_reached and self._phase_elapsed() > 0.5:
                self._drive(False)
                self._set_phase(PICKUP)
            elif self._phase_elapsed() > 10.0:
                self._abort("push back timed out")

        # ── PICKUP ─────────────────────────────────────────────────────────
        elif self.phase == PICKUP:
            elapsed = self._phase_elapsed()
            if elapsed < TILT_SETTLE_S:
                if elapsed < 0.05:
                    self._esp("tilt 0")
            elif elapsed < TILT_SETTLE_S + GRIPPER_SETTLE_S:
                if elapsed < TILT_SETTLE_S + 0.05:
                    self._esp("grip 58")
            elif elapsed < TILT_SETTLE_S + GRIPPER_SETTLE_S + TILT_SETTLE_S:
                if elapsed < TILT_SETTLE_S + GRIPPER_SETTLE_S + 0.05:
                    self._esp("tilt 65")
            else:
                if self.mission == "test_pickup":
                    self.get_logger().info("[NinjaMission] test_pickup done.")
                    self._set_phase(DONE)
                else:
                    self._start_nav_to_nest()

        # ── NAV_TO_NEST ────────────────────────────────────────────────────
        elif self.phase == NAV_TO_NEST:
            timeout = float(self.get_parameter("nav_timeout_s").value)
            if self.nav_reached and self._phase_elapsed() > 0.5:
                self._drive(False)
                self._set_phase(DROP)
            elif self._phase_elapsed() > timeout:
                self._abort(f"nav to nest timed out after {timeout:.0f}s")

        # ── DROP ───────────────────────────────────────────────────────────
        elif self.phase == DROP:
            elapsed = self._phase_elapsed()
            if elapsed < TILT_SETTLE_S:
                if elapsed < 0.05:
                    self._esp("tilt 9")
            elif elapsed < TILT_SETTLE_S + GRIPPER_SETTLE_S:
                if elapsed < TILT_SETTLE_S + 0.05:
                    self._esp("release")
            elif elapsed < TILT_SETTLE_S + GRIPPER_SETTLE_S + TILT_SETTLE_S:
                if elapsed < TILT_SETTLE_S + GRIPPER_SETTLE_S + 0.05:
                    self._esp("tilt 65")
            else:
                if self._pending_fridge is not None:
                    fridge = self._pending_fridge
                    self._pending_fridge = None
                    self._start_approach_to_fridge(fridge, auto_enable=True)
                else:
                    self.get_logger().info("[NinjaMission] Mission complete.")
                    self._set_phase(DONE)
                    self.mission = None

        # ── DONE / ABORTED / ERROR ─────────────────────────────────────────
        elif self.phase in (DONE, ABORTED, ERROR):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = NinjaMissionNode()
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
