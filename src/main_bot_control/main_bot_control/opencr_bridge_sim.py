#!/usr/bin/env python3
"""
opencr_bridge_sim.py — Faithful simulation of the ESP32/OpenCR drive controller.

Ports the full nav state machine, drive controller, telemetry protocol, and
command parsing from Full_flip_drive.ino. Dustpan/flipper/carwash mechanics
are stubbed (ACK'd immediately, DONE'd).

Drives the Gazebo robot via /cmd_vel. In simulation, the drive controller uses
the configured localization pose source, defaulting to /bot_pose_fused.
Ground truth is for telemetry/debug only unless explicitly selected in YAML.

Adds a preferred "highway" path planner for GO/GO_CENTER/GO_DUSTPAN commands.
Arena geometry, highway lines, target locations, forced exits, speeds, and
approach behavior are loaded from config/control_tuning.yaml. The node now
fails loudly if the config is missing or incomplete; tuning must happen in YAML.
"""

import math
import time
import heapq
import os
import json

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty, Float32, Float64, Int32, Int32MultiArray, String


# ── Helpers ───────────────────────────────────────────────────────

PI = math.pi


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def deg2rad(d: float) -> float:
    return d * PI / 180.0


def rad2deg(r: float) -> float:
    return r * 180.0 / PI


def clampf(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def step_toward(current: float, target: float, max_step: float) -> float:
    if max_step <= 0.0:
        return target
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + math.copysign(max_step, delta)


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        v = value.strip().lower()
        if v in ("1", "true", "yes", "on"):
            return True
        if v in ("0", "false", "no", "off"):
            return False
    raise ValueError(f"not a boolean: {value!r}")


def quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ── Nav states ────────────────────────────────────────────────────

class NavState:
    DRIVE_TO_APPROACH = "DRIVE_TO_APPROACH"
    ALIGN_FOR_APPROACH = "ALIGN_FOR_APPROACH"
    DRIVE_TO_POINT = "DRIVE_TO_POINT"
    ALIGN_FINAL_YAW = "ALIGN_FINAL_YAW"
    GOAL_REACHED = "GOAL_REACHED"


class RunMode:
    IDLE = "IDLE"
    GOAL = "GOAL"


class RefPoint:
    AXIS = 0
    CENTER = 1
    DUSTPAN = 2


# ── Main node ─────────────────────────────────────────────────────

class OpenCRBridgeSim(Node):
    def __init__(self):
        super().__init__("opencr_bridge_sim")

        # Config-owned values.  Do not put arena geometry, target points, or
        # tuned speeds here; they are loaded from control_tuning.yaml below.
        self._init_config_placeholders()

        # Command ramping runtime state.
        self._cmd_linear_mm_s = 0.0
        self._cmd_yaw_rad_s = 0.0
        self._last_cmd_time = time.monotonic()

        # Controller runtime state.
        self.alpha_integral = 0.0

        self.go_home_phase = 0
        self.home_stage_x = 0.0
        self.home_stage_y = 0.0

        # ── State ─────────────────────────────────────────────
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.theta_rad = 0.0
        self.odom_valid = False
        self.drive_pose_time_s = None

        # Opponent pose from overhead camera. Used only for collision stop;
        # a missing/stale opponent pose never blocks startup or strategy execution.
        self.enemy_x_mm = 0.0
        self.enemy_y_mm = 0.0
        self.enemy_theta_rad = 0.0
        self.enemy_pose_valid = False
        self.enemy_pose_time_s = None
        self.avoidance_blocked = False
        self.avoidance_action = "clear"
        self._avoidance_was_blocked = False
        self._last_avoidance_status_publish_s = 0.0

        self.odom_x_m = 0.0
        self.odom_y_m = 0.0
        self.odom_yaw = 0.0

        self.imu_yaw_deg = 0.0
        self.imu_gz = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.goal_active = False
        self.goal_label = ""
        self.goal_ref = RefPoint.AXIS

        self.approach_x = 0.0
        self.approach_y = 0.0
        self.approach_active = False

        self.run_mode = RunMode.IDLE
        self.nav_state = NavState.GOAL_REACHED

        self.final_yaw_in_tol_since = None

        # Active highway waypoint path.
        # Each item is (axis_x_mm, axis_y_mm, yaw_rad).
        self.path_active = False
        self.path_waypoints = []
        self.path_index = 0
        # Indices of generated safe-corner arc waypoints. These are allowed to
        # roll through without a stop/yaw settle, but only at corner speed.
        self.path_smooth_indices = set()
        # Indices that must be driven as fixed-yaw straight movements
        # (final pickup/placement push, and reverse backup if explicitly used).
        self.path_fixed_yaw_indices = set()
        self.path_force_reverse_indices = set()
        self._pending_smooth_indices = set()
        self.path_final_label = ""
        self.path_final_ref = RefPoint.AXIS
        # Explicit dustpan ownership for fixed-yaw paths.
        # zone/pantry actions set this True; generic drive_path leaves it False.
        self.path_auto_dustpan = False
        self.path_start_x = 0.0
        self.path_start_y = 0.0
        self._smooth_turn_tier = None  # "aggressive", "conservative", or None

        # If a GO command arrives before the first world pose, do NOT fall back
        # to the old direct-approach behavior. Queue it and start the highway
        # planner as soon as /bot_pose_ground_truth arrives.
        self.pending_goal = None

        # Used for zone approach: face the zone, reverse a little, then drive forward.
        self.align_for_approach_override_yaw = None

        self.brick_state = ['?', '?', '?', '?']

        self.telemetry_hz = 10.0

        # Actuator runtime state. Actual actuator angles/speeds are config-owned
        # and loaded from control_tuning.yaml before these state variables are initialized.
        self._last_actuator_update = time.monotonic()

        # Track whether target/exit maps came from YAML. If they did,
        # _refresh_derived_tuning_values() must not overwrite them with
        # fallback-derived exits.
        self._jenga_forced_exit_from_config = False
        self._pantry_forced_exit_from_config = False

        # Load every tuning value from control_tuning.yaml. Missing config is a
        # hard error so stale Python values cannot silently control the robot.
        self._load_control_config()
        self._refresh_derived_tuning_values()

        self.dustpan_angle = self.DUSTPAN_UP_ANGLE  # actual commanded angle ramped toward target
        self.dustpan_target_angle = self.DUSTPAN_UP_ANGLE
        self._dustpan_lowered_time = None  # tracks when dustpan was lowered for approach
        self.carwash_arm_angle = self.CARWASH_ARM_UP
        self.carwash_arm_target_angle = self.CARWASH_ARM_UP
        self.carwash_roller_speed = self.CARWASH_ROLLER_STOP
        self.carwash_roller_target_speed = self.CARWASH_ROLLER_STOP

        # ── Publishers ────────────────────────────────────────
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.dustpan_angle_pub = self.create_publisher(Float64, "/dustpan/angle", 10)
        self.carwash_arm_pub = self.create_publisher(Float64, "/carwash/arm_angle", 10)
        self.carwash_roller_pub = self.create_publisher(Float64, "/carwash/roller_speed", 10)
        self.connected_pub = self.create_publisher(Bool, "/opencr/connected", 10)
        self.status_pub = self.create_publisher(String, "/opencr/status", 20)
        self.odom_pose_pub = self.create_publisher(Pose2D, "/opencr/odom_pose", 20)
        self.goal_pose_pub = self.create_publisher(Pose2D, "/opencr/goal_pose", 20)
        self.highway_path_pub = self.create_publisher(String, "/opencr/highway_path", 20)
        self.avoidance_status_pub = self.create_publisher(String, "/opencr/avoidance_status", 20)
        self.imu_yaw_deg_pub = self.create_publisher(Float32, "/opencr/imu_yaw_deg", 20)
        self.gyro_z_pub = self.create_publisher(Float32, "/opencr/gyro_z", 20)
        self.ack_pub = self.create_publisher(String, "/opencr/ack", 20)
        self.done_pub = self.create_publisher(String, "/opencr/done", 20)
        self.error_pub = self.create_publisher(String, "/opencr/error", 20)
        self.event_pub = self.create_publisher(String, "/opencr/event", 20)
        self.brick_state_pub = self.create_publisher(String, "/opencr/brick_state", 20)
        self.gains_pub = self.create_publisher(String, "/opencr/gains", 20)
        self.raw_line_pub = self.create_publisher(String, "/opencr/raw_line", 50)

        # ── Subscribers (same topics as real opencr_bridge) ───
        self.create_subscription(Pose2D, "/opencr/cmd/go", self._go_cb, 10)
        self.create_subscription(Pose2D, "/opencr/cmd/go_center", self._go_center_cb, 10)
        self.create_subscription(Pose2D, "/opencr/cmd/go_dustpan", self._go_dustpan_cb, 10)
        self.create_subscription(Empty, "/opencr/cmd/go_home", self._go_home_cb, 10)
        self.create_subscription(Int32, "/opencr/cmd/go_zone", self._go_zone_cb, 10)
        self.create_subscription(Int32, "/opencr/cmd/go_pantry", self._go_pantry_cb, 10)
        self.create_subscription(Empty, "/opencr/cmd/stop", self._stop_cb, 10)
        self.create_subscription(Empty, "/opencr/cmd/estop", self._estop_cb, 10)
        self.create_subscription(Int32, "/opencr/cmd/flip", self._flip_cb, 10)
        self.create_subscription(Int32MultiArray, "/opencr/cmd/flip_seq", self._flip_seq_cb, 10)
        self.create_subscription(String, "/opencr/cmd/set_pattern", self._set_pattern_cb, 10)
        self.create_subscription(String, "/opencr/cmd/set_bricks", self._set_bricks_cb, 10)
        self.create_subscription(Pose2D, "/opencr/cmd/reset_odom", self._reset_odom_cb, 10)
        self.create_subscription(Empty, "/opencr/cmd/get_state", self._get_state_cb, 10)
        self.create_subscription(Int32, "/opencr/cmd/telemetry_hz", self._telem_hz_cb, 10)
        self.create_subscription(String, "/opencr/cmd/raw", self._raw_cb, 10)

        # ── Mission sequencer ─────────────────────────────────
        # Automates: go_zone → carwash_pull → go_pantry → carwash_push → backup
        self.mission_active = False
        self.mission_state = "IDLE"
        self.mission_zone_id = 0
        self.mission_pantry_id = 0
        self.mission_timer_start = 0.0
        self.mission_queue = []  # list of (zone_id, pantry_id) pairs

        # Total mission timer:
        # Starts when a valid mission command is received.
        # Stops when MISSION_ALL_COMPLETE is sent on /opencr/done.
        self.mission_total_start_time = None
        self.mission_total_task_count = 0

        # ── Strategy sequencer ───────────────────────────────
        # Generic config-driven route runner. Strategy content is loaded from
        # control_tuning.yaml; the Python code only knows how to execute step
        # types. This keeps strategy coordinates and target choices tunable.
        self.strategy_active = False
        self.strategy_name = ""
        self.strategy_steps = []
        self.strategy_index = 0
        self.strategy_waiting_drive_label = None
        self.strategy_timer_until = None
        self.strategy_timer_kind = None
        self.strategy_total_start_time = None
        self.strategy_finish_behavior = {}

        # One-shot guard for the configurable match-time stop-all.
        self._match_stop_all_sent = False

        self.create_subscription(String, "/opencr/cmd/mission", self._mission_cb, 10)

        # Main pose feed used by the drive controller. This is intentionally
        # configurable and defaults to /bot_pose_fused. Use /bot_pose_ground_truth
        # only when deliberately debugging ideal sim behavior.
        self.create_subscription(Pose2D, self.DRIVE_POSE_TOPIC, self._drive_pose_cb, 10)
        self.get_logger().warn(f"DRIVE CONTROLLER POSE SOURCE: {self.DRIVE_POSE_TOPIC}")
        self.create_subscription(Pose2D, "/vision/enemy_pose", self._enemy_pose_cb, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(Imu, "/imu", self._imu_cb, 10)

        # ── Timers ────────────────────────────────────────────
        self.create_timer(0.005, self._control_loop)   # 200 Hz like the .ino (5ms)
        self.create_timer(0.1, self._telemetry_loop)   # 10 Hz

        self._pub_bool(self.connected_pub, True)
        self._pub_str(self.event_pub, "READY")
        self.get_logger().info("opencr_bridge_sim started (ESP32 faithful port + highway planner)")

    def _sim_time_s(self) -> float:
        """Current ROS time in seconds.

        In simulation, sim.launch.py sets use_sim_time=True for this node, so
        this follows Gazebo /clock. On the real robot it follows wall time.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    # ══════════════════════════════════════════════════════════
    #  YAML tuning config
    # ══════════════════════════════════════════════════════════
    def _init_config_placeholders(self):
        """
        Placeholders only. Real tuning values must come from control_tuning.yaml.

        The point of this method is to make stale arena/speed constants impossible
        to tune accidentally. If YAML is missing or incomplete, _load_control_config()
        raises before the robot starts.
        """
        # Robot/field geometry.
        self.OFFSET_AXIS = 0.0
        self.OFFSET_CENTER = 0.0
        self.OFFSET_DUSTPAN = 0.0
        self.FIELD_X_MIN = 0.0
        self.FIELD_X_MAX = 0.0
        self.FIELD_Y_MIN = 0.0
        self.FIELD_Y_MAX = 0.0
        self.BOT_FRONT = 0.0
        self.BOT_REAR = 0.0
        self.BOT_HALF_W = 0.0

        # Localization source used by the simulated drive controller.
        # Default is the fused estimate, not Gazebo ground truth.
        self.DRIVE_POSE_TOPIC = "/bot_pose_fused"
        self.DRIVE_POSE_TIMEOUT_S = 0.50

        # Opponent perimeter / avoidance decision tree. These values are optional
        # in YAML so older configs can still start, but tune them in
        # control_tuning.yaml instead of editing Python.
        self.AVOIDANCE_ENABLED = True
        self.AVOIDANCE_ENEMY_POSE_TIMEOUT_S = 0.50
        self.AVOIDANCE_OPPONENT_FRONT_MARGIN_MM = 500.0
        self.AVOIDANCE_OPPONENT_REAR_MARGIN_MM = 300.0
        self.AVOIDANCE_OPPONENT_SIDE_MARGIN_MM = 300.0
        self.AVOIDANCE_DECISION_INSIDE_PERIMETER = "stop"  # stop | escape
        self.AVOIDANCE_DECISION_GOAL_INSIDE_PERIMETER = "hold"  # hold | ignore
        self.AVOIDANCE_STOP_PUBLISH_ERROR = True
        self.AVOIDANCE_HOLD_PUBLISH_ERROR = False
        self.AVOIDANCE_ESCAPE_LINEAR_MM_S = 80.0
        self.AVOIDANCE_ESCAPE_MAX_YAW_RAD_S = 0.45
        self.AVOIDANCE_ESCAPE_YAW_GAIN = 1.5
        self.AVOIDANCE_ESCAPE_YAW_TOL_RAD = deg2rad(15.0)

        # Team-side selection. YAML targets are authored for yellow;
        # blue mirrors autonomous targets across TEAM_MIRROR_LINE_X_MM.
        self.TEAM_COLOR = "yellow"
        self.TEAM_MIRROR_LINE_X_MM = 1500.0

        # Match-time safety stop. Loaded from control_tuning.yaml/match_timer.
        self.MATCH_TIMER_ENABLED = True
        self.MATCH_STOP_ALL_AT_S = 100.0

        # Highway planner / corridor.
        self.USE_HIGHWAY_PLANNER = False
        self.USE_HIGHWAY_FOR_HOME = False
        self.HIGHWAY_X = []
        self.HIGHWAY_Y = []
        self.HIGHWAY_NODE_EPS = 1e-3
        self.HIGHWAY_OUTER_X_MIN = 0.0
        self.HIGHWAY_OUTER_X_MAX = 0.0
        self.HIGHWAY_OUTER_Y_MIN = 0.0
        self.HIGHWAY_OUTER_Y_MAX = 0.0
        self.HIGHWAY_INNER_X_MIN = 0.0
        self.HIGHWAY_INNER_X_MAX = 0.0
        self.HIGHWAY_INNER_Y_MIN = 0.0
        self.HIGHWAY_INNER_Y_MAX = 0.0
        self.HIGHWAY_MIN_DIRECT_DIST = 0.0
        self.HIGHWAY_DUPLICATE_TOL = 0.0
        self.HIGHWAY_PASS_THROUGH_RADIUS = 0.0
        self.HIGHWAY_STRAIGHT_ANGLE_TOL = 0.0
        self.HIGHWAY_OVERSHOOT_CROSS_TRACK_MM = 0.0
        self.HIGHWAY_BEHIND_SKIP_DIST_MM = 0.0
        self.HIGHWAY_BEHIND_SKIP_ANGLE = 0.0
        self.HIGHWAY_ENTRY_SELECTION_SLACK_MM = 0.0
        self.HIGHWAY_SMOOTH_TURN_AGGRESSIVE = 0.0
        self.HIGHWAY_SMOOTH_TURN_CONSERVATIVE = 0.0
        self.HIGHWAY_SPEED_MM_S = 0.0
        self.HIGHWAY_ENTRY_EXIT_SPEED_MM_S = 0.0
        self.HIGHWAY_TURN_SPEED_MM_S = 0.0
        self.HIGHWAY_CORNER_CUT_ENABLED = False
        self.HIGHWAY_CORNER_CUT_DIST_MM = 0.0
        self.HIGHWAY_CORNER_CUT_MIN_SEG_MM = 0.0
        self.HIGHWAY_CORNER_CUT_MIN_ANGLE = 0.0
        self.HIGHWAY_CORNER_CUT_MAX_ANGLE = 0.0
        self.HIGHWAY_CORNER_ARC_POINTS = 0
        self.HIGHWAY_CORNER_PASS_RADIUS = 0.0
        self.HIGHWAY_CORNER_PASS_ANGLE = 0.0
        self.HIGHWAY_CORNER_SAFETY_MARGIN = 0.0
        self.HIGHWAY_CORNER_SPEED_MM_S = 0.0

        # Mission target maps.
        self.JENGA_ZONES = {}
        self.JENGA_ZONE_YAWS_DEG = {}
        self.JENGA_ZONE_FORCED_HIGHWAY_EXIT = {}
        self.JENGA_ZONE_TARGET_OFFSETS_MM = {}
        self.PANTRY_LOCATIONS = {}
        self.PANTRY_APPROACH_YAWS_DEG = {}
        self.PANTRY_FORCED_HIGHWAY_EXIT = {}
        self.PANTRY_TARGET_OFFSETS_MM = {}
        self.PANTRY_HALF_SIZE_MM = 0.0

        # Named strategy routes loaded from YAML.
        self.STRATEGIES = {}

        # Drive limits and gains.
        self.BASE_LINEAR_MM_S = 0.0
        self.BASE_YAW_RAD_S = 0.0
        self.MAX_LINEAR_MM_S = 0.0
        self.MAX_REVERSE_MM_S = 0.0
        self.MAX_YAW_RAD_S = 0.0
        self.AUTO_K_RHO_FROM_BASE_SPEED = False
        self.DRIVE_REACH_BASE_SPEED_DIST_MM = 0.0
        self.CMD_RAMP_ENABLED = True
        self.LINEAR_ACCEL_MM_S2 = 0.0
        self.LINEAR_DECEL_MM_S2 = 0.0
        self.YAW_ACCEL_RAD_S2 = 0.0
        self.YAW_DECEL_RAD_S2 = 0.0
        self.K_rho = 0.0
        self.K_alpha = 0.0
        self.K_alpha_i = 0.0
        self.ALPHA_I_LIMIT = 0.0
        self.FINAL_YAW_K = 0.0
        self.FINAL_YAW_MIN = 0.0
        self.FINAL_YAW_MAX = 0.0
        self.FINAL_YAW_HOLD_S = 0.0
        self.POS_TOL_MM = 0.0
        self.FINAL_YAW_TOL = 0.0

        # Approach/reverse/home/mission.
        self.APPROACH_OFFSET = 0.0
        self.APPROACH_MIN_DIST = 0.0
        self.REVERSE_MAX_DIST = 0.0
        self.ALLOW_REVERSE = False
        self.REVERSE_HEADING_THRESHOLD_DEG = 0.0
        self.BOUNDARY_SLOWDOWN_DIST = 0.0
        self.BOUNDARY_MIN_SPEED_FRAC = 0.0
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_yaw_deg = 0.0
        self.HOME_STAGING_OFFSET = 0.0
        self.HOME_REVERSE_SPEED = 0.0
        self.JENGA_APPROACH_BACKUP_MM = 0.0
        self.JENGA_APPROACH_START_MM = 0.0
        self.JENGA_APPROACH_MIN_ALIGN_STANDOFF_MM = 0.0
        self.JENGA_APPROACH_PREFER_FORWARD_START = False
        self.JENGA_APPROACH_FORBIDDEN_MARGIN = 0.0
        self.JENGA_STRAIGHT_SPEED_MM_S = 0.0
        self.JENGA_STRAIGHT_MIN_SPEED_MM_S = 0.0
        self.JENGA_STRAIGHT_YAW_TOL = 0.0
        self.JENGA_CORRECTION_REVERSE_ENABLED = False
        self.JENGA_CORRECTION_REVERSE_SPEED_MM_S = 0.0
        self.PANTRY_APPROACH_BACKUP_MM = 0.0
        self.PANTRY_APPROACH_START_MM = 0.0
        self.PANTRY_APPROACH_MIN_ALIGN_STANDOFF_MM = 0.0
        self.PANTRY_APPROACH_PREFER_FORWARD_START = False
        self.PANTRY_STRAIGHT_SPEED_MM_S = 0.0
        self.PANTRY_STRAIGHT_MIN_SPEED_MM_S = 0.0
        self.PANTRY_CORRECTION_REVERSE_ENABLED = False
        self.PANTRY_CORRECTION_REVERSE_SPEED_MM_S = 0.0
        self.MISSION_CARWASH_PULL_TIME = 0.0
        self.MISSION_CARWASH_PUSH_TIME = 0.0
        self.MISSION_SETTLE_TIME = 0.0
        self.MISSION_BACKUP_DIST_MM = 0.0
        self.MISSION_BACKUP_SPEED_MM_S = 0.0

        # Actuators.
        self.DUSTPAN_UP_ANGLE = 0.0
        self.DUSTPAN_DOWN_ANGLE = 0.0
        self.DUSTPAN_MOVE_SPEED_RAD_S = 0.0
        self.DUSTPAN_LOWER_SETTLE_S = 0.0
        self.CARWASH_ARM_UP = 0.0
        self.CARWASH_ARM_DOWN = 0.0
        self.CARWASH_ARM_MOVE_SPEED_RAD_S = 0.0
        self.CARWASH_ROLLER_PULL = 0.0
        self.CARWASH_ROLLER_PUSH = 0.0
        self.CARWASH_ROLLER_STOP = 0.0
        self.CARWASH_ROLLER_ACCEL_RAD_S2 = 0.0

    def _require_cfg_keys(self, cfg, keys, source_path):
        missing = []
        for dotted in keys:
            node = cfg
            for part in dotted.split('.'):
                if not isinstance(node, dict) or part not in node:
                    missing.append(dotted)
                    break
                node = node[part]
        if missing:
            preview = ', '.join(missing[:25])
            extra = '' if len(missing) <= 25 else f' ... and {len(missing) - 25} more'
            raise RuntimeError(
                f"control_tuning.yaml is missing required opencr_bridge_sim keys in {source_path}: "
                f"{preview}{extra}. Refusing to run with hidden Python defaults."
            )

    def _lookup_cfg(self, cfg, path):
        cur = cfg
        for key in path:
            if not isinstance(cur, dict) or key not in cur:
                return None
            cur = cur[key]
        return cur

    def _apply_cfg_value(self, cfg, path, attr, converter=float):
        value = self._lookup_cfg(cfg, path)
        if value is None:
            return
        try:
            setattr(self, attr, converter(value))
        except Exception as exc:
            dotted = ".".join(path)
            self.get_logger().warn(
                f"Ignoring invalid control_tuning.yaml value {dotted}={value!r}: {exc}"
            )

    def _apply_cfg_list(self, cfg, path, attr, converter=float):
        value = self._lookup_cfg(cfg, path)
        if value is None:
            return
        if not isinstance(value, (list, tuple)) or len(value) < 1:
            dotted = ".".join(path)
            self.get_logger().warn(f"Ignoring invalid control_tuning.yaml list {dotted}={value!r}")
            return
        try:
            setattr(self, attr, [converter(v) for v in value])
        except Exception as exc:
            dotted = ".".join(path)
            self.get_logger().warn(
                f"Ignoring invalid control_tuning.yaml list {dotted}={value!r}: {exc}"
            )

    def _coerce_cfg_point(self, value):
        if isinstance(value, dict):
            if "x_mm" in value and "y_mm" in value:
                return float(value["x_mm"]), float(value["y_mm"])
            if "x" in value and "y" in value:
                return float(value["x"]), float(value["y"])
        if isinstance(value, (list, tuple)) and len(value) >= 2:
            return float(value[0]), float(value[1])
        raise ValueError(f"expected [x_mm, y_mm] or {{}}, got {value!r}")

    def _apply_cfg_point_map(self, cfg, path, attr):
        value = self._lookup_cfg(cfg, path)
        if value is None:
            return False
        dotted = ".".join(path)
        if not isinstance(value, dict):
            self.get_logger().warn(f"Ignoring invalid control_tuning.yaml map {dotted}={value!r}")
            return False
        out = {}
        try:
            for key, point in value.items():
                out[int(key)] = self._coerce_cfg_point(point)
        except Exception as exc:
            self.get_logger().warn(
                f"Ignoring invalid control_tuning.yaml map {dotted}={value!r}: {exc}"
            )
            return False
        setattr(self, attr, out)
        return True

    def _apply_cfg_yaw_map(self, cfg, path, attr):
        value = self._lookup_cfg(cfg, path)
        if value is None:
            return False
        dotted = ".".join(path)
        if not isinstance(value, dict):
            self.get_logger().warn(f"Ignoring invalid control_tuning.yaml map {dotted}={value!r}")
            return False
        out = {}
        try:
            for key, yaw in value.items():
                out[int(key)] = float(yaw)
        except Exception as exc:
            self.get_logger().warn(
                f"Ignoring invalid control_tuning.yaml map {dotted}={value!r}: {exc}"
            )
            return False
        setattr(self, attr, out)
        return True

    def _cfg_number(self, cfg, path, default):
        value = self._lookup_cfg(cfg, path)
        if value is None:
            return default
        try:
            return float(value)
        except Exception as exc:
            dotted = ".".join(path)
            self.get_logger().warn(
                f"Ignoring invalid control_tuning.yaml value {dotted}={value!r}: {exc}"
            )
            return default

    def _cfg_bool(self, cfg, path, default):
        value = self._lookup_cfg(cfg, path)
        if value is None:
            return default
        return parse_bool(value)

    def _apply_base_speed_profile(self, cfg):
        """
        If config contains opencr_bridge_sim.base_speeds, use that as the
        master tuning system.

        This intentionally overwrites the old fixed speed fields after they are
        loaded, so there is one clear place to change global speed:
          base_speeds.linear_mm_s
          base_speeds.yaw_rad_s
        """
        base = self._lookup_cfg(cfg, ["base_speeds"])
        if not isinstance(base, dict):
            return

        self.BASE_LINEAR_MM_S = max(
            1.0,
            self._cfg_number(cfg, ["base_speeds", "linear_mm_s"], self.BASE_LINEAR_MM_S),
        )
        self.BASE_YAW_RAD_S = max(
            0.01,
            self._cfg_number(cfg, ["base_speeds", "yaw_rad_s"], self.BASE_YAW_RAD_S),
        )

        def sf(name, default):
            return self._cfg_number(cfg, ["speed_factors", name], default)

        def af(name, default):
            return self._cfg_number(cfg, ["acceleration_factors", name], default)

        self.AUTO_K_RHO_FROM_BASE_SPEED = self._cfg_bool(
            cfg, ["drive_response", "auto_k_rho_from_base_speed"], True
        )
        self.DRIVE_REACH_BASE_SPEED_DIST_MM = max(
            1.0,
            self._cfg_number(
                cfg,
                ["drive_response", "reach_base_speed_dist_mm"],
                self.DRIVE_REACH_BASE_SPEED_DIST_MM,
            ),
        )

        # Absolute command limits.
        self.MAX_LINEAR_MM_S = self.BASE_LINEAR_MM_S * sf("max_linear", 1.0)
        self.MAX_REVERSE_MM_S = self.BASE_LINEAR_MM_S * sf("max_reverse", 0.65)
        self.MAX_YAW_RAD_S = self.BASE_YAW_RAD_S * sf("max_yaw", 1.0)

        # Command ramping / acceleration.
        self.LINEAR_ACCEL_MM_S2 = self.BASE_LINEAR_MM_S * af("linear_accel_per_s", 3.0)
        self.LINEAR_DECEL_MM_S2 = self.BASE_LINEAR_MM_S * af("linear_decel_per_s", 4.0)
        self.YAW_ACCEL_RAD_S2 = self.BASE_YAW_RAD_S * af("yaw_accel_per_s", 3.0)
        self.YAW_DECEL_RAD_S2 = self.BASE_YAW_RAD_S * af("yaw_decel_per_s", 4.0)

        # Make the point controller more aggressive when base speed increases.
        if self.AUTO_K_RHO_FROM_BASE_SPEED:
            self.K_rho = self.BASE_LINEAR_MM_S / self.DRIVE_REACH_BASE_SPEED_DIST_MM

        # Highway speeds.
        self.HIGHWAY_SPEED_MM_S = self.BASE_LINEAR_MM_S * sf("highway", 1.0)
        self.HIGHWAY_ENTRY_EXIT_SPEED_MM_S = self.BASE_LINEAR_MM_S * sf("highway_entry_exit", 0.8)
        self.HIGHWAY_TURN_SPEED_MM_S = self.BASE_LINEAR_MM_S * sf("highway_turn", 0.6)
        self.HIGHWAY_CORNER_SPEED_MM_S = self.BASE_LINEAR_MM_S * sf("highway_corner", sf("highway_turn", 0.6))

        # Final zone/pantry straight speeds.
        self.JENGA_STRAIGHT_SPEED_MM_S = self.BASE_LINEAR_MM_S * sf("zone_straight", 0.25)
        self.JENGA_STRAIGHT_MIN_SPEED_MM_S = self.BASE_LINEAR_MM_S * sf("zone_straight_min", 0.06)
        self.PANTRY_STRAIGHT_SPEED_MM_S = self.BASE_LINEAR_MM_S * sf("pantry_straight", 0.25)
        self.PANTRY_STRAIGHT_MIN_SPEED_MM_S = self.BASE_LINEAR_MM_S * sf("pantry_straight_min", 0.06)
        self.JENGA_CORRECTION_REVERSE_SPEED_MM_S = self.BASE_LINEAR_MM_S * sf("zone_correction_reverse", 0.10)
        self.PANTRY_CORRECTION_REVERSE_SPEED_MM_S = self.BASE_LINEAR_MM_S * sf("pantry_correction_reverse", 0.10)

        # Other mission linear speeds.
        self.HOME_REVERSE_SPEED = self.BASE_LINEAR_MM_S * sf("home_reverse", 0.30)
        self.MISSION_BACKUP_SPEED_MM_S = self.BASE_LINEAR_MM_S * sf("mission_backup", 0.20)

        # Yaw/final alignment speeds.
        self.FINAL_YAW_MAX = self.BASE_YAW_RAD_S * sf("final_yaw_max", 0.27)
        self.FINAL_YAW_MIN = self.BASE_YAW_RAD_S * sf("final_yaw_min", 0.02)

        # Sim actuator angular speeds.
        self.DUSTPAN_MOVE_SPEED_RAD_S = self.BASE_YAW_RAD_S * sf("dustpan_move", 1.0)
        self.CARWASH_ARM_MOVE_SPEED_RAD_S = self.BASE_YAW_RAD_S * sf("carwash_arm_move", 1.0)
        self.CARWASH_ROLLER_PULL = self.BASE_YAW_RAD_S * sf("roller_pull", 3.33)
        self.CARWASH_ROLLER_PUSH = -self.BASE_YAW_RAD_S * sf("roller_push", 3.33)
        self.CARWASH_ROLLER_ACCEL_RAD_S2 = self.BASE_YAW_RAD_S * sf("roller_accel", 16.67)

    def _refresh_derived_tuning_values(self):
        """Refresh values that depend on configurable highway lines/zone."""
        hx = sorted(float(v) for v in (self.HIGHWAY_X or []))
        hy = sorted(float(v) for v in (self.HIGHWAY_Y or []))

        # Fallback: if line values are missing, derive centerlines from the
        # hollow corridor geometry so the planner still has a sane route.
        if len(hx) < 2:
            hx = [
                0.5 * (self.HIGHWAY_OUTER_X_MIN + self.HIGHWAY_INNER_X_MIN),
                0.5 * (self.HIGHWAY_INNER_X_MAX + self.HIGHWAY_OUTER_X_MAX),
            ]
        if len(hy) < 2:
            hy = [
                0.5 * (self.HIGHWAY_OUTER_Y_MIN + self.HIGHWAY_INNER_Y_MIN),
                0.5 * (self.HIGHWAY_INNER_Y_MAX + self.HIGHWAY_OUTER_Y_MAX),
            ]

        self.HIGHWAY_X = [hx[0], hx[-1]]
        self.HIGHWAY_Y = [hy[0], hy[-1]]

        x_lo = min(self.HIGHWAY_X)
        x_hi = max(self.HIGHWAY_X)
        y_hi = max(self.HIGHWAY_Y)

        if not getattr(self, "_jenga_forced_exit_from_config", False):
            # Fallback-derived exits. If jenga_zones.forced_highway_exit is
            # present in YAML, this block is skipped completely.
            generated = {}
            for zone_id, (zx, zy) in self.JENGA_ZONES.items():
                if zone_id in (1, 2):
                    generated[zone_id] = (x_lo, zy)
                elif zone_id in (7, 8):
                    generated[zone_id] = (x_hi, zy)
                elif zone_id in (3, 5):
                    generated[zone_id] = (zx, y_hi)
            self.JENGA_ZONE_FORCED_HIGHWAY_EXIT = generated

        if not getattr(self, "_pantry_forced_exit_from_config", False):
            generated = {}
            for pantry_id, (px, py) in self.PANTRY_LOCATIONS.items():
                if pantry_id in (1, 2):
                    generated[pantry_id] = (px, y_hi)
                elif pantry_id == 3:
                    generated[pantry_id] = (x_lo, py)
                elif pantry_id == 7:
                    generated[pantry_id] = (x_hi, py)
            self.PANTRY_FORCED_HIGHWAY_EXIT = generated

        # Make obviously invalid speed tuning safer.
        self.MAX_LINEAR_MM_S = max(0.0, self.MAX_LINEAR_MM_S)
        self.MAX_REVERSE_MM_S = max(0.0, self.MAX_REVERSE_MM_S)
        self.MAX_YAW_RAD_S = max(0.0, self.MAX_YAW_RAD_S)
        self.LINEAR_ACCEL_MM_S2 = max(0.0, self.LINEAR_ACCEL_MM_S2)
        self.LINEAR_DECEL_MM_S2 = max(0.0, self.LINEAR_DECEL_MM_S2)
        self.YAW_ACCEL_RAD_S2 = max(0.0, self.YAW_ACCEL_RAD_S2)
        self.YAW_DECEL_RAD_S2 = max(0.0, self.YAW_DECEL_RAD_S2)
        self.HIGHWAY_SPEED_MM_S = max(0.0, self.HIGHWAY_SPEED_MM_S)
        self.HIGHWAY_ENTRY_EXIT_SPEED_MM_S = max(0.0, self.HIGHWAY_ENTRY_EXIT_SPEED_MM_S)
        self.HIGHWAY_TURN_SPEED_MM_S = max(0.0, self.HIGHWAY_TURN_SPEED_MM_S)
        self.HIGHWAY_CORNER_SPEED_MM_S = max(0.0, self.HIGHWAY_CORNER_SPEED_MM_S)
        self.HIGHWAY_OVERSHOOT_CROSS_TRACK_MM = max(1.0, self.HIGHWAY_OVERSHOOT_CROSS_TRACK_MM)
        self.HIGHWAY_BEHIND_SKIP_DIST_MM = max(1.0, self.HIGHWAY_BEHIND_SKIP_DIST_MM)
        self.HIGHWAY_BEHIND_SKIP_ANGLE = clampf(self.HIGHWAY_BEHIND_SKIP_ANGLE, 0.0, PI)
        self.JENGA_STRAIGHT_SPEED_MM_S = max(0.0, self.JENGA_STRAIGHT_SPEED_MM_S)
        self.JENGA_STRAIGHT_MIN_SPEED_MM_S = max(0.0, self.JENGA_STRAIGHT_MIN_SPEED_MM_S)
        self.PANTRY_STRAIGHT_SPEED_MM_S = max(0.0, self.PANTRY_STRAIGHT_SPEED_MM_S)
        self.PANTRY_STRAIGHT_MIN_SPEED_MM_S = max(0.0, self.PANTRY_STRAIGHT_MIN_SPEED_MM_S)
        self.AVOIDANCE_ENEMY_POSE_TIMEOUT_S = max(0.05, float(self.AVOIDANCE_ENEMY_POSE_TIMEOUT_S))
        self.AVOIDANCE_OPPONENT_FRONT_MARGIN_MM = max(0.0, float(self.AVOIDANCE_OPPONENT_FRONT_MARGIN_MM))
        self.AVOIDANCE_OPPONENT_REAR_MARGIN_MM = max(0.0, float(self.AVOIDANCE_OPPONENT_REAR_MARGIN_MM))
        self.AVOIDANCE_OPPONENT_SIDE_MARGIN_MM = max(0.0, float(self.AVOIDANCE_OPPONENT_SIDE_MARGIN_MM))
        self.JENGA_APPROACH_MIN_ALIGN_STANDOFF_MM = max(0.0, self.JENGA_APPROACH_MIN_ALIGN_STANDOFF_MM)
        self.PANTRY_APPROACH_MIN_ALIGN_STANDOFF_MM = max(0.0, self.PANTRY_APPROACH_MIN_ALIGN_STANDOFF_MM)
        self.JENGA_CORRECTION_REVERSE_SPEED_MM_S = max(0.0, self.JENGA_CORRECTION_REVERSE_SPEED_MM_S)
        self.PANTRY_CORRECTION_REVERSE_SPEED_MM_S = max(0.0, self.PANTRY_CORRECTION_REVERSE_SPEED_MM_S)
        self.HOME_REVERSE_SPEED = max(0.0, self.HOME_REVERSE_SPEED)
        self.REVERSE_MAX_DIST = max(0.0, self.REVERSE_MAX_DIST)
        self.REVERSE_HEADING_THRESHOLD_DEG = clampf(self.REVERSE_HEADING_THRESHOLD_DEG, 0.0, 180.0)
        self.DUSTPAN_MOVE_SPEED_RAD_S = max(0.0, self.DUSTPAN_MOVE_SPEED_RAD_S)
        self.DUSTPAN_LOWER_SETTLE_S = max(0.0, self.DUSTPAN_LOWER_SETTLE_S)
        self.CARWASH_ARM_MOVE_SPEED_RAD_S = max(0.0, self.CARWASH_ARM_MOVE_SPEED_RAD_S)
        self.CARWASH_ROLLER_ACCEL_RAD_S2 = max(0.0, self.CARWASH_ROLLER_ACCEL_RAD_S2)
        self.MISSION_BACKUP_SPEED_MM_S = max(1.0, abs(self.MISSION_BACKUP_SPEED_MM_S))
        self.BOUNDARY_MIN_SPEED_FRAC = clampf(self.BOUNDARY_MIN_SPEED_FRAC, 0.0, 1.0)

    def _load_control_config(self):
        """
        Load simulation drive/actuator tuning from YAML.

        Default path after colcon install:
          install/main_bot_control/share/main_bot_control/config/control_tuning.yaml

        Override at launch/runtime with:
          ros2 run main_bot_control opencr_bridge_sim --ros-args \
            -p control_config_path:=/absolute/path/to/control_tuning.yaml
        """
        try:
            default_path = os.path.join(
                get_package_share_directory("main_bot_control"),
                "config",
                "control_tuning.yaml",
            )
        except Exception:
            default_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "config",
                "control_tuning.yaml",
            )

        if not self.has_parameter("control_config_path"):
            self.declare_parameter("control_config_path", default_path)
        cfg_path = self.get_parameter("control_config_path").get_parameter_value().string_value

        # Useful fallback when running this Python file directly from the source tree.
        candidates = [cfg_path]
        source_tree_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "config",
            "control_tuning.yaml",
        )
        if source_tree_path not in candidates:
            candidates.append(source_tree_path)

        actual_path = next((path for path in candidates if path and os.path.exists(path)), None)
        if actual_path is None:
            raise RuntimeError(
                f"control_tuning.yaml not found. Tried: {candidates}. "
                "Refusing to run with hidden Python defaults."
            )

        try:
            with open(actual_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            raise RuntimeError(f"Could not read control_tuning.yaml at {actual_path}: {exc}") from exc

        cfg = data.get("opencr_bridge_sim", data)
        if not isinstance(cfg, dict):
            raise RuntimeError(f"Invalid YAML root in {actual_path}: expected a mapping")

        required_keys = [
            "field.x_min_mm", "field.x_max_mm", "field.y_min_mm", "field.y_max_mm",
            "robot_geometry.offset_axis_mm", "robot_geometry.offset_center_mm",
            "robot_geometry.offset_dustpan_mm", "robot_geometry.bot_front_mm",
            "robot_geometry.bot_rear_mm", "robot_geometry.bot_half_width_mm",
            "jenga_zones.locations", "jenga_zones.approach_yaws_deg",
            "jenga_zones.forced_highway_exit",
            "pantries.half_size_mm", "pantries.locations", "pantries.approach_yaws_deg",
            "pantries.forced_highway_exit",
            "base_speeds.linear_mm_s", "base_speeds.yaw_rad_s",
            "drive_response.auto_k_rho_from_base_speed", "drive_response.reach_base_speed_dist_mm",
            "speed_factors.max_linear", "speed_factors.max_reverse", "speed_factors.max_yaw",
            "speed_factors.highway", "speed_factors.highway_entry_exit",
            "speed_factors.highway_turn", "speed_factors.highway_corner",
            "speed_factors.zone_straight", "speed_factors.zone_straight_min",
            "speed_factors.pantry_straight", "speed_factors.pantry_straight_min",
            "speed_factors.zone_correction_reverse", "speed_factors.pantry_correction_reverse",
            "speed_factors.home_reverse", "speed_factors.mission_backup",
            "speed_factors.final_yaw_max", "speed_factors.final_yaw_min",
            "speed_factors.dustpan_move", "speed_factors.carwash_arm_move",
            "speed_factors.roller_pull", "speed_factors.roller_push", "speed_factors.roller_accel",
            "acceleration_factors.linear_accel_per_s", "acceleration_factors.linear_decel_per_s",
            "acceleration_factors.yaw_accel_per_s", "acceleration_factors.yaw_decel_per_s",
            "drive_gains.k_rho", "drive_gains.k_alpha", "drive_gains.k_alpha_i",
            "drive_gains.alpha_i_limit",
            "goal_tolerances.position_mm", "goal_tolerances.final_yaw_deg",
            "final_yaw.k", "final_yaw.hold_s", "final_yaw.tolerance_deg",
            "highway.use_highway_planner", "highway.use_highway_for_home",
            "highway.x_lines_mm", "highway.y_lines_mm",
            "highway.outer_x_min_mm", "highway.outer_x_max_mm",
            "highway.outer_y_min_mm", "highway.outer_y_max_mm",
            "highway.inner_x_min_mm", "highway.inner_x_max_mm",
            "highway.inner_y_min_mm", "highway.inner_y_max_mm",
            "highway.min_direct_dist_mm", "highway.duplicate_tolerance_mm",
            "highway.pass_through_radius_mm", "highway.straight_angle_tolerance_deg",
            "highway.overshoot_cross_track_mm", "highway.behind_skip_dist_mm",
            "highway.behind_skip_angle_deg", "highway.entry_selection_slack_mm",
            "highway.smooth_turn_aggressive_deg", "highway.smooth_turn_conservative_deg",
            "highway.corner_cut_enabled", "highway.corner_cut_dist_mm",
            "highway.corner_cut_min_seg_mm", "highway.corner_cut_min_angle_deg",
            "highway.corner_cut_max_angle_deg", "highway.corner_arc_points",
            "highway.corner_pass_radius_mm", "highway.corner_pass_angle_deg",
            "highway.corner_safety_margin_mm",
            "approach_point.offset_mm", "approach_point.min_dist_mm",
            "reverse.enabled", "reverse.max_dist_mm", "reverse.heading_threshold_deg",
            "zone_approach.backup_mm", "zone_approach.start_mm",
            "zone_approach.min_align_standoff_mm", "zone_approach.prefer_forward_start",
            "zone_approach.forbidden_margin_mm", "zone_approach.straight_yaw_tolerance_deg",
            "zone_approach.correction_reverse_enabled",
            "pantry_approach.backup_mm", "pantry_approach.start_mm",
            "pantry_approach.min_align_standoff_mm", "pantry_approach.prefer_forward_start",
            "pantry_approach.correction_reverse_enabled",
            "home.x_mm", "home.y_mm", "home.yaw_deg", "home.staging_offset_mm",
            "boundary_slowdown.slowdown_dist_mm", "boundary_slowdown.min_speed_fraction",
            "dustpan.up_angle_deg", "dustpan.down_angle_deg", "dustpan.lower_settle_s",
            "carwash.arm_up_angle_deg", "carwash.arm_down_angle_deg", "carwash.roller_stop_rad_s",
            "mission.carwash_pull_time_s", "mission.carwash_push_time_s",
            "mission.settle_time_s", "mission.backup_dist_mm",
            "strategies.strategy_1.steps",
        ]
        self._require_cfg_keys(cfg, required_keys, actual_path)

        # Arena geometry and target maps: all real values come from YAML.
        self._apply_cfg_value(cfg, ["field", "x_min_mm"], "FIELD_X_MIN")
        self._apply_cfg_value(cfg, ["field", "x_max_mm"], "FIELD_X_MAX")
        self._apply_cfg_value(cfg, ["field", "y_min_mm"], "FIELD_Y_MIN")
        self._apply_cfg_value(cfg, ["field", "y_max_mm"], "FIELD_Y_MAX")

        self._apply_cfg_value(cfg, ["team", "color"], "TEAM_COLOR", lambda v: str(v).strip().lower())
        self._apply_cfg_value(cfg, ["team", "mirror_line_x_mm"], "TEAM_MIRROR_LINE_X_MM")

        self._apply_cfg_value(cfg, ["match_timer", "enabled"], "MATCH_TIMER_ENABLED", parse_bool)
        self._apply_cfg_value(cfg, ["match_timer", "stop_all_at_s"], "MATCH_STOP_ALL_AT_S")

        self._apply_cfg_value(cfg, ["robot_geometry", "offset_axis_mm"], "OFFSET_AXIS")
        self._apply_cfg_value(cfg, ["robot_geometry", "offset_center_mm"], "OFFSET_CENTER")
        self._apply_cfg_value(cfg, ["robot_geometry", "offset_dustpan_mm"], "OFFSET_DUSTPAN")
        self._apply_cfg_value(cfg, ["robot_geometry", "bot_front_mm"], "BOT_FRONT")
        self._apply_cfg_value(cfg, ["robot_geometry", "bot_rear_mm"], "BOT_REAR")
        self._apply_cfg_value(cfg, ["robot_geometry", "bot_half_width_mm"], "BOT_HALF_W")

        self._apply_cfg_value(
            cfg, ["localization", "drive_pose_topic"],
            "DRIVE_POSE_TOPIC", lambda v: str(v).strip(),
        )
        self._apply_cfg_value(
            cfg, ["localization", "drive_pose_timeout_s"],
            "DRIVE_POSE_TIMEOUT_S",
        )

        self._apply_cfg_value(cfg, ["avoidance", "enabled"], "AVOIDANCE_ENABLED", parse_bool)
        self._apply_cfg_value(cfg, ["avoidance", "enemy_pose_timeout_s"], "AVOIDANCE_ENEMY_POSE_TIMEOUT_S")
        self._apply_cfg_value(cfg, ["avoidance", "opponent_front_margin_mm"], "AVOIDANCE_OPPONENT_FRONT_MARGIN_MM")
        self._apply_cfg_value(cfg, ["avoidance", "opponent_rear_margin_mm"], "AVOIDANCE_OPPONENT_REAR_MARGIN_MM")
        self._apply_cfg_value(cfg, ["avoidance", "opponent_side_margin_mm"], "AVOIDANCE_OPPONENT_SIDE_MARGIN_MM")
        self._apply_cfg_value(
            cfg, ["avoidance", "decision_tree", "inside_perimeter"],
            "AVOIDANCE_DECISION_INSIDE_PERIMETER",
            lambda v: str(v).strip().lower(),
        )
        self._apply_cfg_value(
            cfg, ["avoidance", "decision_tree", "goal_inside_perimeter"],
            "AVOIDANCE_DECISION_GOAL_INSIDE_PERIMETER",
            lambda v: str(v).strip().lower(),
        )
        self._apply_cfg_value(
            cfg, ["avoidance", "decision_tree", "stop", "publish_error"],
            "AVOIDANCE_STOP_PUBLISH_ERROR", parse_bool,
        )
        self._apply_cfg_value(
            cfg, ["avoidance", "decision_tree", "hold", "publish_error"],
            "AVOIDANCE_HOLD_PUBLISH_ERROR", parse_bool,
        )
        self._apply_cfg_value(
            cfg, ["avoidance", "decision_tree", "escape", "linear_mm_s"],
            "AVOIDANCE_ESCAPE_LINEAR_MM_S",
        )
        self._apply_cfg_value(
            cfg, ["avoidance", "decision_tree", "escape", "max_yaw_rad_s"],
            "AVOIDANCE_ESCAPE_MAX_YAW_RAD_S",
        )
        self._apply_cfg_value(
            cfg, ["avoidance", "decision_tree", "escape", "yaw_gain"],
            "AVOIDANCE_ESCAPE_YAW_GAIN",
        )
        self._apply_cfg_value(
            cfg, ["avoidance", "decision_tree", "escape", "yaw_tolerance_deg"],
            "AVOIDANCE_ESCAPE_YAW_TOL_RAD", lambda v: deg2rad(float(v)),
        )
        if self.AVOIDANCE_DECISION_INSIDE_PERIMETER not in ("stop", "escape"):
            self.get_logger().warn(
                "Invalid avoidance.decision_tree.inside_perimeter="
                f"{self.AVOIDANCE_DECISION_INSIDE_PERIMETER!r}; using 'stop'"
            )
            self.AVOIDANCE_DECISION_INSIDE_PERIMETER = "stop"
        if self.AVOIDANCE_DECISION_GOAL_INSIDE_PERIMETER not in ("hold", "ignore"):
            self.get_logger().warn(
                "Invalid avoidance.decision_tree.goal_inside_perimeter="
                f"{self.AVOIDANCE_DECISION_GOAL_INSIDE_PERIMETER!r}; using 'hold'"
            )
            self.AVOIDANCE_DECISION_GOAL_INSIDE_PERIMETER = "hold"

        self._apply_cfg_point_map(cfg, ["jenga_zones", "locations"], "JENGA_ZONES")
        self._apply_cfg_yaw_map(cfg, ["jenga_zones", "approach_yaws_deg"], "JENGA_ZONE_YAWS_DEG")
        if not self._apply_cfg_point_map(cfg, ["jenga_zones", "target_offsets_mm"], "JENGA_ZONE_TARGET_OFFSETS_MM"):
            self.JENGA_ZONE_TARGET_OFFSETS_MM = {}
        self._jenga_forced_exit_from_config = self._apply_cfg_point_map(
            cfg, ["jenga_zones", "forced_highway_exit"], "JENGA_ZONE_FORCED_HIGHWAY_EXIT"
        )

        self._apply_cfg_value(cfg, ["pantries", "half_size_mm"], "PANTRY_HALF_SIZE_MM")
        self._apply_cfg_point_map(cfg, ["pantries", "locations"], "PANTRY_LOCATIONS")
        self._apply_cfg_yaw_map(cfg, ["pantries", "approach_yaws_deg"], "PANTRY_APPROACH_YAWS_DEG")
        if not self._apply_cfg_point_map(cfg, ["pantries", "target_offsets_mm"], "PANTRY_TARGET_OFFSETS_MM"):
            self.PANTRY_TARGET_OFFSETS_MM = {}
        self._pantry_forced_exit_from_config = self._apply_cfg_point_map(
            cfg, ["pantries", "forced_highway_exit"], "PANTRY_FORCED_HIGHWAY_EXIT"
        )

        strategies_cfg = self._lookup_cfg(cfg, ["strategies"])
        if not isinstance(strategies_cfg, dict):
            raise RuntimeError("control_tuning.yaml strategies must be a mapping")
        self.STRATEGIES = strategies_cfg

        self._apply_cfg_value(cfg, ["drive_limits", "max_linear_mm_s"], "MAX_LINEAR_MM_S")
        self._apply_cfg_value(cfg, ["drive_limits", "max_reverse_mm_s"], "MAX_REVERSE_MM_S")
        self._apply_cfg_value(cfg, ["drive_limits", "max_yaw_rad_s"], "MAX_YAW_RAD_S")

        self._apply_cfg_value(cfg, ["command_ramp", "enabled"], "CMD_RAMP_ENABLED", parse_bool)
        self._apply_cfg_value(cfg, ["command_ramp", "linear_accel_mm_s2"], "LINEAR_ACCEL_MM_S2")
        self._apply_cfg_value(cfg, ["command_ramp", "linear_decel_mm_s2"], "LINEAR_DECEL_MM_S2")
        self._apply_cfg_value(cfg, ["command_ramp", "yaw_accel_rad_s2"], "YAW_ACCEL_RAD_S2")
        self._apply_cfg_value(cfg, ["command_ramp", "yaw_decel_rad_s2"], "YAW_DECEL_RAD_S2")

        self._apply_cfg_value(cfg, ["drive_gains", "k_rho"], "K_rho")
        self._apply_cfg_value(cfg, ["drive_gains", "k_alpha"], "K_alpha")
        self._apply_cfg_value(cfg, ["drive_gains", "k_alpha_i"], "K_alpha_i")
        self._apply_cfg_value(cfg, ["drive_gains", "alpha_i_limit"], "ALPHA_I_LIMIT")

        self._apply_cfg_value(cfg, ["goal_tolerances", "position_mm"], "POS_TOL_MM")
        self._apply_cfg_value(cfg, ["goal_tolerances", "final_yaw_deg"], "FINAL_YAW_TOL", lambda v: deg2rad(float(v)))

        self._apply_cfg_value(cfg, ["final_yaw", "k"], "FINAL_YAW_K")
        self._apply_cfg_value(cfg, ["final_yaw", "min_rad_s"], "FINAL_YAW_MIN")
        self._apply_cfg_value(cfg, ["final_yaw", "max_rad_s"], "FINAL_YAW_MAX")
        self._apply_cfg_value(cfg, ["final_yaw", "hold_s"], "FINAL_YAW_HOLD_S")
        self._apply_cfg_value(cfg, ["final_yaw", "tolerance_deg"], "FINAL_YAW_TOL", lambda v: deg2rad(float(v)))

        self._apply_cfg_list(cfg, ["highway", "x_lines_mm"], "HIGHWAY_X")
        self._apply_cfg_list(cfg, ["highway", "y_lines_mm"], "HIGHWAY_Y")
        self._apply_cfg_value(cfg, ["highway", "outer_x_min_mm"], "HIGHWAY_OUTER_X_MIN")
        self._apply_cfg_value(cfg, ["highway", "outer_x_max_mm"], "HIGHWAY_OUTER_X_MAX")
        self._apply_cfg_value(cfg, ["highway", "outer_y_min_mm"], "HIGHWAY_OUTER_Y_MIN")
        self._apply_cfg_value(cfg, ["highway", "outer_y_max_mm"], "HIGHWAY_OUTER_Y_MAX")
        self._apply_cfg_value(cfg, ["highway", "inner_x_min_mm"], "HIGHWAY_INNER_X_MIN")
        self._apply_cfg_value(cfg, ["highway", "inner_x_max_mm"], "HIGHWAY_INNER_X_MAX")
        self._apply_cfg_value(cfg, ["highway", "inner_y_min_mm"], "HIGHWAY_INNER_Y_MIN")
        self._apply_cfg_value(cfg, ["highway", "inner_y_max_mm"], "HIGHWAY_INNER_Y_MAX")
        self._apply_cfg_value(cfg, ["highway", "min_direct_dist_mm"], "HIGHWAY_MIN_DIRECT_DIST")
        self._apply_cfg_value(cfg, ["highway", "duplicate_tolerance_mm"], "HIGHWAY_DUPLICATE_TOL")
        self._apply_cfg_value(cfg, ["highway", "pass_through_radius_mm"], "HIGHWAY_PASS_THROUGH_RADIUS")
        self._apply_cfg_value(cfg, ["highway", "straight_angle_tolerance_deg"], "HIGHWAY_STRAIGHT_ANGLE_TOL", lambda v: deg2rad(float(v)))
        self._apply_cfg_value(cfg, ["highway", "overshoot_cross_track_mm"], "HIGHWAY_OVERSHOOT_CROSS_TRACK_MM")
        self._apply_cfg_value(cfg, ["highway", "behind_skip_dist_mm"], "HIGHWAY_BEHIND_SKIP_DIST_MM")
        self._apply_cfg_value(cfg, ["highway", "behind_skip_angle_deg"], "HIGHWAY_BEHIND_SKIP_ANGLE", lambda v: deg2rad(float(v)))
        self._apply_cfg_value(cfg, ["highway", "entry_selection_slack_mm"], "HIGHWAY_ENTRY_SELECTION_SLACK_MM")
        self._apply_cfg_value(cfg, ["highway", "smooth_turn_aggressive_deg"], "HIGHWAY_SMOOTH_TURN_AGGRESSIVE", lambda v: deg2rad(float(v)))
        self._apply_cfg_value(cfg, ["highway", "smooth_turn_conservative_deg"], "HIGHWAY_SMOOTH_TURN_CONSERVATIVE", lambda v: deg2rad(float(v)))
        self._apply_cfg_value(cfg, ["highway", "use_highway_planner"], "USE_HIGHWAY_PLANNER", parse_bool)
        self._apply_cfg_value(cfg, ["highway", "use_highway_for_home"], "USE_HIGHWAY_FOR_HOME", parse_bool)

        # Corner cutting — replaces sharp highway corners with two tangent points.
        self._apply_cfg_value(cfg, ["highway", "corner_cut_enabled"], "HIGHWAY_CORNER_CUT_ENABLED", parse_bool)
        self._apply_cfg_value(cfg, ["highway", "corner_cut_dist_mm"], "HIGHWAY_CORNER_CUT_DIST_MM")
        self._apply_cfg_value(cfg, ["highway", "corner_cut_min_seg_mm"], "HIGHWAY_CORNER_CUT_MIN_SEG_MM")
        self._apply_cfg_value(cfg, ["highway", "corner_cut_min_angle_deg"], "HIGHWAY_CORNER_CUT_MIN_ANGLE", lambda v: deg2rad(float(v)))
        self._apply_cfg_value(cfg, ["highway", "corner_cut_max_angle_deg"], "HIGHWAY_CORNER_CUT_MAX_ANGLE", lambda v: deg2rad(float(v)))
        self._apply_cfg_value(cfg, ["highway", "corner_arc_points"], "HIGHWAY_CORNER_ARC_POINTS", lambda v: int(v))
        self._apply_cfg_value(cfg, ["highway", "corner_pass_radius_mm"], "HIGHWAY_CORNER_PASS_RADIUS")
        self._apply_cfg_value(cfg, ["highway", "corner_pass_angle_deg"], "HIGHWAY_CORNER_PASS_ANGLE", lambda v: deg2rad(float(v)))
        self._apply_cfg_value(cfg, ["highway", "corner_safety_margin_mm"], "HIGHWAY_CORNER_SAFETY_MARGIN")

        self._apply_cfg_value(cfg, ["approach_point", "offset_mm"], "APPROACH_OFFSET")
        self._apply_cfg_value(cfg, ["approach_point", "min_dist_mm"], "APPROACH_MIN_DIST")

        self._apply_cfg_value(cfg, ["reverse", "enabled"], "ALLOW_REVERSE", parse_bool)
        self._apply_cfg_value(cfg, ["reverse", "max_dist_mm"], "REVERSE_MAX_DIST")
        self._apply_cfg_value(cfg, ["reverse", "heading_threshold_deg"], "REVERSE_HEADING_THRESHOLD_DEG")

        self._apply_cfg_value(cfg, ["zone_approach", "backup_mm"], "JENGA_APPROACH_BACKUP_MM")
        self._apply_cfg_value(cfg, ["zone_approach", "start_mm"], "JENGA_APPROACH_START_MM")
        self._apply_cfg_value(cfg, ["zone_approach", "min_align_standoff_mm"], "JENGA_APPROACH_MIN_ALIGN_STANDOFF_MM")
        self._apply_cfg_value(cfg, ["zone_approach", "prefer_forward_start"], "JENGA_APPROACH_PREFER_FORWARD_START", parse_bool)
        self._apply_cfg_value(cfg, ["zone_approach", "forbidden_margin_mm"], "JENGA_APPROACH_FORBIDDEN_MARGIN")
        self._apply_cfg_value(cfg, ["zone_approach", "straight_yaw_tolerance_deg"], "JENGA_STRAIGHT_YAW_TOL", lambda v: deg2rad(float(v)))
        self._apply_cfg_value(cfg, ["zone_approach", "correction_reverse_enabled"], "JENGA_CORRECTION_REVERSE_ENABLED", parse_bool)

        self._apply_cfg_value(cfg, ["pantry_approach", "backup_mm"], "PANTRY_APPROACH_BACKUP_MM")
        self._apply_cfg_value(cfg, ["pantry_approach", "start_mm"], "PANTRY_APPROACH_START_MM")
        self._apply_cfg_value(cfg, ["pantry_approach", "min_align_standoff_mm"], "PANTRY_APPROACH_MIN_ALIGN_STANDOFF_MM")
        self._apply_cfg_value(cfg, ["pantry_approach", "prefer_forward_start"], "PANTRY_APPROACH_PREFER_FORWARD_START", parse_bool)
        self._apply_cfg_value(cfg, ["pantry_approach", "correction_reverse_enabled"], "PANTRY_CORRECTION_REVERSE_ENABLED", parse_bool)

        self._apply_cfg_value(cfg, ["home", "staging_offset_mm"], "HOME_STAGING_OFFSET")
        self._apply_cfg_value(cfg, ["home", "x_mm"], "home_x")
        self._apply_cfg_value(cfg, ["home", "y_mm"], "home_y")
        self._apply_cfg_value(cfg, ["home", "yaw_deg"], "home_yaw_deg")

        # Must happen after map points/yaws, highway geometry, and home are loaded.
        self._apply_team_mirror_to_loaded_config()

        self._apply_cfg_value(cfg, ["boundary_slowdown", "slowdown_dist_mm"], "BOUNDARY_SLOWDOWN_DIST")
        self._apply_cfg_value(cfg, ["boundary_slowdown", "min_speed_fraction"], "BOUNDARY_MIN_SPEED_FRAC")

        self._apply_cfg_value(cfg, ["dustpan", "up_angle_deg"], "DUSTPAN_UP_ANGLE", lambda v: deg2rad(float(v)))
        self._apply_cfg_value(cfg, ["dustpan", "down_angle_deg"], "DUSTPAN_DOWN_ANGLE", lambda v: deg2rad(float(v)))
        self._apply_cfg_value(cfg, ["dustpan", "lower_settle_s"], "DUSTPAN_LOWER_SETTLE_S")

        self._apply_cfg_value(cfg, ["carwash", "arm_up_angle_deg"], "CARWASH_ARM_UP", lambda v: deg2rad(float(v)))
        self._apply_cfg_value(cfg, ["carwash", "arm_down_angle_deg"], "CARWASH_ARM_DOWN", lambda v: deg2rad(float(v)))
        self._apply_cfg_value(cfg, ["carwash", "roller_stop_rad_s"], "CARWASH_ROLLER_STOP")

        self._apply_cfg_value(cfg, ["mission", "carwash_pull_time_s"], "MISSION_CARWASH_PULL_TIME")
        self._apply_cfg_value(cfg, ["mission", "carwash_push_time_s"], "MISSION_CARWASH_PUSH_TIME")
        self._apply_cfg_value(cfg, ["mission", "settle_time_s"], "MISSION_SETTLE_TIME")
        self._apply_cfg_value(cfg, ["mission", "backup_dist_mm"], "MISSION_BACKUP_DIST_MM")

        # Apply the master base-speed profile last so base_speeds wins over
        # old fixed speed fields if both exist in the YAML.
        self._apply_base_speed_profile(cfg)

        self.get_logger().info(
            f"Loaded control tuning from {actual_path} | "
            f"BASE_LINEAR={self.BASE_LINEAR_MM_S:.1f} BASE_YAW={self.BASE_YAW_RAD_S:.3f} "
            f"HIGHWAY={self.HIGHWAY_SPEED_MM_S:.1f} ENTRY={self.HIGHWAY_ENTRY_EXIT_SPEED_MM_S:.1f} "
            f"TURN={self.HIGHWAY_TURN_SPEED_MM_S:.1f} CORNER={self.HIGHWAY_CORNER_SPEED_MM_S:.1f} "
            f"K_RHO={self.K_rho:.3f} "
            f"FIELD=({self.FIELD_X_MIN:.0f},{self.FIELD_Y_MIN:.0f})-({self.FIELD_X_MAX:.0f},{self.FIELD_Y_MAX:.0f}) "
            f"ZONES={len(self.JENGA_ZONES)} PANTRIES={len(self.PANTRY_LOCATIONS)}"
        )

    # ══════════════════════════════════════════════════════════
    #  Gazebo feeds
    # ══════════════════════════════════════════════════════════
    def _drive_pose_cb(self, msg: Pose2D):
        """Use the configured localization pose source for the drive controller."""
        self.x_mm = msg.x * 1000.0
        self.y_mm = msg.y * 1000.0
        self.theta_rad = msg.theta
        self.drive_pose_time_s = time.monotonic()
        first_pose = not self.odom_valid
        self.odom_valid = True

        # A GO command can arrive very early, before this first pose callback.
        # The previous version silently bypassed the highway planner in that case.
        # Start the queued command now, using the configured localization pose.
        if first_pose and self.pending_goal is not None:
            gx, gy, gyaw_deg, ref, label = self.pending_goal
            self.pending_goal = None
            self._pub_str(self.event_pub, "POSE_READY_STARTING_PENDING_GOAL")
            self._set_new_goal(gx, gy, gyaw_deg, ref, label)

    def _drive_pose_fresh(self) -> bool:
        if not self.odom_valid or self.drive_pose_time_s is None:
            return False
        if self.DRIVE_POSE_TIMEOUT_S <= 0.0:
            return True
        return (time.monotonic() - self.drive_pose_time_s) <= self.DRIVE_POSE_TIMEOUT_S

    def _enemy_pose_cb(self, msg: Pose2D):
        """Opponent pose from the overhead camera, in world metres."""
        self.enemy_x_mm = float(msg.x) * 1000.0
        self.enemy_y_mm = float(msg.y) * 1000.0
        self.enemy_theta_rad = wrap(float(msg.theta))
        self.enemy_pose_valid = True
        self.enemy_pose_time_s = time.monotonic()

    def _odom_cb(self, msg: Odometry):
        """Raw odom from Gazebo — used for telemetry PID estimate only."""
        self.odom_x_m = msg.pose.pose.position.x
        self.odom_y_m = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.odom_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        self.imu_yaw_deg = rad2deg(quat_to_yaw(q.x, q.y, q.z, q.w))
        self.imu_gz = msg.angular_velocity.z

    # ══════════════════════════════════════════════════════════
    #  Reference-point offset (mm forward from wheel axis)
    # ══════════════════════════════════════════════════════════
    def _offset_for_ref(self, ref: int) -> float:
        if ref == RefPoint.CENTER:
            return self.OFFSET_CENTER
        if ref == RefPoint.DUSTPAN:
            return self.OFFSET_DUSTPAN
        return self.OFFSET_AXIS



    def _target_with_offset(self, base_x: float, base_y: float, offset_map, item_id: int, step=None):
        """Apply configurable world/field XY offsets to a zone/pantry target.

        Offsets are in millimetres in FIELD coordinates, not robot-relative:
          +x = right side of board
          +y = top side of board

        YAML offset maps apply every time the ID is used.
        Strategy steps can add one-off offsets using:
          offset_mm: [x, y]
          target_offset_mm: [x, y]
          x_offset_mm / y_offset_mm
          offset_x_mm / offset_y_mm
        """
        dx, dy = (0.0, 0.0)

        if isinstance(offset_map, dict):
            dx, dy = offset_map.get(int(item_id), (0.0, 0.0))

        dx = float(dx)
        dy = float(dy)

        if isinstance(step, dict):
            for key in ("target_offset_mm", "offset_mm"):
                if key in step:
                    sx, sy = self._coerce_cfg_point(step[key])
                    dx += float(sx)
                    dy += float(sy)

            dx += float(step.get("x_offset_mm", step.get("offset_x_mm", 0.0)))
            dy += float(step.get("y_offset_mm", step.get("offset_y_mm", 0.0)))

        return base_x + dx, base_y + dy, dx, dy

    def _ref_name(self, ref: int) -> str:
        return {RefPoint.AXIS: "AXIS", RefPoint.CENTER: "CENTER",
                RefPoint.DUSTPAN: "DUSTPAN"}.get(ref, "?")


    # ══════════════════════════════════════════════════════════
    #  Team mirroring helpers
    # ══════════════════════════════════════════════════════════
    def _team_is_blue(self) -> bool:
        return str(getattr(self, "TEAM_COLOR", "yellow")).strip().lower() == "blue"

    def _mirror_point_for_team(self, x_mm: float, y_mm: float):
        if not self._team_is_blue():
            return float(x_mm), float(y_mm)
        mx = float(getattr(self, "TEAM_MIRROR_LINE_X_MM", 1500.0))
        return 2.0 * mx - float(x_mm), float(y_mm)

    def _mirror_yaw_deg_for_team(self, yaw_deg: float) -> float:
        if not self._team_is_blue():
            return float(yaw_deg)
        return rad2deg(wrap(math.pi - deg2rad(float(yaw_deg))))

    def _mirror_point_map_for_team(self, point_map):
        if not self._team_is_blue() or not isinstance(point_map, dict):
            return point_map
        return {int(k): self._mirror_point_for_team(x, y) for k, (x, y) in point_map.items()}

    def _mirror_offset_map_for_team(self, offset_map):
        # Offsets are vectors, not absolute positions. Mirroring across a vertical
        # x-line flips the x component but keeps y unchanged.
        if not self._team_is_blue() or not isinstance(offset_map, dict):
            return offset_map
        return {int(k): (-float(dx), float(dy)) for k, (dx, dy) in offset_map.items()}

    def _mirror_yaw_map_for_team(self, yaw_map):
        if not self._team_is_blue() or not isinstance(yaw_map, dict):
            return yaw_map
        return {int(k): self._mirror_yaw_deg_for_team(yaw) for k, yaw in yaw_map.items()}

    def _mirror_x_bounds_for_team(self, x_min: float, x_max: float):
        if not self._team_is_blue():
            return float(x_min), float(x_max)
        mx = float(getattr(self, "TEAM_MIRROR_LINE_X_MM", 1500.0))
        a = 2.0 * mx - float(x_max)
        b = 2.0 * mx - float(x_min)
        return min(a, b), max(a, b)

    def _apply_team_mirror_to_loaded_config(self):
        team = str(getattr(self, "TEAM_COLOR", "yellow")).strip().lower()
        if team not in ("yellow", "blue"):
            self.get_logger().warn(f"Unknown team color {team!r}; using yellow/no mirror")
            self.TEAM_COLOR = "yellow"
            return

        if not self._team_is_blue():
            self.get_logger().info("TEAM_MODE yellow: using targets exactly as written in YAML")
            return

        self.JENGA_ZONES = self._mirror_point_map_for_team(self.JENGA_ZONES)
        self.JENGA_ZONE_TARGET_OFFSETS_MM = self._mirror_offset_map_for_team(self.JENGA_ZONE_TARGET_OFFSETS_MM)
        self.JENGA_ZONE_YAWS_DEG = self._mirror_yaw_map_for_team(self.JENGA_ZONE_YAWS_DEG)
        self.JENGA_ZONE_FORCED_HIGHWAY_EXIT = self._mirror_point_map_for_team(self.JENGA_ZONE_FORCED_HIGHWAY_EXIT)

        self.PANTRY_LOCATIONS = self._mirror_point_map_for_team(self.PANTRY_LOCATIONS)
        self.PANTRY_TARGET_OFFSETS_MM = self._mirror_offset_map_for_team(self.PANTRY_TARGET_OFFSETS_MM)
        self.PANTRY_APPROACH_YAWS_DEG = self._mirror_yaw_map_for_team(self.PANTRY_APPROACH_YAWS_DEG)
        self.PANTRY_FORCED_HIGHWAY_EXIT = self._mirror_point_map_for_team(self.PANTRY_FORCED_HIGHWAY_EXIT)

        self.home_x, self.home_y = self._mirror_point_for_team(self.home_x, self.home_y)
        self.home_yaw_deg = self._mirror_yaw_deg_for_team(self.home_yaw_deg)

        if isinstance(self.HIGHWAY_X, list):
            self.HIGHWAY_X = sorted([self._mirror_point_for_team(x, 0.0)[0] for x in self.HIGHWAY_X])

        self.HIGHWAY_OUTER_X_MIN, self.HIGHWAY_OUTER_X_MAX = self._mirror_x_bounds_for_team(
            self.HIGHWAY_OUTER_X_MIN, self.HIGHWAY_OUTER_X_MAX
        )
        self.HIGHWAY_INNER_X_MIN, self.HIGHWAY_INNER_X_MAX = self._mirror_x_bounds_for_team(
            self.HIGHWAY_INNER_X_MIN, self.HIGHWAY_INNER_X_MAX
        )

        self.get_logger().info(
            f"TEAM_MODE blue: mirrored autonomous targets across x={self.TEAM_MIRROR_LINE_X_MM:.1f} mm"
        )

    # ══════════════════════════════════════════════════════════
    #  Boundary helpers
    # ══════════════════════════════════════════════════════════
    def _clamp_goal_to_bounds(self, ax, ay, heading, clearance=0.0):
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        offsets_fwd = [self.BOT_FRONT, self.BOT_FRONT, -self.BOT_REAR, -self.BOT_REAR]
        offsets_lat = [self.BOT_HALF_W, -self.BOT_HALF_W, self.BOT_HALF_W, -self.BOT_HALF_W]

        px_pos = px_neg = py_pos = py_neg = 0.0
        for i in range(4):
            cx = ax + offsets_fwd[i] * cos_h - offsets_lat[i] * sin_h
            cy = ay + offsets_fwd[i] * sin_h + offsets_lat[i] * cos_h
            if cx > self.FIELD_X_MAX - clearance:
                px_pos = max(px_pos, cx - (self.FIELD_X_MAX - clearance))
            if cx < self.FIELD_X_MIN + clearance:
                px_neg = max(px_neg, (self.FIELD_X_MIN + clearance) - cx)
            if cy > self.FIELD_Y_MAX - clearance:
                py_pos = max(py_pos, cy - (self.FIELD_Y_MAX - clearance))
            if cy < self.FIELD_Y_MIN + clearance:
                py_neg = max(py_neg, (self.FIELD_Y_MIN + clearance) - cy)

        ax += px_neg - px_pos
        ay += py_neg - py_pos
        clamped = (px_pos + px_neg + py_pos + py_neg) > 0
        return ax, ay, clamped

    def _boundary_speed_fraction(self) -> float:
        cl_x_min = self.x_mm - self.FIELD_X_MIN - self.BOT_FRONT
        cl_x_max = self.FIELD_X_MAX - self.x_mm - self.BOT_FRONT
        cl_y_min = self.y_mm - self.FIELD_Y_MIN - self.BOT_HALF_W
        cl_y_max = self.FIELD_Y_MAX - self.y_mm - self.BOT_HALF_W
        clearance = max(0.0, min(cl_x_min, cl_x_max, cl_y_min, cl_y_max))
        if clearance >= self.BOUNDARY_SLOWDOWN_DIST:
            return 1.0
        frac = self.BOUNDARY_MIN_SPEED_FRAC + \
               (1.0 - self.BOUNDARY_MIN_SPEED_FRAC) * (clearance / self.BOUNDARY_SLOWDOWN_DIST)
        return clampf(frac, self.BOUNDARY_MIN_SPEED_FRAC, 1.0)

    # ══════════════════════════════════════════════════════════
    #  Preferred highway planner (line path with zone guardrails)
    # ══════════════════════════════════════════════════════════

    def _highway_point_candidates(self, x, y):
        """Candidate projections from an arbitrary point onto the highway grid."""
        x_lo, x_hi = min(self.HIGHWAY_X), max(self.HIGHWAY_X)
        y_lo, y_hi = min(self.HIGHWAY_Y), max(self.HIGHWAY_Y)

        raw = []
        for hx in self.HIGHWAY_X:
            raw.append((hx, clampf(y, y_lo, y_hi)))
        for hy in self.HIGHWAY_Y:
            raw.append((clampf(x, x_lo, x_hi), hy))

        # Corners are useful when the robot starts outside both highway spans.
        for hx in self.HIGHWAY_X:
            for hy in self.HIGHWAY_Y:
                raw.append((hx, hy))

        return self._dedupe_points(raw)

    def _nearest_highway_point(self, x, y):
        """Nearest point on the preferred highway line grid."""
        candidates = self._highway_point_candidates(x, y)
        hx, hy = min(candidates, key=lambda p: math.hypot(x - p[0], y - p[1]))
        return hx, hy

    def _same_highway_segment(self, a, b) -> bool:
        """True if two points lie on the same straight highway segment."""
        ax, ay = a
        bx, by = b
        x_lo, x_hi = min(self.HIGHWAY_X), max(self.HIGHWAY_X)
        y_lo, y_hi = min(self.HIGHWAY_Y), max(self.HIGHWAY_Y)
        eps = self.HIGHWAY_NODE_EPS

        if abs(ax - bx) < eps and any(abs(ax - hx) < eps for hx in self.HIGHWAY_X):
            return (y_lo - eps <= ay <= y_hi + eps and
                    y_lo - eps <= by <= y_hi + eps)

        if abs(ay - by) < eps and any(abs(ay - hy) < eps for hy in self.HIGHWAY_Y):
            return (x_lo - eps <= ax <= x_hi + eps and
                    x_lo - eps <= bx <= x_hi + eps)

        return False

    def _dedupe_points(self, pts):
        out = []
        for p in pts:
            if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > self.HIGHWAY_DUPLICATE_TOL:
                out.append(p)
        return out

    def _shortest_highway_route(self, entry, exit_pt):
        """Shortest route along the preferred rectangular highway line grid."""
        x1, x2 = min(self.HIGHWAY_X), max(self.HIGHWAY_X)
        y1, y2 = min(self.HIGHWAY_Y), max(self.HIGHWAY_Y)
        corners = [(x1, y1), (x1, y2), (x2, y1), (x2, y2)]
        raw_nodes = [entry, exit_pt] + corners

        nodes = []
        for p in raw_nodes:
            if not any(math.hypot(p[0] - q[0], p[1] - q[1]) <= self.HIGHWAY_DUPLICATE_TOL for q in nodes):
                nodes.append(p)

        start_idx = min(range(len(nodes)), key=lambda i: math.hypot(nodes[i][0] - entry[0], nodes[i][1] - entry[1]))
        goal_idx = min(range(len(nodes)), key=lambda i: math.hypot(nodes[i][0] - exit_pt[0], nodes[i][1] - exit_pt[1]))

        graph = {i: [] for i in range(len(nodes))}
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                if self._same_highway_segment(nodes[i], nodes[j]):
                    d = math.hypot(nodes[i][0] - nodes[j][0], nodes[i][1] - nodes[j][1])
                    graph[i].append((j, d))
                    graph[j].append((i, d))

        dist = [float("inf")] * len(nodes)
        prev = [-1] * len(nodes)
        dist[start_idx] = 0.0
        heap = [(0.0, start_idx)]

        while heap:
            d, i = heapq.heappop(heap)
            if d > dist[i]:
                continue
            if i == goal_idx:
                break
            for j, w in graph[i]:
                nd = d + w
                if nd < dist[j]:
                    dist[j] = nd
                    prev[j] = i
                    heapq.heappush(heap, (nd, j))

        if dist[goal_idx] == float("inf"):
            return [entry, exit_pt]

        route = []
        i = goal_idx
        while i != -1:
            route.append(nodes[i])
            i = prev[i]
        route.reverse()
        return self._dedupe_points(route)

    def _highway_route_length(self, route):
        if not route or len(route) < 2:
            return 0.0
        return sum(
            math.hypot(route[i][0] - route[i - 1][0], route[i][1] - route[i - 1][1])
            for i in range(1, len(route))
        )

    def _select_highway_entry(self, final_x, final_y):
        """Choose a nearby highway entry without allowing a long diagonal grab.

        Candidate entries are first restricted to points within
        HIGHWAY_ENTRY_SELECTION_SLACK_MM of the nearest projection. Within that
        local set, choose the one with the best total path cost.
        """
        candidates = self._dedupe_points(self._highway_point_candidates(self.x_mm, self.y_mm))
        if not candidates:
            return self._nearest_highway_point(self.x_mm, self.y_mm)

        exit_pt = self._nearest_highway_point(final_x, final_y)
        nearest_entry_dist = min(math.hypot(c[0] - self.x_mm, c[1] - self.y_mm) for c in candidates)
        slack = max(0.0, float(getattr(self, "HIGHWAY_ENTRY_SELECTION_SLACK_MM", 0.0)))
        allowed = [
            c for c in candidates
            if math.hypot(c[0] - self.x_mm, c[1] - self.y_mm) <= nearest_entry_dist + slack
        ] or candidates

        def cost(c):
            entry_dist = math.hypot(c[0] - self.x_mm, c[1] - self.y_mm)
            route = self._shortest_highway_route(c, exit_pt)
            return entry_dist + self._highway_route_length(route)

        return min(allowed, key=cost)

    def _build_highway_waypoints(self, final_x, final_y, final_yaw):
        """
        Build waypoint list in axis coordinates.

        The planner follows the preferred highway lines. Sharp highway corners
        are replaced by a small circular arc that is checked against the hollow
        highway corridor before it is accepted. Unlike the previous diagonal
        corner cut, this does not cross the inner forbidden rectangle.
        """
        self._pending_smooth_indices = set()

        direct_dist = math.hypot(final_x - self.x_mm, final_y - self.y_mm)
        if direct_dist < self.HIGHWAY_MIN_DIRECT_DIST:
            return [(final_x, final_y, final_yaw)]

        entry = self._select_highway_entry(final_x, final_y)
        exit_pt = self._nearest_highway_point(final_x, final_y)
        route = self._shortest_highway_route(entry, exit_pt)

        points = []
        if math.hypot(entry[0] - self.x_mm, entry[1] - self.y_mm) > self.HIGHWAY_DUPLICATE_TOL:
            points.append(entry)

        points.extend(route[1:] if points and route else route)

        if not points or math.hypot(final_x - points[-1][0], final_y - points[-1][1]) > self.HIGHWAY_DUPLICATE_TOL:
            points.append((final_x, final_y))

        points = self._dedupe_points(points)
        if not points:
            return [(final_x, final_y, final_yaw)]

        if self.HIGHWAY_CORNER_CUT_ENABLED and len(points) >= 3:
            points, smooth_indices = self._smooth_safe_highway_corners(points)
            self._pending_smooth_indices = smooth_indices

        waypoints = []
        for i, p in enumerate(points):
            if i < len(points) - 1:
                nx, ny = points[i + 1]
                yaw = math.atan2(ny - p[1], nx - p[0])
            else:
                yaw = final_yaw
            waypoints.append((p[0], p[1], wrap(yaw)))

        return waypoints

    def _point_in_inner_forbidden(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (
            self.HIGHWAY_INNER_X_MIN + margin < x < self.HIGHWAY_INNER_X_MAX - margin
            and self.HIGHWAY_INNER_Y_MIN + margin < y < self.HIGHWAY_INNER_Y_MAX - margin
        )

    def _point_in_highway_corridor(self, x: float, y: float, margin: float = 0.0) -> bool:
        """True if a path-center point is inside the hollow highway corridor."""
        inside_outer = (
            self.HIGHWAY_OUTER_X_MIN + margin <= x <= self.HIGHWAY_OUTER_X_MAX - margin
            and self.HIGHWAY_OUTER_Y_MIN + margin <= y <= self.HIGHWAY_OUTER_Y_MAX - margin
        )
        if not inside_outer:
            return False
        return not self._point_in_inner_forbidden(x, y, margin)

    def _dedupe_tagged_points(self, tagged_points):
        """Dedupe while preserving whether surviving points are smooth-corner points."""
        out = []
        for x, y, is_smooth in tagged_points:
            if out and math.hypot(x - out[-1][0], y - out[-1][1]) <= self.HIGHWAY_DUPLICATE_TOL:
                px, py, prev_smooth = out[-1]
                out[-1] = (px, py, bool(prev_smooth or is_smooth))
            else:
                out.append((x, y, bool(is_smooth)))
        return out
    
    

    def _smooth_safe_highway_corners(self, points):
        """
        Replace each interior corner with a sampled circular arc.

        The important difference from the old corner_cut is that the generated
        samples are checked against the hollow highway corridor. If the arc
        would enter the inner forbidden rectangle, the original sharp corner is
        kept instead of creating an unsafe shortcut.
        """
        cut_dist = max(0.0, float(self.HIGHWAY_CORNER_CUT_DIST_MM))
        min_seg = max(0.0, float(self.HIGHWAY_CORNER_CUT_MIN_SEG_MM))
        min_turn = max(0.0, float(self.HIGHWAY_CORNER_CUT_MIN_ANGLE))
        max_turn = clampf(float(self.HIGHWAY_CORNER_CUT_MAX_ANGLE), min_turn, PI - deg2rad(1.0))
        arc_points = max(3, int(self.HIGHWAY_CORNER_ARC_POINTS))
        safety_margin = max(0.0, float(self.HIGHWAY_CORNER_SAFETY_MARGIN))

        tagged = [(points[0][0], points[0][1], False)]

        for i in range(1, len(points) - 1):
            px, py = points[i - 1]
            cx, cy = points[i]
            nx, ny = points[i + 1]

            in_dx, in_dy = cx - px, cy - py
            out_dx, out_dy = nx - cx, ny - cy
            in_len = math.hypot(in_dx, in_dy)
            out_len = math.hypot(out_dx, out_dy)

            if in_len < min_seg or out_len < min_seg:
                tagged.append((cx, cy, False))
                continue

            in_ux, in_uy = in_dx / in_len, in_dy / in_len
            out_ux, out_uy = out_dx / out_len, out_dy / out_len
            dot = clampf(in_ux * out_ux + in_uy * out_uy, -1.0, 1.0)
            turn_angle = math.acos(dot)
            cross = in_ux * out_uy - in_uy * out_ux

            if turn_angle < min_turn or turn_angle > max_turn or abs(cross) < 1e-6:
                tagged.append((cx, cy, False))
                continue

            tangent = min(cut_dist, in_len * 0.45, out_len * 0.45)
            if tangent < self.HIGHWAY_DUPLICATE_TOL * 2.0:
                tagged.append((cx, cy, False))
                continue

            radius = tangent / max(1e-6, math.tan(turn_angle * 0.5))

            cut_in_x = cx - in_ux * tangent
            cut_in_y = cy - in_uy * tangent
            cut_out_x = cx + out_ux * tangent
            cut_out_y = cy + out_uy * tangent

            # Center is on the inside of the turn, perpendicular to the incoming
            # segment. For an east→north turn this gives center=(cut_in.x, cut_in.y+r),
            # producing the safe quarter arc around the corridor corner instead of
            # the unsafe diagonal through the forbidden rectangle.
            if cross > 0.0:  # left turn
                norm_x, norm_y = -in_uy, in_ux
            else:            # right turn
                norm_x, norm_y = in_uy, -in_ux
            center_x = cut_in_x + norm_x * radius
            center_y = cut_in_y + norm_y * radius

            start_ang = math.atan2(cut_in_y - center_y, cut_in_x - center_x)
            end_ang = math.atan2(cut_out_y - center_y, cut_out_x - center_x)
            if cross > 0.0:
                while end_ang <= start_ang:
                    end_ang += 2.0 * PI
            else:
                while end_ang >= start_ang:
                    end_ang -= 2.0 * PI

            arc = []
            for j in range(arc_points):
                t = j / max(1, arc_points - 1)
                a = start_ang + (end_ang - start_ang) * t
                ax = center_x + radius * math.cos(a)
                ay = center_y + radius * math.sin(a)
                arc.append((ax, ay))

            # Validate densely, not only at the sparse output samples.
            safe = True
            dense_count = max(12, arc_points * 4)
            for j in range(dense_count + 1):
                t = j / dense_count
                a = start_ang + (end_ang - start_ang) * t
                ax = center_x + radius * math.cos(a)
                ay = center_y + radius * math.sin(a)
                if not self._point_in_highway_corridor(ax, ay, safety_margin):
                    safe = False
                    break

            if not safe:
                self._pub_str(
                    self.event_pub,
                    f"HIGHWAY_CORNER_SMOOTH_SKIPPED unsafe {cx:.1f} {cy:.1f}"
                )
                tagged.append((cx, cy, False))
                continue

            for ax, ay in arc:
                tagged.append((ax, ay, True))

        tagged.append((points[-1][0], points[-1][1], False))
        tagged = self._dedupe_tagged_points(tagged)

        smooth_indices = {idx for idx, (_, _, is_smooth) in enumerate(tagged) if is_smooth}
        out_points = [(x, y) for x, y, _ in tagged]
        return out_points, smooth_indices

    def _is_smooth_corner_waypoint(self, index=None) -> bool:
        if index is None:
            index = self.path_index
        return bool(getattr(self, "path_smooth_indices", set()) and index in self.path_smooth_indices)

    def _is_fixed_yaw_path_index(self, index=None) -> bool:
        if index is None:
            index = self.path_index
        return bool(getattr(self, "path_fixed_yaw_indices", set()) and index in self.path_fixed_yaw_indices)

    def _is_force_reverse_path_index(self, index=None) -> bool:
        if index is None:
            index = self.path_index
        return bool(getattr(self, "path_force_reverse_indices", set()) and index in self.path_force_reverse_indices)

    def _start_next_path_waypoint(self, force_align=True):
        if not self.path_active or self.path_index >= len(self.path_waypoints):
            return False

        x, y, yaw = self.path_waypoints[self.path_index]
        is_final = self.path_index == len(self.path_waypoints) - 1
        label = self.path_final_label if is_final else f"{self.path_final_label}_WP{self.path_index + 1}"

        # Highway waypoints are already axis points. Do not generate an extra
        # approach point; the waypoint list itself is the planned path.
        self._activate_axis_goal(x, y, yaw, RefPoint.AXIS, label, use_approach=False, send_ack=False)

        self.align_for_approach_override_yaw = None

        # Fixed-yaw zone/pantry motion special case. Only waypoints explicitly
        # marked in path_fixed_yaw_indices use this behavior. That lets us route
        # to a safe pre-approach point normally, align there, and then drive the
        # final pickup/placement push straight without also forcing the previous
        # waypoint to reverse/straight-drive.
        if self._is_fixed_yaw_path_index(self.path_index):
            self.align_for_approach_override_yaw = yaw
            force_align = True
            reverse_tag = " reverse=1" if self._is_force_reverse_path_index(self.path_index) else ""
            self._pub_str(
                self.event_pub,
                f"FIXED_YAW_PREALIGN {rad2deg(yaw):.1f}{reverse_tag}"
            )
        else:
            # ── Two-tier smooth highway driving ──
            # Intermediate highway waypoints use two thresholds:
            #
            #   heading_err < aggressive (default 40°):
            #     Full speed, no pre-alignment, no final yaw stop.
            #     The bot curves smoothly through at cruise speed.
            #
            #   heading_err < conservative (default 90°):
            #     No pre-alignment stop, but the bot WILL stop at the
            #     waypoint for a quick yaw correction before advancing.
            #     Speed is reduced to highway_turn speed for stability.
            #
            #   heading_err >= conservative:
            #     Full stop-rotate-drive (original behavior).
            #
            # The first waypoint (path_index == 0) always uses full alignment
            # since the bot may be stationary facing any direction.
            self._smooth_turn_tier = None  # reset
            if not is_final and self.path_index > 0:
                if self._is_smooth_corner_waypoint(self.path_index):
                    # Safe-corner arc samples are already corridor-checked and
                    # speed-limited. Do not stop/yaw-settle on them.
                    force_align = False
                    self._smooth_turn_tier = "corner"
                else:
                    heading_to_wp = math.atan2(y - self.y_mm, x - self.x_mm)
                    heading_err = abs(wrap(heading_to_wp - self.theta_rad))
                    if heading_err < self.HIGHWAY_SMOOTH_TURN_AGGRESSIVE:
                        force_align = False
                        self._smooth_turn_tier = "aggressive"
                    elif heading_err < self.HIGHWAY_SMOOTH_TURN_CONSERVATIVE:
                        force_align = False
                        self._smooth_turn_tier = "conservative"

        # Force a stop-and-rotate only when needed (large heading error or
        # precision final approach). Otherwise go straight to DRIVE_TO_POINT
        # for smooth curving motion.
        if force_align and math.hypot(x - self.x_mm, y - self.y_mm) > self.POS_TOL_MM:
            self.nav_state = NavState.ALIGN_FOR_APPROACH
            self._pub_str(self.event_pub, f"STATE {self.run_mode} {self.nav_state} 1 0")
        tier_str = f" tier={self._smooth_turn_tier}" if getattr(self, '_smooth_turn_tier', None) else ""
        self._pub_str(
            self.event_pub,
            f"PATH_WP {self.path_index + 1}/{len(self.path_waypoints)} "
            f"{x:.1f} {y:.1f} {rad2deg(yaw):.1f}{tier_str}"
        )
        return True

    def _maybe_pass_straight_through_waypoint(self) -> bool:
        """
        If the current waypoint is just a straight-through point, switch to the
        next waypoint before the robot slows to a stop at the node.
        """
        if not self.path_active:
            return False

        if self.path_index >= len(self.path_waypoints) - 1:
            return False

        cx, cy, _ = self.path_waypoints[self.path_index]
        nx, ny, _ = self.path_waypoints[self.path_index + 1]

        is_smooth_corner = self._is_smooth_corner_waypoint(self.path_index)
        pass_radius = self.HIGHWAY_CORNER_PASS_RADIUS if is_smooth_corner else self.HIGHWAY_PASS_THROUGH_RADIUS

        dist_to_current = math.hypot(cx - self.x_mm, cy - self.y_mm)
        if dist_to_current > pass_radius:
            return False

        if self.path_index == 0:
            px, py = self.path_start_x, self.path_start_y
        else:
            px, py, _ = self.path_waypoints[self.path_index - 1]

        in_len = math.hypot(cx - px, cy - py)
        out_len = math.hypot(nx - cx, ny - cy)
        if in_len < self.HIGHWAY_DUPLICATE_TOL or out_len < self.HIGHWAY_DUPLICATE_TOL:
            return False

        in_heading = math.atan2(cy - py, cx - px)
        out_heading = math.atan2(ny - cy, nx - cx)
        turn_angle = abs(wrap(out_heading - in_heading))

        angle_tol = self.HIGHWAY_CORNER_PASS_ANGLE if is_smooth_corner else self.HIGHWAY_STRAIGHT_ANGLE_TOL
        if turn_angle > angle_tol:
            return False

        self._pub_str(
            self.event_pub,
            f"PATH_WP_PASSTHROUGH {self.path_index + 1}/{len(self.path_waypoints)} "
            f"{cx:.1f} {cy:.1f}"
        )

        self.path_index += 1

        # Do NOT force a new alignment here. It is the same straight segment,
        # so forcing ALIGN_FOR_APPROACH is exactly what caused the slowdown.
        self._start_next_path_waypoint(force_align=False)
        return True

    def _maybe_advance_overshot_waypoint(self) -> bool:
        """
        If an intermediate highway waypoint has been passed, advance to the
        next waypoint instead of trying to turn back toward it.

        This prevents the high-speed overshoot case where the waypoint ends up
        behind the robot, causing the non-reversing point controller to spin.
        """
        if not self.path_active:
            return False
        if self.path_index >= len(self.path_waypoints) - 1:
            return False
        if self._is_zone_straight_waypoint():
            return False

        cx, cy, _ = self.path_waypoints[self.path_index]
        if self.path_index == 0:
            px, py = self.path_start_x, self.path_start_y
        else:
            px, py, _ = self.path_waypoints[self.path_index - 1]

        seg_x = cx - px
        seg_y = cy - py
        seg_len = math.hypot(seg_x, seg_y)
        if seg_len < self.HIGHWAY_DUPLICATE_TOL:
            return False

        rob_x = self.x_mm - px
        rob_y = self.y_mm - py
        along = (rob_x * seg_x + rob_y * seg_y) / seg_len
        cross_track = abs(rob_x * seg_y - rob_y * seg_x) / seg_len
        dist_to_wp = math.hypot(cx - self.x_mm, cy - self.y_mm)
        heading_to_wp = math.atan2(cy - self.y_mm, cx - self.x_mm)
        heading_err = abs(wrap(heading_to_wp - self.theta_rad))

        passed_plane = (
            along >= seg_len - self.POS_TOL_MM
            and cross_track <= self.HIGHWAY_OVERSHOOT_CROSS_TRACK_MM
        )
        # Safer behind-skip:
        # Do NOT skip just because the waypoint is behind us. That was too
        # careless near obstacles. Only allow it if we are also close to the
        # waypoint, close to the intended segment, and almost at the end of
        # that segment.
        wp_behind_nearby = (
            dist_to_wp <= self.HIGHWAY_BEHIND_SKIP_DIST_MM
            and heading_err >= self.HIGHWAY_BEHIND_SKIP_ANGLE
            and cross_track <= self.HIGHWAY_OVERSHOOT_CROSS_TRACK_MM
            and along >= seg_len - self.HIGHWAY_BEHIND_SKIP_DIST_MM
        )

        if not (passed_plane or wp_behind_nearby):
            return False

        self._pub_str(
            self.event_pub,
            f"PATH_WP_OVERSHOT {self.path_index + 1}/{len(self.path_waypoints)} "
            f"dist={dist_to_wp:.1f} cross={cross_track:.1f} along={along:.1f}"
        )
        self.path_index += 1
        self.alpha_integral = 0.0

        # For generated safe-corner arc points, keep rolling; for normal highway
        # nodes, remain conservative after overshoot.
        self._start_next_path_waypoint(force_align=not self._is_smooth_corner_waypoint(self.path_index))
        return True


    # ══════════════════════════════════════════════════════════
    #  Drive helpers (faithful port)
    # ══════════════════════════════════════════════════════════
    def _current_highway_speed_limit(self) -> float:
        """Return speed cap for the current intermediate path waypoint."""
        if not self.path_active or self.path_index >= len(self.path_waypoints):
            return self.MAX_LINEAR_MM_S

        speed = self.HIGHWAY_SPEED_MM_S

        # Entry/exit segments are usually near obstacles or field edges.
        if self.path_index == 0 or self.path_index >= len(self.path_waypoints) - 2:
            speed = min(speed, self.HIGHWAY_ENTRY_EXIT_SPEED_MM_S)

        # Safe-corner arc points get their own speed cap. This keeps smooth
        # cornering from becoming the old aggressive overshoot problem.
        if (self._is_smooth_corner_waypoint(self.path_index)
                or self._is_smooth_corner_waypoint(self.path_index + 1)):
            speed = min(speed, self.HIGHWAY_CORNER_SPEED_MM_S)

        # Conservative smooth turn: bot is arcing into a sharper turn,
        # reduce speed for stability.
        if self._smooth_turn_tier == "conservative":
            speed = min(speed, self.HIGHWAY_TURN_SPEED_MM_S)

        # If the next waypoint requires a turn, slow this segment down.
        if self.path_index < len(self.path_waypoints) - 2:
            cx, cy, _ = self.path_waypoints[self.path_index]
            nx, ny, _ = self.path_waypoints[self.path_index + 1]
            ax = cx - self.x_mm
            ay = cy - self.y_mm
            bx = nx - cx
            by = ny - cy
            if math.hypot(ax, ay) > 1.0 and math.hypot(bx, by) > 1.0:
                in_heading = math.atan2(ay, ax)
                out_heading = math.atan2(by, bx)
                if abs(wrap(out_heading - in_heading)) > self.HIGHWAY_STRAIGHT_ANGLE_TOL:
                    speed = min(speed, self.HIGHWAY_TURN_SPEED_MM_S)

        return max(0.0, speed)

    # ══════════════════════════════════════════════════════════
    #  Opponent stop-zone guard
    # ══════════════════════════════════════════════════════════
    def _rect_corners_mm(self, x_mm: float, y_mm: float, yaw_rad: float,
                         front_mm: float, rear_mm: float, half_width_mm: float):
        """Return oriented rectangle corners in world mm.

        The pose is the wheel-axis/reference point. +front is along yaw,
        +lateral is to the robot's left in field coordinates.
        """
        c = math.cos(yaw_rad)
        s = math.sin(yaw_rad)
        corners = []
        for fwd, lat in (
            (front_mm, half_width_mm),
            (front_mm, -half_width_mm),
            (-rear_mm, -half_width_mm),
            (-rear_mm, half_width_mm),
        ):
            corners.append((x_mm + fwd * c - lat * s,
                            y_mm + fwd * s + lat * c))
        return corners

    def _project_polygon_axis(self, pts, ax, ay):
        vals = [x * ax + y * ay for x, y in pts]
        return min(vals), max(vals)

    def _polygons_overlap(self, pts_a, pts_b) -> bool:
        """Separating-axis test for two convex polygons."""
        eps = 1e-6
        for pts in (pts_a, pts_b):
            for i in range(len(pts)):
                x1, y1 = pts[i]
                x2, y2 = pts[(i + 1) % len(pts)]
                ex, ey = x2 - x1, y2 - y1
                length = math.hypot(ex, ey)
                if length <= eps:
                    continue
                ax, ay = -ey / length, ex / length
                a_min, a_max = self._project_polygon_axis(pts_a, ax, ay)
                b_min, b_max = self._project_polygon_axis(pts_b, ax, ay)
                if a_max < b_min or b_max < a_min:
                    return False
        return True

    def _enemy_pose_age_s(self):
        if self.enemy_pose_time_s is None:
            return None
        return time.monotonic() - self.enemy_pose_time_s

    def _enemy_pose_fresh(self) -> bool:
        age = self._enemy_pose_age_s()
        return bool(
            self.enemy_pose_valid
            and age is not None
            and age <= self.AVOIDANCE_ENEMY_POSE_TIMEOUT_S
        )

    def _own_collision_rect_mm(self):
        return self._rect_corners_mm(
            self.x_mm, self.y_mm, self.theta_rad,
            self.BOT_FRONT, self.BOT_REAR, self.BOT_HALF_W,
        )

    def _enemy_avoidance_rect_mm(self):
        return self._rect_corners_mm(
            self.enemy_x_mm, self.enemy_y_mm, self.enemy_theta_rad,
            self.BOT_FRONT + self.AVOIDANCE_OPPONENT_FRONT_MARGIN_MM,
            self.BOT_REAR + self.AVOIDANCE_OPPONENT_REAR_MARGIN_MM,
            self.BOT_HALF_W + self.AVOIDANCE_OPPONENT_SIDE_MARGIN_MM,
        )

    def _avoidance_should_stop(self) -> bool:
        if not self.AVOIDANCE_ENABLED:
            return False
        if not self._enemy_pose_fresh():
            return False
        return self._polygons_overlap(self._own_collision_rect_mm(), self._enemy_avoidance_rect_mm())

    def _avoidance_goal_blocked(self) -> bool:
        """True when the active goal/waypoint would place our bot inside the opponent perimeter.

        This is checked after escape/stop has moved us outside the perimeter.
        Without this, the normal controller immediately tries to drive back to
        the original waypoint even if that waypoint is still blocked.
        """
        if not self.AVOIDANCE_ENABLED:
            return False
        if not self._enemy_pose_fresh():
            return False
        if not self.goal_active:
            return False

        goal_rect = self._rect_corners_mm(
            self.goal_x, self.goal_y, self.goal_yaw,
            self.BOT_FRONT, self.BOT_REAR, self.BOT_HALF_W,
        )
        return self._polygons_overlap(goal_rect, self._enemy_avoidance_rect_mm())

    def _avoidance_escape_command(self):
        """Turn away from the opponent center, then creep forward until clear."""
        away_yaw = math.atan2(self.y_mm - self.enemy_y_mm, self.x_mm - self.enemy_x_mm)
        yaw_err = wrap(away_yaw - self.theta_rad)
        max_yaw = abs(float(self.AVOIDANCE_ESCAPE_MAX_YAW_RAD_S))
        yaw_cmd = clampf(self.AVOIDANCE_ESCAPE_YAW_GAIN * yaw_err, -max_yaw, max_yaw)

        # Rotate in place until roughly facing away. Once aligned, creep forward
        # with a small heading correction. This keeps the escape behavior simple
        # and easy to reason about from the YAML decision tree.
        if abs(yaw_err) > self.AVOIDANCE_ESCAPE_YAW_TOL_RAD:
            self.avoidance_action = "escape_turn_away"
            self._drive_vw(0.0, yaw_cmd, immediate=False)
        else:
            self.avoidance_action = "escape_drive_away"
            self._drive_vw(abs(float(self.AVOIDANCE_ESCAPE_LINEAR_MM_S)), yaw_cmd, immediate=False)
        return away_yaw, yaw_err

    def _publish_avoidance_status(self, force: bool = False):
        now = time.monotonic()
        if not force and now - self._last_avoidance_status_publish_s < 0.10:
            return
        self._last_avoidance_status_publish_s = now

        age = self._enemy_pose_age_s()
        # Keep telemetry accurate even while idle; only _apply_avoidance_decision_tree()
        # is allowed to emit stop/clear events or affect navigation state.
        physical_blocked = self._avoidance_should_stop()
        goal_blocked = self._avoidance_goal_blocked()
        self.avoidance_blocked = physical_blocked or goal_blocked
        desired_away_yaw = None
        yaw_err = None
        if self.enemy_pose_valid:
            desired_away_yaw = math.atan2(self.y_mm - self.enemy_y_mm, self.x_mm - self.enemy_x_mm)
            yaw_err = wrap(desired_away_yaw - self.theta_rad)
        payload = {
            "enabled": bool(self.AVOIDANCE_ENABLED),
            "decision_inside_perimeter": str(self.AVOIDANCE_DECISION_INSIDE_PERIMETER),
            "decision_goal_inside_perimeter": str(self.AVOIDANCE_DECISION_GOAL_INSIDE_PERIMETER),
            "action": str(self.avoidance_action),
            "blocked": bool(self.avoidance_blocked),
            "physical_blocked": bool(physical_blocked),
            "goal_blocked": bool(goal_blocked),
            "enemy_pose_valid": bool(self.enemy_pose_valid),
            "enemy_pose_fresh": bool(self._enemy_pose_fresh()),
            "enemy_pose_age_s": None if age is None else round(float(age), 3),
            "source": "/vision/enemy_pose",
            "own_rect_mm": [[round(float(x), 1), round(float(y), 1)] for x, y in self._own_collision_rect_mm()],
            "enemy_rect_mm": [[round(float(x), 1), round(float(y), 1)] for x, y in self._enemy_avoidance_rect_mm()] if self.enemy_pose_valid else [],
            "enemy_pose_mm": [
                round(float(self.enemy_x_mm), 1),
                round(float(self.enemy_y_mm), 1),
                round(rad2deg(self.enemy_theta_rad), 1),
            ] if self.enemy_pose_valid else None,
            "front_margin_mm": float(self.AVOIDANCE_OPPONENT_FRONT_MARGIN_MM),
            "rear_margin_mm": float(self.AVOIDANCE_OPPONENT_REAR_MARGIN_MM),
            "side_margin_mm": float(self.AVOIDANCE_OPPONENT_SIDE_MARGIN_MM),
            "escape_linear_mm_s": float(self.AVOIDANCE_ESCAPE_LINEAR_MM_S),
            "escape_max_yaw_rad_s": float(self.AVOIDANCE_ESCAPE_MAX_YAW_RAD_S),
            "escape_yaw_tolerance_deg": round(rad2deg(self.AVOIDANCE_ESCAPE_YAW_TOL_RAD), 1),
            "away_yaw_deg": None if desired_away_yaw is None else round(rad2deg(desired_away_yaw), 1),
            "away_yaw_error_deg": None if yaw_err is None else round(rad2deg(yaw_err), 1),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.avoidance_status_pub.publish(msg)

    def _apply_avoidance_decision_tree(self) -> bool:
        """Run the YAML-configured avoidance branches.

        Branch 1 handles physical overlap. Branch 2 handles the case where we
        have escaped, but the current target/waypoint would drive us straight
        back into the opponent perimeter.
        """
        physical_blocked = self._avoidance_should_stop()
        goal_blocked = self._avoidance_goal_blocked()
        self.avoidance_blocked = physical_blocked or goal_blocked

        if physical_blocked:
            if not self._avoidance_was_blocked:
                mode = self.AVOIDANCE_DECISION_INSIDE_PERIMETER
                self._pub_str(self.event_pub, f"AVOIDANCE_ENTER mode={mode} enemy_perimeter_overlap")
                if mode == "stop" and self.AVOIDANCE_STOP_PUBLISH_ERROR:
                    self._pub_str(self.error_pub, "AVOIDANCE_STOP")
            self._avoidance_was_blocked = True

            if self.AVOIDANCE_DECISION_INSIDE_PERIMETER == "escape":
                self._avoidance_escape_command()
            else:
                self.avoidance_action = "stop_hold"
                self._stop_motors(immediate=True)

            self._publish_avoidance_status(force=True)
            return True

        if goal_blocked and self.AVOIDANCE_DECISION_GOAL_INSIDE_PERIMETER == "hold":
            if not self._avoidance_was_blocked:
                self._pub_str(self.event_pub, "AVOIDANCE_GOAL_BLOCKED hold")
                if self.AVOIDANCE_HOLD_PUBLISH_ERROR:
                    self._pub_str(self.error_pub, "AVOIDANCE_GOAL_BLOCKED")
            self._avoidance_was_blocked = True
            self.avoidance_action = "goal_blocked_hold"
            self._stop_motors(immediate=True)
            self._publish_avoidance_status(force=True)
            return True

        self.avoidance_action = "clear"
        if self._avoidance_was_blocked:
            self._pub_str(self.event_pub, "AVOIDANCE_CLEAR")
            self._avoidance_was_blocked = False
            self._publish_avoidance_status(force=True)
        return False

    # Backward-compatible name for older local edits that may call this helper.
    def _apply_avoidance_stop_guard(self) -> bool:
        return self._apply_avoidance_decision_tree()

    def _drive_vw(self, linear_mm_s: float, yaw_rad_s: float, immediate: bool = False):
        """Convert mm/s + rad/s to a Twist and publish, with optional command ramping."""
        linear_mm_s = clampf(linear_mm_s, -self.MAX_REVERSE_MM_S, self.MAX_LINEAR_MM_S)
        yaw_rad_s = clampf(yaw_rad_s, -self.MAX_YAW_RAD_S, self.MAX_YAW_RAD_S)

        now = time.monotonic()
        dt = clampf(now - self._last_cmd_time, 0.001, 0.1)
        self._last_cmd_time = now

        if self.CMD_RAMP_ENABLED and not immediate:
            linear_limit = self.LINEAR_ACCEL_MM_S2
            if (linear_mm_s * self._cmd_linear_mm_s < 0.0 or
                    abs(linear_mm_s) < abs(self._cmd_linear_mm_s)):
                linear_limit = self.LINEAR_DECEL_MM_S2

            yaw_limit = self.YAW_ACCEL_RAD_S2
            if (yaw_rad_s * self._cmd_yaw_rad_s < 0.0 or
                    abs(yaw_rad_s) < abs(self._cmd_yaw_rad_s)):
                yaw_limit = self.YAW_DECEL_RAD_S2

            self._cmd_linear_mm_s = step_toward(
                self._cmd_linear_mm_s, linear_mm_s, linear_limit * dt
            )
            self._cmd_yaw_rad_s = step_toward(
                self._cmd_yaw_rad_s, yaw_rad_s, yaw_limit * dt
            )
        else:
            self._cmd_linear_mm_s = linear_mm_s
            self._cmd_yaw_rad_s = yaw_rad_s

        t = Twist()
        t.linear.x = self._cmd_linear_mm_s / 1000.0   # mm/s → m/s
        t.angular.z = self._cmd_yaw_rad_s
        self.cmd_vel_pub.publish(t)

    def _stop_motors(self, immediate: bool = False):
        self._drive_vw(0.0, 0.0, immediate=immediate)

    def _set_dustpan_angle(self, angle_rad: float):
        """Set desired dustpan target angle. Telemetry loop ramps the command."""
        self.dustpan_target_angle = angle_rad

    def _dustpan_up(self):
        """Raise dustpan for driving."""
        self._set_dustpan_angle(self.DUSTPAN_UP_ANGLE)

    def _dustpan_down(self):
        """Lower dustpan for scooping/placing."""
        self._set_dustpan_angle(self.DUSTPAN_DOWN_ANGLE)

    def _carwash_arm_up(self):
        """Raise carwash arm to stowed position."""
        self.carwash_arm_target_angle = self.CARWASH_ARM_UP

    def _carwash_arm_down(self):
        """Lower carwash arm to press on pieces."""
        self.carwash_arm_target_angle = self.CARWASH_ARM_DOWN

    def _carwash_spin(self, speed: float):
        """Set roller target spin speed (positive = pull in, negative = push out, 0 = stop)."""
        self.carwash_roller_target_speed = speed

    def _carwash_stow(self):
        """Arm up, roller stop."""
        self._carwash_arm_up()
        self._carwash_spin(self.CARWASH_ROLLER_STOP)

    def _update_actuator_motion(self):
        """Ramp position/speed commands for Gazebo actuator controllers."""
        now = time.monotonic()
        dt = clampf(now - self._last_actuator_update, 0.001, 0.2)
        self._last_actuator_update = now

        self.dustpan_angle = step_toward(
            self.dustpan_angle, self.dustpan_target_angle, self.DUSTPAN_MOVE_SPEED_RAD_S * dt
        )
        self.carwash_arm_angle = step_toward(
            self.carwash_arm_angle, self.carwash_arm_target_angle, self.CARWASH_ARM_MOVE_SPEED_RAD_S * dt
        )
        self.carwash_roller_speed = step_toward(
            self.carwash_roller_speed,
            self.carwash_roller_target_speed,
            self.CARWASH_ROLLER_ACCEL_RAD_S2 * dt,
        )

    def _drive_toward_xy(self, tx, ty, allow_reverse, max_speed=0.0) -> bool:
        """Port of driveTowardXY(). Returns True when arrived."""
        eff_max = max_speed if max_speed > 0 else self.MAX_LINEAR_MM_S
        dx = tx - self.x_mm
        dy = ty - self.y_mm
        rho = math.hypot(dx, dy)

        if rho <= self.POS_TOL_MM:
            return True

        target_heading = math.atan2(dy, dx)
        alpha = wrap(target_heading - self.theta_rad)
        alpha_ctrl = alpha
        reversing = False

        if (allow_reverse and self.ALLOW_REVERSE
                and rho < self.REVERSE_MAX_DIST
                and abs(alpha) > deg2rad(self.REVERSE_HEADING_THRESHOLD_DEG)):
            reversing = True
            alpha_ctrl = wrap(alpha + (-PI if alpha > 0 else PI))

        # Integral on heading error
        if abs(alpha_ctrl) < deg2rad(30.0):
            self.alpha_integral += alpha_ctrl * 0.005  # dt ≈ 5ms
            self.alpha_integral = clampf(self.alpha_integral,
                                         -self.ALPHA_I_LIMIT, self.ALPHA_I_LIMIT)
        else:
            self.alpha_integral = 0.0

        yaw_cmd = self.K_alpha * alpha_ctrl + self.K_alpha_i * self.alpha_integral

        if reversing:
            linear_cmd = -abs(self.K_rho * rho * math.cos(alpha_ctrl))
        else:
            # Do not drive forward when the target is behind the robot. The old
            # abs(cos(alpha)) behavior could make the bot drive away from an
            # overshot waypoint while spinning.
            forward_factor = max(0.0, math.cos(alpha_ctrl))
            linear_cmd = self.K_rho * rho * forward_factor

        linear_cmd = clampf(linear_cmd, -eff_max, eff_max)
        yaw_cmd = clampf(yaw_cmd, -self.MAX_YAW_RAD_S, self.MAX_YAW_RAD_S)

        # Boundary slowdown
        linear_cmd *= self._boundary_speed_fraction()

        self._drive_vw(linear_cmd, yaw_cmd)
        return False

    def _is_zone_straight_waypoint(self) -> bool:
        """True for any path waypoint explicitly marked fixed-yaw.

        This is intentionally no longer limited to labels starting with GO_ZONE
        or GO_PANTRY. Strategy YAML can now use fixed_yaw: true and/or
        reverse: true on any drive_path label.
        """
        if not self.path_active:
            return False
        return self._is_fixed_yaw_path_index(self.path_index)

    def _drive_zone_straight_waypoint(self) -> bool:
        """
        Drive to the current zone waypoint while holding goal_yaw.

        This intentionally ignores lateral point-steering. The robot first
        faces the zone, then moves only forward/backward along that heading.
        """
        tx = self.goal_x
        ty = self.goal_y
        hold_yaw = self.goal_yaw

        dx = tx - self.x_mm
        dy = ty - self.y_mm

        forward_x = math.cos(hold_yaw)
        forward_y = math.sin(hold_yaw)

        # Signed distance along the required approach direction.
        # Positive = target is in front of the held yaw.
        # Negative = target is behind the held yaw.
        along = dx * forward_x + dy * forward_y
        dist = math.hypot(dx, dy)
        force_reverse = self._is_force_reverse_path_index(self.path_index)

        force_reverse = self._is_force_reverse_path_index(self.path_index)

        yaw_err = wrap(hold_yaw - self.theta_rad)

        # If the position/forward distance is done, still require the held yaw to be
        # fully settled before reporting the waypoint complete. This prevents strategy
        # steps like roller_push_start from starting while the bot is still turning.
        if dist <= self.POS_TOL_MM or abs(along) <= self.POS_TOL_MM:
            if self._rotate_toward_yaw(hold_yaw):
                self._stop_motors()
                return True
            return False

        # If the robot is not facing the zone accurately enough, rotate first.
        if abs(yaw_err) > self.JENGA_STRAIGHT_YAW_TOL:
            yaw_cmd = self.FINAL_YAW_K * yaw_err
            yaw_cmd = clampf(yaw_cmd, -self.FINAL_YAW_MAX, self.FINAL_YAW_MAX)
            if abs(yaw_cmd) < self.FINAL_YAW_MIN:
                yaw_cmd = self.FINAL_YAW_MIN if yaw_cmd >= 0 else -self.FINAL_YAW_MIN
            self._drive_vw(0.0, yaw_cmd)
            return False

        straight_max_speed = self.JENGA_STRAIGHT_SPEED_MM_S
        straight_min_speed = self.JENGA_STRAIGHT_MIN_SPEED_MM_S
        if self.goal_label.startswith("GO_PANTRY"):
            straight_max_speed = self.PANTRY_STRAIGHT_SPEED_MM_S
            straight_min_speed = self.PANTRY_STRAIGHT_MIN_SPEED_MM_S

        speed = self.K_rho * abs(along)
        speed = clampf(speed, 0.0, straight_max_speed)

        if abs(along) > 50.0:
            speed = max(speed, straight_min_speed)

        if force_reverse:
            # Explicit strategy command: never turn around and drive forward.
            # The waypoint must be behind the held yaw; otherwise stop and report
            # a YAML/route error instead of driving into the wrong place.
            if along > self.POS_TOL_MM:
                self._pub_str(
                    self.error_pub,
                    f"FORCE_REVERSE_TARGET_IN_FRONT label={self.path_final_label} along={along:.1f}"
                )
                self._stop_motors()
                return False
            linear_cmd = -speed
        else:
            linear_cmd = speed if along > 0.0 else -speed

        # Yaw-hold only. No point-steering.
        yaw_cmd = clampf(
            self.FINAL_YAW_K * yaw_err,
            -self.FINAL_YAW_MAX,
            self.FINAL_YAW_MAX,
        )

        self._drive_vw(linear_cmd, yaw_cmd)
        return False

    def _rotate_toward_yaw(self, target_yaw) -> bool:
        """Port of rotateTowardYaw(). Returns True when settled."""
        yaw_err = wrap(target_yaw - self.theta_rad)

        if abs(yaw_err) <= self.FINAL_YAW_TOL:
            if self.final_yaw_in_tol_since is None:
                self.final_yaw_in_tol_since = time.monotonic()
            self._stop_motors()
            return (time.monotonic() - self.final_yaw_in_tol_since) >= self.FINAL_YAW_HOLD_S
        else:
            self.final_yaw_in_tol_since = None
            yaw_cmd = self.FINAL_YAW_K * yaw_err
            yaw_cmd = clampf(yaw_cmd, -self.FINAL_YAW_MAX, self.FINAL_YAW_MAX)
            if abs(yaw_cmd) < self.FINAL_YAW_MIN:
                yaw_cmd = self.FINAL_YAW_MIN if yaw_cmd >= 0 else -self.FINAL_YAW_MIN
            self._drive_vw(0.0, yaw_cmd)
            return False

    # ══════════════════════════════════════════════════════════
    #  Goal setup
    # ══════════════════════════════════════════════════════════
    def _activate_axis_goal(self, ax, ay, yaw, ref, label, use_approach=True, send_ack=True):
        """Activate one axis-frame goal. This is used by both direct GO and path waypoints."""
        self.goal_x = ax
        self.goal_y = ay
        self.goal_yaw = yaw
        self.goal_active = True
        self.alpha_integral = 0.0
        self.goal_label = label
        self.goal_ref = ref
        self.run_mode = RunMode.GOAL
        self.final_yaw_in_tol_since = None

        # Normal direct goals can use the old approach-point behavior.
        # Highway-planned waypoints disable this because the waypoint list is
        # already the approach path.
        dist = math.hypot(ax - self.x_mm, ay - self.y_mm)
        if use_approach and dist > self.APPROACH_MIN_DIST:
            ap_x = ax - self.APPROACH_OFFSET * math.cos(yaw)
            ap_y = ay - self.APPROACH_OFFSET * math.sin(yaw)
            ap_x = clampf(ap_x, self.BOT_FRONT + 20, self.FIELD_X_MAX - self.BOT_FRONT - 20)
            ap_y = clampf(ap_y, self.BOT_HALF_W + 20, self.FIELD_Y_MAX - self.BOT_HALF_W - 20)
            self.approach_x = ap_x
            self.approach_y = ap_y
            self.approach_active = True
            self.nav_state = NavState.DRIVE_TO_APPROACH
            self._pub_str(self.event_pub, f"APPROACH {ap_x:.1f} {ap_y:.1f}")
        else:
            self.approach_active = False
            self.nav_state = NavState.DRIVE_TO_POINT

        self._pub_str(self.event_pub, f"STATE {self.run_mode} {self.nav_state} 1 0")
        if send_ack:
            self._pub_str(self.ack_pub, label)
            self._pub_str(self.event_pub, "GO_STARTED")

    def _set_new_goal(self, gx, gy, gyaw_deg, ref, label):
        yaw = wrap(deg2rad(gyaw_deg))
        offset = self._offset_for_ref(ref)
        ax = gx - offset * math.cos(yaw)
        ay = gy - offset * math.sin(yaw)
        ax, ay, clamped = self._clamp_goal_to_bounds(ax, ay, yaw, 5.0)
        if clamped:
            self._pub_str(self.event_pub, "WARN GOAL_CLAMPED_TO_BOUNDS")

        self.path_active = False
        self.path_waypoints = []
        self.path_index = 0
        # Indices of generated safe-corner arc waypoints. These are allowed to
        # roll through without a stop/yaw settle, but only at corner speed.
        self.path_smooth_indices = set()
        # Indices that must be driven as fixed-yaw straight movements
        # (final pickup/placement push, and reverse backup if explicitly used).
        self.path_fixed_yaw_indices = set()
        self.path_force_reverse_indices = set()
        self._pending_smooth_indices = set()
        self.path_final_label = ""
        self.path_final_ref = ref
        self.path_auto_dustpan = False

        uses_highway = (
            label in ("GO", "GO_CENTER", "GO_DUSTPAN", "GO_HOME")
            or label.startswith("GO_ZONE")
        )

        if self.USE_HIGHWAY_PLANNER and uses_highway:
            if not self.odom_valid:
                # Do NOT fall back to old direct drive if the pose is not ready.
                # Queue the command and start it once /bot_pose_ground_truth arrives.
                self.pending_goal = (gx, gy, gyaw_deg, ref, label)
                self.goal_active = False
                self.run_mode = RunMode.IDLE
                self.nav_state = NavState.GOAL_REACHED
                self.approach_active = False
                self.path_active = False
                self.path_waypoints = []
                self.path_index = 0
                self.path_smooth_indices = set()
                self.path_fixed_yaw_indices = set()
                self.path_force_reverse_indices = set()
                self._stop_motors()
                self._pub_str(self.ack_pub, label)
                self._pub_str(self.event_pub, "WAITING_FOR_POSE_BEFORE_HIGHWAY_PLAN")
                return

            waypoints = self._build_highway_waypoints(ax, ay, yaw)

            # Important:
            # Even a one-waypoint highway plan must NOT fall back to the old
            # approach-point controller, because that is what caused
            # APPROACH 2500.0 170.0 and made the bot ignore the highway system.
            if waypoints:
                self.path_active = True
                self.path_waypoints = waypoints
                self.path_index = 0
                self.path_smooth_indices = set(getattr(self, "_pending_smooth_indices", set()))
                self.path_fixed_yaw_indices = set()
                self.path_force_reverse_indices = set()
                self.path_final_label = label
                self.path_final_ref = ref
                self.path_auto_dustpan = False
                self.path_start_x = self.x_mm
                self.path_start_y = self.y_mm

                self._dustpan_up()  # Raise dustpan for driving
                self._carwash_stow()  # Stow carwash arm

                self._pub_str(self.ack_pub, label)
                self._pub_str(self.event_pub, f"PATH_START {label} {len(waypoints)}")
                self._start_next_path_waypoint()
                return

        # Fallback/direct mode.
        self._activate_axis_goal(ax, ay, yaw, ref, label, use_approach=True, send_ack=True)

    def _go_home(self):
        # GO_HOME intentionally keeps the original special homing behavior and
        # does not use the highway planner.
        self.path_active = False
        self.path_waypoints = []
        self.path_index = 0
        # Indices of generated safe-corner arc waypoints. These are allowed to
        # roll through without a stop/yaw settle, but only at corner speed.
        self.path_smooth_indices = set()
        # Indices that must be driven as fixed-yaw straight movements
        # (final pickup/placement push, and reverse backup if explicitly used).
        self.path_fixed_yaw_indices = set()
        self.path_force_reverse_indices = set()
        self._pending_smooth_indices = set()
        self.path_final_label = ""
        self.path_auto_dustpan = False

        home_yaw = wrap(deg2rad(self.home_yaw_deg))
        self.home_stage_x = self.home_x - self.HOME_STAGING_OFFSET * math.cos(home_yaw)
        self.home_stage_y = self.home_y - self.HOME_STAGING_OFFSET * math.sin(home_yaw)
        self.home_stage_x = clampf(self.home_stage_x, self.BOT_FRONT + 20,
                                    self.FIELD_X_MAX - self.BOT_FRONT - 20)
        self.home_stage_y = clampf(self.home_stage_y, self.BOT_HALF_W + 20,
                                    self.FIELD_Y_MAX - self.BOT_HALF_W - 20)

        dist = math.hypot(self.home_stage_x - self.x_mm, self.home_stage_y - self.y_mm)
        if dist < self.APPROACH_MIN_DIST:
            self.go_home_phase = 1
        else:
            self.go_home_phase = 0

        if self.go_home_phase == 0:
            heading_to_home = math.atan2(self.home_y - self.home_stage_y,
                                          self.home_x - self.home_stage_x)
            self.goal_x = self.home_stage_x
            self.goal_y = self.home_stage_y
            self.goal_yaw = heading_to_home
        else:
            self.goal_x = self.home_x
            self.goal_y = self.home_y
            self.goal_yaw = wrap(deg2rad(self.home_yaw_deg))

        self.goal_active = True
        self.nav_state = NavState.DRIVE_TO_POINT
        self.run_mode = RunMode.GOAL
        self.goal_label = "GO_HOME"
        self.goal_ref = RefPoint.CENTER
        self.alpha_integral = 0.0
        self.approach_active = False

    def _abort_drive(self, immediate_stop: bool = False):
        self.goal_active = False
        self.nav_state = NavState.GOAL_REACHED
        self.run_mode = RunMode.IDLE
        self.goal_label = ""
        self.goal_ref = RefPoint.AXIS
        self.alpha_integral = 0.0
        self.approach_active = False
        self.path_active = False
        self.path_waypoints = []
        self.path_index = 0
        # Indices of generated safe-corner arc waypoints. These are allowed to
        # roll through without a stop/yaw settle, but only at corner speed.
        self.path_smooth_indices = set()
        # Indices that must be driven as fixed-yaw straight movements
        # (final pickup/placement push, and reverse backup if explicitly used).
        self.path_fixed_yaw_indices = set()
        self.path_force_reverse_indices = set()
        self._pending_smooth_indices = set()
        self.path_final_label = ""
        self.path_auto_dustpan = False
        self.pending_goal = None
        self.align_for_approach_override_yaw = None
        self.strategy_active = False
        self.strategy_name = ""
        self.strategy_steps = []
        self.strategy_index = 0
        self.strategy_waiting_drive_label = None
        self.strategy_timer_until = None
        self.strategy_timer_kind = None
        self.strategy_total_start_time = None
        self._stop_motors(immediate=immediate_stop)

    def _finish_current_goal(self):
        """Called after ALIGN_FINAL_YAW succeeds."""
        # If this was an intermediate highway waypoint, load the next waypoint
        # instead of reporting DONE.
        if self.path_active and self.path_index < len(self.path_waypoints) - 1:
            self._pub_str(self.event_pub, f"PATH_WP_REACHED {self.path_index + 1}/{len(self.path_waypoints)}")
            self.path_index += 1
            self._start_next_path_waypoint()
            return

        done_label = self.path_final_label if self.path_active else self.goal_label

        self.nav_state = NavState.GOAL_REACHED
        self.goal_active = False
        self.run_mode = RunMode.IDLE
        self.approach_active = False
        self.path_active = False
        self.path_waypoints = []
        self.path_index = 0
        # Indices of generated safe-corner arc waypoints. These are allowed to
        # roll through without a stop/yaw settle, but only at corner speed.
        self.path_smooth_indices = set()
        # Indices that must be driven as fixed-yaw straight movements
        # (final pickup/placement push, and reverse backup if explicitly used).
        self.path_fixed_yaw_indices = set()
        self.path_force_reverse_indices = set()
        self._pending_smooth_indices = set()
        self.path_final_label = ""
        self.align_for_approach_override_yaw = None
        self._stop_motors()
        self._pub_str(self.event_pub, "REACHED")
        if done_label:
            self._pub_str(self.done_pub, done_label)
        self.goal_label = ""

        # Advance config-driven strategy sequencer if active.
        if self.strategy_active and self.strategy_waiting_drive_label == done_label:
            self.strategy_waiting_drive_label = None
            self.strategy_index += 1
            self._strategy_start_next_step()
            return

        # Advance mission state machine if active
        if self.mission_active:
            if self.mission_state == "GOING_TO_ZONE" and done_label.startswith("GO_ZONE"):
                self.mission_state = "SETTLING_AT_ZONE"
                self.mission_timer_start = time.monotonic()
                self._pub_str(self.event_pub, "MISSION_ARRIVED_AT_ZONE")
            elif self.mission_state == "GOING_TO_PANTRY" and done_label.startswith("GO_PANTRY"):
                self.mission_state = "SETTLING_AT_PANTRY"
                self.mission_timer_start = time.monotonic()
                self._pub_str(self.event_pub, "MISSION_ARRIVED_AT_PANTRY")


    def _stop_all_for_match_timeout(self, now_s: float, limit_s: float):
        """Hard stop triggered by simulated match time limit."""
        # Abort all autonomous sequencing first so nothing starts again after stop.
        self._abort_drive(immediate_stop=True)
        self.mission_active = False
        self.mission_state = "IDLE"
        self.mission_queue = []
        self.mission_total_start_time = None
        self.mission_total_task_count = 0

        # Stop/freeze actuators. The roller must stop; arm and dustpan targets
        # are frozen at their current commanded positions so they do not keep moving.
        self.carwash_roller_target_speed = self.CARWASH_ROLLER_STOP
        self.carwash_roller_speed = self.CARWASH_ROLLER_STOP
        self.carwash_arm_target_angle = self.carwash_arm_angle
        self.dustpan_target_angle = self.dustpan_angle

        self._stop_motors(immediate=True)
        text = f"MATCH_TIME_LIMIT_STOP_ALL sim_time_s={now_s:.3f} limit_s={limit_s:.3f}"
        self._pub_str(self.event_pub, text)
        self._pub_str(self.done_pub, text)
        self.get_logger().warn(text)

    def _check_match_time_limit(self) -> bool:
        """Return True once the match-time limit has just triggered stop-all."""
        if not bool(self.MATCH_TIMER_ENABLED):
            return False

        limit_s = float(self.MATCH_STOP_ALL_AT_S)
        if limit_s <= 0.0:
            return False

        now_s = self._sim_time_s()

        # If Gazebo/world time is reset while this node keeps running, allow the
        # stop-all to trigger again on the next match.
        if self._match_stop_all_sent:
            if now_s < max(0.0, limit_s - 1.0):
                self._match_stop_all_sent = False
            return False

        if now_s >= limit_s:
            self._match_stop_all_sent = True
            self._stop_all_for_match_timeout(now_s, limit_s)
            return True

        return False

    # ══════════════════════════════════════════════════════════
    #  Control loop (5ms / 200Hz) — nav state machine
    # ══════════════════════════════════════════════════════════
    def _control_loop(self):
        if self._check_match_time_limit():
            return

        if not self.odom_valid:
            return

        if not self._drive_pose_fresh():
            self._stop_motors()
            return

        # Mission/strategy sequencers tick even when drive is idle.
        self._mission_tick()
        self._strategy_tick()

        if self.run_mode == RunMode.IDLE:
            self._stop_motors()
            return

        if not self.goal_active:
            self._stop_motors()
            return

        # Opponent perimeter decision tree. This does not replan yet; it either
        # holds still or runs a simple escape branch while blocked. Once clear,
        # the existing controller continues toward the same waypoint/goal.
        if self._apply_avoidance_decision_tree():
            return

        if self.nav_state == NavState.DRIVE_TO_POINT:
            if self._maybe_pass_straight_through_waypoint():
                return
            if self._maybe_advance_overshot_waypoint():
                return

        if self.nav_state == NavState.DRIVE_TO_APPROACH:
            if self._drive_toward_xy(self.approach_x, self.approach_y, True):
                self.nav_state = NavState.ALIGN_FOR_APPROACH
                self.final_yaw_in_tol_since = None
                self.alpha_integral = 0.0
                self._stop_motors()
                self._pub_str(self.event_pub, "APPROACH_REACHED")

        elif self.nav_state == NavState.ALIGN_FOR_APPROACH:
            if self.align_for_approach_override_yaw is not None:
                heading_to_goal = self.align_for_approach_override_yaw
            else:
                heading_to_goal = math.atan2(self.goal_y - self.y_mm,
                                              self.goal_x - self.x_mm)

            if self._rotate_toward_yaw(heading_to_goal):
                self.nav_state = NavState.DRIVE_TO_POINT
                self.alpha_integral = 0.0
                self.approach_active = False
                self.align_for_approach_override_yaw = None
                self._pub_str(self.event_pub, "ALIGNED_FOR_FINAL")

        elif self.nav_state == NavState.DRIVE_TO_POINT:
            is_zone_goal = (
                self.goal_label.startswith("GO_ZONE")
                or self.path_final_label.startswith("GO_ZONE")
                or self.goal_label.startswith("GO_PANTRY")
                or self.path_final_label.startswith("GO_PANTRY")
            )

            allow_rev = (
                self.goal_label == "BOUNDARY_RECOVER"
                or (self.goal_label == "GO_HOME" and self.go_home_phase == 2)
                or is_zone_goal
            )

            speed_limit = 0.0

            if self.path_active:
                is_final_path_wp = self.path_index >= len(self.path_waypoints) - 1
                if not is_final_path_wp:
                    # Intermediate path waypoints are highway travel. Limit speed
                    # separately from final approach/pickup speed.
                    speed_limit = self._current_highway_speed_limit()
                elif self.path_final_label.startswith("GO_ZONE") or self.path_final_label.startswith("GO_PANTRY"):
                    # The final small straight motion has its own zone/pantry speed limits.
                    speed_limit = 0.0

            if self._is_zone_straight_waypoint():
                # Only paths that explicitly own the dustpan may auto-lower it.
                # But do NOT lower before the fixed-yaw alignment is done.
                if self.path_auto_dustpan and self.path_index == len(self.path_waypoints) - 1:
                    yaw_err = abs(wrap(self.goal_yaw - self.theta_rad))

                    # Let _drive_zone_straight_waypoint() rotate first while dustpan is still up.
                    if yaw_err <= self.JENGA_STRAIGHT_YAW_TOL:
                        if self._dustpan_lowered_time is None:
                            self._dustpan_down()
                            self._dustpan_lowered_time = time.monotonic()
                            self._stop_motors()
                            return

                        # Wait for dustpan to settle before the final pickup/placement push.
                        if time.monotonic() - self._dustpan_lowered_time < self.DUSTPAN_LOWER_SETTLE_S:
                            self._stop_motors()
                            return

                if self._drive_zone_straight_waypoint():
                    self._dustpan_lowered_time = None
                    self._finish_current_goal()
                return

            if self.goal_label == "GO_HOME" and self.go_home_phase == 2:
                speed_limit = self.HOME_REVERSE_SPEED

            if self._drive_toward_xy(self.goal_x, self.goal_y, allow_rev, speed_limit):
                # ── Skip final yaw alignment for intermediate highway waypoints ──
                # Aggressive tier: skip final yaw entirely, advance immediately.
                # Conservative tier: do the normal final yaw correction (slower but stable).
                # No tier / final waypoint: always do final yaw.
                is_intermediate_hw_wp = (
                    self.path_active
                    and self.path_index < len(self.path_waypoints) - 1
                    and not self._is_zone_straight_waypoint()
                )
                if is_intermediate_hw_wp and self._smooth_turn_tier in ("aggressive", "corner"):
                    self._pub_str(self.event_pub,
                                  f"PATH_WP_REACHED {self.path_index + 1}/{len(self.path_waypoints)} (skip align)")
                    self.path_index += 1
                    self._start_next_path_waypoint(force_align=(self._smooth_turn_tier != "corner"))
                    return

                self.nav_state = NavState.ALIGN_FINAL_YAW
                self.final_yaw_in_tol_since = None
                self.alpha_integral = 0.0
                self._stop_motors()

        elif self.nav_state == NavState.ALIGN_FINAL_YAW:
            # Check drift
            rho = math.hypot(self.goal_x - self.x_mm, self.goal_y - self.y_mm)
            if rho > self.POS_TOL_MM * 2.0:
                self.nav_state = NavState.DRIVE_TO_POINT
                self.final_yaw_in_tol_since = None
                return

            if self._rotate_toward_yaw(self.goal_yaw):
                # GO_HOME multi-phase transitions
                if self.goal_label == "GO_HOME" and not self.path_active:
                    home_yaw = wrap(deg2rad(self.home_yaw_deg))
                    if self.go_home_phase == 0:
                        heading = math.atan2(self.home_y - self.y_mm,
                                              self.home_x - self.x_mm)
                        self.goal_yaw = heading
                        self.go_home_phase = 1
                        self.nav_state = NavState.ALIGN_FINAL_YAW
                        self.final_yaw_in_tol_since = None
                        self._pub_str(self.event_pub, "GO_HOME PHASE1_ALIGN")
                        return
                    if self.go_home_phase == 1:
                        self.goal_x = self.home_x - self.OFFSET_CENTER * math.cos(home_yaw)
                        self.goal_y = self.home_y - self.OFFSET_CENTER * math.sin(home_yaw)
                        self.goal_yaw = home_yaw
                        self.go_home_phase = 2
                        self.nav_state = NavState.DRIVE_TO_POINT
                        self.alpha_integral = 0.0
                        self._pub_str(self.event_pub, "GO_HOME PHASE2_REVERSE")
                        return
                    if self.go_home_phase == 2:
                        self.goal_yaw = home_yaw
                        self.go_home_phase = 3
                        self.nav_state = NavState.ALIGN_FINAL_YAW
                        self.final_yaw_in_tol_since = None
                        self._pub_str(self.event_pub, "GO_HOME PHASE3_FINAL_ALIGN")
                        return

                self._finish_current_goal()

        elif self.nav_state == NavState.GOAL_REACHED:
            self._stop_motors()

    def _publish_highway_path(self):
        """Publish the currently planned highway path for debug overlays."""
        payload = {
            "active": bool(self.path_active and len(self.path_waypoints) > 0),
            "label": self.path_final_label,
            "path_index": int(self.path_index),
            "current_pose_mm": [float(self.x_mm), float(self.y_mm), rad2deg(self.theta_rad)],
            "goal_pose_mm": [float(self.goal_x), float(self.goal_y), rad2deg(self.goal_yaw)] if self.goal_active else None,
            "waypoints_mm": [
                [float(x), float(y), rad2deg(yaw)]
                for (x, y, yaw) in self.path_waypoints
            ],
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.highway_path_pub.publish(msg)

    # ══════════════════════════════════════════════════════════
    #  Telemetry loop (10Hz) — mirrors .ino TEL format
    # ══════════════════════════════════════════════════════════
    def _telemetry_loop(self):
        self._pub_bool(self.connected_pub, True)

        # Odom pose topic mirrors the pose currently used by the drive controller.
        # In sim this should normally be /bot_pose_fused, not ground truth.
        pose = Pose2D()
        pose.x = self.x_mm / 1000.0
        pose.y = self.y_mm / 1000.0
        pose.theta = self.theta_rad
        self.odom_pose_pub.publish(pose)

        if self.goal_active:
            gp = Pose2D()
            gp.x = self.goal_x / 1000.0
            gp.y = self.goal_y / 1000.0
            gp.theta = self.goal_yaw
            self.goal_pose_pub.publish(gp)

        self._publish_highway_path()
        self._publish_avoidance_status()

        yaw_msg = Float32()
        yaw_msg.data = self.imu_yaw_deg
        self.imu_yaw_deg_pub.publish(yaw_msg)

        gz_msg = Float32()
        gz_msg.data = self.imu_gz
        self.gyro_z_pub.publish(gz_msg)

        nav = self.nav_state if self.goal_active else NavState.GOAL_REACHED
        status_str = f"{self.run_mode} {nav} {'1' if self.goal_active else '0'} 0"
        self._pub_str(self.status_pub, status_str)

        self._pub_str(self.brick_state_pub,
                      " ".join(self.brick_state))

        # Keep commanding actuator setpoints. The target values are ramped here so
        # actuator speed is tunable from YAML instead of jumping instantly.
        self._update_actuator_motion()

        msg = Float64(); msg.data = self.dustpan_angle
        self.dustpan_angle_pub.publish(msg)
        msg = Float64(); msg.data = self.carwash_arm_angle
        self.carwash_arm_pub.publish(msg)
        msg = Float64(); msg.data = self.carwash_roller_speed
        self.carwash_roller_pub.publish(msg)

    # ══════════════════════════════════════════════════════════
    #  Command callbacks
    # ══════════════════════════════════════════════════════════
    def _go_cb(self, msg: Pose2D):
        # msg.x, msg.y in metres, msg.theta in radians (from telemetry_console)
        self._set_new_goal(msg.x * 1000, msg.y * 1000,
                           rad2deg(msg.theta), RefPoint.AXIS, "GO")

    def _go_center_cb(self, msg: Pose2D):
        self._set_new_goal(msg.x * 1000, msg.y * 1000,
                           rad2deg(msg.theta), RefPoint.CENTER, "GO_CENTER")

    def _go_dustpan_cb(self, msg: Pose2D):
        self._set_new_goal(msg.x * 1000, msg.y * 1000,
                           rad2deg(msg.theta), RefPoint.DUSTPAN, "GO_DUSTPAN")

    def _go_zone_cb(self, msg: Int32):
        zone_id = int(msg.data)

        if zone_id not in self.JENGA_ZONES:
            self._pub_str(self.error_pub, f"INVALID_ZONE {zone_id}")
            self._pub_str(self.event_pub, f"WARN INVALID_ZONE {zone_id}")
            return

        if not self.odom_valid:
            self._pub_str(self.error_pub, "GO_ZONE_WAITING_FOR_POSE")
            self._pub_str(self.event_pub, "WAITING_FOR_POSE_BEFORE_GO_ZONE")
            return

        base_x, base_y = self.JENGA_ZONES[zone_id]
        target_x, target_y, target_off_x, target_off_y = self._target_with_offset(
            base_x, base_y, self.JENGA_ZONE_TARGET_OFFSETS_MM, zone_id
        )
        final_yaw_deg = self.JENGA_ZONE_YAWS_DEG[zone_id]
        final_yaw = wrap(deg2rad(final_yaw_deg))

        # Convert the requested dustpan/front target into the wheel-axis target.
        # This makes the dustpan/front reach the Jenga zone center.
        offset = self._offset_for_ref(RefPoint.DUSTPAN)
        final_axis_x = target_x - offset * math.cos(final_yaw)
        final_axis_y = target_y - offset * math.sin(final_yaw)
        final_axis_x, final_axis_y, clamped = self._clamp_goal_to_bounds(
            final_axis_x, final_axis_y, final_yaw, 5.0
        )
        if clamped:
            self._pub_str(self.event_pub, "WARN GO_ZONE_AXIS_CLAMPED_TO_BOUNDS")

        fwd_x = math.cos(final_yaw)
        fwd_y = math.sin(final_yaw)

        # Prefer a forward-only final approach: route to a point safely behind
        # the final dustpan pose, align there, then drive into the pieces once.
        # This prevents the old behavior where the bot drove onto/near the
        # pieces, rotated, backed away, and then drove forward again.
        approach_start_dist = max(
            self.JENGA_APPROACH_START_MM,
            self.JENGA_APPROACH_BACKUP_MM,
            self.JENGA_APPROACH_MIN_ALIGN_STANDOFF_MM,
        )
        start_x = final_axis_x - approach_start_dist * fwd_x
        start_y = final_axis_y - approach_start_dist * fwd_y
        start_x, start_y, start_clamped = self._clamp_goal_to_bounds(start_x, start_y, final_yaw, 5.0)
        start_safe = (
            self.JENGA_APPROACH_PREFER_FORWARD_START
            and not start_clamped
            and not self._point_in_inner_forbidden(start_x, start_y, self.JENGA_APPROACH_FORBIDDEN_MARGIN)
        )

        fixed_yaw_indices = set()

        if start_safe:
            lineup_x, lineup_y = start_x, start_y
            self._pub_str(
                self.event_pub,
                f"GO_ZONE_FORWARD_START {zone_id} {lineup_x:.1f} {lineup_y:.1f}"
            )
        else:
            # Fallback: use the old forced/nearest highway exit and reverse backup.
            forced_exit = self.JENGA_ZONE_FORCED_HIGHWAY_EXIT.get(zone_id)
            if forced_exit is not None:
                lineup_x, lineup_y = forced_exit
                self._pub_str(
                    self.event_pub,
                    f"GO_ZONE_FORCED_HIGHWAY_EXIT {zone_id} {lineup_x:.1f} {lineup_y:.1f}"
                )
            else:
                lineup_x, lineup_y = self._nearest_highway_point(final_axis_x, final_axis_y)

        waypoints = self._build_highway_waypoints(lineup_x, lineup_y, final_yaw)
        if not waypoints:
            waypoints = [(lineup_x, lineup_y, final_yaw)]
        else:
            waypoints[-1] = (lineup_x, lineup_y, final_yaw)

        if start_safe:
            # Only the final push is fixed-yaw straight. The approach-start
            # waypoint itself is reached by the normal point controller, then
            # the normal final-yaw alignment happens there.
            if math.hypot(final_axis_x - waypoints[-1][0], final_axis_y - waypoints[-1][1]) > 5.0:
                waypoints.append((final_axis_x, final_axis_y, final_yaw))
            else:
                waypoints[-1] = (final_axis_x, final_axis_y, final_yaw)
            fixed_yaw_indices.add(len(waypoints) - 1)
        else:
            # Old safe fallback: reverse away from the close line-up point, then
            # drive forward. Both backup and final are fixed-yaw straight moves.
            backup_x = lineup_x - self.JENGA_APPROACH_BACKUP_MM * fwd_x
            backup_y = lineup_y - self.JENGA_APPROACH_BACKUP_MM * fwd_y
            backup_x, backup_y, clamped = self._clamp_goal_to_bounds(backup_x, backup_y, final_yaw, 5.0)
            if clamped:
                self._pub_str(self.event_pub, "WARN GO_ZONE_BACKUP_CLAMPED_TO_BOUNDS")

            if math.hypot(backup_x - waypoints[-1][0], backup_y - waypoints[-1][1]) > 5.0:
                waypoints.append((backup_x, backup_y, final_yaw))
                fixed_yaw_indices.add(len(waypoints) - 1)

            if math.hypot(final_axis_x - waypoints[-1][0], final_axis_y - waypoints[-1][1]) > 5.0:
                waypoints.append((final_axis_x, final_axis_y, final_yaw))
            else:
                waypoints[-1] = (final_axis_x, final_axis_y, final_yaw)
            fixed_yaw_indices.add(len(waypoints) - 1)

        self.path_active = True
        self.path_waypoints = waypoints
        self.path_index = 0
        self.path_smooth_indices = set(getattr(self, "_pending_smooth_indices", set()))
        self.path_fixed_yaw_indices = fixed_yaw_indices
        self.path_force_reverse_indices = set()
        self.path_final_label = f"GO_ZONE_{zone_id}"
        self.path_final_ref = RefPoint.DUSTPAN
        self.path_auto_dustpan = True
        self.path_start_x = self.x_mm
        self.path_start_y = self.y_mm
        self.goal_ref = RefPoint.DUSTPAN

        self._dustpan_up()  # Raise dustpan for highway driving
        self._carwash_stow()  # Stow carwash arm
        self._dustpan_lowered_time = None  # Reset for next approach
        self._pub_str(self.ack_pub, f"GO_ZONE_{zone_id}")
        self._pub_str(
            self.event_pub,
            f"GO_ZONE_REQUEST {zone_id} "
            f"base={base_x:.1f},{base_y:.1f} "
            f"offset={target_off_x:.1f},{target_off_y:.1f} "
            f"target={target_x:.1f},{target_y:.1f} "
            f"lineup={lineup_x:.1f},{lineup_y:.1f} "
            f"axis={final_axis_x:.1f},{final_axis_y:.1f} "
            f"yaw={final_yaw_deg:.1f} "
            f"wps={len(waypoints)}"
        )
        self._pub_str(self.event_pub, f"PATH_START GO_ZONE_{zone_id} {len(waypoints)}")
        self._start_next_path_waypoint()

    def _go_pantry_cb(self, msg: Int32):
        pantry_id = int(msg.data)

        if pantry_id not in self.PANTRY_LOCATIONS:
            self._pub_str(self.error_pub, f"INVALID_PANTRY {pantry_id}")
            self._pub_str(self.event_pub, f"WARN INVALID_PANTRY {pantry_id}")
            return

        if not self.odom_valid:
            self._pub_str(self.error_pub, "GO_PANTRY_WAITING_FOR_POSE")
            self._pub_str(self.event_pub, "WAITING_FOR_POSE_BEFORE_GO_PANTRY")
            return

        base_x, base_y = self.PANTRY_LOCATIONS[pantry_id]
        target_x, target_y, target_off_x, target_off_y = self._target_with_offset(
            base_x, base_y, self.PANTRY_TARGET_OFFSETS_MM, pantry_id
        )
        final_yaw_deg = self.PANTRY_APPROACH_YAWS_DEG[pantry_id]
        final_yaw = wrap(deg2rad(final_yaw_deg))

        # Stop so dustpan tip reaches the pantry EDGE (100mm from center).
        # Move the target 100mm back from the pantry center along approach direction.
        pantry_half = self.PANTRY_HALF_SIZE_MM
        edge_x = target_x - pantry_half * math.cos(final_yaw)
        edge_y = target_y - pantry_half * math.sin(final_yaw)

        # Convert dustpan edge target into wheel-axis target.
        offset = self._offset_for_ref(RefPoint.DUSTPAN)
        final_axis_x = edge_x - offset * math.cos(final_yaw)
        final_axis_y = edge_y - offset * math.sin(final_yaw)
        final_axis_x, final_axis_y, clamped = self._clamp_goal_to_bounds(
            final_axis_x, final_axis_y, final_yaw, 5.0
        )
        if clamped:
            self._pub_str(self.event_pub, "WARN GO_PANTRY_AXIS_CLAMPED_TO_BOUNDS")

        fwd_x = math.cos(final_yaw)
        fwd_y = math.sin(final_yaw)

        approach_start_dist = max(
            self.PANTRY_APPROACH_START_MM,
            self.PANTRY_APPROACH_BACKUP_MM,
            self.PANTRY_APPROACH_MIN_ALIGN_STANDOFF_MM,
        )
        start_x = final_axis_x - approach_start_dist * fwd_x
        start_y = final_axis_y - approach_start_dist * fwd_y
        start_x, start_y, start_clamped = self._clamp_goal_to_bounds(start_x, start_y, final_yaw, 5.0)
        start_safe = (
            self.PANTRY_APPROACH_PREFER_FORWARD_START
            and not start_clamped
            and not self._point_in_inner_forbidden(start_x, start_y, 0.0)
        )

        fixed_yaw_indices = set()

        if start_safe:
            lineup_x, lineup_y = start_x, start_y
            self._pub_str(
                self.event_pub,
                f"GO_PANTRY_FORWARD_START {pantry_id} {lineup_x:.1f} {lineup_y:.1f}"
            )
        else:
            forced_exit = self.PANTRY_FORCED_HIGHWAY_EXIT.get(pantry_id)
            if forced_exit is not None:
                lineup_x, lineup_y = forced_exit
                self._pub_str(
                    self.event_pub,
                    f"GO_PANTRY_FORCED_HIGHWAY_EXIT {pantry_id} {lineup_x:.1f} {lineup_y:.1f}"
                )
            else:
                lineup_x, lineup_y = self._nearest_highway_point(final_axis_x, final_axis_y)

        waypoints = self._build_highway_waypoints(lineup_x, lineup_y, final_yaw)
        if not waypoints:
            waypoints = [(lineup_x, lineup_y, final_yaw)]
        else:
            waypoints[-1] = (lineup_x, lineup_y, final_yaw)

        if start_safe:
            if math.hypot(final_axis_x - waypoints[-1][0], final_axis_y - waypoints[-1][1]) > 5.0:
                waypoints.append((final_axis_x, final_axis_y, final_yaw))
            else:
                waypoints[-1] = (final_axis_x, final_axis_y, final_yaw)
            fixed_yaw_indices.add(len(waypoints) - 1)
        else:
            backup_x = lineup_x - self.PANTRY_APPROACH_BACKUP_MM * fwd_x
            backup_y = lineup_y - self.PANTRY_APPROACH_BACKUP_MM * fwd_y
            backup_x, backup_y, clamped = self._clamp_goal_to_bounds(backup_x, backup_y, final_yaw, 5.0)
            if clamped:
                self._pub_str(self.event_pub, "WARN GO_PANTRY_BACKUP_CLAMPED_TO_BOUNDS")

            if math.hypot(backup_x - waypoints[-1][0], backup_y - waypoints[-1][1]) > 5.0:
                waypoints.append((backup_x, backup_y, final_yaw))
                fixed_yaw_indices.add(len(waypoints) - 1)

            if math.hypot(final_axis_x - waypoints[-1][0], final_axis_y - waypoints[-1][1]) > 5.0:
                waypoints.append((final_axis_x, final_axis_y, final_yaw))
            else:
                waypoints[-1] = (final_axis_x, final_axis_y, final_yaw)
            fixed_yaw_indices.add(len(waypoints) - 1)

        self.path_active = True
        self.path_waypoints = waypoints
        self.path_index = 0
        self.path_smooth_indices = set(getattr(self, "_pending_smooth_indices", set()))
        self.path_fixed_yaw_indices = fixed_yaw_indices
        self.path_force_reverse_indices = set()
        self.path_final_label = f"GO_PANTRY_{pantry_id}"
        self.path_final_ref = RefPoint.DUSTPAN
        self.path_auto_dustpan = True
        self.path_start_x = self.x_mm
        self.path_start_y = self.y_mm
        self.goal_ref = RefPoint.DUSTPAN

        self._dustpan_up()  # Raise dustpan for highway driving
        self._carwash_stow()  # Stow carwash arm
        self._dustpan_lowered_time = None  # Reset for next approach
        self._pub_str(self.ack_pub, f"GO_PANTRY_{pantry_id}")
        self._pub_str(
            self.event_pub,
            f"GO_PANTRY_REQUEST {pantry_id} "
            f"base={base_x:.1f},{base_y:.1f} "
            f"offset={target_off_x:.1f},{target_off_y:.1f} "
            f"target={target_x:.1f},{target_y:.1f} "
            f"lineup={lineup_x:.1f},{lineup_y:.1f} "
            f"axis={final_axis_x:.1f},{final_axis_y:.1f} "
            f"yaw={final_yaw_deg:.1f} "
            f"wps={len(waypoints)}"
        )
        self._pub_str(self.event_pub, f"PATH_START GO_PANTRY_{pantry_id} {len(waypoints)}")
        self._start_next_path_waypoint()

    def _go_home_cb(self, _msg):
        if self.USE_HIGHWAY_PLANNER and self.USE_HIGHWAY_FOR_HOME:
            # Use the same highway path system as normal GO commands.
            # The target is the home CENTER pose; _set_new_goal converts that
            # to the wheel-axis goal internally.
            self._set_new_goal(self.home_x, self.home_y, self.home_yaw_deg,
                               RefPoint.CENTER, "GO_HOME")
            self._pub_str(self.event_pub, "GO_HOME_HIGHWAY_STARTED")
            return

        # Fallback to the old special home routine.
        self._go_home()
        self._pub_str(self.ack_pub, "GO_HOME")
        self._pub_str(self.event_pub, "GO_HOME_STARTED")

    def _stop_cb(self, _msg):
        self._abort_drive()
        self._pub_str(self.ack_pub, "STOP")
        self._pub_str(self.event_pub, "STOPPED")

    def _estop_cb(self, _msg):
        self._abort_drive(immediate_stop=True)
        self._pub_str(self.ack_pub, "ESTOP")
        self._pub_str(self.event_pub, "ESTOP")

    def _flip_cb(self, msg: Int32):
        self._pub_str(self.ack_pub, f"FLIP {msg.data}")
        self._pub_str(self.done_pub, f"FLIP {msg.data}")

    def _flip_seq_cb(self, msg: Int32MultiArray):
        csv = ",".join(str(int(v)) for v in msg.data)
        self._pub_str(self.ack_pub, f"FLIP_SEQ {csv}")
        self._pub_str(self.done_pub, f"FLIP_SEQ {csv}")

    def _set_pattern_cb(self, msg: String):
        self._pub_str(self.ack_pub, f"SET_PATTERN {msg.data}")
        self._pub_str(self.done_pub, f"SET_PATTERN {msg.data}")

    def _set_bricks_cb(self, msg: String):
        parts = msg.data.strip().upper().replace(",", " ").split()
        if len(parts) == 4:
            self.brick_state = list(parts)
        self._pub_str(self.ack_pub, f"SET_BRICKS {msg.data}")
        self._pub_str(self.event_pub, f"BRICKS {' '.join(self.brick_state)}")

    def _reset_odom_cb(self, msg: Pose2D):
        # In sim, odom comes from Gazebo — we can't truly reset it.
        # Just acknowledge.
        self._pub_str(self.ack_pub, "RESET_ODOM")

    def _get_state_cb(self, _msg):
        self._pub_str(self.ack_pub, "GET_STATE")
        self._send_gains()

    def _telem_hz_cb(self, msg: Int32):
        self._pub_str(self.ack_pub, f"TELEM {msg.data}")

    def _raw_cb(self, msg: String):
        text = msg.data.strip().upper()
        self._pub_str(self.raw_line_pub, f"SIM_RAW: {text}")

        if text == "GET_GAINS":
            self._pub_str(self.ack_pub, "GET_GAINS")
            self._send_gains()
        elif text == "RELOAD_CONFIG":
            self._load_control_config()
            self._refresh_derived_tuning_values()
            self._pub_str(self.ack_pub, "RELOAD_CONFIG")
            self._send_gains()
        elif text in ("STRATEGY 1", "STRATEGY_1", "STRATEGY1"):
            self._strategy_start("strategy_1")
        elif text.startswith("STRATEGY "):
            name = "strategy_" + text.split(maxsplit=1)[1].strip().lower().replace(" ", "_")
            self._strategy_start(name)
        elif text.startswith("SET_MAX_LINEAR "):
            try:
                self.MAX_LINEAR_MM_S = max(0.0, float(text.split()[1]))
                self._pub_str(self.ack_pub, "SET_MAX_LINEAR")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_MAX_REVERSE "):
            try:
                self.MAX_REVERSE_MM_S = max(0.0, float(text.split()[1]))
                self._pub_str(self.ack_pub, "SET_MAX_REVERSE")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_HIGHWAY_SPEED "):
            try:
                self.HIGHWAY_SPEED_MM_S = max(0.0, float(text.split()[1]))
                self._pub_str(self.ack_pub, "SET_HIGHWAY_SPEED")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_HIGHWAY_ENTRY_SPEED "):
            try:
                self.HIGHWAY_ENTRY_EXIT_SPEED_MM_S = max(0.0, float(text.split()[1]))
                self._pub_str(self.ack_pub, "SET_HIGHWAY_ENTRY_SPEED")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_HIGHWAY_TURN_SPEED "):
            try:
                self.HIGHWAY_TURN_SPEED_MM_S = max(0.0, float(text.split()[1]))
                self._pub_str(self.ack_pub, "SET_HIGHWAY_TURN_SPEED")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_SMOOTH_AGGRESSIVE_DEG "):
            try:
                self.HIGHWAY_SMOOTH_TURN_AGGRESSIVE = deg2rad(max(0.0, float(text.split()[1])))
                self._pub_str(self.ack_pub, "SET_SMOOTH_AGGRESSIVE_DEG")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_SMOOTH_CONSERVATIVE_DEG "):
            try:
                self.HIGHWAY_SMOOTH_TURN_CONSERVATIVE = deg2rad(max(0.0, float(text.split()[1])))
                self._pub_str(self.ack_pub, "SET_SMOOTH_CONSERVATIVE_DEG")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_LINEAR_ACCEL "):
            try:
                self.LINEAR_ACCEL_MM_S2 = max(0.0, float(text.split()[1]))
                self._pub_str(self.ack_pub, "SET_LINEAR_ACCEL")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_LINEAR_DECEL "):
            try:
                self.LINEAR_DECEL_MM_S2 = max(0.0, float(text.split()[1]))
                self._pub_str(self.ack_pub, "SET_LINEAR_DECEL")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_YAW_ACCEL "):
            try:
                self.YAW_ACCEL_RAD_S2 = max(0.0, float(text.split()[1]))
                self._pub_str(self.ack_pub, "SET_YAW_ACCEL")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_YAW_DECEL "):
            try:
                self.YAW_DECEL_RAD_S2 = max(0.0, float(text.split()[1]))
                self._pub_str(self.ack_pub, "SET_YAW_DECEL")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_DUSTPAN_SPEED "):
            try:
                self.DUSTPAN_MOVE_SPEED_RAD_S = max(0.0, float(text.split()[1]))
                self._pub_str(self.ack_pub, "SET_DUSTPAN_SPEED")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_CARWASH_ARM_SPEED "):
            try:
                self.CARWASH_ARM_MOVE_SPEED_RAD_S = max(0.0, float(text.split()[1]))
                self._pub_str(self.ack_pub, "SET_CARWASH_ARM_SPEED")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_ROLLER_ACCEL "):
            try:
                self.CARWASH_ROLLER_ACCEL_RAD_S2 = max(0.0, float(text.split()[1]))
                self._pub_str(self.ack_pub, "SET_ROLLER_ACCEL")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_K_RHO "):
            try:
                self.K_rho = float(text.split()[1])
                self._pub_str(self.ack_pub, "SET_K_RHO")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_K_ALPHA "):
            try:
                self.K_alpha = float(text.split()[1])
                self._pub_str(self.ack_pub, "SET_K_ALPHA")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_K_ALPHA_I "):
            try:
                self.K_alpha_i = float(text.split()[1])
                self._pub_str(self.ack_pub, "SET_K_ALPHA_I")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_K_YAW "):
            try:
                self.FINAL_YAW_K = float(text.split()[1])
                self._pub_str(self.ack_pub, "SET_K_YAW")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_POS_TOL "):
            try:
                self.POS_TOL_MM = float(text.split()[1])
                self._pub_str(self.ack_pub, "SET_POS_TOL")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_YAW_TOL_DEG "):
            try:
                self.FINAL_YAW_TOL = deg2rad(float(text.split()[1]))
                self._pub_str(self.ack_pub, "SET_YAW_TOL_DEG")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_YAW_MIN "):
            try:
                self.FINAL_YAW_MIN = float(text.split()[1])
                self._pub_str(self.ack_pub, "SET_YAW_MIN")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text.startswith("SET_YAW_MAX "):
            try:
                self.FINAL_YAW_MAX = float(text.split()[1])
                self._pub_str(self.ack_pub, "SET_YAW_MAX")
                self._send_gains()
            except (IndexError, ValueError):
                pass
        elif text == "PUSH_OUT":
            self._pub_str(self.ack_pub, "PUSH_OUT")
            self._pub_str(self.done_pub, "PUSH_OUT")
        elif text == "PULL_IN":
            self._pub_str(self.ack_pub, "PULL_IN")
            self._pub_str(self.done_pub, "PULL_IN")
        elif text == "CARWASH_SPIN_POSITIVE":
            self._carwash_arm_down()
            self._carwash_spin(self.CARWASH_ROLLER_PULL)
            self._pub_str(self.ack_pub, text)
        elif text == "CARWASH_SPIN_NEGATIVE":
            self._carwash_arm_down()
            self._carwash_spin(self.CARWASH_ROLLER_PUSH)
            self._pub_str(self.ack_pub, text)
        elif text == "CARWASH_SPIN_STOP":
            self._carwash_stow()
            self._pub_str(self.ack_pub, text)
        elif text.startswith("CARWASH_ARM_"):
            try:
                angle = float(text.split()[-1])
                self.carwash_arm_target_angle = deg2rad(angle)
                self._pub_str(self.ack_pub, text)
            except (IndexError, ValueError):
                self._pub_str(self.ack_pub, text)
        elif text.startswith("CARWASH_"):
            self._pub_str(self.ack_pub, text)
        elif text == "STOP":
            self._stop_cb(Empty())
            self._mission_abort()
        elif text == "ESTOP":
            self._estop_cb(Empty())
            self._mission_abort()


    # ══════════════════════════════════════════════════════════
    #  Config-driven strategy sequencer
    # ══════════════════════════════════════════════════════════

    def _strategy_step_action(self, step):
        if not isinstance(step, dict):
            return ""
        return str(step.get("action", step.get("type", ""))).strip().lower()

    def _strategy_start(self, name: str):
        name = str(name).strip().lower().replace(" ", "_")
        strategy = self.STRATEGIES.get(name)
        if not isinstance(strategy, dict):
            self._pub_str(self.error_pub, f"UNKNOWN_STRATEGY {name}")
            return
        steps = strategy.get("steps")
        if not isinstance(steps, list) or not steps:
            self._pub_str(self.error_pub, f"EMPTY_STRATEGY {name}")
            return
        if not self.odom_valid:
            self._pub_str(self.error_pub, f"{name.upper()}_WAITING_FOR_POSE")
            return

        # Stop any normal mission, then start the strategy. _abort_drive()
        # intentionally clears old drive state; strategy state is set after it.
        self._mission_abort()
        self._abort_drive(immediate_stop=True)

        self.strategy_active = True
        self.strategy_name = name
        self.strategy_steps = list(steps)
        self.strategy_index = 0
        self.strategy_waiting_drive_label = None
        self.strategy_timer_until = None
        self.strategy_timer_kind = None
        self.strategy_total_start_time = time.monotonic()
        finish_cfg = strategy.get("finish", {})
        self.strategy_finish_behavior = dict(finish_cfg) if isinstance(finish_cfg, dict) else {}

        self._pub_str(self.ack_pub, name.upper())
        self._pub_str(self.event_pub, f"STRATEGY_START {name} steps={len(self.strategy_steps)}")
        self._strategy_start_next_step()

    def _strategy_finish(self):
        elapsed_s = 0.0
        if self.strategy_total_start_time is not None:
            elapsed_s = time.monotonic() - self.strategy_total_start_time
        name = self.strategy_name or "strategy"
        self.strategy_active = False
        self.strategy_name = ""
        self.strategy_steps = []
        self.strategy_index = 0
        self.strategy_waiting_drive_label = None
        self.strategy_timer_until = None
        self.strategy_timer_kind = None
        self.strategy_total_start_time = None

        finish_cfg = getattr(self, "strategy_finish_behavior", {})
        if not isinstance(finish_cfg, dict):
            finish_cfg = {}
        carwash_finish = str(finish_cfg.get("carwash", "stow")).strip().lower()
        dustpan_finish = str(finish_cfg.get("dustpan", "up")).strip().lower()

        if carwash_finish in ("stop", "roller_stop", "keep_arm"):
            self._carwash_spin(self.CARWASH_ROLLER_STOP)
        else:
            self._carwash_stow()

        if dustpan_finish in ("down", "keep_down"):
            self._dustpan_down()
        elif dustpan_finish in ("keep", "unchanged"):
            pass
        else:
            self._dustpan_up()

        self.strategy_finish_behavior = {}
        self._stop_motors()
        self._pub_str(self.event_pub, f"STRATEGY_COMPLETE {name} elapsed_s={elapsed_s:.3f}")
        self._pub_str(self.done_pub, f"STRATEGY_COMPLETE {name} elapsed_s={elapsed_s:.3f}")

    def _strategy_timer_start(self, duration_s: float, kind: str):
        self.strategy_timer_until = time.monotonic() + max(0.0, float(duration_s))
        self.strategy_timer_kind = kind
        self._pub_str(self.event_pub, f"STRATEGY_TIMER {kind} {duration_s:.3f}s")

    def _strategy_tick(self):
        if not self.strategy_active:
            return
        if self.strategy_timer_until is None:
            return
        if time.monotonic() < self.strategy_timer_until:
            return

        kind = self.strategy_timer_kind or "wait"
        self.strategy_timer_until = None
        self.strategy_timer_kind = None

        if kind in ("pull", "push"):
            self._carwash_stow()
            if kind == "pull":
                self._dustpan_up()
        elif kind == "dustpan_down":
            # Leave the dustpan down for the next step.
            pass

        self._pub_str(self.event_pub, f"STRATEGY_TIMER_DONE {kind}")
        self.strategy_index += 1
        self._strategy_start_next_step()

    def _strategy_start_next_step(self):
        if not self.strategy_active:
            return
        if self.strategy_index >= len(self.strategy_steps):
            self._strategy_finish()
            return

        step = self.strategy_steps[self.strategy_index]
        if not isinstance(step, dict):
            self._pub_str(self.error_pub, f"STRATEGY_BAD_STEP index={self.strategy_index}")
            self.strategy_active = False
            return

        action = self._strategy_step_action(step)
        self._pub_str(
            self.event_pub,
            f"STRATEGY_STEP {self.strategy_index + 1}/{len(self.strategy_steps)} {action}"
        )

        if action in ("wait", "settle"):
            self._strategy_timer_start(float(step.get("duration_s", self.MISSION_SETTLE_TIME)), "wait")
            return

        if action in ("pull", "carwash_pull"):
            self._carwash_arm_down()
            self._carwash_spin(self.CARWASH_ROLLER_PULL)
            self._strategy_timer_start(float(step.get("duration_s", self.MISSION_CARWASH_PULL_TIME)), "pull")
            return

        if action in ("push", "carwash_push"):
            self._carwash_arm_down()
            self._carwash_spin(self.CARWASH_ROLLER_PUSH)
            self._strategy_timer_start(float(step.get("duration_s", self.MISSION_CARWASH_PUSH_TIME)), "push")
            return

        if action in ("roller_push_start", "carwash_push_start"):
            self._carwash_arm_down()
            self._carwash_spin(self.CARWASH_ROLLER_PUSH)
            self.strategy_index += 1
            self._strategy_start_next_step()
            return

        if action in ("roller_stop", "carwash_stop"):
            self._carwash_stow()
            self.strategy_index += 1
            self._strategy_start_next_step()
            return

        if action == "dustpan_down":
            self._dustpan_down()
            self._strategy_timer_start(float(step.get("settle_s", self.DUSTPAN_LOWER_SETTLE_S)), "dustpan_down")
            return

        if action == "dustpan_up":
            self._dustpan_up()
            self.strategy_index += 1
            self._strategy_start_next_step()
            return

        if action == "zone":
            self._strategy_start_zone_step(step)
            return

        if action == "pantry":
            self._strategy_start_pantry_step(step)
            return

        if action in ("drive_path", "path"):
            self._strategy_start_drive_path_step(step)
            return

        if action == "home":
            self._strategy_start_home_step(step)
            return

        self._pub_str(self.error_pub, f"STRATEGY_UNKNOWN_ACTION {action}")
        self.strategy_active = False

    def _strategy_ref_from_text(self, ref_text):
        ref = str(ref_text or "axis").strip().lower()
        if ref == "center":
            return RefPoint.CENTER
        if ref == "dustpan":
            return RefPoint.DUSTPAN
        return RefPoint.AXIS

    def _strategy_axis_pose(self, x_mm, y_mm, yaw_deg, ref_text="axis"):
        yaw = wrap(deg2rad(float(yaw_deg)))
        ref = self._strategy_ref_from_text(ref_text)
        offset = self._offset_for_ref(ref)
        return (
            float(x_mm) - offset * math.cos(yaw),
            float(y_mm) - offset * math.sin(yaw),
            yaw,
        )

    def _strategy_start_axis_path(
        self,
        waypoints,
        label,
        fixed_yaw_indices=None,
        force_reverse_indices=None,
        final_ref=RefPoint.AXIS,
        auto_dustpan=False,
    ):
        if not waypoints:
            self._pub_str(self.error_pub, f"STRATEGY_EMPTY_PATH {label}")
            self.strategy_active = False
            return

        self.path_active = True
        self.path_waypoints = list(waypoints)
        self.path_index = 0
        self.path_smooth_indices = set()
        self.path_fixed_yaw_indices = set(fixed_yaw_indices or set())
        self.path_force_reverse_indices = set(force_reverse_indices or set())
        self._pending_smooth_indices = set()
        self.path_final_label = label
        self.path_final_ref = final_ref
        self.path_auto_dustpan = bool(auto_dustpan)
        self.path_start_x = self.x_mm
        self.path_start_y = self.y_mm
        self.goal_ref = final_ref
        self.strategy_waiting_drive_label = label
        self._dustpan_lowered_time = None

        self._pub_str(self.event_pub, f"PATH_START {label} {len(waypoints)} strategy_direct=1")
        self._start_next_path_waypoint()

    def _strategy_start_zone_step(self, step):
        zone_id = int(step.get("id", step.get("zone", 0)))
        if zone_id not in self.JENGA_ZONES:
            self._pub_str(self.error_pub, f"STRATEGY_INVALID_ZONE {zone_id}")
            self.strategy_active = False
            return

        base_x, base_y = self.JENGA_ZONES[zone_id]
        target_x, target_y, target_off_x, target_off_y = self._target_with_offset(
            base_x, base_y, self.JENGA_ZONE_TARGET_OFFSETS_MM, zone_id, step
        )
        yaw_deg = float(step.get("yaw_deg", self.JENGA_ZONE_YAWS_DEG[zone_id]))
        yaw = wrap(deg2rad(yaw_deg))
        fwd_x, fwd_y = math.cos(yaw), math.sin(yaw)
        offset = self._offset_for_ref(RefPoint.DUSTPAN)
        final_axis_x = target_x - offset * fwd_x
        final_axis_y = target_y - offset * fwd_y
        start_dist = max(0.0, float(step.get("start_mm", self.JENGA_APPROACH_START_MM)))
        start_axis_x = final_axis_x - start_dist * fwd_x
        start_axis_y = final_axis_y - start_dist * fwd_y

        label = str(step.get("label", f"GO_ZONE_{zone_id}_STRATEGY"))
        self._pub_str(
            self.event_pub,
            f"STRATEGY_ZONE_TARGET {zone_id} "
            f"base={base_x:.1f},{base_y:.1f} "
            f"offset={target_off_x:.1f},{target_off_y:.1f} "
            f"target={target_x:.1f},{target_y:.1f}"
        )
        waypoints = [(start_axis_x, start_axis_y, yaw), (final_axis_x, final_axis_y, yaw)]
        self._dustpan_up()
        self._carwash_stow()
        self._strategy_start_axis_path(
            waypoints,
            label,
            fixed_yaw_indices={1},
            final_ref=RefPoint.DUSTPAN,
            auto_dustpan=True,
        )

    def _strategy_start_pantry_step(self, step):
        pantry_id = int(step.get("id", step.get("pantry", 0)))
        if pantry_id not in self.PANTRY_LOCATIONS:
            self._pub_str(self.error_pub, f"STRATEGY_INVALID_PANTRY {pantry_id}")
            self.strategy_active = False
            return

        base_x, base_y = self.PANTRY_LOCATIONS[pantry_id]
        target_x, target_y, target_off_x, target_off_y = self._target_with_offset(
            base_x, base_y, self.PANTRY_TARGET_OFFSETS_MM, pantry_id, step
        )

        # IMPORTANT:
        # The loaded pantry yaw map is already mirrored when team.color=blue.
        # But a strategy step can override yaw_deg directly in YAML. Those
        # explicit strategy yaw values are authored for yellow, so they must
        # be mirrored here.
        if "yaw_deg" in step:
            raw_yaw_deg = float(step["yaw_deg"])
            yaw_deg = self._mirror_yaw_deg_for_team(raw_yaw_deg)
        else:
            raw_yaw_deg = float(self.PANTRY_APPROACH_YAWS_DEG[pantry_id])
            yaw_deg = raw_yaw_deg

        yaw = wrap(deg2rad(yaw_deg))
        fwd_x, fwd_y = math.cos(yaw), math.sin(yaw)

        pantry_half = float(step.get("half_size_mm", self.PANTRY_HALF_SIZE_MM))

        # Stop so dustpan tip reaches the pantry edge, not the pantry center.
        edge_x = target_x - pantry_half * fwd_x
        edge_y = target_y - pantry_half * fwd_y

        # Convert dustpan edge target into wheel-axis target.
        offset = self._offset_for_ref(RefPoint.DUSTPAN)
        final_axis_x = edge_x - offset * fwd_x
        final_axis_y = edge_y - offset * fwd_y

        start_dist = max(0.0, float(step.get("start_mm", self.PANTRY_APPROACH_START_MM)))
        start_axis_x = final_axis_x - start_dist * fwd_x
        start_axis_y = final_axis_y - start_dist * fwd_y

        label = str(step.get("label", f"GO_PANTRY_{pantry_id}_STRATEGY"))

        approach_side = "left" if start_axis_x < target_x else "right"
        self._pub_str(
            self.event_pub,
            f"STRATEGY_PANTRY_TARGET {pantry_id} "
            f"team={getattr(self, 'TEAM_COLOR', 'yellow')} "
            f"base={base_x:.1f},{base_y:.1f} "
            f"offset={target_off_x:.1f},{target_off_y:.1f} "
            f"target={target_x:.1f},{target_y:.1f} "
            f"raw_yaw={raw_yaw_deg:.1f} "
            f"yaw={yaw_deg:.1f} "
            f"approach_side={approach_side} "
            f"start_axis={start_axis_x:.1f},{start_axis_y:.1f} "
            f"final_axis={final_axis_x:.1f},{final_axis_y:.1f}"
        )

        waypoints = [(start_axis_x, start_axis_y, yaw), (final_axis_x, final_axis_y, yaw)]

        self._dustpan_up()
        self._carwash_stow()
        self._strategy_start_axis_path(
            waypoints,
            label,
            fixed_yaw_indices={1},
            final_ref=RefPoint.DUSTPAN,
            auto_dustpan=True,
        )

    def _strategy_start_drive_path_step(self, step):
        raw_wps = step.get("waypoints", [])
        if not isinstance(raw_wps, list) or not raw_wps:
            self._pub_str(self.error_pub, "STRATEGY_DRIVE_PATH_NEEDS_WAYPOINTS")
            self.strategy_active = False
            return

        label = str(step.get("label", f"STRATEGY_PATH_{self.strategy_index + 1}"))
        default_yaw = float(step.get("yaw_deg", 0.0))
        default_ref = step.get("ref", "axis")
        force_reverse = parse_bool(step.get("reverse", False))
        fixed_yaw = parse_bool(step.get("fixed_yaw", False)) or force_reverse
        waypoints = []
        fixed = set()
        reverse = set()

        for i, wp in enumerate(raw_wps):
            if not isinstance(wp, dict):
                self._pub_str(self.error_pub, f"STRATEGY_BAD_WAYPOINT {label} index={i}")
                self.strategy_active = False
                return
            x = wp.get("x_mm", wp.get("x"))
            y = wp.get("y_mm", wp.get("y"))
            if x is None or y is None:
                self._pub_str(self.error_pub, f"STRATEGY_WAYPOINT_MISSING_XY {label} index={i}")
                self.strategy_active = False
                return
            yaw_deg = float(wp.get("yaw_deg", default_yaw))
            if self._team_is_blue():
                x, y = self._mirror_point_for_team(float(x), float(y))
                yaw_deg = self._mirror_yaw_deg_for_team(yaw_deg)
            ref = wp.get("ref", default_ref)
            waypoints.append(self._strategy_axis_pose(x, y, yaw_deg, ref))
            wp_reverse = force_reverse or parse_bool(wp.get("reverse", False))
            if fixed_yaw or parse_bool(wp.get("fixed_yaw", False)) or wp_reverse:
                fixed.add(i)
            if wp_reverse:
                reverse.add(i)

        final_ref = self._strategy_ref_from_text(raw_wps[-1].get("ref", default_ref))
        self._strategy_start_axis_path(
            waypoints,
            label,
            fixed_yaw_indices=fixed,
            force_reverse_indices=reverse,
            final_ref=final_ref,
            auto_dustpan=parse_bool(step.get("auto_dustpan", False)),
        )

    def _strategy_start_home_step(self, step):
        has_explicit_home_pose = any(k in step for k in ("x_mm", "x", "y_mm", "y", "yaw_deg"))
        yaw_deg = float(step.get("yaw_deg", self.home_yaw_deg))
        x_mm = float(step.get("x_mm", step.get("x", self.home_x)))
        y_mm = float(step.get("y_mm", step.get("y", self.home_y)))
        if has_explicit_home_pose and self._team_is_blue():
            x_mm, y_mm = self._mirror_point_for_team(x_mm, y_mm)
            yaw_deg = self._mirror_yaw_deg_for_team(yaw_deg)
        label = str(step.get("label", "STRATEGY_HOME"))
        waypoint = self._strategy_axis_pose(x_mm, y_mm, yaw_deg, "center")
        self._dustpan_up()
        self._carwash_stow()
        self._strategy_start_axis_path([waypoint], label, fixed_yaw_indices=set(), final_ref=RefPoint.CENTER)

    # ══════════════════════════════════════════════════════════
    #  Mission sequencer — automates zone pickup → pantry delivery
    #
    #  States:
    #    IDLE                  → waiting for mission command
    #    GOING_TO_ZONE         → driving to zone (waiting for DONE)
    #    SETTLING_AT_ZONE      → short pause after arriving at zone
    #    PULLING_PIECES        → carwash pulling pieces in
    #    STOWING_AFTER_PULL    → carwash stop, dustpan up, short pause
    #    GOING_TO_PANTRY       → driving to pantry (waiting for DONE)
    #    SETTLING_AT_PANTRY    → short pause after arriving at pantry
    #    PUSHING_PIECES        → carwash pushing pieces out
    #    STOWING_AFTER_PUSH    → carwash stop, short pause
    #    BACKING_AWAY          → reverse away from pantry
    #    MISSION_DONE          → complete, check queue for next
    # ══════════════════════════════════════════════════════════

    def _mission_cb(self, msg: String):
        """Parse mission command: 'zone_id pantry_id' or 'zone_id pantry_id zone_id pantry_id ...'"""
        parts = msg.data.strip().split()
        if len(parts) < 2 or len(parts) % 2 != 0:
            self._pub_str(self.error_pub, f"MISSION_BAD_FORMAT: use 'zone pantry [zone pantry ...]'")
            return

        pairs = []
        for i in range(0, len(parts), 2):
            try:
                z = int(parts[i])
                p = int(parts[i + 1])
                if z not in self.JENGA_ZONES:
                    self._pub_str(self.error_pub, f"INVALID_ZONE {z}")
                    return
                if p not in self.PANTRY_LOCATIONS:
                    self._pub_str(self.error_pub, f"INVALID_PANTRY {p}")
                    return
                pairs.append((z, p))
            except ValueError:
                self._pub_str(self.error_pub, f"MISSION_BAD_FORMAT: '{parts[i]}' '{parts[i+1]}'")
                return

        self.mission_queue = list(pairs)

        self.mission_total_start_time = time.monotonic()
        self.mission_total_task_count = len(pairs)

        self._pub_str(self.ack_pub, f"MISSION {len(pairs)} tasks queued")
        self._pub_str(self.event_pub, f"MISSION_START {pairs}")
        self._pub_str(self.event_pub, f"MISSION_TIMER_START tasks={len(pairs)}")
        self._mission_start_next()

    def _mission_start_next(self):
        """Start the next zone→pantry pair from the queue."""
        if not self.mission_queue:
            self.mission_active = False
            self.mission_state = "IDLE"

            if self.mission_total_start_time is not None:
                elapsed_s = time.monotonic() - self.mission_total_start_time
            else:
                elapsed_s = 0.0

            done_text = (
                f"MISSION_ALL_COMPLETE elapsed_s={elapsed_s:.3f} "
                f"tasks={self.mission_total_task_count}"
            )

            self._pub_str(self.event_pub, done_text)
            self._pub_str(self.done_pub, done_text)

            self.mission_total_start_time = None
            self.mission_total_task_count = 0
            return

        self.mission_zone_id, self.mission_pantry_id = self.mission_queue.pop(0)
        self.mission_active = True
        self.mission_state = "GOING_TO_ZONE"

        self._pub_str(self.event_pub,
                      f"MISSION_TASK zone={self.mission_zone_id} pantry={self.mission_pantry_id} "
                      f"remaining={len(self.mission_queue)}")

        # Trigger go_zone
        zone_msg = Int32()
        zone_msg.data = self.mission_zone_id
        self._go_zone_cb(zone_msg)

    def _mission_abort(self):
        """Cancel the current mission."""
        if self.mission_active:
            self.mission_active = False
            self.mission_state = "IDLE"
            self.mission_queue = []
            self._carwash_stow()

            if self.mission_total_start_time is not None:
                elapsed_s = time.monotonic() - self.mission_total_start_time
                self._pub_str(self.event_pub, f"MISSION_ABORTED elapsed_s={elapsed_s:.3f}")
            else:
                self._pub_str(self.event_pub, "MISSION_ABORTED")

            self.mission_total_start_time = None
            self.mission_total_task_count = 0

    def _mission_tick(self):
        """Called from _control_loop at 200Hz. Drives the mission state machine."""
        if not self.mission_active:
            return

        now = time.monotonic()

        if self.mission_state == "GOING_TO_ZONE":
            # Waiting for the zone drive to complete.
            # _finish_current_goal publishes DONE which we check below.
            pass

        elif self.mission_state == "SETTLING_AT_ZONE":
            if now - self.mission_timer_start >= self.MISSION_SETTLE_TIME:
                self.mission_state = "PULLING_PIECES"
                self.mission_timer_start = now
                self._carwash_arm_down()
                self._carwash_spin(self.CARWASH_ROLLER_PULL)
                self._pub_str(self.event_pub, "MISSION_CARWASH_PULL_START")

        elif self.mission_state == "PULLING_PIECES":
            if now - self.mission_timer_start >= self.MISSION_CARWASH_PULL_TIME:
                self.mission_state = "STOWING_AFTER_PULL"
                self.mission_timer_start = now
                self._carwash_stow()
                self._dustpan_up()
                self._pub_str(self.event_pub, "MISSION_CARWASH_PULL_DONE")

        elif self.mission_state == "STOWING_AFTER_PULL":
            if now - self.mission_timer_start >= self.MISSION_SETTLE_TIME:
                self.mission_state = "GOING_TO_PANTRY"
                self._pub_str(self.event_pub,
                              f"MISSION_GOING_TO_PANTRY {self.mission_pantry_id}")
                pantry_msg = Int32()
                pantry_msg.data = self.mission_pantry_id
                self._go_pantry_cb(pantry_msg)

        elif self.mission_state == "GOING_TO_PANTRY":
            # Waiting for pantry drive to complete
            pass

        elif self.mission_state == "SETTLING_AT_PANTRY":
            if now - self.mission_timer_start >= self.MISSION_SETTLE_TIME:
                self.mission_state = "PUSHING_PIECES"
                self.mission_timer_start = now
                self._carwash_arm_down()
                self._carwash_spin(self.CARWASH_ROLLER_PUSH)
                self._pub_str(self.event_pub, "MISSION_CARWASH_PUSH_START")

        elif self.mission_state == "PUSHING_PIECES":
            if now - self.mission_timer_start >= self.MISSION_CARWASH_PUSH_TIME:
                self.mission_state = "STOWING_AFTER_PUSH"
                self.mission_timer_start = now
                self._carwash_stow()
                self._pub_str(self.event_pub, "MISSION_CARWASH_PUSH_DONE")

        elif self.mission_state == "STOWING_AFTER_PUSH":
            if now - self.mission_timer_start >= self.MISSION_SETTLE_TIME:
                self.mission_state = "BACKING_AWAY"
                self.mission_timer_start = now
                self._dustpan_up()
                self._pub_str(self.event_pub, "MISSION_BACKING_AWAY")

        elif self.mission_state == "BACKING_AWAY":
            # Reverse slowly straight back for a fixed time (no turning!)
            elapsed = now - self.mission_timer_start
            backup_speed = max(1.0, abs(self.MISSION_BACKUP_SPEED_MM_S))
            backup_time = self.MISSION_BACKUP_DIST_MM / backup_speed
            if elapsed < backup_time:
                self._drive_vw(-backup_speed, 0.0)  # reverse straight, no rotation
            else:
                self._stop_motors()
                self.mission_state = "MISSION_DONE"
                self._pub_str(self.event_pub, "MISSION_BACKUP_DONE")

        elif self.mission_state == "MISSION_DONE":
            self._pub_str(self.event_pub,
                          f"MISSION_TASK_COMPLETE zone={self.mission_zone_id} "
                          f"pantry={self.mission_pantry_id}")
            self._mission_start_next()

    def _send_gains(self):
        s = (f"K_RHO={self.K_rho:.4f} K_ALPHA={self.K_alpha:.4f} "
             f"K_ALPHA_I={self.K_alpha_i:.4f} K_YAW={self.FINAL_YAW_K:.4f} "
             f"POS_TOL_MM={self.POS_TOL_MM:.2f} "
             f"YAW_TOL_DEG={rad2deg(self.FINAL_YAW_TOL):.2f} "
             f"YAW_MIN={self.FINAL_YAW_MIN:.4f} YAW_MAX={self.FINAL_YAW_MAX:.4f} "
             f"MAX_LINEAR={self.MAX_LINEAR_MM_S:.1f} MAX_REVERSE={self.MAX_REVERSE_MM_S:.1f} "
             f"HIGHWAY_SPEED={self.HIGHWAY_SPEED_MM_S:.1f} "
             f"SMOOTH_AGG={rad2deg(self.HIGHWAY_SMOOTH_TURN_AGGRESSIVE):.0f} "
             f"SMOOTH_CON={rad2deg(self.HIGHWAY_SMOOTH_TURN_CONSERVATIVE):.0f} "
             f"RAMP_LINEAR_ACCEL={self.LINEAR_ACCEL_MM_S2:.1f} "
             f"RAMP_LINEAR_DECEL={self.LINEAR_DECEL_MM_S2:.1f} "
             f"DUSTPAN_SPEED={self.DUSTPAN_MOVE_SPEED_RAD_S:.2f} "
             f"CARWASH_ARM_SPEED={self.CARWASH_ARM_MOVE_SPEED_RAD_S:.2f} "
             f"ROLLER_ACCEL={self.CARWASH_ROLLER_ACCEL_RAD_S2:.1f}")
        self._pub_str(self.gains_pub, s)

    # ── Pub helpers ───────────────────────────────────────────
    def _pub_str(self, pub, text: str):
        m = String()
        m.data = text
        pub.publish(m)

    def _pub_bool(self, pub, val: bool):
        m = Bool()
        m.data = val
        pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = OpenCRBridgeSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()