#!/usr/bin/env python3
"""
position_verifier_node.py

Small screenshot-friendly ROS 2 terminal monitor for verifying Ninja overhead pose
and navigation accuracy during Eurobot 2026 thesis tests.

It does not command the robot. It only listens to:
  /ninja/pose
  /ninja/goal_pose
  /ninja/go_to_point_status
  /ninja/mission_status

It displays:
  - live overhead pose
  - current goal
  - camera sanity region estimate
  - last navigation completion error
  - recent completion history

It also appends completion rows to a CSV file so the test results can be used
later in the report.
"""

import csv
import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from std_msgs.msg import String


NAV_PHASES = {"NAV_APPROACH", "NAV_TO_FRIDGE", "GOTO", "GO_TO_POINT"}


@dataclass
class PoseSnapshot:
    x: float
    y: float
    theta: float
    stamp_s: float

    @property
    def theta_deg(self) -> float:
        return math.degrees(self.theta)


@dataclass
class CompletionRow:
    stamp_s: float
    source: str
    target_x: float
    target_y: float
    target_theta_deg: float
    actual_x: float
    actual_y: float
    actual_theta_deg: float
    dx: float
    dy: float
    dtheta_deg: float
    dist_mm: float
    sanity: str


def wrap_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def fmt(value: Optional[float], width: int = 7, digits: int = 1) -> str:
    if value is None:
        return "-".rjust(width)
    return f"{value:{width}.{digits}f}"


def parse_json(text: str) -> Dict[str, Any]:
    try:
        data = json.loads(text)
        return data if isinstance(data, dict) else {"value": data}
    except Exception:
        return {"raw": text}


class PositionVerifierNode(Node):
    def __init__(self) -> None:
        super().__init__("ninja_position_verifier")

        self.declare_parameter("pose_topic", "/ninja/pose")
        self.declare_parameter("goal_topic", "/ninja/goal_pose")
        self.declare_parameter("go_to_point_status_topic", "/ninja/go_to_point_status")
        self.declare_parameter("mission_status_topic", "/ninja/mission_status")
        self.declare_parameter("display_hz", 1.0)
        self.declare_parameter("clear_screen", True)
        self.declare_parameter(
            "csv_path",
            os.path.join(os.environ.get("EUROBOT_WS", os.getcwd()), "logs", "ninja_position_verification.csv"),
        )

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.go_to_point_status_topic = str(self.get_parameter("go_to_point_status_topic").value)
        self.mission_status_topic = str(self.get_parameter("mission_status_topic").value)
        self.clear_screen = bool(self.get_parameter("clear_screen").value)
        display_hz = max(0.2, float(self.get_parameter("display_hz").value))
        self.csv_path = Path(str(self.get_parameter("csv_path").value)).expanduser()

        self.pose: Optional[PoseSnapshot] = None
        self.goal: Optional[PoseSnapshot] = None
        self.last_goal_time_s: Optional[float] = None
        self.go_to_point_status: Dict[str, Any] = {}
        self.mission_status: Dict[str, Any] = {}
        self.last_mission_phase: Optional[str] = None
        self.last_completion_time_s = 0.0
        self.last_completion_key: Optional[Tuple[int, int, int, str]] = None
        self.history: List[CompletionRow] = []

        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._ensure_csv_header()

        self.create_subscription(Pose2D, self.pose_topic, self._on_pose, 10)
        self.create_subscription(Pose2D, self.goal_topic, self._on_goal, 10)
        self.create_subscription(String, self.go_to_point_status_topic, self._on_go_to_point_status, 10)
        self.create_subscription(String, self.mission_status_topic, self._on_mission_status, 10)

        self.create_timer(1.0 / display_hz, self._render)
        self.get_logger().info("Ninja position verifier running. This node only monitors; it does not command the robot.")
        self.get_logger().info(f"CSV log: {self.csv_path}")

    def _ensure_csv_header(self) -> None:
        if self.csv_path.exists() and self.csv_path.stat().st_size > 0:
            return
        with self.csv_path.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "time_s",
                "source",
                "target_x_mm",
                "target_y_mm",
                "target_theta_deg",
                "actual_x_mm",
                "actual_y_mm",
                "actual_theta_deg",
                "dx_mm",
                "dy_mm",
                "dtheta_deg",
                "distance_mm",
                "sanity_region",
            ])

    def _append_csv(self, row: CompletionRow) -> None:
        with self.csv_path.open("a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                f"{row.stamp_s:.3f}",
                row.source,
                f"{row.target_x:.1f}",
                f"{row.target_y:.1f}",
                f"{row.target_theta_deg:.1f}",
                f"{row.actual_x:.1f}",
                f"{row.actual_y:.1f}",
                f"{row.actual_theta_deg:.1f}",
                f"{row.dx:.1f}",
                f"{row.dy:.1f}",
                f"{row.dtheta_deg:.1f}",
                f"{row.dist_mm:.1f}",
                row.sanity,
            ])

    def _on_pose(self, msg: Pose2D) -> None:
        self.pose = PoseSnapshot(float(msg.x), float(msg.y), float(msg.theta), time.time())

    def _on_goal(self, msg: Pose2D) -> None:
        self.goal = PoseSnapshot(float(msg.x), float(msg.y), float(msg.theta), time.time())
        self.last_goal_time_s = time.time()

    def _on_go_to_point_status(self, msg: String) -> None:
        self.go_to_point_status = parse_json(msg.data)
        reached = bool(self.go_to_point_status.get("reached", False))
        reason = str(self.go_to_point_status.get("reason", ""))
        if reached or reason == "goal_reached":
            self._record_completion("go_to_point")

    def _on_mission_status(self, msg: String) -> None:
        self.mission_status = parse_json(msg.data)
        phase = self._extract_phase(self.mission_status)
        if phase is not None and self.last_mission_phase is not None:
            if self.last_mission_phase in NAV_PHASES and phase != self.last_mission_phase:
                self._record_completion(f"mission_exit_{self.last_mission_phase}")
        if phase is not None:
            self.last_mission_phase = phase

    def _extract_phase(self, data: Dict[str, Any]) -> Optional[str]:
        for key in ("phase", "state", "nav_phase"):
            val = data.get(key)
            if isinstance(val, str) and val:
                return val
        return None

    def _extract_pose_from_status(self) -> Optional[PoseSnapshot]:
        pose_data = self.go_to_point_status.get("pose")
        if isinstance(pose_data, dict):
            x = pose_data.get("x_mm", pose_data.get("x"))
            y = pose_data.get("y_mm", pose_data.get("y"))
            heading = pose_data.get("heading_deg", pose_data.get("theta_deg"))
            if x is not None and y is not None:
                theta = math.radians(float(heading)) if heading is not None else (self.pose.theta if self.pose else 0.0)
                return PoseSnapshot(float(x), float(y), theta, time.time())
        return None

    def _record_completion(self, source: str) -> None:
        now = time.time()
        if self.goal is None:
            return
        actual = self._extract_pose_from_status() or self.pose
        if actual is None:
            return

        # De-duplicate repeated status messages from the same goal completion.
        key = (round(self.goal.x), round(self.goal.y), round(actual.x), source)
        if self.last_completion_key == key and (now - self.last_completion_time_s) < 2.0:
            return
        self.last_completion_key = key
        self.last_completion_time_s = now

        dx = actual.x - self.goal.x
        dy = actual.y - self.goal.y
        dtheta = wrap_deg(actual.theta_deg - self.goal.theta_deg)
        dist = math.hypot(dx, dy)
        row = CompletionRow(
            stamp_s=now,
            source=source,
            target_x=self.goal.x,
            target_y=self.goal.y,
            target_theta_deg=self.goal.theta_deg,
            actual_x=actual.x,
            actual_y=actual.y,
            actual_theta_deg=actual.theta_deg,
            dx=dx,
            dy=dy,
            dtheta_deg=dtheta,
            dist_mm=dist,
            sanity=self._sanity_region(actual),
        )
        self.history.append(row)
        self.history = self.history[-8:]
        self._append_csv(row)
        self.get_logger().info(
            f"{source}: target=({row.target_x:.0f},{row.target_y:.0f},{row.target_theta_deg:.0f}deg) "
            f"actual=({row.actual_x:.0f},{row.actual_y:.0f},{row.actual_theta_deg:.0f}deg) "
            f"error=({row.dx:+.0f},{row.dy:+.0f},{row.dtheta_deg:+.1f}deg), dist={row.dist_mm:.0f}mm"
        )

    def _sanity_region(self, pose: PoseSnapshot) -> str:
        x, y = pose.x, pose.y
        if 2400 <= x <= 3000 and 1550 <= y <= 2000:
            return "blue nest / start-finish"
        if 0 <= x <= 600 and 1550 <= y <= 2000:
            return "yellow nest"
        if 600 <= x <= 2400 and 1550 <= y <= 2000:
            if abs(x - 2300) <= 130 and abs(y - 1900) <= 130:
                return "NB ninja start zone"
            if abs(x - 1900) <= 130 and abs(y - 1925) <= 100:
                return "F1 pre-pickup corridor"
            if abs(x - 1650) <= 130 and abs(y - 1925) <= 100:
                return "F2 pre-pickup corridor"
            return "granary"
        if 0 <= x <= 3000 and 0 <= y < 1550:
            return "main arena"
        return "outside expected arena"

    def _status_line(self) -> str:
        if not self.go_to_point_status:
            return "-"
        reason = str(self.go_to_point_status.get("reason", "-"))
        active = self.go_to_point_status.get("active", None)
        reached = self.go_to_point_status.get("reached", None)
        bits = [f"reason={reason}"]
        if active is not None:
            bits.append(f"active={active}")
        if reached is not None:
            bits.append(f"reached={reached}")
        return " ".join(bits)

    def _mission_line(self) -> str:
        if not self.mission_status:
            return "-"
        phase = self._extract_phase(self.mission_status)
        if phase:
            return f"phase={phase}"
        raw = self.mission_status.get("raw")
        if raw:
            return str(raw)[:70]
        return json.dumps(self.mission_status)[:70]

    def _render(self) -> None:
        now = time.time()
        if self.clear_screen:
            print("\033[2J\033[H", end="")

        pose_age = None if self.pose is None else now - self.pose.stamp_s
        goal_age = None if self.goal is None else now - self.goal.stamp_s
        sanity = "no pose yet" if self.pose is None else self._sanity_region(self.pose)

        print("┌──────────────────────────────────────────────────────────────────────────────┐")
        print("│ Ninja Position Verification / Overhead Accuracy Monitor                      │")
        print("├────────────────────┬──────────┬──────────┬──────────┬──────────┬────────────┤")
        print("│ Item               │ x [mm]   │ y [mm]   │ θ [deg]  │ age [s]  │ note       │")
        print("├────────────────────┼──────────┼──────────┼──────────┼──────────┼────────────┤")
        if self.pose:
            print(
                f"│ Live /ninja/pose  │ {fmt(self.pose.x)} │ {fmt(self.pose.y)} │ "
                f"{fmt(self.pose.theta_deg)} │ {fmt(pose_age, 7, 2)} │ overhead   │"
            )
        else:
            print("│ Live /ninja/pose  │        - │        - │        - │        - │ waiting    │")
        if self.goal:
            print(
                f"│ Current goal      │ {fmt(self.goal.x)} │ {fmt(self.goal.y)} │ "
                f"{fmt(self.goal.theta_deg)} │ {fmt(goal_age, 7, 2)} │ target     │"
            )
        else:
            print("│ Current goal      │        - │        - │        - │        - │ none       │")
        print("├────────────────────┴──────────┴──────────┴──────────┴──────────┴────────────┤")
        print(f"│ Camera sanity: {sanity[:60]:<60} │")
        print(f"│ go_to_point:   {self._status_line()[:60]:<60} │")
        print(f"│ mission:       {self._mission_line()[:60]:<60} │")
        print(f"│ CSV log:       {str(self.csv_path)[:60]:<60} │")
        print("├──────────────────────────────────────────────────────────────────────────────┤")
        print("│ Recent goal-vs-actual completions                                           │")
        print("├──────────────┬──────────────┬──────────────┬──────────┬──────────┬──────────┤")
        print("│ source       │ target x,y   │ actual x,y   │ dx [mm]  │ dy [mm]  │ dist [mm]│")
        print("├──────────────┼──────────────┼──────────────┼──────────┼──────────┼──────────┤")
        rows = self.history[-5:]
        if not rows:
            print("│ -            │ -            │ -            │        - │        - │        - │")
        else:
            for row in rows:
                src = row.source[:12]
                target = f"{row.target_x:.0f},{row.target_y:.0f}"[:12]
                actual = f"{row.actual_x:.0f},{row.actual_y:.0f}"[:12]
                print(
                    f"│ {src:<12} │ {target:<12} │ {actual:<12} │ "
                    f"{fmt(row.dx)} │ {fmt(row.dy)} │ {fmt(row.dist_mm)} │"
                )
        print("└──────────────┴──────────────┴──────────────┴──────────┴──────────┴──────────┘")
        print("Tip: run this beside the GUI while recording screenshots/video for the thesis.", flush=True)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = PositionVerifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
