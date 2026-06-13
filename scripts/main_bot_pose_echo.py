#!/usr/bin/env python3
"""Print main bot ID1 pose in the same simple style as `ros2 topic echo --once /ninja/pose`.

Reads /overhead/world_state_json once and extracts the main robot pose.
Run from a sourced ROS terminal while overhead_control is running.
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import subprocess
import sys
from typing import Any, Iterable


def clean_ros2_field_output(text: str) -> str:
    lines = []
    for line in text.splitlines():
        s = line.strip()
        if not s or s == "---":
            continue
        if s.startswith("data:"):
            s = s.split("data:", 1)[1].strip()
        lines.append(s)
    raw = "\n".join(lines).strip()

    # If ros2 printed a quoted string, unquote it.
    if (raw.startswith("'") and raw.endswith("'")) or (raw.startswith('"') and raw.endswith('"')):
        try:
            raw = ast.literal_eval(raw)
        except Exception:
            pass
    return raw


def get_first(d: dict[str, Any], names: Iterable[str]) -> Any:
    for name in names:
        if name in d and d[name] is not None:
            return d[name]
    return None


def xy_of(obj: dict[str, Any]) -> tuple[Any, Any]:
    x = get_first(obj, ["x_mm", "x", "world_x_mm", "center_x_mm", "cx_mm", "center_x"])
    y = get_first(obj, ["y_mm", "y", "world_y_mm", "center_y_mm", "cy_mm", "center_y"])
    return x, y


def theta_of(obj: dict[str, Any]) -> Any:
    # Prefer radians, like /ninja/pose.
    rad = get_first(obj, ["theta", "theta_rad", "heading_rad", "angle_rad", "yaw", "yaw_rad"])
    if rad is not None:
        return rad

    deg = get_first(obj, ["theta_deg", "heading_deg", "angle_deg", "yaw_deg"])
    if deg is not None:
        try:
            return math.radians(float(deg))
        except Exception:
            return deg
    return None


def iter_dicts(obj: Any, path: str = ""):
    if isinstance(obj, dict):
        yield path, obj
        for k, v in obj.items():
            yield from iter_dicts(v, f"{path}.{k}" if path else str(k))
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            yield from iter_dicts(v, f"{path}[{i}]")


def object_id(obj: dict[str, Any]) -> Any:
    return get_first(obj, ["id", "marker_id", "aruco_id", "tag_id"])


def find_main_bot(world: dict[str, Any], marker_id: int) -> dict[str, Any] | None:
    robots = world.get("robots") or {}

    # Best known locations first.
    for candidate in [
        robots.get("main_robot"),
        robots.get("own_official_robot"),
    ]:
        if isinstance(candidate, dict):
            x, y = xy_of(candidate)
            if x is not None and y is not None:
                return candidate

    team = robots.get("team_official_robots")
    if isinstance(team, list):
        for candidate in team:
            if isinstance(candidate, dict):
                oid = object_id(candidate)
                x, y = xy_of(candidate)
                if (oid == marker_id or oid is None) and x is not None and y is not None:
                    return candidate

    # Fallback: recursively find ID1 with x/y. Prefer robot paths over target/planner duplicates.
    matches: list[tuple[int, str, dict[str, Any]]] = []
    for path, obj in iter_dicts(world):
        if object_id(obj) != marker_id:
            continue
        x, y = xy_of(obj)
        if x is None or y is None:
            continue
        score = 0
        low = path.lower()
        if "robots.main_robot" in low:
            score += 100
        if "robots" in low:
            score += 40
        if "target" in low or "planner" in low or "nav_commands" in low:
            score -= 20
        matches.append((score, path, obj))

    if not matches:
        return None
    matches.sort(key=lambda t: t[0], reverse=True)
    return matches[0][2]


def fmt(value: Any) -> str:
    if value is None:
        return "-"
    try:
        return f"{float(value):.1f}"
    except Exception:
        return str(value)


def main() -> int:
    parser = argparse.ArgumentParser(description="Print main bot pose from /overhead/world_state_json")
    parser.add_argument("--id", type=int, default=1, help="Main bot ArUco/marker ID, default: 1")
    parser.add_argument("--timeout", type=float, default=10.0, help="Timeout in seconds, default: 10")
    args = parser.parse_args()

    try:
        result = subprocess.run(
            ["ros2", "topic", "echo", "--once", "/overhead/world_state_json", "--field", "data"],
            text=True,
            capture_output=True,
            timeout=args.timeout,
            check=False,
        )
    except subprocess.TimeoutExpired:
        print("main bot not found")
        print("reason: timeout waiting for /overhead/world_state_json")
        return 1

    raw = clean_ros2_field_output(result.stdout)
    if not raw:
        print("main bot not found")
        print("reason: no /overhead/world_state_json data")
        if result.stderr.strip():
            print(result.stderr.strip(), file=sys.stderr)
        return 1

    try:
        world = json.loads(raw)
    except json.JSONDecodeError as exc:
        print("main bot not found")
        print(f"reason: could not parse world_state_json: {exc}")
        return 1

    robot = find_main_bot(world, args.id)
    if not robot:
        print("main bot not found")
        return 1

    x, y = xy_of(robot)
    theta = theta_of(robot)

    print(f"x: {fmt(x)}")
    print(f"y: {fmt(y)}")
    print(f"theta: {fmt(theta)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
