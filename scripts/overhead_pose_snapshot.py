#!/usr/bin/env python3
"""Clean overhead pose/coordinate snapshot for Eurobot report evidence.

Reads the active ROS 2 topics and prints a concise report-friendly summary:
- Ninja pose from /ninja/pose
- Main bot from /overhead/world_state_json
- Enemy bot from /overhead/opponent_robots_json
- Stable crate/Jenga coordinates with theta when available
- Useful main-bot and Ninja target outputs

Raw topic data and CSV evidence are saved under evidence/.
"""

from __future__ import annotations

import csv
import json
import math
import os
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable

ROOT = Path.cwd()
EVIDENCE_DIR = ROOT / "evidence"
EVIDENCE_DIR.mkdir(exist_ok=True)

CRATE_IDS = {36, 47, 41}
ROBOT_IDS = {1, 6, 55}


def _run(cmd: list[str], timeout: float = 4.0) -> str:
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=timeout)
        return p.stdout.strip()
    except subprocess.TimeoutExpired:
        return ""


def read_json_topic(topic: str, timeout_s: float = 4.0) -> Any | None:
    # Use bash so we can strip ROS echo separators exactly like the manual commands.
    cmd = [
        "bash",
        "-lc",
        f"timeout {timeout_s:.1f}s ros2 topic echo --once {topic} --field data | sed '/^---$/d'",
    ]
    out = _run(cmd, timeout=timeout_s + 1.0)
    if not out:
        return None
    # Sometimes ROS echo can wrap or include whitespace; find the first JSON object/array.
    s = out.strip()
    start_candidates = [i for i in [s.find("{"), s.find("[")] if i >= 0]
    if start_candidates:
        s = s[min(start_candidates):]
    try:
        return json.loads(s)
    except Exception:
        return None


def read_pose_topic(topic: str = "/ninja/pose", timeout_s: float = 4.0) -> dict[str, float] | None:
    out = _run(["bash", "-lc", f"timeout {timeout_s:.1f}s ros2 topic echo --once {topic}"], timeout=timeout_s + 1.0)
    if not out:
        return None
    vals: dict[str, float] = {}
    for key in ("x", "y", "theta"):
        m = re.search(rf"^\s*{key}:\s*([-+0-9.eE]+)", out, re.MULTILINE)
        if m:
            try:
                vals[key] = float(m.group(1))
            except ValueError:
                pass
    if "x" in vals and "y" in vals:
        return vals
    return None


def save_json(path: Path, data: Any) -> None:
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=False)


def get_id(d: dict[str, Any]) -> int | None:
    for k in ("id", "marker_id", "aruco_id", "tag_id", "robot_id", "crate_id"):
        if k in d:
            try:
                return int(d[k])
            except Exception:
                pass
    return None


def get_x(d: dict[str, Any]) -> float | None:
    for k in ("x", "x_mm", "world_x", "world_x_mm", "center_x_mm", "target_x", "goal_x", "cx_mm"):
        if k in d:
            try:
                return float(d[k])
            except Exception:
                pass
    # Some target messages use nested position.
    pos = d.get("position") or d.get("pose") or d.get("center") or d.get("target")
    if isinstance(pos, dict):
        return get_x(pos)
    return None


def get_y(d: dict[str, Any]) -> float | None:
    for k in ("y", "y_mm", "world_y", "world_y_mm", "center_y_mm", "target_y", "goal_y", "cy_mm"):
        if k in d:
            try:
                return float(d[k])
            except Exception:
                pass
    pos = d.get("position") or d.get("pose") or d.get("center") or d.get("target")
    if isinstance(pos, dict):
        return get_y(pos)
    return None


def get_theta_deg(d: dict[str, Any]) -> float | None:
    # Explicit degree fields first. Include long_axis_deg for crates.
    deg_keys = (
        "theta_deg", "heading_deg", "yaw_deg", "angle_deg", "orientation_deg",
        "long_axis_deg", "raw_angle_deg", "marker_angle_deg", "rotation_deg",
    )
    for k in deg_keys:
        if k in d and d[k] is not None:
            try:
                return float(d[k])
            except Exception:
                pass

    # Radian-ish fields. The generic 'theta' in /ninja/pose is radians, but some world objects may use deg.
    for k in ("theta", "heading", "yaw", "theta_rad", "heading_rad", "yaw_rad", "angle_rad"):
        if k in d and d[k] is not None:
            try:
                v = float(d[k])
            except Exception:
                continue
            if k.endswith("_rad") or abs(v) <= 2.0 * math.pi + 0.01:
                return math.degrees(v)
            return v

    # Nested pose/orientation.
    for k in ("pose", "position", "orientation", "robot_pose"):
        child = d.get(k)
        if isinstance(child, dict):
            t = get_theta_deg(child)
            if t is not None:
                return t
    return None


def fmt(v: float | None, width: int = 7, digits: int = 1, suffix: str = "") -> str:
    if v is None:
        return f"{'-':>{width}}"
    return f"{v:>{width}.{digits}f}{suffix}"


def iter_dicts(obj: Any, path: str = "") -> Iterable[tuple[str, dict[str, Any]]]:
    if isinstance(obj, dict):
        yield path, obj
        for k, v in obj.items():
            p = f"{path}.{k}" if path else str(k)
            yield from iter_dicts(v, p)
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            p = f"{path}[{i}]"
            yield from iter_dicts(v, p)


def find_first_by_path(world: Any, path: str) -> Any | None:
    cur = world
    for part in path.split("."):
        if not isinstance(cur, dict):
            return None
        cur = cur.get(part)
        if cur is None:
            return None
    return cur


def find_robot(world: Any, robot_id: int, preferred_paths: list[str]) -> tuple[dict[str, Any], str] | tuple[None, None]:
    for p in preferred_paths:
        obj = find_first_by_path(world, p)
        if isinstance(obj, dict):
            rid = get_id(obj)
            x, y = get_x(obj), get_y(obj)
            if x is not None and y is not None and (rid is None or rid == robot_id):
                return obj, p
        elif isinstance(obj, list):
            for i, item in enumerate(obj):
                if isinstance(item, dict) and get_id(item) == robot_id and get_x(item) is not None and get_y(item) is not None:
                    return item, f"{p}[{i}]"
    for p, d in iter_dicts(world):
        if get_id(d) == robot_id and get_x(d) is not None and get_y(d) is not None:
            return d, p
    return None, None


def opponent_to_list(data: Any) -> list[dict[str, Any]]:
    if data is None:
        return []
    # Common structures: list, {opponent_official_robots: [...]}, {robots: [...]}
    if isinstance(data, list):
        return [x for x in data if isinstance(x, dict)]
    if isinstance(data, dict):
        for key in ("opponent_official_robots", "opponent_robots", "robots", "all", "objects"):
            val = data.get(key)
            if isinstance(val, list):
                return [x for x in val if isinstance(x, dict)]
        # If top level itself is one robot.
        if get_x(data) is not None and get_y(data) is not None:
            return [data]
    return []


def collect_crates(world: Any) -> list[dict[str, Any]]:
    stable = find_first_by_path(world, "stable_crates")
    if not isinstance(stable, list):
        # Fallback: find first list key named stable_crates anywhere.
        for p, d in iter_dicts(world):
            val = d.get("stable_crates")
            if isinstance(val, list):
                stable = val
                break
    raw = find_first_by_path(world, "raw_crate_detections")
    if not isinstance(raw, list):
        raw = []

    def nearby_theta(crate: dict[str, Any]) -> float | None:
        t = get_theta_deg(crate)
        if t is not None:
            return t
        cid, x, y = get_id(crate), get_x(crate), get_y(crate)
        if cid is None or x is None or y is None:
            return None
        best = None
        best_dist = 999999.0
        for r in raw:
            if not isinstance(r, dict) or get_id(r) != cid:
                continue
            rx, ry = get_x(r), get_y(r)
            if rx is None or ry is None:
                continue
            dist = math.hypot(rx - x, ry - y)
            if dist < best_dist:
                best = r
                best_dist = dist
        if best is not None and best_dist < 40.0:
            return get_theta_deg(best)
        return None

    out: list[dict[str, Any]] = []
    if isinstance(stable, list):
        for i, c in enumerate(stable):
            if not isinstance(c, dict):
                continue
            cid = get_id(c)
            x, y = get_x(c), get_y(c)
            if cid in CRATE_IDS and x is not None and y is not None:
                out.append({
                    "id": cid,
                    "x": x,
                    "y": y,
                    "theta_deg": nearby_theta(c),
                    "source": f"stable_crates[{i}]",
                })
    # If stable_crates is unavailable, use raw detections directly.
    if not out and isinstance(raw, list):
        for i, c in enumerate(raw):
            if not isinstance(c, dict):
                continue
            cid = get_id(c)
            x, y = get_x(c), get_y(c)
            if cid in CRATE_IDS and x is not None and y is not None:
                out.append({"id": cid, "x": x, "y": y, "theta_deg": get_theta_deg(c), "source": f"raw_crate_detections[{i}]"})
    return sorted(out, key=lambda c: (c["id"], c["y"], c["x"]))


def find_named_target(data: Any, name_hint: str = "target") -> tuple[dict[str, Any], str] | tuple[None, None]:
    # Prefer dicts whose path says target/fridge/goal and which have x/y.
    candidates: list[tuple[int, str, dict[str, Any]]] = []
    for p, d in iter_dicts(data):
        x, y = get_x(d), get_y(d)
        if x is None or y is None:
            continue
        pl = p.lower()
        # Avoid reporting robot pose as the target when possible.
        if "robot" in pl and "target" not in pl:
            continue
        score = 0
        for word in ("target", "goal", "center", "fridge", "midpoint", "crate"):
            if word in pl:
                score += 10
        if name_hint and name_hint.lower() in pl:
            score += 20
        # Prefer items with useful metadata or ID.
        if get_id(d) is not None:
            score += 2
        if any(k in d for k in ("target_type", "active", "status", "type")):
            score += 5
        candidates.append((score, p, d))
    if not candidates:
        return None, None
    candidates.sort(key=lambda x: (-x[0], len(x[1])))
    _, p, d = candidates[0]
    return d, p


def describe_target(label: str, data: Any, source: str) -> str | None:
    if data is None:
        return None
    obj, p = find_named_target(data, label)
    if obj is None:
        return None
    cid = get_id(obj)
    x, y = get_x(obj), get_y(obj)
    theta = get_theta_deg(obj)
    extra = []
    if isinstance(obj, dict):
        for k in ("target_type", "type", "status", "active", "action"):
            if k in obj:
                extra.append(f"{k}={obj[k]}")
    id_s = f"ID{cid} " if cid is not None else ""
    theta_s = f" theta={theta:.1f}°" if theta is not None else ""
    extra_s = f" ({', '.join(extra)})" if extra else ""
    return f"  {label}: {id_s}x={x:.1f} mm  y={y:.1f} mm{theta_s}  source={source}:{p}{extra_s}"


def print_robot_line(name: str, rid: int, x: float | None, y: float | None, theta: float | None, source: str) -> str:
    return f"  {name:<8} ID{rid:<3} x={fmt(x, 7)} mm  y={fmt(y, 7)} mm  theta={fmt(theta, 7, 1, '°')}  source={source}"


def main() -> int:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")

    ninja_pose = read_pose_topic("/ninja/pose")
    opponent_data = read_json_topic("/overhead/opponent_robots_json")
    world = read_json_topic("/overhead/world_state_json")
    main_target_topic = read_json_topic("/overhead/main_bot_target_json", timeout_s=2.5)
    cluster_target_topic = read_json_topic("/overhead/main_bot_cluster_target_json", timeout_s=2.5)
    ninja_target_topic = read_json_topic("/overhead/ninja_fridge_target_json", timeout_s=2.5)

    # Save raw evidence.
    if ninja_pose is not None:
        (EVIDENCE_DIR / f"ninja_pose_{ts}.txt").write_text(
            f"x: {ninja_pose.get('x')}\ny: {ninja_pose.get('y')}\ntheta: {ninja_pose.get('theta')}\n",
            encoding="utf-8",
        )
    if opponent_data is not None:
        save_json(EVIDENCE_DIR / f"opponent_robots_{ts}.json", opponent_data)
    if world is not None:
        save_json(EVIDENCE_DIR / f"world_state_{ts}.json", world)
    if main_target_topic is not None:
        save_json(EVIDENCE_DIR / f"main_bot_target_{ts}.json", main_target_topic)
    if cluster_target_topic is not None:
        save_json(EVIDENCE_DIR / f"main_bot_cluster_target_{ts}.json", cluster_target_topic)
    if ninja_target_topic is not None:
        save_json(EVIDENCE_DIR / f"ninja_fridge_target_{ts}.json", ninja_target_topic)

    lines: list[str] = []
    lines.append("=== Clean overhead coordinate snapshot ===")
    lines.append("")
    lines.append("Robots")

    if ninja_pose is not None:
        theta_deg = math.degrees(float(ninja_pose.get("theta", 0.0)))
        lines.append(print_robot_line("Ninja", 55, ninja_pose.get("x"), ninja_pose.get("y"), theta_deg, "/ninja/pose"))
    elif world is not None:
        obj, src = find_robot(world, 55, ["robots.ninja", "robots.own_ninja_candidates", "robots.all"])
        if obj:
            lines.append(print_robot_line("Ninja", 55, get_x(obj), get_y(obj), get_theta_deg(obj), src or "world_state"))
        else:
            lines.append("  Ninja    ID55  not found")
    else:
        lines.append("  Ninja    ID55  not found")

    if world is not None:
        obj, src = find_robot(world, 1, ["robots.main_robot", "robots.team_official_robots", "robots.all"])
        if obj:
            lines.append(print_robot_line("Main bot", 1, get_x(obj), get_y(obj), get_theta_deg(obj), src or "world_state"))
        else:
            lines.append("  Main bot ID1   not found")
    else:
        lines.append("  Main bot ID1   world state unavailable")

    enemy_obj = None
    enemy_src = None
    for i, r in enumerate(opponent_to_list(opponent_data)):
        rid = get_id(r)
        if rid == 6 or rid is None:
            enemy_obj = r
            enemy_src = f"opponent_official_robots[{i}]"
            break
    if enemy_obj is None and world is not None:
        enemy_obj, enemy_src = find_robot(world, 6, ["robots.opponent_official_robots", "robots.opponent_robots", "robots.all"])
    if enemy_obj is not None:
        lines.append(print_robot_line("Enemy", 6, get_x(enemy_obj), get_y(enemy_obj), get_theta_deg(enemy_obj), enemy_src or "/overhead/opponent_robots_json"))
    else:
        lines.append("  Enemy    ID6   not found")

    crates = collect_crates(world) if world is not None else []
    lines.append("")
    lines.append(f"Stable crate/Jenga coordinates ({len(crates)} objects)")
    if crates:
        lines.append(f"{'id':>4} {'x [mm]':>9} {'y [mm]':>9} {'theta':>9}  source")
        for c in crates:
            theta = c.get("theta_deg")
            theta_s = f"{theta:8.1f}°" if theta is not None else f"{'-':>9}"
            lines.append(f"{c['id']:>4} {c['x']:>9.1f} {c['y']:>9.1f} {theta_s}  {c['source']}")
    else:
        lines.append("  No stable crate/Jenga objects found")

    lines.append("")
    lines.append("Useful planner/target output")
    target_lines: list[str] = []

    # Main bot selected target crate: prefer direct topic, else world-state locations.
    for label, data, source in [
        ("Main bot target crate", main_target_topic, "/overhead/main_bot_target_json"),
        ("Main bot target crate", world, "world_state"),
    ]:
        line = describe_target(label, data, source)
        if line:
            target_lines.append(line)
            break

    # Optional cluster target if available.
    line = describe_target("Main bot cluster target", cluster_target_topic or world, "/overhead/main_bot_cluster_target_json" if cluster_target_topic else "world_state")
    if line and line not in target_lines:
        target_lines.append(line)

    # Ninja target: prefer the dedicated fridge target topic, else world planner/targets.
    line = describe_target("Ninja target", ninja_target_topic, "/overhead/ninja_fridge_target_json")
    if not line and world is not None:
        # Try common world-state sections directly first.
        for p in ("planner_layer.ninja_fridge_target", "targets.ninja", "nav_commands.ninja.source_target", "compact_commands.ninja"):
            section = find_first_by_path(world, p)
            line = describe_target("Ninja target", section, f"world_state:{p}")
            if line:
                break
    if line:
        target_lines.append(line)

    if target_lines:
        lines.extend(target_lines)
    else:
        lines.append("  No useful planner/target objects found")

    # Warnings/tips.
    if enemy_obj is None:
        lines.append("")
        lines.append("NOTE: Enemy bot ID6 was not found. Check marker visibility and opponent settings.")

    # Save CSV and snapshot text.
    csv_path = EVIDENCE_DIR / f"overhead_crates_{ts}.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["id", "x_mm", "y_mm", "theta_deg", "source"])
        writer.writeheader()
        for c in crates:
            writer.writerow({"id": c["id"], "x_mm": c["x"], "y_mm": c["y"], "theta_deg": c.get("theta_deg"), "source": c["source"]})

    snapshot_path = EVIDENCE_DIR / f"overhead_snapshot_{ts}.txt"
    text = "\n".join(lines) + "\n"
    snapshot_path.write_text(text, encoding="utf-8")

    print(text)
    print("Saved evidence files:")
    print(f"  {snapshot_path}")
    print(f"  {csv_path}")
    if ninja_pose is not None:
        print(f"  evidence/ninja_pose_{ts}.txt")
    if opponent_data is not None:
        print(f"  evidence/opponent_robots_{ts}.json")
    if world is not None:
        print(f"  evidence/world_state_{ts}.json")
    if main_target_topic is not None:
        print(f"  evidence/main_bot_target_{ts}.json")
    if cluster_target_topic is not None:
        print(f"  evidence/main_bot_cluster_target_{ts}.json")
    if ninja_target_topic is not None:
        print(f"  evidence/ninja_fridge_target_{ts}.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
