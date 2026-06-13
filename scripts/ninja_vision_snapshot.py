#!/usr/bin/env python3
"""
Compact Ninja onboard vision snapshot.

Run while ninja_crate_align_only.launch.py is running and the terminal has been sourced:
  source /opt/ros/jazzy/setup.bash
  source ~/eurobot_net.sh
  source install/setup.bash
  python3 scripts/ninja_vision_snapshot.py

This version reads ROS topics using `ros2 topic echo --once`, which matches the
manual command that was verified to work.
"""

import json
import subprocess
import sys
import time
from typing import Any, Dict, Optional


def read_json_string_topic(topic: str, attempts: int = 8, timeout_s: float = 6.0) -> Optional[Dict[str, Any]]:
    """Read a std_msgs/String JSON payload using ros2 topic echo --field data."""
    cmd = ["ros2", "topic", "echo", "--once", topic, "--field", "data"]

    for _ in range(attempts):
        try:
            result = subprocess.run(
                cmd,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout_s,
                check=False,
            )
        except subprocess.TimeoutExpired:
            time.sleep(0.4)
            continue

        text = result.stdout.strip()
        if not text:
            time.sleep(0.4)
            continue

        # ros2 echo usually appends "---". Remove separators and warning-like lines.
        lines = []
        for line in text.splitlines():
            s = line.strip()
            if not s or s == "---":
                continue
            if s.startswith("WARNING:") or s.startswith("Could not determine"):
                continue
            lines.append(s)

        payload = "\n".join(lines).strip()
        if not payload:
            time.sleep(0.4)
            continue

        # Some shells/tools may preserve wrapping quotes; json.loads handles normal JSON.
        try:
            return json.loads(payload)
        except json.JSONDecodeError:
            # Try extracting the first {...} block if extra text slipped in.
            start = payload.find("{")
            end = payload.rfind("}")
            if start >= 0 and end > start:
                try:
                    return json.loads(payload[start : end + 1])
                except json.JSONDecodeError:
                    pass

        time.sleep(0.4)

    return None


def fmt(value: Any, decimals: int = 1) -> str:
    if value is None:
        return "None"
    try:
        return f"{float(value):.{decimals}f}"
    except (TypeError, ValueError):
        return str(value)


def main() -> int:
    crate = read_json_string_topic("/ninja/vision/crate", attempts=10, timeout_s=7.0)
    if crate is None:
        print("No /ninja/vision/crate data received.")
        print("Check that ninja_crate_align_only.launch.py is running and that this terminal uses the same ~/eurobot_net.sh environment.")
        print("Manual check: ros2 topic echo --once /ninja/vision/crate")
        return 1

    # Align is optional for this compact report; action is computed from crate geometry
    # so it does not flicker to NO_PAIR when align_status catches a different frame.
    align = read_json_string_topic("/ninja/vision/align_status", attempts=3, timeout_s=3.0) or {}

    pair = crate.get("pair") or {}
    markers = crate.get("markers") or []

    center_tol = None
    try:
        center_tol = (align.get("tolerances") or {}).get("center_px")
    except AttributeError:
        center_tol = None
    if center_tol is None:
        center_tol = 35.0

    target_size = None
    try:
        target_size = (align.get("target") or {}).get("size")
    except AttributeError:
        target_size = None
    if target_size is None:
        target_size = 182.0

    # Practical pickup threshold. The real robot cannot get closer than this setup.
    pickup_min_size = max(0.0, float(target_size) - 10.0)

    pair_seen = bool(pair.get("seen"))
    pair_center_error = pair.get("center_error_px")
    pair_size = pair.get("marker_size_px")

    center_ready = False
    if pair_seen and pair_center_error is not None:
        center_ready = abs(float(pair_center_error)) <= float(center_tol)

    if not pair_seen:
        action = "NO_PAIR"
    elif not center_ready:
        action = "CENTER_PAIR"
    elif pair_size is not None and float(pair_size) >= pickup_min_size:
        action = "PICKUP_READY"
    else:
        action = "APPROACH_PAIR"

    print("=== Ninja vision / align snapshot ===")

    for m in markers:
        mid = m.get("id")
        print(
            f"ID{mid}: center_error={fmt(m.get('center_error_px'))}px, "
            f"marker_size={fmt(m.get('marker_size_px'))}px"
        )

    print(
        f"Pair: seen={pair_seen}, ids={pair.get('ids')}, "
        f"center_error={fmt(pair_center_error)}px, "
        f"separation={fmt(pair.get('separation_px'))}px"
    )
    print(f"Center: ready={center_ready}, tolerance={fmt(center_tol)}px")
    print(f"Action: {action}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
