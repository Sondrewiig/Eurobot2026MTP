#!/usr/bin/env python3
"""
Collects several /ninja/vision/align_status samples at the physical pickup pose
and prints recommended pair calibration values for crate_align_node.py.

Run while ninja_crate_align_only.launch.py is running and both crate markers are visible.
"""
import argparse
import json
import math
import statistics
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional


def read_once(timeout_s: float = 4.0) -> Optional[Dict[str, Any]]:
    cmd = [
        "ros2", "topic", "echo", "--once",
        "/ninja/vision/align_status", "--field", "data",
    ]
    try:
        p = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return None
    out = "\n".join(line for line in p.stdout.splitlines() if line.strip() != "---").strip()
    if not out:
        return None
    try:
        return json.loads(out)
    except json.JSONDecodeError:
        return None


def mean(values: List[float]) -> float:
    return statistics.mean(values) if values else float("nan")


def stdev(values: List[float]) -> float:
    return statistics.stdev(values) if len(values) > 1 else 0.0


def rng(values: List[float]) -> str:
    return f"{min(values):.1f}..{max(values):.1f}" if values else "-"


def recommend_tol(values: List[float], floor: float, margin: float, cap: float) -> float:
    if not values:
        return floor
    # 3 sigma + margin, bounded by a practical minimum/maximum.
    val = 3.0 * stdev(values) + margin
    return max(floor, min(cap, val))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--samples", type=int, default=30, help="number of valid pair samples to collect")
    ap.add_argument("--delay", type=float, default=0.15, help="delay between samples [s]")
    ap.add_argument("--timeout", type=float, default=60.0, help="overall timeout [s]")
    args = ap.parse_args()

    print("Place the crates at the desired physical pickup position.")
    print("Keep both ID36 and ID47 visible and still.")
    print(f"Collecting {args.samples} valid pair samples...\n")

    centers: List[float] = []
    sizes: List[float] = []
    seps: List[float] = []
    actions: List[str] = []
    profiles: List[str] = []
    target_seen = None

    t0 = time.time()
    attempts = 0
    while len(centers) < args.samples and (time.time() - t0) < args.timeout:
        attempts += 1
        d = read_once()
        if not d or not d.get("ok"):
            time.sleep(args.delay)
            continue
        cur = d.get("current") or {}
        c = cur.get("pair_center_error_px")
        s = cur.get("pair_size_px")
        sep = cur.get("pair_separation_px")
        if c is None or s is None or sep is None:
            time.sleep(args.delay)
            continue
        centers.append(float(c))
        sizes.append(float(s))
        seps.append(float(sep))
        actions.append(str(d.get("action")))
        profiles.append(str(d.get("profile")))
        target_seen = d.get("target") or target_seen
        print(f"sample {len(centers):02d}/{args.samples}: center={c:.1f}px size={s:.1f}px sep={sep:.1f}px action={d.get('action')}")
        time.sleep(args.delay)

    if not centers:
        print("\nNo valid pair samples collected. Check that /ninja/vision/align_status is publishing and pair is visible.")
        return 1

    center_mean = mean(centers)
    size_mean = mean(sizes)
    sep_mean = mean(seps)

    center_tol = recommend_tol(centers, floor=35.0, margin=10.0, cap=60.0)
    size_tol = recommend_tol(sizes, floor=12.0, margin=8.0, cap=35.0)
    sep_tol = recommend_tol(seps, floor=30.0, margin=10.0, cap=65.0)

    print("\n=== Pickup calibration result ===")
    print(f"valid_samples: {len(centers)} / attempts {attempts}")
    print(f"profile_seen: {max(set(profiles), key=profiles.count) if profiles else '-'}")
    print(f"actions_seen: {sorted(set(actions))}")

    print("\nMeasured at physical pickup pose:")
    print(f"pair_center_error_px: mean={center_mean:.1f}, std={stdev(centers):.1f}, range={rng(centers)}")
    print(f"pair_size_px:         mean={size_mean:.1f}, std={stdev(sizes):.1f}, range={rng(sizes)}")
    print(f"pair_separation_px:   mean={sep_mean:.1f}, std={stdev(seps):.1f}, range={rng(seps)}")

    if target_seen:
        print("\nCurrent target from node:")
        print(f"center={target_seen.get('center')}  size={target_seen.get('size')}  separation={target_seen.get('separation')}")

    print("\nRecommended target values:")
    print(f"target_center_px = {center_mean:.1f}")
    print(f"target_size_px = {size_mean:.1f}")
    print(f"target_separation_px = {sep_mean:.1f}")

    print("\nRecommended tolerances:")
    print(f"center_tolerance_px = {center_tol:.1f}")
    print(f"size_tolerance_px = {size_tol:.1f}")
    print(f"separation_tolerance_px = {sep_tol:.1f}")

    print("\nSend this output back if you want me to patch crate_align_node.py with these exact values.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
