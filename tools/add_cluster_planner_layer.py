#!/usr/bin/env python3
"""
Add group-level planner outputs to overhead_camera_node.py without replacing
existing single-crate planner topics.

New topics:
  /overhead/main_bot_cluster_target_json
  /overhead/main_bot_cluster_target_queue_json
  /overhead/ninja_fridge_target_json
  /overhead/ninja_fridge_target_queue_json

This script patches the current local source file in-place so corrected
arena/top-down dimensions already in the repo are preserved.
"""
from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path.cwd()
NODE = ROOT / "src" / "overhead_control" / "overhead_control" / "overhead_camera_node.py"

if not NODE.exists():
    raise SystemExit(f"Could not find {NODE}. Run this from the repository root")

s = NODE.read_text(encoding="utf-8")

if "main_bot_cluster_target_pub" in s:
    print("Cluster planner layer already appears to be installed. Nothing changed.")
    raise SystemExit(0)

publisher_marker = '''        self.ninja_target_queue_pub = self.create_publisher(
            String,
            "/overhead/ninja_target_queue_json",
            10,
        )
'''
publisher_insert = publisher_marker + '''
        # Group-level planner layer. These outputs do not replace the original
        # single-crate targets above; they are additional higher-level targets
        # for report/debug use and later controller integration.
        self.main_bot_cluster_target_pub = self.create_publisher(
            String,
            "/overhead/main_bot_cluster_target_json",
            10,
        )

        self.main_bot_cluster_target_queue_pub = self.create_publisher(
            String,
            "/overhead/main_bot_cluster_target_queue_json",
            10,
        )

        self.ninja_fridge_target_pub = self.create_publisher(
            String,
            "/overhead/ninja_fridge_target_json",
            10,
        )

        self.ninja_fridge_target_queue_pub = self.create_publisher(
            String,
            "/overhead/ninja_fridge_target_queue_json",
            10,
        )
'''
if publisher_marker not in s:
    raise SystemExit("Could not find publisher insertion marker. No file changed.")
s = s.replace(publisher_marker, publisher_insert, 1)

methods_marker = '''    # ============================================================
    # ROBOT FOOTPRINT DRAWING
    # ============================================================
'''
methods_insert = r'''    # ============================================================
    # GROUP-LEVEL PLANNER LAYER
    # ============================================================

    @staticmethod
    def _planner_num(value, default=None):
        try:
            return float(value)
        except Exception:
            return default

    @staticmethod
    def _planner_member_summary(crate):
        return {
            "track_id": crate.get("track_id"),
            "aruco_id": crate.get("aruco_id"),
            "crate_type": crate.get("crate_type"),
            "x_mm": crate.get("x_mm"),
            "y_mm": crate.get("y_mm"),
            "long_axis_deg": crate.get("long_axis_deg"),
        }

    def _planner_non_empty_crates(self, stable_crates):
        """Return stable crate detections except empty-crate tags.

        Empty crates (ID 41) are excluded because the main robot cluster pickup
        is intended for hazelnut crate groups. Ninja fridge targeting also uses
        hazelnut crates, with known fridge-center fallback when detections are
        incomplete.
        """
        out = []
        for crate in stable_crates:
            try:
                if int(crate.get("aruco_id", -1)) == 41:
                    continue
            except Exception:
                pass
            if self._planner_num(crate.get("x_mm")) is None:
                continue
            if self._planner_num(crate.get("y_mm")) is None:
                continue
            out.append(crate)
        return out

    def _planner_cluster_crates(self, crates, max_link_distance_mm=260.0):
        """Simple distance-based grouping for nearby crate detections."""
        n = len(crates)
        visited = [False] * n
        clusters = []

        for i in range(n):
            if visited[i]:
                continue
            visited[i] = True
            stack = [i]
            members = []

            while stack:
                idx = stack.pop()
                c = crates[idx]
                members.append(c)
                cx = float(c["x_mm"])
                cy = float(c["y_mm"])

                for j in range(n):
                    if visited[j]:
                        continue
                    o = crates[j]
                    ox = float(o["x_mm"])
                    oy = float(o["y_mm"])
                    d = float(np.hypot(cx - ox, cy - oy))
                    if d <= float(max_link_distance_mm):
                        visited[j] = True
                        stack.append(j)

            clusters.append(members)

        return clusters

    def _planner_mean_axis_deg(self, crates):
        """Mean long-axis angle with 180-degree symmetry."""
        vals = []
        for crate in crates:
            a = self._planner_num(crate.get("long_axis_deg"))
            if a is not None:
                vals.append(float(a))
        if not vals:
            return None
        # Double-angle average makes 0/180 equivalent.
        sx = float(np.mean([np.cos(np.deg2rad(2.0 * a)) for a in vals]))
        sy = float(np.mean([np.sin(np.deg2rad(2.0 * a)) for a in vals]))
        if abs(sx) < 1e-9 and abs(sy) < 1e-9:
            return None
        return round(float(normalize_angle_deg(np.rad2deg(np.arctan2(sy, sx)) / 2.0)), 1)

    def _planner_cluster_object(self, members, cluster_id):
        xs = [float(c["x_mm"]) for c in members]
        ys = [float(c["y_mm"]) for c in members]
        center_x = float(np.mean(xs))
        center_y = float(np.mean(ys))
        color_counts = {}
        for c in members:
            key = str(c.get("crate_type", c.get("aruco_id", "unknown")))
            color_counts[key] = color_counts.get(key, 0) + 1

        return {
            "cluster_id": int(cluster_id),
            "target_type": "crate_cluster",
            "size": int(len(members)),
            "ready_for_four_crate_pickup": bool(len(members) >= 4),
            "center_x_mm": round(center_x, 1),
            "center_y_mm": round(center_y, 1),
            "spread_x_mm": round(float(max(xs) - min(xs)) if xs else 0.0, 1),
            "spread_y_mm": round(float(max(ys) - min(ys)) if ys else 0.0, 1),
            "mean_long_axis_deg": self._planner_mean_axis_deg(members),
            "color_counts": color_counts,
            "members": [self._planner_member_summary(c) for c in members],
            "planner_note": (
                "Cluster center is a higher-level pickup target. It does not replace "
                "the original single-crate target planner unless a controller explicitly uses this topic."
            ),
        }

    def _planner_pose_behind_center(self, center_x, center_y, heading_deg, distance_mm):
        a = np.deg2rad(float(heading_deg))
        return {
            "x_mm": round(float(center_x - np.cos(a) * distance_mm), 1),
            "y_mm": round(float(center_y - np.sin(a) * distance_mm), 1),
            "heading_deg": round(float(normalize_angle_deg(heading_deg)), 1),
            "approach_distance_mm": round(float(distance_mm), 1),
            "target_reference": "cluster_or_fridge_center",
        }

    def _planner_cluster_candidate_for_main_bot(self, robot_pose, cluster):
        cx = float(cluster["center_x_mm"])
        cy = float(cluster["center_y_mm"])
        axis = cluster.get("mean_long_axis_deg")
        if axis is None:
            # Fallback: approach roughly from current robot direction if visible,
            # otherwise use a neutral heading.
            if robot_pose is not None:
                axis = float(np.rad2deg(np.arctan2(cy - float(robot_pose["y_mm"]), cx - float(robot_pose["x_mm"]))))
            else:
                axis = 0.0

        heading_options = [float(axis), float(axis) + 180.0]
        poses = []
        for h in heading_options:
            pose = self._planner_pose_behind_center(
                cx,
                cy,
                normalize_angle_deg(h),
                float(self.crate_approach_distance_mm),
            )
            ok, reason = self.validate_main_bot_approach_point(pose["x_mm"], pose["y_mm"])
            pose["valid_for_main_bot"] = bool(ok)
            pose["main_bot_reason"] = reason
            if robot_pose is not None:
                pose["distance_to_robot_mm"] = round(self.distance_between_points_mm(robot_pose, pose), 1)
            else:
                pose["distance_to_robot_mm"] = None
            poses.append(pose)

        valid = [p for p in poses if p.get("valid_for_main_bot", False)]
        if robot_pose is not None and valid:
            best_pose = sorted(valid, key=lambda p: float(p["distance_to_robot_mm"]))[0]
        elif valid:
            best_pose = valid[0]
        else:
            best_pose = poses[0] if poses else None

        return {
            "rank": None,
            "actor": "main_bot",
            "target_type": "crate_cluster",
            "cluster": cluster,
            "pickup_center": {
                "x_mm": cluster["center_x_mm"],
                "y_mm": cluster["center_y_mm"],
                "meaning": "centroid_of_detected_crate_cluster",
            },
            "approach_pose": best_pose,
            "candidate_approach_poses": poses,
            "distance_to_approach_mm": None if best_pose is None else best_pose.get("distance_to_robot_mm"),
            "selection_method": "cluster_centroid_for_main_bot_v1",
        }

    def build_main_bot_cluster_targets(self, robots, stable_crates):
        robot_pose = robots.get("main_robot") if isinstance(robots, dict) else None
        crates = self._planner_non_empty_crates(stable_crates)
        raw_clusters = self._planner_cluster_crates(crates, max_link_distance_mm=260.0)
        cluster_objs = []
        for i, members in enumerate(raw_clusters, start=1):
            if len(members) < 2:
                continue
            cluster_objs.append(self._planner_cluster_object(members, i))

        queue = [self._planner_cluster_candidate_for_main_bot(robot_pose, c) for c in cluster_objs]

        def sort_key(item):
            cluster = item.get("cluster") or {}
            dist = item.get("distance_to_approach_mm")
            if dist is None:
                dist = 1e9
            return (
                0 if cluster.get("ready_for_four_crate_pickup") else 1,
                -int(cluster.get("size", 0)),
                float(dist),
            )

        queue = sorted(queue, key=sort_key)
        for i, item in enumerate(queue, start=1):
            item["rank"] = i

        target = {
            "active": False,
            "actor": "main_bot",
            "target_type": "crate_cluster",
            "reason": "unknown",
            "robot": robot_pose,
            "selected_cluster": None,
            "pickup_center": None,
            "approach_pose": None,
            "queue_length": len(queue),
            "top_candidates": queue[:5],
            "selection_method": "cluster_centroid_for_main_bot_v1",
            "old_single_crate_topics_unchanged": True,
        }

        if robot_pose is None:
            target["reason"] = "main_robot_not_visible_or_not_selected"
            return target, queue
        if not queue:
            target["reason"] = "no_cluster_with_at_least_two_crates"
            return target, queue

        best = queue[0]
        target.update({
            "active": True,
            "reason": "ok",
            "selected_cluster": best.get("cluster"),
            "pickup_center": best.get("pickup_center"),
            "approach_pose": best.get("approach_pose"),
            "distance_to_approach_mm": best.get("distance_to_approach_mm"),
        })
        return target, queue

    def _planner_fridge_defs(self):
        """Known fridge centers used by the additional Ninja fridge layer.

        The layer uses these known centers as fallback if one or both fridge
        crates are not detected. This does not modify the existing top-down map.
        """
        return [
            {"fridge_id": "blue_f1", "team": "blue", "center_x_mm": 1900.0, "center_y_mm": 1725.0, "half_x_mm": 50.0, "half_y_mm": 75.0},
            {"fridge_id": "blue_f2", "team": "blue", "center_x_mm": 1650.0, "center_y_mm": 1775.0, "half_x_mm": 50.0, "half_y_mm": 75.0},
            {"fridge_id": "yellow_f1", "team": "yellow", "center_x_mm": 1100.0, "center_y_mm": 1725.0, "half_x_mm": 50.0, "half_y_mm": 75.0},
            {"fridge_id": "yellow_f2", "team": "yellow", "center_x_mm": 1350.0, "center_y_mm": 1775.0, "half_x_mm": 50.0, "half_y_mm": 75.0},
        ]

    def _planner_crates_near_fridge(self, stable_crates, fridge, margin_mm=120.0):
        cx = float(fridge["center_x_mm"])
        cy = float(fridge["center_y_mm"])
        hx = float(fridge["half_x_mm"]) + float(margin_mm)
        hy = float(fridge["half_y_mm"]) + float(margin_mm)
        crates = []
        for crate in self._planner_non_empty_crates(stable_crates):
            x = float(crate["x_mm"])
            y = float(crate["y_mm"])
            if (cx - hx) <= x <= (cx + hx) and (cy - hy) <= y <= (cy + hy):
                crates.append(crate)
        crates = sorted(crates, key=lambda c: float(np.hypot(float(c["x_mm"]) - cx, float(c["y_mm"]) - cy)))
        return crates

    def _planner_fridge_candidate_for_ninja(self, robot_pose, fridge, stable_crates):
        crates = self._planner_crates_near_fridge(stable_crates, fridge)
        used = crates[:2]
        known_x = float(fridge["center_x_mm"])
        known_y = float(fridge["center_y_mm"])

        if len(used) >= 2:
            center_x = float(np.mean([float(c["x_mm"]) for c in used]))
            center_y = float(np.mean([float(c["y_mm"]) for c in used]))
            reason = "two_detected_crates_midpoint"
            confidence = "high"
        elif len(used) == 1:
            # Blend one visible crate with known fridge center to avoid chasing
            # a single bad detection too far away from the actual fridge.
            center_x = (float(used[0]["x_mm"]) + known_x) / 2.0
            center_y = (float(used[0]["y_mm"]) + known_y) / 2.0
            reason = "one_detected_crate_blended_with_known_fridge_center"
            confidence = "medium"
        else:
            center_x = known_x
            center_y = known_y
            reason = "known_fridge_center_fallback"
            confidence = "fallback"

        # Ninja normally approaches fridges from the rear/granary side, facing
        # toward lower y in the current arena convention.
        approach = {
            "x_mm": round(center_x, 1),
            "y_mm": round(min(1950.0, center_y + 140.0), 1),
            "heading_deg": -90.0,
            "target_reference": "fridge_pair_midpoint_or_known_center",
        }
        ok, ok_reason = self.validate_ninja_body_point(approach["x_mm"], approach["y_mm"])
        approach["valid_for_ninja_body"] = bool(ok)
        approach["ninja_body_reason"] = ok_reason

        dist = None
        if robot_pose is not None:
            dist = round(self.distance_between_points_mm(robot_pose, approach), 1)

        own_team = str(self.team_side).lower() == str(fridge.get("team", "")).lower()

        return {
            "rank": None,
            "actor": "ninja",
            "target_type": "fridge_pair",
            "fridge_id": fridge["fridge_id"],
            "fridge_team": fridge["team"],
            "own_team_fridge": bool(own_team),
            "known_fridge_center": {
                "x_mm": round(known_x, 1),
                "y_mm": round(known_y, 1),
            },
            "pickup_center": {
                "x_mm": round(center_x, 1),
                "y_mm": round(center_y, 1),
                "meaning": reason,
                "confidence": confidence,
            },
            "detected_crates_near_fridge": len(crates),
            "used_crates": [self._planner_member_summary(c) for c in used],
            "approach_pose": approach,
            "distance_to_approach_mm": dist,
            "selection_method": "fridge_pair_midpoint_or_known_center_for_ninja_v1",
        }

    def build_ninja_fridge_targets(self, robots, stable_crates):
        robot_pose = robots.get("ninja") if isinstance(robots, dict) else None
        queue = [
            self._planner_fridge_candidate_for_ninja(robot_pose, fridge, stable_crates)
            for fridge in self._planner_fridge_defs()
        ]

        def sort_key(item):
            dist = item.get("distance_to_approach_mm")
            if dist is None:
                dist = 1e9
            # Prefer own-team fridges for the current side, then closer targets.
            return (0 if item.get("own_team_fridge") else 1, float(dist))

        queue = sorted(queue, key=sort_key)
        for i, item in enumerate(queue, start=1):
            item["rank"] = i

        target = {
            "active": False,
            "actor": "ninja",
            "target_type": "fridge_pair",
            "reason": "unknown",
            "robot": robot_pose,
            "selected_fridge": None,
            "pickup_center": None,
            "approach_pose": None,
            "queue_length": len(queue),
            "top_candidates": queue,
            "selection_method": "fridge_pair_midpoint_or_known_center_for_ninja_v1",
            "old_single_crate_topics_unchanged": True,
        }

        if robot_pose is None:
            target["reason"] = "ninja_not_visible_or_not_selected"
            return target, queue
        if not queue:
            target["reason"] = "no_fridge_definitions"
            return target, queue

        best = queue[0]
        target.update({
            "active": True,
            "reason": "ok",
            "selected_fridge": best,
            "pickup_center": best.get("pickup_center"),
            "approach_pose": best.get("approach_pose"),
            "distance_to_approach_mm": best.get("distance_to_approach_mm"),
        })
        return target, queue

    def build_cluster_planner_layer(self, robots, stable_crates):
        main_target, main_queue = self.build_main_bot_cluster_targets(robots, stable_crates)
        ninja_target, ninja_queue = self.build_ninja_fridge_targets(robots, stable_crates)
        return {
            "main_bot_cluster_target": main_target,
            "main_bot_cluster_target_queue": main_queue,
            "ninja_fridge_target": ninja_target,
            "ninja_fridge_target_queue": ninja_queue,
            "notes": [
                "This group-level planner layer is additional debug/planning output.",
                "Existing single-crate target, target_queue, nav_command, and compact_command topics are unchanged.",
                "Main bot layer targets crate-cluster centroids for four-crate pickup reasoning.",
                "Ninja layer targets fridge pair midpoint, with known fridge-center fallback when detections are incomplete.",
            ],
        }

'''
if methods_marker not in s:
    raise SystemExit("Could not find method insertion marker. No file changed.")
s = s.replace(methods_marker, methods_insert + methods_marker, 1)

old = '''        targets = self.build_targets(robots, stable_crates)
        nav_commands = self.build_nav_commands(targets)
'''
new = '''        targets = self.build_targets(robots, stable_crates)
        planner_layer = self.build_cluster_planner_layer(robots, stable_crates)
        nav_commands = self.build_nav_commands(targets)
'''
if old not in s:
    raise SystemExit("Could not find targets/nav_commands marker. No file changed.")
s = s.replace(old, new, 1)

old = '''            "targets": targets,
            "nav_commands": nav_commands,
'''
new = '''            "targets": targets,
            "planner_layer": planner_layer,
            "nav_commands": nav_commands,
'''
if old not in s:
    raise SystemExit("Could not find world_state target insertion marker. No file changed.")
s = s.replace(old, new, 1)

queue_publish_marker = '''        ninja_queue_msg = String()
        ninja_queue_msg.data = json.dumps(targets.get("ninja_queue", []))
        self.ninja_target_queue_pub.publish(ninja_queue_msg)
'''
queue_publish_insert = queue_publish_marker + '''
        main_cluster_target_msg = String()
        main_cluster_target_msg.data = json.dumps(planner_layer.get("main_bot_cluster_target", {}))
        self.main_bot_cluster_target_pub.publish(main_cluster_target_msg)

        main_cluster_queue_msg = String()
        main_cluster_queue_msg.data = json.dumps(planner_layer.get("main_bot_cluster_target_queue", []))
        self.main_bot_cluster_target_queue_pub.publish(main_cluster_queue_msg)

        ninja_fridge_target_msg = String()
        ninja_fridge_target_msg.data = json.dumps(planner_layer.get("ninja_fridge_target", {}))
        self.ninja_fridge_target_pub.publish(ninja_fridge_target_msg)

        ninja_fridge_queue_msg = String()
        ninja_fridge_queue_msg.data = json.dumps(planner_layer.get("ninja_fridge_target_queue", []))
        self.ninja_fridge_target_queue_pub.publish(ninja_fridge_queue_msg)
'''
if queue_publish_marker not in s:
    raise SystemExit("Could not find queue publish insertion marker. No file changed.")
s = s.replace(queue_publish_marker, queue_publish_insert, 1)

NODE.write_text(s, encoding="utf-8")
print("Installed cluster/fridge planner layer into:", NODE)
print("New topics:")
print("  /overhead/main_bot_cluster_target_json")
print("  /overhead/main_bot_cluster_target_queue_json")
print("  /overhead/ninja_fridge_target_json")
print("  /overhead/ninja_fridge_target_queue_json")
print("Existing single-crate target topics were kept unchanged.")
