import json
import time
from pathlib import Path
from typing import Dict, Tuple, Optional
from dataclasses import dataclass, field

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


# ============================================================
# TRACKER DATA
# ============================================================

@dataclass
class CrateTrack:
    track_id: int
    aruco_id: int
    crate_type: str

    x_mm: float
    y_mm: float
    long_axis_deg: Optional[float]

    x_floor_projection_mm: float
    y_floor_projection_mm: float
    tag_height_mm: float
    zone_estimate: str
    size_px: float

    last_seen_frame: int
    confidence: int = 1
    seen_count: int = 1
    missed_frames: int = 0

    history: list = field(default_factory=list)


# ============================================================
# ANGLE HELPERS
# ============================================================

def normalize_angle_deg(angle):
    if angle is None:
        return None
    return float(angle % 360.0)


def angle_difference_deg(a, b):
    """
    Smallest absolute difference between two angles in degrees.
    Returns 0..180.
    """
    if a is None or b is None:
        return 180.0

    diff = (a - b + 180.0) % 360.0 - 180.0
    return abs(diff)


def smooth_angle_deg(old_angle, new_angle, alpha):
    """
    Smooth angle without wrap-around problems.
    """
    if old_angle is None:
        return normalize_angle_deg(new_angle)

    if new_angle is None:
        return normalize_angle_deg(old_angle)

    old_rad = np.deg2rad(old_angle)
    new_rad = np.deg2rad(new_angle)

    old_vec = np.array([np.cos(old_rad), np.sin(old_rad)])
    new_vec = np.array([np.cos(new_rad), np.sin(new_rad)])

    vec = (1.0 - alpha) * old_vec + alpha * new_vec

    angle = np.rad2deg(np.arctan2(vec[1], vec[0]))
    return normalize_angle_deg(angle)


# ============================================================
# CRATE TRACKER
# ============================================================

class CrateTracker:
    def __init__(
        self,
        cluster_distance_mm=140.0,
        duplicate_merge_distance_mm=30.0,
        max_missed_frames=20,
        min_confidence_to_show=2,
        smooth_alpha_pos=0.30,
        smooth_alpha_angle=0.25,
    ):
        self.tracks = []
        self.next_track_id = 0

        self.cluster_distance_mm = float(cluster_distance_mm)
        self.duplicate_merge_distance_mm = float(duplicate_merge_distance_mm)
        self.max_missed_frames = int(max_missed_frames)
        self.min_confidence_to_show = int(min_confidence_to_show)
        self.smooth_alpha_pos = float(smooth_alpha_pos)
        self.smooth_alpha_angle = float(smooth_alpha_angle)

    @staticmethod
    def distance_mm(a_x, a_y, b_x, b_y):
        return float(np.hypot(a_x - b_x, a_y - b_y))

    def reset(self):
        self.tracks = []
        self.next_track_id = 0

    def update(self, detections, frame_idx):
        matched_track_ids = set()

        # Match each raw detection to nearest existing track with same ArUco ID.
        for det in detections:
            det_id = int(det["aruco_id"])
            det_x = float(det["x_mm"])
            det_y = float(det["y_mm"])

            best_track = None
            best_dist = 1e9

            for track in self.tracks:
                if track.aruco_id != det_id:
                    continue

                if track.track_id in matched_track_ids:
                    continue

                d = self.distance_mm(det_x, det_y, track.x_mm, track.y_mm)

                if d < best_dist:
                    best_dist = d
                    best_track = track

            if best_track is not None and best_dist <= self.cluster_distance_mm:
                self.update_track(best_track, det, frame_idx)
                matched_track_ids.add(best_track.track_id)
            else:
                new_track = self.create_track(det, frame_idx)
                self.tracks.append(new_track)
                matched_track_ids.add(new_track.track_id)

        # Mark unmatched tracks as missed.
        for track in self.tracks:
            if track.track_id not in matched_track_ids:
                track.missed_frames += 1
                track.confidence = max(0, track.confidence - 1)

        self.remove_dead_tracks()
        self.merge_true_duplicates()

    def create_track(self, det, frame_idx):
        track = CrateTrack(
            track_id=self.next_track_id,
            aruco_id=int(det["aruco_id"]),
            crate_type=det["crate_type"],

            x_mm=float(det["x_mm"]),
            y_mm=float(det["y_mm"]),
            long_axis_deg=det["long_axis_deg"],

            x_floor_projection_mm=float(det["x_floor_projection_mm"]),
            y_floor_projection_mm=float(det["y_floor_projection_mm"]),
            tag_height_mm=float(det["tag_height_mm"]),
            zone_estimate=det["zone_estimate"],
            size_px=float(det["size_px"]),

            last_seen_frame=frame_idx,
            confidence=1,
            seen_count=1,
            missed_frames=0,
        )

        track.history.append((track.x_mm, track.y_mm))
        self.next_track_id += 1
        return track

    def update_track(self, track, det, frame_idx):
        alpha = self.smooth_alpha_pos

        track.x_mm = (1.0 - alpha) * track.x_mm + alpha * float(det["x_mm"])
        track.y_mm = (1.0 - alpha) * track.y_mm + alpha * float(det["y_mm"])

        track.x_floor_projection_mm = (
            (1.0 - alpha) * track.x_floor_projection_mm
            + alpha * float(det["x_floor_projection_mm"])
        )
        track.y_floor_projection_mm = (
            (1.0 - alpha) * track.y_floor_projection_mm
            + alpha * float(det["y_floor_projection_mm"])
        )

        track.long_axis_deg = smooth_angle_deg(
            track.long_axis_deg,
            det["long_axis_deg"],
            self.smooth_alpha_angle,
        )

        track.tag_height_mm = float(det["tag_height_mm"])
        track.zone_estimate = det["zone_estimate"]
        track.size_px = float(det["size_px"])

        track.last_seen_frame = frame_idx
        track.missed_frames = 0
        track.seen_count += 1
        track.confidence = min(20, track.confidence + 1)

        track.history.append((track.x_mm, track.y_mm))
        if len(track.history) > 20:
            track.history.pop(0)

    def remove_dead_tracks(self):
        self.tracks = [
            t for t in self.tracks
            if t.missed_frames <= self.max_missed_frames and t.confidence > 0
        ]

    def merge_true_duplicates(self):
        """
        Conservative duplicate merge.

        Important:
        Real crates can be close together in groups of 4.
        So we only merge if same ID and extremely close, around 30 mm.
        This should merge accidental duplicate tracks, not separate crates.
        """
        merged = True

        while merged:
            merged = False

            for i in range(len(self.tracks)):
                if merged:
                    break

                for j in range(i + 1, len(self.tracks)):
                    a = self.tracks[i]
                    b = self.tracks[j]

                    if a.aruco_id != b.aruco_id:
                        continue

                    d = self.distance_mm(a.x_mm, a.y_mm, b.x_mm, b.y_mm)

                    if d > self.duplicate_merge_distance_mm:
                        continue

                    # Also require roughly similar angle if both have angle.
                    if (
                        a.long_axis_deg is not None
                        and b.long_axis_deg is not None
                        and angle_difference_deg(a.long_axis_deg, b.long_axis_deg) > 35.0
                    ):
                        continue

                    keep, remove = self.choose_track_to_keep(a, b)
                    self.absorb_track(keep, remove)

                    self.tracks.remove(remove)
                    merged = True
                    break

    @staticmethod
    def choose_track_to_keep(a, b):
        if a.confidence > b.confidence:
            return a, b
        if b.confidence > a.confidence:
            return b, a

        # If same confidence, keep older/lower ID.
        if a.track_id <= b.track_id:
            return a, b
        return b, a

    def absorb_track(self, keep, remove):
        total_conf = max(1, keep.confidence + remove.confidence)

        keep.x_mm = (
            keep.x_mm * keep.confidence + remove.x_mm * remove.confidence
        ) / total_conf

        keep.y_mm = (
            keep.y_mm * keep.confidence + remove.y_mm * remove.confidence
        ) / total_conf

        keep.x_floor_projection_mm = (
            keep.x_floor_projection_mm * keep.confidence
            + remove.x_floor_projection_mm * remove.confidence
        ) / total_conf

        keep.y_floor_projection_mm = (
            keep.y_floor_projection_mm * keep.confidence
            + remove.y_floor_projection_mm * remove.confidence
        ) / total_conf

        keep.long_axis_deg = smooth_angle_deg(
            keep.long_axis_deg,
            remove.long_axis_deg,
            0.5,
        )

        keep.confidence = min(20, keep.confidence + remove.confidence)
        keep.seen_count += remove.seen_count
        keep.missed_frames = min(keep.missed_frames, remove.missed_frames)
        keep.last_seen_frame = max(keep.last_seen_frame, remove.last_seen_frame)

        keep.history.extend(remove.history)
        keep.history = keep.history[-20:]

    def stable_tracks(self):
        return [
            t for t in self.tracks
            if t.confidence >= self.min_confidence_to_show
        ]

    def to_json_list(self):
        output = []

        for t in self.stable_tracks():
            approach_headings = []

            if t.long_axis_deg is not None:
                approach_headings = [
                    round(t.long_axis_deg, 1),
                    round((t.long_axis_deg + 180.0) % 360.0, 1),
                ]

            output.append({
                "track_id": t.track_id,
                "aruco_id": t.aruco_id,
                "crate_type": t.crate_type,

                "x_mm": round(t.x_mm, 1),
                "y_mm": round(t.y_mm, 1),

                "x_floor_projection_mm": round(t.x_floor_projection_mm, 1),
                "y_floor_projection_mm": round(t.y_floor_projection_mm, 1),

                "tag_height_mm": round(t.tag_height_mm, 1),
                "zone_estimate": t.zone_estimate,

                "long_axis_deg": None if t.long_axis_deg is None else round(t.long_axis_deg, 1),
                "approach_headings_deg": approach_headings,

                "size_px": round(t.size_px, 1),
                "confidence": t.confidence,
                "seen_count": t.seen_count,
                "missed_frames": t.missed_frames,
            })

        return output


# ============================================================
# ROS2 OVERHEAD NODE
# ============================================================

class OverheadCameraNode(Node):
    # IMPORTANT:
    # - The top-down synthetic map is published as RGB8, so top-down colors use RGB.
    # - The camera/debug overlay is an OpenCV BGR image, so debug overlay colors use BGR.

    # Official-like RGB colors for /overhead/topdown_image
    COLOR_TEAM_BLUE = (0, 91, 140)
    COLOR_TEAM_YELLOW = (247, 181, 0)
    COLOR_EMPTY_BLACK = (42, 41, 42)
    COLOR_GRANARY = (76, 43, 32)
    COLOR_PANTRY = (0, 170, 0)
    COLOR_REF = (255, 255, 255)
    COLOR_TEXT = (230, 230, 230)
    COLOR_GRID = (70, 70, 70)
    COLOR_NINJA = (180, 0, 180)
    COLOR_MAIN_ROBOT = (0, 220, 0)
    COLOR_ENEMY_ROBOT = (220, 0, 0)
    COLOR_FRIDGE = (0, 0, 200)
    COLOR_COLLECTION = (160, 160, 160)
    COLOR_NINJA_NEST = (200, 100, 0)

    # BGR colors for camera/debug overlay
    BGR_TEAM_BLUE = (140, 91, 0)
    BGR_TEAM_YELLOW = (0, 181, 247)
    BGR_EMPTY_BLACK = (42, 41, 42)
    BGR_GRANARY = (32, 43, 76)
    BGR_REF = (255, 255, 255)
    BGR_TEXT = (230, 230, 230)

    def __init__(self):
        super().__init__("overhead_camera_node")

        # ============================================================
        # ROS PARAMETERS
        # ============================================================

        self.declare_parameter("team_side", "blue")

        self.declare_parameter("device", "/dev/video2")
        self.declare_parameter("camera_width", 3840)
        self.declare_parameter("camera_height", 2160)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("camera_fourcc", "MJPG")

        self.declare_parameter("display_width", 1280)
        self.declare_parameter("display_height", 720)

        self.declare_parameter("arena_width_mm", 3000.0)
        self.declare_parameter("arena_height_mm", 2000.0)

        self.declare_parameter("camera_x_mm", 1725.0)
        self.declare_parameter("camera_y_mm", 2100.0)
        self.declare_parameter("camera_height_mm", 1670.0)

        self.declare_parameter("marker_20_x", 600.0)
        self.declare_parameter("marker_20_y", 1400.0)

        self.declare_parameter("marker_21_x", 2400.0)
        self.declare_parameter("marker_21_y", 1400.0)

        self.declare_parameter("marker_22_x", 600.0)
        self.declare_parameter("marker_22_y", 600.0)

        self.declare_parameter("marker_23_x", 2400.0)
        self.declare_parameter("marker_23_y", 600.0)

        self.declare_parameter("publish_images", True)
        self.declare_parameter("topdown_background_path", "")
        self.declare_parameter("topdown_overlay_alpha", 0.60)
        self.declare_parameter("topdown_bg_offset_x_px", 0)
        self.declare_parameter("topdown_bg_offset_y_px", 0)

        # Approach-pose generation for crate pickup.
        # Approach points are placed along the crate long axis.
        self.declare_parameter("crate_approach_distance_mm", 220.0)
        self.declare_parameter("approach_wall_margin_mm", 120.0)

        # Ninja/SIMA movement rule.
        # The ninja body must stay inside the granary zone. It must not drive down
        # into the main area during our planned strategy. Later, if the actuator
        # can legally project over the granary line, set ninja_actuator_projection_reach_mm.
        self.declare_parameter("ninja_keep_body_inside_granary", True)
        self.declare_parameter("ninja_target_requires_granary_or_reach", True)
        self.declare_parameter("ninja_actuator_projection_reach_mm", 0.0)

        # Robot / optional ninja marker heights.
        # Keep these at 0.0 for floor testing with loose tags.
        # Later set them to the real tag height above the arena floor.
        self.declare_parameter("official_robot_tag_height_mm", 0.0)
        self.declare_parameter("ninja_tag_height_mm", 0.0)

        # Opponent marker handling.
        # Keep detect_opponent_robots false during our own arena/test-table work,
        # because DICT_4X4_100 can occasionally false-decode yellow/opponent IDs.
        # Enable it for real opponent testing or real matches.
        self.declare_parameter("detect_opponent_robots", False)

        # If opponent detection is enabled, require stronger confirmation before
        # publishing/drawing opponent robot markers. Own robot markers are still fast.
        self.declare_parameter("opponent_robot_min_confidence_to_show", 4)

        # Opponent marker heights.
        # Default -1 means "use the same height as our robot/ninja".
        # For testing loose enemy markers flat on the floor, set these to 0.0.
        self.declare_parameter("opponent_official_robot_tag_height_mm", -1.0)

        # Draw rejected opponent/robot-like detections as debug crosses on top-down.
        # This helps when a marker ID is detected but gets rejected because the
        # assumed height is wrong for the temporary test setup.
        self.declare_parameter("draw_rejected_robot_detections", True)

        # Optional explicit robot selection.
        # -1 means auto-select only if exactly one matching marker is visible.
        # Set these once you know which official marker is on the main robot
        # and which optional marker is mounted on the ninja/SIMA.
        self.declare_parameter("main_robot_aruco_id", -1)
        self.declare_parameter("ninja_aruco_id", -1)

        # Only one optional ninja/SIMA marker should normally be mounted.
        # If ninja_aruco_id is set, this removes the other two same-side ninja IDs
        # from the allow-list. This reduces false positives and ambiguity.
        self.declare_parameter("filter_to_configured_ninja_id", True)

        # Main official robot tag may be unknown or assigned later. Default false
        # keeps all own official robot IDs allowed; set true only once the real
        # main_robot_aruco_id is known and fixed.
        self.declare_parameter("filter_to_configured_main_robot_id", False)

        # Robot footprint dimensions and tag offsets.
        # Main bot heading is aligned with its width direction.
        self.declare_parameter("main_robot_width_mm", 256.0)
        self.declare_parameter("main_robot_length_mm", 306.0)

        # Ninja heading is aligned with its length direction.
        self.declare_parameter("ninja_width_mm", 140.0)
        self.declare_parameter("ninja_length_mm", 151.5)

        # Ninja ArUco tag offset:
        # - from rear is measured along the ninja length/forward axis.
        # - lateral offset is measured from the ninja centerline.
        #   Positive means the tag center is to the robot-left of the footprint center.
        #   If the drawn rectangle shifts to the wrong side, change this to -10.5.
        self.declare_parameter("ninja_tag_from_rear_mm", 31.5)
        self.declare_parameter("ninja_tag_lateral_offset_mm", 10.5)

        # Object/world validation.
        # This is the practical version of the 3 m x 2 m x camera-height tracking volume:
        # detect markers in the camera, height-correct them using object type, then only
        # accept objects whose corrected body/tag center is plausibly inside the arena.
        self.declare_parameter("world_position_validation_enabled", True)
        self.declare_parameter("crate_world_margin_mm", 80.0)
        self.declare_parameter("robot_world_margin_mm", 300.0)

        # Main robot top plate color note.
        # Current temporary test box has MDF/wood top.
        # Future main bot top plate is expected to be black, but this is not used for
        # target selection yet. It is kept in JSON/settings so we do not forget it.
        self.declare_parameter("main_robot_top_plate_color_mode", "mdf")

        # Arena ROI mask.
        # After the first homography is known, mask detection to the projected
        # 3000 x 2000 mm arena rectangle plus margin. This ignores visual clutter
        # outside the table without requiring a full 3D camera model.
        self.declare_parameter("aruco_enable_arena_roi_mask", True)
        self.declare_parameter("arena_roi_margin_mm", 180.0)

        # Homography/ROI recovery robustness.
        # Reference marker duplicates or false positives must never corrupt the
        # arena homography, because a bad homography can make the ROI mask hide
        # the whole table.
        self.declare_parameter("homography_max_reprojection_error_mm", 80.0)
        self.declare_parameter("aruco_roi_full_frame_recovery", True)

        # Robot/ninja smoothing. Robot markers are naturally more flickery than
        # crate tracks because we had not been smoothing them yet.
        self.declare_parameter("robot_track_max_missed_frames", 10)
        self.declare_parameter("robot_smoothing_alpha", 0.35)
        self.declare_parameter("robot_smoothing_alpha_angle", 0.25)

        # Target smoothing / display cleanup.
        self.declare_parameter("target_switch_distance_hysteresis_mm", 150.0)
        self.declare_parameter("topdown_target_queue_max_labels", 4)

        # Clean top-down crate visualization.
        # clean = planning map: rotated color-coded crates + short label + A/B ends.
        # debug/full = show detailed crate labels and all approach candidates.
        self.declare_parameter("topdown_crate_display_mode", "clean")
        self.declare_parameter("topdown_show_all_approach_candidates", False)
        self.declare_parameter("topdown_show_crate_debug_labels", False)
        self.declare_parameter("topdown_show_crate_orientation_arrow", True)
        self.declare_parameter("topdown_show_crate_ab_ends", True)
        self.declare_parameter("topdown_show_target_queue_labels", False)
        self.declare_parameter("topdown_show_game_zones", True)
        self.declare_parameter("crate_body_length_mm", 150.0)
        self.declare_parameter("crate_body_width_mm", 50.0)

        # Navigation-command generation.
        # The overhead target is split into a safe staged approach:
        # 1. drive to pre-align pose
        # 2. rotate to final approach heading
        # 3. drive straight to the approach pose
        # 4. let onboard close-range alignment/pickup take over
        self.declare_parameter("main_bot_pre_align_distance_mm", 250.0)
        self.declare_parameter("ninja_pre_align_distance_mm", 120.0)
        self.declare_parameter("pickup_handoff_distance_mm", 220.0)

        # Compact command stream for onboard robot communication.
        # This is the topic the robot should consume first, not the large debug JSON.
        self.declare_parameter("compact_command_enabled", True)
        self.declare_parameter("compact_command_ttl_ms", 1000)
        self.declare_parameter("compact_command_goal_rounding_mm", 1.0)
        self.declare_parameter("compact_command_heading_rounding_deg", 1.0)

        # ArUco detection tuning for far/small tags.
        # These defaults are more permissive than OpenCV defaults.
        self.declare_parameter("aruco_enable_clahe", True)
        self.declare_parameter("aruco_enable_sharpen", True)
        self.declare_parameter("aruco_second_pass", True)
        self.declare_parameter("aruco_clahe_clip_limit", 2.0)
        self.declare_parameter("aruco_clahe_tile_grid", 8)
        self.declare_parameter("aruco_sharpen_amount", 0.70)
        self.declare_parameter("aruco_min_marker_perimeter_rate", 0.008)
        self.declare_parameter("aruco_max_marker_perimeter_rate", 4.0)
        self.declare_parameter("aruco_adaptive_thresh_win_size_min", 3)
        self.declare_parameter("aruco_adaptive_thresh_win_size_max", 53)
        self.declare_parameter("aruco_adaptive_thresh_win_size_step", 10)
        self.declare_parameter("aruco_corner_refinement", True)
        self.declare_parameter("aruco_duplicate_center_px", 20.0)

        # Tracker parameters
        self.declare_parameter("cluster_distance_mm", 140.0)
        self.declare_parameter("duplicate_merge_distance_mm", 30.0)
        self.declare_parameter("max_missed_frames", 20)
        self.declare_parameter("min_confidence_to_show", 2)
        self.declare_parameter("smoothing_alpha", 0.30)
        self.declare_parameter("smoothing_alpha_angle", 0.25)

        # ============================================================
        # READ PARAMETERS
        # ============================================================

        self.team_side = self.get_parameter("team_side").value

        self.device = self.get_parameter("device").value
        self.camera_width = int(self.get_parameter("camera_width").value)
        self.camera_height = int(self.get_parameter("camera_height").value)
        self.camera_fps = int(self.get_parameter("camera_fps").value)
        self.camera_fourcc = self.get_parameter("camera_fourcc").value

        self.display_width = int(self.get_parameter("display_width").value)
        self.display_height = int(self.get_parameter("display_height").value)

        self.arena_width_mm = float(self.get_parameter("arena_width_mm").value)
        self.arena_height_mm = float(self.get_parameter("arena_height_mm").value)

        self.camera_x_mm = float(self.get_parameter("camera_x_mm").value)
        self.camera_y_mm = float(self.get_parameter("camera_y_mm").value)
        self.camera_height_mm = float(self.get_parameter("camera_height_mm").value)

        self.publish_images = bool(self.get_parameter("publish_images").value)
        self.topdown_background_path = str(self.get_parameter("topdown_background_path").value)
        self.topdown_overlay_alpha = float(self.get_parameter("topdown_overlay_alpha").value)
        self.topdown_bg_offset_x_px = int(self.get_parameter("topdown_bg_offset_x_px").value)
        self.topdown_bg_offset_y_px = int(self.get_parameter("topdown_bg_offset_y_px").value)

        self.crate_approach_distance_mm = float(
            self.get_parameter("crate_approach_distance_mm").value
        )
        self.approach_wall_margin_mm = float(
            self.get_parameter("approach_wall_margin_mm").value
        )

        self.ninja_keep_body_inside_granary = bool(
            self.get_parameter("ninja_keep_body_inside_granary").value
        )
        self.ninja_target_requires_granary_or_reach = bool(
            self.get_parameter("ninja_target_requires_granary_or_reach").value
        )
        self.ninja_actuator_projection_reach_mm = float(
            self.get_parameter("ninja_actuator_projection_reach_mm").value
        )

        self.official_robot_tag_height_mm = float(
            self.get_parameter("official_robot_tag_height_mm").value
        )
        self.ninja_tag_height_mm = float(
            self.get_parameter("ninja_tag_height_mm").value
        )

        self.detect_opponent_robots = bool(
            self.get_parameter("detect_opponent_robots").value
        )
        self.opponent_robot_min_confidence_to_show = int(
            self.get_parameter("opponent_robot_min_confidence_to_show").value
        )
        self.opponent_official_robot_tag_height_mm = float(
            self.get_parameter("opponent_official_robot_tag_height_mm").value
        )
        self.draw_rejected_robot_detections = bool(
            self.get_parameter("draw_rejected_robot_detections").value
        )

        self.main_robot_aruco_id = int(self.get_parameter("main_robot_aruco_id").value)
        self.ninja_aruco_id = int(self.get_parameter("ninja_aruco_id").value)
        self.filter_to_configured_ninja_id = bool(
            self.get_parameter("filter_to_configured_ninja_id").value
        )
        self.filter_to_configured_main_robot_id = bool(
            self.get_parameter("filter_to_configured_main_robot_id").value
        )

        self.main_robot_width_mm = float(self.get_parameter("main_robot_width_mm").value)
        self.main_robot_length_mm = float(self.get_parameter("main_robot_length_mm").value)

        self.ninja_width_mm = float(self.get_parameter("ninja_width_mm").value)
        self.ninja_length_mm = float(self.get_parameter("ninja_length_mm").value)
        self.ninja_tag_from_rear_mm = float(
            self.get_parameter("ninja_tag_from_rear_mm").value
        )
        self.ninja_tag_lateral_offset_mm = float(
            self.get_parameter("ninja_tag_lateral_offset_mm").value
        )

        self.world_position_validation_enabled = bool(
            self.get_parameter("world_position_validation_enabled").value
        )
        self.crate_world_margin_mm = float(
            self.get_parameter("crate_world_margin_mm").value
        )
        self.robot_world_margin_mm = float(
            self.get_parameter("robot_world_margin_mm").value
        )
        self.main_robot_top_plate_color_mode = str(
            self.get_parameter("main_robot_top_plate_color_mode").value
        )

        self.aruco_enable_arena_roi_mask = bool(
            self.get_parameter("aruco_enable_arena_roi_mask").value
        )
        self.arena_roi_margin_mm = max(
            float(self.get_parameter("arena_roi_margin_mm").value),
            450.0,
        )
        self.homography_max_reprojection_error_mm = float(
            self.get_parameter("homography_max_reprojection_error_mm").value
        )
        self.aruco_roi_full_frame_recovery = bool(
            self.get_parameter("aruco_roi_full_frame_recovery").value
        )
        self.robot_track_max_missed_frames = int(
            self.get_parameter("robot_track_max_missed_frames").value
        )
        self.robot_smoothing_alpha = float(
            self.get_parameter("robot_smoothing_alpha").value
        )
        self.robot_smoothing_alpha_angle = float(
            self.get_parameter("robot_smoothing_alpha_angle").value
        )
        self.target_switch_distance_hysteresis_mm = float(
            self.get_parameter("target_switch_distance_hysteresis_mm").value
        )
        self.topdown_target_queue_max_labels = int(
            self.get_parameter("topdown_target_queue_max_labels").value
        )

        self.topdown_crate_display_mode = str(
            self.get_parameter("topdown_crate_display_mode").value
        ).strip().lower()
        if self.topdown_crate_display_mode not in {"clean", "debug", "full"}:
            self.topdown_crate_display_mode = "clean"

        self.topdown_show_all_approach_candidates = bool(
            self.get_parameter("topdown_show_all_approach_candidates").value
        )
        self.topdown_show_crate_debug_labels = bool(
            self.get_parameter("topdown_show_crate_debug_labels").value
        )
        self.topdown_show_crate_orientation_arrow = bool(
            self.get_parameter("topdown_show_crate_orientation_arrow").value
        )
        self.topdown_show_crate_ab_ends = bool(
            self.get_parameter("topdown_show_crate_ab_ends").value
        )
        self.topdown_show_target_queue_labels = bool(
            self.get_parameter("topdown_show_target_queue_labels").value
        )
        self.topdown_show_game_zones = bool(
            self.get_parameter("topdown_show_game_zones").value
        )
        self.crate_body_length_mm = float(
            self.get_parameter("crate_body_length_mm").value
        )
        self.crate_body_width_mm = float(
            self.get_parameter("crate_body_width_mm").value
        )

        self.main_bot_pre_align_distance_mm = float(
            self.get_parameter("main_bot_pre_align_distance_mm").value
        )
        self.ninja_pre_align_distance_mm = float(
            self.get_parameter("ninja_pre_align_distance_mm").value
        )
        self.pickup_handoff_distance_mm = float(
            self.get_parameter("pickup_handoff_distance_mm").value
        )

        self.compact_command_enabled = bool(
            self.get_parameter("compact_command_enabled").value
        )
        self.compact_command_ttl_ms = int(
            self.get_parameter("compact_command_ttl_ms").value
        )
        self.compact_command_goal_rounding_mm = float(
            self.get_parameter("compact_command_goal_rounding_mm").value
        )
        self.compact_command_heading_rounding_deg = float(
            self.get_parameter("compact_command_heading_rounding_deg").value
        )

        self.aruco_enable_clahe = bool(self.get_parameter("aruco_enable_clahe").value)
        self.aruco_enable_sharpen = bool(self.get_parameter("aruco_enable_sharpen").value)
        self.aruco_second_pass = bool(self.get_parameter("aruco_second_pass").value)
        self.aruco_clahe_clip_limit = float(self.get_parameter("aruco_clahe_clip_limit").value)
        self.aruco_clahe_tile_grid = int(self.get_parameter("aruco_clahe_tile_grid").value)
        self.aruco_sharpen_amount = float(self.get_parameter("aruco_sharpen_amount").value)
        self.aruco_min_marker_perimeter_rate = float(
            self.get_parameter("aruco_min_marker_perimeter_rate").value
        )
        self.aruco_max_marker_perimeter_rate = float(
            self.get_parameter("aruco_max_marker_perimeter_rate").value
        )
        self.aruco_adaptive_thresh_win_size_min = int(
            self.get_parameter("aruco_adaptive_thresh_win_size_min").value
        )
        self.aruco_adaptive_thresh_win_size_max = int(
            self.get_parameter("aruco_adaptive_thresh_win_size_max").value
        )
        self.aruco_adaptive_thresh_win_size_step = int(
            self.get_parameter("aruco_adaptive_thresh_win_size_step").value
        )
        self.aruco_corner_refinement = bool(
            self.get_parameter("aruco_corner_refinement").value
        )
        self.aruco_duplicate_center_px = float(
            self.get_parameter("aruco_duplicate_center_px").value
        )

        self.arena_markers_mm: Dict[int, Tuple[float, float]] = {
            20: (
                float(self.get_parameter("marker_20_x").value),
                float(self.get_parameter("marker_20_y").value),
            ),
            21: (
                float(self.get_parameter("marker_21_x").value),
                float(self.get_parameter("marker_21_y").value),
            ),
            22: (
                float(self.get_parameter("marker_22_x").value),
                float(self.get_parameter("marker_22_y").value),
            ),
            23: (
                float(self.get_parameter("marker_23_x").value),
                float(self.get_parameter("marker_23_y").value),
            ),
        }

        self.arena_ref_ids = set(self.arena_markers_mm.keys())

        # IDs
        self.jenga_ids = {36, 47}
        self.empty_crate_ids = {41}

        # Official Eurobot robot IDs.
        # The exact official tag on the main robot may be unknown until the final robot/match setup.
        # Therefore main_robot_aruco_id only filters the allow-list if
        # filter_to_configured_main_robot_id is true.
        self.blue_official_robot_ids = {1, 2, 3, 4, 5}
        self.yellow_official_robot_ids = {6, 7, 8, 9, 10}

        # Optional team-owned ninja/SIMA helper markers.
        # These require DICT_4X4_100 instead of DICT_4X4_50.
        # Only one of these should normally be used on the physical ninja.
        self.blue_ninja_ids = {55, 56, 57}
        self.yellow_ninja_ids = {75, 76, 77}

        if str(self.team_side).lower() == "blue":
            self.own_official_robot_ids_available = set(self.blue_official_robot_ids)
            self.opponent_official_robot_ids_available = set(self.yellow_official_robot_ids)
            self.own_ninja_ids_available = set(self.blue_ninja_ids)
        else:
            self.own_official_robot_ids_available = set(self.yellow_official_robot_ids)
            self.opponent_official_robot_ids_available = set(self.blue_official_robot_ids)
            self.own_ninja_ids_available = set(self.yellow_ninja_ids)

        self.own_official_robot_ids = set(self.own_official_robot_ids_available)
        if (
            self.filter_to_configured_main_robot_id
            and self.main_robot_aruco_id in self.own_official_robot_ids_available
        ):
            self.own_official_robot_ids = {int(self.main_robot_aruco_id)}

        self.opponent_official_robot_ids = set(self.opponent_official_robot_ids_available)

        self.own_ninja_ids = set(self.own_ninja_ids_available)
        if (
            self.filter_to_configured_ninja_id
            and self.ninja_aruco_id in self.own_ninja_ids_available
        ):
            self.own_ninja_ids = {int(self.ninja_aruco_id)}


        self.configured_id_notes = []
        if self.ninja_aruco_id >= 0 and self.ninja_aruco_id not in self.own_ninja_ids_available:
            self.configured_id_notes.append(
                f"ninja_aruco_id {self.ninja_aruco_id} is not a valid own-side ninja ID for side {self.team_side}"
            )
        if self.main_robot_aruco_id >= 0 and self.main_robot_aruco_id not in self.own_official_robot_ids_available:
            self.configured_id_notes.append(
                f"main_robot_aruco_id {self.main_robot_aruco_id} is not a valid own-side official robot ID for side {self.team_side}"
            )

        self.always_allowed_aruco_ids = (
            set(self.arena_ref_ids)
            | set(self.jenga_ids)
            | set(self.empty_crate_ids)
            | set(self.own_official_robot_ids)
            | set(self.own_ninja_ids)
        )

        self.opponent_candidate_aruco_ids = set(self.opponent_official_robot_ids)

        if self.detect_opponent_robots:
            self.allowed_aruco_ids = (
                set(self.always_allowed_aruco_ids)
                | set(self.opponent_candidate_aruco_ids)
            )
        else:
            self.allowed_aruco_ids = set(self.always_allowed_aruco_ids)

        # Debug counters for IDs ignored by the allow-list.
        self.last_ignored_aruco_ids = []
        self.last_ignored_opponent_candidate_ids = []

        # Lightweight robot tracks keyed by ArUco ID.
        # This smooths x/y/heading and keeps a robot alive for a few missed frames.
        self.robot_tracks = {}

        # Remember selected target to avoid jumping between similar candidates.
        self.target_memory = {
            "main_bot": None,
            "ninja": None,
        }

        # Compact command sequence state.
        # command_seq changes only when the target/goal changes.
        # publish_seq changes every published message.
        self.compact_command_publish_seq = {
            "main_bot": 0,
            "ninja": 0,
        }
        self.compact_command_seq = {
            "main_bot": 0,
            "ninja": 0,
        }
        self.compact_command_last_key = {
            "main_bot": None,
            "ninja": None,
        }

        # Heights above arena floor
        self.jenga_tag_height_main_mm = 30.0
        self.granary_height_mm = 55.0
        self.jenga_tag_height_granary_mm = 85.0

        # Homography
        self.H_px_to_mm: Optional[np.ndarray] = None
        self.last_homography_time = 0.0
        self.last_used_ref_ids = []
        self.last_homography_reprojection_mean_mm = None
        self.last_homography_reprojection_max_mm = None
        self.last_homography_rejected_reason = ""
        self.last_aruco_detection_mode = "startup_full_frame"

        # OpenCV / ROS image bridge
        self.bridge = CvBridge()

        # Optional official arena-map background for the top-down view
        self.topdown_background = None

        # Tracker
        self.crate_tracker = CrateTracker(
            cluster_distance_mm=float(self.get_parameter("cluster_distance_mm").value),
            duplicate_merge_distance_mm=float(self.get_parameter("duplicate_merge_distance_mm").value),
            max_missed_frames=int(self.get_parameter("max_missed_frames").value),
            min_confidence_to_show=int(self.get_parameter("min_confidence_to_show").value),
            smooth_alpha_pos=float(self.get_parameter("smoothing_alpha").value),
            smooth_alpha_angle=float(self.get_parameter("smoothing_alpha_angle").value),
        )

        self.load_topdown_background()

        # ============================================================
        # ROS PUBLISHERS
        # ============================================================

        self.status_pub = self.create_publisher(
            String,
            "/overhead/status",
            10,
        )

        self.detected_ids_pub = self.create_publisher(
            String,
            "/overhead/detected_ids_json",
            10,
        )

        self.world_state_pub = self.create_publisher(
            String,
            "/overhead/world_state_json",
            10,
        )

        self.main_bot_target_pub = self.create_publisher(
            String,
            "/overhead/main_bot_target_json",
            10,
        )

        self.ninja_target_pub = self.create_publisher(
            String,
            "/overhead/ninja_target_json",
            10,
        )

        self.main_bot_target_queue_pub = self.create_publisher(
            String,
            "/overhead/main_bot_target_queue_json",
            10,
        )

        self.ninja_target_queue_pub = self.create_publisher(
            String,
            "/overhead/ninja_target_queue_json",
            10,
        )

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

        self.main_bot_nav_command_pub = self.create_publisher(
            String,
            "/overhead/main_bot_nav_command_json",
            10,
        )

        self.ninja_nav_command_pub = self.create_publisher(
            String,
            "/overhead/ninja_nav_command_json",
            10,
        )

        self.main_bot_compact_command_pub = self.create_publisher(
            String,
            "/overhead/main_bot_compact_command_json",
            10,
        )

        self.ninja_compact_command_pub = self.create_publisher(
            String,
            "/overhead/ninja_compact_command_json",
            10,
        )

        self.opponent_robots_pub = self.create_publisher(
            String,
            "/overhead/opponent_robots_json",
            10,
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            "/overhead/debug_image",
            10,
        )

        self.topdown_image_pub = self.create_publisher(
            Image,
            "/overhead/topdown_image",
            10,
        )

        # ============================================================
        # ARUCO SETUP
        # ============================================================

        self.get_logger().info("Creating ArUco dictionary")

        # Use DICT_4X4_100 so optional ninja IDs 55/56/57 and 75/76/77 can be detected.
        # Safety: after detection we still filter manually to only the allowed IDs.
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_100
        )

        self.aruco_params = self.create_aruco_detector_parameters()

        self.get_logger().info("ArUco dictionary OK: DICT_4X4_100 with allowed-ID filtering")
        self.get_logger().info(f"Allowed ArUco IDs: {sorted(list(self.allowed_aruco_ids))}")
        self.get_logger().info(
            f"Enabled own official IDs: {sorted(list(self.own_official_robot_ids))}, "
            f"enabled own ninja IDs: {sorted(list(self.own_ninja_ids))}, "
            f"opponent detection: {self.detect_opponent_robots}, "
        )
        if self.configured_id_notes:
            self.get_logger().warn("; ".join(self.configured_id_notes))
        self.get_logger().info(
            "ArUco tuned detection: "
            f"clahe={self.aruco_enable_clahe}, "
            f"sharpen={self.aruco_enable_sharpen}, "
            f"second_pass={self.aruco_second_pass}, "
            f"min_perimeter_rate={self.aruco_min_marker_perimeter_rate}"
        )

        # ============================================================
        # CAMERA
        # ============================================================

        self.get_logger().info("Starting overhead camera node with stable crate tracker")
        self.get_logger().info(f"Team side: {self.team_side}")
        self.get_logger().info(f"Device: {self.device}")

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera: {self.device}")
            raise RuntimeError(f"Could not open camera: {self.device}")

        fourcc = cv2.VideoWriter_fourcc(*self.camera_fourcc)
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)

        self.get_logger().info("Camera opened")
        self.get_logger().info(
            f"Actual camera: "
            f"{self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x"
            f"{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} @ "
            f"{self.cap.get(cv2.CAP_PROP_FPS)} fps"
        )

        self.get_logger().info(
            "Tracker settings: "
            f"cluster={self.crate_tracker.cluster_distance_mm} mm, "
            f"merge={self.crate_tracker.duplicate_merge_distance_mm} mm, "
            f"max_missed={self.crate_tracker.max_missed_frames}"
        )

        self.frame_count = 0

        # 5 Hz for now
        self.timer = self.create_timer(0.20, self.timer_callback)

    # ============================================================
    # ARUCO HELPERS
    # ============================================================

    def create_aruco_detector_parameters(self):
        # OpenCV 4.6 on ROS2 Jazzy usually has the legacy API.
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            params = cv2.aruco.DetectorParameters_create()
        else:
            params = cv2.aruco.DetectorParameters()

        def set_if_present(name, value):
            if hasattr(params, name):
                setattr(params, name, value)

        # More permissive for small/far markers in the overhead view.
        set_if_present("adaptiveThreshWinSizeMin", self.aruco_adaptive_thresh_win_size_min)
        set_if_present("adaptiveThreshWinSizeMax", self.aruco_adaptive_thresh_win_size_max)
        set_if_present("adaptiveThreshWinSizeStep", self.aruco_adaptive_thresh_win_size_step)
        set_if_present("adaptiveThreshConstant", 7)

        set_if_present("minMarkerPerimeterRate", self.aruco_min_marker_perimeter_rate)
        set_if_present("maxMarkerPerimeterRate", self.aruco_max_marker_perimeter_rate)
        set_if_present("polygonalApproxAccuracyRate", 0.035)
        set_if_present("minCornerDistanceRate", 0.03)
        set_if_present("minDistanceToBorder", 2)
        set_if_present("minMarkerDistanceRate", 0.03)

        # Better corner precision for pose/angle stability.
        if self.aruco_corner_refinement:
            corner_refine = getattr(cv2.aruco, "CORNER_REFINE_SUBPIX", 1)
        else:
            corner_refine = getattr(cv2.aruco, "CORNER_REFINE_NONE", 0)
        set_if_present("cornerRefinementMethod", corner_refine)
        set_if_present("cornerRefinementWinSize", 5)
        set_if_present("cornerRefinementMaxIterations", 30)
        set_if_present("cornerRefinementMinAccuracy", 0.05)

        # Decode robustness.
        set_if_present("markerBorderBits", 1)
        set_if_present("perspectiveRemovePixelPerCell", 8)
        set_if_present("perspectiveRemoveIgnoredMarginPerCell", 0.13)
        set_if_present("minOtsuStdDev", 3.0)
        set_if_present("errorCorrectionRate", 0.60)

        return params

    def apply_arena_roi_mask_for_aruco(self, gray):
        """
        Restrict ArUco detection to the projected arena rectangle once homography
        is available.

        This approximates your idea of only processing the 3 m x 2 m table area.
        It is a 2D image mask, not a full 3D box/frustum, but it removes most
        outside-table clutter and false positives with very low cost.
        """
        if not self.aruco_enable_arena_roi_mask:
            return gray

        if self.H_px_to_mm is None:
            return gray

        try:
            margin = max(0.0, float(self.arena_roi_margin_mm))
            arena_corners_mm = np.array(
                [[
                    [-margin, -margin],
                    [self.arena_width_mm + margin, -margin],
                    [self.arena_width_mm + margin, self.arena_height_mm + margin],
                    [-margin, self.arena_height_mm + margin],
                ]],
                dtype=np.float32,
            )

            H_inv = np.linalg.inv(self.H_px_to_mm)
            arena_corners_px = cv2.perspectiveTransform(arena_corners_mm, H_inv)[0]
            poly = np.round(arena_corners_px).astype(np.int32)

            mask = np.zeros_like(gray, dtype=np.uint8)
            cv2.fillConvexPoly(mask, poly, 255)

            # Neutral grey outside the arena keeps adaptive thresholding stable.
            out = np.full_like(gray, 127)
            out[mask > 0] = gray[mask > 0]
            return out
        except Exception:
            return gray

    def preprocess_gray_for_aruco(self, gray):
        work = gray

        if self.aruco_enable_clahe:
            tile = max(2, int(self.aruco_clahe_tile_grid))
            clahe = cv2.createCLAHE(
                clipLimit=max(0.1, float(self.aruco_clahe_clip_limit)),
                tileGridSize=(tile, tile),
            )
            work = clahe.apply(work)

        if self.aruco_enable_sharpen:
            amount = max(0.0, float(self.aruco_sharpen_amount))
            blur = cv2.GaussianBlur(work, (0, 0), 1.0)
            work = cv2.addWeighted(work, 1.0 + amount, blur, -amount, 0)

        return work

    def filter_allowed_aruco_detections(self, corners, ids):
        """
        Detect using DICT_4X4_100, but only keep IDs that the game/system expects.
        This lets us support optional ninja tags over 50 without trusting every 0-99 ID.
        """
        if ids is None:
            return [], None

        filtered_corners = []
        filtered_ids = []
        ignored_ids = []
        ignored_opponent_candidate_ids = []

        for marker_id, marker_corners in zip(ids.flatten(), corners):
            marker_id = int(marker_id)
            if marker_id in self.allowed_aruco_ids:
                filtered_corners.append(marker_corners)
                filtered_ids.append(marker_id)
            else:
                ignored_ids.append(marker_id)
                if marker_id in getattr(self, "opponent_candidate_aruco_ids", set()):
                    ignored_opponent_candidate_ids.append(marker_id)

        self.last_ignored_aruco_ids = sorted(list(set(ignored_ids)))
        self.last_ignored_opponent_candidate_ids = sorted(list(set(ignored_opponent_candidate_ids)))

        if not filtered_ids:
            return [], None

        return filtered_corners, np.array(filtered_ids, dtype=np.int32).reshape(-1, 1)

    def detect_aruco_single_pass(self, gray):
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params,
        )
        return corners, ids, rejected

    def merge_aruco_detections(self, base_corners, base_ids, add_corners, add_ids):
        if base_ids is None:
            base_corner_list = []
            base_id_list = []
        else:
            base_corner_list = list(base_corners)
            base_id_list = [int(x) for x in base_ids.flatten()]

        if add_ids is None:
            if base_id_list:
                return base_corner_list, np.array(base_id_list, dtype=np.int32).reshape(-1, 1)
            return [], None

        for marker_id, marker_corners in zip(add_ids.flatten(), add_corners):
            marker_id = int(marker_id)
            center = self.marker_center_px(marker_corners)

            duplicate = False
            for existing_id, existing_corners in zip(base_id_list, base_corner_list):
                if existing_id != marker_id:
                    continue
                existing_center = self.marker_center_px(existing_corners)
                if np.linalg.norm(center - existing_center) <= self.aruco_duplicate_center_px:
                    duplicate = True
                    break

            if not duplicate:
                base_corner_list.append(marker_corners)
                base_id_list.append(marker_id)

        if base_id_list:
            return base_corner_list, np.array(base_id_list, dtype=np.int32).reshape(-1, 1)

        return [], None

    def detect_aruco_on_gray(self, gray):
        """Run first/second-pass ArUco detection on a prepared grayscale image."""
        corners, ids, rejected = self.detect_aruco_single_pass(gray)
        rejected_count = 0 if rejected is None else len(rejected)

        if self.aruco_second_pass and (self.aruco_enable_clahe or self.aruco_enable_sharpen):
            enhanced = self.preprocess_gray_for_aruco(gray)
            corners2, ids2, rejected2 = self.detect_aruco_single_pass(enhanced)
            rejected_count += 0 if rejected2 is None else len(rejected2)
            corners, ids = self.merge_aruco_detections(corners, ids, corners2, ids2)

        corners, ids = self.filter_allowed_aruco_detections(corners, ids)
        return corners, ids, rejected_count

    def detect_aruco(self, gray):
        use_roi = bool(self.aruco_enable_arena_roi_mask and self.H_px_to_mm is not None)

        if use_roi:
            roi_gray = self.apply_arena_roi_mask_for_aruco(gray)
            corners, ids, rejected_count = self.detect_aruco_on_gray(roi_gray)
            self.last_aruco_detection_mode = "arena_roi"

            if ids is None and self.aruco_roi_full_frame_recovery:
                corners2, ids2, rejected_count2 = self.detect_aruco_on_gray(gray)
                if ids2 is not None:
                    corners, ids = corners2, ids2
                    rejected_count += rejected_count2
                    self.last_aruco_detection_mode = "full_frame_recovery"
        else:
            corners, ids, rejected_count = self.detect_aruco_on_gray(gray)
            self.last_aruco_detection_mode = "full_frame"

        return corners, ids, [None] * rejected_count

    @staticmethod
    def marker_center_px(marker_corners):
        pts = marker_corners.reshape(4, 2)
        return np.mean(pts, axis=0)

    @staticmethod
    def marker_size_px(marker_corners):
        pts = marker_corners.reshape(4, 2)

        sides = [
            np.linalg.norm(pts[0] - pts[1]),
            np.linalg.norm(pts[1] - pts[2]),
            np.linalg.norm(pts[2] - pts[3]),
            np.linalg.norm(pts[3] - pts[0]),
        ]

        return float(np.mean(sides))

    # ============================================================
    # HOMOGRAPHY
    # ============================================================

    def compute_homography(self, corners, ids):
        if ids is None:
            return None, []

        best_ref = {}
        for marker_id, marker_corners in zip(ids.flatten(), corners):
            marker_id = int(marker_id)
            if marker_id not in self.arena_markers_mm:
                continue

            size_px = self.marker_size_px(marker_corners)
            if marker_id not in best_ref or size_px > best_ref[marker_id]["size_px"]:
                best_ref[marker_id] = {
                    "center_px": self.marker_center_px(marker_corners),
                    "size_px": size_px,
                }

        used_ids = sorted(best_ref.keys())
        required_ids = sorted(list(self.arena_ref_ids))

        if not set(required_ids).issubset(set(used_ids)):
            self.last_homography_rejected_reason = "missing_unique_reference_id"
            return None, used_ids

        image_points = []
        arena_points = []
        for marker_id in required_ids:
            image_points.append(best_ref[marker_id]["center_px"])
            arena_points.append(self.arena_markers_mm[marker_id])

        image_points = np.array(image_points, dtype=np.float32)
        arena_points = np.array(arena_points, dtype=np.float32)

        H, _ = cv2.findHomography(image_points, arena_points, method=0)
        if H is None or not np.all(np.isfinite(H)):
            self.last_homography_rejected_reason = "find_homography_failed"
            return None, used_ids

        try:
            projected = cv2.perspectiveTransform(image_points.reshape(-1, 1, 2), H).reshape(-1, 2)
            errors = np.linalg.norm(projected - arena_points, axis=1)
            mean_err = float(np.mean(errors))
            max_err = float(np.max(errors))
        except Exception:
            self.last_homography_rejected_reason = "reprojection_check_failed"
            return None, used_ids

        self.last_homography_reprojection_mean_mm = round(mean_err, 2)
        self.last_homography_reprojection_max_mm = round(max_err, 2)

        if max_err > float(self.homography_max_reprojection_error_mm):
            self.last_homography_rejected_reason = f"reprojection_error_too_high:{max_err:.1f}mm"
            return None, used_ids

        self.last_homography_rejected_reason = ""
        return H, required_ids

    @staticmethod
    def px_to_mm(point_px, H):
        src = np.array([[[point_px[0], point_px[1]]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(src, H)
        x_mm, y_mm = dst[0, 0]
        return float(x_mm), float(y_mm)

    @staticmethod
    def mm_to_px(point_mm, H):
        H_inv = np.linalg.inv(H)
        src = np.array([[[point_mm[0], point_mm[1]]]], dtype=np.float32)
        dst = cv2.perspectiveTransform(src, H_inv)
        u, v = dst[0, 0]
        return int(u), int(v)

    # ============================================================
    # HEIGHT CORRECTION
    # ============================================================

    def correct_height_projection(self, x_floor, y_floor, object_height_mm):
        scale = (self.camera_height_mm - object_height_mm) / self.camera_height_mm

        x_corr = self.camera_x_mm + (x_floor - self.camera_x_mm) * scale
        y_corr = self.camera_y_mm + (y_floor - self.camera_y_mm) * scale

        return float(x_corr), float(y_corr)

    def is_in_granary_estimate(self, x_mm, y_mm):
        return 600.0 <= x_mm <= 2400.0 and 1550.0 <= y_mm <= 2000.0

    # ============================================================
    # MARKER ORIENTATION
    # ============================================================

    def marker_angle_deg(self, marker_corners):
        if self.H_px_to_mm is None:
            return None

        pts = marker_corners.reshape(4, 2)

        p0 = self.px_to_mm(pts[0], self.H_px_to_mm)
        p1 = self.px_to_mm(pts[1], self.H_px_to_mm)

        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]

        angle = np.degrees(np.arctan2(dy, dx))
        angle = angle % 360.0

        return float(angle)

    # ============================================================
    # WORLD / ARENA VALIDATION
    # ============================================================

    def is_world_position_plausible(self, x_mm, y_mm, margin_mm):
        """
        Object-specific arena validation after height correction.

        This is intentionally softer than a hard 0..3000 / 0..2000 cut because:
        - robot body centers can be close to walls,
        - high tags project far before correction,
        - calibration/test rigs are not perfectly aligned.
        """
        if not self.world_position_validation_enabled:
            return True

        x = float(x_mm)
        y = float(y_mm)
        m = float(max(0.0, margin_mm))

        return (
            -m <= x <= self.arena_width_mm + m
            and -m <= y <= self.arena_height_mm + m
        )

    def world_validation_note(self, x_mm, y_mm, margin_mm):
        if self.is_world_position_plausible(x_mm, y_mm, margin_mm):
            return "ok"
        return "outside_corrected_world_bounds"

    # ============================================================
    # ROBOT / NINJA DETECTIONS
    # ============================================================

    def robot_marker_kind(self, marker_id):
        marker_id = int(marker_id)

        if marker_id in self.own_official_robot_ids:
            return "own_official_robot"
        if marker_id in self.opponent_official_robot_ids:
            return "opponent_official_robot"
        if marker_id in self.own_ninja_ids:
            return "own_ninja"
        return None

    def robot_marker_height_mm(self, marker_id):
        kind = self.robot_marker_kind(marker_id)

        if kind == "own_ninja":
            return float(self.ninja_tag_height_mm)

        if kind == "own_official_robot":
            return float(self.official_robot_tag_height_mm)

        if kind == "opponent_official_robot":
            if float(self.opponent_official_robot_tag_height_mm) >= 0.0:
                return float(self.opponent_official_robot_tag_height_mm)
            return float(self.official_robot_tag_height_mm)

        return 0.0

    @staticmethod
    def apply_body_center_offset_from_tag(
        tag_x_mm,
        tag_y_mm,
        heading_deg,
        forward_offset_mm,
        lateral_offset_mm,
    ):
        """
        Convert from ArUco tag center to footprint/body center.

        heading_deg points along the local forward axis.
        forward_offset_mm is positive forward from tag center to body center.
        lateral_offset_mm is positive to local left from tag center to body center.
        """
        if heading_deg is None:
            return float(tag_x_mm), float(tag_y_mm)

        a = np.deg2rad(float(heading_deg))

        # Forward axis in arena coordinates.
        fx = np.cos(a)
        fy = np.sin(a)

        # Local left axis.
        lx = -np.sin(a)
        ly = np.cos(a)

        x = float(tag_x_mm) + fx * float(forward_offset_mm) + lx * float(lateral_offset_mm)
        y = float(tag_y_mm) + fy * float(forward_offset_mm) + ly * float(lateral_offset_mm)
        return x, y

    def robot_body_center_from_tag_center(self, kind, tag_x_mm, tag_y_mm, heading_deg):
        """
        Return corrected footprint/body center from the measured ArUco tag center.

        For the main robot we currently assume the official marker is centered
        on the robot footprint.

        For the ninja/SIMA, the tag is not centered:
        - tag is 31.5 mm from the rear along the length direction
        - tag is 10.5 mm offset from the width centerline
        """
        kind = str(kind)

        if "ninja" not in kind:
            return float(tag_x_mm), float(tag_y_mm), {
                "body_center_source": "tag_center_assumed_centered",
                "tag_forward_offset_to_body_center_mm": 0.0,
                "tag_lateral_offset_to_body_center_mm": 0.0,
            }

        # If tag is measured from rear, the body center is half length minus that
        # distance in the forward direction from tag to center.
        forward_offset = float(self.ninja_length_mm) / 2.0 - float(self.ninja_tag_from_rear_mm)

        # Parameter convention:
        # ninja_tag_lateral_offset_mm is the tag offset relative to footprint center,
        # positive to robot-left. Therefore body center relative to tag is negative.
        lateral_offset = -float(self.ninja_tag_lateral_offset_mm)

        x, y = self.apply_body_center_offset_from_tag(
            tag_x_mm,
            tag_y_mm,
            heading_deg,
            forward_offset,
            lateral_offset,
        )

        return x, y, {
            "body_center_source": "tag_center_plus_ninja_offset",
            "ninja_tag_from_rear_mm": round(float(self.ninja_tag_from_rear_mm), 1),
            "ninja_tag_lateral_offset_mm": round(float(self.ninja_tag_lateral_offset_mm), 1),
            "tag_forward_offset_to_body_center_mm": round(float(forward_offset), 1),
            "tag_lateral_offset_to_body_center_mm": round(float(lateral_offset), 1),
            "offset_convention": "heading=forward; positive lateral tag offset means tag is left of body center",
        }

    @staticmethod
    def select_marker_by_config(candidates, configured_id):
        """
        Select a specific marker if configured.
        If configured_id is -1, only auto-select when exactly one candidate is visible.
        This avoids accidentally picking the wrong official robot when multiple markers are seen.
        """
        if configured_id is not None and int(configured_id) >= 0:
            for candidate in candidates:
                if int(candidate.get("aruco_id", -999)) == int(configured_id):
                    return candidate, "configured_visible"

            return None, "configured_not_visible"

        if len(candidates) == 1:
            return candidates[0], "auto_single_visible"

        if len(candidates) == 0:
            return None, "no_candidate_visible"

        return None, "multiple_candidates_config_required"

    def empty_robot_state(self, reason):
        return {
            "team_official_robots": [],
            "opponent_official_robots": [],
            "own_ninja_candidates": [],
            "all": [],
            "all_tracks_raw": [],
            "suppressed_robot_detections": [],
            "suppressed_robot_count": 0,
            "rejected_robot_detections": [],
            "rejected_robot_count": 0,
            "ignored_opponent_candidate_ids": [],

            "main_robot": None,
            "main_robot_selection_status": reason,
            "configured_main_robot_aruco_id": self.main_robot_aruco_id,
            "filter_to_configured_main_robot_id": self.filter_to_configured_main_robot_id,

            "ninja": None,
            "ninja_selection_status": reason,
            "configured_ninja_aruco_id": self.ninja_aruco_id,
            "filter_to_configured_ninja_id": self.filter_to_configured_ninja_id,
        }

    def robot_is_opponent(self, robot):
        return str(robot.get("team_relation", "")) == "opponent" or str(robot.get("kind", "")).startswith("opponent_")

    def opponent_robot_confirmed(self, robot):
        if not self.robot_is_opponent(robot):
            return True

        if not self.detect_opponent_robots:
            return False

        return int(robot.get("confidence", 0)) >= int(self.opponent_robot_min_confidence_to_show)

    def filter_robot_tracks_for_public_output(self, all_tracks):
        """
        Own robot tracks are published immediately.
        Opponent robot tracks are only published when opponent detection is enabled
        and the track has survived long enough to pass the confidence threshold.

        This removes one-frame false-positive yellow/opponent IDs from the map.
        """
        visible = []
        suppressed = []

        for robot in all_tracks:
            r = dict(robot)

            if self.robot_is_opponent(r):
                r["opponent_detection_enabled"] = bool(self.detect_opponent_robots)
                r["opponent_confirmed"] = bool(self.opponent_robot_confirmed(r))
                r["opponent_min_confidence_to_show"] = int(self.opponent_robot_min_confidence_to_show)

                if not r["opponent_confirmed"]:
                    r["suppressed_reason"] = (
                        "opponent_detection_disabled"
                        if not self.detect_opponent_robots
                        else "opponent_confidence_below_threshold"
                    )
                    suppressed.append(r)
                    continue

            visible.append(r)

        return visible, suppressed

    def update_robot_tracks(self, raw_robots):
        """
        Smooth robot/ninja marker detections and keep them alive briefly when
        a marker flickers for a few frames.
        """
        now_frame = int(self.frame_count)
        raw_by_id = {int(r["aruco_id"]): dict(r) for r in raw_robots}

        # Update seen tracks or create new ones.
        for marker_id, raw in raw_by_id.items():
            old = self.robot_tracks.get(marker_id)

            if old is None:
                trk = dict(raw)
                trk.update({
                    "visible_this_frame": True,
                    "missed_frames": 0,
                    "seen_count": 1,
                    "confidence": 1,
                    "last_seen_frame": now_frame,
                    "smoothed": False,
                })
                self.robot_tracks[marker_id] = trk
                continue

            alpha = float(self.robot_smoothing_alpha)
            alpha_angle = float(self.robot_smoothing_alpha_angle)

            smoothed = dict(raw)
            smoothed["x_mm"] = round((1.0 - alpha) * float(old["x_mm"]) + alpha * float(raw["x_mm"]), 1)
            smoothed["y_mm"] = round((1.0 - alpha) * float(old["y_mm"]) + alpha * float(raw["y_mm"]), 1)

            if "tag_center_x_mm" in raw:
                smoothed["tag_center_x_mm"] = round(
                    (1.0 - alpha) * float(old.get("tag_center_x_mm", raw["tag_center_x_mm"]))
                    + alpha * float(raw["tag_center_x_mm"]),
                    1,
                )
            if "tag_center_y_mm" in raw:
                smoothed["tag_center_y_mm"] = round(
                    (1.0 - alpha) * float(old.get("tag_center_y_mm", raw["tag_center_y_mm"]))
                    + alpha * float(raw["tag_center_y_mm"]),
                    1,
                )

            smoothed["x_floor_projection_mm"] = round(
                (1.0 - alpha) * float(old.get("x_floor_projection_mm", raw["x_floor_projection_mm"]))
                + alpha * float(raw["x_floor_projection_mm"]),
                1,
            )
            smoothed["y_floor_projection_mm"] = round(
                (1.0 - alpha) * float(old.get("y_floor_projection_mm", raw["y_floor_projection_mm"]))
                + alpha * float(raw["y_floor_projection_mm"]),
                1,
            )
            smoothed["heading_deg"] = None if raw.get("heading_deg") is None else round(
                smooth_angle_deg(old.get("heading_deg"), raw.get("heading_deg"), alpha_angle),
                1,
            )
            smoothed["size_px"] = round(
                (1.0 - alpha) * float(old.get("size_px", raw["size_px"])) + alpha * float(raw["size_px"]),
                1,
            )
            smoothed.update({
                "visible_this_frame": True,
                "missed_frames": 0,
                "seen_count": int(old.get("seen_count", 0)) + 1,
                "confidence": min(20, int(old.get("confidence", 0)) + 2),
                "last_seen_frame": now_frame,
                "smoothed": True,
            })
            self.robot_tracks[marker_id] = smoothed

        # Age tracks not seen this frame.
        for marker_id in list(self.robot_tracks.keys()):
            if marker_id in raw_by_id:
                continue

            trk = self.robot_tracks[marker_id]
            trk["missed_frames"] = int(trk.get("missed_frames", 0)) + 1
            trk["confidence"] = max(0, int(trk.get("confidence", 0)) - 1)
            trk["visible_this_frame"] = False
            trk["held_from_last_detection"] = True

            if trk["missed_frames"] > int(self.robot_track_max_missed_frames):
                del self.robot_tracks[marker_id]

        return sorted(
            [dict(v) for v in self.robot_tracks.values()],
            key=lambda r: (str(r.get("kind", "")), int(r.get("aruco_id", 999))),
        )

    def extract_robot_detections(self, corners, ids):
        raw_robots = []
        rejected_robot_detections = []
        detection_note = "ok"

        if ids is None:
            detection_note = "no_markers_detected"
        elif self.H_px_to_mm is None:
            detection_note = "homography_inactive"
        else:
            for marker_id, marker_corners in zip(ids.flatten(), corners):
                marker_id = int(marker_id)
                kind = self.robot_marker_kind(marker_id)

                if kind is None:
                    continue

                center_px = self.marker_center_px(marker_corners)
                x_floor, y_floor = self.px_to_mm(center_px, self.H_px_to_mm)
                tag_height = self.robot_marker_height_mm(marker_id)

                x_corr, y_corr = self.correct_height_projection(
                    x_floor,
                    y_floor,
                    tag_height,
                )

                heading_deg = self.marker_angle_deg(marker_corners)
                heading_out = None if heading_deg is None else round(float(heading_deg), 1)

                body_x, body_y, offset_info = self.robot_body_center_from_tag_center(
                    kind,
                    x_corr,
                    y_corr,
                    heading_deg,
                )

                world_note = self.world_validation_note(
                    body_x,
                    body_y,
                    self.robot_world_margin_mm,
                )
                if world_note != "ok":
                    # Keep debug info so we can understand why a yellow/opponent
                    # marker was detected but not added to the top-down map.
                    rejected_robot_detections.append({
                        "aruco_id": marker_id,
                        "kind": kind,
                        "team_relation": "own" if kind.startswith("own_") else "opponent",
                        "rejected_reason": world_note,
                        "assumed_tag_height_mm": round(float(tag_height), 1),
                        "x_mm_after_height_correction": round(float(body_x), 1),
                        "y_mm_after_height_correction": round(float(body_y), 1),
                        "tag_center_x_mm_after_height_correction": round(float(x_corr), 1),
                        "tag_center_y_mm_after_height_correction": round(float(y_corr), 1),
                        "x_floor_projection_mm": round(float(x_floor), 1),
                        "y_floor_projection_mm": round(float(y_floor), 1),
                        "heading_deg": heading_out,
                        "size_px": round(float(self.marker_size_px(marker_corners)), 1),
                        "debug_note": (
                            "If this is a loose test marker lying flat on the floor, "
                            "set opponent_official_robot_tag_height_mm to 0.0."
                        ),
                    })
                    continue

                robot_msg = {
                    "aruco_id": marker_id,
                    "kind": kind,
                    "team_relation": "own" if kind.startswith("own_") else "opponent",

                    # x_mm/y_mm are the footprint/body center used for planning.
                    "x_mm": round(float(body_x), 1),
                    "y_mm": round(float(body_y), 1),
                    "world_validation": world_note,

                    # Tag center is kept separately for debugging and calibration.
                    "tag_center_x_mm": round(float(x_corr), 1),
                    "tag_center_y_mm": round(float(y_corr), 1),

                    # Floor projection is the uncorrected homography projection of the tag.
                    "x_floor_projection_mm": round(float(x_floor), 1),
                    "y_floor_projection_mm": round(float(y_floor), 1),
                    "tag_height_mm": round(float(tag_height), 1),
                    "heading_deg": heading_out,
                    "heading_note": "ArUco top-edge direction; align tag top edge with robot forward direction",
                    "size_px": round(float(self.marker_size_px(marker_corners)), 1),
                }

                if kind in ("own_official_robot", "opponent_official_robot"):
                    robot_msg["top_plate_color_mode"] = self.main_robot_top_plate_color_mode
                    robot_msg["top_plate_color_note"] = (
                        "current test box is MDF/wood; future main bot expected black"
                    )

                robot_msg.update(offset_info)
                raw_robots.append(robot_msg)

        all_tracks_raw = self.update_robot_tracks(raw_robots)
        all_robots, suppressed_robot_detections = self.filter_robot_tracks_for_public_output(
            all_tracks_raw
        )

        team_official = [r for r in all_robots if r["kind"] == "own_official_robot"]
        opponent_official = [r for r in all_robots if r["kind"] == "opponent_official_robot"]
        own_ninja = [r for r in all_robots if r["kind"] == "own_ninja"]

        main_robot, main_status = self.select_marker_by_config(
            team_official,
            self.main_robot_aruco_id,
        )
        ninja, ninja_status = self.select_marker_by_config(
            own_ninja,
            self.ninja_aruco_id,
        )

        return {
            "team_official_robots": team_official,
            "opponent_official_robots": opponent_official,
            "own_ninja_candidates": own_ninja,
            "all": all_robots,
            "all_tracks_raw": all_tracks_raw,
            "suppressed_robot_detections": suppressed_robot_detections,
            "suppressed_robot_count": len(suppressed_robot_detections),
            "rejected_robot_detections": rejected_robot_detections,
            "raw_visible_count": len(raw_robots),
            "rejected_robot_count": len(rejected_robot_detections),
            "ignored_opponent_candidate_ids": list(getattr(self, "last_ignored_opponent_candidate_ids", [])),
            "detection_note": detection_note,

            "main_robot": main_robot,
            "main_robot_selection_status": main_status,
            "configured_main_robot_aruco_id": self.main_robot_aruco_id,
            "filter_to_configured_main_robot_id": self.filter_to_configured_main_robot_id,

            "ninja": ninja,
            "ninja_selection_status": ninja_status,
            "configured_ninja_aruco_id": self.ninja_aruco_id,
            "filter_to_configured_ninja_id": self.filter_to_configured_ninja_id,
        }

    # ============================================================
    # RAW CRATE DETECTIONS
    # ============================================================

    def extract_raw_crate_detections(self, corners, ids):
        if ids is None or self.H_px_to_mm is None:
            return []

        detections = []

        for marker_id, marker_corners in zip(ids.flatten(), corners):
            marker_id = int(marker_id)

            if marker_id not in self.jenga_ids and marker_id not in self.empty_crate_ids:
                continue

            center_px = self.marker_center_px(marker_corners)
            x_floor, y_floor = self.px_to_mm(center_px, self.H_px_to_mm)

            if self.is_in_granary_estimate(x_floor, y_floor):
                tag_height = self.jenga_tag_height_granary_mm
                zone = "granary_estimate"
            else:
                tag_height = self.jenga_tag_height_main_mm
                zone = "main_area_estimate"

            x_corr, y_corr = self.correct_height_projection(
                x_floor,
                y_floor,
                tag_height,
            )

            world_note = self.world_validation_note(
                x_corr,
                y_corr,
                self.crate_world_margin_mm,
            )
            if world_note != "ok":
                continue

            angle_deg = self.marker_angle_deg(marker_corners)
            long_axis_deg = angle_deg

            if marker_id == 36:
                crate_type = "blue"
            elif marker_id == 47:
                crate_type = "yellow"
            elif marker_id == 41:
                crate_type = "empty_black"
            else:
                crate_type = "unknown"

            detections.append({
                "aruco_id": marker_id,
                "crate_type": crate_type,

                "x_mm": x_corr,
                "y_mm": y_corr,
                "world_validation": world_note,

                "x_floor_projection_mm": x_floor,
                "y_floor_projection_mm": y_floor,

                "tag_height_mm": tag_height,
                "zone_estimate": zone,

                "long_axis_deg": long_axis_deg,
                "size_px": self.marker_size_px(marker_corners),
            })

        return detections

    # ============================================================
    # DRAWING
    # ============================================================

    def draw_arena_grid_on_overlay(self, overlay):
        if self.H_px_to_mm is None:
            return overlay

        arena_corners = [
            (0.0, 0.0),
            (self.arena_width_mm, 0.0),
            (self.arena_width_mm, self.arena_height_mm),
            (0.0, self.arena_height_mm),
        ]

        try:
            pts = [self.mm_to_px(p, self.H_px_to_mm) for p in arena_corners]
            pts_np = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(overlay, [pts_np], isClosed=True, color=(0, 255, 0), thickness=3)

            for x in range(0, int(self.arena_width_mm) + 1, 500):
                p1 = self.mm_to_px((x, 0), self.H_px_to_mm)
                p2 = self.mm_to_px((x, self.arena_height_mm), self.H_px_to_mm)
                cv2.line(overlay, p1, p2, (0, 160, 0), 1)

            for y in range(0, int(self.arena_height_mm) + 1, 500):
                p1 = self.mm_to_px((0, y), self.H_px_to_mm)
                p2 = self.mm_to_px((self.arena_width_mm, y), self.H_px_to_mm)
                cv2.line(overlay, p1, p2, (0, 160, 0), 1)

        except Exception:
            pass

        return overlay

    def draw_debug_overlay(self, frame, corners, ids, raw_crate_detections, stable_crates, rejected_count):
        overlay = frame.copy()

        if self.H_px_to_mm is not None:
            overlay = self.draw_arena_grid_on_overlay(overlay)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(overlay, corners, ids)

            for marker_id, marker_corners in zip(ids.flatten(), corners):
                marker_id = int(marker_id)
                center = self.marker_center_px(marker_corners)
                u, v = int(center[0]), int(center[1])
                size_px = self.marker_size_px(marker_corners)

                if marker_id in self.arena_ref_ids:
                    color = self.BGR_REF
                    kind = "REF"
                elif marker_id in self.jenga_ids:
                    color = self.BGR_TEAM_BLUE if marker_id == 36 else self.BGR_TEAM_YELLOW
                    kind = "RAW_CRATE"
                elif marker_id in self.empty_crate_ids:
                    color = self.BGR_EMPTY_BLACK
                    kind = "RAW_EMPTY"
                else:
                    color = (200, 200, 200)
                    kind = "OTHER"

                label = f"ID {marker_id} {kind} {size_px:.0f}px"

                if self.H_px_to_mm is not None:
                    x, y = self.px_to_mm(center, self.H_px_to_mm)
                    label += f" ({x:.0f},{y:.0f})"

                cv2.circle(overlay, (u, v), 8, color, -1)
                cv2.putText(
                    overlay,
                    label,
                    (u + 10, v - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    color,
                    2,
                )

        # Draw stable tracks
        if self.H_px_to_mm is not None:
            for crate in stable_crates:
                try:
                    px = self.mm_to_px(
                        (crate["x_floor_projection_mm"], crate["y_floor_projection_mm"]),
                        self.H_px_to_mm,
                    )

                    if crate["missed_frames"] == 0:
                        color = (0, 255, 0)
                        status = "LIVE"
                    else:
                        color = (0, 165, 255)
                        status = f"HELD{crate['missed_frames']}"

                    cv2.circle(overlay, px, 18, color, 3)

                    label = (
                        f"J{crate['track_id']} ID{crate['aruco_id']} "
                        f"{status} C{crate['confidence']} "
                        f"x={crate['x_mm']} y={crate['y_mm']} "
                        f"a={crate['long_axis_deg']}"
                    )

                    cv2.putText(
                        overlay,
                        label,
                        (px[0] + 18, px[1] + 18),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        color,
                        2,
                    )

                    # Draw long-axis direction arrow
                    if crate["long_axis_deg"] is not None:
                        angle_rad = np.deg2rad(crate["long_axis_deg"])
                        length_px = 80

                        p1 = (
                            int(px[0] - np.cos(angle_rad) * length_px / 2),
                            int(px[1] - np.sin(angle_rad) * length_px / 2),
                        )
                        p2 = (
                            int(px[0] + np.cos(angle_rad) * length_px / 2),
                            int(px[1] + np.sin(angle_rad) * length_px / 2),
                        )

                        cv2.arrowedLine(overlay, p1, p2, color, 3, tipLength=0.25)

                except Exception:
                    pass

        detected_ids = [] if ids is None else ids.flatten().astype(int).tolist()
        missing_refs = sorted(list(self.arena_ref_ids - set(detected_ids)))

        y = 40

        def put(text, color=(255, 255, 255)):
            nonlocal y
            cv2.putText(
                overlay,
                text,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                color,
                2,
            )
            y += 36

        put("ROS2 Overhead Debug Feed - Phase 2A")
        put(f"team_side: {self.team_side}")
        put(f"ids: {detected_ids}", (255, 255, 0))
        put(f"rejected: {rejected_count}", (200, 200, 200))

        if missing_refs:
            put(f"missing refs: {missing_refs}", (0, 0, 255))
        else:
            put("all arena refs visible", (0, 255, 0))

        if self.H_px_to_mm is None:
            put("homography: NOT ACTIVE", (0, 0, 255))
        else:
            put("homography: ACTIVE", (0, 255, 0))

        put(f"raw crates: {len(raw_crate_detections)}", (255, 255, 0))
        put(f"stable crates: {len(stable_crates)}", (0, 255, 0))

        return overlay

    # ============================================================
    # TOP-DOWN MAP DRAWING
    # ============================================================

    def topdown_px(self, x_mm, y_mm, scale=0.30, margin=40):
        """
        Convert arena millimetres to top-down image pixels.

        Arena coordinate system:
          x = 0..3000 left to right
          y = 0..2000 front to rear

        Image coordinate system:
          rear/high y appears near the top
          front/low y appears near the bottom
        """
        px = margin + int(float(x_mm) * scale)
        py = margin + int((self.arena_height_mm - float(y_mm)) * scale)
        return px, py

    def draw_rect_topdown(self, img, x1, y1, x2, y2, color, label=None, thickness=2):
        p1 = self.topdown_px(x1, y1)
        p2 = self.topdown_px(x2, y2)

        left = min(p1[0], p2[0])
        right = max(p1[0], p2[0])
        top = min(p1[1], p2[1])
        bottom = max(p1[1], p2[1])

        cv2.rectangle(img, (left, top), (right, bottom), color, thickness)

        if label:
            cv2.putText(
                img,
                label,
                (left + 5, top + 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )

    def load_topdown_background(self):
        """
        Load the optional official arena image for /overhead/topdown_image.

        The node will try, in order:
        1. the exact topdown_background_path parameter if it is set
        2. installed package share/config image names
        3. common source-workspace config paths

        The image is converted from OpenCV BGR to RGB because the top-down
        stream is published as rgb8.
        """
        self.topdown_background = None

        candidate_paths = []

        def add_candidate(path):
            if path is None:
                return
            path = str(path).strip()
            if not path:
                return
            if path not in candidate_paths:
                candidate_paths.append(path)

        # 1. Exact path from YAML/launch parameter
        add_candidate(self.topdown_background_path)

        image_names = [
            "arena_map_with_coordinate_overlay.png",
            "arena_map_with_cordinate_overlay.png",
            "Arena_map_with_coordinate_overlay.png",
            "Arena_map_with_cordinate_overlay.png",
        ]

        # 2. Installed package share directory
        try:
            pkg_share = Path(get_package_share_directory("overhead_control"))
            for name in image_names:
                add_candidate(pkg_share / "config" / name)
        except Exception as e:
            self.get_logger().warn(f"Could not resolve package share directory: {e}")

        # 3. Common source workspace paths
        common_roots = [
            Path.cwd(),
            Path.home() / "Master" / "Eurobot2026MTP",
            Path.home() / "Eurobot2026MTP",
        ]
        for root in common_roots:
            for name in image_names:
                add_candidate(root / "src" / "overhead_control" / "config" / name)
                add_candidate(root / "overhead_control" / "config" / name)

        for path_str in candidate_paths:
            path = Path(path_str).expanduser()
            if not path.exists():
                continue

            try:
                bg_bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if bg_bgr is None:
                    continue

                self.topdown_background = cv2.cvtColor(bg_bgr, cv2.COLOR_BGR2RGB)
                self.get_logger().info(
                    f"Loaded top-down background image: {path} "
                    f"shape={bg_bgr.shape[1]}x{bg_bgr.shape[0]}"
                )
                return
            except Exception as e:
                self.get_logger().warn(f"Failed loading top-down background {path}: {e}")

        self.get_logger().warn("Top-down background image not found. Using synthetic map mode.")
        self.get_logger().warn("Tried background paths: " + "; ".join(candidate_paths[:12]))

    def draw_rect_topdown_filled(self, img, x1, y1, x2, y2, border_color, fill_color=None, label=None, thickness=2):
        p1 = self.topdown_px(x1, y1)
        p2 = self.topdown_px(x2, y2)

        left = min(p1[0], p2[0])
        right = max(p1[0], p2[0])
        top = min(p1[1], p2[1])
        bottom = max(p1[1], p2[1])

        if fill_color is not None:
            overlay = img.copy()
            cv2.rectangle(overlay, (left, top), (right, bottom), fill_color, -1)
            cv2.addWeighted(overlay, 0.22, img, 0.78, 0, dst=img)

        cv2.rectangle(img, (left, top), (right, bottom), border_color, thickness)

        if label:
            cv2.putText(
                img,
                label,
                (left + 5, top + 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                border_color,
                2,
            )

    # ============================================================
    # APPROACH POSE GENERATION
    # ============================================================

    @staticmethod
    def point_in_rect(x, y, rect):
        x1, y1, x2, y2 = rect
        return min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2)

    def point_inside_arena_for_main_bot(self, x, y):
        margin = self.approach_wall_margin_mm
        return (
            margin <= x <= self.arena_width_mm - margin
            and margin <= y <= self.arena_height_mm - margin
        )

    def granary_rect(self):
        # Approximation used by the current top-down planner/debugger.
        return (600.0, 1550.0, 2400.0, 2000.0)

    def yellow_nest_rect(self):
        return (0.0, 1550.0, 600.0, 2000.0)

    def blue_nest_rect(self):
        return (2400.0, 1550.0, 3000.0, 2000.0)

    def opponent_nest_rect(self):
        if str(self.team_side).lower() == "blue":
            return self.yellow_nest_rect()
        return self.blue_nest_rect()

    def validate_main_bot_approach_point(self, x, y):
        if not self.point_inside_arena_for_main_bot(x, y):
            return False, "outside_arena_or_wall_margin"

        if self.point_in_rect(x, y, self.granary_rect()):
            return False, "inside_granary_main_bot_forbidden"

        if self.point_in_rect(x, y, self.opponent_nest_rect()):
            return False, "inside_opponent_nest_forbidden"

        return True, "ok"

    def validate_ninja_body_point(self, x, y):
        """
        Check where the ninja/SIMA body is allowed to move.

        Current strategy:
        - Ninja stays in the granary rectangle.
        - It does not drive down into the main arena.
        """
        if not self.ninja_keep_body_inside_granary:
            return True, "ok_ninja_granary_body_limit_disabled"

        if self.point_in_rect(x, y, self.granary_rect()):
            return True, "ok"

        return False, "ninja_body_outside_granary_forbidden"

    def validate_ninja_target_crate_reach(self, crate_x, crate_y):
        """
        Check whether the ninja is allowed to target this crate.

        With reach = 0:
            crate center must be inside granary.

        With reach > 0:
            crate center may be just below/outside the granary line, but only
            within the allowed actuator projection reach. The ninja body still
            stays inside the granary.
        """
        if not self.ninja_target_requires_granary_or_reach:
            return True, "ok_ninja_target_zone_limit_disabled"

        granary = self.granary_rect()

        if self.point_in_rect(crate_x, crate_y, granary):
            return True, "ok_crate_inside_granary"

        reach = float(max(0.0, self.ninja_actuator_projection_reach_mm))
        if reach <= 0.0:
            return False, "crate_outside_granary_ninja_forbidden"

        gx1, gy1, gx2, gy2 = granary
        xmin = min(gx1, gx2)
        xmax = max(gx1, gx2)
        line_y = min(gy1, gy2)

        # Current coordinate convention:
        # granary is the high-y band: y = 1550..2000.
        # main area is above it: y < 1550.
        # Projection over the line can reach from y=1550 down/up into the main area
        # by `ninja_actuator_projection_reach_mm`.
        if xmin <= crate_x <= xmax and (line_y - reach) <= crate_y <= line_y:
            return True, "ok_crate_within_actuator_projection_reach"

        return False, "crate_outside_ninja_actuator_reach"

    def validate_ninja_approach_to_crate(self, approach_x, approach_y, crate_x, crate_y):
        body_ok, body_reason = self.validate_ninja_body_point(approach_x, approach_y)
        if not body_ok:
            return False, body_reason

        target_ok, target_reason = self.validate_ninja_target_crate_reach(crate_x, crate_y)
        if not target_ok:
            return False, target_reason

        return True, target_reason

    def validate_ninja_approach_point(self, x, y):
        # Backward-compatible helper used by navigation pre-align validation.
        return self.validate_ninja_body_point(x, y)

    def enrich_crates_with_approach_poses(self, stable_crates):
        """
        Add approach poses to each stable crate.

        Convention:
        - long_axis_deg is the direction along the crate length.
        - end_a is behind the crate along -axis, heading toward +axis.
        - end_b is behind the crate along +axis, heading toward -axis.

        Each candidate includes:
        - x_mm, y_mm, heading_deg
        - valid_for_main_bot and reason
        - valid_for_ninja and reason
        """
        enriched = []

        for crate in stable_crates:
            crate = dict(crate)
            candidates = []

            angle_deg = crate.get("long_axis_deg")
            if angle_deg is not None:
                x = float(crate["x_mm"])
                y = float(crate["y_mm"])
                a = np.deg2rad(float(angle_deg))
                dx = np.cos(a)
                dy = np.sin(a)

                half_l = float(self.crate_body_length_mm) / 2.0
                crate["display_label"] = self.short_crate_label(crate)
                crate["front_back_defined"] = False
                crate["orientation_convention"] = (
                    "long_axis_deg is the ArUco top-edge direction. "
                    "A-end is the negative long-axis end; B-end is the positive long-axis end. "
                    "No physical front/back is assumed yet."
                )
                crate["tag_forward_heading_deg"] = round(float(angle_deg), 1)
                crate["end_a"] = {
                    "name": "A",
                    "x_mm": round(float(x - dx * half_l), 1),
                    "y_mm": round(float(y - dy * half_l), 1),
                    "role": "negative_long_axis_end",
                    "approach_heading_deg": round(normalize_angle_deg(float(angle_deg)), 1),
                }
                crate["end_b"] = {
                    "name": "B",
                    "x_mm": round(float(x + dx * half_l), 1),
                    "y_mm": round(float(y + dy * half_l), 1),
                    "role": "positive_long_axis_end",
                    "approach_heading_deg": round(normalize_angle_deg(float(angle_deg) + 180.0), 1),
                }

                # Generate several distances along the crate long axis.
                # The nominal distance is used first. Shorter fallback distances help
                # when crates are close to a wall or packed tightly together.
                nominal_d = float(self.crate_approach_distance_mm)
                approach_distances = []
                for d in [nominal_d, 180.0, 140.0, 110.0]:
                    d = float(max(60.0, d))
                    if all(abs(d - old_d) > 1.0 for old_d in approach_distances):
                        approach_distances.append(d)

                raw_candidates = []
                for d in approach_distances:
                    d_name = int(round(d))
                    raw_candidates.extend([
                        (
                            f"end_a_{d_name}",
                            x - dx * d,
                            y - dy * d,
                            normalize_angle_deg(float(angle_deg)),
                            d,
                        ),
                        (
                            f"end_b_{d_name}",
                            x + dx * d,
                            y + dy * d,
                            normalize_angle_deg(float(angle_deg) + 180.0),
                            d,
                        ),
                    ])

                for name, px, py, heading, distance_mm in raw_candidates:
                    main_ok, main_reason = self.validate_main_bot_approach_point(px, py)
                    ninja_ok, ninja_reason = self.validate_ninja_approach_to_crate(
                        px,
                        py,
                        x,
                        y,
                    )

                    candidates.append({
                        "name": name,
                        "x_mm": round(float(px), 1),
                        "y_mm": round(float(py), 1),
                        "heading_deg": round(float(heading), 1),
                        "approach_distance_mm": round(float(distance_mm), 1),
                        "valid_for_main_bot": bool(main_ok),
                        "main_bot_reason": main_reason,
                        "valid_for_ninja": bool(ninja_ok),
                        "ninja_reason": ninja_reason,
                    })

            if angle_deg is None:
                crate["display_label"] = self.short_crate_label(crate)
                crate["front_back_defined"] = False
                crate["orientation_convention"] = (
                    "long_axis_deg unavailable; crate end labels and orientation are not reliable this frame."
                )
                crate["tag_forward_heading_deg"] = None
                crate["end_a"] = None
                crate["end_b"] = None

            ninja_target_ok, ninja_target_reason = self.validate_ninja_target_crate_reach(
                float(crate["x_mm"]),
                float(crate["y_mm"]),
            )
            crate["ninja_target_zone_ok"] = bool(ninja_target_ok)
            crate["ninja_target_zone_reason"] = ninja_target_reason
            crate["ninja_actuator_projection_reach_mm"] = round(
                float(self.ninja_actuator_projection_reach_mm),
                1,
            )

            crate["approach_pose_candidates"] = candidates

            main_candidates = [c for c in candidates if c["valid_for_main_bot"]]
            ninja_candidates = [c for c in candidates if c["valid_for_ninja"]]

            crate["best_main_bot_approach_pose"] = (
                main_candidates[0] if main_candidates else None
            )
            crate["best_ninja_approach_pose"] = (
                ninja_candidates[0] if ninja_candidates else None
            )

            enriched.append(crate)

        return enriched

    # ============================================================
    # SIMPLE TARGET SELECTION
    # ============================================================

    @staticmethod
    def signed_angle_error_deg(target_heading_deg, current_heading_deg):
        """
        Signed shortest angular error from current heading to target heading.
        Positive means rotate counter-clockwise in the arena coordinate convention.
        """
        if target_heading_deg is None or current_heading_deg is None:
            return None
        return float((float(target_heading_deg) - float(current_heading_deg) + 180.0) % 360.0 - 180.0)

    @staticmethod
    def distance_between_points_mm(a, b):
        return float(np.hypot(float(a["x_mm"]) - float(b["x_mm"]), float(a["y_mm"]) - float(b["y_mm"])))

    def build_target_candidates_for_actor(self, actor_name, actor_pose, stable_crates, valid_key):
        """
        Build one best target candidate per crate, sorted by straight-line distance.

        Multiple approach distances are still evaluated, but the public queue only
        exposes the best approach for each crate. This keeps the top-down view and
        command output much cleaner.
        """
        if actor_pose is None:
            return []

        best_by_track = {}
        all_count = 0

        for crate in stable_crates:
            if int(crate.get("aruco_id", -1)) == 41:
                continue

            track_id = crate.get("track_id")

            for approach in crate.get("approach_pose_candidates", []):
                if not approach.get(valid_key, False):
                    continue

                all_count += 1
                distance = self.distance_between_points_mm(actor_pose, approach)
                heading_error = self.signed_angle_error_deg(
                    approach.get("heading_deg"),
                    actor_pose.get("heading_deg"),
                )

                candidate = {
                    "rank": None,
                    "actor": actor_name,
                    "target_crate": {
                        "track_id": crate.get("track_id"),
                        "aruco_id": crate.get("aruco_id"),
                        "crate_type": crate.get("crate_type"),
                        "x_mm": crate.get("x_mm"),
                        "y_mm": crate.get("y_mm"),
                        "long_axis_deg": crate.get("long_axis_deg"),
                    },
                    "approach_pose": approach,
                    "distance_to_approach_mm": round(float(distance), 1),
                    "heading_error_deg": None if heading_error is None else round(float(heading_error), 1),
                    "selection_method": "nearest_valid_approach_pose_one_per_crate_v2",
                    "all_valid_approaches_considered": all_count,
                }

                old = best_by_track.get(track_id)
                if old is None or candidate["distance_to_approach_mm"] < old["distance_to_approach_mm"]:
                    best_by_track[track_id] = candidate

        candidates = sorted(best_by_track.values(), key=lambda c: c["distance_to_approach_mm"])

        for i, candidate in enumerate(candidates, start=1):
            candidate["rank"] = i

        return candidates

    @staticmethod
    def candidate_key(candidate):
        if candidate is None:
            return None
        crate = candidate.get("target_crate") or {}
        approach = candidate.get("approach_pose") or {}
        return (
            crate.get("track_id"),
            approach.get("name"),
        )

    def choose_target_with_hysteresis(self, actor_name, queue):
        """
        Keep the previous target unless a new target is clearly closer.
        This prevents target lines from jumping between similar crates every frame.
        """
        if not queue:
            return None, "no_queue"

        best = queue[0]
        memory = self.target_memory.get(actor_name)
        hysteresis = float(self.target_switch_distance_hysteresis_mm)

        if memory is not None:
            remembered_key = memory.get("key")
            remembered = None
            for candidate in queue:
                if self.candidate_key(candidate) == remembered_key:
                    remembered = candidate
                    break

            if remembered is not None:
                best_dist = float(best["distance_to_approach_mm"])
                remembered_dist = float(remembered["distance_to_approach_mm"])
                if best_dist + hysteresis >= remembered_dist:
                    self.target_memory[actor_name] = {
                        "key": self.candidate_key(remembered),
                        "last_distance_mm": remembered_dist,
                    }
                    return remembered, "held_previous_target_hysteresis"

        self.target_memory[actor_name] = {
            "key": self.candidate_key(best),
            "last_distance_mm": float(best["distance_to_approach_mm"]),
        }
        return best, "selected_nearest"

    def build_target_for_actor(self, actor_name, actor_pose, stable_crates, valid_key):
        """
        Pick the first item from the ranked target candidate queue.
        """
        target = {
            "active": False,
            "actor": actor_name,
            "reason": "unknown",
            "robot": actor_pose,
            "target_crate": None,
            "approach_pose": None,
            "distance_to_approach_mm": None,
            "heading_error_deg": None,
            "selection_method": "nearest_valid_approach_pose_one_per_crate_hysteresis_v2",
            "queue_length": 0,
            "top_candidates": [],
        }

        if actor_pose is None:
            target["reason"] = "actor_not_visible_or_not_selected"
            return target, []

        queue = self.build_target_candidates_for_actor(
            actor_name=actor_name,
            actor_pose=actor_pose,
            stable_crates=stable_crates,
            valid_key=valid_key,
        )

        target["queue_length"] = len(queue)
        target["top_candidates"] = queue[:5]

        if not queue:
            target["reason"] = "no_valid_approach_pose"
            return target, queue

        best, switch_reason = self.choose_target_with_hysteresis(actor_name, queue)
        target.update({
            "active": best is not None,
            "reason": "ok" if best is not None else "no_valid_approach_pose",
            "target_switch_reason": switch_reason,
            "target_crate": None if best is None else best["target_crate"],
            "approach_pose": None if best is None else best["approach_pose"],
            "distance_to_approach_mm": None if best is None else best["distance_to_approach_mm"],
            "heading_error_deg": None if best is None else best["heading_error_deg"],
        })

        return target, queue

    def build_targets(self, robots, stable_crates):
        main_target, main_queue = self.build_target_for_actor(
            actor_name="main_bot",
            actor_pose=robots.get("main_robot"),
            stable_crates=stable_crates,
            valid_key="valid_for_main_bot",
        )

        ninja_target, ninja_queue = self.build_target_for_actor(
            actor_name="ninja",
            actor_pose=robots.get("ninja"),
            stable_crates=stable_crates,
            valid_key="valid_for_ninja",
        )

        return {
            "main_bot": main_target,
            "ninja": ninja_target,
            "main_bot_queue": main_queue,
            "ninja_queue": ninja_queue,
        }

    # ============================================================
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

    # ============================================================
    # ROBOT FOOTPRINT DRAWING
    # ============================================================

    def strong_team_color_rgb(self, side):
        """Bright team colors for top-down robot footprint outlines."""
        side = str(side).lower()
        if side == "blue":
            return (0, 170, 255)
        return (255, 230, 0)

    def robot_display_color_rgb(self, robot):
        """Use side-colored footprints so own/opponent robots are visually distinct."""
        relation = robot.get("team_relation", "own")
        own_side = str(self.team_side).lower()
        opponent_side = "yellow" if own_side == "blue" else "blue"
        side = own_side if relation == "own" else opponent_side
        return self.strong_team_color_rgb(side)

    @staticmethod
    def rotated_rectangle_corners_mm(cx, cy, heading_deg, along_heading_mm, cross_heading_mm):
        """
        Build rectangle corners in arena millimetres.

        heading_deg is the direction of the ArUco top edge in arena coordinates.
        along_heading_mm is the robot dimension aligned with that heading.
        cross_heading_mm is the perpendicular dimension.
        """
        a = np.deg2rad(float(heading_deg))
        hx = np.cos(a)
        hy = np.sin(a)
        px = -hy
        py = hx

        ah = float(along_heading_mm) / 2.0
        ch = float(cross_heading_mm) / 2.0

        return [
            (cx + hx * ah + px * ch, cy + hy * ah + py * ch),
            (cx + hx * ah - px * ch, cy + hy * ah - py * ch),
            (cx - hx * ah - px * ch, cy - hy * ah - py * ch),
            (cx - hx * ah + px * ch, cy - hy * ah + py * ch),
        ]

    def draw_robot_footprint_topdown(
        self,
        img,
        robot,
        label,
        color,
        selected=False,
    ):
        """
        Draw robot footprint as a rotated rectangle instead of a circle.

        Main robot:
          physical width  = 256 mm
          physical length = 306 mm
          heading is aligned with the width direction.

        Ninja/SIMA:
          physical width  = 140 mm
          physical length = 151.5 mm
          heading is aligned with the length direction.
        """
        x = float(robot["x_mm"])
        y = float(robot["y_mm"])
        heading = robot.get("heading_deg")
        center_px = self.topdown_px(x, y)

        kind = robot.get("kind", "")
        is_ninja = "ninja" in kind

        if heading is None:
            # Fallback if heading is unavailable.
            radius = 16 if selected else 11
            cv2.circle(img, center_px, radius, color, 2)
            cv2.putText(
                img,
                label,
                (center_px[0] + 12, center_px[1] + 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )
            return

        if is_ninja:
            # Ninja direction is along its length.
            along_heading_mm = float(self.ninja_length_mm)
            cross_heading_mm = float(self.ninja_width_mm)
        else:
            # Main bot direction is along its width.
            along_heading_mm = float(self.main_robot_width_mm)
            cross_heading_mm = float(self.main_robot_length_mm)

        corners_mm = self.rotated_rectangle_corners_mm(
            x,
            y,
            float(heading),
            along_heading_mm,
            cross_heading_mm,
        )
        pts = np.array(
            [self.topdown_px(px, py) for px, py in corners_mm],
            dtype=np.int32,
        ).reshape((-1, 1, 2))

        fill_color = tuple(int(c * 0.35) for c in color)
        cv2.fillConvexPoly(img, pts, fill_color)

        thickness = 4 if selected else 2
        cv2.polylines(img, [pts], isClosed=True, color=color, thickness=thickness)

        # Heading arrow from center, same direction as the ArUco top edge.
        a = np.deg2rad(float(heading))
        arrow_len = int((along_heading_mm * 0.65) * 0.30)
        p2 = (
            int(center_px[0] + np.cos(a) * arrow_len),
            int(center_px[1] - np.sin(a) * arrow_len),
        )
        cv2.arrowedLine(img, center_px, p2, color, 2, tipLength=0.30)

        cv2.circle(img, center_px, 4, color, -1)

        # If the ArUco tag is not at the body center, draw the tag center too.
        # This is useful for calibrating ninja offset direction/sign.
        if "tag_center_x_mm" in robot and "tag_center_y_mm" in robot:
            tag_px = self.topdown_px(
                float(robot["tag_center_x_mm"]),
                float(robot["tag_center_y_mm"]),
            )
            if abs(tag_px[0] - center_px[0]) > 2 or abs(tag_px[1] - center_px[1]) > 2:
                cv2.drawMarker(
                    img,
                    tag_px,
                    color,
                    markerType=cv2.MARKER_CROSS,
                    markerSize=12,
                    thickness=2,
                )
                cv2.line(img, center_px, tag_px, color, 1)

        cv2.putText(
            img,
            label,
            (center_px[0] + 12, center_px[1] + 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )

    # ============================================================
    # STAGED NAVIGATION COMMANDS
    # ============================================================

    def pose_behind_heading(self, pose, distance_mm):
        """
        Return a pose behind `pose` along the negative heading direction.

        If pose heading points toward the crate, this gives a pre-align pose
        from which the robot can rotate once and then drive straight into the
        final approach pose.
        """
        heading = pose.get("heading_deg")
        if heading is None:
            return None

        a = np.deg2rad(float(heading))
        x = float(pose["x_mm"]) - np.cos(a) * float(distance_mm)
        y = float(pose["y_mm"]) - np.sin(a) * float(distance_mm)

        return {
            "x_mm": round(float(x), 1),
            "y_mm": round(float(y), 1),
            "heading_deg": round(float(heading), 1),
        }

    def validate_nav_pose_for_actor(self, actor_name, pose):
        if pose is None:
            return False, "pose_missing"

        x = float(pose["x_mm"])
        y = float(pose["y_mm"])

        if actor_name == "main_bot":
            ok, reason = self.validate_main_bot_approach_point(x, y)
            return bool(ok), reason

        if actor_name == "ninja":
            ok, reason = self.validate_ninja_approach_point(x, y)
            return bool(ok), reason

        return self.is_world_position_plausible(x, y, 0.0), "generic_world_check"

    def build_nav_command_for_target(self, actor_name, target):
        """
        Build a staged command for the onboard robot controller.

        This is not low-level motor control. It is a clear high-level command:
        - drive to pre-align pose
        - turn to approach heading
        - drive straight to approach pose
        - hand off to onboard close-range pickup/alignment

        The onboard bot can implement each stage using odometry, lidar/ToF,
        onboard camera, or whatever local sensors are available.
        """
        command = {
            "active": False,
            "actor": actor_name,
            "reason": "target_inactive",
            "command_type": "staged_pickup_v1",
            "source_target": target,
            "robot_pose": None,
            "target_crate": None,
            "pre_align_pose": None,
            "approach_pose": None,
            "pickup_handoff": None,
            "stages": [],
            "notes": [
                "Overhead provides global poses; onboard robot should do final close-range alignment.",
                "Final approach is straight along approach_pose.heading_deg.",
            ],
        }

        if target is None or not target.get("active", False):
            if isinstance(target, dict):
                command["reason"] = target.get("reason", "target_inactive")
            return command

        robot = target.get("robot")
        approach = target.get("approach_pose")
        crate = target.get("target_crate")

        if robot is None:
            command["reason"] = "robot_pose_missing"
            return command

        if approach is None:
            command["reason"] = "approach_pose_missing"
            return command

        if actor_name == "main_bot":
            pre_align_distance = float(self.main_bot_pre_align_distance_mm)
        else:
            pre_align_distance = float(self.ninja_pre_align_distance_mm)

        pre_align = self.pose_behind_heading(approach, pre_align_distance)
        pre_ok, pre_reason = self.validate_nav_pose_for_actor(actor_name, pre_align)

        # If pre-align pose is invalid, fall back to the approach pose itself.
        # The command stays active, but the onboard should know there is no
        # separate straight-line pre-approach segment available.
        pre_align_was_fallback = False
        if not pre_ok:
            pre_align_was_fallback = True
            pre_align = {
                "x_mm": round(float(approach["x_mm"]), 1),
                "y_mm": round(float(approach["y_mm"]), 1),
                "heading_deg": round(float(approach["heading_deg"]), 1),
            }

        distance_robot_to_pre = self.distance_between_points_mm(robot, pre_align)
        distance_pre_to_approach = self.distance_between_points_mm(pre_align, approach)
        heading_error_now = self.signed_angle_error_deg(
            approach.get("heading_deg"),
            robot.get("heading_deg"),
        )

        command.update({
            "active": True,
            "reason": "ok",
            "robot_pose": robot,
            "target_crate": crate,
            "pre_align_pose": pre_align,
            "pre_align_valid": bool(pre_ok),
            "pre_align_reason": pre_reason,
            "pre_align_was_fallback": bool(pre_align_was_fallback),
            "approach_pose": approach,
            "distance_robot_to_pre_align_mm": round(float(distance_robot_to_pre), 1),
            "distance_pre_align_to_approach_mm": round(float(distance_pre_to_approach), 1),
            "heading_error_now_to_final_approach_deg": None if heading_error_now is None else round(float(heading_error_now), 1),
            "pickup_handoff": {
                "mode": "onboard_close_range_alignment",
                "handoff_distance_mm": round(float(self.pickup_handoff_distance_mm), 1),
                "expected_crate_center": None if crate is None else {
                    "x_mm": crate.get("x_mm"),
                    "y_mm": crate.get("y_mm"),
                    "long_axis_deg": crate.get("long_axis_deg"),
                },
            },
        })

        command["stages"] = [
            {
                "stage": 1,
                "name": "drive_to_pre_align_pose",
                "goal_pose": pre_align,
                "description": "Coarse global move. Robot may turn/drive using its own local controller.",
            },
            {
                "stage": 2,
                "name": "rotate_to_final_approach_heading",
                "goal_heading_deg": approach.get("heading_deg"),
                "description": "Rotate in place so the pickup mechanism faces along the crate long axis.",
            },
            {
                "stage": 3,
                "name": "drive_straight_to_approach_pose",
                "goal_pose": approach,
                "straight_distance_mm": round(float(distance_pre_to_approach), 1),
                "description": "Drive straight along final approach heading.",
            },
            {
                "stage": 4,
                "name": "handoff_to_onboard_pickup_alignment",
                "pickup_handoff": command["pickup_handoff"],
                "description": "Use onboard camera/sensors for final cm-level alignment and pickup.",
            },
        ]

        return command

    def build_nav_commands(self, targets):
        return {
            "main_bot": self.build_nav_command_for_target(
                "main_bot",
                targets.get("main_bot"),
            ),
            "ninja": self.build_nav_command_for_target(
                "ninja",
                targets.get("ninja"),
            ),
        }

    # ============================================================
    # COMPACT ONBOARD COMMANDS
    # ============================================================

    def rounded_value(self, value, step):
        if value is None:
            return None

        step = float(step)
        if step <= 0.0:
            return round(float(value), 3)

        return round(round(float(value) / step) * step, 3)

    def compact_pose(self, pose):
        if pose is None:
            return None

        return {
            "x_mm": self.rounded_value(
                pose.get("x_mm"),
                self.compact_command_goal_rounding_mm,
            ),
            "y_mm": self.rounded_value(
                pose.get("y_mm"),
                self.compact_command_goal_rounding_mm,
            ),
            "heading_deg": self.rounded_value(
                pose.get("heading_deg"),
                self.compact_command_heading_rounding_deg,
            ),
        }

    def compact_crate(self, crate):
        if crate is None:
            return None

        return {
            "track_id": crate.get("track_id"),
            "aruco_id": crate.get("aruco_id"),
            "crate_type": crate.get("crate_type"),
            "x_mm": self.rounded_value(
                crate.get("x_mm"),
                self.compact_command_goal_rounding_mm,
            ),
            "y_mm": self.rounded_value(
                crate.get("y_mm"),
                self.compact_command_goal_rounding_mm,
            ),
            "long_axis_deg": self.rounded_value(
                crate.get("long_axis_deg"),
                self.compact_command_heading_rounding_deg,
            ),
            "display_label": crate.get("display_label"),
            "front_back_defined": crate.get("front_back_defined", False),
            "tag_forward_heading_deg": self.rounded_value(
                crate.get("tag_forward_heading_deg"),
                self.compact_command_heading_rounding_deg,
            ),
            "end_a": self.compact_pose(crate.get("end_a")),
            "end_b": self.compact_pose(crate.get("end_b")),
        }

    def compact_robot_pose(self, robot):
        if robot is None:
            return None

        return {
            "aruco_id": robot.get("aruco_id"),
            "x_mm": self.rounded_value(
                robot.get("x_mm"),
                self.compact_command_goal_rounding_mm,
            ),
            "y_mm": self.rounded_value(
                robot.get("y_mm"),
                self.compact_command_goal_rounding_mm,
            ),
            "heading_deg": self.rounded_value(
                robot.get("heading_deg"),
                self.compact_command_heading_rounding_deg,
            ),
            "visible_this_frame": robot.get("visible_this_frame"),
            "confidence": robot.get("confidence"),
        }

    def compact_command_key(self, actor_name, nav_command):
        if nav_command is None or not nav_command.get("active", False):
            reason = "missing"
            if isinstance(nav_command, dict):
                reason = nav_command.get("reason", reason)
            return f"{actor_name}:inactive:{reason}"

        crate = nav_command.get("target_crate") or {}
        pre = nav_command.get("pre_align_pose") or {}
        approach = nav_command.get("approach_pose") or {}

        return "|".join([
            str(actor_name),
            str(crate.get("track_id")),
            str(crate.get("aruco_id")),
            str(crate.get("crate_type")),
            str(self.compact_pose(pre)),
            str(self.compact_pose(approach)),
        ])

    def build_compact_command_for_nav(self, actor_name, nav_command):
        now = time.time()

        self.compact_command_publish_seq[actor_name] += 1

        key = self.compact_command_key(actor_name, nav_command or {})
        if key != self.compact_command_last_key.get(actor_name):
            self.compact_command_seq[actor_name] += 1
            self.compact_command_last_key[actor_name] = key

        command_seq = int(self.compact_command_seq[actor_name])
        publish_seq = int(self.compact_command_publish_seq[actor_name])

        if nav_command is None or not nav_command.get("active", False):
            reason = "missing_nav_command"
            if isinstance(nav_command, dict):
                reason = nav_command.get("reason", reason)

            return {
                "active": False,
                "actor": actor_name,
                "command_type": "compact_staged_pickup_v1",
                "command_seq": command_seq,
                "publish_seq": publish_seq,
                "stamp": now,
                "ttl_ms": int(self.compact_command_ttl_ms),
                "reason": reason,
                "requested_stage": "idle",
                "goal_pose": None,
                "pre_align_pose": None,
                "approach_pose": None,
                "target_crate": None,
                "robot_pose": None,
                "safety": {
                    "stop_if_command_older_than_ms": int(self.compact_command_ttl_ms),
                    "ignore_if_active_false": True,
                    "ignore_if_seq_older_than_last_seen": True,
                },
            }

        pre = self.compact_pose(nav_command.get("pre_align_pose"))
        approach = self.compact_pose(nav_command.get("approach_pose"))
        crate = self.compact_crate(nav_command.get("target_crate"))
        robot = self.compact_robot_pose(nav_command.get("robot_pose"))

        # First onboard objective: go to pre-align pose.
        # The onboard state machine should then rotate and drive straight to approach.
        requested_stage = "goto_pre_align"
        goal_pose = pre

        return {
            "active": bool(self.compact_command_enabled),
            "actor": actor_name,
            "command_type": "compact_staged_pickup_v1",
            "command_seq": command_seq,
            "publish_seq": publish_seq,
            "stamp": now,
            "ttl_ms": int(self.compact_command_ttl_ms),
            "reason": "ok" if self.compact_command_enabled else "compact_command_disabled",
            "requested_stage": requested_stage,
            "goal_pose": goal_pose,
            "pre_align_pose": pre,
            "approach_pose": approach,
            "target_crate": crate,
            "robot_pose": robot,
            "handoff": {
                "mode": "onboard_close_range_alignment",
                "handoff_distance_mm": self.rounded_value(
                    self.pickup_handoff_distance_mm,
                    self.compact_command_goal_rounding_mm,
                ),
            },
            "stage_plan": [
                "goto_pre_align",
                "rotate_to_approach_heading",
                "drive_straight_to_approach",
                "handoff_to_onboard_pickup",
            ],
            "safety": {
                "stop_if_command_older_than_ms": int(self.compact_command_ttl_ms),
                "ignore_if_active_false": True,
                "ignore_if_seq_older_than_last_seen": True,
                "do_not_directly_use_large_debug_json_for_motion": True,
            },
        }

    def build_compact_commands(self, nav_commands):
        return {
            "main_bot": self.build_compact_command_for_nav(
                "main_bot",
                nav_commands.get("main_bot"),
            ),
            "ninja": self.build_compact_command_for_nav(
                "ninja",
                nav_commands.get("ninja"),
            ),
        }

    def crate_type_prefix(self, crate):
        """Return a very short crate type prefix for clean top-down labels."""
        crate_type = str(crate.get("crate_type", "unknown"))
        aruco_id = int(crate.get("aruco_id", -1))

        if crate_type == "blue" or aruco_id == 36:
            return "B"
        if crate_type == "yellow" or aruco_id == 47:
            return "Y"
        if crate_type == "empty_black" or aruco_id == 41:
            return "E"
        return "U"

    def short_crate_label(self, crate):
        """Readable compact label: B1, Y4, E2, U7."""
        return f"{self.crate_type_prefix(crate)}{int(crate.get('track_id', -1))}"

    def crate_topdown_colors_rgb(self, crate):
        """
        Return (border, fill, text) RGB colors for a crate in the top-down image.
        The top-down image is published as RGB8, so tuples are RGB, not BGR.
        """
        prefix = self.crate_type_prefix(crate)

        if prefix == "B":
            return self.COLOR_TEAM_BLUE, self.COLOR_TEAM_BLUE, (255, 255, 255)
        if prefix == "Y":
            return self.COLOR_TEAM_YELLOW, self.COLOR_TEAM_YELLOW, (35, 35, 35)
        if prefix == "E":
            return (230, 230, 230), self.COLOR_EMPTY_BLACK, (255, 255, 255)
        return (210, 210, 210), (130, 130, 130), (255, 255, 255)

    def draw_centered_text(self, img, text, center_px, color, font_scale=0.48, thickness=2):
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), baseline = cv2.getTextSize(str(text), font, font_scale, thickness)
        x = int(center_px[0] - tw / 2)
        y = int(center_px[1] + th / 2)
        cv2.putText(img, str(text), (x, y), font, font_scale, color, thickness)

    def draw_rotated_crate_topdown(self, img, crate):
        """
        Draw one crate as a 150 mm x 50 mm rotated rectangle.

        Orientation convention:
        - long_axis_deg is the ArUco top-edge direction in arena coordinates.
        - A-end is the negative long-axis end.
        - B-end is the positive long-axis end.
        - We do not assume a physical front/back unless the tags are mounted with
          a fixed physical convention later.
        """
        try:
            x = float(crate["x_mm"])
            y = float(crate["y_mm"])
            angle_deg = crate.get("long_axis_deg")
            if angle_deg is None:
                angle_deg = 0.0
            angle_deg = float(angle_deg)

            a = np.deg2rad(angle_deg)
            axis = np.array([np.cos(a), np.sin(a)], dtype=float)
            perp = np.array([-np.sin(a), np.cos(a)], dtype=float)

            half_l = max(10.0, float(self.crate_body_length_mm) / 2.0)
            half_w = max(5.0, float(self.crate_body_width_mm) / 2.0)
            center = np.array([x, y], dtype=float)

            corners_mm = [
                center - axis * half_l - perp * half_w,
                center + axis * half_l - perp * half_w,
                center + axis * half_l + perp * half_w,
                center - axis * half_l + perp * half_w,
            ]
            corners_px = np.array(
                [self.topdown_px(float(p[0]), float(p[1])) for p in corners_mm],
                dtype=np.int32,
            )

            border_color, fill_color, text_color = self.crate_topdown_colors_rgb(crate)

            overlay = img.copy()
            cv2.fillConvexPoly(overlay, corners_px, fill_color)
            cv2.addWeighted(overlay, 0.58, img, 0.42, 0, dst=img)
            cv2.polylines(img, [corners_px], True, border_color, 2)

            center_px = self.topdown_px(x, y)
            self.draw_centered_text(
                img,
                self.short_crate_label(crate),
                center_px,
                text_color,
                font_scale=0.46,
                thickness=2,
            )

            # Long-axis / tag direction arrow. This is orientation, not a physical
            # "front" unless the tag mounting convention defines one later.
            if self.topdown_show_crate_orientation_arrow:
                start = center - axis * (half_l * 0.20)
                end = center + axis * (half_l * 0.72)
                p_start = self.topdown_px(float(start[0]), float(start[1]))
                p_end = self.topdown_px(float(end[0]), float(end[1]))
                cv2.arrowedLine(img, p_start, p_end, text_color, 2, tipLength=0.30)

            # A/B end labels are less ambiguous than front/back.
            if self.topdown_show_crate_ab_ends:
                end_a = center - axis * half_l
                end_b = center + axis * half_l
                p_a = self.topdown_px(float(end_a[0]), float(end_a[1]))
                p_b = self.topdown_px(float(end_b[0]), float(end_b[1]))
                cv2.circle(img, p_a, 3, text_color, -1)
                cv2.circle(img, p_b, 3, text_color, -1)
                cv2.putText(
                    img,
                    "A",
                    (p_a[0] - 12, p_a[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.34,
                    text_color,
                    1,
                )
                cv2.putText(
                    img,
                    "B",
                    (p_b[0] + 5, p_b[1] + 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.34,
                    text_color,
                    1,
                )

            if self.topdown_show_crate_debug_labels or self.topdown_crate_display_mode in {"debug", "full"}:
                debug_label = (
                    f"{self.short_crate_label(crate)} ID{int(crate.get('aruco_id', -1))} "
                    f"C{int(crate.get('confidence', 0))} "
                    f"a={round(angle_deg, 1)}"
                )
                cv2.putText(
                    img,
                    debug_label,
                    (center_px[0] + 10, center_px[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    border_color,
                    1,
                )
        except Exception:
            # Visualization must never stop the camera/planner loop.
            pass

    def draw_topdown_map(self, stable_crates, robots=None, targets=None):
        """
        Draw a clean arena map from the current world model.

        If topdown_background_path is set, use the official arena image as the
        background and draw the detected objects over it. Otherwise use the
        synthetic dark debug map.
        """
        scale = 0.30
        margin = 40

        width_px = int(self.arena_width_mm * scale) + 2 * margin
        height_px = int(self.arena_height_mm * scale) + 2 * margin

        # Base image: either official arena background or synthetic background.
        #
        # Important:
        # - The top-down canvas has margins around the arena.
        # - The official arena-map PNG should fit only inside the arena rectangle,
        #   not be stretched across the full canvas including margins.
        arena_w_px = int(self.arena_width_mm * scale)
        arena_h_px = int(self.arena_height_mm * scale)

        img = np.zeros((height_px, width_px, 3), dtype=np.uint8)
        img[:] = (35, 35, 35)

        if self.topdown_background is not None:
            bg_arena = cv2.resize(
                self.topdown_background,
                (arena_w_px, arena_h_px),
                interpolation=cv2.INTER_AREA,
            )
            ox = self.topdown_bg_offset_x_px
            oy = self.topdown_bg_offset_y_px
            y0 = margin + oy
            x0 = margin + ox
            y1 = y0 + arena_h_px
            x1 = x0 + arena_w_px
            y0c = max(y0, 0)
            x0c = max(x0, 0)
            y1c = min(y1, height_px)
            x1c = min(x1, width_px)
            bg_y0 = y0c - y0
            bg_x0 = x0c - x0
            bg_y1 = bg_y0 + (y1c - y0c)
            bg_x1 = bg_x0 + (x1c - x0c)
            img[y0c:y1c, x0c:x1c] = bg_arena[bg_y0:bg_y1, bg_x0:bg_x1]
            overlay = img.copy()
            draw_img = overlay
        else:
            draw_img = img

        # Arena boundary
        self.draw_rect_topdown(
            draw_img,
            0,
            0,
            self.arena_width_mm,
            self.arena_height_mm,
            (220, 220, 220),
            "arena 3000x2000",
            2,
        )

        # Grid every 500 mm
        for x in range(0, int(self.arena_width_mm) + 1, 500):
            p1 = self.topdown_px(x, 0)
            p2 = self.topdown_px(x, self.arena_height_mm)
            cv2.line(draw_img, p1, p2, (70, 70, 70), 1)
            cv2.putText(
                draw_img,
                str(x),
                (p1[0] - 20, height_px - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (120, 120, 120),
                1,
            )

        for y in range(0, int(self.arena_height_mm) + 1, 500):
            p1 = self.topdown_px(0, y)
            p2 = self.topdown_px(self.arena_width_mm, y)
            cv2.line(draw_img, p1, p2, (70, 70, 70), 1)
            cv2.putText(
                draw_img,
                str(y),
                (5, p1[1] + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (120, 120, 120),
                1,
            )

        # Clean official-like debug palette.
        yellow_border = self.COLOR_TEAM_YELLOW
        yellow_fill = self.COLOR_TEAM_YELLOW
        blue_border = self.COLOR_TEAM_BLUE
        blue_fill = self.COLOR_TEAM_BLUE
        brown_border = self.COLOR_GRANARY
        brown_fill = self.COLOR_GRANARY
        pantry_border = self.COLOR_PANTRY

        # Fixed zones: rear strip = yellow nest | granary | blue nest
        self.draw_rect_topdown_filled(draw_img, 0, 1550, 600, 2000, yellow_border, yellow_fill, "yellow nest", 3)
        self.draw_rect_topdown_filled(draw_img, 600, 1550, 2400, 2000, brown_border, brown_fill, "granary +55mm", 3)
        self.draw_rect_topdown_filled(draw_img, 2400, 1550, 3000, 2000, blue_border, blue_fill, "blue nest", 3)

        # Pantries, approximate centers from the plan. Each is 200x200 mm.
        pantry_centers = [
            (1250, 1450),
            (1750, 1450),
            (100, 800),
            (800, 800),
            (1500, 800),
            (2200, 800),
            (2900, 800),
            (700, 100),
            (1500, 100),
            (2300, 100),
        ]

        for i, (cx, cy) in enumerate(pantry_centers):
            self.draw_rect_topdown_filled(
                draw_img,
                cx - 100,
                cy - 100,
                cx + 100,
                cy + 100,
                pantry_border,
                None,
                f"P{i + 1}",
                2,
            )

        # ---- Game zones: fridges, collection areas, ninja nests ----
        if self.topdown_show_game_zones:

            fridges = [
                ("F1",  1900, 1725, 50, 75, self.COLOR_TEAM_BLUE),
                ("F2",  1650, 1775, 50, 75, self.COLOR_TEAM_BLUE),
                ("F1",  1100, 1725, 50, 75, self.COLOR_TEAM_YELLOW),
                ("F2",  1350, 1775, 50, 75, self.COLOR_TEAM_YELLOW),
            ]
            for label, cx, cy, hw_x, hw_y, color in fridges:
                self.draw_rect_topdown_filled(
                    draw_img, cx - hw_x, cy - hw_y, cx + hw_x, cy + hw_y,
                    color, color, label, 2,
                )

            ninja_nests = [
                ("NB", 2300, 1900, self.COLOR_TEAM_BLUE),
                ("NY",  700, 1900, self.COLOR_TEAM_YELLOW),
            ]
            for label, cx, cy, color in ninja_nests:
                self.draw_rect_topdown_filled(
                    draw_img, cx - 100, cy - 100, cx + 100, cy + 100,
                    color, color, label, 1,
                )

            collection_areas = [
                ("C1",   800, 1675, 100, 75),
                ("C2",  2200, 1675, 100, 75),
                ("C3",   175, 1200, 75, 100),
                ("C6",  2825, 1200, 75, 100),
                ("C7",   175,  400, 75, 100),
                ("C10", 2825,  400, 75, 100),
                ("C4",  1150,  800, 100, 75),
                ("C5",  1850,  800, 100, 75),
                ("C8",  1100,  175, 100, 75),
                ("C9",  1900,  175, 100, 75),
            ]
            for label, cx, cy, hx, hy in collection_areas:
                self.draw_rect_topdown(
                    draw_img, cx - hx, cy - hy, cx + hx, cy + hy,
                    self.COLOR_COLLECTION, label, 1,
                )

        # Arena reference markers
        for marker_id, (x, y) in self.arena_markers_mm.items():
            px = self.topdown_px(x, y)
            cv2.circle(draw_img, px, 7, self.COLOR_REF, -1)
            cv2.putText(
                draw_img,
                f"ID{marker_id}",
                (px[0] + 8, px[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                self.COLOR_REF,
                1,
            )

        # Stable crate tracks.
        # Clean mode draws each crate as a true 150 x 50 mm rotated rectangle with
        # a short type+track label (B1/Y2/E3) and A/B end markers. Debug/full mode
        # can still show every approach candidate when needed.
        show_all_approach_candidates = (
            self.topdown_show_all_approach_candidates
            or self.topdown_crate_display_mode in {"debug", "full"}
        )

        for crate in stable_crates:
            self.draw_rotated_crate_topdown(draw_img, crate)

            if not show_all_approach_candidates:
                continue

            # Optional detailed approach-pose candidate drawing. This is disabled
            # in clean mode because it makes the planning map too busy.
            for candidate in crate.get("approach_pose_candidates", []):
                cpx = self.topdown_px(candidate["x_mm"], candidate["y_mm"])

                if candidate.get("valid_for_main_bot"):
                    cand_color = self.COLOR_MAIN_ROBOT
                    label_prefix = "M"
                elif candidate.get("valid_for_ninja"):
                    cand_color = self.COLOR_NINJA
                    label_prefix = "N"
                else:
                    cand_color = (120, 120, 120)
                    label_prefix = "X"

                cv2.circle(draw_img, cpx, 5, cand_color, 1)

                if candidate.get("heading_deg") is not None:
                    ha = np.deg2rad(float(candidate["heading_deg"]))
                    arrow_len = int(90 * scale)
                    hp2 = (
                        int(cpx[0] + np.cos(ha) * arrow_len),
                        int(cpx[1] - np.sin(ha) * arrow_len),
                    )
                    cv2.arrowedLine(draw_img, cpx, hp2, cand_color, 1, tipLength=0.30)

                cv2.putText(
                    draw_img,
                    f"{label_prefix}:{candidate['name']}",
                    (cpx[0] + 6, cpx[1] + 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.40,
                    cand_color,
                    1,
                )

        # Robot / ninja markers as real footprint rectangles.
        if robots is not None:
            selected_main_id = None
            selected_ninja_id = None
            if robots.get("main_robot") is not None:
                selected_main_id = int(robots["main_robot"]["aruco_id"])
            if robots.get("ninja") is not None:
                selected_ninja_id = int(robots["ninja"]["aruco_id"])

            for robot in robots.get("all", []):
                try:
                    kind = robot.get("kind", "")
                    marker_id = int(robot.get("aruco_id", -999))

                    if kind == "own_official_robot":
                        prefix = "R"
                    elif kind == "own_ninja":
                        prefix = "N"
                    elif kind == "opponent_official_robot":
                        prefix = "ENEMY"
                    else:
                        prefix = "U"

                    selected = False
                    if selected_main_id is not None and marker_id == selected_main_id:
                        prefix = "MAIN"
                        selected = True
                    elif selected_ninja_id is not None and marker_id == selected_ninja_id:
                        prefix = "NINJA"
                        selected = True

                    color = self.robot_display_color_rgb(robot)
                    label = f"{prefix} ID{robot['aruco_id']}"

                    self.draw_robot_footprint_topdown(
                        draw_img,
                        robot,
                        label=label,
                        color=color,
                        selected=selected,
                    )
                except Exception:
                    pass

        # Suppressed robot-like detections.
        # These are low-confidence opponent tracks. They are not used for planning.
        if robots is not None and self.draw_rejected_robot_detections and self.detect_opponent_robots:
            for robot in robots.get("suppressed_robot_detections", []):
                try:
                    if robot.get("team_relation") != "opponent":
                        continue

                    x = float(robot.get("x_mm"))
                    y = float(robot.get("y_mm"))
                    if not self.is_world_position_plausible(x, y, 100.0):
                        continue

                    p = self.topdown_px(x, y)
                    color = self.robot_display_color_rgb(robot)
                    cv2.drawMarker(
                        draw_img,
                        p,
                        color,
                        markerType=cv2.MARKER_CROSS,
                        markerSize=14,
                        thickness=1,
                    )
                    cv2.putText(
                        draw_img,
                        f"SUP ID{robot.get('aruco_id')}",
                        (p[0] + 8, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.38,
                        color,
                        1,
                    )
                except Exception:
                    pass

        # Rejected robot-like detections.
        # These are not used for planning. They are only drawn to explain cases
        # where an opponent/yellow marker ID was detected but rejected by height/world validation.
        if robots is not None and self.draw_rejected_robot_detections:
            for robot in robots.get("rejected_robot_detections", []):
                try:
                    # Use floor projection for debug drawing, because height correction
                    # may have pushed the assumed body center outside the arena.
                    x = float(robot.get("x_floor_projection_mm"))
                    y = float(robot.get("y_floor_projection_mm"))
                    if not self.is_world_position_plausible(x, y, 100.0):
                        continue

                    p = self.topdown_px(x, y)
                    color = self.robot_display_color_rgb(robot)
                    cv2.drawMarker(
                        draw_img,
                        p,
                        color,
                        markerType=cv2.MARKER_TILTED_CROSS,
                        markerSize=18,
                        thickness=2,
                    )
                    cv2.putText(
                        draw_img,
                        f"REJ ID{robot.get('aruco_id')}",
                        (p[0] + 8, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.42,
                        color,
                        1,
                    )
                except Exception:
                    pass

        # Ninja/SIMA actuator projection reach band.
        # This is debug-only: the ninja body must still stay in the granary,
        # but the actuator can later be allowed to project over the granary line.
        try:
            reach = float(self.ninja_actuator_projection_reach_mm)
            if reach > 0.0:
                gx1, gy1, gx2, gy2 = self.granary_rect()
                line_y = min(gy1, gy2)
                p1 = self.topdown_px(min(gx1, gx2), line_y)
                p2 = self.topdown_px(max(gx1, gx2), line_y - reach)
                overlay = draw_img.copy()
                cv2.rectangle(
                    overlay,
                    (min(p1[0], p2[0]), min(p1[1], p2[1])),
                    (max(p1[0], p2[0]), max(p1[1], p2[1])),
                    (190, 0, 190),
                    -1,
                )
                cv2.addWeighted(overlay, 0.12, draw_img, 0.88, 0, dst=draw_img)
                cv2.putText(
                    draw_img,
                    f"NINJA REACH {reach:.0f}mm",
                    (min(p1[0], p2[0]) + 8, min(p1[1], p2[1]) + 18),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (220, 80, 220),
                    1,
                )
        except Exception:
            pass

        # Target queue ranks.
        # Disabled by default in clean mode. Selected target path is still drawn.
        show_target_queue_labels = (
            self.topdown_show_target_queue_labels
            or self.topdown_crate_display_mode in {"debug", "full"}
        )
        if targets is not None and show_target_queue_labels:
            for actor_name, queue_key, color, prefix in [
                ("main_bot", "main_bot_queue", self.COLOR_MAIN_ROBOT, "M"),
                ("ninja", "ninja_queue", self.COLOR_NINJA, "N"),
            ]:
                for item in targets.get(queue_key, [])[:max(0, int(self.topdown_target_queue_max_labels))]:
                    approach = item.get("approach_pose")
                    if approach is None:
                        continue
                    try:
                        p = self.topdown_px(float(approach["x_mm"]), float(approach["y_mm"]))
                        cv2.circle(draw_img, p, 9, color, 1)
                        cv2.putText(
                            draw_img,
                            f"{prefix}{item.get('rank')}",
                            (p[0] + 8, p[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.45,
                            color,
                            1,
                        )
                    except Exception:
                        pass

        # Navigation pre-align poses
        if targets is not None:
            # nav_commands is not passed here directly, so compute lightweight
            # visual pre-align poses from selected targets.
            for actor_name, target, color, prefix in [
                ("main_bot", targets.get("main_bot", {}), self.COLOR_MAIN_ROBOT, "PRE-M"),
                ("ninja", targets.get("ninja", {}), self.COLOR_NINJA, "PRE-N"),
            ]:
                if not isinstance(target, dict) or not target.get("active", False):
                    continue
                approach = target.get("approach_pose")
                if approach is None:
                    continue
                try:
                    d = self.main_bot_pre_align_distance_mm if actor_name == "main_bot" else self.ninja_pre_align_distance_mm
                    pre = self.pose_behind_heading(approach, d)
                    ok, _ = self.validate_nav_pose_for_actor(actor_name, pre)
                    if not ok:
                        continue
                    p = self.topdown_px(float(pre["x_mm"]), float(pre["y_mm"]))
                    cv2.drawMarker(
                        draw_img,
                        p,
                        color,
                        markerType=cv2.MARKER_DIAMOND,
                        markerSize=16,
                        thickness=2,
                    )
                    cv2.putText(
                        draw_img,
                        prefix,
                        (p[0] + 8, p[1] + 14),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.42,
                        color,
                        1,
                    )
                except Exception:
                    pass

        # Selected target lines.
        # Only draw the selected active targets.
        # Do not iterate over every item in `targets`, because it also contains
        # queue lists such as main_bot_queue and ninja_queue.
        if targets is not None:
            for actor_name in ["main_bot", "ninja"]:
                target = targets.get(actor_name, {})
                if not isinstance(target, dict):
                    continue
                if not target.get("active", False):
                    continue

                robot = target.get("robot")
                approach = target.get("approach_pose")
                crate = target.get("target_crate")

                if robot is None or approach is None:
                    continue

                try:
                    robot_px = self.topdown_px(float(robot["x_mm"]), float(robot["y_mm"]))
                    approach_px = self.topdown_px(float(approach["x_mm"]), float(approach["y_mm"]))

                    if actor_name == "main_bot":
                        color = self.COLOR_MAIN_ROBOT
                        label = "MAIN TARGET"
                    else:
                        color = self.COLOR_NINJA
                        label = "NINJA TARGET"

                    cv2.line(draw_img, robot_px, approach_px, color, 2)
                    cv2.circle(draw_img, approach_px, 13, color, 2)

                    if crate is not None:
                        crate_px = self.topdown_px(float(crate["x_mm"]), float(crate["y_mm"]))
                        cv2.circle(draw_img, crate_px, 15, color, 2)

                    cv2.putText(
                        draw_img,
                        label,
                        (approach_px[0] + 12, approach_px[1] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        color,
                        2,
                    )
                except Exception:
                    pass

        # Blend overlay back onto official map if we are in background mode
        if self.topdown_background is not None:
            alpha = min(max(self.topdown_overlay_alpha, 0.0), 1.0)
            img = cv2.addWeighted(draw_img, alpha, img, 1.0 - alpha, 0.0)
        else:
            img = draw_img

        # Text status
        bg_mode = "official-map" if self.topdown_background is not None else "synthetic"
        cv2.putText(
            img,
            f"Overhead top-down ({bg_mode}) | team={self.team_side} | crates={len(stable_crates)} | crate_view={self.topdown_crate_display_mode}",
            (margin, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            self.COLOR_TEXT,
            2,
        )

        return img

    # ============================================================
    # TIMER CALLBACK
    # ============================================================

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Failed to read frame")
            return

        self.frame_count += 1

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        try:
            corners, ids, rejected = self.detect_aruco(gray)
        except Exception as e:
            self.get_logger().error(f"ArUco detection error: {e}")
            return

        detected_ids = [] if ids is None else ids.flatten().astype(int).tolist()
        rejected_count = 0 if rejected is None else len(rejected)

        homography_updated_this_frame = False

        if ids is not None:
            detected_set = set(detected_ids)

            if self.arena_ref_ids.issubset(detected_set):
                H_new, used_ids = self.compute_homography(corners, ids)

                if H_new is not None:
                    self.H_px_to_mm = H_new
                    self.last_homography_time = time.time()
                    self.last_used_ref_ids = sorted(used_ids)
                    homography_updated_this_frame = True

        markers = []

        if ids is not None:
            for marker_id, marker_corners in zip(ids.flatten(), corners):
                marker_id = int(marker_id)
                center = self.marker_center_px(marker_corners)
                size_px = self.marker_size_px(marker_corners)

                marker_data = {
                    "id": marker_id,
                    "center_px": [
                        round(float(center[0]), 1),
                        round(float(center[1]), 1),
                    ],
                    "size_px": round(size_px, 1),
                }

                if self.H_px_to_mm is not None:
                    x_mm, y_mm = self.px_to_mm(center, self.H_px_to_mm)
                    marker_data["x_floor_projection_mm"] = round(x_mm, 1)
                    marker_data["y_floor_projection_mm"] = round(y_mm, 1)

                markers.append(marker_data)

        robots = self.extract_robot_detections(corners, ids)

        raw_crate_detections = self.extract_raw_crate_detections(corners, ids)
        self.crate_tracker.update(raw_crate_detections, self.frame_count)
        stable_crates = self.crate_tracker.to_json_list()
        stable_crates = self.enrich_crates_with_approach_poses(stable_crates)

        targets = self.build_targets(robots, stable_crates)
        planner_layer = self.build_cluster_planner_layer(robots, stable_crates)
        nav_commands = self.build_nav_commands(targets)
        compact_commands = self.build_compact_commands(nav_commands)

        status_msg = String()
        status_msg.data = (
            f"frame={self.frame_count} | "
            f"detected_ids={detected_ids} | "
            f"homography_active={self.H_px_to_mm is not None} "
            f"mode={self.last_aruco_detection_mode} | "
            f"raw_crates={len(raw_crate_detections)} | "
            f"stable_crates={len(stable_crates)} | "
            f"robots={len(robots.get('all', []))} raw={robots.get('raw_visible_count', 0)} "
            f"opp={'on' if self.detect_opponent_robots else 'off'} "
            f"supp={robots.get('suppressed_robot_count', 0)} "
            f"rej={robots.get('rejected_robot_count', 0)} | "
            f"main_target={targets['main_bot'].get('active', False)} "
            f"q={len(targets.get('main_bot_queue', []))} "
            f"cmd={nav_commands['main_bot'].get('active', False)} "
            f"compact={compact_commands['main_bot'].get('active', False)} "
            f"seq={compact_commands['main_bot'].get('command_seq', 0)} | "
            f"ninja_target={targets['ninja'].get('active', False)} "
            f"q={len(targets.get('ninja_queue', []))} "
            f"cmd={nav_commands['ninja'].get('active', False)} "
            f"compact={compact_commands['ninja'].get('active', False)} "
            f"seq={compact_commands['ninja'].get('command_seq', 0)}"
        )
        self.status_pub.publish(status_msg)

        detected_data = {
            "timestamp": time.time(),
            "frame": self.frame_count,
            "detected_ids": detected_ids,
            "marker_count": len(markers),
            "rejected_count": rejected_count,
            "ignored_aruco_ids": list(getattr(self, "last_ignored_aruco_ids", [])),
            "ignored_opponent_candidate_ids": list(getattr(self, "last_ignored_opponent_candidate_ids", [])),
            "markers": markers,
        }

        ids_msg = String()
        ids_msg.data = json.dumps(detected_data)
        self.detected_ids_pub.publish(ids_msg)

        world_state = {
            "timestamp": time.time(),
            "frame": self.frame_count,
            "team_side": self.team_side,

            "homography": {
                "active": self.H_px_to_mm is not None,
                "updated_this_frame": homography_updated_this_frame,
                "last_update_time": self.last_homography_time,
                "used_ref_ids": self.last_used_ref_ids,
                "required_ref_ids": sorted(list(self.arena_ref_ids)),
                "reprojection_mean_mm": self.last_homography_reprojection_mean_mm,
                "reprojection_max_mm": self.last_homography_reprojection_max_mm,
                "last_rejected_reason": self.last_homography_rejected_reason,
            },

            "detected_ids": detected_ids,
            "raw_crate_detections": raw_crate_detections,
            "stable_crates": stable_crates,
            "robots": robots,
            "targets": targets,
            "planner_layer": planner_layer,
            "nav_commands": nav_commands,
            "compact_commands": compact_commands,

            "approach_settings": {
                "crate_approach_distance_mm": self.crate_approach_distance_mm,
                "approach_wall_margin_mm": self.approach_wall_margin_mm,
                "granary_rect_mm": list(self.granary_rect()),
                "opponent_nest_rect_mm": list(self.opponent_nest_rect()),
                "main_bot_pre_align_distance_mm": self.main_bot_pre_align_distance_mm,
                "ninja_pre_align_distance_mm": self.ninja_pre_align_distance_mm,
                "pickup_handoff_distance_mm": self.pickup_handoff_distance_mm,
                "ninja_keep_body_inside_granary": self.ninja_keep_body_inside_granary,
                "ninja_target_requires_granary_or_reach": self.ninja_target_requires_granary_or_reach,
                "ninja_actuator_projection_reach_mm": self.ninja_actuator_projection_reach_mm,
                "ninja_rule_note": (
                    "Ninja/SIMA body is kept inside granary. "
                    "Actuator projection reach can be added later without allowing the body into the main area."
                ),
                "compact_command_enabled": self.compact_command_enabled,
                "compact_command_ttl_ms": self.compact_command_ttl_ms,
                "compact_command_goal_rounding_mm": self.compact_command_goal_rounding_mm,
                "compact_command_heading_rounding_deg": self.compact_command_heading_rounding_deg,
            },

            "target_settings": {
                "target_switch_distance_hysteresis_mm": self.target_switch_distance_hysteresis_mm,
                "topdown_target_queue_max_labels": self.topdown_target_queue_max_labels,
                "topdown_show_target_queue_labels": self.topdown_show_target_queue_labels,
                "topdown_crate_display_mode": self.topdown_crate_display_mode,
                "topdown_show_all_approach_candidates": self.topdown_show_all_approach_candidates,
                "topdown_show_crate_debug_labels": self.topdown_show_crate_debug_labels,
                "topdown_show_crate_orientation_arrow": self.topdown_show_crate_orientation_arrow,
                "topdown_show_crate_ab_ends": self.topdown_show_crate_ab_ends,
                "crate_body_length_mm": self.crate_body_length_mm,
                "crate_body_width_mm": self.crate_body_width_mm,
                "crate_orientation_note": (
                    "Clean top-down view uses A/B crate ends instead of front/back. "
                    "A=end opposite long_axis direction, B=end along long_axis direction. "
                    "long_axis_deg is the ArUco top-edge direction."
                ),
                "queue_mode": "one_best_approach_per_crate",
            },

            "robot_settings": {
                "blue_official_robot_ids": sorted(list(self.blue_official_robot_ids)),
                "yellow_official_robot_ids": sorted(list(self.yellow_official_robot_ids)),
                "blue_ninja_ids": sorted(list(self.blue_ninja_ids)),
                "yellow_ninja_ids": sorted(list(self.yellow_ninja_ids)),
                "own_official_robot_ids_available": sorted(list(self.own_official_robot_ids_available)),
                "own_official_robot_ids_enabled": sorted(list(self.own_official_robot_ids)),
                "opponent_official_robot_ids_enabled": sorted(list(self.opponent_official_robot_ids)),
                "own_ninja_ids_available": sorted(list(self.own_ninja_ids_available)),
                "own_ninja_ids_enabled": sorted(list(self.own_ninja_ids)),
                "configured_id_notes": list(self.configured_id_notes),
                "official_robot_tag_height_mm": self.official_robot_tag_height_mm,
                "ninja_tag_height_mm": self.ninja_tag_height_mm,
                "detect_opponent_robots": self.detect_opponent_robots,
                "opponent_robot_min_confidence_to_show": self.opponent_robot_min_confidence_to_show,
                "filter_to_configured_main_robot_id": self.filter_to_configured_main_robot_id,
                "filter_to_configured_ninja_id": self.filter_to_configured_ninja_id,
                    "opponent_official_robot_tag_height_mm": self.opponent_official_robot_tag_height_mm,
                "draw_rejected_robot_detections": self.draw_rejected_robot_detections,
                "main_robot_aruco_id": self.main_robot_aruco_id,
                "ninja_aruco_id": self.ninja_aruco_id,
                "main_robot_width_mm": self.main_robot_width_mm,
                "main_robot_length_mm": self.main_robot_length_mm,
                "ninja_width_mm": self.ninja_width_mm,
                "ninja_length_mm": self.ninja_length_mm,
                "ninja_tag_from_rear_mm": self.ninja_tag_from_rear_mm,
                "ninja_tag_lateral_offset_mm": self.ninja_tag_lateral_offset_mm,
                "main_robot_top_plate_color_mode": self.main_robot_top_plate_color_mode,
                "world_position_validation_enabled": self.world_position_validation_enabled,
                "crate_world_margin_mm": self.crate_world_margin_mm,
                "robot_world_margin_mm": self.robot_world_margin_mm,
                "robot_track_max_missed_frames": self.robot_track_max_missed_frames,
                "robot_smoothing_alpha": self.robot_smoothing_alpha,
                "robot_smoothing_alpha_angle": self.robot_smoothing_alpha_angle,
            },

            "aruco_settings": {
                "dictionary": "DICT_4X4_100",
                "allowed_ids": sorted(list(self.allowed_aruco_ids)),
                "always_allowed_ids": sorted(list(self.always_allowed_aruco_ids)),
                "opponent_candidate_ids": sorted(list(self.opponent_candidate_aruco_ids)),
                "ignored_opponent_candidate_ids_last_frame": list(getattr(self, "last_ignored_opponent_candidate_ids", [])),
                "enable_arena_roi_mask": self.aruco_enable_arena_roi_mask,
                "arena_roi_margin_mm_effective": self.arena_roi_margin_mm,
                "roi_full_frame_recovery": self.aruco_roi_full_frame_recovery,
                "detection_mode": self.last_aruco_detection_mode,
                "homography_max_reprojection_error_mm": self.homography_max_reprojection_error_mm,
                "enable_clahe": self.aruco_enable_clahe,
                "enable_sharpen": self.aruco_enable_sharpen,
                "second_pass": self.aruco_second_pass,
                "min_marker_perimeter_rate": self.aruco_min_marker_perimeter_rate,
                "adaptive_thresh_win_size_min": self.aruco_adaptive_thresh_win_size_min,
                "adaptive_thresh_win_size_max": self.aruco_adaptive_thresh_win_size_max,
                "adaptive_thresh_win_size_step": self.aruco_adaptive_thresh_win_size_step,
                "corner_refinement": self.aruco_corner_refinement,
            },
        }

        world_msg = String()
        world_msg.data = json.dumps(world_state)
        self.world_state_pub.publish(world_msg)

        main_target_msg = String()
        main_target_msg.data = json.dumps(targets["main_bot"])
        self.main_bot_target_pub.publish(main_target_msg)

        ninja_target_msg = String()
        ninja_target_msg.data = json.dumps(targets["ninja"])
        self.ninja_target_pub.publish(ninja_target_msg)

        main_queue_msg = String()
        main_queue_msg.data = json.dumps(targets.get("main_bot_queue", []))
        self.main_bot_target_queue_pub.publish(main_queue_msg)

        ninja_queue_msg = String()
        ninja_queue_msg.data = json.dumps(targets.get("ninja_queue", []))
        self.ninja_target_queue_pub.publish(ninja_queue_msg)

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

        main_nav_msg = String()
        main_nav_msg.data = json.dumps(nav_commands["main_bot"])
        self.main_bot_nav_command_pub.publish(main_nav_msg)

        ninja_nav_msg = String()
        ninja_nav_msg.data = json.dumps(nav_commands["ninja"])
        self.ninja_nav_command_pub.publish(ninja_nav_msg)

        main_compact_msg = String()
        main_compact_msg.data = json.dumps(compact_commands["main_bot"])
        self.main_bot_compact_command_pub.publish(main_compact_msg)

        ninja_compact_msg = String()
        ninja_compact_msg.data = json.dumps(compact_commands["ninja"])
        self.ninja_compact_command_pub.publish(ninja_compact_msg)

        opponent_msg = String()
        opponent_msg.data = json.dumps({
            "detect_opponent_robots": self.detect_opponent_robots,
            "opponent_robot_min_confidence_to_show": self.opponent_robot_min_confidence_to_show,
            "opponent_candidate_ids": sorted(list(self.opponent_candidate_aruco_ids)),
            "ignored_opponent_candidate_ids": robots.get("ignored_opponent_candidate_ids", []),
            "opponent_official_robots": robots.get("opponent_official_robots", []),
            "suppressed_robot_detections": [
                r for r in robots.get("suppressed_robot_detections", [])
                if r.get("team_relation") == "opponent"
            ],
            "rejected_robot_detections": [
                r for r in robots.get("rejected_robot_detections", [])
                if r.get("team_relation") == "opponent"
            ],
            "note": (
                "During our arena testing keep detect_opponent_robots=false, so false yellow/opponent IDs are ignored. "
                "For real opponent testing set detect_opponent_robots=true; opponent tracks are then published only after the confidence threshold."
            ),
        })
        self.opponent_robots_pub.publish(opponent_msg)

        if self.publish_images:
            overlay = self.draw_debug_overlay(
                frame,
                corners,
                ids,
                raw_crate_detections,
                stable_crates,
                rejected_count,
            )
            view_bgr = cv2.resize(
                overlay,
                (self.display_width, self.display_height),
                interpolation=cv2.INTER_AREA,
            )
            view_rgb = cv2.cvtColor(view_bgr, cv2.COLOR_BGR2RGB)
            img_msg = self.bridge.cv2_to_imgmsg(view_rgb, encoding="rgb8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "overhead_camera"
            self.debug_image_pub.publish(img_msg)

            topdown = self.draw_topdown_map(stable_crates, robots, targets)
            topdown_msg = self.bridge.cv2_to_imgmsg(topdown, encoding="rgb8")
            topdown_msg.header.stamp = self.get_clock().now().to_msg()
            topdown_msg.header.frame_id = "overhead_topdown"
            self.topdown_image_pub.publish(topdown_msg)

        self.get_logger().info(
            f"frame={self.frame_count} "
            f"ids={detected_ids} "
            f"H={self.H_px_to_mm is not None} "
            f"raw={len(raw_crate_detections)} "
            f"stable={len(stable_crates)} "
            f"robots={len(robots.get('all', []))} "
            f"rej_robots={len(robots.get('rejected_robot_detections', []))}"
        )

        if homography_updated_this_frame:
            self.get_logger().info(
                f"Homography updated using IDs {self.last_used_ref_ids} "
                f"err={self.last_homography_reprojection_max_mm}mm"
            )

    # ============================================================
    # CLEANUP
    # ============================================================

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = OverheadCameraNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()