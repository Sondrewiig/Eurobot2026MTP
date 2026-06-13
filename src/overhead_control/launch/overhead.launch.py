#!/usr/bin/env python3
"""
overhead_camera_node.py

Main overhead vision node. Runs on the laptop.

Reads frames from the Brio 4K USB camera, applies a homography to produce
a top-down arena view, detects ArUco markers for crates and robots, tracks
crates across frames, and publishes the full world state as JSON.

Publishes:
  /overhead/world_state_json        std_msgs/String  — full arena state: crates, robots, targets
  /overhead/opponent_robots_json    std_msgs/String  — confirmed opponent robot positions
  /overhead/debug_image             sensor_msgs/Image — annotated top-down view
  /overhead/topdown_image           sensor_msgs/Image — clean top-down view
  /overhead/status                  std_msgs/String  — one-line health summary

Configuration is loaded from overhead_blue.yaml or overhead_yellow.yaml
depending on the team side argument passed at launch.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    pkg_dir = Path(get_package_share_directory("overhead_control"))

    side = LaunchConfiguration("side")
    publish_images = LaunchConfiguration("publish_images")
    device = LaunchConfiguration("device")
    detect_opponent_robots = LaunchConfiguration("detect_opponent_robots")
    opponent_robot_min_confidence_to_show = LaunchConfiguration(
        "opponent_robot_min_confidence_to_show"
    )
    draw_rejected_robot_detections = LaunchConfiguration(
        "draw_rejected_robot_detections"
    )
    official_robot_tag_height_mm = LaunchConfiguration(
        "official_robot_tag_height_mm"
    )
    ninja_tag_height_mm = LaunchConfiguration("ninja_tag_height_mm")
    opponent_official_robot_tag_height_mm = LaunchConfiguration(
        "opponent_official_robot_tag_height_mm"
    )

    params_file = PythonExpression([
        "'",
        str(pkg_dir / "config" / "overhead_"),
        "' + '",
        side,
        "' + '.yaml'"
    ])

    arena_map_path = PathJoinSubstitution([
        FindPackageShare("overhead_control"),
        "config",
        "arena_map_with_coordinate_overlay.png",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "side",
            default_value="blue",
            description="Team side / camera mounting side: blue or yellow",
        ),
        DeclareLaunchArgument(
            "publish_images",
            default_value="true",
            description="Publish /overhead/debug_image and /overhead/topdown_image",
        ),
        DeclareLaunchArgument("device", default_value="/dev/video2"),
        # Enable opponent marker detection by default for documentation/testing.
        # On blue side, official enemy robot marker IDs are 6..10, so ID6 will be
        # published automatically on /overhead/opponent_robots_json when visible.
        DeclareLaunchArgument("detect_opponent_robots", default_value="true"),
        # Use low confidence for screenshot/testing so a newly placed ID6 appears
        # quickly. Increase to 4 for stricter match-like filtering if needed.
        DeclareLaunchArgument(
            "opponent_robot_min_confidence_to_show", default_value="1"
        ),
        DeclareLaunchArgument(
            "draw_rejected_robot_detections", default_value="true"
        ),
        DeclareLaunchArgument("official_robot_tag_height_mm", default_value="350.0"),
        DeclareLaunchArgument("ninja_tag_height_mm", default_value="200.0"),
        # For documentation tests the enemy ID6 marker is often placed flat on
        # the arena or on a low dummy object. Keep opponent height correction at
        # 0 by default so the marker is not rejected because of the normal robot
        # tag-height assumption. Override to 350.0 if ID6 is mounted on a robot.
        DeclareLaunchArgument("opponent_official_robot_tag_height_mm", default_value="300.0"),
        Node(
            package="overhead_control",
            executable="overhead_camera_node",
            name="overhead_camera_node",
            output="screen",
            parameters=[
                params_file,
                {
                    "team_side": side,
                    "device": device,
                    "publish_images": ParameterValue(
                        publish_images, value_type=bool
                    ),
                    "topdown_background_path": arena_map_path,
                    "topdown_overlay_alpha": 0.70,
                    # main_robot_aruco_id and ninja_aruco_id come from the
                    # side YAML (overhead_<side>.yaml) so the marker IDs live
                    # in one place. To change them, edit that YAML.
                    "detect_opponent_robots": ParameterValue(
                        detect_opponent_robots, value_type=bool
                    ),
                    "opponent_robot_min_confidence_to_show": ParameterValue(
                        opponent_robot_min_confidence_to_show, value_type=int
                    ),
                    "draw_rejected_robot_detections": ParameterValue(
                        draw_rejected_robot_detections, value_type=bool
                    ),
                    "official_robot_tag_height_mm": ParameterValue(
                        official_robot_tag_height_mm, value_type=float
                    ),
                    "ninja_tag_height_mm": ParameterValue(
                        ninja_tag_height_mm, value_type=float
                    ),
                    "opponent_official_robot_tag_height_mm": ParameterValue(
                        opponent_official_robot_tag_height_mm, value_type=float
                    ),
                },
            ],
        ),

        # Bridge the overhead world_state into a clean Pose2D on /ninja/pose.
        # Single-marker pipeline; reads robots.ninja from /overhead/world_state_json.
        # Replaces the old two-marker bridge that used to run on the Ninja Pi.
        Node(
            package="overhead_control",
            executable="ninja_pose_from_overhead",
            name="ninja_pose_from_overhead",
            output="screen",
            parameters=[{
                # -1 = trust whichever marker the overhead camera node selected
                # as the ninja (driven by ninja_aruco_id in the side YAML).
                # This keeps the marker ID defined in ONE place (the YAML).
                "ninja_marker_id": -1,
                # The overhead camera node already publishes the corrected
                # ninja body center in robots.ninja.x_mm/y_mm, so don't apply
                # a second tag-to-center offset here.
                "use_overhead_body_center": True,
                # Tag is mounted ~90 deg off bot-forward; +90 makes /ninja/pose
                # read 0 at true forward. Measured live on the bench.
                "heading_offset_deg": 0.0,
                # Reject pose if more than this many overhead frames have
                # passed without a fresh detection.
                "max_allowed_missed_frames": 10,
            }],
        ),

        # Main bot pose from overhead.
        # Reads robots.main_robot from /overhead/world_state_json.
        # Publishes:
        #   /vision/robot_pose  (metres) -- pose_fuser -> opencr_bridge chain
        #   /main_bot/pose      (mm)     -- for logs and mm-native consumers
        # Unit difference matches each consumer: opencr_bridge multiplies
        # msg.x * 1000.0, so /vision/robot_pose must be in metres.
        Node(
            package="overhead_control",
            executable="main_bot_pose_from_overhead",
            name="main_bot_pose_from_overhead",
            output="screen",
            parameters=[{
                # -1 = trust whichever robot the overhead selected as main_robot.
                # main_robot_aruco_id is set in the side YAML (overhead_blue/yellow.yaml).
                "main_bot_marker_id": -1,
                "use_overhead_body_center": True,
                "heading_offset_deg": 0.0,
                "max_allowed_missed_frames": 10,
            }],
        ),

        # Opponent keep-out circles from overhead.
        # Reads opponent_official_robots from
        # /overhead/world_state_json and publishes inflated circles to
        # /overhead/enemy_obstacles_json.
        # radius = base_radius + tolerance_padding + staleness_growth * missed_frames
        Node(
            package="overhead_control",
            executable="enemy_pose_from_overhead",
            name="enemy_pose_from_overhead",
            output="screen",
            parameters=[{
                "detect_opponent_robots": ParameterValue(
                    detect_opponent_robots, value_type=bool
                ),
                "official_robot_base_radius_mm": 200.0,
                "ninja_base_radius_mm": 150.0,
                "tolerance_padding_mm": 100.0,
                "staleness_growth_mm_per_frame": 30.0,
                "max_staleness_growth_mm": 200.0,
                "min_confidence": ParameterValue(
                    opponent_robot_min_confidence_to_show, value_type=int
                ),
            }],
        ),
    ])
