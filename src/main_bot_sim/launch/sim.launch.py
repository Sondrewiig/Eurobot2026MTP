from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import re
import tempfile
import math
import yaml


def _read_team_from_control_tuning(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        cfg = data.get("opencr_bridge_sim", data)
        team = str(cfg.get("team", {}).get("color", "yellow")).strip().lower()
        return team if team in ("yellow", "blue") else "yellow"
    except Exception:
        return "yellow"


def _wrap_pi(angle_rad):
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _read_control_tuning(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return data.get("opencr_bridge_sim", data) or {}
    except Exception:
        return {}


def _read_team_from_control_tuning(path):
    cfg = _read_control_tuning(path)
    team = str(cfg.get("team", {}).get("color", "yellow")).strip().lower()
    return team if team in ("yellow", "blue") else "yellow"


def _spawn_pose_for_team_from_control_tuning(path, requested_team):
    """Return requested team spawn pose as x_m, y_m, yaw_rad.

    Uses opencr_bridge_sim/team/spawn from control_tuning.yaml.
    If the spawn block is missing, falls back to home and mirrors x/yaw for blue.
    """
    cfg = _read_control_tuning(path)
    team_cfg = cfg.get("team", {}) if isinstance(cfg, dict) else {}
    team = str(requested_team).strip().lower()
    if team not in ("yellow", "blue"):
        team = "yellow"

    spawn_cfg = team_cfg.get("spawn", {}) if isinstance(team_cfg, dict) else {}
    pose_cfg = spawn_cfg.get(team, {}) if isinstance(spawn_cfg, dict) else {}

    if isinstance(pose_cfg, dict) and {"x_mm", "y_mm", "yaw_deg"}.issubset(pose_cfg.keys()):
        x_mm = float(pose_cfg["x_mm"])
        y_mm = float(pose_cfg["y_mm"])
        yaw_deg = float(pose_cfg["yaw_deg"])
    else:
        # Fallback: use home pose, and mirror x/yaw if requested team is blue.
        home = cfg.get("home", {}) if isinstance(cfg, dict) else {}
        x_mm = float(home.get("x_mm", 300.0))
        y_mm = float(home.get("y_mm", 1800.0))
        yaw_deg = float(home.get("yaw_deg", 90.0))
        if team == "blue":
            mirror_x = float(team_cfg.get("mirror_line_x_mm", 1500.0))
            x_mm = 2.0 * mirror_x - x_mm
            yaw_deg = math.degrees(_wrap_pi(math.pi - math.radians(yaw_deg)))

    return x_mm / 1000.0, y_mm / 1000.0, _wrap_pi(math.radians(yaw_deg))


def _team_spawn_pose_from_control_tuning(path):
    team = _read_team_from_control_tuning(path)
    x_m, y_m, yaw_rad = _spawn_pose_for_team_from_control_tuning(path, team)
    return team, x_m, y_m, yaw_rad


def _tag_config_from_control_tuning(path):
    """Return active/dummy robot tag IDs derived from team.color.

    Blue robots must use ArUco IDs 1-5. Yellow robots must use IDs 6-10.
    control_tuning.yaml can pick the exact ID inside each allowed range.
    """
    cfg = _read_control_tuning(path)
    team = _read_team_from_control_tuning(path)
    team_cfg = cfg.get("team", {}) if isinstance(cfg, dict) else {}
    tag_cfg = team_cfg.get("aruco_tags", {}) if isinstance(team_cfg, dict) else {}

    active_blue = int(tag_cfg.get("active_blue_id", 1))
    active_yellow = int(tag_cfg.get("active_yellow_id", 6))
    dummy_blue = int(tag_cfg.get("dummy_blue_id", 2))
    dummy_yellow = int(tag_cfg.get("dummy_yellow_id", 7))

    def clamp_allowed(tag_id, allowed, fallback):
        return tag_id if tag_id in allowed else fallback

    active_blue = clamp_allowed(active_blue, range(1, 6), 1)
    dummy_blue = clamp_allowed(dummy_blue, range(1, 6), 2)
    active_yellow = clamp_allowed(active_yellow, range(6, 11), 6)
    dummy_yellow = clamp_allowed(dummy_yellow, range(6, 11), 7)

    if team == "blue":
        return {
            "team": team,
            "active_tag_id": active_blue,
            "dummy_team": "yellow",
            "dummy_tag_id": dummy_yellow,
            "enemy_tag_ids": [6, 7, 8, 9, 10],
        }

    return {
        "team": team,
        "active_tag_id": active_yellow,
        "dummy_team": "blue",
        "dummy_tag_id": dummy_blue,
        "enemy_tag_ids": [1, 2, 3, 4, 5],
    }


def _bot_tag_texture_uri(sim_share, tag_id):
    return f"file://{os.path.join(sim_share, 'materials', 'textures', f'Bot_ID_{int(tag_id):02d}.png')}"


def _dummy_bot_model_sdf(name, x_m, y_m, yaw_rad, tag_uri, tag_id):
    pose = f"{x_m:.3f} {y_m:.3f} 0 0 0 {yaw_rad:.12f}"
    return f"""

<!-- Static opponent dummy used only for perception/avoidance development. -->
<model name="{name}" canonical_link="chassis">
  <static>true</static>
  <self_collide>false</self_collide>
  <pose>{pose}</pose>

  <link name="chassis">
    <visual name="dummy_chassis_visual">
      <pose>0.10 0 0.20 0 0 0</pose>
      <geometry>
        <box><size>0.25 0.30 0.35</size></box>
      </geometry>
      <material>
        <ambient>0.08 0.08 0.08 1</ambient>
        <diffuse>0.08 0.08 0.08 1</diffuse>
        <specular>0.15 0.15 0.15 1</specular>
      </material>
    </visual>

    <collision name="dummy_chassis_collision">
      <pose>0.10 0 0.18 0 0 0</pose>
      <geometry>
        <box><size>0.25 0.30 0.35</size></box>
      </geometry>
    </collision>

    <visual name="dummy_tag_post_visual">
      <pose>0.10 0 0.3875 0 0 0</pose>
      <geometry>
        <box><size>0.02 0.02 0.025</size></box>
      </geometry>
      <material>
        <ambient>0.15 0.15 0.15 1</ambient>
        <diffuse>0.15 0.15 0.15 1</diffuse>
        <specular>0.05 0.05 0.05 1</specular>
      </material>
    </visual>

    <visual name="dummy_tag_backing_visual">
      <pose>0.10 0 0.4015 0 0 0</pose>
      <geometry>
        <box><size>0.12 0.12 0.003</size></box>
      </geometry>
      <material>
        <ambient>0 0 0 1</ambient>
        <diffuse>0 0 0 1</diffuse>
        <specular>0 0 0 1</specular>
      </material>
    </visual>

    <visual name="dummy_bot_tag_{int(tag_id):02d}_visual">
      <pose>0.10 0 0.4045 0 0 0</pose>
      <geometry>
        <box><size>0.10 0.10 0.002</size></box>
      </geometry>
      <cast_shadows>false</cast_shadows>
      <material>
        <ambient>1 1 1 1</ambient>
        <diffuse>1 1 1 1</diffuse>
        <specular>0 0 0 1</specular>
        <pbr>
          <metal>
            <albedo_map>{tag_uri}</albedo_map>
            <roughness>1.0</roughness>
            <metalness>0.0</metalness>
          </metal>
        </pbr>
      </material>
    </visual>
  </link>
</model>
"""


def _make_team_spawn_world(base_world_path, control_tuning_yaml, sim_share):
    team, x_m, y_m, yaw_rad = _team_spawn_pose_from_control_tuning(control_tuning_yaml)
    tag_cfg = _tag_config_from_control_tuning(control_tuning_yaml)
    active_tag_id = tag_cfg["active_tag_id"]
    dummy_team = tag_cfg["dummy_team"]
    dummy_tag_id = tag_cfg["dummy_tag_id"]
    dummy_x_m, dummy_y_m, dummy_yaw_rad = _spawn_pose_for_team_from_control_tuning(control_tuning_yaml, dummy_team)

    active_tag_uri = _bot_tag_texture_uri(sim_share, active_tag_id)
    dummy_tag_uri = _bot_tag_texture_uri(sim_share, dummy_tag_id)

    with open(base_world_path, "r", encoding="utf-8") as f:
        sdf = f.read()

    # Make texture paths portable after moving the simulation package into this repo.
    playground_uri = f"file://{os.path.join(sim_share, 'materials', 'textures', 'playground_mat.png')}"
    sdf = re.sub(
        r"file://[^<]*materials/textures/playground_mat\.png",
        playground_uri,
        sdf,
    )

    # Replace only the top-level pose directly under model name="main_bot".
    pattern = (
        r'(<model name="main_bot"[^>]*>.*?'
        r'<!-- Initial position of BOT--\s*>\s*<pose>)'
        r'([^<]+)'
        r'(</pose>)'
    )

    replacement_pose = f"{x_m:.3f} {y_m:.3f} 0 0 0 {yaw_rad:.12f}"
    new_sdf, n = re.subn(pattern, r"\g<1>" + replacement_pose + r"\g<3>", sdf, count=1, flags=re.S)
    if n != 1:
        # Fallback if the comment text has changed, still constrained to the bot model.
        pattern = r'(<model name="main_bot"[^>]*>\s*<self_collide>.*?</self_collide>\s*<pose>)([^<]+)(</pose>)'
        new_sdf, n = re.subn(pattern, r"\g<1>" + replacement_pose + r"\g<3>", sdf, count=1, flags=re.S)

    if n != 1:
        raise RuntimeError("Could not find main_bot initial <pose> in building_robot.sdf")

    # Replace the active robot's top tag texture with the team-selected ArUco ID.
    tag_pattern = (
        r'(<visual name="bot_tag_[^"]*_visual"[^>]*>.*?'
        r'<albedo_map>)'
        r'([^<]+)'
        r'(</albedo_map>)'
    )
    new_sdf, n = re.subn(tag_pattern, r"\g<1>" + active_tag_uri + r"\g<3>", new_sdf, count=1, flags=re.S)
    if n != 1:
        raise RuntimeError("Could not find main_bot top tag albedo_map in building_robot.sdf")

    new_sdf = re.sub(
        r'<visual name="bot_tag_[^"]*_visual">',
        f'<visual name="bot_tag_{active_tag_id:02d}_visual">',
        new_sdf,
        count=1,
    )

    # Add one static opponent bot in the opposite spawn area. This is only a
    # visible/collidable dummy; it has no controllers and no avoidance behavior.
    dummy_sdf = _dummy_bot_model_sdf(
        name="opponent_dummy_bot",
        x_m=dummy_x_m,
        y_m=dummy_y_m,
        yaw_rad=dummy_yaw_rad,
        tag_uri=dummy_tag_uri,
        tag_id=dummy_tag_id,
    )
    new_sdf, n = re.subn(r'(</world>\s*</sdf>)', dummy_sdf + r"\n\g<1>", new_sdf, count=1, flags=re.S)
    if n != 1:
        raise RuntimeError("Could not append opponent_dummy_bot before </world>")

    out_path = os.path.join(tempfile.gettempdir(), f"main_bot_{team}_spawn_world.sdf")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(new_sdf)

    enemy_tag_ids = tag_cfg["enemy_tag_ids"]
    print(
        f"[sim.launch] team={team} spawn_pose={replacement_pose} active_tag={active_tag_id} "
        f"dummy_team={dummy_team} dummy_tag={dummy_tag_id} enemy_tags={enemy_tag_ids} world={out_path}"
    )
    return out_path, active_tag_id, enemy_tag_ids


def generate_launch_description():
    sim_share = get_package_share_directory("main_bot_sim")
    control_share = get_package_share_directory("main_bot_control")
    base_world_path = os.path.join(sim_share, "worlds", "building_robot.sdf")
    tags_yaml = os.path.join(control_share, "config", "tags.yaml")
    control_tuning_yaml = os.path.join(control_share, "config", "control_tuning.yaml")
    world_path, active_robot_tag_id, enemy_robot_tag_ids = _make_team_spawn_world(base_world_path, control_tuning_yaml, sim_share)
    overhead_team = _read_team_from_control_tuning(control_tuning_yaml)
    overhead_topic_base = f"/overhead_cam_{overhead_team}"

    # Spawn pose in world frame — needed by pose_fuser and pose_logger for odom→world.
    _team, spawn_x_m, spawn_y_m, spawn_yaw_rad = _team_spawn_pose_from_control_tuning(control_tuning_yaml)

    # --- Gazebo ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    # --- Bridge (Gazebo <-> ROS) ---
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/camera/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/left/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/camera/right/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/world/Eurobot_world/dynamic_pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/world/Eurobot_world/pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/dustpan/angle@std_msgs/msg/Float64@gz.msgs.Double",
            "/carwash/arm_angle@std_msgs/msg/Float64@gz.msgs.Double",
            "/carwash/roller_speed@std_msgs/msg/Float64@gz.msgs.Double",
        ],
        output="screen",
    )

    # --- Stereo processing ---
    stereo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("stereo_image_proc"),
                "launch",
                "stereo_image_proc.launch.py",
            )
        ),
        launch_arguments={"namespace": "camera"}.items(),
    )

    # --- AprilTag detector ---
    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("main_bot_sim"),
                "config",
                "apriltag.yaml",
            ])
        ],
        remappings=[
            ("image_rect", "/camera/left/image_raw"),
            ("camera_info", "/camera/left/camera_info"),
        ],
    )

    # --- Left ArUco detector ---
    aruco_left = Node(
        package="main_bot_control",
        executable="aruco_detect",
        name="aruco_detect_left",
        output="screen",
        parameters=[
            {"image_topic": "/camera/left/image_raw"},
            {"camera_info_topic": "/camera/left/camera_info"},
            {"tags_yaml": tags_yaml},
            {"debug_image_topic": "/aruco/debug_image"},
            {"detections_topic": "/aruco/detections_json"},
            {"use_left_half_only": False},
            {"detect_scale": 0.9},
            {"publish_debug_every_n": 2},
            {"debug_max_width": 640},
            {"use_sim_time": True},
        ],
    )

    # --- Right ArUco detector ---
    aruco_right = Node(
        package="main_bot_control",
        executable="aruco_detect",
        name="aruco_detect_right",
        output="screen",
        parameters=[
            {"image_topic": "/camera/right/image_raw"},
            {"camera_info_topic": "/camera/right/camera_info"},
            {"tags_yaml": tags_yaml},
            {"debug_image_topic": "/aruco_right/debug_image"},
            {"detections_topic": "/aruco_right/detections_json"},
            {"use_left_half_only": False},
            {"detect_scale": 0.9},
            {"publish_debug_every_n": 2},
            {"debug_max_width": 640},
            {"use_sim_time": True},
        ],
        remappings=[("/aruco_ids", "/aruco_right/ids")],
    )

    # --- Tag localization ---
    tag_localization = Node(
        package="main_bot_control",
        executable="tag_localization",
        output="screen",
        parameters=[
            {"tags_yaml": tags_yaml},
            {"use_sim_time": True},
        ],
    )

    # --- Pose fuser ---
    pose_fuser = Node(
        package="main_bot_control",
        executable="pose_fuser_sim",
        name="pose_fuser_sim",
        output="screen",
        parameters=[
            os.path.join(control_share, "config", "pose_fuser.yaml"),
            {
                "spawn_x_m": spawn_x_m,
                "spawn_y_m": spawn_y_m,
                "spawn_yaw_rad": spawn_yaw_rad,
            },
        ],
    )

    # --- Ground truth from Gazebo ---
    ground_truth = Node(
        package="main_bot_control",
        executable="ground_truth_pose",
        name="ground_truth_pose",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"tf_topic": "/world/Eurobot_world/dynamic_pose/info"},
            {"pose_index": 0},
            {"publish_topic": "/bot_pose_ground_truth"},
        ],
    )

    enemy_ground_truth = Node(
        package="main_bot_control",
        executable="ground_truth_pose",
        name="enemy_ground_truth_pose",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"tf_topic": "/world/Eurobot_world/pose/info"},
            {"pose_index": -1},
            {"publish_topic": "/enemy_pose_ground_truth"},
        ],
    )

    # --- Simulated ESP32 bridge (replaces serial opencr_bridge) ---
    opencr_sim = Node(
        package="main_bot_control",
        executable="opencr_bridge_sim",
        name="opencr_bridge",
        output="screen",
        parameters=[
            {"control_config_path": control_tuning_yaml},
            {"use_sim_time": True},
        ],
    )

    # --- Telemetry console GUI ---
    telemetry = Node(
        package="main_bot_control",
        executable="telemetry_console",
        output="screen",
        parameters=[
            {"tags_yaml": tags_yaml},
            {"left_debug_topic": "/aruco/debug_image"},
            {"right_debug_topic": "/aruco_right/debug_image"},
            {"use_sim_time": True},
        ],
    )

    # --- Overhead vision pipeline ---
    overhead_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[f"{overhead_topic_base}/image"],
        output="screen",
    )

    overhead_camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"{overhead_topic_base}/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
        ],
        output="screen",
    )

    overhead_rectifier = Node(
        package="overhead_control",
        executable="overhead_rectifier_node",
        name="overhead_rectifier_node",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("overhead_control"),
                "config",
                "overhead_tags.yaml",
            ]),
            {"control_config_path": control_tuning_yaml},
            {"robot_tag_id": active_robot_tag_id},
            {"enemy_tag_ids": enemy_robot_tag_ids},
        ],
        output="screen",
    )

    # --- Pose logger (CSV recording) ---
    pose_logger = Node(
        package="main_bot_control",
        executable="pose_logger",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {
                "spawn_x_m": spawn_x_m,
                "spawn_y_m": spawn_y_m,
                "spawn_yaw_rad": spawn_yaw_rad,
            },
        ],
    )

    return LaunchDescription([
        gazebo,
        bridge,
        stereo,
        apriltag,
        aruco_left,
        aruco_right,
        tag_localization,
        pose_fuser,
        ground_truth,
        enemy_ground_truth,
        opencr_sim,
        telemetry,
        overhead_image_bridge,
        overhead_camera_info_bridge,
        overhead_rectifier,
        pose_logger,
    ])