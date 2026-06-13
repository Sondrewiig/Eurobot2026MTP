"""
ninja_coordinate_drive_tuned.launch.py

Ninja Pi-side drive stack: esp32_bridge + go_to_point.

Pose is computed on the laptop (overhead_control/ninja_pose_from_overhead)
and arrives on the Pi via /ninja/pose over ROS. This launch contains only
the local motion stack.

Parameters are tuned for overhead-guided navigation: the bridge rate matches
the controller correction rate, turn scale is limited to prevent overshoot
under overhead feedback lag, and the reverse guard is relaxed enough to allow
in-place rotation while still protecting against unintended reverse during
forward drive.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    start_enabled = LaunchConfiguration("start_enabled")

    bridge_command_rate_hz = LaunchConfiguration("bridge_command_rate_hz")
    bridge_max_pwm = LaunchConfiguration("bridge_max_pwm")
    bridge_min_pwm = LaunchConfiguration("bridge_min_pwm")
    bridge_turn_scale = LaunchConfiguration("bridge_turn_scale")
    bridge_drive_guard_norm = LaunchConfiguration("bridge_drive_guard_norm")
    bridge_max_pwm_step_per_tick = LaunchConfiguration(
        "bridge_max_pwm_step_per_tick"
    )
    bridge_linear_full_scale_mps = LaunchConfiguration(
        "bridge_linear_full_scale_mps"
    )
    bridge_angular_full_scale_radps = LaunchConfiguration(
        "bridge_angular_full_scale_radps"
    )

    gtp_max_linear_mps = LaunchConfiguration("gtp_max_linear_mps")
    gtp_max_angular_radps = LaunchConfiguration("gtp_max_angular_radps")
    gtp_k_linear = LaunchConfiguration("gtp_k_linear")
    gtp_k_angular = LaunchConfiguration("gtp_k_angular")
    gtp_turn_in_place_threshold_deg = LaunchConfiguration(
        "gtp_turn_in_place_threshold_deg"
    )
    gtp_xy_tolerance_mm = LaunchConfiguration("gtp_xy_tolerance_mm")
    gtp_heading_tolerance_deg = LaunchConfiguration("gtp_heading_tolerance_deg")
    gtp_use_goal_heading = LaunchConfiguration("gtp_use_goal_heading")
    gtp_slowdown_radius_mm = LaunchConfiguration("gtp_slowdown_radius_mm")
    gtp_pose_timeout_s = LaunchConfiguration("gtp_pose_timeout_s")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("start_enabled", default_value="false"),

        DeclareLaunchArgument("bridge_command_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("bridge_max_pwm", default_value="220"),
        DeclareLaunchArgument("bridge_min_pwm", default_value="150"),
        # Gentler turning authority: a given angular command makes a smaller
        # wheel differential, so the bot pivots slowly instead of whipping.
        DeclareLaunchArgument("bridge_turn_scale", default_value="0.45"),
        DeclareLaunchArgument("bridge_drive_guard_norm", default_value="0.20"),
        # Slower slew so a turn command can't slam to full spin in one tick.
        DeclareLaunchArgument(
            "bridge_max_pwm_step_per_tick", default_value="20"
        ),
        DeclareLaunchArgument(
            "bridge_linear_full_scale_mps", default_value="0.20"
        ),
        DeclareLaunchArgument(
            "bridge_angular_full_scale_radps", default_value="1.20"
        ),

        DeclareLaunchArgument("gtp_max_linear_mps", default_value="0.04"),
        # Lower spin cap so overshoot is small.
        DeclareLaunchArgument("gtp_max_angular_radps", default_value="0.18"),
        DeclareLaunchArgument("gtp_k_linear", default_value="0.8"),
        # For the back-and-forth spin. With
        # overhead feedback lag, high angular gain overshoots and oscillates.
        DeclareLaunchArgument("gtp_k_angular", default_value="0.50"),
        # Don't demand near-perfect aim before driving.
        # Letting it drive forward while gently correcting damps the spin-hunt.
        DeclareLaunchArgument(
            "gtp_turn_in_place_threshold_deg", default_value="45.0"
        ),
        # Higher tolerance for counting the last few mm.
        DeclareLaunchArgument("gtp_xy_tolerance_mm", default_value="30.0"),
        DeclareLaunchArgument("gtp_heading_tolerance_deg", default_value="10.0"),
        DeclareLaunchArgument("gtp_use_goal_heading", default_value="false"),
        DeclareLaunchArgument(
            "gtp_slowdown_radius_mm", default_value="350.0"
        ),
        # Stop driving if /ninja/pose is stale by more than this many seconds.
        DeclareLaunchArgument("gtp_pose_timeout_s", default_value="0.5"),

        Node(
            package="ninja_bot_control",
            executable="esp32_bridge",
            name="esp32_bridge",
            output="screen",
            parameters=[{
                "port": port,
                "command_rate_hz": ParameterValue(
                    bridge_command_rate_hz, value_type=float
                ),
                "max_pwm": ParameterValue(bridge_max_pwm, value_type=int),
                "min_pwm": ParameterValue(bridge_min_pwm, value_type=int),
                "turn_scale": ParameterValue(
                    bridge_turn_scale, value_type=float
                ),
                "drive_guard_norm": ParameterValue(
                    bridge_drive_guard_norm, value_type=float
                ),
                "max_pwm_step_per_tick": ParameterValue(
                    bridge_max_pwm_step_per_tick, value_type=int
                ),
                "linear_full_scale_mps": ParameterValue(
                    bridge_linear_full_scale_mps, value_type=float
                ),
                "angular_full_scale_radps": ParameterValue(
                    bridge_angular_full_scale_radps, value_type=float
                ),
                "prevent_reverse_while_driving": True,
                "send_only_on_change": False,
                "pwm_change_threshold": 2,
                "resend_interval_s": 0.05,
            }],
        ),

        Node(
            package="ninja_bot_control",
            executable="go_to_point",
            name="ninja_go_to_point",
            output="screen",
            parameters=[{
                "start_enabled": ParameterValue(
                    start_enabled, value_type=bool
                ),
                "pose_topic": "/ninja/pose",
                "goal_topic": "/ninja/goal_pose",
                "cmd_vel_topic": "/cmd_vel",

                "max_linear_mps": ParameterValue(
                    gtp_max_linear_mps, value_type=float
                ),
                "max_angular_radps": ParameterValue(
                    gtp_max_angular_radps, value_type=float
                ),
                "k_linear": ParameterValue(gtp_k_linear, value_type=float),
                "k_angular": ParameterValue(gtp_k_angular, value_type=float),
                "turn_in_place_threshold_deg": ParameterValue(
                    gtp_turn_in_place_threshold_deg, value_type=float
                ),
                "xy_tolerance_mm": ParameterValue(
                    gtp_xy_tolerance_mm, value_type=float
                ),
                "heading_tolerance_deg": ParameterValue(
                    gtp_heading_tolerance_deg, value_type=float
                ),
                "use_goal_heading": ParameterValue(
                    gtp_use_goal_heading, value_type=bool
                ),
                "slowdown_radius_mm": ParameterValue(
                    gtp_slowdown_radius_mm, value_type=float
                ),
                "pose_timeout_s": ParameterValue(
                    gtp_pose_timeout_s, value_type=float
                ),
            }],
        ),
    ])
