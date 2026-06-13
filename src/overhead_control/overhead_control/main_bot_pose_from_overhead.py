#!/usr/bin/env python3
"""
Publish main bot pose from the overhead world_state_json.

Mirrors ninja_pose_from_overhead.py but reads robots.main_robot and
publishes on two topics with different units to match each consumer:

Input:
  /overhead/world_state_json   std_msgs/String  JSON from overhead_camera_node

Output:
  /vision/robot_pose           geometry_msgs/Pose2D  x/y in METRES, theta in radians
  /main_bot/pose               geometry_msgs/Pose2D  x/y in MILLIMETRES, theta in radians
  /main_bot/pose_json          std_msgs/String       debug JSON
  /main_bot/pose_status        std_msgs/String       short status

Unit difference is intentional and matches each consumer:
  /vision/robot_pose  -- metres  -- pose_fuser.py passes this through to /bot_pose_fused,
                                    opencr_bridge.py reads that as metres and multiplies
                                    by 1000 before sending SYNC_POSE to the ESP firmware.
                                    This matches how tag_localization.py publishes
                                    /bot_pose_estimate (also metres), and how the sim's
                                    overhead_rectifier_node publishes /vision/robot_pose.
  /main_bot/pose      -- mm     -- consistent with world_state native units and
                                    /ninja/pose, easier to read in logs and rqt.

The main bot tag is assumed to be centered on the robot body
(body_center_source = tag_center_assumed_centered in world_state), so
use_overhead_body_center=True gives the correct body center with no
additional offset needed.  The marker_to_center parameters are kept for
completeness but default to zero.
"""

import json
import math
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String


def norm_angle_rad(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class MainBotPoseFromOverhead(Node):
    def __init__(self) -> None:
        super().__init__('main_bot_pose_from_overhead')

        self.declare_parameter('world_state_topic', '/overhead/world_state_json')

        # /vision/robot_pose (metres) feeds pose_fuser -> opencr_bridge
        self.declare_parameter('vision_pose_topic', '/vision/robot_pose')

        # /main_bot/pose (mm) for debugging and any dedicated consumer
        self.declare_parameter('pose_topic', '/main_bot/pose')

        self.declare_parameter('pose_json_topic', '/main_bot/pose_json')
        self.declare_parameter('status_topic', '/main_bot/pose_status')

        # Set to the main bot's actual ArUco ID (1 for blue side by default).
        # Use -1 to accept whichever marker the overhead selected as main_robot.
        self.declare_parameter('main_bot_marker_id', 1)

        # Extra heading correction if tag forward and robot forward are not aligned.
        self.declare_parameter('heading_offset_deg', 0.0)

        # Keep True -- overhead_camera_node already publishes the corrected body
        # center in robots.main_robot.x_mm/y_mm (tag_center_assumed_centered).
        self.declare_parameter('use_overhead_body_center', True)

        # Only used when use_overhead_body_center=False.
        # Convention: x = forward from marker to body center, y = left.
        self.declare_parameter('marker_to_center_x_mm', 0.0)
        self.declare_parameter('marker_to_center_y_mm', 0.0)

        # Reject the pose if more than this many consecutive overhead frames
        # have passed without a fresh detection.  10 frames ~ 0.5 s at 20 fps.
        self.declare_parameter('max_allowed_missed_frames', 10)

        self.world_state_topic = self.get_parameter('world_state_topic').get_parameter_value().string_value
        self.vision_pose_topic = self.get_parameter('vision_pose_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.pose_json_topic = self.get_parameter('pose_json_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value

        # /vision/robot_pose in metres -- feeds pose_fuser -> opencr_bridge
        self.vision_pose_pub = self.create_publisher(Pose2D, self.vision_pose_topic, 10)

        # /main_bot/pose in mm -- for logs, rqt, and any mm-native consumer
        self.pose_pub = self.create_publisher(Pose2D, self.pose_topic, 10)

        self.pose_json_pub = self.create_publisher(String, self.pose_json_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.sub = self.create_subscription(
            String, self.world_state_topic, self.on_world_state, 10
        )

        self.last_status_time = 0.0
        self.last_publish_time = 0.0
        self.publish_count = 0

        self.get_logger().info(
            f'main_bot_pose_from_overhead started | '
            f'in={self.world_state_topic} | '
            f'out(m)={self.vision_pose_topic} out(mm)={self.pose_topic} | '
            f'main_bot_marker_id={self.get_parameter("main_bot_marker_id").value}'
        )

    def status(self, text: str, throttle_s: float = 1.0) -> None:
        now = time.time()
        if now - self.last_status_time >= throttle_s:
            self.last_status_time = now
            self.get_logger().info(text)
            msg = String()
            msg.data = text
            self.status_pub.publish(msg)

    def extract_main_bot(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        robots = data.get('robots', {})
        main_bot = robots.get('main_robot')
        if not isinstance(main_bot, dict):
            return None
        return main_bot

    def valid_main_bot(self, main_bot: Dict[str, Any]) -> Tuple[bool, str]:
        configured_id = int(self.get_parameter('main_bot_marker_id').value)
        actual_id = main_bot.get('aruco_id', None)
        if configured_id >= 0 and actual_id != configured_id:
            return False, f'wrong_main_bot_id actual={actual_id} expected={configured_id}'

        max_missed = int(self.get_parameter('max_allowed_missed_frames').value)
        missed = int(main_bot.get('missed_frames', 999))
        if missed > max_missed:
            return False, f'main_bot_pose_too_stale missed_frames={missed}'

        if main_bot.get('world_validation', 'ok') != 'ok':
            return False, f'world_validation={main_bot.get("world_validation")}'

        for key in ('heading_deg',):
            if key not in main_bot or main_bot[key] is None:
                return False, f'missing_{key}'

        if bool(self.get_parameter('use_overhead_body_center').value):
            if main_bot.get('x_mm') is None or main_bot.get('y_mm') is None:
                return False, 'missing_body_center'
        else:
            if main_bot.get('tag_center_x_mm') is None or main_bot.get('tag_center_y_mm') is None:
                return False, 'missing_tag_center'

        return True, 'ok'

    def make_pose_mm(self, main_bot: Dict[str, Any]) -> Pose2D:
        """Build pose with x/y in millimetres."""
        heading_deg = (
            float(main_bot['heading_deg'])
            + float(self.get_parameter('heading_offset_deg').value)
        )
        theta = norm_angle_rad(math.radians(heading_deg))

        if bool(self.get_parameter('use_overhead_body_center').value):
            x = float(main_bot['x_mm'])
            y = float(main_bot['y_mm'])
        else:
            tag_x = float(main_bot['tag_center_x_mm'])
            tag_y = float(main_bot['tag_center_y_mm'])
            fwd = float(self.get_parameter('marker_to_center_x_mm').value)
            left = float(self.get_parameter('marker_to_center_y_mm').value)
            x = tag_x + fwd * math.cos(theta) - left * math.sin(theta)
            y = tag_y + fwd * math.sin(theta) + left * math.cos(theta)

        pose = Pose2D()
        pose.x = x
        pose.y = y
        pose.theta = theta
        return pose

    def on_world_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.status(f'bad_world_state_json: {e}')
            return

        main_bot = self.extract_main_bot(data)
        if main_bot is None:
            self.status('no_main_bot_selected_in_world_state')
            return

        ok, reason = self.valid_main_bot(main_bot)
        if not ok:
            self.status(reason)
            return

        # Build pose in mm (native world_state units)
        pose_mm = self.make_pose_mm(main_bot)

        # Publish mm on /main_bot/pose -- for logs and any mm-native consumer
        self.pose_pub.publish(pose_mm)

        # Publish metres on /vision/robot_pose -- for pose_fuser -> opencr_bridge
        # opencr_bridge.sync_camera_pose_timer_cb does x_mm = msg.x * 1000.0
        pose_m = Pose2D()
        pose_m.x = pose_mm.x / 1000.0
        pose_m.y = pose_mm.y / 1000.0
        pose_m.theta = pose_mm.theta
        self.vision_pose_pub.publish(pose_m)

        self.last_publish_time = time.time()
        self.publish_count += 1

        dbg = {
            'stamp': self.last_publish_time,
            'source': 'overhead_world_state_json',
            'aruco_id': main_bot.get('aruco_id'),
            'x_mm': round(pose_mm.x, 1),
            'y_mm': round(pose_mm.y, 1),
            'x_m': round(pose_m.x, 4),
            'y_m': round(pose_m.y, 4),
            'theta_rad': round(pose_mm.theta, 4),
            'heading_deg': round(math.degrees(pose_mm.theta), 1),
            'visible_this_frame': main_bot.get('visible_this_frame'),
            'missed_frames': main_bot.get('missed_frames'),
            'confidence': main_bot.get('confidence'),
            'body_center_source': main_bot.get('body_center_source'),
            'publish_count': self.publish_count,
        }
        out = String()
        out.data = json.dumps(dbg, separators=(',', ':'))
        self.pose_json_pub.publish(out)

        if self.publish_count % 20 == 0:
            self.status(
                f'pose ok id={main_bot.get("aruco_id")} '
                f'x={pose_mm.x:.0f}mm y={pose_mm.y:.0f}mm '
                f'heading={math.degrees(pose_mm.theta):.1f}deg '
                f'count={self.publish_count}',
                throttle_s=0.0,
            )


def main() -> None:
    rclpy.init()
    node = MainBotPoseFromOverhead()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
