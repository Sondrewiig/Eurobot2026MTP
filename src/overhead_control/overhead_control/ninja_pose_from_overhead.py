#!/usr/bin/env python3
"""
Step 29 - Publish /ninja/pose from the overhead world_state_json.

Runs on the overhead laptop or on any machine that can see /overhead/world_state_json.

Input:
  /overhead/world_state_json  std_msgs/String JSON

Output:
  /ninja/pose                 geometry_msgs/Pose2D
  /ninja/pose_json            std_msgs/String JSON debug
  /ninja/pose_status          std_msgs/String short status

Pose convention:
  pose.x     = world X in millimetres
  pose.y     = world Y in millimetres
  pose.theta = heading in radians

Default behavior:
  Uses the already-corrected overhead ninja body center: robots.ninja.x_mm/y_mm.
  This avoids double-applying the current ninja tag offset from overhead_camera_node.

If you later want this node to apply marker-to-center offset itself, set:
  use_overhead_body_center:=false
  marker_to_center_x_mm:=...
  marker_to_center_y_mm:=...
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


class NinjaPoseFromOverhead(Node):
    def __init__(self) -> None:
        super().__init__('ninja_pose_from_overhead')

        self.declare_parameter('world_state_topic', '/overhead/world_state_json')
        self.declare_parameter('pose_topic', '/ninja/pose')
        self.declare_parameter('pose_json_topic', '/ninja/pose_json')
        self.declare_parameter('status_topic', '/ninja/pose_status')

        # Set to 57 for your current ninja. Use -1 to accept whichever overhead selected as ninja.
        self.declare_parameter('ninja_marker_id', 57)

        # Extra heading correction if tag forward and robot forward are not aligned.
        self.declare_parameter('heading_offset_deg', 0.0)

        # Keep True with the current overhead_camera_node, because it already publishes ninja body center.
        self.declare_parameter('use_overhead_body_center', True)

        # Only used when use_overhead_body_center=False.
        # Convention: x = forward from marker to body center, y = left from marker to body center.
        self.declare_parameter('marker_to_center_x_mm', 0.0)
        self.declare_parameter('marker_to_center_y_mm', 0.0)

        # Allow a short ArUco miss while overhead smoothing still has a pose.
        self.declare_parameter('max_allowed_missed_frames', 2)

        self.world_state_topic = self.get_parameter('world_state_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.pose_json_topic = self.get_parameter('pose_json_topic').get_parameter_value().string_value
        self.status_topic = self.get_parameter('status_topic').get_parameter_value().string_value

        self.pose_pub = self.create_publisher(Pose2D, self.pose_topic, 10)
        self.pose_json_pub = self.create_publisher(String, self.pose_json_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.sub = self.create_subscription(String, self.world_state_topic, self.on_world_state, 10)

        self.last_status_time = 0.0
        self.last_publish_time = 0.0
        self.publish_count = 0

        self.get_logger().info(
            f'Publishing {self.pose_topic} from {self.world_state_topic}; '
            f'ninja_marker_id={self.get_parameter("ninja_marker_id").value}'
        )

    def status(self, text: str, throttle_s: float = 1.0) -> None:
        now = time.time()
        if now - self.last_status_time >= throttle_s:
            self.last_status_time = now
            self.get_logger().info(text)
            msg = String()
            msg.data = text
            self.status_pub.publish(msg)

    def extract_ninja(self, data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        robots = data.get('robots', {})
        ninja = robots.get('ninja')
        if not isinstance(ninja, dict):
            return None
        return ninja

    def valid_ninja(self, ninja: Dict[str, Any]) -> Tuple[bool, str]:
        configured_id = int(self.get_parameter('ninja_marker_id').value)
        actual_id = ninja.get('aruco_id', None)
        if configured_id >= 0 and actual_id != configured_id:
            return False, f'wrong_ninja_id actual={actual_id} expected={configured_id}'

        max_missed = int(self.get_parameter('max_allowed_missed_frames').value)
        missed = int(ninja.get('missed_frames', 999))
        if missed > max_missed:
            return False, f'ninja_pose_too_stale missed_frames={missed}'

        if ninja.get('world_validation', 'ok') != 'ok':
            return False, f'world_validation={ninja.get("world_validation")}'

        for key in ('heading_deg',):
            if key not in ninja or ninja[key] is None:
                return False, f'missing_{key}'

        if bool(self.get_parameter('use_overhead_body_center').value):
            if ninja.get('x_mm') is None or ninja.get('y_mm') is None:
                return False, 'missing_body_center'
        else:
            if ninja.get('tag_center_x_mm') is None or ninja.get('tag_center_y_mm') is None:
                return False, 'missing_tag_center'

        return True, 'ok'

    def make_pose(self, ninja: Dict[str, Any]) -> Pose2D:
        heading_deg = float(ninja['heading_deg']) + float(self.get_parameter('heading_offset_deg').value)
        theta = norm_angle_rad(math.radians(heading_deg))

        if bool(self.get_parameter('use_overhead_body_center').value):
            x = float(ninja['x_mm'])
            y = float(ninja['y_mm'])
        else:
            tag_x = float(ninja['tag_center_x_mm'])
            tag_y = float(ninja['tag_center_y_mm'])
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

        ninja = self.extract_ninja(data)
        if ninja is None:
            self.status('no_ninja_selected_in_world_state')
            return

        ok, reason = self.valid_ninja(ninja)
        if not ok:
            self.status(reason)
            return

        pose = self.make_pose(ninja)
        self.pose_pub.publish(pose)
        self.last_publish_time = time.time()
        self.publish_count += 1

        dbg = {
            'stamp': self.last_publish_time,
            'source': 'overhead_world_state_json',
            'aruco_id': ninja.get('aruco_id'),
            'x_mm': round(pose.x, 1),
            'y_mm': round(pose.y, 1),
            'theta_rad': round(pose.theta, 4),
            'heading_deg': round(math.degrees(pose.theta), 1),
            'visible_this_frame': ninja.get('visible_this_frame'),
            'missed_frames': ninja.get('missed_frames'),
            'confidence': ninja.get('confidence'),
            'publish_count': self.publish_count,
        }
        out = String()
        out.data = json.dumps(dbg, separators=(',', ':'))
        self.pose_json_pub.publish(out)

        if self.publish_count % 20 == 0:
            self.status(
                f'pose ok id={ninja.get("aruco_id")} x={pose.x:.0f} y={pose.y:.0f} '
                f'heading={math.degrees(pose.theta):.1f} count={self.publish_count}',
                throttle_s=0.0,
            )


def main() -> None:
    rclpy.init()
    node = NinjaPoseFromOverhead()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
