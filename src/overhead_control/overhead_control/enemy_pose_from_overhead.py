#!/usr/bin/env python3
"""
Publish opponent keep-out circles from the overhead world_state_json.

Reads confirmed opponent robots from /overhead/world_state_json and
publishes an inflated keep-out circle for each one.  No avoidance
reaction lives here -- this node only publishes the geometry.  Wire
/overhead/enemy_obstacles_json into the main bot planner or
drive controller on the bot side.

Input:
  /overhead/world_state_json            std_msgs/String  JSON

Output:
  /overhead/enemy_obstacles_json     std_msgs/String  JSON array
  /overhead/enemy_obstacles_status   std_msgs/String  short status

Obstacle JSON format (one entry per confirmed opponent):
  [
    {
      "aruco_id": 6,
      "kind": "opponent_official_robot",
      "x_mm": 1239.6,
      "y_mm": 1435.9,
      "radius_mm": 350.0,
      "missed_frames": 0,
      "confidence": 20,
      "active": true
    },
    ...
  ]
  Empty array [] when no opponents are confirmed or detection is disabled.

Why circles with tolerance:
  The overhead sees an ArUco marker on top of the opponent robot.  The true
  robot body extends beyond the marker, the exact opponent dimensions are
  unknown, and position estimates become stale between detections.  Each
  opponent is therefore modelled as a keep-out circle:

    radius = base_radius_mm + tolerance_padding_mm
             + staleness_growth_mm_per_frame * missed_frames
    (staleness growth capped at max_staleness_growth_mm)

  base_radius_mm covers the typical half-diagonal of an opponent body.
  tolerance_padding_mm adds an explicit safety buffer.
  Staleness growth inflates the circle when the tracker is coasting
  without a fresh detection, because the bot may have moved.
"""

import json
import time
from typing import Any, Dict, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EnemyPoseFromOverhead(Node):
    def __init__(self) -> None:
        super().__init__('enemy_pose_from_overhead')

        self.declare_parameter('world_state_topic', '/overhead/world_state_json')
        self.declare_parameter('enemy_topic', '/overhead/enemy_obstacles_json')
        self.declare_parameter('status_topic', '/overhead/enemy_obstacles_status')

        # Radius geometry
        self.declare_parameter('official_robot_base_radius_mm', 200.0)
        self.declare_parameter('ninja_base_radius_mm', 150.0)
        self.declare_parameter('tolerance_padding_mm', 100.0)
        self.declare_parameter('staleness_growth_mm_per_frame', 30.0)
        self.declare_parameter('max_staleness_growth_mm', 200.0)

        # Filtering -- use 1 for testing (fast response), 4 for match (fewer false pos)
        self.declare_parameter('min_confidence', 4)
        self.declare_parameter('detect_opponent_robots', True)

        self.enemy_topic = self.get_parameter('enemy_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.world_state_topic = self.get_parameter('world_state_topic').value

        self.enemy_pub = self.create_publisher(String, self.enemy_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.sub = self.create_subscription(
            String, self.world_state_topic, self.on_world_state, 10
        )

        self.last_status_time = 0.0
        self.publish_count = 0

        self.get_logger().info(
            f'enemy_pose_from_overhead started | '
            f'in={self.world_state_topic} | '
            f'out={self.enemy_topic}'
        )

    def status(self, text: str, throttle_s: float = 2.0) -> None:
        now = time.time()
        if now - self.last_status_time >= throttle_s:
            self.last_status_time = now
            self.get_logger().info(text)
            msg = String()
            msg.data = text
            self.status_pub.publish(msg)

    def compute_radius(self, robot: Dict[str, Any]) -> float:
        kind = str(robot.get('kind', ''))
        if 'ninja' in kind:
            base = float(self.get_parameter('ninja_base_radius_mm').value)
        else:
            base = float(self.get_parameter('official_robot_base_radius_mm').value)

        padding = float(self.get_parameter('tolerance_padding_mm').value)
        growth_per_frame = float(self.get_parameter('staleness_growth_mm_per_frame').value)
        max_growth = float(self.get_parameter('max_staleness_growth_mm').value)

        missed = int(robot.get('missed_frames', 0))
        staleness = min(missed * growth_per_frame, max_growth)
        return base + padding + staleness

    def is_confirmed(self, robot: Dict[str, Any]) -> bool:
        min_conf = int(self.get_parameter('min_confidence').value)
        if int(robot.get('confidence', 0)) < min_conf:
            return False
        if robot.get('world_validation', 'ok') != 'ok':
            return False
        return True

    def collect_opponents(self, data: Dict[str, Any]) -> List[Dict[str, Any]]:
        robots = data.get('robots', {})
        confirmed = []
        for robot in robots.get('opponent_official_robots', []) or []:
            if isinstance(robot, dict) and self.is_confirmed(robot):
                confirmed.append(robot)
        return confirmed

    def build_obstacle(self, robot: Dict[str, Any]) -> Dict[str, Any]:
        return {
            'aruco_id': robot.get('aruco_id'),
            'kind': robot.get('kind', 'opponent'),
            'x_mm': round(float(robot.get('x_mm', 0.0)), 1),
            'y_mm': round(float(robot.get('y_mm', 0.0)), 1),
            'radius_mm': round(self.compute_radius(robot), 1),
            'missed_frames': robot.get('missed_frames', 0),
            'confidence': robot.get('confidence', 0),
            'visible_this_frame': robot.get('visible_this_frame', False),
            'active': True,
        }

    def publish_enemy(self, obstacles: List[Dict[str, Any]]) -> None:
        out = String()
        out.data = json.dumps(obstacles, separators=(',', ':'))
        self.enemy_pub.publish(out)

    def on_world_state(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.status(f'bad_world_state_json: {e}')
            return

        if not bool(self.get_parameter('detect_opponent_robots').value):
            self.publish_enemy([])
            self.status('opponent_detection_disabled', throttle_s=5.0)
            return

        opponents = self.collect_opponents(data)
        obstacles = [self.build_obstacle(r) for r in opponents]
        self.publish_enemy(obstacles)
        self.publish_count += 1

        if obstacles:
            ids = [str(o['aruco_id']) for o in obstacles]
            radii = [o['radius_mm'] for o in obstacles]
            self.status(f'obstacles={len(obstacles)} ids={ids} radii={radii}mm', throttle_s=2.0)
        else:
            self.status('no_confirmed_opponents', throttle_s=5.0)


def main() -> None:
    rclpy.init()
    node = EnemyPoseFromOverhead()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
