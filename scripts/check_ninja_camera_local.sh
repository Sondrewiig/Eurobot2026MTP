#!/bin/bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${EUROBOT_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/ros_tailscale_env.sh

echo "=== ENV ==="
env | grep -E 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|ROS_DISCOVERY_SERVER|ROS_SUPER_CLIENT|ROS_STATIC_PEERS|FASTRTPS_DEFAULT_PROFILES_FILE' || true

echo "=== camera/detector topics ==="
ros2 topic info /camera/image_raw || true
ros2 topic info /ninja/vision/status || true
ros2 topic info /ninja/vision/crate || true
ros2 topic info /ninja/vision/debug_image || true
ros2 topic info /ninja/vision/debug_image/compressed || true

echo "=== status: should print once within 8 sec ==="
timeout 8 ros2 topic echo --once /ninja/vision/status || echo "NO /ninja/vision/status received"

echo "=== crate JSON: should print once within 8 sec ==="
timeout 8 ros2 topic echo --once /ninja/vision/crate || echo "NO /ninja/vision/crate received"

echo "=== compressed debug hz: should be about 1 Hz ==="
timeout 12 ros2 topic hz /ninja/vision/debug_image/compressed || echo "NO compressed debug data received"

echo "=== raw debug hz: should be about 1 Hz ==="
timeout 12 ros2 topic hz /ninja/vision/debug_image || echo "NO raw debug data received"
