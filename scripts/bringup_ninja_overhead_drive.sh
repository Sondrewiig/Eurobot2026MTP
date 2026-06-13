#!/bin/bash
# Drive-only Ninja stack for overhead-camera navigation.
# Run this on the Ninja Pi. It does NOT start the Pi camera or crate alignment.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_PATH="${WS_PATH:-$(cd "$SCRIPT_DIR/.." && pwd)}"

source /opt/ros/jazzy/setup.bash
source "$WS_PATH/install/setup.bash"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-not_set}"
echo "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-not_set}"
echo "Launching Ninja overhead drive-only stack: esp32_bridge + go_to_point"

exec ros2 launch ninja_bot_control ninja_coordinate_drive_tuned.launch.py \
  start_enabled:=false \
  "$@"
