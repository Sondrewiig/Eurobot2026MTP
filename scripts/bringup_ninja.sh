#!/bin/bash
# bringup_ninja.sh - run on the Ninja Pi
#
# Full Ninja stack for the operator GUI workflow:
#   tuned drive stack + camera_ros CSI camera + crate detector + crate align.
#
# The laptop GUI subscribes to /ninja/vision/debug_image and /ninja/vision/align_status.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_PATH="${WS_PATH:-$(cd "$SCRIPT_DIR/.." && pwd)}"

source /opt/ros/jazzy/setup.bash
source "$WS_PATH/install/setup.bash"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-not_set}"
echo "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-not_set}"

if [ ! -e /dev/ttyUSB0 ]; then
    echo "WARNING: /dev/ttyUSB0 not present. ESP32 may be disconnected."
fi

if ! ros2 pkg prefix camera_ros >/dev/null 2>&1; then
    echo "ERROR: camera_ros is not installed. Run: sudo apt install ros-jazzy-camera-ros"
    exit 2
fi

echo "Launching full Ninja GUI/camera stack..."
exec ros2 launch ninja_bot_control ninja_full_gui_camera.launch.py "$@"
