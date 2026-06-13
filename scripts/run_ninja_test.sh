#!/bin/bash
# run_ninja_test.sh - ninja Pi side
#
# Brings up the tuned drive stack plus the onboard camera/crate vision.
# Drive stays DISARMED until you publish /ninja/enable_drive true
# (start_enabled defaults to false). This gives you a safety lock that's
# separate from "did a goal arrive".
#
# Test sequence:
#   1. Run this script with wheels lifted.
#   2. On the laptop, confirm /overhead/world_state_json shows the ninja.
#   3. Confirm /ninja/pose is publishing on the Pi: ros2 topic hz /ninja/pose
#   4. Send a tiny goal to verify the geometry (see notes below).
#   5. Only then put the ninja on the granary and arm it.
#
# Arm:
#   ros2 topic pub --once /ninja/enable_drive std_msgs/Bool "{data: true}"
# Disarm (kills /cmd_vel immediately, keeps the goal so you can re-arm):
#   ros2 topic pub --once /ninja/enable_drive std_msgs/Bool "{data: false}"

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_PATH="${WS_PATH:-$(cd "$SCRIPT_DIR/.." && pwd)}"

source /opt/ros/jazzy/setup.bash
source "$WS_PATH/install/setup.bash"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-not_set}"
echo "WS_PATH=$WS_PATH"

if [ ! -e /dev/ttyUSB0 ]; then
    echo "WARNING: /dev/ttyUSB0 not present. ESP32 may be disconnected."
fi

if ! ros2 pkg prefix camera_ros >/dev/null 2>&1; then
    echo "ERROR: camera_ros is not installed. Run: sudo apt install ros-jazzy-camera-ros"
    exit 2
fi

exec ros2 launch ninja_bot_control ninja_full_gui_camera.launch.py \
    start_enabled:=false \
    "$@"
