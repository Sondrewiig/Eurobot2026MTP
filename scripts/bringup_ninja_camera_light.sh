#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${WS_PATH:-$(cd "$SCRIPT_DIR/.." && pwd)}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/ros_tailscale_env.sh

pkill -f camera_node || true
pkill -f crate_detector || true
pkill -f crate_align || true

ros2 launch ninja_bot_control ninja_camera_ros_vision.launch.py \
  width:=320 \
  height:=240 \
  debug_image_rate_hz:=0.5 \
  debug_image_max_width:=160 \
  process_every_n_frames:=2 \
  show_window:=false
