#!/bin/bash
# run_overhead_test.sh - laptop side
#
# Brings up overhead_camera_node with the values you want for an arena test:
#   - team side blue
#   - ninja marker ID pinned (so the overhead doesn't refuse to pick when
#     multiple ninja-range tags are visible)
#   - publish_images on so you can see /overhead/debug_image and
#     /overhead/topdown_image in rqt_image_view
#
# Override anything by passing extra args, e.g.:
#   ./scripts/run_overhead_test.sh side:=yellow ninja_aruco_id:=75
#
# Optional env: WS_PATH (defaults to this repository root)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_PATH="${WS_PATH:-$(cd "$SCRIPT_DIR/.." && pwd)}"

source /opt/ros/jazzy/setup.bash
source "$WS_PATH/install/setup.bash"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-not_set}"
echo "WS_PATH=$WS_PATH"

# Defaults for our current blue-side test. Override on the command line.
SIDE="${SIDE:-blue}"
NINJA_ID="${NINJA_ID:-57}"
DEVICE="${DEVICE:-/dev/video2}"

exec ros2 launch overhead_control overhead.launch.py \
    side:=${SIDE} \
    device:=${DEVICE} \
    ninja_aruco_id:=${NINJA_ID} \
    publish_images:=true \
    "$@"
