#!/bin/bash
# bringup_operator_gui.sh - normally run on the laptop
#
# Starts the Ninja operator GUI. The Pi should already be running bringup_ninja.sh.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_PATH="${WS_PATH:-$(cd "$SCRIPT_DIR/.." && pwd)}"

source /opt/ros/jazzy/setup.bash
source "$WS_PATH/install/setup.bash"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-not_set}"
echo "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-not_set}"

exec ros2 run overhead_control operator_gui "$@"
