#!/bin/bash
# bringup_overhead.sh - run on the laptop
#
# Brings up the overhead camera + ninja pose adapter in a single terminal.
# Adjust the workspace path on the line below.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_PATH="${WS_PATH:-$(cd "$SCRIPT_DIR/.." && pwd)}"

echo "Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash
echo "Sourcing workspace at $WS_PATH..."
source "$WS_PATH/install/setup.bash"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-not_set}"
echo "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-not_set}"

# If you use Fast DDS discovery server, uncomment and set:
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export ROS_DISCOVERY_SERVER=100.101.185.35:11811

echo "Launching overhead..."
exec ros2 launch overhead_control overhead.launch.py "$@"
