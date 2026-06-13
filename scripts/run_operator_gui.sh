#!/bin/bash
# run_operator_gui.sh - laptop side
#
# Lightweight wrapper to start the operator GUI on the laptop while
# the overhead and ninja stacks are running on their own machines.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_PATH="${WS_PATH:-$(cd "$SCRIPT_DIR/.." && pwd)}"

source /opt/ros/jazzy/setup.bash
source "$WS_PATH/install/setup.bash"

exec ros2 run overhead_control operator_gui "$@"
