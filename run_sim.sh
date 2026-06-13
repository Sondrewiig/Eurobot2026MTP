#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

printf '=== Main Bot Simulation ===\n\n'

echo "[INFO] Workspace: $WORKSPACE"

source /opt/ros/jazzy/setup.bash

echo "[INFO] Killing stale ROS/Gazebo processes..."

set +e
ros2 daemon stop 2>/dev/null

pkill -9 -u "$USER" -f "$WORKSPACE/install/main_bot_control" 2>/dev/null
pkill -9 -u "$USER" -f "$WORKSPACE/install/main_bot_sim" 2>/dev/null
pkill -9 -u "$USER" -f "$WORKSPACE/install/overhead_control" 2>/dev/null

pkill -9 -u "$USER" -f "opencr_bridge_sim" 2>/dev/null
pkill -9 -u "$USER" -f "pose_fuser_sim" 2>/dev/null
pkill -9 -u "$USER" -f "overhead_rectifier_node" 2>/dev/null
pkill -9 -u "$USER" -f "ground_truth_pose" 2>/dev/null
pkill -9 -u "$USER" -f "tag_localization" 2>/dev/null
pkill -9 -u "$USER" -f "aruco_detect" 2>/dev/null
pkill -9 -u "$USER" -f "apriltag" 2>/dev/null
pkill -9 -u "$USER" -f "stereo_image_proc" 2>/dev/null
pkill -9 -u "$USER" -f "telemetry_console" 2>/dev/null

pkill -9 -u "$USER" -f "ros2" 2>/dev/null
pkill -9 -u "$USER" -f "launch_ros" 2>/dev/null
pkill -9 -u "$USER" -f "parameter_bridge" 2>/dev/null
pkill -9 -u "$USER" -f "image_bridge" 2>/dev/null
pkill -9 -u "$USER" -f "gz sim" 2>/dev/null
pkill -9 -u "$USER" -f "gzserver" 2>/dev/null
pkill -9 -u "$USER" -f "gzclient" 2>/dev/null
pkill -9 -u "$USER" -f "ruby.*gz" 2>/dev/null
pkill -9 -u "$USER" -f "ruby" 2>/dev/null
set -e

sleep 3

rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
rm -f /dev/shm/fast_datasharing_* /dev/shm/sem.fast_datasharing_* 2>/dev/null || true

ros2 daemon start

echo "[INFO] Building workspace..."
cd "$WORKSPACE"

# Force rebuild of the CMake sim package because it installs/copies world assets.
rm -rf build/main_bot_sim install/main_bot_sim

colcon build --symlink-install
source "$WORKSPACE/install/setup.bash"

echo "[INFO] Launching simulation..."
echo ""
echo "  Sim stack:"
echo "    - main_bot_sim: Gazebo world and ROS-Gz bridges"
echo "    - main_bot_control: robot control, localization, telemetry, and simulated OpenCR"
echo "    - overhead_control: simulated overhead camera rectifier and pose publisher"
echo ""
echo "  Use the telemetry command console to drive the robot:"
echo "    go 1.5 1.0 -90"
echo "    go_mm 300 1800 -90"
echo "    stop / estop / go home"
echo ""
echo "[INFO] Press Ctrl+C to stop everything."
echo ""

ros2 launch main_bot_sim sim.launch.py
