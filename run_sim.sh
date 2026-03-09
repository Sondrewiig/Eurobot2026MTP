#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
cd ~/sondre_ws
colcon build
source install/setup.bash

gnome-terminal --title="Sondre Telemetry" -- bash -lc "
source /opt/ros/jazzy/setup.bash
source ~/sondre_ws/install/setup.bash
ros2 run sondre_bot_control telemetry_console
exec bash
" &

ros2 launch sondre_bot_sim sim.launch.py