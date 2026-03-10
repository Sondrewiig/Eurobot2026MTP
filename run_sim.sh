#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
cd ~/sondre_ws
colcon build --symlink-install
source ~/sondre_ws/install/setup.bash

gnome-terminal --title="Sondre Telemetry" -- bash -lc "
source /opt/ros/jazzy/setup.bash
source ~/sondre_ws/install/setup.bash
sleep 3
ros2 run sondre_bot_control telemetry_console
exec bash
" &

gnome-terminal --title="Overhead Vision" -- bash -lc "
source /opt/ros/jazzy/setup.bash
source ~/sondre_ws/install/setup.bash
sleep 6
ros2 launch sondre_bot_vision overhead_vision.launch.py
exec bash
" &

ros2 launch sondre_bot_sim sim.launch.py