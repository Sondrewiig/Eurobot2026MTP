#!/bin/bash
# ROS 2 environment for Tailscale (remote access, any location).
# Fast DDS discovery server runs on the overhead laptop at 100.101.185.35.
#
# Source this in every terminal on any Pi or laptop connected via Tailscale:
#   source <repo-root>/network/ros_tailscale_env.sh

export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=100.101.185.35:11811
export ROS_SUPER_CLIENT=TRUE

unset ROS_STATIC_PEERS
unset ROS_LOCALHOST_ONLY
