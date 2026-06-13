#!/bin/bash
# ROS 2 environment for D-Link LAN (arena network, 192.168.0.x).
# Fast DDS discovery server runs on the overhead laptop at 192.168.0.10.
#
# Source this in every terminal on any Pi or laptop connected to the D-Link router:
#   source <repo-root>/network/ros_dlink_env.sh

export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=192.168.0.10:11811
export ROS_SUPER_CLIENT=TRUE

unset ROS_STATIC_PEERS
unset ROS_LOCALHOST_ONLY
