#!/bin/bash
# ROS 2 environment for standalone local testing (single machine, no network).
# Use this when the overhead laptop is not running or unreachable.
# Nodes on this machine can talk to each other but NOT to other machines.
#
#   source <repo-root>/network/ros_local_env.sh

export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

unset ROS_DISCOVERY_SERVER
unset ROS_SUPER_CLIENT
unset ROS_STATIC_PEERS
unset ROS_LOCALHOST_ONLY
unset FASTRTPS_DEFAULT_PROFILES_FILE
