#!/bin/bash
# Quick check after Pi bringup. Run on either machine after sourcing workspace.
set -e

for topic in \
    /camera/image_raw \
    /ninja/vision/crate \
    /ninja/vision/debug_image \
    /ninja/vision/align_status \
    /ninja/pose; do
    echo "=== $topic ==="
    ros2 topic info "$topic" || true
    echo
done
