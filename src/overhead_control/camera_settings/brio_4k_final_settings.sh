#!/usr/bin/env bash
# correct_settings.sh
#
# Final tuned Logitech Brio overhead-camera control profile for Eurobot 2026 testing.
#
# Resolution/FPS are selected by overhead_blue.yaml / overhead_yellow.yaml, not by
# this script. The final selected YAML camera profile is:
#   camera_width: 3840
#   camera_height: 2160
#   camera_fps: 30
#   camera_fourcc: "MJPG"
#
# Notes from testing:
# - 4K 30 FPS MJPG gave the best crate detection/orientation stability.
# - 1920x1080 30 FPS gave a faster pose rate but less stable far-side crate detection.
# - Norway power-line frequency is 50 Hz.

set -e

DEVICE="${1:-/dev/video2}"

echo "[BRIO] Applying final tuned overhead camera controls on ${DEVICE}"

v4l2-ctl -d "${DEVICE}" --set-ctrl=brightness=80
v4l2-ctl -d "${DEVICE}" --set-ctrl=contrast=125
v4l2-ctl -d "${DEVICE}" --set-ctrl=saturation=120

# Manual white balance from tuning.
v4l2-ctl -d "${DEVICE}" --set-ctrl=white_balance_automatic=0
v4l2-ctl -d "${DEVICE}" --set-ctrl=white_balance_temperature=4500

# Keep gain low to avoid noisy ArUco edges.
v4l2-ctl -d "${DEVICE}" --set-ctrl=gain=0

# Norway / Europe mains flicker compensation.
v4l2-ctl -d "${DEVICE}" --set-ctrl=power_line_frequency=1

# Sharpening helped far tags, but very high values may add noise.
v4l2-ctl -d "${DEVICE}" --set-ctrl=sharpness=225

# This value worked in the current tested setup.
v4l2-ctl -d "${DEVICE}" --set-ctrl=backlight_compensation=1

# Manual exposure from tuning.
v4l2-ctl -d "${DEVICE}" --set-ctrl=auto_exposure=1
v4l2-ctl -d "${DEVICE}" --set-ctrl=exposure_time_absolute=100
v4l2-ctl -d "${DEVICE}" --set-ctrl=exposure_dynamic_framerate=0

# Manual fixed focus from tuning.
v4l2-ctl -d "${DEVICE}" --set-ctrl=focus_automatic_continuous=0
v4l2-ctl -d "${DEVICE}" --set-ctrl=focus_absolute=4

echo "[BRIO] Applied. Current selected controls:"
v4l2-ctl -d "${DEVICE}" --list-ctrls | grep -Ei "focus|exposure|gain|white|sharp|brightness|contrast|saturation|backlight|power" || true

echo
echo "[BRIO] Reminder: overhead_blue.yaml / overhead_yellow.yaml should use:"
echo "  camera_width: 3840"
echo "  camera_height: 2160"
echo "  camera_fps: 30"
echo "  camera_fourcc: \"MJPG\""
