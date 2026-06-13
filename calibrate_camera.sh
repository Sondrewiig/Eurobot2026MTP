#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"
VIDEO_DEVICE="/dev/video0"
IMAGE_SIZE="[1344,376]"
FPS="[1,15]"

BOARD_SIZE="${1:-8x6}"     # inner corners, e.g. 8x6
SQUARE_SIZE="${2:-0.024}"  # square size in meters, e.g. 0.024 = 24 mm

CALIB_DIR="$WORKSPACE/config"
CALIB_YAML="$CALIB_DIR/zed_left_camera.yaml"
LOG_DIR="$WORKSPACE/logs"
mkdir -p "$CALIB_DIR" "$LOG_DIR"

cleanup() {
    echo
    echo "[INFO] Stopping calibration pipeline..."
    pkill -f "v4l2_camera_node" || true
    pkill -f "zed_left_splitter" || true
    pkill -f "camera_calibration cameracalibrator" || true
}
trap cleanup EXIT INT TERM

echo "[INFO] Sourcing ROS and workspace..."
set +u
source /opt/ros/jazzy/setup.bash
cd "$WORKSPACE"

if [ ! -f "$WORKSPACE/install/setup.bash" ]; then
    echo "[ERROR] Workspace is not built yet."
    echo "Run:"
    echo "  cd $WORKSPACE"
    echo "  colcon build --symlink-install --packages-select main_bot_control"
    exit 1
fi

source "$WORKSPACE/install/setup.bash"
set -u

echo "[INFO] Killing stale processes..."
pkill -f "v4l2_camera_node" || true
pkill -f "zed_left_splitter" || true
pkill -f "camera_calibration cameracalibrator" || true
sleep 1

echo "[INFO] Starting camera..."
nohup ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="$VIDEO_DEVICE" \
  -p pixel_format:=YUYV \
  -p image_size:="$IMAGE_SIZE" \
  -p time_per_frame:="$FPS" \
  -r image_raw:=/zed/image_raw \
  -r camera_info:=/zed/camera_info \
  > "$LOG_DIR/calib_camera.log" 2>&1 &
CAM_PID=$!

sleep 2

echo "[INFO] Starting left-eye splitter..."
nohup ros2 run main_bot_control zed_left_splitter --ros-args \
  -p image_in:=/zed/image_raw \
  -p camera_info_in:=/zed/camera_info \
  -p left_image_out:=/camera/left/image_raw \
  -p right_image_out:=/camera/right/image_raw \
  -p left_camera_info_out:=/camera/left/camera_info \
  -p right_camera_info_out:=/camera/right/camera_info \
  -p left_calibration_yaml:="$CALIB_YAML" \
  -p right_calibration_yaml:="$CALIB_YAML" \
  > "$LOG_DIR/calib_splitter.log" 2>&1 &
SPLIT_PID=$!

sleep 2

echo "[INFO] Starting calibration GUI..."
echo "[INFO] Checkerboard size: $BOARD_SIZE"
echo "[INFO] Square size:       $SQUARE_SIZE m"
echo "[INFO] Target YAML path:  $CALIB_YAML"
echo
echo "[INFO] Show a CHECKERBOARD to the camera, not an ArUco tag."
echo "[INFO] Move it around the frame, near/far, and tilted."
echo "[INFO] When done, click CALIBRATE, then SAVE."
echo

ros2 run camera_calibration cameracalibrator \
  --size "$BOARD_SIZE" \
  --square "$SQUARE_SIZE" \
  --no-service-check \
  image:=/camera/left/image_raw \
  camera:=/camera/left