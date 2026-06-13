#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"
VIDEO_DEVICE="${VIDEO_DEVICE:-}"

CONFIG_DIR="$WORKSPACE/src/main_bot_control/main_bot_control/config"
CAMERA_PARAMS_YAML="$CONFIG_DIR/v4l2_camera.yaml"
CAMERA_CONTROLS_YAML="$CONFIG_DIR/camera_controls.yaml"
TAGS_YAML="$CONFIG_DIR/tags.yaml"

LEFT_CALIB_YAML="$WORKSPACE/config/zed_left_camera.yaml"
RIGHT_CALIB_YAML="$WORKSPACE/config/zed_right_camera.yaml"

OPENCR_PORT="/dev/ttyUSB0"
ACTUATOR_PORT="/dev/ttyUSB1"
OPENCR_BAUD="115200"

LOG_DIR="$WORKSPACE/logs"
mkdir -p "$LOG_DIR"

find_zed_device() {
    # Prefer the stable ZED by-id image stream, but resolve it to /dev/videoX
    # because usb_cam does not reliably accept /dev/v4l/by-id symlinks.
    for d in /dev/v4l/by-id/*ZED*video-index0 /dev/v4l/by-id/*zed*video-index0 /dev/v4l/by-id/*Technologies*video-index0; do
        [ -e "$d" ] || continue
        real="$(readlink -f "$d")"
        if v4l2-ctl -d "$real" --list-formats-ext 2>/dev/null | grep -q "YUYV"; then
            echo "$real"
            return 0
        fi
    done

    # Fallback: scan only devices listed under "ZED 2i".
    while IFS= read -r d; do
        [ -e "$d" ] || continue
        if v4l2-ctl -d "$d" --list-formats-ext 2>/dev/null | grep -q "YUYV"; then
            echo "$d"
            return 0
        fi
    done < <(v4l2-ctl --list-devices | awk '
        /ZED 2i/ {zed=1; next}
        zed && /\/dev\/video/ {gsub(/^[ 	]+/,""); print}
        /^$/ {zed=0}
    ')
}




CAM_PID=""
SPLIT_PID=""
ARUCO_LEFT_PID=""
ARUCO_RIGHT_PID=""
LOC_PID=""
POSE_FUSER_PID=""
OPENCR_PID=""
RBPI_PID=""
TEL_PID=""
ACTUATOR_PID=""


apply_camera_controls() {
    local yaml_file="$1"
    local device="$2"

    echo "[INFO] Applying camera controls from $yaml_file..."

    python3 - "$yaml_file" "$device" <<'PYEOF'
import sys
import subprocess
import yaml

yaml_path, device = sys.argv[1], sys.argv[2]

try:
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}
except Exception as e:
    print(f"[WARN] Could not read {yaml_path}: {e}")
    sys.exit(0)

controls = data.get("camera_controls", {}) or {}
if not controls:
    print("[WARN] No camera_controls block found")
    sys.exit(0)

for name, value in controls.items():
    cmd = ["v4l2-ctl", "-d", device, "-c", f"{name}={value}"]
    print(f"[INFO] {' '.join(cmd)}")
    try:
        subprocess.run(cmd, check=False)
    except Exception as e:
        print(f"[WARN] Failed: {e}")
PYEOF
}


cleanup() {
    echo
    echo "[INFO] Stopping pipeline..."

    pkill -f "v4l2_camera_node" || true
    pkill -f "usb_cam_node_exe" || true
    pkill -f "zed_left_splitter" || true
    pkill -f "aruco_detect" || true
    pkill -f "tag_localization" || true
pkill -f "pose_fuser" || true
    pkill -f "pose_fuser" || true
    pkill -f "opencr_bridge" || true
    pkill -f "actuator_bridge" || true
    pkill -f "rbpi_metrics" || true
    pkill -f "telemetry_console" || true

    sleep 1
}

trap cleanup INT TERM


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


if [ ! -f "$CAMERA_PARAMS_YAML" ]; then
    echo "[ERROR] Missing camera config file: $CAMERA_PARAMS_YAML"
    exit 1
fi

if [ ! -f "$LEFT_CALIB_YAML" ]; then
    echo "[ERROR] Missing left calibration file: $LEFT_CALIB_YAML"
    exit 1
fi

if [ ! -f "$RIGHT_CALIB_YAML" ]; then
    echo "[ERROR] Missing right calibration file: $RIGHT_CALIB_YAML"
    exit 1
fi

if [ ! -f "$TAGS_YAML" ]; then
    echo "[ERROR] Missing tags file: $TAGS_YAML"
    exit 1
fi


echo "[INFO] Killing stale processes..."
pkill -f "v4l2_camera_node" || true
pkill -f "usb_cam_node_exe" || true
pkill -f "zed_left_splitter" || true
pkill -f "aruco_detect" || true
pkill -f "tag_localization" || true
pkill -f "opencr_bridge" || true
    pkill -f "actuator_bridge" || true
pkill -f "rbpi_metrics" || true
pkill -f "telemetry_console" || true
sleep 1


echo "[INFO] Waiting for ZED camera..."

if [ -z "${VIDEO_DEVICE:-}" ]; then
    for i in $(seq 1 20); do
        VIDEO_DEVICE="$(find_zed_device || true)"
        if [ -n "${VIDEO_DEVICE:-}" ] && [ -e "$VIDEO_DEVICE" ]; then
            break
        fi
        sleep 1
    done
fi

if [ -n "${VIDEO_DEVICE:-}" ] && [ -e "$VIDEO_DEVICE" ]; then
    VIDEO_DEVICE="$(readlink -f "$VIDEO_DEVICE")"
fi

if [ -z "${VIDEO_DEVICE:-}" ] || [ ! -e "$VIDEO_DEVICE" ]; then
    echo "[ERROR] Could not find ZED camera device."
    echo "[INFO] Available cameras:"
    v4l2-ctl --list-devices || true
    echo "[INFO] USB devices:"
    lsusb | grep -iE "zed|stereo|2b03" || true
    exit 1
fi

echo "[INFO] Using camera device: $VIDEO_DEVICE"
apply_camera_controls "$CAMERA_CONTROLS_YAML" "$VIDEO_DEVICE"

echo "[INFO] Starting ZED camera with usb_cam raw YUYV"
nohup ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="$VIDEO_DEVICE" \
  -p io_method:=mmap \
  -p pixel_format:=yuyv \
  -p image_width:=1344 \
  -p image_height:=376 \
  -p framerate:=15.0 \
  -p camera_name:=zed \
  -p frame_id:=zed_camera \
  -r image_raw:=/zed/image_raw \
  -r camera_info:=/zed/camera_info \
  > "$LOG_DIR/camera.log" 2>&1 &
CAM_PID=$!

sleep 2

#apply_camera_controls "$CAMERA_CONTROLS_YAML" "$VIDEO_DEVICE"

echo "[INFO] Camera settings come from:"
echo "  $CAMERA_PARAMS_YAML"
echo "  $CAMERA_CONTROLS_YAML"


echo "[INFO] Starting stereo splitter..."
nohup ros2 run main_bot_control zed_left_splitter --ros-args \
  -p image_in:=/zed/image_raw \
  -p camera_info_in:=/zed/camera_info \
  -p left_image_out:=/camera/left/image_raw \
  -p right_image_out:=/camera/right/image_raw \
  -p left_camera_info_out:=/camera/left/camera_info \
  -p right_camera_info_out:=/camera/right/camera_info \
  -p left_calibration_yaml:="$LEFT_CALIB_YAML" \
  -p right_calibration_yaml:="$RIGHT_CALIB_YAML" \
  > "$LOG_DIR/splitter.log" 2>&1 &
SPLIT_PID=$!

sleep 2


echo "[INFO] Starting left ArUco detector..."
nohup ros2 run main_bot_control aruco_detect --ros-args \
  -p image_topic:=/camera/left/image_raw \
  -p camera_info_topic:=/camera/left/camera_info \
  -p tags_yaml:="$TAGS_YAML" \
  -p debug_image_topic:=/aruco/debug_image \
  -p detections_topic:=/aruco/detections_json \
  -p use_left_half_only:=false \
  -p detect_scale:=0.9 \
  -p publish_debug_every_n:=2 \
  -p debug_max_width:=960 \
  > "$LOG_DIR/aruco_left.log" 2>&1 &
ARUCO_LEFT_PID=$!

sleep 2


echo "[INFO] Starting right ArUco detector..."
nohup ros2 run main_bot_control aruco_detect --ros-args \
  -p image_topic:=/camera/right/image_raw \
  -p camera_info_topic:=/camera/right/camera_info \
  -p tags_yaml:="$TAGS_YAML" \
  -p debug_image_topic:=/aruco_right/debug_image \
  -p detections_topic:=/aruco_right/detections_json \
  -p use_left_half_only:=false \
  -p detect_scale:=1.0 \
  -p publish_debug_every_n:=2 \
  -p debug_max_width:=960 \
  -r /aruco_ids:=/aruco_right/ids \
  > "$LOG_DIR/aruco_right.log" 2>&1 &
ARUCO_RIGHT_PID=$!

sleep 2


echo "[INFO] Starting tag localization..."
nohup ros2 run main_bot_control tag_localization --ros-args \
  -p tags_yaml:="$TAGS_YAML" \
  > "$LOG_DIR/tag_localization.log" 2>&1 &
LOC_PID=$!

sleep 1

echo "[INFO] Starting pose fuser..."
nohup ros2 run main_bot_control pose_fuser --ros-args \
  -p overhead_topic:=/vision/robot_pose \
  -p aruco_topic:=/bot_pose_estimate \
  -p odom_topic:=/opencr/odom_pose \
  -p fused_topic:=/bot_pose_fused \
  -p status_topic:=/localization_status \
  > "$LOG_DIR/pose_fuser.log" 2>&1 &
POSE_FUSER_PID=$!

sleep 1


echo "[INFO] Starting OpenCR bridge..."
nohup ros2 run main_bot_control opencr_bridge --ros-args \
  -p port:="$OPENCR_PORT" \
  -p baud:="$OPENCR_BAUD" \
  -p camera_pose_topic:=/bot_pose_fused \
  > "$LOG_DIR/opencr.log" 2>&1 &
OPENCR_PID=$!

sleep 1

echo "[INFO] Starting actuator bridge..."
nohup ros2 run main_bot_control actuator_bridge --ros-args \
  -p port:="$ACTUATOR_PORT" \
  -p baud:=115200 \
  > "$LOG_DIR/actuator.log" 2>&1 &
ACTUATOR_PID=$!

sleep 1


echo "[INFO] Starting Raspberry Pi health metrics publisher..."
nohup ros2 run main_bot_control rbpi_metrics \
  > "$LOG_DIR/rbpi_metrics.log" 2>&1 &
RBPI_PID=$!

sleep 1


echo "[INFO] Starting telemetry GUI..."
nohup ros2 run main_bot_control telemetry_console --ros-args \
  -p tags_yaml:="$TAGS_YAML" \
  -p left_debug_topic:=/aruco/debug_image \
  -p right_debug_topic:=/aruco_right/debug_image \
  -p dustpan_config_yaml:="$CONFIG_DIR/dustpan_config.yaml" \
  > "$LOG_DIR/telemetry.log" 2>&1 &
TEL_PID=$!

sleep 2


echo "[INFO] Pipeline started."
echo "[INFO] PIDs:"
echo "  camera=$CAM_PID"
echo "  splitter=$SPLIT_PID"
echo "  aruco_left=$ARUCO_LEFT_PID"
echo "  aruco_right=$ARUCO_RIGHT_PID"
echo "  localization=$LOC_PID"
echo "  pose_fuser=$POSE_FUSER_PID"
echo "  opencr=$OPENCR_PID"
echo "  actuator=$ACTUATOR_PID"
echo "  rbpi_metrics=$RBPI_PID"
echo "  telemetry=$TEL_PID"
echo
echo "[INFO] Logs:"
echo "  $LOG_DIR/camera.log"
echo "  $LOG_DIR/splitter.log"
echo "  $LOG_DIR/aruco_left.log"
echo "  $LOG_DIR/aruco_right.log"
echo "  $LOG_DIR/tag_localization.log"
echo "  $LOG_DIR/pose_fuser.log"
echo "  $LOG_DIR/opencr.log"
echo "  $LOG_DIR/actuator.log"
echo "  $LOG_DIR/rbpi_metrics.log"
echo "  $LOG_DIR/telemetry.log"
echo
echo "[INFO] Camera settings come from:"
echo "  $CAMERA_PARAMS_YAML"
echo "  $CAMERA_CONTROLS_YAML"
echo
echo "[INFO] Press Ctrl+C in this terminal to stop everything."


while true; do
    sleep 1
done