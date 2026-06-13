# Eurobot Overhead — Useful Commands

Keep this file short. Use it for commands you actually run.

---

## 1. Build / source

### Normal build after code changes

```bash
cd ~/Eurobot2026MTP
source /opt/ros/jazzy/setup.bash
colcon build --packages-select overhead_control --symlink-install
source install/setup.bash
```

### Clean rebuild if build/install is stale

Use this after changing `setup.py`, launch files, config install files, or if ROS uses old files.

```bash
cd ~/Eurobot2026MTP
rm -rf build/overhead_control install/overhead_control log
source /opt/ros/jazzy/setup.bash
colcon build --packages-select overhead_control --symlink-install
source install/setup.bash
```

### Source in a new terminal

```bash
cd ~/Eurobot2026MTP
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Optional `~/.bashrc` lines:

```bash
source /opt/ros/jazzy/setup.bash
source ~/Eurobot2026MTP/install/setup.bash
```

---

## 2. Start overhead

### Apply saved Brio 4K settings

```bash
cd ~/Eurobot2026MTP
./src/overhead_control/camera_settings/brio_4k_manual.sh
```

### Launch blue side

```bash
ros2 launch overhead_control overhead.launch.py side:=blue publish_images:=true
```

### Launch yellow side

```bash
ros2 launch overhead_control overhead.launch.py side:=yellow publish_images:=true
```

---

## 3. View images

### Camera debug image

```bash
ros2 run image_tools showimage --ros-args -r image:=/overhead/debug_image
```

### Top-down image

```bash
ros2 run image_tools showimage --ros-args -r image:=/overhead/topdown_image
```

Install viewer if missing:

```bash
sudo apt install ros-jazzy-image-tools
```

---

## 4. ROS topic checks

### List overhead topics

```bash
ros2 topic list | grep overhead
```

### Watch short status

```bash
ros2 topic echo /overhead/status
```

Good example:

```text
homography_active=True | raw_crates=3 | stable_crates=3 | robots=0
```

### Check image publish rate

```bash
ros2 topic hz /overhead/debug_image
ros2 topic hz /overhead/topdown_image
```

### Pretty-print full world state once

```bash
ros2 topic echo --once /overhead/world_state_json --field data | head -n 1 | python3 -m json.tool
```

### Show only stable crates

```bash
ros2 topic echo --once /overhead/world_state_json --field data | head -n 1 | python3 -c 'import sys,json; d=json.load(sys.stdin); print(json.dumps(d["stable_crates"], indent=2))'
```

### Show only robot / ninja markers

```bash
ros2 topic echo --once /overhead/world_state_json --field data | head -n 1 | python3 -c 'import sys,json; d=json.load(sys.stdin); print(json.dumps(d["robots"], indent=2))'
```

### Show crate positions only

```bash
ros2 topic echo --once /overhead/world_state_json --field data | head -n 1 | python3 -c 'import sys,json; d=json.load(sys.stdin); [print(c["track_id"], c["aruco_id"], c["crate_type"], c["x_mm"], c["y_mm"], c["long_axis_deg"]) for c in d["stable_crates"]]'
```

### Show main-bot approach poses only

```bash
ros2 topic echo --once /overhead/world_state_json --field data | head -n 1 | python3 -c 'import sys,json; d=json.load(sys.stdin); [print(c["track_id"], c["crate_type"], c.get("best_main_bot_approach_pose")) for c in d["stable_crates"]]'
```

---

## 5. Brio 4K camera checks

### List cameras

```bash
v4l2-ctl --list-devices
```

### Show all current Brio settings

```bash
v4l2-ctl -d /dev/video2 --all
```

### Show supported formats / FPS

```bash
v4l2-ctl -d /dev/video2 --list-formats-ext
```

### Confirm 4K MJPG 30 FPS

```bash
v4l2-ctl -d /dev/video2 --get-fmt-video
v4l2-ctl -d /dev/video2 --get-parm
```

---

## 6. Current working Brio values

These are the known working values from testing.

```bash
v4l2-ctl -d /dev/video2 --set-fmt-video=width=3840,height=2160,pixelformat=MJPG
v4l2-ctl -d /dev/video2 --set-parm=30

v4l2-ctl -d /dev/video2 -c auto_exposure=1
v4l2-ctl -d /dev/video2 -c exposure_time_absolute=180
v4l2-ctl -d /dev/video2 -c exposure_dynamic_framerate=0

v4l2-ctl -d /dev/video2 -c brightness=80
v4l2-ctl -d /dev/video2 -c contrast=125
v4l2-ctl -d /dev/video2 -c saturation=120

v4l2-ctl -d /dev/video2 -c white_balance_automatic=0
v4l2-ctl -d /dev/video2 -c white_balance_temperature=2800

v4l2-ctl -d /dev/video2 -c gain=0
v4l2-ctl -d /dev/video2 -c sharpness=240
v4l2-ctl -d /dev/video2 -c power_line_frequency=1
v4l2-ctl -d /dev/video2 -c backlight_compensation=1

v4l2-ctl -d /dev/video2 -c focus_automatic_continuous=0
v4l2-ctl -d /dev/video2 -c focus_absolute=4

v4l2-ctl -d /dev/video2 -c pan_absolute=0
v4l2-ctl -d /dev/video2 -c tilt_absolute=0
v4l2-ctl -d /dev/video2 -c zoom_absolute=100

v4l2-ctl -d /dev/video2 -c led1_mode=3
v4l2-ctl -d /dev/video2 -c led1_frequency=0
```

---

## 7. Fast camera tuning tests

Try these only when far tags are hard to detect.

### Focus

```bash
v4l2-ctl -d /dev/video2 -c focus_absolute=0
v4l2-ctl -d /dev/video2 -c focus_absolute=5
v4l2-ctl -d /dev/video2 -c focus_absolute=10
v4l2-ctl -d /dev/video2 -c focus_absolute=20
```

### Exposure

```bash
v4l2-ctl -d /dev/video2 -c exposure_time_absolute=130
v4l2-ctl -d /dev/video2 -c exposure_time_absolute=180
v4l2-ctl -d /dev/video2 -c exposure_time_absolute=220
v4l2-ctl -d /dev/video2 -c exposure_time_absolute=250
```

### Sharpness

```bash
v4l2-ctl -d /dev/video2 -c sharpness=128
v4l2-ctl -d /dev/video2 -c sharpness=160
v4l2-ctl -d /dev/video2 -c sharpness=200
```

---

## 8. Debug / recovery

### Check ROS distro

```bash
echo $ROS_DISTRO
```

Expected:

```text
jazzy
```

### Check where ROS installed the package

```bash
ros2 pkg prefix overhead_control
```

### Check OpenCV ArUco support

```bash
python3 - <<'PY'
import cv2
print("OpenCV:", cv2.__version__)
print("Has aruco:", hasattr(cv2, "aruco"))
print("Has ArucoDetector:", hasattr(cv2.aruco, "ArucoDetector") if hasattr(cv2, "aruco") else False)
PY
```

### Test camera with OpenCV

```bash
python3 - <<'PY'
import cv2
cap = cv2.VideoCapture("/dev/video2", cv2.CAP_V4L2)
print("opened:", cap.isOpened())
print("width:", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
print("height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
ret, frame = cap.read()
print("read frame:", ret, None if frame is None else frame.shape)
cap.release()
PY
```

### Kill stuck image viewer

```bash
pkill -f showimage
```

### Restart NoMachine

```bash
pkill -f camera_node
pkill -f crate_detector
pkill -f crate_align
pkill -f ninja_full_gui_camera
pkill -f ninja_camera_ros_vision

sudo /usr/NX/bin/nxserver --restart

sudo /etc/NX/nxserver --status
sudo systemctl restart nxserver.service
sudo systemctl status nxserver.service
```
