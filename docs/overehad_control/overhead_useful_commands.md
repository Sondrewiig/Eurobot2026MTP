# Overhead — commands and setup

## Build and source

```bash
cd ~/Eurobot2026MTP
source /opt/ros/jazzy/setup.bash
colcon build --packages-select overhead_control --symlink-install
source install/setup.bash
```

---

## Network

Source one of these in every terminal depending on setup:

```bash
source network/ros_dlink_env.sh       # arena D-Link router (192.168.0.x)
source network/ros_tailscale_env.sh   # remote via Tailscale
```

---

## Launch

Apply Brio 4K camera settings before launching:

```bash
./src/overhead_control/camera_settings/brio_4k_final_settings.sh
```

Blue side:

```bash
./scripts/bringup_overhead.sh side:=blue
```

Yellow side:

```bash
./scripts/bringup_overhead.sh side:=yellow
```

Operator GUI (separate terminal):

```bash
./scripts/bringup_operator_gui.sh
```

---

## Brio 4K camera

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video2 --get-fmt-video
v4l2-ctl -d /dev/video2 --get-parm
```

### Known working values

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
```

---

## ROS topic checks

```bash
ros2 topic list | grep overhead
ros2 topic echo /overhead/status
ros2 topic hz /overhead/debug_image
```

### Ninja position

```bash
ros2 topic echo --once /overhead/world_state_json --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); print(json.dumps(d["robots"]["ninja"], indent=2))'
```

### Main bot position

```bash
ros2 topic echo --once /overhead/world_state_json --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); print(json.dumps(d["robots"]["main_robot"], indent=2))'
```

### Full world state

```bash
ros2 topic echo --once /overhead/world_state_json --field data | head -n 1 | python3 -m json.tool
```

### Save snapshot to file

```bash
python3 scripts/overhead_pose_snapshot.py
```

---

## Restart NoMachine

```bash
sudo /usr/NX/bin/nxserver --restart
sudo systemctl status nxserver.service
```