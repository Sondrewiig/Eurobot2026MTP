# Ninja bot — commands and setup

## Build and source

```bash
cd ~/Eurobot2026MTP
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ninja_bot_control
source install/setup.bash
```

---

## Launch

### Full match stack — drive + camera + crate align (Ninja Pi)

Drive and crate alignment were validated separately.Drive and crate alignment were validated separately. 
Both subsystems write to /cmd_vel and conflict when run together. Unifying them requires the same mode-switching 
approach used in the main bot, which was identified but not implemented within the project timeline.

```bash
./scripts/bringup_ninja.sh
```

### Drive only — overhead navigation, no onboard camera (Ninja Pi)

```bash
./scripts/bringup_ninja_overhead_drive.sh
```

### Crate alignment — camera + align, no go_to_point (Ninja Pi)

Run in a second terminal after `bringup_ninja_overhead_drive.sh`.

```bash
ros2 launch ninja_bot_control ninja_crate_align_only.launch.py
```

### Arm / disarm drive

```bash
ros2 topic pub --once /ninja/enable_drive std_msgs/Bool "{data: true}"
ros2 topic pub --once /ninja/enable_drive std_msgs/Bool "{data: false}"
```

---

## ESP32 commands

### Motors
```
forward    backward    left    right    stop
m1 <-255 to 255>
m2 <-255 to 255>
motors <left> <right>
```

### Motor tuning
```
trim <left_trim> <right_trim>
minpwm <0-255>    maxpwm <0-255>    ramp <0-255>
watchdog on/off    timeout <50-5000>
testmotor <motor> <start> <end> <step>
```

### Gripper and tilt
```
tiltup    tiltdown    tilt <0-180>
grip <0-180>    twocrates    onecrate    release
startposition    neutralposition    eating    stopeating
```

### Sensors
```
vlx             toggles VLX distance sensor stream
vlxstatus
voltage
voltagestream   toggles voltage stream
```

---

## ROS checks

```bash
ros2 topic hz /ninja/pose
ros2 topic echo /ninja/telemetry
ros2 topic pub --once /ninja/esp32_cmd std_msgs/msg/String "{data: 'ping'}"
```