# Eurobot 2026 Useful Terminal Commands

This file is split by machine:

1. **Overhead laptop first**
2. **Main Pi second**
3. **Ninja Pi third**

Current assumed setup:

```bash
ROS_DOMAIN_ID=26
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
LAPTOP_TS_IP=100.101.185.35
ROS_DISCOVERY_SERVER=100.101.185.35:11811
NINJA_ARUCO_ID=56
```

If the laptop Tailscale IP changes, replace `100.101.185.35` everywhere with the new laptop Tailscale IP.

---

# 1. Overhead Laptop Commands

The overhead laptop runs the camera node, publishes world state, and acts as the Fast DDS discovery server.

---

## 1.1 Check laptop Tailscale IP

```bash
tailscale ip -4
```

Shows the laptop Tailscale IP. Use this IP in `ROS_DISCOVERY_SERVER`.

---

## 1.2 Check Tailscale status

```bash
tailscale status
```

Shows whether the main Pi and ninja Pi are online/reachable through Tailscale.

---

## 1.3 Start Fast DDS discovery server

Run this in a dedicated laptop terminal and leave it open:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

fastdds discovery --server-id 0
```

This lets ROS2 nodes on the laptop and Pis discover each other over Tailscale without relying on multicast.

---

## 1.4 Setup overhead laptop ROS2 environment

Run this in every laptop terminal where you use ROS2:

```bash
cd ~/Eurobot2026MTP
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=100.101.185.35:11811
export ROS_SUPER_CLIENT=TRUE

ros2 daemon stop
```

The daemon reset avoids stale ROS2 graph information after changing discovery settings.

---

## 1.5 Build overhead package

```bash
cd ~/Eurobot2026MTP
source /opt/ros/jazzy/setup.bash

colcon build --packages-select overhead_control --symlink-install
source install/setup.bash
```

Builds the overhead camera package.

---

## 1.6 Clean rebuild overhead package

```bash
cd ~/Eurobot2026MTP
rm -rf build/overhead_control install/overhead_control log

source /opt/ros/jazzy/setup.bash
colcon build --packages-select overhead_control --symlink-install
source install/setup.bash
```

Use this if old code seems to still be running after a rebuild.

---

## 1.7 Launch overhead node for blue side with ninja ID 56

```bash
cd ~/Eurobot2026MTP
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=100.101.185.35:11811
export ROS_SUPER_CLIENT=TRUE

ros2 launch overhead_control overhead.launch.py side:=blue publish_images:=true \
ninja_aruco_id:=56
```

Starts the overhead camera node and selects ninja ArUco ID `56`.

---

## 1.8 Launch overhead with opponent detection disabled

```bash
ros2 launch overhead_control overhead.launch.py side:=blue publish_images:=true \
ninja_aruco_id:=56 \
detect_opponent_robots:=false
```

Best for testing your own main bot and ninja without opponent false positives.

---

## 1.9 Launch overhead with opponent debug enabled

```bash
ros2 launch overhead_control overhead.launch.py side:=blue publish_images:=true \
ninja_aruco_id:=56 \
detect_opponent_robots:=true \
opponent_robot_min_confidence_to_show:=999 \
draw_rejected_robot_detections:=true
```

Useful for debugging false opponent detections without letting them affect planning.

---

## 1.10 List overhead topics from laptop

```bash
ros2 topic list --no-daemon | grep overhead
```

Shows all overhead topics currently visible to the laptop.

---

## 1.11 Show overhead status

```bash
ros2 topic echo --no-daemon /overhead/status
```

Shows short status lines such as frame count, detected IDs, homography state, crate count, and robot count.

---

## 1.12 Pretty print full world state once

```bash
ros2 topic echo --no-daemon --once /overhead/world_state_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -m json.tool
```

Prints the full overhead JSON once in readable format.

---

## 1.13 Pretty print main bot compact command once

```bash
ros2 topic echo --no-daemon --once /overhead/main_bot_compact_command_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -m json.tool
```

Shows the compact command intended for the main bot.

---

## 1.14 Pretty print ninja compact command once

```bash
ros2 topic echo --no-daemon --once /overhead/ninja_compact_command_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -m json.tool
```

Shows the compact command intended for the ninja bot.

---

## 1.15 Monitor main bot compact command

```bash
watch -n 0.3 'ros2 topic echo --no-daemon --once /overhead/main_bot_compact_command_json std_msgs/msg/String --field data | head -n 1 | python3 -c '\''import sys,json,time; d=json.load(sys.stdin); print("active:", d["active"], "reason:", d["reason"]); print("cmd_seq:", d["command_seq"], "pub_seq:", d["publish_seq"], "age_ms:", round((time.time()-d["stamp"])*1000)); print("stage:", d["requested_stage"]); print("goal:", d["goal_pose"]); print("robot:", d["robot_pose"]); print("crate:", d["target_crate"])'\'''
```

Live monitor for the main bot command.

---

## 1.16 Monitor ninja compact command

```bash
watch -n 0.3 'ros2 topic echo --no-daemon --once /overhead/ninja_compact_command_json std_msgs/msg/String --field data | head -n 1 | python3 -c '\''import sys,json,time; d=json.load(sys.stdin); print("active:", d["active"], "reason:", d["reason"]); print("cmd_seq:", d["command_seq"], "pub_seq:", d["publish_seq"], "age_ms:", round((time.time()-d["stamp"])*1000)); print("stage:", d["requested_stage"]); print("goal:", d["goal_pose"]); print("robot:", d["robot_pose"]); print("crate:", d["target_crate"])'\'''
```

Live monitor for the ninja command.

---

## 1.17 View debug camera image

```bash
ros2 run image_tools showimage --ros-args -r image:=/overhead/debug_image
```

Shows the camera view with detected ArUco markers and debug overlays.

---

## 1.18 View top-down map image

```bash
ros2 run image_tools showimage --ros-args -r image:=/overhead/topdown_image
```

Shows the generated top-down arena map with crates, robots, targets, and paths.

---

## 1.19 Show detected ArUco IDs

```bash
ros2 topic echo --no-daemon --once /overhead/detected_ids_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -m json.tool
```

Shows the raw detected marker IDs.

---

## 1.20 Show robot section only

```bash
ros2 topic echo --no-daemon --once /overhead/world_state_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); print(json.dumps(d["robots"], indent=2))'
```

Prints only the robot/ninja tracking part of the world state.

---

## 1.21 Show main robot only

```bash
ros2 topic echo --no-daemon --once /overhead/world_state_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); print(json.dumps(d["robots"]["main_robot"], indent=2))'
```

Prints the selected main robot pose.

---

## 1.22 Show ninja robot only

```bash
ros2 topic echo --no-daemon --once /overhead/world_state_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); print(json.dumps(d["robots"]["ninja"], indent=2))'
```

Prints the selected ninja pose.

---

## 1.23 Show stable crates

```bash
ros2 topic echo --no-daemon --once /overhead/world_state_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); [print("track", c["track_id"], "id", c["aruco_id"], c["crate_type"], "x", round(c["x_mm"],1), "y", round(c["y_mm"],1), "angle", round(c["long_axis_deg"],1)) for c in d["stable_crates"]]'
```

Shows each stable crate with track ID, type, position, and angle.

---

## 1.24 Show main bot target queue

```bash
ros2 topic echo --no-daemon --once /overhead/main_bot_target_queue_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -c 'import sys,json; q=json.load(sys.stdin); [print(i["rank"], "track", i["target_crate"]["track_id"], i["target_crate"]["crate_type"], "dist", i["distance_to_approach_mm"], i["approach_pose"]["name"]) for i in q]'
```

Shows the ranked target queue for the main bot.

---

## 1.25 Show ninja target queue

```bash
ros2 topic echo --no-daemon --once /overhead/ninja_target_queue_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -c 'import sys,json; q=json.load(sys.stdin); [print(i["rank"], "track", i["target_crate"]["track_id"], i["target_crate"]["crate_type"], "dist", i["distance_to_approach_mm"], i["approach_pose"]["name"]) for i in q]'
```

Shows the ranked target queue for the ninja bot.

---

## 1.26 Check NoMachine server status

```bash
sudo systemctl status nxserver
```

Shows whether NoMachine is running on the laptop.

---

## 1.27 Restart NoMachine server

```bash
sudo systemctl restart nxserver
```

Use this if remote desktop connection gets stuck.

---

# 2. Main Pi Commands

The main Pi subscribes to the main bot compact command and later sends low-level commands to the ESP over USB serial.

---

## 2.1 Check main Pi Tailscale IP

```bash
tailscale ip -4
```

Shows the main Pi Tailscale IP.

---

## 2.2 Check Tailscale status

```bash
tailscale status
```

Shows whether the overhead laptop and ninja Pi are reachable.

---

## 2.3 Ping overhead laptop through Tailscale

```bash
tailscale ping 100.101.185.35
```

Confirms Tailscale connectivity to the overhead laptop.

---

## 2.4 Normal ping to overhead laptop

```bash
ping -c 3 100.101.185.35
```

Confirms normal IP connectivity to the overhead laptop.

---

## 2.5 Setup main Pi ROS2 environment

Run this in every main Pi terminal where you use ROS2:

```bash
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=100.101.185.35:11811
export ROS_SUPER_CLIENT=TRUE

ros2 daemon stop
```

Connects the main Pi ROS2 CLI/nodes to the laptop discovery server.

---

## 2.6 List overhead topics visible from main Pi

```bash
ros2 topic list --no-daemon | grep overhead
```

Confirms the main Pi can see the overhead laptop topics.

---

## 2.7 Echo main bot compact command

```bash
ros2 topic echo --no-daemon /overhead/main_bot_compact_command_json std_msgs/msg/String
```

Prints the raw compact command stream for the main bot.

---

## 2.8 Pretty print main bot compact command once

```bash
ros2 topic echo --no-daemon --once /overhead/main_bot_compact_command_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -m json.tool
```

Readable one-shot test of the main bot command.

---

## 2.9 Main Pi command summary monitor

```bash
watch -n 0.3 'ros2 topic echo --no-daemon --once /overhead/main_bot_compact_command_json std_msgs/msg/String --field data | head -n 1 | python3 -c '\''import sys,json,time; d=json.load(sys.stdin); print("active:", d["active"], "reason:", d["reason"]); print("cmd_seq:", d["command_seq"], "pub_seq:", d["publish_seq"], "age_ms:", round((time.time()-d["stamp"])*1000)); print("stage:", d["requested_stage"]); print("goal:", d["goal_pose"]); print("crate:", d["target_crate"])'\'''
```

Useful live command monitor on the main Pi.

---

## 2.10 Check USB serial devices

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Shows connected ESP serial devices.

---

## 2.11 Check detailed USB devices

```bash
lsusb
```

Shows USB hardware connected to the Pi.

---

## 2.12 Check serial permissions

```bash
groups
```

Check whether the user is in the `dialout` group for serial access.

---

## 2.13 Add user to dialout group

```bash
sudo usermod -a -G dialout $USER
```

Allows serial access to `/dev/ttyUSB*` or `/dev/ttyACM*`. Log out and back in after running this.

---

## 2.14 Quick serial monitor test

Replace `/dev/ttyACM0` if your ESP appears as a different device:

```bash
python3 -m serial.tools.miniterm /dev/ttyACM0 115200
```

Opens a simple serial terminal to the ESP.

---

## 2.15 Install Python serial library

```bash
sudo apt update
sudo apt install python3-serial
```

Installs Python serial support for future Pi-to-ESP bridge node.

---

## 2.16 Start tmux session for main Pi

```bash
tmux new -s main
```

Starts a persistent terminal session. Good for running robot code without losing it if NoMachine disconnects.

---

## 2.17 Detach from tmux

Press:

```text
Ctrl+b then d
```

Leaves the tmux session running in the background.

---

## 2.18 Reattach to tmux

```bash
tmux attach -t main
```

Returns to the main Pi tmux session.

---

## 2.19 Check NoMachine server status

```bash
sudo systemctl status nxserver
```

Shows whether NoMachine is running on the main Pi.

---

## 2.20 Restart NoMachine server

```bash
sudo systemctl restart nxserver
```

Use if the NoMachine session is stuck.

---

# 3. Ninja Pi Commands

The ninja Pi subscribes to the ninja compact command and later sends low-level commands to its ESP over USB serial.

Current ninja ArUco ID:

```text
56
```

---

## 3.1 Check ninja Pi Tailscale IP

```bash
tailscale ip -4
```

Shows the ninja Pi Tailscale IP.

---

## 3.2 Check Tailscale status

```bash
tailscale status
```

Shows whether the overhead laptop and main Pi are reachable.

---

## 3.3 Ping overhead laptop through Tailscale

```bash
tailscale ping 100.101.185.35
```

Confirms Tailscale connectivity to the overhead laptop.

---

## 3.4 Normal ping to overhead laptop

```bash
ping -c 3 100.101.185.35
```

Confirms normal IP connectivity to the overhead laptop.

---

## 3.5 Setup ninja Pi ROS2 environment

Run this in every ninja Pi terminal where you use ROS2:

```bash
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=100.101.185.35:11811
export ROS_SUPER_CLIENT=TRUE

ros2 daemon stop
```

Connects the ninja Pi ROS2 CLI/nodes to the laptop discovery server.

---

## 3.6 List overhead topics visible from ninja Pi

```bash
ros2 topic list --no-daemon | grep overhead
```

Confirms the ninja Pi can see the overhead laptop topics.

---

## 3.7 Echo ninja compact command

```bash
ros2 topic echo --no-daemon /overhead/ninja_compact_command_json std_msgs/msg/String
```

Prints the raw compact command stream for the ninja bot.

---

## 3.8 Pretty print ninja compact command once

```bash
ros2 topic echo --no-daemon --once /overhead/ninja_compact_command_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -m json.tool
```

Readable one-shot test of the ninja command.

---

## 3.9 Ninja Pi command summary monitor

```bash
watch -n 0.3 'ros2 topic echo --no-daemon --once /overhead/ninja_compact_command_json std_msgs/msg/String --field data | head -n 1 | python3 -c '\''import sys,json,time; d=json.load(sys.stdin); print("active:", d["active"], "reason:", d["reason"]); print("cmd_seq:", d["command_seq"], "pub_seq:", d["publish_seq"], "age_ms:", round((time.time()-d["stamp"])*1000)); print("stage:", d["requested_stage"]); print("goal:", d["goal_pose"]); print("crate:", d["target_crate"])'\'''
```

Useful live command monitor on the ninja Pi.

---

## 3.10 Show only ninja robot pose from world state

```bash
ros2 topic echo --no-daemon --once /overhead/world_state_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); print(json.dumps(d["robots"]["ninja"], indent=2))'
```

Confirms the overhead laptop is detecting the selected ninja.

---

## 3.11 Show ninja target queue

```bash
ros2 topic echo --no-daemon --once /overhead/ninja_target_queue_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -c 'import sys,json; q=json.load(sys.stdin); [print(i["rank"], "track", i["target_crate"]["track_id"], i["target_crate"]["crate_type"], "dist", i["distance_to_approach_mm"], i["approach_pose"]["name"]) for i in q]'
```

Shows the ranked target queue for the ninja.

---

## 3.12 Check USB serial devices

```bash
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Shows connected ESP serial devices.

---

## 3.13 Check detailed USB devices

```bash
lsusb
```

Shows USB hardware connected to the ninja Pi.

---

## 3.14 Check serial permissions

```bash
groups
```

Check whether the user is in the `dialout` group for serial access.

---

## 3.15 Add user to dialout group

```bash
sudo usermod -a -G dialout $USER
```

Allows serial access to `/dev/ttyUSB*` or `/dev/ttyACM*`. Log out and back in after running this.

---

## 3.16 Quick serial monitor test

Replace `/dev/ttyACM0` if your ESP appears as a different device:

```bash
python3 -m serial.tools.miniterm /dev/ttyACM0 115200
```

Opens a simple serial terminal to the ninja ESP.

---

## 3.17 Install Python serial library

```bash
sudo apt update
sudo apt install python3-serial
```

Installs Python serial support for future Pi-to-ESP bridge node.

---

## 3.18 Start tmux session for ninja Pi

```bash
tmux new -s ninja
```

Starts a persistent terminal session. Good for running robot code without losing it if NoMachine disconnects.

---

## 3.19 Detach from tmux

Press:

```text
Ctrl+b then d
```

Leaves the tmux session running in the background.

---

## 3.20 Reattach to tmux

```bash
tmux attach -t ninja
```

Returns to the ninja Pi tmux session.

---

## 3.21 Check NoMachine server status

```bash
sudo systemctl status nxserver
```

Shows whether NoMachine is running on the ninja Pi.

---

## 3.22 Restart NoMachine server

```bash
sudo systemctl restart nxserver
```

Use if the NoMachine session is stuck.

---

# 4. Quick Test Order

Use this order when testing communication.

---

## 4.1 Laptop terminal 1

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

fastdds discovery --server-id 0
```

Start discovery server.

---

## 4.2 Laptop terminal 2

```bash
cd ~/Eurobot2026MTP
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=100.101.185.35:11811
export ROS_SUPER_CLIENT=TRUE

ros2 launch overhead_control overhead.launch.py side:=blue publish_images:=true \
ninja_aruco_id:=56
```

Start overhead camera node.

---

## 4.3 Ninja Pi terminal

```bash
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=100.101.185.35:11811
export ROS_SUPER_CLIENT=TRUE

ros2 daemon stop
ros2 topic list --no-daemon | grep overhead
ros2 topic echo --no-daemon --once /overhead/ninja_compact_command_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -m json.tool
```

Check ninja can receive laptop commands.

---

## 4.4 Main Pi terminal

```bash
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=100.101.185.35:11811
export ROS_SUPER_CLIENT=TRUE

ros2 daemon stop
ros2 topic list --no-daemon | grep overhead
ros2 topic echo --no-daemon --once /overhead/main_bot_compact_command_json std_msgs/msg/String --field data | \
head -n 1 | \
python3 -m json.tool
```

Check main Pi can receive laptop commands.

---

# 5. Common Fixes

---

## 5.1 Wrong discovery server format

Wrong:

```bash
export ROS_DISCOVERY_SERVER=100.101.185.35.11811
```

Correct:

```bash
export ROS_DISCOVERY_SERVER=100.101.185.35:11811
```

Use a colon before the port.

---

## 5.2 Topic appears missing

Try:

```bash
ros2 daemon stop
ros2 topic list --no-daemon | grep overhead
```

The normal ROS2 daemon can cache stale discovery information.

---

## 5.3 Echo says topic type unknown

Use the type explicitly:

```bash
ros2 topic echo --no-daemon /overhead/ninja_compact_command_json std_msgs/msg/String
```

or:

```bash
ros2 topic echo --no-daemon /overhead/main_bot_compact_command_json std_msgs/msg/String
```

---

## 5.4 Check all ROS environment values

```bash
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
echo $ROS_DISCOVERY_SERVER
echo $ROS_SUPER_CLIENT
```

Expected:

```text
26
rmw_fastrtps_cpp
100.101.185.35:11811
TRUE
```

---

## 5.5 Check if discovery server is still running

```bash
pgrep -a fastdds
```

Shows the Fast DDS discovery server process if it is running.

---

## 5.6 Kill discovery server if needed

```bash
pkill -f "fastdds discovery"
```

Stops the discovery server process.

---

## 5.7 Restart ROS2 daemon

```bash
ros2 daemon stop
ros2 daemon start
```

Restarts the ROS2 CLI graph helper.

---

## 5.8 Use tmux for safe long-running commands

```bash
tmux new -s eurobot
```

Use tmux when running robot commands over NoMachine or SSH. The process keeps running even if the remote desktop disconnects.

---

# 6. Notes

- `ROS_DOMAIN_ID=26` is the ROS2 group/channel. It must match on laptop, main Pi, and ninja Pi.
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` tells ROS2 to use Fast DDS.
- `ROS_DISCOVERY_SERVER=100.101.185.35:11811` tells ROS2 to use the overhead laptop as the discovery server.
- `ROS_SUPER_CLIENT=TRUE` helps ROS2 CLI tools see all topics when using Fast DDS Discovery Server.
- NoMachine is only for remote desktop.
- Tailscale is the network path.
- ROS2 topics are the laptop-to-Pi command path.
- USB serial is the Pi-to-ESP command path.
- Do not connect motor movement until the Pi can reliably print compact commands first.
