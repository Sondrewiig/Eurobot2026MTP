# Network setup

Two network configurations are used. Source the correct one in every terminal.

```bash
source network/ros_dlink_env.sh       # arena D-Link router 
source network/ros_tailscale_env.sh   # remote via Tailscale
```

---

## Fast DDS discovery server

The overhead laptop acts as the ROS2 discovery server. Start this first in a dedicated terminal and leave it running:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=26
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

fastdds discovery --server-id 0
```

Check laptop Tailscale IP:

```bash
tailscale status
```

---

## Quick test order

**1. Laptop** — start discovery server (terminal 1), then overhead (terminal 2):

```bash
./src/overhead_control/camera_settings/brio_4k_final_settings.sh
./scripts/bringup_overhead.sh side:=blue
```

**2. Ninja Pi** — confirm topics are visible:

```bash
source network/ros_tailscale_env.sh
ros2 daemon stop
ros2 topic list --no-daemon | grep overhead
```

**3. Ninja Pi** — confirm pose is arriving:

```bash
ros2 topic echo --no-daemon --once /ninja/pose
```

---

## Common fixes

Topic appears missing — stale daemon cache:

```bash
ros2 daemon stop
ros2 topic list --no-daemon
```

Check all ROS environment values are set:

```bash
echo $ROS_DOMAIN_ID          # 26
echo $RMW_IMPLEMENTATION     # rmw_fastrtps_cpp
echo $ROS_DISCOVERY_SERVER   # 100.101.185.35:11811
echo $ROS_SUPER_CLIENT       # TRUE
```

Check discovery server is running:

```bash
pgrep -a fastdds
```

Kill and restart if needed:

```bash
pkill -f "fastdds discovery"
fastdds discovery --server-id 0
```