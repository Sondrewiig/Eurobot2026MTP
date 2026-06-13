# Eurobot Overhead — Terminal Layout

---

## Terminal 1 — Start overhead node

```bash
cd ~/Eurobot2026MTP
source /opt/ros/jazzy/setup.bash
source install/setup.bash

./src/overhead_control/camera_settings/brio_4k_manual.sh
ros2 launch overhead_control overhead.launch.py side:=blue publish_images:=true \
main_robot_aruco_id:=1 \
filter_to_configured_main_robot_id:=true \
ninja_aruco_id:=57
```

For yellow side, replace `side:=blue` with:

```bash
side:=yellow
```

---

## Terminal 2 — Camera debug image

```bash
cd ~/Eurobot2026MTP
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run image_tools showimage --ros-args -r image:=/overhead/debug_image
```

---

## Terminal 3 — Top-down image

```bash
cd ~/Eurobot2026MTP
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run image_tools showimage --ros-args -r image:=/overhead/topdown_image
```

---

## Terminal 4 — Status / quick JSON checks

First prepare the terminal:

```bash
cd ~/Eurobot2026MTP
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### Live status

```bash
ros2 topic echo /overhead/status
```

### List overhead topics

```bash
ros2 topic list | grep overhead
```

---

## Robot / ninja position checks

### Main bot coordinates

```bash
ros2 topic echo --once /overhead/world_state_json --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); print(json.dumps(d["robots"]["main_robot"], indent=2))'
```

### Ninja coordinates

```bash
ros2 topic echo --once /overhead/world_state_json --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); print(json.dumps(d["robots"]["ninja"], indent=2))'
```

### All robots / ninja

```bash
ros2 topic echo --once /overhead/world_state_json --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); print(json.dumps(d["robots"], indent=2))'
```

---

## Crate checks

### All stable crates

```bash
ros2 topic echo --once /overhead/world_state_json --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); [print("track", c["track_id"], "id", c["aruco_id"], c["crate_type"], "x", round(c["x_mm"],1), "y", round(c["y_mm"],1), "angle", round(c["long_axis_deg"],1)) for c in d["stable_crates"]]'
```

### Crate approach candidates and validity reasons

```bash
ros2 topic echo --once /overhead/world_state_json --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); 
for c in d["stable_crates"]:
 print("\ntrack", c["track_id"], c["crate_type"], "x", c["x_mm"], "y", c["y_mm"]);
 for p in c.get("approach_pose_candidates", []):
  print(" ", p["name"], "main_valid=", p.get("valid_for_main_bot"), "reason=", p.get("main_bot_reason"), "x", p["x_mm"], "y", p["y_mm"])'
```

---

## Main bot target / movement command

### Full main bot target

```bash
ros2 topic echo --once /overhead/main_bot_target_json --field data | \
head -n 1 | \
python3 -m json.tool
```

### Full main bot staged navigation command

```bash
ros2 topic echo --once /overhead/main_bot_nav_command_json --field data | \
head -n 1 | \
python3 -m json.tool
```

### Compact main bot staged navigation command

```bash
ros2 topic echo --once /overhead/main_bot_nav_command_json --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); print("active:", d["active"], "reason:", d["reason"]); print("pre:", d["pre_align_pose"]); print("approach:", d["approach_pose"]); print("crate:", d["target_crate"])'
```

### Main bot target queue

```bash
ros2 topic echo --once /overhead/main_bot_target_queue_json --field data | \
head -n 1 | \
python3 -c 'import sys,json; q=json.load(sys.stdin); [print(i["rank"], "track", i["target_crate"]["track_id"], i["target_crate"]["crate_type"], "dist", i["distance_to_approach_mm"], i["approach_pose"]["name"]) for i in q]'
```

### Live watch main bot staged command

```bash
watch -n 0.5 'ros2 topic echo --once /overhead/main_bot_nav_command_json --field data | head -n 1 | python3 -c '"'"'import sys,json; d=json.load(sys.stdin); print("active:", d["active"], "reason:", d["reason"]); print("pre:", d["pre_align_pose"]); print("approach:", d["approach_pose"])'"'"''
```

---

## Ninja target / movement command

### Full ninja target

```bash
ros2 topic echo --once /overhead/ninja_target_json --field data | \
head -n 1 | \
python3 -m json.tool
```

### Full ninja staged navigation command

```bash
ros2 topic echo --once /overhead/ninja_nav_command_json --field data | \
head -n 1 | \
python3 -m json.tool
```

### Compact ninja staged navigation command

```bash
ros2 topic echo --once /overhead/ninja_nav_command_json --field data | \
head -n 1 | \
python3 -c 'import sys,json; d=json.load(sys.stdin); print("active:", d["active"], "reason:", d["reason"]); print("pre:", d["pre_align_pose"]); print("approach:", d["approach_pose"]); print("crate:", d["target_crate"])'
```

### Ninja target queue

```bash
ros2 topic echo --once /overhead/ninja_target_queue_json --field data | \
head -n 1 | \
python3 -c 'import sys,json; q=json.load(sys.stdin); [print(i["rank"], "track", i["target_crate"]["track_id"], i["target_crate"]["crate_type"], "dist", i["distance_to_approach_mm"], i["approach_pose"]["name"]) for i in q]'
```

---

## Full world state

### Pretty-print full world state once

```bash
ros2 topic echo --once /overhead/world_state_json --field data | \
head -n 1 | \
python3 -m json.tool
```

### Save one world-state snapshot to file

```bash
ros2 topic echo --once /overhead/world_state_json --field data | \
head -n 1 | \
python3 -m json.tool > overhead_world_state_snapshot.json
```
