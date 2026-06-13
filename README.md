# Eurobot 2026 MTP

Merged Eurobot 2026 ROS 2 workspace containing:

- `main_bot_control` — main robot control, physical robot launch pipeline, simulation bridge, localization, and telemetry.
- `main_bot_sim` — Gazebo simulation world, launch file, bridges, and simulation assets.
- `overhead_control` — physical overhead camera stack and simulation overhead rectifier/pose publisher.
- `ninja_bot_control` — Ninja/SIMA control stack.

## Build

```bash
cd Eurobot2026MTP
colcon build --symlink-install
source install/setup.bash
```

## Run simulation

```bash
./run_sim.sh
```

This builds the workspace and launches `main_bot_sim`, `main_bot_control`, and the simulated overhead pipeline in `overhead_control`.

## Run physical main bot

```bash
./run_bot.sh
```

This starts the physical ZED camera pipeline, ArUco localization, pose fusion, OpenCR bridge, actuator bridge, Raspberry Pi metrics publisher, and telemetry GUI through `main_bot_control`.


## Run overhead camera (laptop)

Apply camera settings, then launch:

```bash
./src/overhead_control/camera_settings/brio_4k_final_settings.sh
./scripts/bringup_overhead.sh side:=blue
```

Start the operator GUI in a second terminal:

```bash
./scripts/bringup_operator_gui.sh
```

## Run Ninja SIMA (Ninja Pi)

Drive stack with overhead navigation:

```bash
./scripts/bringup_ninja_overhead_drive.sh
```

Crate alignment in a second terminal:

```bash
ros2 launch ninja_bot_control ninja_crate_align_only.launch.py
```

Drive and crate alignment were validated separately. Both subsystems write to `/cmd_vel` and conflict when run together. Unifying them requires the same mode switching approach used in the main bot,
which was identified but not implemented within the project timeline.

## Network

Source the correct environment in every terminal before running ROS 2:

```bash
source network/ros_dlink_env.sh       # arena D-Link router
source network/ros_tailscale_env.sh   # remote via Tailscale
```


