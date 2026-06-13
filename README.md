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
