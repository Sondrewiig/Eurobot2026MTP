# Merge notes

This repository is the cleaned merge of `Eurobot2026MTP` branch `Thanish` with the simulation packages from the simulation repository.

## Renamed packages

- previous control package -> `main_bot_control`
- previous simulation package -> `main_bot_sim`
- The simulation-only overhead rectifier from the former standalone simulation vision package has been moved into `overhead_control` as `overhead_rectifier_node`.

## Launchers

- `./run_sim.sh` builds the workspace and launches `main_bot_sim sim.launch.py`.
  - Gazebo and bridge launch from `main_bot_sim`.
  - Robot control, telemetry, localization, and simulated OpenCR launch from `main_bot_control`.
  - Simulated overhead pose publishing launches from `overhead_control`.

- `./run_bot.sh` keeps the physical main-bot pipeline and now uses `main_bot_control`.
  - ZED camera/splitter
  - ArUco detection
  - tag localization
  - physical Pose2D-based `pose_fuser`
  - OpenCR bridge
  - actuator bridge
  - Raspberry Pi metrics
  - telemetry console

## Pose fusers

- `pose_fuser` is kept as the physical Pose2D-based fuser used by `run_bot.sh`.
- `pose_fuser_sim` is the simulation odom+IMU fuser used by `main_bot_sim/launch/sim.launch.py`.

## Build

```bash
cd Eurobot2026MTP
colcon build --symlink-install
source install/setup.bash
```
