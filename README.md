# arm_grasp_pipeline

Perception-driven pick & place pipeline for a UR5 robot arm.

## Overview

This project implements a full grasp pipeline in ROS2 Jazzy with two trajectory execution approaches:

| Branch | Approach |
|---|---|
| `baseline` | Vanilla MoveIt2 with default joint trajectory controller |
| `main` | Custom quintic polynomial trajectory generator + PD + gravity compensation controller |

See `analysis/` for a quantitative comparison of jerk, velocity profiles, and tracking error between both approaches.

## Stack

- ROS2 Jazzy
- MoveIt2
- Gazebo Harmonic
- PCL (Point Cloud Library)
- ros2_control
- C++17
- Docker
- GitHub Actions CI

## Project structure

```
src/
  trajectory_generator/     # Quintic polynomial trajectory profiler (C++)
  grasp_controller/         # Custom ros2_control PD + gravity comp plugin (C++)
  perception_pipeline/      # PCL-based object detection (C++)
  grasp_action_server/      # ROS2 action server tying all components (C++)
analysis/
  compare_trajectories.py   # Reads ROS2 bags, generates comparison plots
  plots/                    # Pre-generated comparison figures
docker/
  Dockerfile
.github/
  workflows/
    ci.yml                  # Build + test on every push
```

## Running

```bash
# Build
colcon build --symlink-install

# Launch simulation (baseline)
ros2 launch grasp_action_server sim_baseline.launch.py

# Launch simulation (custom controller)
ros2 launch grasp_action_server sim_custom.launch.py
```