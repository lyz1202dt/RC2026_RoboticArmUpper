# MuJoCo phase-1 simulation

This folder contains a minimal MuJoCo setup for phase-1 validation:
- Load the 6-DoF robot model.
- Move each joint to a fixed target pose.
- Verify basic kinematics and joint limits before ROS2 bridge integration.

## Prerequisites

```bash
pip install mujoco
```

For GUI visualization on Linux, make sure OpenGL/GLFW runtime is available.

## Run

From workspace root:

```bash
python3 src/robotic_arm/mujoco/run_fixed_pose.py
```

Headless mode:

```bash
python3 src/robotic_arm/mujoco/run_fixed_pose.py --no-viewer
```

## What this validates

- Model can be parsed and simulated in MuJoCo.
- Joint names and limits are consistent for `joint1` to `joint6`.
- Fixed target pose command path is working.

## Notes

- Mesh collisions are disabled intentionally in phase-1 to improve stability and reduce tuning complexity.
- Phase-2 should add ROS2 command/state bridge.
