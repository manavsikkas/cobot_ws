# cobot_ws

ROS2 workspace for the Warebot mobile robot — a differential-drive platform based on the ClearPath A200 chassis. Includes URDF/Xacro descriptions, Gazebo Ignition simulation, ros2_control configuration, and launch files for visualisation and control.

## Platform

The Warebot is configured as a ClearPath A200 with:
- **Drivetrain** — 4-wheel differential drive, wheel separation 0.555 m, radius 0.165 m
- **Attachments** — front/rear bumpers, top plate (modular via Xacro)
- **Controllers** — `diff_drive_controller` + `joint_state_broadcaster` via ros2_control at 50 Hz

## Packages

| Package | Description |
|---|---|
| `warebot` | Main robot package — URDF/Xacro, launch files, ros2_control config |
| `clearpath_common` | Shared ClearPath model resources |
| `clearpath_config` | Platform-level configuration (`robot.yaml`) |

## Requirements

- ROS2 (Jazzy or compatible, tested with colcon/ament)
- `xacro`, `robot_state_publisher`, `joint_state_publisher`, `rviz2`
- `ros2_control`, `ros2_controllers`, `diff_drive_controller`
- Gazebo Ignition

## Build

```bash
cd cobot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
# Full visualisation — Gazebo Ignition + RViz2 + robot state publisher
ros2 launch warebot display.launch.py

# Robot state publisher only (for use with external simulation)
ros2 launch warebot rsp.launch.py
```

## Configuration

Robot platform parameters are in `src/warebot/config/robot.yaml`. Controller gains and joint names are in `config/ros2_control.yaml`.

## License

See individual packages for license information.
