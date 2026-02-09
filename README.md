# cobot_ws

ROS 2 workspace for the Warebot mobile manipulator and ClearPath-related packages.

## Contents

- **warebot** — Warebot mobile manipulator (A200): URDF/xacro description, Gazebo simulation, ros2_control config, and launch files
- **clearpath_common** — ClearPath shared models and resources
- **clearpath_config** — ClearPath configuration packages

## Requirements

- ROS 2 (tested with colcon/ament)
- Dependencies: `xacro`, `robot_state_publisher`, `joint_state_publisher`, `rviz2`, `ros2_control`, `ros2_controllers`, Gazebo (Ignition)

## Build

```bash
cd cobot_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Source the workspace, then use the package launch files, for example:

```bash
source install/setup.bash
ros2 launch warebot display.launch.py   # RViz + robot state publisher
# or
ros2 launch warebot rsp.launch.py       # Robot state publisher only
```

## License

See individual packages for license information.
