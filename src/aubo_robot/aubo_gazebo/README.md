# AUBO Gazebo Support

This package provides the Gazebo-side assets for the AUBO E5 stack:

- Gazebo launch files
- E5 simulation xacro
- controller configuration for the single-joint position controllers

The actual Gazebo bridge node, `aubo_gazebo_driver`, is built in the `aubo_driver` package.

## Main Files

- `launch/aubo_e5_gazebo_control.launch`
- `config/aubo_e5_gazebo_control.yaml`
- `urdf/aubo_e5.xacro`

## Launch Modes

### Normal mode

```bash
roslaunch aubo_gazebo aubo_e5_gazebo_control.launch shadow:=false paused:=false
```

Behavior:

- starts Gazebo and spawns the E5 model
- loads six single-joint position controllers plus `joint_state_controller`
- starts `aubo_gazebo_driver` in normal mode
- `aubo_gazebo_driver` subscribes to `joint_path_command`

This is the Gazebo backend used by linked execution when `sim_only:=true`.

### Shadow mode

```bash
roslaunch aubo_gazebo aubo_e5_gazebo_control.launch shadow:=true paused:=false
```

Behavior:

- starts the same Gazebo scene and controllers
- starts `aubo_gazebo_driver` in shadow mode
- `aubo_gazebo_driver` subscribes to `/real/joint_states` by default
- Gazebo becomes a one-way mirror of the real robot

This is the Gazebo backend used by linked execution when `sim_only:=false`.

## Controller Layout

The current controller YAML loads:

- `joint_state_controller`
- `shoulder_joint_position_controller`
- `upperArm_joint_position_controller`
- `foreArm_joint_position_controller`
- `wrist1_joint_position_controller`
- `wrist2_joint_position_controller`
- `wrist3_joint_position_controller`

The controller names must stay aligned with `aubo_driver/src/aubo_gazebo_driver.cpp`,
which publishes directly to the matching command topics.

## Robot Description Source

The current MoveIt and Gazebo launch flow loads the E5 description from:

```text
aubo_gazebo/urdf/aubo_e5.xacro
```

This matters because the repository also contains related assets under `aubo_description/`,
but the active planning/simulation path in this workspace uses the Gazebo-side E5 xacro above.

## Dependencies

At runtime you will typically need ROS packages equivalent to:

- `gazebo_ros`
- `gazebo_ros_control`
- `controller_manager`
- `joint_state_controller`
- `position_controllers`
- `xacro`

Install the package set that matches your ROS distribution.
Older instructions that mention only `ros-kinetic-*` packages are not accurate for this workspace anymore.
