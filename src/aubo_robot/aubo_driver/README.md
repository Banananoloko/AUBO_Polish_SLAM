# AUBO Driver

This package contains two runtime components:

- `aubo_driver`: the real robot driver
- `aubo_gazebo_driver`: the Gazebo command / mirror bridge

The package is used by the current AUBO E5 stack in both pure-simulation mode and real-robot linked-execution mode.

## Nodes

### `aubo_driver`

Main entry: `src/driver_node.cpp`

Responsibilities:

- connect to the AUBO controller using the bundled AUBO controller library
- publish robot state, feedback, status, and IO state
- receive interpolated trajectory points from `aubo_robot_simulator`
- switch between teach-pendant control and ROS control
- provide IO / FK / IK services

Important characteristics from the current code:

- update loop: `500 Hz`
- async spinner threads: `6`
- maximum supported axis count in the driver class: `8`

Published topics:

- `joint_states`
- `feedback_states`
- `/aubo_driver/real_pose`
- `robot_status`
- `/aubo_driver/io_states`
- `/aubo_driver/rib_status`
- `aubo_driver/cancel_trajectory`

Subscribed topics:

- `trajectory_execution_event`
- `robot_control`
- `moveItController_cmd`
- `teach_cmd`
- `moveAPI_cmd`
- `/aubo_driver/controller_switch`

Services:

- `/aubo_driver/set_io`
- `/aubo_driver/get_ik`
- `/aubo_driver/get_fk`

### `aubo_gazebo_driver`

Main entry: `src/aubo_gazebo_driver.cpp`

This node drives Gazebo in two modes.

Normal mode:

- subscribes to `joint_path_command`
- maps trajectory joint names to the six E5 joints
- publishes each point to the six single-joint position controllers in Gazebo

Shadow mode:

- subscribes to `~real_joint_states_topic`, default `/real/joint_states`
- mirrors real robot joint values into Gazebo
- includes time-discontinuity protection for Gazebo pause/reset events

Gazebo command topics owned by this node:

- `/aubo_e5/shoulder_joint_position_controller/command`
- `/aubo_e5/upperArm_joint_position_controller/command`
- `/aubo_e5/foreArm_joint_position_controller/command`
- `/aubo_e5/wrist1_joint_position_controller/command`
- `/aubo_e5/wrist2_joint_position_controller/command`
- `/aubo_e5/wrist3_joint_position_controller/command`

## Launch Files

### Real driver only

`launch/aubo_driver_real.launch`

This launch file starts `aubo_driver` inside the `/real` namespace and sets `/aubo_driver/server_host`.
It is mainly useful when you want a minimal real-driver process that publishes `/real/joint_states` for Gazebo shadow mirroring.

Example:

```bash
roslaunch aubo_driver aubo_driver_real.launch robot_ip:=192.168.1.10
```

### Recommended system-level launch

For day-to-day use, prefer the workspace-level launch:

```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.1.10 sim_only:=false
```

That launch also starts:

- `aubo_joint_trajectory_action`
- `aubo_robot_simulator`
- Gazebo in shadow mode
- `joint_state_mirror_adapter`
- linked execution monitor / aggregator nodes

## Build Requirements

This package is not a pure source-only ROS driver.

The real driver executable depends on bundled binary artifacts:

- `lib/lib64/aubocontroller/libauborobotcontroller.so`
- local protobuf runtime in `../opt/lib`
- additional bundled libs under `lib/lib64`

If `libauborobotcontroller.so` is missing, CMake will skip building `aubo_driver`, `testIO`, and `testAuboAPI`,
but it will still build `aubo_gazebo_driver`.

## Control Switching

The driver listens on `/aubo_driver/controller_switch`:

- `0`: robot-controller / teach-pendant side
- `1`: ROS controller side

The workspace startup helper `aubo_linked_execution/scripts/aubo_robot_startup.py` switches this automatically in linked mode.

## Notes

- The driver writes CSV logs to hard-coded paths under `/home/aubo-fy/aubo_ws/` in the current implementation.
- Topic and service names in older online documentation often do not match the current codebase.
  Use this README and the source files as the reference for the present workspace.
