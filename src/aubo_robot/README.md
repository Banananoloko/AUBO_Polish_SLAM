# AUBO E5 ROS Stack

This directory contains the AUBO-specific ROS packages used by the current `aubo_polish` workspace.
It is centered on the AUBO E5 robot and supports three related workflows:

- MoveIt planning in RViz
- Gazebo simulation and visualization
- Real robot execution with Gazebo shadow mirroring

## Main Packages

```text
aubo_robot/
├── aubo_controller/        # FollowJointTrajectory action server and interpolation bridge
├── aubo_description/       # Meshes and auxiliary robot-description assets
├── aubo_driver/            # Real robot driver and aubo_gazebo_driver
├── aubo_e5_moveit_config/  # MoveIt SRDF, kinematics, controller-manager launch files
├── aubo_gazebo/            # Gazebo world, E5 xacro, and joint controller config
├── aubo_kinematics/        # Analytic kinematics plugins for older AUBO models
├── aubo_msgs/              # AUBO custom messages and services
├── PointCLoud_SLAM/        # Point-cloud processing utilities
└── aubo_demo/              # Historical MoveIt demo programs
```

## Current Control Paths

### Recommended: linked execution

The workspace-level recommended entry point is:

```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch sim_only:=true
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.1.10 sim_only:=false
```

This path uses:

- `aubo_joint_trajectory_action` as the ROS-Industrial style action server
- `aubo_robot_simulator` as the interpolation / bridge node
- `aubo_driver` for the real robot
- `aubo_gazebo_driver` in normal mode for pure simulation, or in shadow mode for real-robot mirroring
- `aubo_linked_execution` for aggregated success/failure handling

### Legacy paths

The repository still contains older launch files:

- `aubo_e5_moveit_config/launch/moveit_planning_execution.launch`
- `aubo_e5_moveit_config/launch/demo_gazebo.launch`

These are still useful for debugging specific subsystems, but they are no longer the clearest end-to-end entry points.

## Build Notes

Build from the workspace root:

```bash
cd /home/wuqz/aubo_polish
catkin_make
```

Package-specific notes:

- `aubo_driver` links against bundled binary libraries in `aubo_driver/lib/lib64` and local protobuf libraries in `../opt/lib`.
- `aubo_controller` depends on MoveIt and vendored `industrial_core`.
- `aubo_e5_moveit_config` loads the current E5 robot description from `aubo_gazebo/urdf/aubo_e5.xacro`.

## Important Details

- The `aubo_joint_trajectory_action` action name is derived from `/robot_name`.
  For E5, the action server is `aubo_e5_controller/follow_joint_trajectory`.
- The MoveIt controller-manager layer can switch between:
  - `aubo_e5`: direct controller list from `config/controllers.yaml`
  - `linked_execution`: controller list from `aubo_linked_execution/config/linked_execution_controllers.yaml`
- In the current linked-execution setup, the real robot is the only executor.
  Gazebo is a synchronized visualization target, not a second independent controller.

## Which Files Matter Most

If you need to understand the current implementation quickly, start here:

- `aubo_driver/src/aubo_driver.cpp`
- `aubo_driver/src/aubo_gazebo_driver.cpp`
- `aubo_controller/src/joint_trajectory_action.cpp`
- `aubo_controller/script/aubo_controller/aubo_robot_simulator`
- `aubo_e5_moveit_config/launch/move_group.launch`
- `aubo_gazebo/launch/aubo_e5_gazebo_control.launch`
- `../aubo_linked_execution/launch/aubo_e5_linked_execution.launch`

## Documentation

- `AUBO_E5_Structure_Guide.md`: updated structure/dependency guide for the E5 stack
- `aubo_driver/README.md`: real driver and bridge details
- `aubo_gazebo/README.md`: Gazebo launch and controller topology
- `aubo_description/README.md`: status of robot description assets
