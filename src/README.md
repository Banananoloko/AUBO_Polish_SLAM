# aubo_polish Workspace

This directory is the `src/` tree of a catkin workspace for an AUBO E5 polishing and visualization stack.
It combines the AUBO ROS packages, vendored ROS-Industrial dependencies, a linked real-robot/Gazebo execution layer,
and optional 3D camera / point-cloud modules.

## Workspace Layout

```text
src/
├── aubo_robot/              # AUBO E5 description, driver, MoveIt config, Gazebo support
├── aubo_linked_execution/   # Recommended unified execution entrypoint
├── aubo_unity_bridge/       # Unity3D simulation backend (NEW)
├── industrial_core/         # Vendored ROS-Industrial dependencies
├── mecheye_ros_interface/   # Mech-Eye ROS 1 interface
├── aubo_sdk_demo/           # Demo linked against external AUBO SDK
├── aubo_legacy_demo/        # Demo linked against bundled legacy controller libs
└── README.md
```

## Recommended Entry Points

There are multiple execution paths in the repository. The recommended one is the linked execution flow with support for both Gazebo and Unity backends:

```text
MoveIt / RViz
  -> linked_execution_controller/follow_joint_trajectory
  -> aubo_e5_controller/follow_joint_trajectory
  -> joint_path_command
  -> aubo_robot_simulator
  -> moveItController_cmd
  -> aubo_driver
  -> real robot
  -> /joint_states
  -> joint_state_mirror_adapter
  -> /real/joint_states
  -> [Gazebo OR Unity] mirror backend
```

Recommended launch files:

```bash
# Gazebo backend (default)
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch sim_only:=true
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.1.10

# Unity backend (NEW)
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch use_unity:=true sim_only:=true
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.1.10 use_unity:=true
```

Current modes:

- `sim_only:=true, use_unity:=false`: MoveIt controls Gazebo directly
- `sim_only:=false, use_unity:=false`: MoveIt drives real robot, Gazebo mirrors (default)
- `sim_only:=true, use_unity:=true`: MoveIt controls Unity directly
- `sim_only:=false, use_unity:=true`: MoveIt drives real robot, Unity mirrors (NEW)

Legacy entry points are still present for debugging and compatibility:

- `aubo_e5_moveit_config/launch/moveit_planning_execution.launch`
- `aubo_e5_moveit_config/launch/demo_gazebo.launch`

These legacy launch files are not the recommended system-level entry points anymore.

## Build Notes

Build from the workspace root, not from `src/`:

```bash
cd /home/wuqz/aubo_polish
catkin_make
```

The workspace assumes ROS 1 catkin and targets the AUBO E5 stack on Ubuntu/ROS environments compatible with MoveIt and Gazebo.
The exact package set depends on which subsystems you need.

Core runtime/build requirements:

- ROS catkin workspace
- MoveIt
- Gazebo with `gazebo_ros_control`
- `controller_manager`, `joint_state_controller`, `position_controllers`
- `xacro`

Important package-specific requirements:

- `aubo_robot/aubo_driver` expects bundled controller binaries under `aubo_robot/aubo_driver/lib/lib64`
  and uses the local protobuf runtime under `aubo_robot/opt/lib`.
- `mecheye_ros_interface` expects Mech-Eye SDK headers and libraries under `/opt/mech-mind/mech-eye-sdk`.
- `aubo_sdk_demo` requires `AUBO_SDK_ROOT` to point to an external SDK installation.

## Package Guide

The most relevant packages for the current system are:

- `aubo_robot/aubo_driver`: real robot driver and Gazebo bridge node
- `aubo_robot/aubo_controller`: ROS-Industrial style trajectory action server and interpolation bridge
- `aubo_robot/aubo_e5_moveit_config`: MoveIt configuration and controller-manager launch files
- `aubo_robot/aubo_gazebo`: Gazebo world, E5 control YAML, E5 simulation xacro
- `aubo_linked_execution`: linked execution action server, monitor, startup helper, joint-state adapter
- `aubo_unity_bridge`: Unity3D simulation backend, ROS-TCP bridge, joint state publisher (NEW)

Supporting packages:

- `aubo_robot/aubo_msgs`: custom messages/services for IO and FK/IK services
- `aubo_robot/aubo_description`: meshes and alternate robot-description assets
- `aubo_robot/PointCLoud_SLAM`: point cloud processing pipeline
- `industrial_core/*`: vendored ROS-Industrial packages

## Documentation Map

- `../docs/ARCHITECTURE.md`: complete system architecture with Gazebo and Unity data flows
- `../docs/LINKED_EXECUTION_DESIGN.md`: design rationale for the linked execution path
- `../docs/UNITY_MIGRATION.md`: Unity backend migration guide and debugging
- `../docs/MOTION_PLANNING.md`: path planning algorithms and configuration
- `../docs/TROUBLESHOOTING.md`: common issues and solutions
- `PACKAGE_STRUCTURE.md`: code-oriented structure and dependency guide for the E5 stack
- `aubo_robot/aubo_driver/README.md`: driver topics, services, and real-robot launch details
- `aubo_robot/aubo_gazebo/README.md`: Gazebo launch and controller layout
- `aubo_unity_bridge/README.md`: Unity bridge package documentation

## Safety

When `sim_only:=false`, the real robot is the actual executor. Gazebo is only a mirror.
Do not rely on Gazebo as a safety barrier. Keep the workcell clear and the emergency stop under operator control before sending trajectories.
