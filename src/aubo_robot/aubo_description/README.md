# AUBO Description Assets

This package contains meshes, launch files, and robot-description assets related to AUBO robots.

In the current `aubo_polish` workspace, its practical role is:

- provide E5 mesh assets under `meshes/aubo_e5`
- provide transmission / gazebo helper xacros reused by other descriptions
- keep alternate or historical description files that are still useful for reference

## Important Status Note

The active E5 planning/simulation flow in this workspace does not load its robot description from this package directly.
Current MoveIt launch files load:

```text
aubo_gazebo/urdf/aubo_e5.xacro
```

So this package should be treated as an asset package plus auxiliary description package, not the primary source of truth
for the currently running MoveIt/Gazebo E5 model.

## Files of Interest

- `urdf/aubo.transmission.xacro`
- `urdf/aubo.gazebo.xacro`
- `urdf/aubo_e5.urdf`
- `urdf/aubo_e5.urdf.xacro`
- `config/joint_names_aubo_e5.yaml`

## Caution

Some files in this package reflect older generation layouts or historical export paths.
When you need the current runtime model for the E5 stack, verify against:

- `aubo_gazebo/urdf/aubo_e5.xacro`
- `aubo_e5_moveit_config/launch/planning_context.launch`
