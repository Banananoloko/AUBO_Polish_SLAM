# AUBO E5 系统架构

> 支持 Gazebo 和 Unity 双仿真后端的完整机器人控制系统架构

**最后更新**: 2026-04-28  
**支持后端**: Gazebo (原生) + Unity (新增)

---

## 1. 系统概览

AUBO E5 联动系统采用三层架构，支持两种仿真后端：

```
┌─────────────────────────────────────────┐
│  用户交互层                              │
│  RViz + MoveIt 路径规划与碰撞检测        │
└──────────────┬──────────────────────────┘
               │ FollowJointTrajectory Goal
               ↓
┌─────────────────────────────────────────┐
│  安全监控层 (safety_monitor)             │
│  • 启动位置同步验证                      │
│  • 大幅度运动检测 (0.5 rad)              │
│  • 轨迹起点验证 (0.15 rad)               │
└──────────────┬──────────────────────────┘
               │ safe_to_execute
               ↓
┌─────────────────────────────────────────┐
│  联动聚合层 (aubo_linked_execution)      │
│  • 协调中枢：双重成功判定                 │
│  • 安全阻断：执行前检查                   │
│  • 后端选择：Gazebo 或 Unity              │
│  • 状态适配：关节重排                    │
│  • 自动上电：模式切换                    │
└──────┬──────────────────────┬───────────┘
       │                      │
       ↓                      ↓
┌──────────────────┐   ┌──────────────────┐
│  实机执行链       │   │  仿真镜像链      │
│  • 轨迹跟踪       │──→│  • Shadow 模式   │
│  • AUBO SDK      │   │  • 实时同步      │
│  • 500Hz 控制    │   │  • 位置控制      │
└──────────────────┘   └──────────────────┘
                              │
                    ┌─────────┴─────────┐
                    ↓                   ↓
            ┌──────────────┐    ┌──────────────┐
            │  Gazebo      │    │  Unity       │
            │  后端        │    │  后端        │
            └──────────────┘    └──────────────┘
```

### 启动模式矩阵

| 命令 | sim_only | use_unity | 后端 | 用途 |
|------|----------|-----------|------|------|
| `aubo_e5_linked_execution.launch` | false | false | 实机 + Gazebo (shadow) | 实机测试 + 仿真镜像 |
| `... sim_only:=true` | true | false | 仅 Gazebo | 算法开发、规划测试 |
| `... use_unity:=true sim_only:=true` | true | true | 仅 Unity | Unity 独立测试 |
| `... use_unity:=true robot_ip:=...` | false | true | 实机 + Unity (shadow) | 实机 + Unity 镜像 |

---

## 2. Gazebo 联动模式数据流

### 2.1 实机 + Gazebo Shadow 模式（默认）

```
MoveIt
  ↓ (goal)
linked_execution_controller/follow_joint_trajectory
  ↓ (转发)
aubo_e5_controller/follow_joint_trajectory
  ↓ (发布)
/joint_path_command
  ↓ (订阅)
aubo_robot_simulator ──→ /joint_states (MoveIt 读取当前状态)
  ↓ (发布)              ──→ /feedback_states (action 判定用，remap 到 sim/)
/moveItController_cmd
  ↓ (订阅)
aubo_driver
  ↓ (AUBO SDK)
实机执行
  ↓ (发布)
/joint_states (实机真实状态)
  ↓ (订阅)
joint_state_mirror_adapter
  ↓ (重排后发布)
/real/joint_states
  ↓ (订阅)
aubo_gazebo_driver (shadow=true)
  ↓ (写入 Gazebo 控制器)
Gazebo 6 个 position_controller
  ↓ (Gazebo 物理引擎)
Gazebo 关节运动
  ↓ (joint_state_controller 发布)
/aubo_e5/joint_states (Gazebo 镜像状态)
  ↓ (订阅)
linked_execution_monitor (对比实机与 Gazebo 收敛)
```

**关键设计原则**：
1. 实机是唯一执行主体
2. Gazebo 只做镜像呈现，不参与控制决策
3. 联动成功 = 实机成功 AND Gazebo 收敛成功
4. aubo_robot_simulator 的 feedback_states/joint_states 必须 remap 到 sim/ 命名空间

### 2.2 仅 Gazebo 仿真模式

```
MoveIt
  ↓
aubo_e5_controller/follow_joint_trajectory
  ↓
/joint_path_command
  ↓
aubo_robot_simulator ──→ /joint_states (MoveIt 读取，无 remap)
  ↓                   ──→ /feedback_states (action 判定，无 remap)
/moveItController_cmd
  ↓
aubo_gazebo_driver (shadow=false, 订阅 /joint_path_command)
  ↓
Gazebo 控制器
  ↓
/aubo_e5/joint_states
```

---

## 3. Unity 联动模式数据流

### 3.1 实机 + Unity Shadow 模式

```
MoveIt
  ↓
linked_execution_controller/follow_joint_trajectory
  ↓
aubo_e5_controller/follow_joint_trajectory (由父 launch 启动)
  ↓
/joint_path_command
  ↓
aubo_robot_simulator (由父 launch 启动，remap 到 sim/) ──→ sim/joint_states
  ↓                                                      ──→ sim/feedback_states
/moveItController_cmd
  ↓
aubo_driver
  ↓
实机执行
  ↓
/joint_states (实机真实状态)
  ↓
joint_state_mirror_adapter
  ↓
/real/joint_states
  ↓
unity_command_forwarder (shadow=true, 订阅 /real/joint_states)
  ↓
/unity/joint_command (发布末端目标)
  ↓
ROS-TCP-Endpoint
  ↓
Unity 端 AuboJointCommandSubscriber
  ↓
ArticulationBody xDrive (PD 跟踪)
  ↓
Unity 关节运动
  ↓
AuboJointStatePublisher (50Hz 发布)
  ↓
/unity/joint_states
  ↓
unity_joint_states_publisher (重发到 /aubo_e5/joint_states)
  ↓
linked_execution_monitor (对比实机与 Unity 收敛)
```

### 3.2 仅 Unity 仿真模式

```
MoveIt
  ↓
aubo_e5_controller/follow_joint_trajectory (由 unity_bridge 启动)
  ↓
/joint_path_command
  ↓
aubo_robot_simulator (由 unity_bridge 启动，无 remap) ──→ /joint_states
  ↓                                                    ──→ /feedback_states
/moveItController_cmd
  ↓
unity_command_forwarder (normal 模式, 取末端 waypoint)
  ↓
/unity/joint_command
  ↓
ROS-TCP-Endpoint
  ↓
Unity 端 AuboJointCommandSubscriber
  ↓
ArticulationBody xDrive
  ↓
Unity 关节运动
  ↓
AuboJointStatePublisher
  ↓
/unity/joint_states
  ↓
unity_joint_states_publisher
  ↓
/aubo_e5/joint_states (MoveIt 读取)
```

---

## 4. 关键话题与数据流

| 话题名称 | 消息类型 | 发布者 | 订阅者 | 频率 | 说明 |
|---------|---------|--------|--------|------|------|
| `/joint_path_command` | trajectory_msgs/JointTrajectory | aubo_joint_trajectory_action | aubo_robot_simulator | 按需 | MoveIt 下发的完整轨迹 |
| `/moveItController_cmd` | trajectory_msgs/JointTrajectoryPoint | aubo_robot_simulator | aubo_driver | 200Hz | 插补后的轨迹点 |
| `/joint_states` | sensor_msgs/JointState | aubo_driver | joint_state_mirror_adapter | 500Hz | 实机当前关节状态 |
| `/real/joint_states` | sensor_msgs/JointState | joint_state_mirror_adapter | aubo_gazebo_driver / unity_command_forwarder | 500Hz | 重排后的实机状态 |
| `/feedback_states` | control_msgs/FollowJointTrajectoryFeedback | aubo_driver | aubo_joint_trajectory_action | 500Hz | 实机轨迹跟踪反馈 |
| `/robot_status` | industrial_msgs/RobotStatus | aubo_driver | aubo_joint_trajectory_action, aubo_robot_startup | 10Hz | 机器人状态（急停、报警等） |
| `/aubo_e5/joint_states` | sensor_msgs/JointState | Gazebo / unity_joint_states_publisher | linked_execution_monitor | 10Hz | 仿真当前关节状态 |
| `/unity/joint_states` | sensor_msgs/JointState | AuboJointStatePublisher (Unity) | unity_joint_states_publisher | 50Hz | Unity 关节状态 |
| `/unity/joint_command` | sensor_msgs/JointState | unity_command_forwarder | ROS-TCP-Endpoint | 按需 | 发送给 Unity 的指令 |
| `/linked_execution/monitor_goal` | sensor_msgs/JointState | linked_execution_action_server | linked_execution_monitor | 按需 | 轨迹终点目标 |
| `/linked_execution/monitor_status` | std_msgs/String | linked_execution_monitor | linked_execution_action_server | 10Hz | 仿真收敛状态 |

---

## 5. Action 接口

| Action Server | 类型 | 提供者 | 客户端 | 说明 |
|--------------|------|--------|--------|------|
| `/linked_execution_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectoryAction | linked_execution_action_server | MoveIt | 联动聚合层入口 |
| `/aubo_e5_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectoryAction | aubo_joint_trajectory_action | linked_execution_action_server | 实机控制器 |

---

## 6. 服务接口

| 服务名称 | 类型 | 提供者 | 说明 |
|---------|------|--------|------|
| `/aubo_driver/set_io` | aubo_msgs/SetIO | aubo_driver | 设置数字/模拟 IO |
| `/aubo_driver/get_ik` | aubo_msgs/GetIK | aubo_driver | 逆运动学求解 |
| `/aubo_driver/get_fk` | aubo_msgs/GetFK | aubo_driver | 正运动学求解 |

---

## 7. 控制模式切换

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/robot_control` | std_msgs/String | 机器人控制命令 (powerOn, powerOff) |
| `/aubo_driver/controller_switch` | std_msgs/Int32 | 控制模式切换 (0=示教器, 1=RosMoveIt) |

---

## 8. Gazebo vs Unity 对比

| 特性 | Gazebo | Unity |
|------|--------|-------|
| 物理引擎 | ODE/Bullet | PhysX |
| 关节状态频率 | 10Hz (joint_state_controller) | 50Hz (AuboJointStatePublisher) |
| 控制方式 | JointPositionController (PID) | ArticulationBody xDrive (PD) |
| 碰撞检测 | 内置 | PhysX 内置 |
| 可视化 | 原生 3D 环境 | 高保真渲染 |
| 性能 | 中等（RTF ~1.0） | 高（取决于 GPU） |
| 学习曲线 | 陡峭 | 平缓 |
| 扩展性 | 插件系统 | C# 脚本 |

---

## 9. 核心组件

### 联动聚合层 (`aubo_linked_execution`)

| 组件 | 功能 |
|------|------|
| `linked_execution_action_server.py` | 协调中枢，接收 MoveIt 目标，集成安全检查，等待双重成功确认 |
| `safety_monitor.py` | 安全监控器，检测大幅度运动和轨迹起点不匹配 |
| `linked_execution_monitor.py` | 仿真收敛监视器，检测是否到达目标位置 |
| `joint_state_mirror_adapter.py` | 关节状态重排适配器，确保顺序一致 |
| `aubo_robot_startup.py` | 实机自动上电、控制模式切换和位置同步验证 |

### 实机执行链 (`aubo_robot`)

| 组件 | 功能 |
|------|------|
| `aubo_joint_trajectory_action` | Action Server，接收轨迹并监控执行 |
| `aubo_robot_simulator.py` | 轨迹插补桥，200Hz 插补轨迹点 |
| `aubo_driver` | AUBO SDK 驱动，500Hz 实时控制 |

### Gazebo 镜像链 (`aubo_gazebo`)

| 组件 | 功能 |
|------|------|
| `aubo_gazebo_driver` | Shadow 模式驱动，镜像实机状态到 Gazebo |
| Gazebo Controllers | 6 个单关节位置控制器 |

### Unity 镜像链 (`aubo_unity_bridge`)

| 组件 | 功能 |
|------|------|
| `unity_command_forwarder.py` | 转发轨迹指令到 Unity（normal/shadow 双模） |
| `unity_joint_states_publisher.py` | 重发 Unity 关节状态到 ROS |
| `unity_execution_monitor.py` | 监视 Unity 执行状态 |
| `AuboJointCommandSubscriber.cs` (Unity) | 订阅指令驱动 xDrive |
| `AuboJointStatePublisher.cs` (Unity) | 发布关节状态 |

---

## 10. 启动命令参考

```bash
# Gazebo 联动模式（实机 + Gazebo 镜像）
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    robot_ip:=192.168.10.230

# Gazebo 仿真模式（仅 Gazebo）
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    sim_only:=true

# Unity 仿真模式（仅 Unity）
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    use_unity:=true sim_only:=true

# Unity 联动模式（实机 + Unity 镜像）
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    robot_ip:=192.168.10.230 use_unity:=true
```

---

**相关文档**：
- [联动设计](LINKED_EXECUTION_DESIGN.md) - 详细设计原理与实现状态
- [故障排查](TROUBLESHOOTING.md) - 常见问题解决
- [安全系统](SAFETY_SYSTEM.md) - 安全机制详解
