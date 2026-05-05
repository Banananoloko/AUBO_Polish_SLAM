# aubo_unity_bridge

> ROS 与 Unity3D 的双向通讯桥接包

**最后更新**: 2026-04-28  
**状态**: 生产就绪（P4/P5 完成）

---

## 概述

`aubo_unity_bridge` 是连接 ROS 和 Unity3D 的中间层，使 AUBO E5 机器人可以在 Unity 中进行高保真仿真和实时镜像。

### 核心功能

- ✅ 双向通讯：ROS ↔ Unity 关节状态和指令
- ✅ Shadow 模式：实机状态实时镜像到 Unity
- ✅ Normal 模式：仅 Unity 仿真
- ✅ 轨迹执行监视：检测 Unity 是否到达目标
- ✅ MoveIt 集成：直接与 MoveIt 规划器配合

---

## 快速开始

### 前置条件

- ROS Noetic
- Unity 2021 LTS 或更新版本
- ROS-TCP-Connector 插件（Unity 端）
- URDF Importer 插件（Unity 端）

### 启动 ROS 端

```bash
# 仅 Unity 仿真模式
roslaunch aubo_unity_bridge unity_bridge.launch

# 实机 + Unity Shadow 模式（需要在 linked_execution 中启用）
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    robot_ip:=192.168.1.10 use_unity:=true
```

### 启动 Unity 端

1. 打开 `/home/wuqz/UnityProjects/aubo_polish`
2. 点击 Play 按钮
3. 检查 Console 中的连接状态

---

## 包结构

```
aubo_unity_bridge/
├── launch/
│   └── unity_bridge.launch              # 主启动文件
├── scripts/
│   ├── fake_aubo_joint_states.py        # 初始化关节状态
│   ├── unity_joint_states_publisher.py  # 重发 Unity 状态
│   ├── unity_command_forwarder.py       # 转发指令（normal/shadow）
│   └── unity_execution_monitor.py       # 监视执行状态
├── config/
│   └── unity_topics.yaml                # 话题契约
└── docs/
    └── Migration_Plan.md                # 原始迁移计划（参考）
```

---

## 核心节点

### 1. fake_aubo_joint_states

**功能**：初始化关节状态，防止 MoveIt 启动时报错

**发布话题**：
- `/joint_states` - 初始关节状态

**参数**：
- `initial_position` - 初始关节角度（默认零位）

---

### 2. unity_joint_states_publisher

**功能**：订阅 Unity 发布的关节状态，重发到 ROS 标准话题

**订阅话题**：
- `/unity/joint_states` - Unity 发布的关节状态（50Hz）

**发布话题**：
- `/aubo_e5/joint_states` - 重发给 MoveIt 和 RViz

**参数**：
- `republish_rate` - 重发频率（默认 50Hz）

---

### 3. unity_command_forwarder

**功能**：转发轨迹指令到 Unity（支持 normal 和 shadow 两种模式）

**订阅话题**：
- `/joint_path_command` - MoveIt 下发的轨迹（normal 模式）
- `/real/joint_states` - 实机状态（shadow 模式）

**发布话题**：
- `/unity/joint_command` - 发送给 Unity 的指令

**参数**：
- `mode` - 运行模式（`normal` 或 `shadow`，默认 `normal`）
- `shadow` - 是否启用 shadow 模式（布尔值）

**模式说明**：
- **normal**：取轨迹末端 waypoint，发送给 Unity（单点控制）
- **shadow**：订阅实机状态，实时转发给 Unity（镜像模式）

---

### 4. unity_execution_monitor

**功能**：监视 Unity 执行状态，检测是否到达目标

**订阅话题**：
- `/unity/joint_states` - Unity 关节状态

**发布话题**：
- `/unity/execution_status` - 执行状态（RUNNING / SUCCEEDED / FAILED）

**参数**：
- `arrival_tolerance` - 到达容差（默认 0.05 rad）
- `timeout` - 执行超时时间（默认 30s）

---

## 启动参数

### unity_bridge.launch

```xml
<!-- 是否加载 robot_description（默认 true） -->
<arg name="load_robot_description" default="true"/>

<!-- 是否启用 shadow 模式（默认 false） -->
<arg name="shadow" default="false"/>

<!-- 是否启动 action server（默认 true） -->
<arg name="with_action_server" default="true"/>

<!-- ROS-TCP-Endpoint 地址（默认 localhost） -->
<arg name="ros_tcp_ip" default="127.0.0.1"/>

<!-- ROS-TCP-Endpoint 端口（默认 10000） -->
<arg name="ros_tcp_port" default="10000"/>
```

### 使用示例

```bash
# 仅 Unity 仿真（不加载 robot_description，不启动 action server）
roslaunch aubo_unity_bridge unity_bridge.launch \
    load_robot_description:=false \
    with_action_server:=false

# 实机 + Unity shadow 模式
roslaunch aubo_unity_bridge unity_bridge.launch \
    shadow:=true \
    with_action_server:=false

# 自定义 ROS-TCP-Endpoint 地址
roslaunch aubo_unity_bridge unity_bridge.launch \
    ros_tcp_ip:=192.168.1.100 \
    ros_tcp_port:=5005
```

---

## 话题契约

详见 `config/unity_topics.yaml`：

| 话题 | 方向 | 消息类型 | 频率 | 说明 |
|------|------|---------|------|------|
| `/unity/joint_states` | ROS ← Unity | sensor_msgs/JointState | 50Hz | Unity 发布的关节状态 |
| `/unity/joint_command` | ROS → Unity | sensor_msgs/JointState | 按需 | 发送给 Unity 的指令 |
| `/aubo_e5/joint_states` | ROS | sensor_msgs/JointState | 50Hz | 重发给 MoveIt 的状态 |
| `/unity/execution_status` | ROS | std_msgs/String | 10Hz | 执行状态 |

---

## Unity 端脚本

### AuboJointStatePublisher.cs

**功能**：定期发布关节状态到 ROS

**绑定对象**：6 个 ArticulationBody（各关节）

**发布频率**：50Hz

**关键方法**：
```csharp
void PublishJointStates()
{
    // 读取 ArticulationBody 位置
    // 发布到 /unity/joint_states
}
```

### AuboJointCommandSubscriber.cs

**功能**：订阅 ROS 指令，驱动 ArticulationBody xDrive

**订阅话题**：`/unity/joint_command`

**关键方法**：
```csharp
void OnJointCommandReceived(JointState msg)
{
    // 解析指令
    // 设置 xDrive.targetPosition
    // 检测到达
}
```

**PD 参数**（在 Inspector 中配置）：
- `stiffness` - 刚度（默认 8000）
- `damping` - 阻尼（默认 500）
- `forceLimit` - 力矩限制（默认 150 N·m）

---

## 常见问题

### Q: Unity 启动后机器人姿态混乱

**A**: xDrive PD 参数没有写入。检查：
1. URDF Importer 是否禁用了自带 Controller
2. `AuboJointCommandSubscriber` 是否正确初始化
3. Unity Console 中是否有 `init shoulder_joint:` 日志

### Q: `/unity/joint_states` 没有数据

**A**: 检查：
1. Unity 是否点了 Play
2. `AuboJointStatePublisher` 是否绑定了 6 个 ArticulationBody
3. ROS-TCP-Connector 连接状态

### Q: ROS 发指令但 Unity 没动

**A**: 检查：
1. `ros_tcp_endpoint` 连接状态
2. 关节名是否一致
3. Unity Console 中是否有 `[Unknown joint]` 警告

### Q: Shadow 模式下 Unity 跟踪有延迟

**A**: 可以尝试：
1. 增加 stiffness 参数（×2）
2. 在 `AuboJointCommandSubscriber` 中添加 velocity 前馈
3. 检查网络延迟

---

## 调试命令

### 检查节点运行状态

```bash
rosnode list | grep unity
rosnode info /unity_command_forwarder
```

### 检查话题频率

```bash
rostopic hz /unity/joint_states
rostopic hz /unity/joint_command
rostopic hz /aubo_e5/joint_states
```

### 手动发送指令

```bash
rostopic pub -1 /unity/joint_command sensor_msgs/JointState "{
  header: {stamp: now},
  name: ['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint'],
  position: [0, -0.3, 0.6, -0.3, 0, 0],
  velocity: [], effort: []
}"
```

### 查看关节状态

```bash
rostopic echo /unity/joint_states
rostopic echo /aubo_e5/joint_states
```

---

## 性能指标

| 指标 | 值 |
|------|-----|
| 关节状态发布频率 | 50Hz |
| 指令转发延迟 | < 50ms |
| 到达检测精度 | ±0.05 rad |
| 支持关节数 | 6 |
| 最大轨迹点数 | 无限制 |

---

## 相关文档

- [系统架构](../docs/ARCHITECTURE.md) - 完整数据流
- [Unity 迁移](../docs/UNITY_MIGRATION.md) - 迁移指南
- [故障排查](../docs/TROUBLESHOOTING.md) - 常见问题
- [操作指南](../OPERATION_GUIDE.md) - 使用指南

---

## 许可证

本包遵循项目主许可证。

---

**维护者**: AUBO Polish Project Team  
**最后更新**: 2026-04-28
