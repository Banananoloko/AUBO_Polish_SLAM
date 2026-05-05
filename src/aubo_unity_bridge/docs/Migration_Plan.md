# Gazebo → Unity3D 仿真迁移：现状与计划

> Unity3D 全面替代 Gazebo 作为 AUBO E5 联动系统的仿真后端。
> 上层（MoveIt / linked_execution_action_server / safety_monitor / aubo_driver）零改动。

---

## 当前进展

### ✅ ROS 端（aubo_unity_bridge）
- 4 个 Python 节点：`fake_aubo_joint_states`、`unity_joint_states_publisher`、`unity_command_forwarder`（normal+shadow 双模）、`unity_execution_monitor`
- `launch/unity_bridge.launch` 参数化启动
- `config/unity_topics.yaml` 话题契约

### ✅ Unity 端（/home/wuqz/UnityProjects/aubo_polish）
- ROS-TCP-Connector + URDF Importer 已装
- URDF 导入 + ArticulationBody 配置完成
- `Assets/Scripts/AuboJointStatePublisher.cs`（50Hz 发布关节状态）
- `Assets/Scripts/AuboJointCommandSubscriber.cs`（订阅指令驱动 xDrive，启动时强制写入 PD 参数）
- 禁用 URDF Importer 自带 Controller（防止覆盖 xDrive）

### ✅ 双向通讯已通
- `rostopic pub /unity/joint_command ...` → Unity 机器臂快速到达目标姿态
- Unity 关节状态 50Hz 回流到 `/aubo_e5/joint_states`
- RViz 通过 `robot_state_publisher` 同步显示

### 关键参数
| 关节 | Stiffness | Damping | Force Limit (N·m) |
|------|-----------|---------|-------------------|
| shoulder | 8000 | 500 | 150 |
| upperArm | 7500 | 500 | 150 |
| foreArm  | 5000 | 400 | 100 |
| wrist1/2 | 800 | 250 | 30 |
| wrist3   | 400 | 150 | 30 |

PhysX 求解器：30/10 迭代，Fixed Timestep 0.005s（200Hz）。

---

## 当前已知问题

| # | 问题 | 影响 | 优先级 |
|---|------|------|--------|
| 1 | Shadow 模式未用 velocity 前馈，仅 PD 跟踪位置 | 实机镜像有 5-10ms 相位滞后 | 低 |
| 2 | URDF xacro 文件引用了不存在的包 `your_robot_description` | unity_bridge 改用纯 URDF 绕过；其他 launch 不受影响 | 低 |
| 3 | sim_only 模式下 RViz 不跟随 Gazebo 执行（沿用旧问题） | 仅影响 Gazebo 后端，Unity 后端不受影响 | 低 |

---

## 下一步计划

### P4 — MoveIt + Unity 集成测试 ✅ 启动配置就绪

`unity_bridge.launch` 现在自带 `aubo_joint_trajectory_action` + `aubo_robot_simulator`
（受 `with_action_server` 开关控制，独立启动时默认 true，被联动 launch 包含时关掉）。
MoveIt 的 `aubo_e5_controller/follow_joint_trajectory` 直接由 unity_bridge 提供。

```bash
# 终端 1：起 Unity 桥（含 ROS-TCP / 关节状态 / 指令转发 / 收敛监视 / 轨迹动作服务器 / 模拟器）
roslaunch aubo_unity_bridge unity_bridge.launch
# Unity 点 Play（确认 ROSConnection 状态绿色）

# 终端 2：起 MoveIt（默认控制器管理器即 aubo_e5_controller）
roslaunch aubo_e5_moveit_config move_group.launch

# 终端 3：交互式发目标
rosrun aubo_planner goto_pose.py
目标坐标> 0.4 0.0 0.5
```
**预期**：MoveIt 规划成功 → `aubo_joint_trajectory_action` 接收 → 发布 `/joint_path_command` →
`unity_command_forwarder` 取末端 waypoint 推给 Unity → Unity 内臂运动到目标 →
Unity Console 打印 `command received` + `arrive`。

> ⚠️ 当前 normal 模式只取轨迹**最后一点**送 Unity，靠 ArticulationBody PD 单段插补。
> 中间路点 / 速度曲线在 Unity 内部不重现，但末端定位正确。如需轨迹回放精度，再扩展
> `unity_command_forwarder._forward_trajectory` 按 time_from_start 节流下发。

### P5 — 联动 launch 增加 use_unity 分支 ✅ 已实现

`aubo_e5_linked_execution.launch` 新增：
- `<arg name="use_unity" default="false"/>`
- 仿真后端二选一（`<group if/unless="$(arg use_unity)">` 包裹）：
  - `use_unity:=false`（默认）→ 走原 Gazebo 路径（含 sim_only / shadow 两种）
  - `use_unity:=true`  → 走 `unity_bridge.launch`，自动按 `sim_only` 选 normal/shadow，
    并传 `load_robot_description:=false` + `with_action_server:=false`（避免与父 launch
    的 `planning_context.launch` / `aubo_joint_trajectory_action` 冲突）
- `gazebo_rtf_monitor` 改为仅 Gazebo 联动模式启用（`use_unity:=true` 时跳过）

启动矩阵：

| 命令 | sim_only | use_unity | 后端 |
|------|----------|-----------|------|
| `aubo_e5_linked_execution.launch` | false | false | 实机 + Gazebo (shadow) |
| `... sim_only:=true` | true | false | 仅 Gazebo |
| `... use_unity:=true sim_only:=true` | true | true | 仅 Unity |
| `... use_unity:=true robot_ip:=...` | false | true | 实机 + Unity (shadow)（即 P6）|

### P6 — 实机 + Unity Shadow 模式测试
```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    robot_ip:=192.168.1.10 use_unity:=true
```
**预期**：实机运动 → Unity 实时镜像；MoveIt 规划经 `linked_execution_action_server` 双路成功判定。

### P7 — Shadow 模式跟踪精度优化（按需）
仅当 P6 测出明显滞后（>20ms）才做：
- **方案 A**：Shadow 模式下 stiffness × 2，跟踪误差减半
- **方案 B**：`AuboJointCommandSubscriber` 读取 `msg.velocity` 写入 `xDrive.targetVelocity` 做前馈

---

## P4 / P5 Debug 命令包

下面这套命令在 P4 或 P5 任一阶段卡住时按从上到下的顺序排查，定位断点。

### 1. 节点 / 话题存活检查
```bash
rosnode list | grep -E 'unity|aubo|move_group'
rostopic list | grep -E 'unity|joint_path_command|joint_states|follow_joint_trajectory'
```
**应当看到**：`unity_command_forwarder`、`unity_joint_states_publisher`、`unity_execution_monitor`、
`aubo_joint_trajectory_action`、`aubo_robot_simulator`、`move_group`、`robot_state_publisher`、`ros_tcp_endpoint`。

### 2. Unity ↔ ROS 双向通讯
```bash
# Unity → ROS：50Hz 关节状态
rostopic hz /unity/joint_states           # 期望 ≈ 50
rostopic hz /aubo_e5/joint_states         # 期望 ≈ 50（unity_joint_states_publisher 重发）

# ROS → Unity：手动下发指令，看 Unity Console 的 [command received]/[arrive]
rostopic pub -1 /unity/joint_command sensor_msgs/JointState "{
  header: {stamp: now}, name: ['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint'],
  position: [0,-0.3,0.6,-0.3,0,0], velocity: [], effort: []}"
```
若 `/unity/joint_states` 没有 → 检查 Unity Play 是否点了、`AuboJointStatePublisher` 是否绑定了 6 个 ArticulationBody。
若 ROS 端发了指令但 Unity 没动 → 看 `ros_tcp_endpoint` 终端是否报 endpoint 错误，或 Unity Console 的 `Unknown joint` 警告。

### 3. MoveIt → 轨迹动作链路
```bash
# 4.1 MoveIt 是否能 connect 到动作服务器（必须 SUCCEEDED）
rostopic echo -n 1 /aubo_e5_controller/follow_joint_trajectory/status

# 4.2 aubo_joint_trajectory_action 是否在产出 /joint_path_command
rostopic echo -n 1 /joint_path_command   # goto_pose 执行时应看到

# 4.3 unity_command_forwarder 是否在产出 /unity/joint_command
rostopic echo -n 1 /unity/joint_command  # 应包含 6 个关节的 position

# 4.4 aubo_robot_simulator 是否产出 feedback_states（让动作服务器判完成）
rostopic hz /feedback_states             # 期望 > 0
```
若 4.1 没有 SUCCEEDED → 多半是动作服务器没起或 feedback_states 没流（4.4 检查）。
若 4.2 有但 4.3 无 → `unity_command_forwarder` 的 mode 不是 normal（确认未误传 `shadow:=true`）。

### 4. TF / robot_description 一致性
```bash
rosparam get /robot_description | head -c 200    # 必须有内容
rosparam get /robot_name                          # 应为 'aubo_e5'
rosrun tf tf_echo base_link wrist3_Link 2>&1 | head -5
```
若 robot_description 缺失 → unity_bridge 的 `load_robot_description` 被关掉、且父 launch 没加载。
若 RViz 不动 / TF 不更新 → `unity_joint_states_publisher` 没把 `/unity/joint_states` 重发到 `/aubo_e5/joint_states`。

### 5. 常见故障速查

| 现象 | 大概率原因 | 验证 |
|------|------------|------|
| MoveIt Plan OK，Execute 一直转圈 | 动作服务器没收到 feedback_states | `rostopic hz /feedback_states` |
| Unity 启动后立刻塌成乱姿 | xDrive PD 没写入（被 URDF Importer 覆盖） | Unity Console 找 `init shoulder_joint:` 行 |
| `/aubo_e5/joint_states` 50Hz 但 RViz 不动 | TF 没起或 robot_description 不一致 | `rostopic echo /tf` + `rosparam get /robot_description` |
| `goto_pose.py` 报 `Unable to identify ... action server` | aubo_joint_trajectory_action 没起 | `rosnode list | grep aubo_joint_trajectory_action` |
| MoveIt 报 `Didn't receive robot state ... within 1 seconds` | `/joint_states` 无人发布（MoveIt 的 current_state_monitor 默认订该话题） | `rostopic hz /joint_states` 应 ≥ 10；若空，确认 `aubo_robot_simulator` 没有把 joint_states remap 走，或检查 `aubo_driver` 是否在产出（联动模式） |
| `Inbound TCP/IP connection failed: ... 0 bytes were received` | ros_tcp_endpoint 收到短链路连接（Unity 重连或外部 probe） | 一般无害；若持续刷屏检查 Unity ROSConnection 配置和 IP/端口一致性 |
| `[Unknown joint]` 警告 | 关节名前缀不一致（如带 namespace） | 检查 `unity_command_forwarder` mode 与上游消息的 `name` 字段 |
| Unity 动了但 `arrive` 不出 | PD 太弱、关节卡住、容差太严 | Unity Inspector 调 `arrivalToleranceDeg` 或加 stiffness |

---

## 单点姿态测试（Unity 端验证）

确认 Unity 已点 Play、`unity_bridge.launch` 已起后，在 ROS 端单次下发一个目标姿态：

```bash
# 目标姿态: [0, -0.3, 0.6, -0.3, 0, 0] rad（轻度抬臂，关节都在限位内）
rostopic pub -1 /unity/joint_command sensor_msgs/JointState "{
  header: {stamp: now, frame_id: ''},
  name: ['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint'],
  position: [0.0, -0.3, 0.6, -0.3, 0.0, 0.0],
  velocity: [], effort: []
}"
```

**预期 Unity Console 输出（事件驱动，仅 2 行）**：
```
[AuboJointCommandSubscriber] command received: shoulder_joint cur=…°→tgt=0.00° | upperArm_joint cur=…°→tgt=-17.19° | … (6/6 applied)
[AuboJointCommandSubscriber] arrive: shoulder_joint cur=0.00° tgt=0.00° | … (maxErr=…° maxVel=…rad/s in …s)
```

回归零位（测试可重复）：
```bash
rostopic pub -1 /unity/joint_command sensor_msgs/JointState "{
  header: {stamp: now, frame_id: ''},
  name: ['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint'],
  position: [0,0,0,0,0,0], velocity: [], effort: []}"
```

如果只看到 `command received` 而无 `arrive`：检查 PD 参数、关节是否被卡住、或调大 `arrivalToleranceDeg` / `arrivalTimeoutSeconds`。

---

## 关键文件索引

| 路径 | 说明 |
|------|------|
| `src/aubo_unity_bridge/scripts/` | ROS 侧 4 个 Python 节点 |
| `src/aubo_unity_bridge/launch/unity_bridge.launch` | 主启动文件 |
| `src/aubo_unity_bridge/config/unity_topics.yaml` | 话题契约 |
| `UnityProjects/aubo_polish/Assets/Scripts/Aubo*.cs` | Unity 端 2 个脚本 |

---

*最后更新：2026-04-26 晚*
