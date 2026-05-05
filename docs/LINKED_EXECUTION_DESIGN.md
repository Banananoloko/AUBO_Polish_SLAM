# RViz 到实机执行并在 Gazebo 中呈现的联动链路设计文档

## 1. 背景与目标

当前仓库中已经存在三类相关能力，但它们彼此并未统一成一条完整的执行链：

1. MoveIt / RViz 规划并通过 ROS-Industrial 风格 Action 下发轨迹。
2. AUBO 实机驱动通过 `aubo_driver` 与 AUBO SDK 通信，实现真实机器人运动。
3. Gazebo 通过 `aubo_gazebo_driver` 的 shadow 模式镜像关节状态，实现实机运动的仿真可视化。

本设计文档的目标，是在不改动路径规划算法本身的前提下，给出一条系统级联动方案，使得：

- 用户在 RViz 中完成路径规划和运动指令下发。
- 指令最终驱动 AUBO 实机执行。
- Gazebo 中同时呈现实机的运动结果。
- 整条链路的职责边界清晰，后续可以稳定实现、验证和维护。

本设计明确采用以下原则：

- 实机是唯一执行主体。
- Gazebo 只做镜像呈现，不参与控制决策。
- 上层联动结果必须同时考虑“实机执行成功”和“Gazebo 镜像成功”。

## 2. 现有仓库能力审查

### 2.1 现有实机主链

仓库中已有一条可用的实机执行主链：

`MoveIt / RViz -> aubo_joint_trajectory_action -> joint_path_command -> aubo_robot_simulator(Python) -> moveItController_cmd -> aubo_driver -> AUBO SDK -> 实机`

关键文件与职责：

- `src/aubo_robot/aubo_controller/src/joint_trajectory_action.cpp`
  - 提供 `FollowJointTrajectory` Action Server。
  - 接收 MoveIt 下发的轨迹后，发布到 `joint_path_command`。

- `src/aubo_robot/aubo_controller/script/aubo_controller/aubo_robot_simulator`
  - 该脚本虽然名称叫 simulator，但当前有效角色其实是“轨迹插补和桥接器”。
  - 它订阅 `joint_path_command`，做插补后发布 `moveItController_cmd`。

- `src/aubo_robot/aubo_driver/src/aubo_driver.cpp`
  - 订阅 `moveItController_cmd`。
  - 将插补点转换为 AUBO SDK 可接受的数据，并下发给实机。
  - 同时发布 `joint_states`、`feedback_states`、`robot_status` 等反馈信息。

### 2.2 现有 Gazebo 镜像链

仓库中也已有一条 Gazebo 镜像链：

`/real/joint_states -> aubo_gazebo_driver(shadow 模式) -> /aubo_e5/*_position_controller/command -> Gazebo`

关键文件：

- `src/aubo_robot/aubo_gazebo/launch/aubo_e5_gazebo_control.launch`
  - 启动 Gazebo、机器人模型、controller_manager、robot_state_publisher。
  - 启动 `aubo_gazebo_driver` 并可传入 `shadow:=true`。

- `src/aubo_robot/aubo_driver/src/aubo_gazebo_driver.cpp`
  - 在 `shadow:=true` 时订阅实机关节状态。
  - 将关节值写入 Gazebo 的 6 个单关节位置控制器。
  - 已带有 Gazebo pause/reset 的时间跳变保护逻辑。

### 2.3 现有旧 Gazebo 执行链

仓库里还保留一条旧的 Gazebo 轨迹执行链：

`MoveIt -> ros_control JointTrajectoryController -> Gazebo`

主要位于：

- `src/aubo_robot/aubo_e5_moveit_config/launch/demo_gazebo.launch`
- `src/aubo_robot/aubo_e5_moveit_config/launch/gazebo.launch`
- `src/aubo_robot/aubo_e5_moveit_config/launch/ros_controllers.launch`
- `src/aubo_robot/aubo_e5_moveit_config/config/ros_controllers.yaml`

这条链的特点是：

- Gazebo 作为独立执行器。
- MoveIt 直接向 Gazebo 的 `JointTrajectoryController` 发 action。
- 它与实机主链不是同一个控制模型。

### 2.4 现状问题总结

当前仓库不是“一条统一链”，而是多条链路并存：

- 实机链：ROS-Industrial Action + 自定义插补桥 + AUBO SDK。
- Gazebo 镜像链：关节状态镜像到单关节控制器。
- Gazebo 旧执行链：MoveIt 直接控制 Gazebo 的 `JointTrajectoryController`。

如果不先统一职责，后续实现很容易出现：

- 同一轨迹同时写入实机和 Gazebo 两套控制器，造成时序不一致。
- MoveIt 控制器名、topic 名和反馈语义彼此冲突。
- Gazebo 呈现正确但实机失败，或实机成功但 Gazebo 未跟上，上层无法知道整条链是否真正成功。

## 3. 设计原则

为保证可行性和稳定性，本方案采用以下原则：

1. **单控制源**
   - 上层只保留一套统一执行入口。
   - MoveIt 不直接同时控制实机和 Gazebo 两套执行器。

2. **实机唯一执行体**
   - 真正执行轨迹的是 `aubo_driver + AUBO SDK + 实机`。
   - Gazebo 不参与执行决策，不反向影响实机。

3. **Gazebo 仅镜像呈现**
   - Gazebo 只消费实机反馈状态并显示结果。
   - 不与实机争夺控制权。

4. **双结果判定**
   - 上层认为一次联动执行成功，必须同时满足：
     - 实机执行成功。
     - Gazebo 镜像在规定时间内收敛成功。

5. **与现有代码兼容**
   - 尽量复用 `aubo_joint_trajectory_action`、`aubo_robot_simulator`、`aubo_driver`、`aubo_gazebo_driver`。
   - 不推翻底层驱动逻辑，不涉及路径规划算法改造。

## 4. 目标架构

### 4.1 统一后的链路

统一后的目标链路如下：

```text
RViz / MoveIt
    |
    v
linked_execution_controller/follow_joint_trajectory
    |
    v
aubo_e5_controller/follow_joint_trajectory
    |
    v
joint_path_command
    |
    v
aubo_robot_simulator (Python bridge / interpolation)
    |
    v
moveItController_cmd
    |
    v
aubo_driver
    |
    v
AUBO SDK / 实机
    |
    +----------------------> /joint_states
                                 |
                                 v
                      joint_state_mirror_adapter
                                 |
                                 v
                       /real/joint_states_mirror
                                 |
                                 v
                    aubo_gazebo_driver (shadow=true)
                                 |
                                 v
                 /aubo_e5/*_position_controller/command
                                 |
                                 v
                               Gazebo
```

### 4.2 联动成功的正式定义

一次联动执行成功，定义为：

- 下游实机 Action 成功。
- Gazebo 侧关节状态在规定时间窗口内收敛到目标轨迹终点，并与实机反馈一致到阈值范围内。

因此，上层不能再直接把现有 `aubo_e5_controller/follow_joint_trajectory` 当作最终结果来源，而要新增一个聚合执行入口。

## 5. 新增系统组件设计

### 5.1 `joint_state_mirror_adapter`

#### 作用

该节点位于实机反馈和 Gazebo shadow 输入之间，职责是：

- 订阅实机侧 `joint_states`。
- 按标准关节名顺序重排数据。
- 输出稳定、顺序固定的镜像状态流。
- 作为 Gazebo shadow 模式的统一输入。

#### 为什么必须存在

当前 `aubo_gazebo_driver` 的 shadow 模式默认按 `position[i]` 直接拷贝到 6 个目标关节，不按 `joint_names` 做映射。

这意味着如果 `JointState` 中的关节顺序变化，即使名字正确，Gazebo 也会错位运动。

因此，镜像链必须先经过一个标准化适配层。

#### 输入输出建议

- 输入：`/joint_states`
- 输出：`/real/joint_states_mirror`

#### 行为要求

- 严格按以下顺序输出：
  - `shoulder_joint`
  - `upperArm_joint`
  - `foreArm_joint`
  - `wrist1_joint`
  - `wrist2_joint`
  - `wrist3_joint`
- 如果输入消息缺关节名、存在重复关节名、长度不匹配，应拒绝转发并打日志。
- 时间戳应直接沿用输入消息时间戳。

### 5.2 `linked_execution_monitor`

#### 作用

该节点用于判断 Gazebo 是否真正完成了“镜像成功”。

它不负责执行轨迹，只负责监视以下三类信息：

- 轨迹最终目标点。
- 实机当前关节状态。
- Gazebo 当前关节状态。

#### 输入

- 轨迹终点：来自联动执行器发送的监视目标。
- 实机状态：`/joint_states`
- Gazebo 状态：`/aubo_e5/joint_states`

#### 输出

- 一次联动会话对应的镜像状态：
  - `IDLE`
  - `TRACKING`
  - `SUCCEEDED`
  - `FAILED`
  - `CANCELED`
- 失败原因字符串。
- 建议额外输出最大误差、超时信息，用于调试。

#### 收敛判定建议

默认参数建议：

- 收敛超时时间：`2.0 s`
- 单关节误差阈值：`0.03 rad`
- 连续成功采样次数：`3`
- 状态过期超时：`1.0 s`

判定规则：

- 实机当前状态与目标终点误差小于阈值。
- Gazebo 当前状态与目标终点误差小于阈值。
- Gazebo 当前状态与实机当前状态误差小于阈值。
- 上述条件连续满足 `N` 次采样，则判定 `SUCCEEDED`。
- 超时仍未满足则判定 `FAILED`。

### 5.3 `linked_execution_action_server`

#### 作用

该节点是新的上层统一执行入口。

它对外暴露新的 `FollowJointTrajectoryAction`，对内复用现有的实机 Action Server。

#### 对上层的角色

对 MoveIt / RViz 来说，它就是唯一的“联动执行控制器”。

#### 对下游的角色

它做两件事：

1. 把 goal 转发给现有 `aubo_e5_controller/follow_joint_trajectory`。
2. 同时启动 Gazebo 镜像监视，并等待镜像成功。

#### 成功/失败逻辑

- 如果实机执行失败：整条联动失败。
- 如果实机成功，但 Gazebo 未收敛：整条联动失败。
- 只有实机成功且 Gazebo 收敛成功：整条联动成功。

#### 推荐 Action 名

- 对外：`linked_execution_controller/follow_joint_trajectory`
- 对内下游：`aubo_e5_controller/follow_joint_trajectory`

#### 设计理由

这样做有两个好处：

- 上层只看到一个统一控制器。
- 现有的实机控制器语义不需要被大幅侵入修改。

## 6. 参数、话题与 Action 约定

### 6.1 Action 约定

#### 对上层暴露

- `linked_execution_controller/follow_joint_trajectory`

#### 下游复用

- `aubo_e5_controller/follow_joint_trajectory`

### 6.2 话题约定

#### 现有实机链核心话题

- `joint_path_command`
- `moveItController_cmd`
- `joint_states`
- `feedback_states`
- `robot_status`

#### 新增联动层话题

- `/linked_execution/monitor_goal`
- `/linked_execution/monitor_control`
- `/linked_execution/monitor_status`
- `/real/joint_states_mirror`

#### Gazebo 侧关键话题

- `/aubo_e5/joint_states`
- `/aubo_e5/shoulder_joint_position_controller/command`
- `/aubo_e5/upperArm_joint_position_controller/command`
- `/aubo_e5/foreArm_joint_position_controller/command`
- `/aubo_e5/wrist1_joint_position_controller/command`
- `/aubo_e5/wrist2_joint_position_controller/command`
- `/aubo_e5/wrist3_joint_position_controller/command`

### 6.3 参数约定

建议统一约定以下参数：

- `controller_joint_names`
- `/robot_name = aubo_e5`
- `~real_joint_states_topic = /joint_states`
- `~mirror_output_topic = /real/joint_states_mirror`
- `~gazebo_joint_states_topic = /aubo_e5/joint_states`
- `~joint_tolerance = 0.03`
- `~success_cycles = 3`
- `~timeout_margin = 2.0`

## 7. 启动编排设计

建议新增一个统一入口 launch，而不是继续复用旧的 `moveit_planning_execution.launch`。

原因：

- 现有 `moveit_planning_execution.launch` 中的 `sim` 参数没有真正完成仿真/实机分流。
- 该 launch 同时包含旧链与新链逻辑，不适合作为统一入口继续扩展。

### 7.1 新入口 launch 需要包含的组件

1. MoveIt 配置
   - 关节名配置
   - robot_description / SRDF / kinematics / joint_limits
   - move_group
   - RViz

2. 实机执行侧
   - `aubo_joint_trajectory_action`
   - `aubo_robot_simulator`（插补桥）
   - `aubo_driver`
   - `robot_state_publisher`

3. Gazebo 镜像侧
   - `aubo_e5_gazebo_control.launch shadow:=true`
   - `aubo_gazebo_driver`

4. 新增联动层
   - `joint_state_mirror_adapter`
   - `linked_execution_monitor`
   - `linked_execution_action_server`

### 7.2 MoveIt 控制器配置建议

建议为联动模式单独准备一份 MoveIt controller 配置，而不要直接覆盖旧的 `controllers.yaml`。

原因：

- 旧链可能仍需要直接使用 `aubo_e5_controller`。
- 联动链的新默认 controller 应指向 `linked_execution_controller`。
- 二者最好并存，避免影响原有调试入口。

## 8. 方案可行性分析

### 8.1 为什么可行

该方案具有较高可行性，原因如下：

1. 实机执行链已存在。
2. Gazebo shadow 镜像链已存在。
3. 缺失的只是“状态标准化”和“结果聚合”，而不是底层驱动能力。
4. 不需要推翻现有 AUBO SDK 接入方式。
5. 不需要改动路径规划算法和 MoveIt 规划逻辑。

### 8.2 与“双边同时执行”相比的优势

不推荐让 MoveIt 同时控制：

- 实机链
- Gazebo 的 `JointTrajectoryController`

原因：

- 两边控制器模型不同。
- 时间基准不同。
- 插补方式不同。
- 成功判定机制不同。
- 很容易造成“两边都在动，但并非同一动作”的假象。

而采用“实机执行 + Gazebo 镜像”则能保证：

- 控制权只有一份。
- Gazebo 只跟随，不会争抢执行权。
- 当 Gazebo 出现问题时，不会影响真实机器人动作。

### 8.3 当前代码中的直接风险点

#### 风险 1：`aubo_gazebo_driver` shadow 输入顺序依赖

- 现状：按数组下标拷贝，不按名字映射。
- 影响：关节顺序一旦变化，Gazebo 会错位。
- 解决：必须加 `joint_state_mirror_adapter`。

#### 风险 2：`moveit_planning_execution.launch` 逻辑不适合扩展

- 现状：仿真与实机分流不清晰。
- 影响：继续堆逻辑容易造成链路混乱。
- 解决：新建独立统一入口 launch。

#### 风险 3：Gazebo 成功缺少可等待接口

- 现状：没有节点告诉上层“Gazebo 已经镜像到位”。
- 影响：无法实现“双通道都要成功”的语义。
- 解决：新增 `linked_execution_monitor` + `linked_execution_action_server`。

#### 风险 4：Gazebo 镜像频率低于实机内部控制频率

- 现状：Gazebo 镜像侧更偏显示，不是严格实时共执行。
- 影响：中间过程可能存在显示延迟。
- 解决：接受“最终收敛一致”，不追求逐采样严格同步。

## 9. 稳定性分析

### 9.1 稳定性优势

该设计在工程上比较稳定，原因是：

- 实机是唯一控制主体，链路清晰。
- Gazebo 不参与反向控制，不会把仿真误差反馈给实机。
- 旧的底层驱动逻辑可保留，风险集中在新增联动层。
- `aubo_gazebo_driver` 已经具备 Gazebo 时间跳变保护能力。

### 9.2 稳定性边界

该方案追求的是：

- **实机真实执行**
- **Gazebo 可信呈现**
- **最终状态一致**

它不追求：

- Gazebo 与实机逐采样严格一致
- Gazebo 与实机作为两个真正等价执行器并行工作

换句话说，这是一条“联动显示链”，不是“双执行器并联控制链”。

### 9.3 预期最稳定的运行方式

最稳定的使用方式是：

- MoveIt 只向联动 Action 发轨迹。
- 联动 Action 只向现有实机 Action 发轨迹。
- Gazebo 只监听经过标准化后的实机状态。
- 最终成功以“实机完成 + Gazebo 收敛”为准。

## 10. 建议的实施顺序

虽然本次文档不直接展开代码实现，但建议后续真正实现时按以下顺序推进：

1. 实现 `joint_state_mirror_adapter`
   - 先解决 Gazebo shadow 输入不稳定的问题。

2. 实现 `linked_execution_monitor`
   - 先让系统具备“知道 Gazebo 是否跟上”的能力。

3. 实现 `linked_execution_action_server`
   - 把实机 Action 和镜像结果聚合起来。

4. 新建统一 launch
   - 把实机链、Gazebo 链、联动层整合到一起。

5. 单独准备联动版 MoveIt controller 配置
   - 不覆盖旧配置，降低兼容风险。

## 11. 验证与测试场景

后续实现应至少验证以下场景：

1. 基本联动
   - RViz 下发单段关节轨迹。
   - 实机成功运动。
   - Gazebo 正确镜像。
   - 最终联动 Action 返回成功。

2. 多段轨迹
   - 多个 waypoint 的轨迹能够执行。
   - Gazebo 最终姿态与实机一致。

3. Gazebo 未启动
   - 实机可能已执行，但联动结果必须返回失败。
   - 失败原因应明确。

4. Gazebo joint_states 不更新
   - 联动监视超时并失败。

5. 实机急停 / 保护停 / 网络断连
   - 底层实机失败。
   - 联动结果也必须失败。

6. Gazebo pause / reset / resume
   - 镜像不应出现暴走。
   - 收敛判定不应误报成功。

7. 关节顺序被打乱
   - `joint_state_mirror_adapter` 能恢复标准顺序。

## 12. 结论

基于当前仓库的文件依赖关系、节点功能和话题行为，最可行、最稳定的方案不是“让 RViz 同时直接控制实机和 Gazebo 两套执行器”，而是：

- **MoveIt 只驱动实机主链**
- **Gazebo 只镜像实机结果**
- **新增联动聚合层，把实机成功和 Gazebo 收敛统一成一个上层结果**

这是一个面向实际工程的设计方案，具备如下优点：

- 可复用现有代码最多。
- 对现有底层驱动侵入最小。
- 逻辑边界清晰。
- 稳定性明显优于双边同时执行方案。

如果后续需要继续推进实现，建议以本设计文档为基础，优先落地”状态标准化”和”结果聚合”两个新增组件，再做统一 launch 和 MoveIt controller 切换。

---

## 13. 实现状态（2026-03-30 更新）

### 13.1 已完成

所有设计组件均已实现，代码位于 `src/aubo_linked_execution/` 包中：

| 设计组件 | 实现文件 | 状态 |
|---------|---------|------|
| joint_state_mirror_adapter | scripts/joint_state_mirror_adapter.py | ✓ 已实现 |
| linked_execution_monitor | scripts/linked_execution_monitor.py | ✓ 已实现 |
| linked_execution_action_server | scripts/linked_execution_action_server.py | ✓ 已实现 |
| 统一入口 launch | launch/aubo_e5_linked_execution.launch | ✓ 已实现 |
| MoveIt 控制器配置 | config/linked_execution_controllers.yaml | ✓ 已实现 |
| 实机自动上电 | scripts/aubo_robot_startup.py | ✓ 已实现（设计文档外新增） |

### 13.2 实际实现与设计的偏差

1. **mirror 话题名**
   - 设计：`/real/joint_states_mirror`
   - 实际：`/real/joint_states`（与 aubo_gazebo_driver 的默认 shadow 输入一致）

2. **feedback 隔离方式**
   - 设计：未涉及
   - 实际：联动模式下用 `<remap>` 将 aubo_robot_simulator 的 feedback_states/joint_states 重映射到 sim/ 命名空间，防止模拟器反馈混线导致 Execute 虚假成功

3. **sim_only 模式**
   - 设计：未涉及
   - 实际：launch 支持 `sim_only:=true`，切换为标准 aubo_e5 控制器 + Gazebo 普通模式，无需实机

4. **robot_name 参数**
   - 设计：约定 `/robot_name = aubo_e5`
   - 实际发现：Gazebo launch 设置 `/robot_name=”/aubo_e5”`（带前导/）会覆盖，需在 Gazebo include 之后重新设置

5. **实机上电**
   - 设计：未涉及
   - 实际：aubo_driver 不自动上电，新增 aubo_robot_startup.py 完成 powerOn + 控制模式切换

### 13.3 已知限制

- 实机上电依赖硬件前置条件（急停释放、无报警）
- use_sim_time=true 下所有节点使用 Gazebo 仿真时钟，需保证 Gazebo RTF ≈ 1.0
- aubo_driver 在 RosMoveIt 模式下不持续发布 /aubo_driver/real_pose，长期空闲后模拟器内部状态可能与实机偏移（不影响 feedback 判定，因联动模式已隔离）

---

## 相关文档

- [系统架构](ARCHITECTURE.md) - 完整系统设计
- [Unity 迁移](UNITY_MIGRATION.md) - Unity 后端集成
- [数据流分析](Unity_Migration_Data_Flow_Analysis.md) - 架构对比分析
- [故障排查](TROUBLESHOOTING.md) - 常见问题解决
