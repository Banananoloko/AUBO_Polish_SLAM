# P6 Shadow 模式话题映射修复

## 问题描述

在 P6 Shadow 模式下，MoveIt 规划失败，出现以下错误：

```
[ WARN] [1735574800.123456789]: Controller is taking too long to execute trajectory (the expected upper bound for the trajectory execution was 10.000000 seconds). Stopping trajectory.
```

## 根本原因

### 1. `/joint_states` 话题缺失

- **问题**: MoveIt 默认订阅 `/joint_states` 话题来获取机器人当前状态
- **现状**: 只有 `/aubo_e5/joint_states` 话题存在
- **原因**: `robot_state_publisher` 使用了 remap，将其订阅的 `joint_states` 重映射到 `/aubo_e5/joint_states`

```xml
<!-- 原有配置 -->
<node pkg="robot_state_publisher" type="robot_state_publisher"
      name="robot_state_publisher" output="screen">
    <remap from="joint_states" to="/aubo_e5/joint_states"/>
</node>
```

这导致：
- `robot_state_publisher` 正确订阅了 `/aubo_e5/joint_states`
- 但 MoveIt 期望的全局 `/joint_states` 话题不存在
- MoveIt 无法获取机器人当前状态，导致规划失败

### 2. 话题流向分析

```
Unity (物理引擎)
    ↓ 发布 /unity/joint_states
unity_joint_states_publisher.py
    ↓ 转发并重排序
/aubo_e5/joint_states
    ↓ robot_state_publisher 订阅（用于 TF）
    ↓ aubo_robot_simulator 订阅（用于 feedback）
    ✗ MoveIt 无法订阅（期望 /joint_states）
```

## 解决方案

### 添加话题重映射节点

在 `unity_bridge.launch` 中添加 `topic_tools relay` 节点，将 `/aubo_e5/joint_states` 重新发布到 `/joint_states`：

```xml
<!-- 2.1 话题重映射：/aubo_e5/joint_states -> /joint_states (MoveIt 需要) -->
<node pkg="topic_tools" type="relay" name="joint_states_relay"
      args="/aubo_e5/joint_states /joint_states" output="screen"/>
```

### 修复后的话题流向

```
Unity (物理引擎)
    ↓ 发布 /unity/joint_states
unity_joint_states_publisher.py
    ↓ 转发并重排序
/aubo_e5/joint_states
    ↓ robot_state_publisher 订阅（用于 TF）
    ↓ aubo_robot_simulator 订阅（用于 feedback）
    ↓ joint_states_relay 订阅
    ↓ 重新发布
/joint_states
    ↓ MoveIt 订阅 ✓
```

## 验证方法

### 1. 使用测试脚本

```bash
./test_p6_shadow_topics.sh
```

测试脚本会检查：
- `/joint_states` 话题是否存在
- `/aubo_e5/joint_states` 话题是否存在
- `/feedback_states` 话题是否存在
- `/robot_status` 话题是否存在
- 所有关键节点是否正常运行

### 2. 手动验证

```bash
# 1. 启动 Unity Bridge (Shadow 模式)
roslaunch aubo_unity_bridge unity_bridge.launch shadow:=true

# 2. 检查话题列表
rostopic list | grep joint_states
# 应该看到：
#   /joint_states
#   /aubo_e5/joint_states
#   /unity/joint_states

# 3. 检查话题频率
rostopic hz /joint_states
# 应该看到约 50 Hz

# 4. 检查话题内容
rostopic echo /joint_states -n 1
# 应该看到 6 个关节的位置数据

# 5. 启动 MoveIt 并测试规划
roslaunch aubo_e5_moveit_config move_group.launch
roslaunch aubo_e5_moveit_config moveit_rviz.launch
```

## 相关文件

### 修改的文件
- `src/aubo_unity_bridge/launch/unity_bridge.launch` - 添加 joint_states_relay 节点

### 新增的文件
- `test_p6_shadow_topics.sh` - 话题映射测试脚本
- `src/aubo_unity_bridge/docs/P6_Shadow_Mode_Fix.md` - 本文档

### 相关文件（未修改）
- `src/aubo_unity_bridge/scripts/unity_joint_states_publisher.py` - Unity 关节状态转发
- `src/aubo_robot/aubo_controller/script/aubo_controller/aubo_robot_simulator` - 模拟器（发布 feedback_states）

## 技术细节

### topic_tools relay 节点

`topic_tools relay` 是 ROS 提供的标准工具，用于话题重映射：

```bash
rosrun topic_tools relay <input_topic> <output_topic>
```

特点：
- 零延迟转发（直接回调）
- 保持消息类型不变
- 不修改消息内容
- 轻量级（无额外处理）

### 为什么不直接修改 robot_state_publisher 的 remap？

如果移除 `robot_state_publisher` 的 remap：

```xml
<!-- 不推荐 -->
<node pkg="robot_state_publisher" type="robot_state_publisher"
      name="robot_state_publisher" output="screen">
    <!-- 移除 remap，直接订阅 /joint_states -->
</node>
```

会导致：
1. 需要修改 `unity_joint_states_publisher.py` 的输出话题为 `/joint_states`
2. 破坏与 Gazebo 模式的一致性（Gazebo 也使用 `/aubo_e5/joint_states`）
3. 需要修改 `aubo_robot_simulator` 的订阅话题
4. 影响其他依赖 `/aubo_e5/joint_states` 的节点

使用 `relay` 节点的优势：
- 最小化修改（只添加一个节点）
- 保持与 Gazebo 模式的一致性
- 不影响现有节点的订阅关系
- 易于理解和维护

## 对比：Gazebo 模式 vs Unity 模式

### Gazebo 模式
```
Gazebo (物理引擎)
    ↓ 发布 /aubo_e5/joint_states (通过 joint_state_controller)
robot_state_publisher 订阅（remap）
aubo_robot_simulator 订阅
    ↓ 发布 /feedback_states
MoveIt 订阅 ??? (需要 /joint_states)
```

**问题**: Gazebo 模式也存在同样的问题！

### Unity 模式（修复后）
```
Unity (物理引擎)
    ↓ 发布 /unity/joint_states
unity_joint_states_publisher.py
    ↓ 发布 /aubo_e5/joint_states
joint_states_relay
    ↓ 发布 /joint_states
MoveIt 订阅 ✓
```

## 后续优化建议

1. **统一话题命名**：考虑在所有模式（Gazebo/Unity/实机）中统一使用 `/joint_states` 作为主话题
2. **添加话题监控**：在 launch 文件中添加话题存在性检查
3. **完善文档**：更新 MoveIt 配置文档，说明话题订阅关系
4. **测试覆盖**：添加自动化测试，验证话题映射的正确性

## 参考资料

- [ROS topic_tools 文档](http://wiki.ros.org/topic_tools)
- [MoveIt 配置指南](http://docs.ros.org/en/noetic/api/moveit_tutorials/html/index.html)
- `src/aubo_unity_bridge/docs/Migration_Plan.md` - Unity 迁移计划
- `src/RViz_Real_Gazebo_Linked_Execution_Design.md` - 联动设计文档

## 修复日期

2026/04/30

## 修复人员

Claude Sonnet 4.6 (1M context)
