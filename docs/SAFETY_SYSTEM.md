# AUBO E5 安全系统说明

## 概述

本文档说明 AUBO E5 实机联动系统的安全保护机制，包括启动位置对齐验证、运行时轨迹安全监控、看门狗机制、手动移动检测、Gazebo RTF 监控和执行阻断功能。

## 安全模块

### 1. 启动位置对齐验证 (aubo_robot_startup.py)

**功能**: 在实机上电并切换到 ROS 控制模式后，验证 RViz 显示的机器人位置与实机实际位置对齐。

**关键方法**: 
- `verify_initial_position_sync()`: 验证位置稳定性
- `verify_rviz_alignment()`: 验证 RViz 与实机位置对齐

**验证流程**:
1. 等待 `/joint_states` 话题发布（超时 10 秒）
2. 检查关节状态稳定性（两次读取间隔 1 秒，误差 < 0.01 rad）
3. 显示当前各关节位置供用户确认
4. **使用 MoveIt Commander 获取 RViz 当前状态**
5. **比较 RViz 状态与实机状态，容差 0.05 rad**
6. 对齐验证通过后继续启动流程

**为什么需要**:
- `aubo_driver` 在 `controller_connected_flag_=false` 时会发布 `target_point_` 而非 `current_joints_`
- 这会导致 RViz 显示错误位置，执行轨迹时可能产生大幅度危险运动
- 启动验证确保 RViz 基于实机反馈显示正确位置

**日志示例**:
```
[startup] Verifying initial position synchronization...
[startup] Current joint positions (rad):
[startup]   shoulder: 0.0000
[startup]   upperArm: -1.5708
[startup]   foreArm: 1.5708
[startup]   wrist1: 0.0000
[startup]   wrist2: 0.0000
[startup]   wrist3: 0.0000
[startup] ✓ Initial position sync verified
[startup] Verifying RViz position alignment with real robot...
[startup] Position alignment check:
[startup]   Max difference: 0.0023 rad (0.1°) at shoulder
[startup]   Tolerance: 0.0500 rad (2.9°)
[startup] ✓ RViz position aligned with real robot
```

**参数配置**:
```xml
<node pkg="aubo_linked_execution" type="aubo_robot_startup.py" name="aubo_robot_startup">
  <param name="startup_timeout" value="30.0"/>
  <param name="position_alignment_tolerance" value="0.05"/>  <!-- 对齐容差 (rad) -->
</node>
```

### 2. 运行时安全监控 (safety_monitor.py)

**功能**: 实时监控规划和执行的轨迹，检测潜在危险运动并发出警告或阻断执行。

**监控项**:

#### 2.1 大幅度运动警告
- **阈值**: 0.5 rad (约 28.6°)
- **触发**: 当前位置与规划轨迹起点的最大关节差值超过阈值
- **动作**: 发布警告到 `/safety_monitor/warning`，设置 `safe_to_execute=False`
- **监听话题**: `/move_group/display_planned_path`

#### 2.2 手动移动检测
- **阈值**: 0.05 rad (约 2.9°)
- **触发**: 规划后、执行前机器人被手动移动
- **动作**: 发布错误日志，阻断执行 (`safe_to_execute=False`)
- **实现**: 在规划时记录位置，执行前比较当前位置

#### 2.3 轨迹起点验证
- **容差**: 0.15 rad (约 8.6°)
- **触发**: 即将执行的轨迹起点与当前位置不匹配
- **动作**: 发布错误日志，阻断执行 (`safe_to_execute=False`)
- **监听话题**: `/joint_path_command`

#### 2.4 碰撞检测状态监控
- **功能**: 监控 MoveIt Planning Scene 的碰撞检测状态
- **监听话题**: `/move_group/monitored_planning_scene` (预留接口)

**发布话题**:
- `/safety_monitor/warning` (std_msgs/String): 警告信息
- `/safety_monitor/safe_to_execute` (std_msgs/Bool): 执行安全标志

**参数配置**:
```xml
<node pkg="aubo_linked_execution" type="safety_monitor.py" name="safety_monitor">
  <param name="large_motion_threshold" value="0.5"/>          <!-- 大幅度运动阈值 (rad) -->
  <param name="trajectory_start_tolerance" value="0.15"/>     <!-- 轨迹起点容差 (rad) -->
  <param name="manual_movement_threshold" value="0.05"/>      <!-- 手动移动阈值 (rad) -->
</node>
```

### 3. 安全监控看门狗 (linked_execution_action_server.py)

**功能**: 监控 safety_monitor 节点健康状态，防止单点故障。

**实现机制**:
- 订阅 `/safety_monitor/safe_to_execute` 并记录消息时间戳
- 定时器每秒检查是否超过 5 秒未收到消息
- 超时后自动设置 `safe_to_execute=False` 并记录错误

**看门狗逻辑**:
```python
def _watchdog_callback(self, event):
    elapsed = (rospy.Time.now() - self._last_safety_msg_time).to_sec()
    if elapsed > self._safety_watchdog_timeout:
        rospy.logerr('[linked_execution] Safety monitor watchdog timeout!')
        self._safe_to_execute = False
```

**参数配置**:
```xml
<node pkg="aubo_linked_execution" type="linked_execution_action_server.py">
  <param name="safety_watchdog_timeout" value="5.0"/>  <!-- 看门狗超时 (秒) -->
</node>
```

### 4. Gazebo RTF 监控 (gazebo_rtf_monitor.py)

**功能**: 监控 Gazebo 实时因子 (Real-Time Factor)，检测仿真性能异常。

**监控机制**:
- 订阅 `/clock` 话题获取仿真时间
- 计算 RTF = (仿真时间增量) / (真实时间增量)
- RTF 超出范围 [0.8, 1.2] 时发出警告
- 检测 `/clock` 停止发布（Gazebo 崩溃）

**为什么需要**:
- RTF < 0.8: Gazebo 运行缓慢，可能导致实机与仿真不同步
- RTF > 1.2: Gazebo 运行过快（异常情况）
- /clock 停止: Gazebo 崩溃或暂停过久

**发布话题**:
- `/gazebo_rtf_monitor/warning` (std_msgs/String): RTF 异常警告

**参数配置**:
```xml
<node pkg="aubo_linked_execution" type="gazebo_rtf_monitor.py" name="gazebo_rtf_monitor">
  <param name="rtf_threshold_low" value="0.8"/>      <!-- RTF 下限 -->
  <param name="rtf_threshold_high" value="1.2"/>     <!-- RTF 上限 -->
  <param name="check_interval" value="2.0"/>         <!-- 检查间隔 (秒) -->
</node>
```

### 5. 执行阻断机制 (linked_execution_action_server.py)

**功能**: 在聚合层 Action Server 中集成安全监控信号，阻断不安全的轨迹执行。

**实现**:
```python
def _execute_cb(self, goal):
    # 0. Check safety monitor status
    if not self._safe_to_execute:
        rospy.logerr('LinkedExecutionActionServer: execution blocked by safety monitor')
        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
        self._server.set_aborted(result, 'Execution blocked by safety monitor')
        return
    # ... 继续执行
```

**订阅话题**: `/safety_monitor/safe_to_execute`

**阻断条件**:
- 大幅度运动检测触发
- 手动移动检测触发
- 轨迹起点验证失败
- Safety monitor 看门狗超时
- 碰撞检测报警（未来扩展）

## 安全系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                        用户操作层                              │
│                    (RViz + MoveIt)                           │
└────────────────────────┬────────────────────────────────────┘
                         │ Plan & Execute
                         ↓
┌─────────────────────────────────────────────────────────────┐
│                    安全监控层                                  │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ safety_monitor.py                                    │   │
│  │  • 大幅度运动检测 (0.5 rad)                            │   │
│  │  • 手动移动检测 (0.05 rad)                             │   │
│  │  • 轨迹起点验证 (0.15 rad)                             │   │
│  │  → /safety_monitor/safe_to_execute                   │   │
│  └──────────────────────────────────────────────────────┘   │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ gazebo_rtf_monitor.py                                │   │
│  │  • RTF 监控 [0.8, 1.2]                               │   │
│  │  • /clock 停止检测                                    │   │
│  │  → /gazebo_rtf_monitor/warning                       │   │
│  └──────────────────────────────────────────────────────┘   │
└────────────────────────┬────────────────────────────────────┘
                         │ safe_to_execute
                         ↓
┌─────────────────────────────────────────────────────────────┐
│                    执行聚合层                                  │
│  ┌──────────────────────────────────────────────────────┐   │
│  │ linked_execution_action_server.py                    │   │
│  │  • 安全检查 (safe_to_execute)                         │   │
│  │  • 看门狗机制 (5s 超时)                                │   │
│  │  • 双重成功验证 (实机 + Gazebo)                        │   │
│  └──────────────────────────────────────────────────────┘   │
└────────────────────────┬────────────────────────────────────┘
                         │ Forward trajectory
                         ↓
┌─────────────────────────────────────────────────────────────┐
│                    执行层                                      │
│  ┌─────────────────┐              ┌─────────────────┐        │
│  │  aubo_driver    │              │ Gazebo (shadow) │        │
│  │  (实机控制)      │◄─────────────┤  (镜像跟踪)      │        │
│  └─────────────────┘  /real/      └─────────────────┘        │
│                       joint_states                           │
└─────────────────────────────────────────────────────────────┘
```

## 安全保护层级

### 第 1 层：启动层
- **aubo_robot_startup.py**: 位置稳定性检查 + RViz 对齐验证
- **容差**: 0.05 rad (2.9°)
- **失败处理**: 拒绝启动，记录详细差异

### 第 2 层：规划层
- **MoveIt**: 路径规划 + 碰撞检测
- **OMPL/CHOMP**: 22 种算法 + 梯度优化
- **Planning Scene**: 自碰撞 + 环境碰撞

### 第 3 层：监控层
- **safety_monitor.py**: 
  - 大幅度运动检测 (0.5 rad)
  - 手动移动检测 (0.05 rad)
  - 轨迹起点验证 (0.15 rad)
- **gazebo_rtf_monitor.py**: RTF 异常检测

### 第 4 层：执行层
- **linked_execution_action_server.py**: 
  - 安全阻断机制
  - 看门狗保护 (5s)
  - 双重成功验证

### 第 5 层：硬件层
- **急停按钮**: 物理紧急停止
- **关节限位**: ±3.05 rad 硬件限制
- **示教器**: 手动控制和状态监控

## 启动流程

联动模式启动时，安全系统自动激活：

```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.1.10
```

**启动顺序**:
1. MoveIt + RViz 启动
2. Gazebo 启动（shadow 模式）
3. `aubo_driver` 连接实机
4. `aubo_robot_startup` 上电 + 切换控制模式 + **位置同步验证**
5. `safety_monitor` 开始监控
6. `linked_execution_action_server` 就绪（集成安全检查）
