# AUBO E5 数据链完整文档

## 从终端输入路径点到实机执行反馈的全链路数据流

---

## 1. 系统架构总览

```
终端输入 (square_demo_control.py)
  │
  ├─ 坐标系转换 (示教器系 ←→ URDF world 系, ΔZ=0.503m)
  │
  ├─ MoveIt 规划层 (OMPL + ITP, joint_limits.yaml 限幅)
  │
  ├─ 联动执行层 (linked_execution_action_server.py)
  │
  ├─ C++ Action 层 (joint_trajectory_action.cpp)
  │
  ├─ 轨迹插补桥 (aubo_robot_simulator, 200Hz)
  │     │
  │     └─ velocity_scale_factor 缩放 (scale_trajectory_speed)
  │
  ├─ 驱动队列层 (aubo_driver, 500Hz 消费)
  │     │
  │     └─ tryPopWaypoint() 速度检查 + 插补
  │
  ├─ CAN 总线 → 实机控制器 (robotServiceSetRobotPosData2Canbus)
  │
  └─ 反馈链 (robot_status, feedback_states, joint_states)
```

---

## 2. 坐标系

### 2.1 两个坐标系

| 坐标系 | 基准点 | Z 偏移 | 使用者 |
|--------|--------|--------|--------|
| **示教器系 (base_link 系)** | 机器人安装面 | Z=0 | 示教器、用户输入、显示 |
| **URDF world 系** | 地面 (pedestal 底面) | Z=+0.503m | MoveIt、ROS、内部计算 |

### 2.2 偏移来源

URDF 中 `pedestal_joint` 定义了从 world → base_link 的固定偏移:

```xml
<joint name="pedestal_joint" type="fixed">
  <parent link="pedestal_Link"/>
  <child link="base_link"/>
  <origin xyz="0.0 0.0 0.503" rpy="0.0 0.0 0.0"/>
</joint>
```

肩关节在 world 系中的 Z = 0.503 (pedestal) + 0.0495 (base_link→shoulder) = **0.5525m**

### 2.3 转换函数 (square_demo_control.py)

```python
PEDESTAL_Z = 0.503

def to_world(tx, ty, tz):
    """示教器系 → world 系"""
    return (tx, ty, tz + PEDESTAL_Z)

def to_teach(wx, wy, wz):
    """world 系 → 示教器系"""
    return (wx, wy, wz - PEDESTAL_Z)
```

所有用户 I/O 使用示教器系，与 MoveIt 交互前通过 `to_world()` 转换，显示前通过 `to_teach()` 转换。

---

## 3. 完整数据流 (逐层)

### 第 1 层：用户输入 → 目标位姿

**文件**: `square_demo_control.py:run_square_trajectory()` / `run_custom_waypoint()`

- 正方形角点 `SQUARE_CORNERS` 使用示教器系坐标
- 用户自定义输入也使用示教器系坐标
- 调用 `to_world()` 转换为 world 系后传入 `execute_pose_target()`

```
输入:  (0.4, 0.10, 0.70)  示教器系
转换:  (0.4, 0.10, 1.203)  world 系
```

### 第 2 层：MoveIt 轨迹规划

**文件**: `square_demo_control.py:execute_pose_target()`

**重要**: 使用 `set_pose_target()` 而非 `set_position_target()`。

```python
# 获取当前末端方向，约束 IK 避免关节绕转
current_pose = self.get_current_ee_pose()
target_pose = geometry_msgs.msg.Pose()
target_pose.position.x = wx  # world 系
target_pose.position.y = wy
target_pose.position.z = wz
target_pose.orientation = current_pose.orientation  # 保持方向不变

self.group.set_pose_target(target_pose)
plan_ok = self.group.go(wait=True)
```

**为什么用 `set_pose_target` 而非 `set_position_target`**：
`set_position_target` 仅约束位置 (3 DOF)，IK 可自由选择末端方向。在横向移动时 (如 Y:-0.10→0.10)，IK 可能选择绕转 ±2π 的关节解 (如 wrist3 从 +3.0 rad 跳到 -3.0 rad)，导致在 `tryPopWaypoint` 中计算 `Δpos ≈ 6.28 rad` → `velocity = 6.28/0.005 = 1256 rad/s`。

`set_pose_target` 同时约束位置+方向 (6 DOF)，IK 保持在当前解分支上，杜绝绕转。

#### 2a. MoveIt 内部流程

```
set_pose_target(pose)
  │
  ├─ IK 求解 (使用当前关节角为种子，约束位置+方向)
  │
  ├─ OMPL 路径规划 (关节空间，位置路点)
  │
  ├─ ITP 时间参数化 (Iterative Time Parameterization)
  │     为每个路点分配时间戳、速度、加速度
  │     速度上限: joint_limits.yaml (1.0/1.25 rad/s ≈ 40% HW)
  │     加速度上限: joint_limits.yaml (7.0/8.3 rad/s² ≈ 40% HW)
  │
  └─ MoveIt Velocity Scaling: × VELOCITY_SCALING (0.5)
        实际 ITP 速度: 1.0 × 0.5 = 0.5 rad/s
```

#### 2b. 关节限幅配置

**文件**: `joint_limits.yaml`

| 关节 | 硬件上限 | 配置值 | 百分比 |
|------|---------|--------|--------|
| shoulder / upperArm / foreArm 速度 | 2.596 rad/s | **1.0 rad/s** | ~38% |
| wrist1 / wrist2 / wrist3 速度 | 3.110 rad/s | **1.25 rad/s** | ~40% |
| shoulder / upperArm / foreArm 加速度 | 17.309 rad/s² | **7.0 rad/s²** | ~40% |
| wrist1 / wrist2 / wrist3 加速度 | 20.737 rad/s² | **8.3 rad/s²** | ~40% |

### 第 3 层：联动执行 Action Server

**文件**: `linked_execution_action_server.py`

```
MoveIt 发出 FollowJointTrajectoryGoal
  │
  ├─ 0. 安全检查: /safety_monitor/safe_to_execute == True?
  │      watchdog 超时 5s → 阻塞执行
  │
  ├─ 1. 转发目标到实机 Action: aubo_e5_controller/follow_joint_trajectory
  │
  ├─ 2. 发布轨迹终点到 Gazebo 监控: /linked_execution/monitor_goal
  │
  ├─ 3. 等待实机 Action 返回 (timeout = trajectory_duration + 10s)
  │
  └─ 4. 等待 Gazebo 收敛 (timeout = trajectory_duration + 8s)
```

### 第 4 层：C++ Joint Trajectory Action

**文件**: `joint_trajectory_action.cpp`

```
接收 FollowJointTrajectoryGoal
  │
  ├─ goalCB():
  │     pub_trajectory_command_.publish(current_traj_)
  │     → 发布为 joint_path_command (JointTrajectory)
  │
  └─ controllerStateCB():
        withinGoalConstraints()  检查关节误差 < goal_threshold_ (0.04 rad)
        robot_status.in_motion == FALSE  确认机器人停稳
        二者同时满足 → setSucceeded()

goal_threshold_ = rosparam("~constraints/goal_threshold", 0.04)
→ per-joint 误差容限 = 0.04/2 = 0.02 rad (≈ 1.15°)  // isWithinRange 用 half_range
```

### 第 5 层：轨迹插补桥 (aubo_robot_simulator)

**文件**: `aubo_robot_simulator` (Python)

**角色**: 将 `joint_path_command` (JointTrajectory) 插补为 `moveItController_cmd` (JointTrajectoryPoint 流)

```
trajectory_callback(msg_in: JointTrajectory)
  │
  ├─ 读取 velocity_scale_factor = rospy.get_param('/aubo_controller/velocity_scale_factor', 1.0)
  │
  ├─ scale_trajectory_speed(msg_in, scale)
  │     time_from_start /= scale   → 时间扩展 (scale=0.5 → 2× 时间)
  │     velocities      *= scale   → 速度缩放 (scale=0.5 → 50%)
  │     accelerations   *= scale²  → 加速度缩放 (scale=0.5 → 25%)
  │
  ├─ _to_controller_order()  重排关节顺序以匹配 controller_joint_names
  │
  └─ _motion_worker()  线性插补 @ 200Hz (motion_update_rate)
        位置: last + α×(current - last)
        速度: last_vel + α×(current_vel - last_vel)  使用轨迹自带速度
        加速度: 恒为 0
        ↓
        moveItController_cmd (JointTrajectoryPoint, 200Hz)
```

**velocity_scale_factor 的影响**:

| scale | time_from_start | 速度值 | 插补点数 | Δpos/5ms | driver 计算速度 |
|-------|----------------|--------|---------|-----------|----------------|
| 1.0 | 1× | 100% | N | Δpos | V |
| 0.5 | 2× | 50% | 2N | Δpos/2 | V/2 |

### 第 6 层：驱动接收与排队 (aubo_driver)

**文件**: `aubo_driver.cpp`

```
moveItPosCallback(msg: JointTrajectoryPoint)  ← moveItController_cmd
  │
  ├─ 检测新轨迹: start_move_==false && buf_queue_.empty()
  │    → need_sync_filter_ = true  (触发 joint_filter_ 同步)
  │
  ├─ roadPointCompare()  过滤与上一点相同的位置 (避免重复)
  │
  └─ 推入 buf_queue_<PlanningState>
       joint_pos_ = msg.positions
       joint_vel_ = msg.velocities   ← 存储但后续未使用!
       joint_acc_ = msg.accelerations

updateControlStatus()  ← 由 main loop 每 2ms (500Hz) 调用
  │
  └─ start_move_ && rib_buffer_size_ < MINIMUM_BUFFER_SIZE (300)
       → setRobotJointsByMoveIt()
           从 buf_queue_ 弹出 → 仅将 joint_pos_ 推入 ros_motion_queue_
           速度信息在此处丢弃!
```

**关键**: `ros_motion_queue_` 只传递位置，MoveIt 精心计算的速度被丢弃。

### 第 7 层：速度检查与插补 (tryPopWaypoint)

**文件**: `aubo_driver.cpp:tryPopWaypoint()`

```
从 ros_motion_queue_ 弹出 cnt 个路点
  │
  ├─ 速度检查: target_joint_velc_[i] = fabs(joint[i] - joint_filter_[i]) / 0.005
  │    (hardcoded 5ms 除数 = 模拟器 200Hz 插补周期)
  │
  ├─ 超过 MaxVelc[i]?
  │    MaxVelc = {2.596, 2.596, 2.596, 3.110, 3.110, 3.110} rad/s (100% HW)
  │    → 插补 n_equalpart = ceil(max_ratio) + 1 个中间点
  │
  ├─ 加速度检查: joint_acc_[i] = fabs(Δvelocity) / 0.005
  │    MaxAcc = {17.309, 17.309, 17.309, 20.737, 20.737, 20.737} rad/s² (100% HW)
  │    仅日志警告，不阻断、不插补!
  │
  └─ 结果: wayPointVector (仅位置) → robotServiceSetRobotPosData2Canbus()
```

**时序不匹配问题**:

模拟器按 5ms 间隔产生路点，但实机控制器内部执行周期约为 2ms (500Hz)。
实机实际速度 = 驱动计算速度 × (5ms / 2ms) = **驱动计算速度 × 2.5**

这是速度超限的根源之一 — 驱动以 100% 硬件上限检查通过，但实机看到 2.5× 更高的速度。

### 第 8 层：机器人内部控制器

```
robotServiceSetRobotPosData2Canbus(wayPointVector)
  │
  ├─ CAN 总线 → 机器人控制板缓冲区
  │    缓冲区大小: macTargetPosDataSize
  │    驱动监控此值，只在有空间时发送
  │
  └─ 机器人内部控制循环 (~2ms)
       按自己的周期插补位置路点
       检查关节速度是否超出安全限幅 (示教器可配置)
       检查 TCP 速度是否超出安全限幅
       超限 → 保护性停止 (示教器显示 "目标速度超出限幅")
```

### 第 9 层：反馈链

```
实机关节编码器
  │
  ├─ aubo_driver::timerCallback()  @ 20Hz (TIMER_SPAN_=50)
  │     robotServiceGetCurrentWaypointInfo() → current_joints_
  │     发布:
  │       /joint_states (sensor_msgs/JointState)
  │       /feedback_states (control_msgs/FollowJointTrajectoryFeedback)
  │       /robot_status (RobotStatus) ← in_motion = start_move_
  │
  ├─ C++ Action: controllerStateCB() 订阅 feedback_states
  │     → withinGoalConstraints() 判断是否到达目标
  │
  ├─ joint_state_mirror_adapter.py:
  │     /real/joint_states → Gazebo shadow (同步虚实)
  │
  └─ linked_execution_monitor.py:
       /linked_execution/monitor_status → linked_execution_action_server
```

---

## 4. 速度/加速度限幅层级汇总

| 层 | 位置 | 参数 | 当前值 | 相对 HW |
|----|------|------|--------|---------|
| 1 | joint_limits.yaml | max_velocity | 1.0/1.25 rad/s | ~40% |
| 1 | joint_limits.yaml | max_acceleration | 7.0/8.3 rad/s² | ~40% |
| 2 | square_demo_control.py | VELOCITY_SCALING | 0.5 | ×0.5 |
| 2 | square_demo_control.py | ACCEL_SCALING | 0.5 | ×0.5 |
| 3 | aubo_e5_linked_execution.launch | /aubo_controller/velocity_scale_factor | 0.5 | ×0.5 |
| 4 | aubo_driver.cpp | MaxVelc (tryPopWaypoint 检查) | 2.596/3.110 rad/s | 100% |
| 4 | aubo_driver.cpp | MaxAcc (tryPopWaypoint 检查) | 17.3/20.7 rad/s² | 100% |
| 5 | aubo_driver.h | VMAX (OTG 电机速度) | 1500 RPM | 50% |
| 5 | aubo_driver.h | AMAX (OTG 电机加速度) | 5000 | 50% |
| 5 | aubo_driver.h | JMAX (OTG 电机加加速度) | 20000 | 50% |
| 6 | 示教器 | 安全速度限幅 | 用户设定 | 未知 |

**有效速度计算 (square_demo 路径)**:

```
实机速度 ≈ joint_limits_vel × VELOCITY_SCALING × velocity_scale_factor
            × (5ms / T_robot)
         = 1.0 × 0.5 × 0.5 × 2.5
         = 0.625 rad/s  (24% HW)
```

**有效速度计算 (RViz 路径，无 VELOCITY_SCALING)**:

```
实机速度 ≈ 1.0 × 1.0 × 0.5 × 2.5
         = 1.25 rad/s  (48% HW)
```

---

## 5. 已修复的关键 Bug

### B1: 示教器 Z 坐标偏移 0.503m

- **原因**: URDF pedestal_joint 0.503m 偏移，示教器报告 base_link 系，ROS 使用 world 系
- **影响**: 用户输入的目标 Z 比实际意图低 0.5m，位移放大 2-3×，速度同比例放大
- **修复**: `square_demo_control.py` 添加 `PEDESTAL_Z=0.503`，所有 I/O 边界通过 `to_world()`/`to_teach()` 转换

### B2: IK 关节绕转 (WP-2→WP-3 超速)

- **原因**: `set_position_target()` 仅约束位置 (3 DOF)，IK 可自由选择方向。横向移动时可能选择绕转 ±2π 的关节解
- **症状**: tryPopWaypoint 计算 Δpos ≈ 6.28 rad → velocity ≈ 1256 rad/s
- **修复**: 改用 `set_pose_target()` 并保持当前末端方向，约束 IK 在 6 DOF

### B3: 5 阶多项式插补速度爆炸

- **原因**: `_motion_worker` 原使用 5 阶多项式插补，系数含 1/T³, 1/T⁴, 1/T⁵ 项。OMPL 密集路点 (T≈0.0005s) 使系数爆炸 → 输出 300-500 rad/s
- **修复**: 改为线性插补 + 使用轨迹自带速度值 (已由 ITP 限幅)

### B4: robot_status 话题竞态

- **原因**: aubo_driver (500Hz) 和 aubo_robot_simulator (50Hz) 都发布 `/robot_status`。模拟器的 `in_motion=FALSE` 覆盖驱动的 `in_motion=TRUE` → C++ Action 在实机未停稳时宣告 SUCCESS → 下一段轨迹暴起急停
- **修复**: launch 文件中为模拟器添加 `<remap from="robot_status" to="sim/robot_status"/>`

### B5: Safety Monitor 看门狗超时

- **原因**: safety_monitor 仅在轨迹事件时发布 `/safety_monitor/safe_to_execute`，无消息超过 5s → linked_execution_action_server 看门狗阻塞执行
- **修复**: 添加 2s 心跳定时器

### B6: verify_arrival 死等阻塞

- **原因**: 原 verify_arrival 用轮询超时等待 (最多 5s)，已通过 go() 确认到达后仍额外等待
- **修复**: 改为单次 FK 采样校验 (go() 已通过 C++ action + in_motion + Gazebo monitor 三层确认)

---

## 6. 关键话题与消息流

```
话题                        方向        类型                      频率

/joint_path_command         pub         JointTrajectory           按需 (每段轨迹一次)
/moveItController_cmd       pub         JointTrajectoryPoint      200Hz
/joint_states               pub         JointState                20Hz (timeCallback)
/aubo_e5/joint_states       pub         JointState                Gazebo 更新
/real/joint_states          pub         JointState                20Hz (mirror_adapter)
/sim/joint_states           pub         JointState                50Hz (simulator, 联动隔离)
/feedback_states            pub         FollowJointTrajectoryFeedback  20Hz
/sim/feedback_states        pub         FollowJointTrajectoryFeedback  50Hz (simulator, 隔离)
/robot_status               pub         RobotStatus               20Hz (driver) / sim隔离
/aubo_driver/rib_status     pub         Int32MultiArray           20Hz (CAN 缓冲区状态)
/aubo_driver/robot_connected param      string                    "1"/"0"
/aubo_controller/velocity_scale_factor  param   float              0.5

linked_execution_controller/follow_joint_trajectory    Action Server  (MoveIt → 聚合层)
aubo_e5_controller/follow_joint_trajectory             Action Server  (聚合层 → 实机)

/safety_monitor/safe_to_execute      pub  Bool           2s heartbeat
/safety_monitor/warning              pub  String         按需
/linked_execution/monitor_status     pub  String         按需
/linked_execution/monitor_goal       pub  JointState     每段轨迹
/linked_execution/monitor_control    pub  String         每段轨迹
```

---

## 7. 关键参数速查

### 7.1 launch 文件参数

| 参数 | 位置 | 默认 | 说明 |
|------|------|------|------|
| `sim_only` | arg | false | 纯仿真模式 (无实机) |
| `robot_ip` | arg | 192.168.1.10 | 实机控制器 IP |
| `/aubo_controller/velocity_scale_factor` | global param | 0.5 | 轨迹速度缩放 |
| `/aubo_driver/robot_connected` | global param | "0"/"1" | 实机连接状态 |
| `/robot_name` | global param | aubo_e5 | 机器人型号 |
| `constraints/goal_threshold` | C++ action param | 0.04 rad | 到达判定公差 |

### 7.2 驱动硬编码常量

| 常量 | 值 | 说明 |
|------|-----|------|
| VMAX | 1500 RPM | 电机速度上限 (OTG用) |
| AMAX | 5000 | 电机加速度上限 (OTG用) |
| JMAX | 20000 | 电机加加速度上限 (OTG用) |
| MaxVelc[0-2] | 2.596 rad/s | 关节 1-3 速度上限 (tryPopWaypoint 检查) |
| MaxVelc[3-5] | 3.110 rad/s | 关节 4-6 速度上限 |
| MaxAcc[0-2] | 17.309 rad/s² | 关节 1-3 加速度上限 |
| MaxAcc[3-5] | 20.737 rad/s² | 关节 4-6 加速度上限 |
| buffer_size_ | 400 | buf_queue_ 触发 start_move 的阈值 |
| MINIMUM_BUFFER_SIZE | 300 | 机器人 CAN 缓冲区最小空闲要求 |
| motion_update_rate | 200 Hz | 模拟器插补频率 |

---

## 8. 调试命令速查

```bash
# 检查当前末端位姿 (world 系)
rostopic echo /joint_states

# 检查实机关节状态
rostopic echo /real/joint_states

# 检查驱动队列 / CAN 缓冲区状态
rostopic echo /aubo_driver/rib_status
# data[0] = buf_queue_ size
# data[1] = control_mode
# data[2] = controller_connected_flag

# 检查 velocity_scale_factor 是否生效
rosparam get /aubo_controller/velocity_scale_factor

# 观察 moveItController_cmd 频率
rostopic hz /moveItController_cmd

# 观察实机速度检查是否触发
# 终端输出 "Joint X velocity ... rad/s exceeds limit" 表示驱动层速度检查触发
# 示教器 "目标速度超出限幅" 表示机器人内部安全限幅触发
```

---

## 9. 文件索引

| 文件 | 角色 |
|------|------|
| `src/aubo_linked_execution/scripts/square_demo_control.py` | 用户端控制器 (TUI + 轨迹执行) |
| `src/aubo_linked_execution/launch/aubo_e5_linked_execution.launch` | 总 launch 文件 |
| `src/aubo_robot/aubo_e5_moveit_config/config/joint_limits.yaml` | MoveIt 关节速度/加速度上限 |
| `src/aubo_linked_execution/scripts/linked_execution_action_server.py` | 联动执行聚合层 |
| `src/aubo_robot/aubo_controller/src/joint_trajectory_action.cpp` | C++ Action Server |
| `src/aubo_robot/aubo_controller/script/aubo_controller/aubo_robot_simulator` | 轨迹插补桥 (JointTrajectory→JointTrajectoryPoint) |
| `src/aubo_robot/aubo_controller/script/aubo_controller/trajectory_speed.py` | 轨迹速度缩放函数 |
| `src/aubo_robot/aubo_driver/src/aubo_driver.cpp` | 实机驱动 (队列、速度检查、CAN 通信) |
| `src/aubo_robot/aubo_driver/src/driver_node.cpp` | 驱动主循环 (500Hz) |
| `src/aubo_robot/aubo_driver/include/aubo_driver/aubo_driver.h` | 驱动头文件 (VMAX/AMAX/JMAX) |
| `src/aubo_robot/aubo_description/urdf/aubo_e5.xacro` | URDF 模型 (含 pedestal 0.503m 偏移) |
| `src/aubo_linked_execution/scripts/safety_monitor.py` | 安全监控 (心跳 + 轨迹起点检查) |
| `src/aubo_linked_execution/scripts/linked_execution_monitor.py` | Gazebo 收敛监控 |
