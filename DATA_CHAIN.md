# AUBO E5 数据链完整文档

## 从用户输入到实机执行反馈的全链路数据流

当前版本基线: 2026-05-25。最新的系统结构、[5] 预设打磨点位、急停、执行前清链、终点 FK 门控和修改记录，见 [CURRENT_SYSTEM_SUMMARY.md](CURRENT_SYSTEM_SUMMARY.md)。本文保留完整数据链细节；若旧段落与当前基线冲突，以 `CURRENT_SYSTEM_SUMMARY.md` 为准。

---

## 1. 系统架构总览

```
┌─────────────────────────────────────────────────────────────────┐
│  输入路径 A: GUI/TUI (square_demo_gui.py → square_demo_control.py)   │
│    用户输入示教器系坐标 → to_world() → MoveIt 目标规划           │
│    xyz 走 position target；xyz+RPY 走 pose target               │
└──────────────────────────┬──────────────────────────────────────┘
                           │
┌─────────────────────────────────────────────────────────────────┐
│  输入路径 B: RViz 交互标记                                        │
│    直接 world 系拖拽 → Plan & Execute                            │
│    无 Z 偏移问题（RViz 天然工作在 world 系）                      │
│    箭头=平移(3-DOF), 圆环=旋转(3-DOF), 同时拖=6-DOF             │
└──────────────────────────┬──────────────────────────────────────┘
                           │
                    MoveIt 规划层
              (OMPL + ITP, joint_limits.yaml 限幅)
                           │
              联动执行层 (linked_execution_action_server.py)
              实机成功即返回 SUCCESS，Gazebo 超时仅 WARN
                           │
              C++ Action 层 (joint_trajectory_action.cpp)
                           │
              轨迹插补桥 (aubo_robot_simulator, Python, 500Hz)
              velocity_scale_factor=0.58 缩放
                           │
              驱动队列层 (aubo_driver, 500Hz 消费)
              tryPopWaypoint() 速度检查 + 插补 (DT=0.002)
                           │
              CAN 总线 → 实机控制器
              (robotServiceSetRobotPosData2Canbus)
                           │
              反馈链 (timerCallback 50Hz)
              joint_states / feedback_states / robot_status
```

### 1.1 启动模式矩阵

| 参数 | 模式 | 控制器管理器 | Gazebo | 实机驱动 | 联动层 |
|------|------|------------|--------|---------|--------|
| `sim_only:=true` | 纯仿真 | aubo_e5 | 直连 | 不启动 | 不启动 |
| `sim_only:=false` (默认) | 联动 | linked_execution | shadow 镜像 | 启动 | 启动 |

启动入口: `run_square_demo.sh`，自动启动 ROS 系统后调用 `square_demo_gui.py`（图形 TUI 界面）。终端菜单暂保留为 `--terminal-menu` 调试入口，默认不再接管输入。

### 1.2 `run_square_demo.sh` 脚本设计

`run_square_demo.sh` 只负责系统编排，不直接规划或发送轨迹：

```
run_square_demo.sh
  │
  ├─ 解析模式
  │    ├─ 默认: sim_only:=true          Gazebo 仿真
  │    ├─ --real <ip>: sim_only:=false  实机执行 + Gazebo/Unity shadow
  │    ├─ --unity: use_unity:=true      使用 Unity 后端，默认 Gazebo
  │    ├─ --no-ui: 只启动 ROS，不打开 TUI
  │    ├─ --gui / --tui: 打开图形 TUI（默认）
  │    └─ --terminal-menu: 调试用旧终端菜单
  │
  ├─ 环境检查
  │    ├─ ROS_DISTRO
  │    ├─ devel/setup.bash
  │    ├─ rospack find aubo_linked_execution
  │    └─ rospack find aubo_gazebo
  │
  ├─ roslaunch aubo_linked_execution aubo_e5_linked_execution.launch
  │    ├─ sim_only:=true               纯 Gazebo / Unity 仿真
  │    ├─ robot_ip:=<ip>               实机联动
  │    ├─ use_unity:=true/false        后端选择
  │    └─ stdout/stderr → logs/square_demo_system_*.log
  │
  ├─ sleep 10 + 检查 roslaunch PID 存活
  │
  └─ rosrun aubo_linked_execution square_demo_gui.py
       ├─ AUBO_TUI_SYSTEM_LOG 指向上述启动日志
       └─ GUI 退出后脚本退出，trap cleanup 杀掉 roslaunch
```

脚本内置终端菜单函数仍在文件中，目的是保留调试和无 GUI 环境兜底；正常使用路径不调用它。用户输入、自定义位姿、多路径点、退出确认均由 `square_demo_gui.py` 负责。

### 1.3 图形 TUI 与核心控制器关系

`square_demo_gui.py` 不再维护独立运动逻辑。它的 `GUIDemoController` 是
`square_demo_control.py:SquareDemoController` 的适配层：

```
GUI Button / Entry
  → GUIDemoController.run_*()
  → SquareDemoController.run_*() / execute_multi_waypoints()
  → MoveIt plan()
  → FollowJointTrajectory action
  → linked_execution_controller
  → aubo_e5_controller
  → aubo_robot_simulator
  → aubo_driver
```

因此图形 TUI 和终端 action 使用同一套安全策略：位置目标 fallback、关节分支跳变拒绝、轨迹起点检查、direct action 执行、已发送轨迹不立即重试、联动层动态重定时。

GUI 实时数据来源：

| GUI 字段 | 来源 | 说明 |
|----------|------|------|
| X/Y/Z/RPY | `/aubo_driver/current_pose` 优先，MoveIt FK fallback | 优先显示与示教器一致的 SDK 位姿；fallback 时 `to_teach()` 转示教器系 |
| 同步延迟 | `/joint_states` vs `/aubo_e5/joint_states` header stamp | Gazebo/Unity backend 都通过 `/aubo_e5/joint_states` 进入 MoveIt 侧 |
| real Hz | `/real/joint_states` 优先，fallback `/joint_states` | 内部维护真实话题频率；当前 UI 显示按需求做 49-51Hz 慢速非周期波动 |
| backend Hz | `/unity/joint_states` 优先，fallback `/aubo_e5/joint_states` | 内部维护真实话题频率；当前 UI 显示按需求做 49-51Hz 慢速非周期波动 |
| 日志 | `LogRedirector` + `logs/square_demo_system_*.log` + `/rosout_agg` | GUI 自身日志、roslaunch 早期启动日志、MoveIt/RViz/控制器实时 ROS 日志均进入 GUI 日志队列 |

---

## 2. 坐标系

### 2.1 两个坐标系对比

| 坐标系 | 基准点 | Z 偏移 | 使用者 |
|--------|--------|--------|--------|
| **示教器系 (base_link 系)** | 机器人安装面 | Z=0 | 示教器、用户输入、显示 |
| **URDF world 系** | 地面 (pedestal 底面) | Z=+0.503m | MoveIt、ROS、内部计算 |

### 2.2 pedestal_joint 偏移来源

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
    """示教器系 → world 系（加 pedestal Z 偏移）"""
    return (tx, ty, tz + PEDESTAL_Z)

def to_teach(wx, wy, wz):
    """world 系 → 示教器系（减 pedestal Z 偏移）"""
    return (wx, wy, wz - PEDESTAL_Z)
```

所有用户 I/O 使用示教器系，与 MoveIt 交互前通过 `to_world()` 转换，显示前通过 `to_teach()` 转换。

### 2.4 RViz 路径无 Z 偏移问题

RViz 交互标记直接工作在 URDF world 系，MoveIt 规划也在 world 系，两者天然一致，**不存在 Z 偏移问题**。Z 偏移问题仅出现在 GUI/TUI 路径（用户以示教器系输入坐标时）。

### 2.5 6-DOF 输入支持

`square_demo_control.py` 新增两个辅助函数，支持用户以 RPY 角度指定末端方向：

```python
def rpy_to_quat(roll_deg, pitch_deg, yaw_deg):
    """RPY (度) → geometry_msgs.msg.Quaternion"""
    from tf.transformations import quaternion_from_euler
    q = quaternion_from_euler(math.radians(roll_deg),
                              math.radians(pitch_deg),
                              math.radians(yaw_deg))
    return geometry_msgs.msg.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def parse_pose_input(parts):
    """解析用户位姿输入，返回 (x, y, z, quat_or_None, error_msg)
    接受 3 个值 (仅位置, 保持当前方向) 或 6 个值 (位置 + RPY 度)"""
    if len(parts) == 3:
        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
        return x, y, z, None, None
    elif len(parts) == 6:
        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
        roll, pitch, yaw = float(parts[3]), float(parts[4]), float(parts[5])
        return x, y, z, rpy_to_quat(roll, pitch, yaw), None
    else:
        return None, None, None, None, '需要 3 个数字 (x y z) 或 6 个数字 (x y z roll pitch yaw 度)'
```

`execute_pose_target()`、`run_custom_waypoint()` 支持可选 `orientation` 参数。多路径点连续模式为保证直线/圆轨迹姿态稳定，会锁定执行开始时的当前末端方向；逐点 RPY 输入会被忽略并记录 WARN，避免 wrist3 翻转。

---

## 3. 完整数据流（逐层）

### 第 1 层：用户输入 → 目标位姿

#### 1a. GUI/TUI 路径（square_demo_control.py）

**文件**: `square_demo_control.py`

用户通过菜单选项输入目标，所有坐标均以示教器系（base_link 系）表示：

```
用户输入 (示教器系):
  选项 [1]: SQUARE_CORNERS 预设角点 (0.4, ±0.10, 0.50/0.70)
  选项 [2]: 手动输入 "x y z" 或 "x y z roll pitch yaw(度)"
  选项 [3]: 多路径点连续笛卡尔轨迹，每行 "x y z"
  选项 [4]: 安全审查状态 (6 个看门狗/门控耦合点)
  选项 [5]: 预设工件打磨测试
            GRIND-PRECHECK
            → 接近点 (-0.6000,-0.0800,0.2600)
            → 直线 3 点 Z=0.1800 固定 RPY(178.0,4.5,-86.0)
            → 垂直抬升 0.10m
            → 圆弧起点上方 gap
            → 下探到圆弧起点
            → 13 点 6D 圆弧循迹 Z=0.1765
  选项 [6]: 轨迹生成测试 / OMPL 与 LERP 说明
  选项 [7]: 介绍 (README)
      │
      ├─ parse_pose_input(parts)
      │     3 值 → (x, y, z, None, None)       仅位置，保持当前方向
      │     6 值 → (x, y, z, Quaternion, None)  位置 + RPY 度转四元数
      │
      ├─ check_waypoint_safety(x, y, z)
      │     距肩关节中心 > 0.886m → 警告
      │     world 系 Z < 0.02m → 警告
      │
      ├─ 单点目标 → execute_pose_target(target_teach, orientation=None)
      │     to_world(tx, ty, tz) → (wx, wy, wz)  加 PEDESTAL_Z=0.503
      │
      ├─ 多路径点 (选项 [3]) → execute_multi_waypoints() → execute_cartesian_waypoint_path()
      │    锁定当前末端四元数
      │    用户路径点按 2cm 稠密化
      │    compute_cartesian_path(eef_step=5mm)
      │    retime_trajectory()
      │    wrist3_joint 转动范围检查
      │    direct FollowJointTrajectory 一次性下发
      │
      ├─ 预设打磨 (选项 [5]) → run_grinding_test()
      │    每段发送前 _flush_execution_pipeline()
      │    发送前终点 FK 门控，防止旧目标/错误阶段轨迹进入实机
      │
      └─ 轨迹生成测试 (选项 [6]) → run_planning_algorithms_overview()
```

示例坐标转换:
```
输入(示教器系):  (0.4, 0.10, 0.70)
转换(world系):  (0.4, 0.10, 1.203)
```

#### 1b. RViz 路径

RViz 交互标记直接工作在 URDF world 系，无需坐标转换：

```
RViz 交互标记 (world 系):
  箭头拖拽   → 平移 (3-DOF 位置)
  圆环拖拽   → 旋转 (3-DOF 方向)
  同时操作   → 6-DOF 位姿
      │
      └─ Plan & Execute
            MoveIt 直接接收 world 系位姿
            无 Z 偏移问题
```

---

### 第 2 层：MoveIt 轨迹规划

**文件**: `square_demo_control.py:execute_pose_target()`

**重要**: 终端路径先 `plan()`，再由脚本直接发送 `FollowJointTrajectoryGoal`。
3-DOF 输入使用 `set_position_target()`，6-DOF 输入使用 `set_pose_target()`。
所有位置目标执行前都会做关节分支跳变和轨迹起点检查。

```python
# xyz 输入:
self.group.set_position_target([wx, wy, wz])

# xyz + RPY 输入:
self.group.set_pose_target(target_pose)

plan_ok, plan = normalize(self.group.plan())
reject_if_joint_branch_jump(plan)
reject_if_start_gap_too_large(plan)
send_follow_joint_trajectory_goal(plan)
```

**为什么允许 `set_position_target`**:
`set_position_target` 仅约束位置 (3 DOF)，IK 有时会选择不同关节解分支。旧实现直接执行这类结果，可能把 wrist3 从 +3.0 rad 跳到 -3.0 rad，导致 `tryPopWaypoint` 看到 `Δpos ≈ 6.28 rad`。当前实现允许位置目标，但会在执行前拒绝相邻点关节跳变超过阈值的轨迹；用户明确输入 RPY 时仍走 6-DOF 姿态约束。

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
  │     速度上限: joint_limits.yaml (0.5/0.6 rad/s)
  │     加速度上限: joint_limits.yaml (3.5/4.0 rad/s²)
  │
  └─ MoveIt Velocity Scaling: × VELOCITY_SCALING (1.0)
        实际 ITP 速度: 0.5 × 1.0 = 0.5 rad/s (J1-3)
                       0.6 × 1.0 = 0.6 rad/s (J4-6)
```

**URDF 速度限制**: URDF 所有关节 `velocity="0"`（无 URDF 速度限制），ITP 完全依赖 `joint_limits.yaml`。

#### 2b. 关节限幅配置

**文件**: `joint_limits.yaml`（所有路径的统一安全上限）

| 关节 | 硬件上限 | 配置值 | 百分比 |
|------|---------|--------|--------|
| shoulder / upperArm / foreArm 速度 | 2.596 rad/s | **0.5 rad/s** | ~19% |
| wrist1 / wrist2 / wrist3 速度 | 3.110 rad/s | **0.6 rad/s** | ~19% |
| shoulder / upperArm / foreArm 加速度 | 17.309 rad/s² | **3.5 rad/s²** | ~20% |
| wrist1 / wrist2 / wrist3 加速度 | 20.737 rad/s² | **4.0 rad/s²** | ~19% |

---

### 第 3 层：联动执行 Action Server

**文件**: `linked_execution_action_server.py`

```
MoveIt 发出 FollowJointTrajectoryGoal
  │
  ├─ 0. 安全检查: _safe_to_execute == True?
  │      看门狗定时器每 1s 检查 safety_monitor 心跳
  │      超过 safety_watchdog_timeout=5s → _safe_to_execute=False → ABORT
  │
  ├─ 1. 轨迹审查
  │      检查 joint_names、NaN、轨迹起点 vs /real/joint_states
  │      检查相邻轨迹点关节跳变
  │
  ├─ 2. 动态重定时
  │      按 velocity_scale_factor 和 5ms/2ms 时序系数预测实机速度/加速度
  │      超过 max_robot_velocity / max_robot_acceleration 时拉长 time_from_start
  │
  ├─ 3. 计算有效轨迹时长: retimed_duration / velocity_scale_factor
  │
  ├─ 4. 转发目标到实机 Action: aubo_e5_controller/follow_joint_trajectory
  │
  ├─ 5. 发布轨迹终点到 Gazebo 监控: /linked_execution/monitor_goal
  │
  ├─ 6. 等待实机 Action 返回 (timeout = effective_duration + 10s)
  │      超时 → ABORT
  │      实机失败 → ABORT
  │
  └─ 7. 等待 Gazebo 收敛 (timeout = effective_duration + 8s)
         ★ 软警告模式: 实机成功即返回 SUCCESS
         ★ Gazebo 超时仅记录 WARN，不 abort
         → set_succeeded(real_result)
```

---

### 第 4 层：C++ Joint Trajectory Action

**文件**: `joint_trajectory_action.cpp`

```
接收 FollowJointTrajectoryGoal
  │
  ├─ goalCB():
  │     pub_trajectory_command_.publish(current_traj_)
  │     → 发布为 /joint_path_command (JointTrajectory)
  │
  └─ controllerStateCB():
        withinGoalConstraints()  检查关节误差 < goal_threshold_/2
        robot_status.in_motion == FALSE  确认机器人停稳
        二者同时满足 → setSucceeded()

goal_threshold_ = rosparam("~constraints/goal_threshold", 0.04 rad)
→ per-joint 误差容限 = 0.04/2 = 0.02 rad (≈ 1.15°)  // isWithinRange 用 half_range
```

---

### 第 5 层：轨迹插补桥（aubo_robot_simulator）

**文件**: `aubo_robot_simulator`（**Python 版本**，C++ 版已注释）

**角色**: 将 `/joint_path_command` (JointTrajectory) 插补为 `/moveItController_cmd` (JointTrajectoryPoint 流)

```
trajectory_callback(msg_in: JointTrajectory)
  │
  ├─ 读取 velocity_scale_factor = rospy.get_param('/aubo_controller/velocity_scale_factor', 1.0)
  │     当前值: 0.58 (launch 文件设定)
  │
  ├─ scale_trajectory_speed(msg_in, scale=0.58)
  │     time_from_start /= scale   → 时间扩展 (0.58 → ~1.72× 时间)
  │     velocities      *= scale   → 速度缩放 (0.58 → 58%)
  │     accelerations   *= scale²  → 加速度缩放 (0.58 → ~34%)
  │
  ├─ _to_controller_order()  重排关节顺序以匹配 controller_joint_names
  │
  └─ _motion_worker()  线性插补 @ 500Hz (motion_update_rate)
        位置: last + α×(current - last)
        速度: last_vel + α×(current_vel - last_vel)  使用轨迹自带速度
        加速度: 恒为 0
        ↓
        /moveItController_cmd (JointTrajectoryPoint, 200Hz)
        消息类型无 joint_names 字段，依赖关节顺序一致性
```

**velocity_scale_factor 的影响**:

| scale | time_from_start | 速度值 | 插补点数 | Δpos/2ms | driver 计算速度 |
|-------|----------------|--------|---------|-----------|----------------|
| 1.0 | 1× | 100% | N | Δpos | V |
| 0.58 | ~1.72× | 58% | ~1.72N | Δpos/1.72 | V/1.72 |

---

### 第 6 层：驱动接收与排队（aubo_driver）

**文件**: `aubo_driver.cpp`

```
moveItPosCallback(msg: JointTrajectoryPoint)  ← /moveItController_cmd
  │
  ├─ 检测新轨迹: !start_move_ && buf_queue_.empty()
  │    → need_sync_filter_.store(true)  (异步触发 joint_filter_ 同步)
  │
  ├─ roadPointCompare(jointAngle, last_recieve_point_)
  │    阈值 THRESHHOLD = 0.000001 rad
  │    过滤与上一点相同的位置（避免重复入队）
  │
  └─ 推入 buf_queue_<PlanningState>
       joint_pos_ = msg.positions
       joint_vel_ = msg.velocities   ← 存储但后续未使用!
       joint_acc_ = msg.accelerations
       buf_queue_.size() > buffer_size_(400) → start_move_ = true

updateControlStatus()  ← 由 main loop 每 2ms (500Hz) 调用
  │
  ├─ data_count_++ → 每 MAXALLOWEDDELAY(50) 次 (0.1s) 检查一次
  │    buf_queue_.size() > 0 && !start_move_ → start_move_ = true
  │
  └─ start_move_ && rib_buffer_size_ < MINIMUM_BUFFER_SIZE(300)
       → setRobotJointsByMoveIt()
           从 buf_queue_ 弹出 → 仅将 joint_pos_ 推入 ros_motion_queue_
           速度信息在此处丢弃!
```

**关键**: `ros_motion_queue_` 只传递位置，MoveIt 精心计算的速度被丢弃，速度由 `tryPopWaypoint` 重新从位置差分计算。

---

### 第 7 层：速度检查与插补（tryPopWaypoint）

**文件**: `aubo_driver.cpp:tryPopWaypoint()`

```
publishWaypointToRobot() 线程 (持续运行)
  │
  ├─ 处理 need_sync_filter_ 异步同步请求
  │    robotServiceGetCurrentWaypointInfo() → joint_filter_
  │    最多重试 MAX_SYNC_RETRIES=5 次
  │
  ├─ 查询 macTargetPosDataSize (CAN 缓冲区剩余空间)
  │    current_macsz < expect_macsz(400) && ros_motion_queue_ 非空
  │    → cnt = ceil((400 - current_macsz) / 6.0)
  │    → tryPopWaypoint(cnt)
  │
  └─ tryPopWaypoint(count):
       从 ros_motion_queue_ 弹出 cnt 个路点
         │
         ├─ 速度检查: DT = 0.002s (匹配机器人 500Hz 伺服)
         │    target_joint_velc_[i] = fabs(joint[i] - joint_filter_[i]) / 0.002
         │
         ├─ 超过 MaxVelc[i]?
         │    MaxVelc = {2.596, 2.596, 2.596, 3.110, 3.110, 3.110} rad/s (100% HW)
         │    → max_ratio = max(target_vel / MaxVelc)
         │    → n_equalpart = ceil(max_ratio) + 1 个中间插补点
         │    → ROS_WARN "Joint X velocity Y rad/s exceeds limit Z rad/s"
         │
         ├─ 加速度检查: joint_acc_[i] = fabs(Δvelocity) / 0.002
         │    MaxAcc = {17.309, 17.309, 17.309, 20.737, 20.737, 20.737} rad/s²
         │    仅日志，不阻断、不插补!
         │
         ├─ 速度诊断 (可选，见下方 7.1)
         │
         └─ 结果: wayPointVector (仅位置) → robotServiceSetRobotPosData2Canbus()
```

#### 7.1 速度诊断功能

`tryPopWaypoint` 内置可选速度诊断，通过 rosparam 开关控制：

```
静态缓存，每 5 秒刷新一次（线程安全）:
  /aubo_driver/velocity_diag_enable        bool, 默认 false
  /aubo_driver/velocity_diag_warn_levels   6元素列表, 默认 [0.5,0.5,0.5,0.6,0.6,0.6]

开启后每批次输出 (ROS_INFO_THROTTLE 2s):
  [vel-diag] batch N wp | J0 Δ=... v=...  J1 Δ=... v=...  ...

超阈值输出 (ROS_WARN_THROTTLE 1s):
  [vel-diag] joint X velocity Y rad/s exceeds configurable warn level Z rad/s
```

---

### 第 8 层：机器人内部控制器

```
robotServiceSetRobotPosData2Canbus(wayPointVector)
  │
  ├─ CAN 总线 → 机器人控制板缓冲区
  │    缓冲区大小: macTargetPosDataSize (最大 2400)
  │    驱动监控此值，只在有空间时发送
  │
  └─ 机器人内部控制循环 (~2ms, 500Hz)
       按自己的周期插补位置路点
       检查关节速度是否超出安全限幅 (示教器可配置)
       检查 TCP 速度是否超出安全限幅
       超限 → 保护性停止 (示教器显示 "目标速度超出限幅")
```

---

### 第 9 层：反馈链

```
实机关节编码器
  │
  ├─ aubo_driver::timerCallback()  @ 50Hz (TIMER_SPAN_=50, ros::Duration(1.0/50)=0.02s)
  │     robotServiceGetCurrentWaypointInfo() → current_joints_
  │     发布:
  │       /joint_states (sensor_msgs/JointState)           50Hz
  │       /feedback_states (FollowJointTrajectoryFeedback)  50Hz
  │       /robot_status (RobotStatus) ← in_motion=start_move_  50Hz
  │       /aubo_driver/rib_status (Int32MultiArray)         50Hz
  │         data[0] = buf_queue_.size()
  │         data[1] = control_mode_
  │         data[2] = controller_connected_flag_
  │
  ├─ C++ Action: controllerStateCB() 订阅 /feedback_states
  │     → withinGoalConstraints() 判断是否到达目标
  │     → robot_status.in_motion == FALSE 确认停稳
  │
  ├─ Gazebo shadow 链:
  │     /joint_states
  │       → joint_state_mirror_adapter.py
  │       → /real/joint_states
  │       → gazebo_driver (shadow 模式)
  │       → /aubo_e5/joint_states
  │       → linked_execution_monitor.py
  │
  └─ linked_execution_monitor.py:
       /linked_execution/monitor_status → linked_execution_action_server
       SUCCEEDED / FAILED / TIMEOUT
```

**注意**: `timerCallback` 频率为 **50Hz**（`TIMER_SPAN_=50`，`ros::Duration(1.0/50)=0.02s`），原文档写的 20Hz 是错误的。

---

## 4. 速度/加速度限幅层级汇总

| 层 | 位置 | 参数 | 当前值 | 相对 HW |
|----|------|------|--------|---------|
| 1 | joint_limits.yaml | max_velocity (J1-3) | **0.5 rad/s** | ~19% |
| 1 | joint_limits.yaml | max_velocity (J4-6) | **0.6 rad/s** | ~19% |
| 1 | joint_limits.yaml | max_acceleration (J1-3) | **3.5 rad/s²** | ~20% |
| 1 | joint_limits.yaml | max_acceleration (J4-6) | **4.0 rad/s²** | ~19% |
| 2 | square_demo_control.py | VELOCITY_SCALING | **1.0** | ×1.0 |
| 2 | square_demo_control.py | ACCEL_SCALING | **1.0** | ×1.0 |
| 3 | aubo_e5_linked_execution.launch | /aubo_controller/velocity_scale_factor | **0.58** | ×0.58 |
| 4 | aubo_driver.cpp | MaxVelc (tryPopWaypoint 检查) | 2.596/3.110 rad/s | 100% |
| 4 | aubo_driver.cpp | MaxAcc (tryPopWaypoint 检查) | 17.3/20.7 rad/s² | 100% |
| 5 | aubo_driver.h | VMAX (OTG 电机速度) | 1500 RPM | 50% |
| 5 | aubo_driver.h | AMAX (OTG 电机加速度) | 5000 | 50% |
| 5 | aubo_driver.h | JMAX (OTG 电机加加速度) | 20000 | 50% |
| 6 | 示教器 | 安全速度限幅 | 用户设定 | 未知 |

**有效速度计算（两条路径统一，joint_limits.yaml 作为唯一安全上限）**:

```
RViz 路径:
  J1-3: 0.5(limits) × 1.0(无MoveIt缩放) × 0.58(scale_factor) × 2.5(timing) = 0.725 rad/s (~28% HW)
  J4-6: 0.6(limits) × 1.0              × 0.58               × 2.5          = 0.870 rad/s (~28% HW)

GUI/TUI 路径:
  J1-3: 0.5(limits) × 1.0(VELOCITY_SCALING) × 0.58(scale_factor) × 2.5(timing) = 0.725 rad/s (~28% HW)
  J4-6: 0.6(limits) × 1.0                   × 0.58               × 2.5          = 0.870 rad/s (~28% HW)
```

两条路径基础速度统一。RViz Plan&Execute 额外经过 `linked_execution_action_server.py` 的动态审查：
若预测实机速度超过 0.72/0.90 rad/s，或计划/相邻离散段反算加速度超过 2.0/2.4 rad/s²，会自动拉长时间戳，防止示教器关节碰撞/力矩保护。

**timing 系数 2.5 的来源**: 模拟器按 5ms 间隔产生路点（200Hz），但实机控制器内部执行周期约为 2ms（500Hz）。实机实际速度 = 驱动计算速度 × (5ms / 2ms) = **驱动计算速度 × 2.5**。这是速度超限的根源之一——驱动以 100% 硬件上限检查通过，但实机看到 2.5× 更高的速度。

---

## 5. 已修复的关键 Bug

### B1: 示教器 Z 坐标偏移 0.503m

- **原因**: URDF pedestal_joint 0.503m 偏移，示教器报告 base_link 系，ROS 使用 world 系
- **影响**: 用户输入的目标 Z 比实际意图低 0.5m，位移放大 2-3×，速度同比例放大
- **修复**: `square_demo_control.py` 添加 `PEDESTAL_Z=0.503`，所有 I/O 边界通过 `to_world()`/`to_teach()` 转换

### B2: IK 关节绕转（WP-2→WP-3 超速）

- **原因**: `set_position_target()` 仅约束位置 (3 DOF)，IK 可自由选择方向。横向移动时可能选择绕转 ±2π 的关节解
- **症状**: tryPopWaypoint 计算 Δpos ≈ 6.28 rad → velocity ≈ 3140 rad/s（DT=0.002）
- **修复**: 改用 `set_pose_target()` 并保持当前末端方向，约束 IK 在 6 DOF

### B3: 5 阶多项式插补速度爆炸

- **原因**: `_motion_worker` 原使用 5 阶多项式插补，系数含 1/T³, 1/T⁴, 1/T⁵ 项。OMPL 密集路点 (T≈0.0005s) 使系数爆炸 → 输出 300-500 rad/s
- **修复**: 改为线性插补 + 使用轨迹自带速度值（已由 ITP 限幅）

### B4: robot_status 话题竞态

- **原因**: aubo_driver (500Hz) 和 aubo_robot_simulator (50Hz) 都发布 `/robot_status`。模拟器的 `in_motion=FALSE` 覆盖驱动的 `in_motion=TRUE` → C++ Action 在实机未停稳时宣告 SUCCESS → 下一段轨迹暴起急停
- **修复**: launch 文件中为模拟器添加 `<remap from="robot_status" to="sim/robot_status"/>`

### B5: Safety Monitor 看门狗超时

- **原因**: safety_monitor 仅在轨迹事件时发布 `/safety_monitor/safe_to_execute`，无消息超过 5s → linked_execution_action_server 看门狗阻塞执行
- **修复**: 添加 2s 心跳定时器

### B6: verify_arrival 死等阻塞

- **原因**: 原 verify_arrival 用轮询超时等待（最多 5s），已通过 go() 确认到达后仍额外等待
- **修复**: 改为单次 FK 采样校验（go() 已通过 C++ action + in_motion + Gazebo monitor 三层确认）

### B7: RViz Plan&Execute 速度超限 + 虚假 "failed"

- **原因**: RViz Plan&Execute 路径不经过 `square_demo_control.py`，缺少 VELOCITY_SCALING=0.5，ITP 速度是 square_demo 的 2 倍（1.0 vs 0.5 rad/s）。实机速度 1.25 rad/s (48% HW) 触发 tryPopWaypoint 速度超限和/或机器人保护性停止，导致 C++ Action abort。
- **症状**: RViz 拖拽末端执行 Plan&Execute → 实机能运动到目标附近但 RViz 显示 "failed"；终端出现 "Joint X velocity exceeds limit" 警告
- **修复1**: `joint_limits.yaml` 速度降为 0.5/0.6 rad/s (~19% HW)，作为所有路径的统一安全上限，不再依赖各路径分别降速
- **修复2**: `square_demo_control.py` VELOCITY_SCALING 从 0.5 提升到 1.0，补偿 joint_limits 的降低，保持用户体验不变
- **修复3**: `linked_execution_action_server.py` Gazebo 收敛检查改为软警告——实机成功即返回 SUCCESS，Gazebo 超时仅记录 WARN 不 abort。消除 Gazebo 镜像延迟导致的虚假失败

---

## 6. 关键话题与消息流

```
话题                                    方向   类型                              频率

/joint_path_command                     pub    JointTrajectory                   按需 (每段轨迹一次)
/moveItController_cmd                   pub    JointTrajectoryPoint              500Hz
/joint_states                           pub    JointState                        50Hz (timerCallback)
/aubo_e5/joint_states                   pub    JointState                        Gazebo 更新
/real/joint_states                      pub    JointState                        50Hz (mirror_adapter)
/sim/joint_states                       pub    JointState                        50Hz (simulator, 联动隔离)
/feedback_states                        pub    FollowJointTrajectoryFeedback     50Hz (driver)
/sim/feedback_states                    pub    FollowJointTrajectoryFeedback     50Hz (simulator, 隔离)
/robot_status                           pub    RobotStatus                       50Hz (driver) / sim隔离
/aubo_driver/rib_status                 pub    Int32MultiArray                   50Hz (CAN 缓冲区状态)
/aubo_driver/robot_connected            param  string                            "1"/"0"
/aubo_controller/velocity_scale_factor  param  float                             0.58

linked_execution_controller/follow_joint_trajectory   Action Server  (MoveIt → 聚合层)
aubo_e5_controller/follow_joint_trajectory            Action Server  (聚合层 → 实机)

/safety_monitor/safe_to_execute         pub    Bool     2s heartbeat
/safety_monitor/warning                 pub    String   按需
/linked_execution/monitor_status        pub    String   按需
/linked_execution/monitor_goal          pub    JointState  每段轨迹
/linked_execution/monitor_control       pub    String   每段轨迹
```

**关节名称顺序**（全链统一）:
```
[shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint]
```

`/moveItController_cmd` 消息类型 `JointTrajectoryPoint`，**无 joint_names 字段**，依赖顺序一致性。

---

## 7. 关键参数速查

### 7.1 launch 文件参数

| 参数 | 位置 | 默认 | 说明 |
|------|------|------|------|
| `sim_only` | arg | false | 纯仿真模式（无实机） |
| `robot_ip` | arg | 192.168.1.10 | 实机控制器 IP |
| `/aubo_controller/velocity_scale_factor` | global param | 0.58 | 轨迹速度缩放 |
| `/aubo_driver/robot_connected` | global param | "0"/"1" | 实机连接状态 |
| `/robot_name` | global param | aubo_e5 | 机器人型号 |
| `constraints/goal_threshold` | C++ action param | 0.04 rad | 到达判定公差 |
| `safety_watchdog_timeout` | linked_execution param | 5.0 s | 安全监控看门狗超时 |
| `real_server_wait_timeout` | linked_execution param | 30.0 s | 等待实机 action server 超时 |

### 7.2 驱动硬编码常量

| 常量 | 值 | 说明 |
|------|-----|------|
| UPDATE_RATE_ | 500 Hz | 主循环频率（updateControlStatus 调用频率） |
| TIMER_SPAN_ | 50 | timerCallback 频率 = 50Hz（ros::Duration(1.0/50)） |
| THRESHHOLD | 0.000001 rad | roadPointCompare 去重阈值 |
| VMAX | 1500 RPM | 电机速度上限（OTG 用） |
| AMAX | 5000 | 电机加速度上限（OTG 用） |
| JMAX | 20000 | 电机加加速度上限（OTG 用） |
| MaxVelc[0-2] | 2.596 rad/s | 关节 1-3 速度上限（tryPopWaypoint 检查） |
| MaxVelc[3-5] | 3.110 rad/s | 关节 4-6 速度上限 |
| MaxAcc[0-2] | 17.309 rad/s² | 关节 1-3 加速度上限 |
| MaxAcc[3-5] | 20.737 rad/s² | 关节 4-6 加速度上限 |
| buffer_size_ | 400 | buf_queue_ 触发 start_move 的阈值 |
| MINIMUM_BUFFER_SIZE | 300 | 机器人 CAN 缓冲区最小空闲要求 |
| ROBOT_WAYPOINT_DT | 0.002 s | 速度检查 DT（匹配机器人 500Hz 伺服） |
| motion_update_rate | 500 Hz | 模拟器插补频率 (与实机 MAC 2ms 消费周期对齐) |

### 7.3 速度诊断参数

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `/aubo_driver/velocity_diag_enable` | bool | false | 速度诊断开关 |
| `/aubo_driver/velocity_diag_warn_levels` | float[6] | [0.5,0.5,0.5,0.6,0.6,0.6] | 各关节警告阈值 (rad/s) |

### 7.4 驱动侧硬限幅 (launch 文件设定)

| 参数 | 类型 | 默认 | 说明 |
|------|------|------|------|
| `/aubo_driver/velocity_safe_limits` | float[6] | [0.5,0.5,0.5,0.6,0.6,0.6] | 驱动层安全速度上限 (rad/s)，超限路点拒绝而非插补 |
| `/aubo_driver/max_waypoint_delta` | float | 0.05 | 相邻路点最大关节跳变量 (rad) |
| `/aubo_driver/reject_overspeed_waypoints` | bool | true | 超速路点拒绝开关 (true=拒绝, false=插补后发送) |

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
# data[1] = control_mode_
# data[2] = controller_connected_flag_

# 检查 velocity_scale_factor 是否生效
rosparam get /aubo_controller/velocity_scale_factor

# 观察 moveItController_cmd 频率
rostopic hz /moveItController_cmd

# 观察实机速度检查是否触发
# 终端输出 "Joint X velocity ... rad/s exceeds limit" 表示驱动层速度检查触发
# 示教器 "目标速度超出限幅" 表示机器人内部安全限幅触发

# 开启速度诊断（运行时动态开关）
rosparam set /aubo_driver/velocity_diag_enable true

# 关闭速度诊断
rosparam set /aubo_driver/velocity_diag_enable false

# 设置自定义警告阈值（6 个关节，rad/s）
rosparam set /aubo_driver/velocity_diag_warn_levels "[0.4, 0.4, 0.4, 0.5, 0.5, 0.5]"

# 6-DOF 输入示例（TUI 菜单 [2] 或 [3]）
# 仅位置（保持当前方向）:
#   0.4 0.0 0.6
# 位置 + RPY 度（指定方向）:
#   0.4 0.0 0.6 0 90 0
```

---

## 9. 文件索引

| 文件 | 角色 |
|------|------|
| `src/aubo_linked_execution/scripts/square_demo_control.py` | 核心用户控制器（轨迹执行、安全检查、支持 3-DOF/6-DOF 输入） |
| `src/aubo_linked_execution/scripts/square_demo_gui.py` | 图形 TUI 适配层（按钮/输入/日志/实时数据，调用核心控制器） |
| `src/aubo_linked_execution/launch/aubo_e5_linked_execution.launch` | 总 launch 文件 |
| `src/aubo_robot/aubo_e5_moveit_config/config/joint_limits.yaml` | MoveIt 关节速度/加速度上限（所有路径统一安全上限） |
| `src/aubo_linked_execution/scripts/linked_execution_action_server.py` | 联动执行聚合层（软警告模式） |
| `src/aubo_robot/aubo_controller/src/joint_trajectory_action.cpp` | C++ Action Server |
| `src/aubo_robot/aubo_controller/script/aubo_controller/aubo_robot_simulator` | 轨迹插补桥（Python，JointTrajectory→JointTrajectoryPoint） |
| `src/aubo_robot/aubo_controller/script/aubo_controller/trajectory_speed.py` | 轨迹速度缩放函数 |
| `src/aubo_robot/aubo_driver/src/aubo_driver.cpp` | 实机驱动（队列、速度检查 DT=0.002、速度诊断、CAN 通信） |
| `src/aubo_robot/aubo_driver/src/driver_node.cpp` | 驱动主循环（500Hz） |
| `src/aubo_robot/aubo_driver/include/aubo_driver/aubo_driver.h` | 驱动头文件（VMAX/AMAX/JMAX，UPDATE_RATE_=500，TIMER_SPAN_=50） |
| `src/aubo_robot/aubo_description/urdf/aubo_e5.xacro` | URDF 模型（含 pedestal 0.503m 偏移，关节 velocity="0"） |
| `src/aubo_linked_execution/scripts/safety_monitor.py` | 安全监控（心跳 + 轨迹起点检查） |
| `src/aubo_linked_execution/scripts/linked_execution_monitor.py` | Gazebo 收敛监控 |
| `run_square_demo.sh` | 一键启动脚本（启动 ROS 系统 + square_demo_gui.py GUI 界面） |
