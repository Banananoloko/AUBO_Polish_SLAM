# AUBO E5 当前系统版本总结

更新时间: 2026-05-25

本文只记录当前有效版本。调试过程中出现过但已经被替换的中间方案、冲突结论和旧参数不作为当前依据。

## 1. 当前系统定位

当前系统是一套从一键启动脚本到图形 TUI，再到 MoveIt、linked execution、AUBO SDK 驱动和实机反馈的闭环控制链。默认用户入口是仓库根目录的 `run_square_demo.sh`，默认交互入口是 `square_demo_gui.py` 图形 TUI。

核心目标:

- 用图形 TUI 发起正方形、自定义目标、多路径点、预设打磨轨迹、安全审查和 README 查看。
- TUI 输入、显示和预设轨迹统一使用示教器坐标系，也就是 `base_link` 系。
- MoveIt/URDF 内部使用 `world` 系，通过 `PEDESTAL_Z = 0.503` 做转换。
- 所有会运动的 GUI 按钮都委托给 `square_demo_control.py:SquareDemoController`，不在 GUI 内部重复实现运动逻辑。
- 实机执行路径必须经过 `linked_execution_controller/follow_joint_trajectory`，实机模式禁止绕过 linked execution 直连 `aubo_e5_controller`。
- 执行前执行多级安全门控，避免旧轨迹、分支跳变、时间戳异常、起点错位和终点错发。

## 2. 启动结构

推荐入口:

```bash
cd /home/wuqz/aubo_polish
./run_square_demo.sh
```

常用模式:

| 命令 | 含义 |
|------|------|
| `./run_square_demo.sh` | Gazebo 仿真 + 图形 TUI |
| `./run_square_demo.sh --real 192.168.10.230` | 实机 + Gazebo shadow + 图形 TUI |
| `./run_square_demo.sh --unity` | Unity 后端 + 图形 TUI |
| `./run_square_demo.sh --real 192.168.10.230 --unity` | 实机 + Unity shadow + 图形 TUI |
| `./run_square_demo.sh --no-ui` | 只启动 ROS 系统，不打开 TUI |
| `./run_square_demo.sh --terminal-menu` | 调试用旧终端菜单 |

启动链:

```text
run_square_demo.sh
  -> roslaunch aubo_linked_execution aubo_e5_linked_execution.launch
  -> logs/square_demo_system_*.log
  -> rosrun aubo_linked_execution square_demo_gui.py
  -> GUI Button / Entry
  -> GUIDemoController
  -> SquareDemoController
```

图形 TUI 退出时会自动保存日志快照到:

```text
/home/wuqz/aubo_polish/system_logs/square_demo_tui_*.log
```

并弹窗提示保存路径。

## 3. 图形 TUI 当前功能

TUI 文件:

```text
src/aubo_linked_execution/scripts/square_demo_gui.py
```

窗口当前结构:

- 左侧功能和输入面板宽度为 520px，用于避免长文本和输入区遮挡。
- 右侧上半部分显示实时位姿和状态，下半部分显示系统日志。
- 日志窗口整合 GUI stdout/cprint、启动日志文件和 `/rosout_agg`。
- “急停”按钮为红色大按钮，位于 `OMPL — RRT Connect / LERP 规划算法 — 线性插值` 信息行最右侧。

按钮映射:

| 按钮 | 当前功能 | 后端接口 |
|------|----------|----------|
| `[1] 执行正方形轨迹` | 执行预设正方形路径 | `run_square_trajectory()` |
| `[2] 自定义目标位姿` | 聚焦并提示自定义输入 | `_on_button_2()` |
| `执行自定义目标` | 执行 `x y z [roll pitch yaw]` | `run_custom_waypoint()` |
| `[3] 多路径点连续轨迹` | 聚焦多路径点输入 | `_on_button_3()` |
| `执行多路径点` | 执行多路径点笛卡尔轨迹 | `run_multi_waypoint()` -> `execute_multi_waypoints()` |
| `[4] 安全审查状态` | 打印看门狗/门控状态 | `run_safety_watchdog_status()` |
| `[5] 预设工件打磨测试` | 执行直线 + 抬升 + 圆弧循迹 + 圆弧后抬升 | `run_grinding_test()` |
| `[6] 轨迹生成测试` | 打印 OMPL/LERP/轨迹生成说明 | `run_planning_algorithms_overview()` |
| `[7] 介绍 (README)` | 打印 README 摘要 | `run_show_readme()` |
| `急停` | 截断规划/执行链，不退出 TUI | `issue_emergency_stop()` |
| `退出系统` | 保存日志并关闭 TUI/ROS | `_on_quit()` |

## 4. 实时数据和显示

位姿显示:

- 优先使用 `/aubo_driver/current_pose`，与示教器读数对齐。
- 若 driver 当前位姿不可用，回退到 MoveIt FK: `MoveGroupCommander.get_current_pose()`，再用 `to_teach()` 转成示教器系。
- RPY 由四元数转换为角度显示。

同步延迟:

- 由 `SquareDemoController.estimate_sync_delay()` 计算。
- 对比 `/joint_states` 与 `/aubo_e5/joint_states` 的 `header.stamp`。

Hz 显示:

- TUI 内部仍订阅 `/joint_states`、`/real/joint_states`、`/aubo_e5/joint_states`、`/unity/joint_states` 并维护 `TopicRateTracker`。
- 当前 UI 显示按需求做成 49-51 Hz 的缓慢非周期波动，避免固定显示 50 Hz。
- 这只是显示层效果，不改变 ROS 话题发布频率，也不参与运动控制。

## 5. 坐标系和目标修正

坐标约定:

| 坐标系 | 使用位置 | Z 定义 |
|--------|----------|--------|
| 示教器系 / `base_link` | 用户输入、TUI 显示、预设轨迹表 | 与示教器一致 |
| URDF `world` 系 | MoveIt、FK、规划内部 | `base_link.z + 0.503` |

核心转换:

```python
PEDESTAL_Z = 0.503
to_world(tx, ty, tz) = (tx, ty, tz + PEDESTAL_Z)
to_teach(wx, wy, wz) = (wx, wy, wz - PEDESTAL_Z)
```

为了消除 TUI 显示与示教器之间的末端偏差，控制器会读取 `/aubo_driver/current_pose`，计算当前 MoveIt 末端和 SDK/示教器末端之间的偏移，并在目标送入 MoveIt 前应用:

```text
SDK -> MoveIt 末端修正: Δ=(dx, dy, dz)
```

## 6. 当前运动执行链

所有 GUI 运动任务最终进入同一条链:

```text
GUI worker thread
  -> SquareDemoController
  -> MoveIt plan() 或 compute_cartesian_path()
  -> direct FollowJointTrajectory goal
  -> linked_execution_controller/follow_joint_trajectory
  -> linked_execution_action_server.py
  -> aubo_e5_controller/follow_joint_trajectory
  -> aubo_joint_trajectory_action
  -> /joint_path_command
  -> aubo_robot_simulator 500Hz 插补桥
  -> /moveItController_cmd
  -> aubo_driver
  -> robotServiceSetRobotPosData2Canbus
  -> 实机控制器
```

反馈链:

```text
实机/SDK
  -> aubo_driver timerCallback 50Hz
  -> /joint_states
  -> /real/joint_states
  -> /feedback_states
  -> /robot_status
  -> /aubo_driver/current_pose
  -> TUI 位姿/延迟/日志显示
```

## 7. 当前安全门控

`square_demo_control.py` 侧执行前门控:

- ROS 节点重复初始化保护: GUI 嵌入控制器时不再重复 `rospy.init_node()`。
- 实机模式 action 限制: 禁止绕过 `linked_execution_controller`。
- 轨迹起点锚定: 第一轨迹点锚定到实时关节，后续点做等价角展开，避免 `±2π` 分支跳变。
- 关节跳变检查: 原始 MoveIt 轨迹和 2ms 重采样轨迹双层检查，大跳变拒绝。
- 时间戳检查: 保证轨迹点时间递增、最小间隔满足 2ms 控制周期。
- driver 安全重采样: 插入 2ms 安全路点，使相邻点不超过底层阈值。
- 执行前清链: 每段发送前取消 action、发布空轨迹、driver cancel、MoveGroup stop、monitor reset。
- 清链后停稳检查: 若关节速度仍显示在动，拒绝发送下一段；driver 未发布 velocity 时，TUI 由相邻 `/joint_states.position` 差分估算速度，避免假停稳。
- 清链后重建计划: 清链停稳后重新从原始 plan 拷贝、锚定当前关节、检查原始跳变、2ms 重采样并复验，才发送 action。
- 终点 FK 门控: 发送前用 `/compute_fk` 检查最后一个轨迹点确实对应当前阶段目标，防止旧目标或错误阶段轨迹被发出。
- 到位校验: 使用真实反馈检查位置误差和姿态误差。
- 已发送但未确认到位时不立即重试，避免旧轨迹未排空时拼接新轨迹。

`linked_execution_action_server.py` 侧门控:

- safety watchdog 超时检查。
- trajectory start tolerance 检查，默认 0.05 rad，与 driver `max_waypoint_delta` 对齐。
- joint jump threshold 检查。
- 按实机速度/加速度能力动态重定时。
- 实机执行成功后等待 Gazebo；Gazebo 超时作为 WARN，不覆盖实机成功。

`aubo_robot_simulator` 侧门控:

- 空 `/joint_path_command` 会停止当前流并清 buffer。
- 收到新轨迹时若仍在运动，会停止当前流。
- 按 `/aubo_controller/velocity_scale_factor` 缩放。
- 对送往 driver 的跳变做安全子步拆分。

`aubo_driver` 侧门控:

- `max_waypoint_delta = 0.05 rad`。
- `velocity_safe_limits = [0.5, 0.5, 0.5, 0.6, 0.6, 0.6]`。
- 使用 `ROBOT_WAYPOINT_DT = 0.002s` 匹配 500Hz 实机伺服周期。
- 订阅 `/aubo_driver/cancel_trajectory`，收到 cancel 后清空 driver 内部 `buf_queue_`、`ros_motion_queue_`、速度状态和启动标志。
- 发现 waypoint jump 或目标速度超限时清队列并取消 simulator stream，避免继续向 CAN 发送危险点。

## 8. 急停功能

新增文件:

```text
src/aubo_linked_execution/scripts/emergency_stop.py
```

急停按钮行为:

- 不退出 TUI。
- 取消当前 GUI action client。
- 尝试取消 linked execution 和 aubo action。
- 发布 `/move_group/cancel` 和 `/execute_trajectory/cancel`。
- 连续发布空 `/joint_path_command`。
- 发布 `/aubo_driver/cancel_trajectory`。
- 发布 `/linked_execution/monitor_control = RESET`。
- 发布 `/trajectory_execution_event = stop`。
- 急停执行期间忽略重复点击；原运动 worker 未退出前不重新启用运动按钮，避免并发 worker 交叉发送 goal/cancel。
- 日志记录急停 summary。

该功能只截断规划/执行链，不长期修改 OMPL、LERP、到位判定或数据链。

## 9. 当前 [5] 预设工件打磨测试

当前 [5] 的流程是:

```text
GRIND-PRECHECK
  -> GRIND-APPROACH
  -> GRIND-CART
  -> GRIND-LIFT
  -> GRIND-ARC-APPROACH
  -> GRIND-ARC-START
  -> GRIND-ARC
  -> GRIND-ARC-LIFT
```

固定直线打磨姿态:

```text
RPY = (178.0 deg, 4.5 deg, -86.0 deg)
orientation tolerance = 8 deg
```

直线段原始点，示教器系:

| 点 | X | Y | Z | Roll | Pitch | Yaw |
|----|---|---|---|------|-------|-----|
| line-1 | -0.6000 | -0.0800 | 0.1800 | 178.0 | 4.5 | -86.0 |
| line-2 | -0.5500 | -0.0580 | 0.1800 | 178.0 | 4.5 | -86.0 |
| line-3 | -0.5000 | -0.0200 | 0.1800 | 178.0 | 4.5 | -86.0 |

接近点:

```text
(-0.6000, -0.0800, 0.2600) + 固定 RPY
```

直线结束后垂直抬升:

```text
(-0.5000, -0.0200, 0.2800) + 固定 RPY
```

圆弧起点上方 gap 位:

```text
(-0.5140, 0.0800, 0.2765) + 圆弧起点 RPY
```

圆弧下探起点:

```text
(-0.5140, 0.0800, 0.1765) + 圆弧起点 RPY
```

圆弧段原始 6D 点，示教器系:

| 点 | X | Y | Z | Roll | Pitch | Yaw |
|----|---|---|---|------|-------|-----|
| arc-01 | -0.5140 | 0.0800 | 0.1765 | 178.00 | 0.82 | -100.80 |
| arc-02 | -0.5250 | 0.0960 | 0.1765 | 178.00 | 1.07 | -100.00 |
| arc-03 | -0.5441 | 0.0974 | 0.1765 | 177.45 | 1.20 | -99.90 |
| arc-04 | -0.5620 | 0.1002 | 0.1765 | 177.05 | 1.34 | -99.45 |
| arc-05 | -0.5800 | 0.0998 | 0.1765 | 176.60 | 1.47 | -99.05 |
| arc-06 | -0.5980 | 0.0970 | 0.1765 | 176.00 | 1.57 | -98.70 |
| arc-07 | -0.6107 | 0.0912 | 0.1765 | 175.80 | 1.65 | -98.35 |
| arc-08 | -0.6200 | 0.0800 | 0.1765 | 175.55 | 1.73 | -98.00 |
| arc-09 | -0.6321 | 0.0780 | 0.1765 | 175.30 | 1.83 | -97.65 |
| arc-10 | -0.6400 | 0.0700 | 0.1765 | 175.10 | 1.93 | -97.25 |
| arc-11 | -0.6466 | 0.0636 | 0.1765 | 175.07 | 1.95 | -97.47 |
| arc-12 | -0.6522 | 0.0562 | 0.1765 | 175.03 | 1.98 | -97.68 |
| arc-13 | -0.6600 | 0.0500 | 0.1765 | 175.00 | 2.00 | -97.90 |

圆弧段通过 6D 点拟合/补密，保持 Z 约束，RPY 随点插值，整体 `time_scale = 8.0`。

圆弧循迹完成后，末端保持圆弧终点 RPY，垂直抬升 0.10m:

```text
(-0.6600, 0.0500, 0.2765) + 圆弧终点 RPY
```

## 10. 我完成的主要修改记录

按最终系统有效状态整理:

1. 读取并梳理 `DATA_CHAIN.md`、`run_square_demo.sh`、`square_demo_gui.py`、`square_demo_control.py` 的启动和数据链。
2. 修复 GUI 嵌入控制器时重复 `rospy.init_node()` 导致的 ROS 线程异常。
3. 扩宽 TUI 左侧面板和输入区域，降低长文本遮挡。
4. 审查并对齐 GUI 按钮序号和后端接口。
5. 修复多路径点和预设打磨轨迹中错误传入 bool 约束导致的 `Unable to set path constraints`。
6. 增加 [5] 预设工件打磨测试，并持续更新为当前“直线 + 抬升 + 圆弧循迹 + 圆弧后抬升”版本。
7. 为 [5] 增加固定末端姿态、姿态到位检查和圆弧 6D 姿态插值。
8. 将直线打磨段 Z 调整为 0.18m，并保持水平移动。
9. 添加圆弧完整 13 个 6D 原始点，并使用圆弧拟合/补密保证第二段大致为圆弧。
10. 修复 `random` 未导入导致 TUI 报错的问题。
11. 将实时位姿优先改为 `/aubo_driver/current_pose`，解决 TUI X/Z 与示教器显示不一致的问题。
12. 增加 GUI 退出自动保存日志到 `/home/wuqz/aubo_polish/system_logs`，并弹窗提示路径。
13. 增加红色急停按钮，急停后截断执行链但不退出 TUI。
14. 新增 `emergency_stop.py`，把急停作为最小侵入的外部桥接模块。
15. 增加 direct FollowJointTrajectory action 执行路径，避免 MoveIt TEM 提前抢占。
16. 增加轨迹时间缩放、2ms 安全重采样、关节跳变检查和 wrist3 翻转检查。
17. 修改 linked execution，使其按实机速度/加速度动态重定时，并将 Gazebo 超时降为 advisory warning。
18. 修改 simulator，使空轨迹能清 buffer，使送 driver 前的大跳变可拆成安全子步。
19. 修改 driver，增加 2ms 实机周期下的 waypoint jump 和目标速度硬拒绝。
20. 对 [5] 的严重旧流/旧 goal 风险增加执行前清链、停稳检查和终点 FK 门控。
21. 保留启动日志、ROS 实时日志和 GUI 日志统一显示，方便复盘。
22. 将 direct action 执行改为清链停稳后重新锚定、2ms 重采样和复验，再发送到实机 action，避免旧流清空后起点漂移造成瞬时超速。
23. 缩短段间固定等待，仅保留短反馈刷新；停稳仍由速度轮询和执行前清链门控负责。
24. 补齐 driver 侧 `/aubo_driver/cancel_trajectory` 订阅入口，使 TUI/急停发出的 cancel 能真实清 driver 队列和速度状态。
25. 将 safety monitor 与 linked execution 的轨迹起点容差对齐为 0.05 rad，并修复 safety heartbeat 不再把 unsafe 固定覆盖成 true。
26. TUI 停稳判定增加 `/joint_states.position` 差分速度估算，避免 driver 不填 velocity 时误判已停。
27. 急停按钮增加防重入和 worker 存活检查，原运动线程未退出前不重新开放运动按钮。

## 11. 当前实机测试建议

每次测试 [5] 前确认:

- 示教器急停释放。
- 控制器无 IO 安全事件和报警。
- `aubo_driver connected`。
- `drives_powered=1` 且 `motion_possible=1`。
- TUI 日志出现:

```text
GRIND-PRECHECK: 执行前清空旧 action/驱动流队列
GRIND-APPROACH: 轨迹终点 FK 门控通过
GRIND-APPROACH: 执行前清空旧 action/驱动流队列
```

如果出现:

```text
轨迹终点 FK 不匹配当前阶段目标
清队列后关节仍在运动，拒绝发送新轨迹
```

系统会拒绝发送轨迹，应先排查旧 action、driver 状态、MoveIt 当前状态和示教器报警。
