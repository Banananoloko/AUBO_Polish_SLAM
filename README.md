# AUBO E5 联动系统

> RViz → 实机 → Gazebo/Unity 三层同步执行的完整机器人控制系统

**支持仿真后端**: Gazebo (原生) + Unity (新增)

**当前版本基线**: 2026-05-25。当前有效结构、按钮映射、[5] 预设打磨轨迹、安全门控和我完成的修改记录，见 [CURRENT_SYSTEM_SUMMARY.md](CURRENT_SYSTEM_SUMMARY.md)。若本文旧段落与该文件冲突，以 `CURRENT_SYSTEM_SUMMARY.md` 为准。

---

## 项目概述

本项目实现了 AUBO E5 协作机器人的可视化规划、实机执行和仿真镜像的完整链路，支持：

- ✅ **RViz 可视化规划**：MoveIt 交互式路径规划（OMPL 22种算法 + CHOMP优化）
- ✅ **路径规划与避障**：完整的碰撞检测（自碰撞 + 环境碰撞 + 关节限位）
- ✅ **安全监控系统**：启动位置验证 + 大幅度运动检测 + 轨迹起点验证
- ✅ **实机高频控制**：500Hz AUBO SDK 实时控制
- ✅ **双仿真后端**：Gazebo（原生）+ Unity（高保真渲染）
- ✅ **实时镜像**：仿真环境同步显示实机状态
- ✅ **双重成功判定**：实机到位 + 仿真收敛双重确认
- ✅ **自动上电启动**：无需手动操作示教器，自动位置同步验证
- ✅ **视觉扩展接口**：预留相机感知和场景建模接口
- ✅ **图形 TUI 控制**：按钮式正方形、自定义位姿、多路径点、预设打磨、安全审查和日志保存
- ✅ **预设打磨测试**：当前 [5] 为直线 0.18m Z 水平循迹 + 0.10m gap 抬升 + 13 点圆弧 6D 循迹
- ✅ **执行链截断急停**：TUI 内红色急停按钮可取消 action、清空轨迹流、driver cancel，且不退出 TUI
- ✅ **轨迹发送前门控**：清旧队列、停稳检查、起点检查、时间戳检查、2ms 重采样、终点 FK 目标一致性检查

---

## 快速开始

### 环境要求
- Ubuntu 20.04 + ROS Noetic
- AUBO E5 机器人（控制器版本 V4+）
- 已编译的 catkin 工作空间

### 一键启动

```bash
# 推荐入口：一键启动脚本（自动启动 ROS + 图形 TUI）
cd ~/aubo_polish
./run_square_demo.sh                  # Gazebo 仿真 + 图形 TUI
./run_square_demo.sh --real <ip>     # 实机 + Gazebo shadow + 图形 TUI
./run_square_demo.sh --unity          # Unity 仿真 + 图形 TUI
./run_square_demo.sh --real <ip> --unity  # 实机 + Unity shadow + 图形 TUI

# 手动 roslaunch（仅启动 ROS，不启动图形 TUI）
source devel/setup.bash

# 联动模式（实机 + Gazebo 镜像）
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    robot_ip:=192.168.10.230

# 仿真模式（仅 Gazebo，无需实机）
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    sim_only:=true
```

### 基本操作

图形 TUI 是当前推荐入口。RViz 仍可用于观察和交互式规划，但常规测试通过 TUI 按钮完成。

| TUI 按钮 | 当前功能 |
|----------|----------|
| `[1]` | 执行正方形轨迹 |
| `[2]` | 自定义目标位姿，支持 `x y z [roll pitch yaw]` |
| `[3]` | 多路径点连续笛卡尔轨迹 |
| `[4]` | 安全审查状态 |
| `[5]` | 预设工件打磨测试: 直线 + gap 抬升 + 圆弧循迹 |
| `[6]` | 轨迹生成测试 / OMPL 与 LERP 说明 |
| `[7]` | 介绍 README |
| `急停` | 截断规划/执行链，不退出 TUI |

RViz 交互式操作仍可按旧流程使用:

1. 在 RViz 中拖动交互式标记到目标位置。
2. 点击 "Plan" 生成路径。
3. 点击 "Execute" 执行运动。
4. 观察实机运动和 Gazebo/Unity shadow 镜像。

---

## 系统架构

### 三层架构设计

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
│  • Gazebo 监视：收敛检测                 │
│  • 状态适配：关节重排                    │
│  • 自动上电：模式切换                    │
└──────┬──────────────────────┬───────────┘
       │                      │
       ↓                      ↓
┌──────────────────┐   ┌──────────────────┐
│  实机执行链       │   │  Gazebo 镜像链    │
│  • 轨迹跟踪       │──→│  • Shadow 模式    │
│  • AUBO SDK      │   │  • 实时同步       │
│  • 500Hz 控制    │   │  • 位置控制       │
└──────────────────┘   └──────────────────┘
```

### 核心组件

#### 1. 联动聚合层 (`aubo_linked_execution`)

| 组件 | 功能 |
|------|------|
| `linked_execution_action_server.py` | 协调中枢，接收 MoveIt 目标，集成安全检查，等待双重成功确认 |
| `safety_monitor.py` | 安全监控器（心跳 + 大幅度运动检测 + 轨迹起点验证 + 手动移动检测） |
| `linked_execution_monitor.py` | Gazebo 收敛监视器，检测是否到达目标位置 |
| `joint_state_mirror_adapter.py` | 关节状态重排适配器，确保顺序一致 |
| `aubo_robot_startup.py` | 实机自动上电、控制模式切换和位置同步验证 |

#### 2. 实机执行链 (`aubo_robot`)

| 组件 | 功能 |
|------|------|
| `aubo_joint_trajectory_action` | Action Server，接收轨迹并监控执行 |
| `aubo_robot_simulator` | 轨迹插补桥，500Hz 插补轨迹点 |
| `aubo_driver` | AUBO SDK 驱动，500Hz 实时控制 |

#### 3. Gazebo 镜像链 (`aubo_gazebo`)

| 组件 | 功能 |
|------|------|
| `aubo_gazebo_driver` | Shadow 模式驱动，镜像实机状态到 Gazebo |
| Gazebo Controllers | 6 个单关节位置控制器 |

---

## 包结构

```
src/
├── aubo_linked_execution/       # 【核心】联动执行包
│   ├── scripts/                 # 18 个 Python 节点/工具
│   │   ├── linked_execution_action_server.py  # 联动聚合层
│   │   ├── safety_monitor.py                 # 安全监控
│   │   ├── linked_execution_monitor.py       # Gazebo 收敛监视
│   │   ├── joint_state_mirror_adapter.py     # 关节状态重排
│   │   ├── aubo_robot_startup.py             # 自动上电
│   │   ├── square_demo_gui.py                # 图形 TUI 控制端
│   │   ├── square_demo_control.py            # 核心轨迹控制器
│   │   ├── gazebo_rtf_monitor.py             # Gazebo RTF 监控
│   │   ├── continuous_motion_demo.py         # 连续运动演示
│   │   ├── obstacle_spawner.py               # 障碍物生成
│   │   ├── test_obstacle_avoidance.py        # 避障测试
│   │   ├── teach_waypoints.py / simple_teach.py / playback_waypoints.py / simple_playback.py  # 示教-回放
│   │   ├── unity_sync_monitor.py / unity_test_controller.py  # Unity 桥接
│   │   └── alert_dialog.py / alert_dialog_demo.py  # 弹窗工具
│   ├── launch/                  # 统一启动入口
│   └── config/                  # MoveIt 控制器配置
│
├── aubo_robot/                  # AUBO 机器人核心包
│   ├── aubo_driver/             # 实机驱动 + Gazebo 驱动
│   ├── aubo_controller/         # 轨迹跟踪控制器
│   ├── aubo_gazebo/             # Gazebo 仿真配置
│   ├── aubo_e5_moveit_config/   # MoveIt 配置（OMPL + CHOMP）
│   ├── aubo_description/        # URDF 机器人模型
│   └── aubo_msgs/               # 自定义消息/服务
│
├── aubo_unity_bridge/           # 【新增】Unity 桥接包
│   ├── scripts/                 # 4 个 Python 节点
│   ├── launch/                  # Unity 启动配置
│   ├── config/                  # 话题契约
│   └── docs/                    # 迁移文档
│
├── industrial_core/             # ROS-Industrial 核心库
└── mecheye_ros_interface/       # Mech-Eye 相机接口（待集成）
```

---

## 关键话题

| 话题名称 | 频率 | 说明 |
|---------|------|------|
| `/joint_states` | 50Hz | 实机当前关节状态 (driver timerCallback) |
| `/joint_path_command` | 按需 | MoveIt 下发的轨迹 (JointTrajectory) |
| `/moveItController_cmd` | 500Hz | 插补后的轨迹点 (JointTrajectoryPoint) |
| `/feedback_states` | 50Hz | 实机执行反馈 |
| `/robot_status` | 50Hz | 机器人状态（急停、报警） |
| `/real/joint_states` | 50Hz | 镜像适配后的实机状态 |
| `/aubo_e5/joint_states` | Gazebo 更新率 | Gazebo 当前关节状态 |
| `/linked_execution/monitor_status` | 10Hz | Gazebo 收敛状态 |
| `/safety_monitor/safe_to_execute` | 2s 心跳 | 执行安全标志 |
| `/safety_monitor/warning` | 按需 | 安全警告信息 |
| `/gazebo_rtf_monitor/warning` | 按需 | Gazebo RTF 性能预警 |

---

## 技术特色

### 1. 路径规划与碰撞检测
- **OMPL 规划器**：22种采样算法（RRTConnect、RRTstar、PRM、KPIECE等）
- **CHOMP 优化器**：梯度优化，平滑轨迹，避障能力强
- **碰撞检测**：自碰撞 + 环境碰撞 + 关节限位
- **配置文件**：`aubo_e5_moveit_config/config/ompl_planning.yaml`

### 2. 连续运动规划
- **多模式支持**：关节空间、笛卡尔空间、预定义位置三种模式
- **配置驱动**：从 YAML 文件读取路径点，无需代码修改
- **循环执行**：支持连续循环运动
- **详细日志**：每个路径点的执行状态和耗时统计

### 3. 避障测试系统
- **随机障碍物生成**：Gazebo + Planning Scene 同步
- **自动化测试**：批量测试避障规划成功率和耗时
- **可视化验证**：RViz 显示规划路径和障碍物
- **视觉集成预留**：支持后期接入相机视觉系统

### 4. 安全监控系统
- **启动位置验证**：确保 RViz 与实机位置同步（容差 0.05 rad）
- **大幅度运动检测**：阈值 0.5 rad (28.6°)，防止危险运动
- **轨迹起点验证**：容差 0.15 rad (8.6°)，执行前检查
- **执行阻断机制**：不安全时自动阻止轨迹执行

### 5. 双重成功判定
- **实机到位**：aubo_driver 反馈确认轨迹执行完成
- **Gazebo 收敛**：monitor 确认 Gazebo 模型到达目标（软警告模式：实机成功即返回 SUCCESS，Gazebo 超时仅 WARN）
- **容错机制**：实机失败则整体 ABORT

### 6. 关节状态重排
- 解决 aubo_driver 关节顺序不一致问题
- 确保 Gazebo shadow 模式稳定镜像

### 7. 反馈隔离机制
- 联动模式下隔离模拟器反馈
- 确保 Action Server 只收到实机真实反馈

### 8. 自动上电流程
- 无需手动操作示教器
- 自动完成 powerOn + 控制模式切换 + 位置同步验证

### 9. 时间跳变保护
- 支持 Gazebo pause/reset 操作
- 避免 shadow 模式失效

---

## 工具脚本

当前根目录只保留一个推荐启动入口:

```bash
./run_square_demo.sh --help
./run_square_demo.sh
./run_square_demo.sh --real 192.168.10.230
```

旧的诊断、示教回放和临时启动脚本已经从根目录清理，避免入口分裂。

---

## 安全注意事项

1. **首次使用**：先在仿真模式测试 (`sim_only:=true`)
2. **实机测试**：确保机器人周围 1 米内无障碍物和人员
3. **急停准备**：操作人员手放在急停按钮附近
4. **监控日志**：密切关注终端的安全警告信息
5. **小幅度测试**：首次运行建议使用小幅度运动测试

详细安全链说明见 `CURRENT_SYSTEM_SUMMARY.md` 和 `DATA_CHAIN.md`。

---

## 文档导航

| 文档 | 用途 |
|------|------|
| **README.md**（本文档） | 项目架构、快速开始、系统概览 |
| **CURRENT_SYSTEM_SUMMARY.md** | 当前最终版本权威快照、按钮映射、[5] 点位、安全门控和修改记录 |
| **DATA_CHAIN.md** | 完整数据链：用户输入→实机反馈全链路 |
| **src/aubo_unity_bridge/README.md** | Unity 桥接包文档 |

**推荐阅读路径**：
- **新手**：README.md → run_square_demo.sh --help
- **实机操作**：CURRENT_SYSTEM_SUMMARY.md → DATA_CHAIN.md
- **系统设计**：CURRENT_SYSTEM_SUMMARY.md → DATA_CHAIN.md
- **故障排查**：DATA_CHAIN.md 中的诊断和安全门控章节

---

## 技术支持

**项目路径**: `/home/wuqz/aubo_polish`
**ROS 版本**: Noetic
**机器人型号**: AUBO E5 (6-DOF)
**控制器版本**: V4+

**遇到问题？**
1. 查看当前版本快照：`CURRENT_SYSTEM_SUMMARY.md`
2. 查看数据链文档：`DATA_CHAIN.md`
3. 查看 TUI 日志窗口和 `/home/wuqz/aubo_polish/system_logs/` 中的自动保存日志
