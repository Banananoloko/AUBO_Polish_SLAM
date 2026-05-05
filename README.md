# AUBO E5 联动系统

> RViz → 实机 → Gazebo/Unity 三层同步执行的完整机器人控制系统

**支持仿真后端**: Gazebo (原生) + Unity (新增)

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

---

## 快速开始

### 环境要求
- Ubuntu 20.04 + ROS Noetic
- AUBO E5 机器人（控制器版本 V4+）
- 已编译的 catkin 工作空间

### 一键启动

```bash
# 1. 进入工作空间
cd ~/aubo_polish
source devel/setup.bash

# 2. 联动模式（实机 + Gazebo 镜像）
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    robot_ip:=192.168.1.10

# 3. 仿真模式（仅 Gazebo，无需实机）
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    sim_only:=true

# 4. Unity 仿真模式（仅 Unity，无需实机）
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    use_unity:=true sim_only:=true

# 5. 实机 + Unity 镜像模式
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    robot_ip:=192.168.1.10 use_unity:=true
```

### 基本操作

1. **设置目标**：在 RViz 中拖动交互式标记到目标位置
2. **规划轨迹**：点击 "Plan" 按钮生成路径
3. **执行运动**：点击 "Execute" 按钮
4. **观察结果**：实机运动，Gazebo 同步镜像

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
| `safety_monitor.py` | 安全监控器，检测大幅度运动和轨迹起点不匹配 |
| `linked_execution_monitor.py` | Gazebo 收敛监视器，检测是否到达目标位置 |
| `joint_state_mirror_adapter.py` | 关节状态重排适配器，确保顺序一致 |
| `aubo_robot_startup.py` | 实机自动上电、控制模式切换和位置同步验证 |

#### 2. 实机执行链 (`aubo_robot`)

| 组件 | 功能 |
|------|------|
| `aubo_joint_trajectory_action` | Action Server，接收轨迹并监控执行 |
| `aubo_robot_simulator.py` | 轨迹插补桥，200Hz 插补轨迹点 |
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
│   ├── scripts/                 # 5 个 Python 节点
│   │   ├── linked_execution_action_server.py
│   │   ├── safety_monitor.py    # 【新增】安全监控
│   │   ├── linked_execution_monitor.py
│   │   ├── joint_state_mirror_adapter.py
│   │   └── aubo_robot_startup.py
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
| `/joint_states` | 500Hz | 实机当前关节状态 |
| `/joint_path_command` | 按需 | MoveIt 下发的轨迹 |
| `/moveItController_cmd` | 200Hz | 插补后的轨迹点 |
| `/feedback_states` | 10Hz | 实机执行反馈 |
| `/robot_status` | 10Hz | 机器人状态（急停、报警） |
| `/real/joint_states` | 500Hz | 镜像适配后的实机状态 |
| `/aubo_e5/joint_states` | 1000Hz | Gazebo 当前关节状态 |
| `/linked_execution/monitor_status` | 10Hz | Gazebo 收敛状态 |
| `/safety_monitor/warning` | 按需 | 安全警告信息 |
| `/safety_monitor/safe_to_execute` | 按需 | 执行安全标志 |

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
- **启动位置验证**：确保 RViz 与实机位置同步（容差 0.01 rad）
- **大幅度运动检测**：阈值 0.5 rad (28.6°)，防止危险运动
- **轨迹起点验证**：容差 0.15 rad (8.6°)，执行前检查
- **执行阻断机制**：不安全时自动阻止轨迹执行

### 5. 双重成功判定
- **实机到位**：aubo_driver 反馈确认轨迹执行完成
- **Gazebo 收敛**：monitor 确认 Gazebo 模型到达目标
- **容错机制**：任一失败则整体 ABORT

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

### system_tools.sh - 系统管理工具
```bash
# 验证系统完整性
./system_tools.sh verify

# 诊断运行时状态
./system_tools.sh diagnose

# 修复 AUBO SDK 依赖
./system_tools.sh fix-rpath
```

---

## 安全注意事项

1. **首次使用**：先在仿真模式测试 (`sim_only:=true`)
2. **实机测试**：确保机器人周围 1 米内无障碍物和人员
3. **急停准备**：操作人员手放在急停按钮附近
4. **监控日志**：密切关注终端的安全警告信息
5. **小幅度测试**：首次运行建议使用小幅度运动测试

详细安全说明见 `SAFETY_SYSTEM.md`

---

## 文档导航

| 文档 | 用途 |
|------|------|
| **README.md**（本文档） | 项目架构、快速开始、系统概览 |
| **OPERATION_GUIDE.md** | 实机操作、连续规划、故障排查 |
| **docs/ARCHITECTURE.md** | 系统架构、数据流、Gazebo + Unity 对比 |
| **docs/UNITY_MIGRATION.md** | Unity 迁移指南、调试命令 |
| **docs/MOTION_PLANNING.md** | 路径规划算法、参数配置 |
| **docs/SAFETY_SYSTEM.md** | 安全机制详解、参数配置 |
| **docs/TROUBLESHOOTING.md** | 常见问题快速排查 |
| **docs/LINKED_EXECUTION_DESIGN.md** | 联动设计原理 |
| **docs/PACKAGE_STRUCTURE.md** | 包结构详解 |
| **src/aubo_unity_bridge/README.md** | Unity 桥接包文档 |

**推荐阅读路径**：
- **新手**：README.md → OPERATION_GUIDE.md 快速参考
- **实机操作**：OPERATION_GUIDE.md
- **系统设计**：docs/ARCHITECTURE.md → docs/LINKED_EXECUTION_DESIGN.md
- **路径规划**：docs/MOTION_PLANNING.md
- **Unity 集成**：docs/UNITY_MIGRATION.md → src/aubo_unity_bridge/README.md
- **故障排查**：docs/TROUBLESHOOTING.md

---

## 技术支持

**项目路径**: `/home/wuqz/aubo_polish`  
**ROS 版本**: Noetic  
**机器人型号**: AUBO E5 (6-DOF)  
**控制器版本**: V4+

**遇到问题？**
1. 运行诊断工具：`./system_tools.sh diagnose`
2. 查看操作指南：`OPERATION_GUIDE.md`
3. 查看设计文档：`src/RViz_Real_Gazebo_Linked_Execution_Design.md`
