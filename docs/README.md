# AUBO E5 项目文档

> 本目录包含 AUBO E5 联动执行系统的完整技术文档。

---

## 文档目录

### 核心设计

- **[ARCHITECTURE.md](ARCHITECTURE.md)** — 系统架构与数据流（三层架构，Gazebo + Unity 双后端，启动模式矩阵，话题/Action/服务接口）
- **[LINKED_EXECUTION_DESIGN.md](LINKED_EXECUTION_DESIGN.md)** — 联动链路设计原理（设计决策、风险点分析、实现状态）
- **[MOTION_PLANNING.md](MOTION_PLANNING.md)** — 路径规划（OMPL/CHOMP/LERP、连续轨迹、避障、坐标系参考）
- **[SAFETY_SYSTEM.md](SAFETY_SYSTEM.md)** — 安全系统（五层保护、启动对齐、三重检查、看门狗、弹窗报警）

### 操作指南

- **[TEACH_PLAYBACK_GUIDE.md](TEACH_PLAYBACK_GUIDE.md)** — 示教-回放（完整版 `teach_waypoints.py` + 极简版 `simple_teach.py`）
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** — 故障排查（快速速查表、启动/执行/Gazebo/Unity/MoveIt 问题）

### 其他参考

- **[CAMERA_CALIBRATION_GUIDE.md](CAMERA_CALIBRATION_GUIDE.md)** — 相机标定（待集成）

---

## 快速启动命令

```bash
# 实机 + Unity 镜像（推荐）
./start_p6.sh

# 仅 Gazebo 仿真
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch sim_only:=true

# 示教-回放
./run_teach_playback.sh

# 正方形轨迹演示（虚实同步）
./run_square_demo.sh

# 系统诊断
./system_tools.sh diagnose

# 实时关节监控
./system_tools.sh monitor
```

---

## 推荐阅读路径

| 目标 | 阅读文档 |
|------|---------|
| 快速上手 | README.md → OPERATION_GUIDE.md |
| 理解通信链路 | ARCHITECTURE.md |
| 理解设计原理 | LINKED_EXECUTION_DESIGN.md |
| 路径规划与轨迹 | MOTION_PLANNING.md |
| 安全机制 | SAFETY_SYSTEM.md |
| 故障排查 | TROUBLESHOOTING.md |

---

*最后更新：2026-05-11*
