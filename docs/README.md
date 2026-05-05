# AUBO E5 项目文档

> 本目录包含 AUBO E5 联动执行系统的完整技术文档。

---

## 📚 文档导航

### 核心设计文档

- **[ARCHITECTURE.md](ARCHITECTURE.md)** - 系统架构与数据流（三层架构，支持 Gazebo + Unity 双后端）
- **[LINKED_EXECUTION_DESIGN.md](LINKED_EXECUTION_DESIGN.md)** - 联动链路设计（实机+仿真同步原理）
- **[MOTION_PLANNING.md](MOTION_PLANNING.md)** - 路径规划与碰撞检测（MoveIt + OMPL）
- **[SAFETY_SYSTEM.md](SAFETY_SYSTEM.md)** - 安全监控机制（启动对齐、碰撞检测）

### 使用指南

- **[MULTI_TARGET_TRAJECTORY_GUIDE.md](MULTI_TARGET_TRAJECTORY_GUIDE.md)** - 多目标轨迹规划使用指南（含坐标系定义、手眼标定）
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - 故障排查指南（快速问题速查表）

### Unity 迁移

- **[UNITY_MIGRATION_NEXT_STEPS.md](UNITY_MIGRATION_NEXT_STEPS.md)** - Unity 迁移计划（P4-P7 完整实施步骤）
- **[UNITY_MOTION_ISSUE_SOLUTION.md](UNITY_MOTION_ISSUE_SOLUTION.md)** - Unity 问题诊断与解决方案

### 测试与验证

- **[TESTING_AND_VERIFICATION.md](TESTING_AND_VERIFICATION.md)** - 测试与验证报告（代码修改验证、日志分析）

---

## 🚀 快速开始

### 联动模式（实机 + Gazebo）
```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.1.10
```

### 仿真模式（仅 Gazebo）
```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch sim_only:=true
```

### Unity Shadow 模式（实机 + Unity）✅ 推荐
```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    robot_ip:=192.168.1.10 use_unity:=true
```

### 多目标轨迹
```bash
# 使用快速启动脚本
./run_trajectory.sh

# 或直接执行
rosrun aubo_linked_execution continuous_motion_demo.py \
    --mode cartesian \
    --config square_motion.yaml
```

---

## 📖 文档分类

### 按主题分类

**系统设计（3个）：**
- ARCHITECTURE.md - 系统架构
- LINKED_EXECUTION_DESIGN.md - 联动设计
- MOTION_PLANNING.md - 路径规划

**安全与故障排查（2个）：**
- SAFETY_SYSTEM.md - 安全监控
- TROUBLESHOOTING.md - 故障排查

**使用指南（1个）：**
- MULTI_TARGET_TRAJECTORY_GUIDE.md - 多目标轨迹（含坐标系定义）

**Unity 迁移（2个）：**
- UNITY_MIGRATION_NEXT_STEPS.md - 迁移计划
- UNITY_MOTION_ISSUE_SOLUTION.md - 问题解决方案

**测试与验证（1个）：**
- TESTING_AND_VERIFICATION.md - 测试报告

---

## 🔍 常见问题

### Q: 如何选择仿真后端？

**A**: 使用 `use_unity` 参数：
- `use_unity:=false`（默认）- 使用 Gazebo
- `use_unity:=true` - 使用 Unity（推荐）

### Q: 如何排查启动失败？

**A**: 参考 [TROUBLESHOOTING.md](TROUBLESHOOTING.md) 的快速问题速查表。

### Q: 如何理解系统架构？

**A**: 阅读 [ARCHITECTURE.md](ARCHITECTURE.md) 了解三层架构设计。

### Q: 如何实现多目标轨迹？

**A**: 参考 [MULTI_TARGET_TRAJECTORY_GUIDE.md](MULTI_TARGET_TRAJECTORY_GUIDE.md) 的完整指南。

### Q: Unity 仿真有什么问题？

**A**: 参考 [UNITY_MOTION_ISSUE_SOLUTION.md](UNITY_MOTION_ISSUE_SOLUTION.md) 了解已解决的问题。

---

## 📊 文档统计

- **总文档数**：9 个
- **核心设计**：4 个
- **使用指南**：2 个
- **Unity 相关**：2 个
- **测试验证**：1 个

---

*最后更新：2026-05-02*
*项目路径：`/home/wuqz/aubo_polish`*
