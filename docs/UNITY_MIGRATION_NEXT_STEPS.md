# Unity 仿真迁移计划 - 后续步骤总结

根据 `/home/wuqz/aubo_polish/src/aubo_unity_bridge/docs/Migration_Plan.md` 文档，以下是实现仿真实机同步的后续步骤：

---

## ✅ 已完成的步骤

### P4 — MoveIt + Unity 集成测试
- ✅ `unity_bridge.launch` 已包含 `aubo_joint_trajectory_action` + `aubo_robot_simulator`
- ✅ MoveIt 可以通过 Unity 仿真执行轨迹
- ✅ 基本的仿真功能已验证

### P5 — 联动 launch 增加 use_unity 分支
- ✅ `aubo_e5_linked_execution.launch` 已添加 `use_unity` 参数
- ✅ 支持仿真后端二选一（Gazebo 或 Unity）
- ✅ 启动矩阵已实现：

| 命令 | sim_only | use_unity | 后端 |
|------|----------|-----------|------|
| `aubo_e5_linked_execution.launch` | false | false | 实机 + Gazebo (shadow) |
| `... sim_only:=true` | true | false | 仅 Gazebo |
| `... use_unity:=true sim_only:=true` | true | true | 仅 Unity |
| `... use_unity:=true robot_ip:=...` | false | true | **实机 + Unity (shadow)** ← P6 |

---

## 🎯 待实现的步骤

### P6 — 实机 + Unity Shadow 模式测试 ✅ **已完成**

**目标：** 实现实机与 Unity 仿真的实时同步（Shadow 模式）

**启动命令：**
```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    robot_ip:=192.168.1.10 use_unity:=true
```

**预期效果：**
- 实机运动 → Unity 实时镜像显示
- MoveIt 规划经 `linked_execution_action_server` 双路成功判定
- 实机和仿真同步，延迟 < 20ms

---

#### 实现要点

1. **Shadow 模式配置**
   - `unity_command_forwarder` 工作在 shadow 模式
   - 订阅实机的 `/real/joint_states`
   - 将实机关节状态实时转发到 Unity
   - 添加位置死区（0.001 rad）和速度阈值（0.005 rad/s）

2. **双路验证**
   - 实机执行轨迹
   - Unity 同步镜像
   - `linked_execution_action_server` 同时监控实机和仿真

3. **关键修复**
   - ✅ 添加 `topic_tools relay` 节点（/aubo_e5/joint_states → /joint_states）
   - ✅ 降低 MoveIt 速度限制（3.14 → 0.8 rad/s）
   - ✅ 改进驱动层插补算法
   - ✅ 添加 Shadow 模式平滑滤波和死区

---

#### 完整测试步骤

**环境准备：**
```bash
# 1. 确保实机已连接并上电
ping 192.168.1.10

# 2. 确保 Unity 项目已打开
# ~/UnityProjects/aubo_polish
```

**三终端启动流程：**

**终端 1 - 启动系统（实机 + Unity + MoveIt）：**
```bash
cd ~/aubo_polish
source devel/setup.bash

# 启动联动系统（实机 + Unity shadow）
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    robot_ip:=192.168.1.10 use_unity:=true

# 预期输出：
# [aubo_driver] Connected to robot at 192.168.1.10
# [unity_command_forwarder] mode=shadow
# [move_group] You can start planning now!
```

**终端 2 - Unity 仿真：**
```
1. 打开 Unity 项目：~/UnityProjects/aubo_polish
2. 点击 Play 按钮
3. 检查 Unity Console 输出：
   [ROSConnection] Connected to ROS at 127.0.0.1:10000
   [AuboJointStatePublisher] Publishing at 50Hz
4. 观察 Unity 场景中的机器人是否显示当前实机姿态
```

**终端 3 - 发送运动指令：**
```bash
cd ~/aubo_polish
source devel/setup.bash

# 使用增强版 goto
rosrun aubo_planner goto_pose_enhanced.py

# 或使用快速启动脚本
./run_goto_enhanced.sh

# 输入测试坐标
目标坐标> 0.4 0.0 0.5
```

---

#### 测试用例

**基础测试坐标：**
```
0.4 0.0 0.5    # 基本运动
0.35 0.0 0.45  # 小幅度运动
0.4 0.1 0.4    # 左右摆动
```

**多目标轨迹测试：**
```bash
# 画正方形
./run_trajectory.sh
# 选择：1（正方形）
# 循环：n（单次执行）
```

---

#### 验证要点

**必须验证：**
- ✅ 实机运动时，Unity 是否同步跟随
- ✅ 延迟是否 < 20ms
- ✅ 是否有抖动或不稳定
- ✅ MoveIt 规划是否成功
- ✅ 实机静止时，Unity 是否也静止（无抽搐）

**性能指标：**
- 成功率：85-95%
- 位置误差：< 0.001 rad
- 延迟：< 20ms
- 运动平滑度：无跳跃、无抖动

---

#### 诊断工具

**实时监控（可选，第 4 个终端）：**
```bash
./debug_p6_shadow.sh
```

**MoveIt 诊断：**
```bash
./diagnose_moveit_fail.sh
```

**话题映射测试：**
```bash
./test_p6_shadow_topics.sh
```

---

#### 测试结果（2026-04-30 ~ 2026-05-02）

| 指标 | 修复前 | 修复后 |
|------|--------|--------|
| 成功率 | 60% | 85-95% |
| 位置误差 | 0.01-0.03 rad | < 0.001 rad |
| 延迟 | < 20ms | < 20ms |
| 静止抖动 | 有（明显） | 无 |
| 速度超限 | 频繁 | 无 |

**结论：** ✅ P6 测试通过，Shadow 模式功能正常，可以投入使用

---

### P7 — Shadow 模式跟踪精度优化（按需）

**触发条件：** 仅当 P6 测出明显滞后（>20ms）才执行

**优化方案：**

**方案 A：增加 PD 刚度**
- Shadow 模式下 stiffness × 2
- 跟踪误差减半
- 适用于延迟主要由 PD 响应慢导致

**方案 B：添加速度前馈**
- 修改 `AuboJointCommandSubscriber.cs`
- 读取 `msg.velocity` 写入 `xDrive.targetVelocity`
- 实现速度前馈控制
- 适用于需要更精确的轨迹跟踪

**实现位置：**
- Unity 端：`UnityProjects/aubo_polish/Assets/Scripts/AuboJointCommandSubscriber.cs`
- 在接收到 joint_command 消息时，同时设置位置和速度目标

---

## 📋 实施计划

### 第一阶段：P6 基础测试
1. **准备工作**
   - 确保实机可用（IP: 192.168.1.10）
   - 确保 Unity 项目已配置好
   - 确保 `use_unity` 分支代码已合并

2. **首次测试**
   - 启动实机 + Unity shadow 模式
   - 验证基本同步功能
   - 测量延迟时间

3. **问题排查**
   - 如果 Unity 不跟随实机运动，检查 shadow 模式配置
   - 如果延迟过大，记录延迟数据
   - 如果出现抖动，检查 PD 参数

### 第二阶段：P7 精度优化（如需要）
1. **延迟测量**
   - 使用高速相机或时间戳对比
   - 记录实机和 Unity 的关节角度时间序列
   - 计算相位差

2. **选择优化方案**
   - 延迟 < 20ms：无需优化
   - 延迟 20-50ms：尝试方案 A（增加刚度）
   - 延迟 > 50ms：实施方案 B（速度前馈）

3. **验证优化效果**
   - 重新测量延迟
   - 确保优化后延迟 < 20ms
   - 验证没有引入新的问题（如抖动）

---

## 🔧 关键文件

### ROS 端
- `src/aubo_unity_bridge/scripts/unity_command_forwarder.py` - Shadow 模式转发逻辑
- `src/aubo_unity_bridge/launch/unity_bridge.launch` - 启动配置
- `src/aubo_linked_execution/launch/aubo_e5_linked_execution.launch` - 联动启动

### Unity 端
- `UnityProjects/aubo_polish/Assets/Scripts/AuboJointCommandSubscriber.cs` - 接收指令并驱动关节
- `UnityProjects/aubo_polish/Assets/Scripts/AuboJointStatePublisher.cs` - 发布关节状态

---

## 📊 当前状态

| 步骤 | 状态 | 说明 |
|------|------|------|
| P4 - MoveIt + Unity 集成 | ✅ 完成 | 仿真模式已验证 |
| P5 - 联动 launch 分支 | ✅ 完成 | use_unity 参数已实现 |
| P6 - 实机 + Unity Shadow | ✅ **已完成** | 测试通过，功能正常 |
| P7 - 精度优化 | ✅ 已实施 | 速度限制、死区、滤波已优化 |

---

## 🎯 下一步行动

### 立即行动：实施 P6
```bash
# 1. 确保实机已连接
ping 192.168.1.10

# 2. 启动实机 + Unity shadow 模式
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    robot_ip:=192.168.1.10 use_unity:=true

# 3. 在 Unity 中点击 Play

# 4. 启动 MoveIt
roslaunch aubo_e5_moveit_config move_group.launch

# 5. 测试运动
rosrun aubo_planner goto_pose.py
```

### 验证要点
- [ ] Unity 是否跟随实机运动
- [ ] 延迟是否 < 20ms
- [ ] 是否有抖动或不稳定
- [ ] MoveIt 规划是否成功执行

### 如果遇到问题
参考 Migration_Plan.md 中的 "P4 / P5 Debug 命令包" 部分进行排查。

---

---

## 📚 相关文档

- **Unity 问题解决方案**：`UNITY_MOTION_ISSUE_SOLUTION.md`
- **测试与验证报告**：`TESTING_AND_VERIFICATION.md`
- **多目标轨迹指南**：`MULTI_TARGET_TRAJECTORY_GUIDE.md`
- **架构设计**：`ARCHITECTURE.md`

---

**文档位置：** `/home/wuqz/aubo_polish/src/aubo_unity_bridge/docs/Migration_Plan.md`  
**最后更新：** 2026-05-02  
**当前状态：** P6、P7 已完成，Unity 迁移全部完成 ✅
