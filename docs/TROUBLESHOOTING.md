# 故障排查指南

> 常见问题快速定位和解决方案

**最后更新**: 2026-04-28

---

## 1. 快速问题速查表

| 现象 | 大概率原因 | 快速解决 |
|------|------------|---------|
| 启动超时 | 示教器急停未释放 | 检查示教器急停按钮 |
| 网络连接失败 | 机器人 IP 不通 | `ping 192.168.1.10` |
| 大幅度运动警告 | RViz 位置与实机不一致 | 检查启动位置同步 |
| Execute 失败 | 安全检查未通过 | 查看 `/safety_monitor/warning` |
| Gazebo 不同步 | 镜像适配器未运行 | 检查 `/real/joint_states` 频率 |
| MoveIt Plan OK，Execute 一直转圈 | 动作服务器没收到 feedback_states | `rostopic hz /feedback_states` |
| Unity 启动后立刻塌成乱姿 | xDrive PD 没写入 | Unity Console 找 `init shoulder_joint:` 行 |
| `/aubo_e5/joint_states` 50Hz 但 RViz 不动 | TF 没起或 robot_description 不一致 | `rostopic echo /tf` + `rosparam get /robot_description` |

---

## 2. 启动问题

### 2.1 启动超时

**现象**：
```
[aubo_robot_startup] Timeout waiting for drives_powered
```

**排查步骤**：
1. 检查示教器：急停是否释放？
2. 检查示教器：是否有报警信息？
3. 手动上电测试：
   ```bash
   rostopic pub /robot_control std_msgs/String "data: 'powerOn'" -1
   rostopic echo /robot_status
   ```
4. 检查网络连接：
   ```bash
   ping 192.168.1.10
   ```

**解决方案**：
- 释放示教器急停按钮
- 清除示教器报警信息
- 检查网络连接和防火墙

---

### 2.2 网络连接失败

**现象**：
```
[aubo_driver] Failed to connect to robot at 192.168.1.10
```

**排查步骤**：
1. 测试网络连通性：
   ```bash
   ping 192.168.1.10
   ```
2. 检查防火墙：
   ```bash
   sudo ufw status
   # 如启用，添加规则：
   sudo ufw allow from 192.168.10.0/24
   ```
3. 查看驱动日志：
   ```bash
   rosnode info /aubo_driver
   rosparam get /aubo_driver/robot_connected
   # 应返回 '1'
   ```

**解决方案**：
- 确保机器人和工作站在同一网络
- 禁用防火墙或添加允许规则
- 检查网线连接

---

## 3. 实机执行问题

### 3.1 Execute 失败但实机已到位

**现象**：实机运动完成，但 RViz 显示 "ABORTED"

**原因**：Gazebo 未收敛到目标位置

**排查步骤**：
1. 检查 Gazebo RTF（Real-Time Factor）：
   - Gazebo 窗口左下角应显示 RTF ≈ 1.0
   - 如 RTF < 0.8，说明仿真过慢
2. 调整监视器参数：
   ```bash
   gedit src/aubo_linked_execution/launch/aubo_e5_linked_execution.launch
   # 修改容差（第 96 行）
   <param name="joint_tolerance" value="0.05"/>  <!-- 放宽到 0.05 rad -->
   ```

**解决方案**：
- 关闭其他程序，释放 CPU 资源
- 降低 Gazebo 仿真精度
- 放宽收敛容差

---

### 3.2 大幅度运动警告

**现象**：
```
[safety_monitor] WARNING: Large motion detected (0.8 rad > 0.5 rad threshold)
```

**原因**：RViz 中的目标位置与实机当前位置差异过大

**排查步骤**：
1. 检查 RViz 中的交互式标记位置
2. 检查实机当前位置：
   ```bash
   rostopic echo /joint_states
   ```
3. 验证启动位置同步：
   ```bash
   rostopic echo /safety_monitor/safe_to_execute
   ```

**解决方案**：
- 在 RViz 中拖动标记到接近当前位置
- 使用 "Random Valid Goal" 生成合理目标
- 检查启动位置同步是否完成

---

### 3.3 轨迹起点验证失败

**现象**：
```
[safety_monitor] WARNING: Trajectory start point mismatch
```

**原因**：规划的轨迹起点与实机当前位置不匹配

**排查步骤**：
1. 检查实机当前位置
2. 在 RViz 中重新规划
3. 检查容差设置：
   ```bash
   rosparam get /safety_monitor/trajectory_start_tolerance
   # 默认 0.15 rad
   ```

**解决方案**：
- 重新规划轨迹
- 放宽容差（如果确实安全）
- 检查实机是否在运动中

---

## 4. Gazebo 同步问题

### 4.1 Gazebo 中机器人不动

**现象**：实机运动，但 Gazebo 中模型静止

**排查步骤**：
1. 检查镜像适配器：
   ```bash
   rostopic hz /real/joint_states
   # 应显示约 500 Hz
   ```
2. 检查 Gazebo 驱动：
   ```bash
   rosnode info /aubo_gazebo_driver
   # 确认 shadow 参数为 true
   ```
3. 检查 Gazebo 是否暂停：
   ```bash
   # 在 Gazebo 中点击 Play 按钮
   ```

**解决方案**：
- 重启 Gazebo：`rosnode kill /gazebo`
- 检查 `/real/joint_states` 是否有数据
- 确认 shadow 模式已启用

---

### 4.2 Gazebo RTF 过低

**现象**：Gazebo 窗口左下角显示 RTF < 0.8

**原因**：仿真过慢，无法实时运行

**排查步骤**：
1. 检查 CPU 使用率：
   ```bash
   top
   ```
2. 检查 Gazebo 进程：
   ```bash
   ps aux | grep gzserver
   ```

**解决方案**：
- 关闭其他程序
- 降低 Gazebo 物理引擎精度
- 使用更简单的场景
- 升级硬件

---

## 5. Unity 集成问题

### 5.1 Unity 启动后立刻塌成乱姿

**现象**：Unity 中机器人关节混乱，不是初始姿态

**原因**：xDrive PD 参数没有写入（被 URDF Importer 覆盖）

**排查步骤**：
1. 查看 Unity Console：
   ```
   [AuboJointCommandSubscriber] init shoulder_joint: ...
   ```
2. 检查 URDF Importer 设置：
   - 确认已禁用自带 Controller

**解决方案**：
- 在 Unity Inspector 中禁用 URDF Importer 的 Controller
- 重启 Unity Play
- 检查 `AuboJointCommandSubscriber` 是否正确初始化

---

### 5.2 `/unity/joint_states` 没有数据

**现象**：
```bash
rostopic hz /unity/joint_states
# 无输出
```

**原因**：Unity 未点 Play 或 `AuboJointStatePublisher` 未绑定

**排查步骤**：
1. 检查 Unity 是否点了 Play
2. 检查 `AuboJointStatePublisher` 是否绑定了 6 个 ArticulationBody
3. 检查 ROS-TCP-Connector 连接状态

**解决方案**：
- 点击 Unity Play 按钮
- 在 Inspector 中检查脚本绑定
- 检查 ROS-TCP-Endpoint 连接

---

### 5.3 ROS 发指令但 Unity 没动

**现象**：
```bash
rostopic pub -1 /unity/joint_command ...
# Unity 无反应
```

**原因**：ROS-TCP-Endpoint 连接错误或关节名不匹配

**排查步骤**：
1. 查看 `ros_tcp_endpoint` 终端输出
2. 查看 Unity Console 的 `[Unknown joint]` 警告
3. 检查关节名是否一致

**解决方案**：
- 检查 ROS-TCP-Endpoint 连接状态
- 确认关节名前缀一致
- 重启 ROS-TCP-Endpoint

---

### 5.4 Unity 动了但 `arrive` 不出

**现象**：
```
[AuboJointCommandSubscriber] command received: ...
# 无 arrive 输出
```

**原因**：PD 参数太弱、关节卡住或容差太严

**排查步骤**：
1. 检查 Unity Inspector 中的 PD 参数
2. 检查 `arrivalToleranceDeg` 和 `arrivalTimeoutSeconds`
3. 手动检查关节是否能动

**解决方案**：
- 增加 stiffness 参数
- 放宽 `arrivalToleranceDeg`
- 增加 `arrivalTimeoutSeconds`

---

## 6. MoveIt 规划问题

### 6.1 MoveIt Plan 失败

**现象**：
```
[move_group] Planning attempt 1 failed
```

**原因**：目标位置不可达或规划器参数不合适

**排查步骤**：
1. 检查目标位置是否在工作空间内
2. 检查是否有碰撞
3. 尝试不同的规划算法

**解决方案**：
- 选择工作空间内的目标
- 清除场景中的障碍物
- 增加规划时间
- 尝试 CHOMP 优化器

---

### 6.2 MoveIt Execute 一直转圈

**现象**：
```
[move_group] Execution request accepted
# 一直等待，不返回结果
```

**原因**：动作服务器没收到 feedback_states

**排查步骤**：
1. 检查 feedback_states 频率：
   ```bash
   rostopic hz /feedback_states
   ```
2. 检查动作服务器状态：
   ```bash
   rostopic echo /aubo_e5_controller/follow_joint_trajectory/status
   ```

**解决方案**：
- 确保 `aubo_robot_simulator` 正在运行
- 检查 feedback_states 是否有数据
- 重启动作服务器

---

### 6.3 MoveIt 报 "Didn't receive robot state"

**现象**：
```
[move_group] Didn't receive robot state ... within 1 seconds
```

**原因**：`/joint_states` 无人发布

**排查步骤**：
1. 检查 joint_states 频率：
   ```bash
   rostopic hz /joint_states
   ```
2. 检查 aubo_driver 或 aubo_robot_simulator 是否运行
3. 检查是否有 remap 问题

**解决方案**：
- 确保 `aubo_driver` 或 `aubo_robot_simulator` 正在运行
- 检查 launch 文件中的 remap 设置
- 重启系统

---

## 7. 诊断工具

### 7.1 系统验证工具

```bash
cd ~/aubo_polish
./system_tools.sh verify
```

**检查项**：
- ROS 环境配置
- 核心包完整性
- Python 脚本权限
- AUBO SDK 库依赖
- 网络连接状态

### 7.2 运行时诊断

```bash
./system_tools.sh diagnose
```

**输出信息**：
- 节点运行状态
- 话题发布频率
- 关键参数值
- 错误日志摘要

### 7.3 RPATH 修复

```bash
./system_tools.sh fix-rpath
```

**用途**：修复 AUBO SDK 库依赖问题

---

## 8. 日志查看

### 8.1 查看节点日志

```bash
# 查看特定节点的日志
rosnode info /aubo_driver
rosnode info /aubo_gazebo_driver
rosnode info /linked_execution_action_server
```

### 8.2 查看话题内容

```bash
# 实时查看话题
rostopic echo /joint_states
rostopic echo /feedback_states
rostopic echo /safety_monitor/warning

# 查看话题频率
rostopic hz /joint_states
rostopic hz /feedback_states
```

### 8.3 查看参数

```bash
# 查看所有参数
rosparam list

# 查看特定参数
rosparam get /robot_name
rosparam get /safety_monitor/large_motion_threshold
```

---

## 9. 常见错误消息

| 错误消息 | 含义 | 解决方案 |
|---------|------|---------|
| `Timeout waiting for drives_powered` | 实机上电超时 | 检查示教器急停和网络 |
| `Failed to connect to robot` | 网络连接失败 | 检查 IP 和防火墙 |
| `Large motion detected` | 运动幅度过大 | 检查目标位置 |
| `Trajectory start point mismatch` | 轨迹起点不匹配 | 重新规划 |
| `Planning attempt failed` | 规划失败 | 调整目标或参数 |
| `Didn't receive robot state` | 没收到关节状态 | 检查驱动程序 |
| `Unknown joint` | 关节名不匹配 | 检查关节名前缀 |
| `Inbound TCP/IP connection failed` | TCP 连接失败 | 检查 ROS-TCP-Endpoint |

---

## 10. 相关文档

- [系统架构](ARCHITECTURE.md) - 系统设计
- [Unity 迁移](UNITY_MIGRATION.md) - Unity 集成指南
- [P4 超时修复](P4_UNITY_TIMEOUT_FIX.md) - MoveIt 超时问题解决
- [联动设计](LINKED_EXECUTION_DESIGN.md) - 设计原理
- [数据流分析](Unity_Migration_Data_Flow_Analysis.md) - 架构对比
