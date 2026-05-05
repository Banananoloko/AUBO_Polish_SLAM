# 实机测试安全指南

## 概述

本指南提供在 AUBO E5 实机上测试运动规划接口的安全建议。

## 修改后的安全特性

### 1. 原始代码的风险

**原始目标位置**：
```python
x=0.5, y=0.2, z=0.3  # 绝对位置，可能距离当前位置很远
roll=0, pitch=1.57, yaw=0  # pitch=90度，可能接近奇异点
```

**风险评估**：
- ❌ 使用绝对位置，不知道移动距离
- ❌ 无速度限制，使用默认速度（可能 100%）
- ❌ 无用户确认，直接执行
- ❌ 姿态可能导致奇异点
- ❌ 无错误恢复机制

### 2. 修改后的安全措施

#### 安全参数
- ✅ 速度缩放：20%（原默认 100%）
- ✅ 加速度缩放：20%（原默认 100%）
- ✅ 移动距离：3cm（笛卡尔）或 5°（关节）
- ✅ 相对于当前位置移动

#### 安全流程
1. **启动前警告**：显示安全检查清单
2. **用户确认**：需要按 Enter 才继续
3. **显示当前状态**：显示当前位置和关节角度
4. **每个测试前确认**：每个动作前都可以跳过
5. **测试结果摘要**：显示所有测试的结果

## 三个安全测试

### 测试 1：笛卡尔空间小幅度移动（超级安全）

**描述**：从当前位置向上移动 3cm

**安全性**：
- 移动距离极小（3cm）
- 方向明确（Z轴向上，远离地面）
- 不改变姿态
- 最容易预测的运动

**适用场景**：
- 首次实机测试
- 验证基本功能
- 不确定当前位置时

### 测试 2：关节空间小幅度移动（非常安全）

**描述**：每个关节移动 5 度

**安全性**：
- 关节空间移动更可预测
- 角度变化很小（5° ≈ 0.087 弧度）
- 避免笛卡尔空间的奇异点问题
- 不会超出关节限位

**适用场景**：
- 测试关节空间规划
- 验证关节控制
- 避免奇异点

### 测试 3：返回 Home 位置（安全）

**描述**：返回到预定义的 Home 位置

**安全性**：
- Home 位置是经过验证的安全位置
- 可以在测试后恢复到已知状态
- MoveIt 内置的命名目标

**适用场景**：
- 测试后恢复
- 验证命名目标功能
- 回到已知安全状态

## 实机测试步骤

### 1. 仿真测试（必需）

在实机测试前，先在仿真中测试：

```bash
# 启动仿真模式
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch sim_only:=true

# 在另一个终端运行测试
cd /home/wuqz/aubo_polish
source devel/setup.bash
python src/aubo_robot/aubo_planner/examples/simple_motion_example.py
```

### 2. 实机准备

1. **物理检查**：
   - 清空机器人工作空间
   - 确保无障碍物和人员
   - 检查急停按钮可用

2. **启动实机系统**：
```bash
# 启动联动模式（实机+Gazebo）
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.1.10
```

3. **验证连接**：
```bash
# 检查关节状态
rostopic echo /joint_states -n 1

# 检查机器人状态
rostopic echo /robot_status -n 1
```

### 3. 执行测试

```bash
# 运行安全测试脚本
cd /home/wuqz/aubo_polish
source devel/setup.bash
python src/aubo_robot/aubo_planner/examples/simple_motion_example.py
```

**测试流程**：
1. 阅读安全警告
2. 确认周围环境安全
3. 按 Enter 继续
4. 观察当前状态显示
5. 每个测试前可以按 Ctrl+C 跳过
6. 观察机器人运动
7. 查看测试结果摘要

### 4. 紧急情况处理

**如果出现异常**：
1. 立即按下急停按钮
2. 检查错误消息
3. 检查机器人状态
4. 在仿真中重现问题
5. 修复后再次测试

## 进阶测试建议

### 增加移动距离

如果 3cm 测试成功，可以逐步增加：

```python
# 5cm 移动
target_z = current_pose.position.z + 0.05

# 10cm 移动
target_z = current_pose.position.z + 0.10
```

### 增加速度

如果低速测试成功，可以逐步增加：

```python
# 30% 速度
planner.planner.move_group.set_max_velocity_scaling_factor(0.3)

# 50% 速度
planner.planner.move_group.set_max_velocity_scaling_factor(0.5)
```

### 测试其他方向

```python
# 向前移动 5cm
target_x = current_pose.position.x + 0.05

# 向右移动 5cm
target_y = current_pose.position.y + 0.05
```

## 安全检查清单

### 测试前
- [ ] 已在仿真中测试
- [ ] 工作空间已清空
- [ ] 急停按钮可用
- [ ] 机器人处于安全位置
- [ ] 了解紧急停止程序

### 测试中
- [ ] 观察机器人运动
- [ ] 准备按急停按钮
- [ ] 注意异常声音或震动
- [ ] 监控终端输出

### 测试后
- [ ] 检查测试结果
- [ ] 机器人返回安全位置
- [ ] 记录任何异常
- [ ] 分析日志文件

## 常见问题

### Q: 规划失败怎么办？

A: 规划失败通常是因为：
- 目标位置不可达
- 目标位置接近奇异点
- 目标位置超出工作空间

解决方法：
1. 检查当前位置
2. 减小移动距离
3. 尝试关节空间规划

### Q: 执行失败怎么办？

A: 执行失败可能是因为：
- 实机通信问题
- 控制器未就绪
- 轨迹跟踪误差过大

解决方法：
1. 检查 `/robot_status`
2. 检查 `/feedback_states`
3. 降低速度和加速度

### Q: 机器人运动不平滑？

A: 可能原因：
- 速度缩放因子过低
- 规划器参数不当
- 轨迹点过少

解决方法：
1. 适当提高速度缩放因子
2. 调整规划器参数
3. 增加轨迹插补点

## 参考资料

- [AUBO E5 启动指南](../../../AUBO_E5_启动指南.txt)
- [联动执行设计文档](../../../RViz_Real_Gazebo_Linked_Execution_Design.md)
- [项目结构指南](../../../AUBO_E5_Structure_Guide.md)
- [运动规划接口文档](../scripts/motion_planning_interface.py)

## 联系支持

如果遇到问题：
1. 查看 `/home/wuqz/aubo_polish/工作记录.txt`
2. 运行诊断脚本：`./diagnose.sh`
3. 检查 ROS 日志：`~/.ros/log/`
