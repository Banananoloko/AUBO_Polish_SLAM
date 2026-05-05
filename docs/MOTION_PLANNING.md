# 路径规划与碰撞检测指南

> OMPL 22 种算法、CHOMP 优化、连续规划、避障测试完整指南

**最后更新**: 2026-04-28

---

## 1. 路径规划与碰撞检测模块

### 1.1 已有的路径规划算法

项目集成了完整的 **MoveIt** 路径规划框架，包含以下算法：

#### OMPL 规划器（默认）

**配置文件**: `src/aubo_robot/aubo_e5_moveit_config/config/ompl_planning.yaml`

**支持的算法**（共 22 种）：
- **RRTConnect**（默认）：快速随机树双向连接，适合简单场景
- **RRTstar**：优化版 RRT，路径质量更好
- **PRM / PRMstar**：概率路线图，适合多次查询
- **KPIECE / LBKPIECE / BKPIECE**：基于分解的规划器
- **EST / BiEST / ProjEST**：扩展空间树
- **TRRT / BiTRRT / LBTRRT**：带温度的 RRT
- **FMT / BFMT**：快速行进树
- **SPARS / SPARStwo**：稀疏路线图
- **STRIDE / LazyPRM / LazyPRMstar**：延迟碰撞检测

**关键参数**：
```yaml
manipulator_e5:
  default_planner_config: RRTConnect
  longest_valid_segment_fraction: 0.005  # 碰撞检测分辨率
```

#### CHOMP 规划器（优化器）

**配置文件**: `src/aubo_robot/aubo_e5_moveit_config/config/chomp_planning.yaml`

**特点**：基于梯度优化的轨迹平滑器

**关键参数**：
```yaml
planning_time_limit: 10.0
max_iterations: 200
collision_clearence: 0.2        # 碰撞安全距离 20cm
collision_threshold: 0.07       # 碰撞检测阈值 7cm
smoothness_cost_weight: 0.1     # 平滑度权重
obstacle_cost_weight: 1.0       # 障碍物代价权重
```

#### LERP 规划器（线性插值）

**配置文件**: `src/aubo_robot/aubo_e5_moveit_config/config/lerp_planning.yaml`

**特点**：简单的线性插值，适合无障碍物的简单运动

---

### 1.2 碰撞检测

**三层碰撞检测**：
1. **自碰撞检测**：机器人各部分之间的碰撞
2. **环境碰撞检测**：机器人与场景中物体的碰撞
3. **关节限位检测**：关节角度是否超出限位

**关节限位**：
```
shoulder_joint:    ±3.05 rad (±174.8°)
upperArm_joint:    ±3.05 rad (±174.8°)
foreArm_joint:     ±3.05 rad (±174.8°)
wrist1_joint:      ±3.05 rad (±174.8°)
wrist2_joint:      ±3.05 rad (±174.8°)
wrist3_joint:      ±3.05 rad (±174.8°)
```

**最大力矩**：300 N·m

---

## 2. 连续运动规划

连续运动规划允许机器人按预定义路径点序列自动执行运动，支持三种模式：关节空间、笛卡尔空间和预定义位置。

### 2.1 快速开始

**启动仿真系统**：
```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch sim_only:=true
```

**运行连续运动演示**（新终端）：
```bash
# 关节空间模式：直接指定各关节角度
rosrun aubo_linked_execution continuous_motion_demo.py --mode joint

# 笛卡尔空间模式：指定末端执行器位置和姿态
rosrun aubo_linked_execution continuous_motion_demo.py --mode cartesian

# 预定义位置模式：使用命名位置（如 home、ready）
rosrun aubo_linked_execution continuous_motion_demo.py --mode named

# 循环执行模式：连续循环运动
rosrun aubo_linked_execution continuous_motion_demo.py --mode joint --loop
```

### 2.2 配置路径点

编辑 `src/aubo_linked_execution/config/motion_waypoints.yaml` 自定义路径点：

```yaml
# 关节空间路径点（弧度）
joint_waypoints:
  - [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]      # 路径点 1
  - [0.5, -0.5, 0.5, 0.0, 0.5, 0.0]      # 路径点 2
  - [1.0, -1.0, 1.0, 0.0, 0.5, 0.0]      # 路径点 3

# 笛卡尔空间路径点（米）
cartesian_waypoints:
  - position: {x: 0.3, y: 0.2, z: 0.4}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  - position: {x: 0.4, y: 0.3, z: 0.5}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}

# 预定义目标序列
named_targets: ["home", "ready", "home"]
```

### 2.3 常见问题

**Q: 连续运动执行失败**
- 检查路径点是否超出工作空间或关节限位
- 在 RViz 中手动测试每个路径点
- 查看终端日志了解具体错误信息

**Q: 笛卡尔空间规划失败**
- 检查目标位置是否在工作空间内
- 尝试调整姿态（orientation）
- 使用关节空间模式作为替代方案

---

## 3. 避障测试系统

避障测试系统在 Gazebo 中动态生成随机障碍物，用于测试 MoveIt 的避障规划能力。

### 3.1 快速开始

**启动避障测试系统**：
```bash
roslaunch aubo_linked_execution test_obstacle_avoidance.launch
```

**生成随机障碍物**（新终端）：
```bash
rosservice call /obstacle_spawner/spawn_obstacles
```

**在 RViz 中手动规划和执行**：
1. 拖动 Interactive Marker 到目标位置
2. 点击 "Plan" 按钮，观察规划路径是否绕开障碍物
3. 点击 "Execute" 执行运动

**清除所有障碍物**：
```bash
rosservice call /obstacle_spawner/clear_obstacles
```

**自动化批量测试**：
```bash
rosrun aubo_linked_execution test_obstacle_avoidance.py
```

### 3.2 配置工作空间

在 `test_obstacle_avoidance.launch` 中修改障碍物生成范围：

```xml
<!-- 工作空间范围（米） -->
<param name="workspace_x_min" value="0.2"/>
<param name="workspace_x_max" value="0.6"/>
<param name="workspace_y_min" value="-0.4"/>
<param name="workspace_y_max" value="0.4"/>
<param name="workspace_z_min" value="0.1"/>
<param name="workspace_z_max" value="0.5"/>

<!-- 障碍物尺寸范围（米） -->
<param name="obstacle_size_min" value="0.05"/>
<param name="obstacle_size_max" value="0.15"/>

<!-- 生成的障碍物数量 -->
<param name="num_obstacles" value="5"/>
```

### 3.3 常见问题

**Q: 障碍物生成后 Planning Scene 中看不到**
- 生成障碍物后等待 2-3 秒
- 在 RViz 中勾选 "Planning Scene" 显示
- 检查 `/obstacle_spawner` 节点日志

**Q: 避障规划失败**
- 清除障碍物重新生成
- 调整工作空间参数，减少障碍物密度
- 选择更远离障碍物的目标位置

**Q: Gazebo 中看到障碍物但 MoveIt 不避障**
- 检查 `obstacle_spawner` 节点日志，确认已添加到 Planning Scene
- 重启系统，确保 MoveIt 正常初始化
- 使用 `rostopic echo /planning_scene` 验证

---

## 4. 规划参数配置

### 4.1 OMPL 规划器参数

编辑 `src/aubo_robot/aubo_e5_moveit_config/config/ompl_planning.yaml`：

```yaml
manipulator_e5:
  default_planner_config: RRTConnect
  
  # 规划时间限制（秒）
  planning_time: 5.0
  
  # 碰撞检测分辨率（轨迹段长度占总长度的比例）
  longest_valid_segment_fraction: 0.005
  
  # 规划器特定参数
  RRTConnect:
    range: 0.0  # 0 表示自动计算
    goal_bias: 0.05
  
  RRTstar:
    range: 0.0
    goal_bias: 0.05
    delay_collision_checking: true
  
  PRM:
    max_nearest_neighbors: 10
    attempts: 10
```

### 4.2 关节限速限加速

编辑 `src/aubo_robot/aubo_e5_moveit_config/config/joint_limits.yaml`：

```yaml
joint_limits:
  shoulder_joint:
    has_velocity_limits: true
    max_velocity: 2.0          # rad/s
    has_acceleration_limits: true
    max_acceleration: 1.0      # rad/s²
  
  upperArm_joint:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  
  # ... 其他关节类似
```

### 4.3 CHOMP 优化器参数

编辑 `src/aubo_robot/aubo_e5_moveit_config/config/chomp_planning.yaml`：

```yaml
planning_time_limit: 10.0
max_iterations: 200
max_iterations_after_collision_free: 5

# 代价函数权重
smoothness_cost_weight: 0.1
obstacle_cost_weight: 1.0
learning_rate: 0.01

# 碰撞检测
collision_clearence: 0.2        # 安全距离（米）
collision_threshold: 0.07       # 检测阈值（米）
use_pseudo_inverse: false
pseudo_inverse_epsilon: 2e-6
joint_update_limit: 0.1
```

---

## 5. 规划算法选择指南

| 场景 | 推荐算法 | 原因 |
|------|---------|------|
| 简单无障碍运动 | RRTConnect | 快速，适合实时规划 |
| 复杂环境避障 | RRTstar + CHOMP | 质量好，平滑度高 |
| 多次查询同一场景 | PRM | 预处理一次，多次查询快 |
| 狭窄通道 | KPIECE | 基于分解，适合狭窄空间 |
| 需要平滑轨迹 | CHOMP | 梯度优化，轨迹光滑 |
| 快速原型 | LERP | 简单直线，无碰撞检测 |

---

## 6. 性能优化建议

### 6.1 规划速度优化

1. **减少碰撞检测分辨率**（谨慎）：
   ```yaml
   longest_valid_segment_fraction: 0.01  # 从 0.005 增加到 0.01
   ```

2. **增加规划时间**：
   ```yaml
   planning_time: 10.0  # 给规划器更多时间
   ```

3. **使用 PRM 预处理**：
   - 第一次规划时间长，后续查询快

### 6.2 轨迹质量优化

1. **使用 CHOMP 优化**：
   - 在 RViz 中规划后，再用 CHOMP 优化

2. **增加 CHOMP 迭代次数**：
   ```yaml
   max_iterations: 300  # 从 200 增加到 300
   ```

3. **调整平滑度权重**：
   ```yaml
   smoothness_cost_weight: 0.2  # 增加平滑度权重
   ```

---

## 7. 相关文档

- [系统架构](ARCHITECTURE.md) - 完整数据流
- [操作指南](../OPERATION_GUIDE.md) - 实机操作
- [故障排查](TROUBLESHOOTING.md) - 常见问题
