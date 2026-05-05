# AUBO E5 多目标轨迹规划使用指南

> 本文档回答手眼标定坐标系定义问题，并提供多目标轨迹规划的完整使用指南。

---

## 📐 问题 1：手眼标定坐标系定义

### 基座坐标系（base_link）

**零位状态定义**：
- 所有关节角度为 0 rad
- 机器臂处于"伸直向上"姿态

**坐标系定义**（基于 URDF）：
```
原点位置：base_link 中心（机器人基座中心）
X 轴：指向机器人前方（红色）
Y 轴：指向机器人左侧（绿色）
Z 轴：竖直向上（蓝色）
```

**验证方法**：
```bash
# 1. 启动系统
roslaunch aubo_e5_moveit_config demo.launch

# 2. 在 RViz 中查看 TF 树
# Displays → TF → Show Axes
# 可以看到 base_link 的 XYZ 轴方向

# 3. 使用 tf_echo 查看坐标系
rosrun tf tf_echo world base_link
rosrun tf tf_echo base_link wrist3_Link
```

**关键参数**（从 URDF）：
- base_link 相对于 pedestal_Link：固定偏移 z=0.503m
- 零位时 shoulder_joint 角度：0 rad（Z 轴旋转）

---

### 工具坐标系（TCP / wrist3_Link）

**零位状态定义**：
- 末端执行器指向正上方（Z 轴方向）
- 末端坐标系为 `wrist3_Link`

**坐标系定义**（零位时）：
```
原点位置：wrist3_Link 中心（末端法兰中心）
X 轴：指向末端前方
Y 轴：指向末端左侧
Z 轴：指向末端上方（与 base_link Z 轴平行）
```

**相对于 base_link 的位置**（零位时）：
```
位置：(0, 0, ~0.8m)  # 机器臂伸直向上
姿态：与 base_link 对齐
```

**验证方法**：
```bash
# 查看零位时的 TCP 位置
rosrun tf tf_echo base_link wrist3_Link

# 输出示例：
# Translation: [0.000, 0.000, 0.800]
# Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
#           in RPY (radian) [0.000, 0.000, 0.000]
```

---

### 旋转轴定向（关节轴）

**所有关节的旋转轴**（从 URDF）：
```
shoulder_joint:  Z 轴（竖直向上）
upperArm_joint:  Z 轴（大臂旋转）
foreArm_joint:   Z 轴（小臂旋转）
wrist1_joint:    Z 轴（腕部旋转 1）
wrist2_joint:    Z 轴（腕部旋转 2）
wrist3_joint:    Z 轴（腕部旋转 3）
```

**注意**：所有关节都是绕 Z 轴旋转（标准 DH 参数约定）

---

### 手眼标定配置

**配置文件位置**：
- `data/comatrix_scene/metadata/T_base_camera.yaml` - Eye-to-Hand（相机固定在基座）
- `data/comatrix_scene/metadata/T_tcp_camera.yaml` - Eye-in-Hand（相机固定在末端）

**Eye-in-Hand（相机固定在末端）**：
```yaml
# T_tcp_camera.yaml
parent_frame: wrist3_Link  # 工具坐标系
child_frame: comatrix_camera
translation_m: [x, y, z]  # 相机相对于 TCP 的偏移
rotation_quat_xyzw: [x, y, z, w]  # 相机相对于 TCP 的旋转
```

**Eye-to-Hand（相机固定在基座）**：
```yaml
# T_base_camera.yaml
parent_frame: base_link  # 基座坐标系
child_frame: comatrix_camera
translation_m: [x, y, z]  # 相机相对于基座的偏移
rotation_quat_xyzw: [x, y, z, w]  # 相机相对于基座的旋转
```

**标定流程**：
1. 使用 `easy_handeye` 或 `visp_hand2eye_calibration` ROS 包
2. 采集多个姿态的数据（建议 15-20 个）
3. 计算变换矩阵
4. 填入配置文件

---

## 🎯 问题 2：多目标轨迹功能

### 功能确认

**✅ 已完全实现**：项目已有完整的多目标轨迹规划和执行能力

**核心脚本**：`src/aubo_linked_execution/scripts/continuous_motion_demo.py`

**支持的三种模式**：

#### 1. 关节空间路径点（Joint Waypoints）
```bash
rosrun aubo_linked_execution continuous_motion_demo.py --mode joint
```

#### 2. 笛卡尔空间路径（Cartesian Path）⭐ 推荐
```bash
rosrun aubo_linked_execution continuous_motion_demo.py --mode cartesian
```

#### 3. 预定义位置序列（Named Targets）
```bash
rosrun aubo_linked_execution continuous_motion_demo.py --mode named
```

---

### 快速开始

**方法 1：使用快速启动脚本**（推荐）
```bash
cd ~/aubo_polish
./run_trajectory.sh
```

交互式选择：
1. 选择轨迹（正方形、圆形、巡航）
2. 是否循环执行
3. 循环次数

**方法 2：直接执行**
```bash
# 画正方形
rosrun aubo_linked_execution continuous_motion_demo.py \
    --mode cartesian \
    --config square_motion.yaml

# 画圆形
rosrun aubo_linked_execution continuous_motion_demo.py \
    --mode cartesian \
    --config circle_motion.yaml

# 多点巡航
rosrun aubo_linked_execution continuous_motion_demo.py \
    --mode cartesian \
    --config patrol_motion.yaml
```

---

### 示例轨迹

#### 1. 正方形轨迹（square_motion.yaml）

**描述**：在基座坐标系下画一个 20cm × 20cm 的正方形

**路径点**：
- 起点：(0.4, -0.1, 0.4)
- 右下角：(0.4, 0.1, 0.4)
- 右上角：(0.4, 0.1, 0.6)
- 左上角：(0.4, -0.1, 0.6)
- 回到起点（闭合）

**执行命令**：
```bash
rosrun aubo_linked_execution continuous_motion_demo.py \
    --mode cartesian \
    --config square_motion.yaml
```

---

#### 2. 圆形轨迹（circle_motion.yaml）

**描述**：画一个半径 10cm 的圆形（8 个点近似）

**中心点**：(0.4, 0.0, 0.5)

**执行命令**：
```bash
rosrun aubo_linked_execution continuous_motion_demo.py \
    --mode cartesian \
    --config circle_motion.yaml
```

---

#### 3. 多点巡航（patrol_motion.yaml）

**描述**：访问 4 个预定义位置

**路径点**：
- 位置 1：(0.3, 0.2, 0.5)
- 位置 2：(0.5, 0.2, 0.4)
- 位置 3：(0.5, -0.2, 0.4)
- 位置 4：(0.3, -0.2, 0.5)
- 回到起点

**执行命令**：
```bash
rosrun aubo_linked_execution continuous_motion_demo.py \
    --mode cartesian \
    --config patrol_motion.yaml
```

---

### 循环执行

**无限循环**：
```bash
rosrun aubo_linked_execution continuous_motion_demo.py \
    --mode cartesian \
    --config square_motion.yaml \
    --loop
```

**指定循环次数**：
```bash
rosrun aubo_linked_execution continuous_motion_demo.py \
    --mode cartesian \
    --config square_motion.yaml \
    --loop \
    --iterations 10
```

---

## 🛠️ 自定义轨迹

### 创建配置文件

创建新文件：`src/aubo_linked_execution/config/my_trajectory.yaml`

```yaml
name: "My Custom Trajectory"
description: "A custom multi-target trajectory"
mode: cartesian

cartesian_waypoints:
  # 路径点 1
  - position: {x: 0.4, y: 0.0, z: 0.5}
    orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}
  
  # 路径点 2
  - position: {x: 0.5, y: 0.1, z: 0.4}
    orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}
  
  # 添加更多路径点...
```

### 配置文件格式说明

**笛卡尔空间路径点**：
```yaml
cartesian_waypoints:
  - position: {x: 0.4, y: 0.0, z: 0.5}  # 单位：米
    orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}  # 四元数
```

**关节空间路径点**：
```yaml
joint_waypoints:
  - [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]  # 单位：弧度
  - [0.3, -0.5, 0.5, 0.0, 0.5, 0.0]
```

**预定义位置序列**：
```yaml
named_targets:
  - "home"
  - "ready"
  - "home"
```

---

## 📊 问题 3：与单次验证通道的兼容性

### 设计原则

✅ **完全兼容**：多目标轨迹功能不影响现有单次验证通道

### 推荐工作流

**日常使用**：

1. **单次验证**（快速测试）：
```bash
rosrun aubo_planner goto_pose_enhanced.py
目标坐标> 0.4 0.0 0.5
```

2. **多目标轨迹**（重复任务）：
```bash
./run_trajectory.sh
```

3. **可视化调试**（复杂轨迹）：
```bash
roslaunch aubo_e5_moveit_config moveit_rviz.launch
```

### 两种方式可以交替使用

```bash
# 终端 1：系统已启动
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch sim_only:=true

# 终端 2：先执行多目标轨迹
rosrun aubo_linked_execution continuous_motion_demo.py \
    --mode cartesian \
    --config square_motion.yaml

# 终端 3：然后使用单次验证
rosrun aubo_planner goto_pose_enhanced.py
目标坐标> 0.4 0.0 0.5
```

**结果**：两种方式互不干扰，功能正常 ✅

---

## 🎨 RViz 可视化

### 启动 RViz + MoveIt

```bash
roslaunch aubo_e5_moveit_config demo.launch
```

### 在 RViz 中手动规划

1. 使用 Interactive Marker 设置目标位置
2. 点击 "Plan" 规划路径
3. 点击 "Execute" 执行
4. 重复设置下一个目标

### 查看 TF 树

在 RViz 中：
- Displays → TF → Show Axes
- 可以看到所有坐标系的 XYZ 轴方向

---

## 📝 配置文件位置

**示例轨迹配置**：
- `src/aubo_linked_execution/config/square_motion.yaml` - 正方形
- `src/aubo_linked_execution/config/circle_motion.yaml` - 圆形
- `src/aubo_linked_execution/config/patrol_motion.yaml` - 巡航
- `src/aubo_linked_execution/config/motion_waypoints.yaml` - 默认

**手眼标定配置**：
- `data/comatrix_scene/metadata/T_base_camera.yaml` - Eye-to-Hand
- `data/comatrix_scene/metadata/T_tcp_camera.yaml` - Eye-in-Hand

**快速启动脚本**：
- `run_trajectory.sh` - 多目标轨迹快速启动

---

## 🔍 常见问题

### Q1: 如何确定坐标系的朝向？

**A**: 使用 RViz 查看 TF 树：
```bash
roslaunch aubo_e5_moveit_config demo.launch
# 在 RViz 中：Displays → TF → Show Axes
```

### Q2: 如何创建自定义轨迹？

**A**: 创建 YAML 配置文件，参考示例轨迹的格式。

### Q3: 如何让机器臂画其他图形？

**A**: 修改配置文件中的路径点坐标，或创建新的配置文件。

### Q4: 多目标轨迹会影响单次验证吗？

**A**: 不会。两种方式完全独立，可以交替使用。

### Q5: 如何调整运动速度？

**A**: 修改 MoveIt 配置文件中的速度限制：
```
src/aubo_robot/aubo_e5_moveit_config/config/joint_limits.yaml
```

---

## 📚 相关文档

- **项目说明**：`README.md`
- **操作指南**：`OPERATION_GUIDE.md`
- **P6 测试指南**：`docs/P6_TEST_GUIDE.md`
- **Unity 迁移指南**：`docs/UNITY_MIGRATION_NEXT_STEPS.md`

---

## 🎯 总结

### 问题 1：手眼标定坐标系定义 ✅

- **base_link**：原点在基座中心，X 向前，Y 向左，Z 向上
- **wrist3_Link**：零位时在 base_link 正上方约 0.8m
- **所有关节**：绕 Z 轴旋转

### 问题 2：多目标轨迹功能 ✅

- **已完全实现**：支持关节空间、笛卡尔空间、预定义位置序列
- **立即可用**：提供正方形、圆形、巡航等示例轨迹
- **易于扩展**：通过 YAML 配置文件定义新轨迹

### 问题 3：与单次验证通道的兼容性 ✅

- **完全兼容**：不影响现有功能
- **可交替使用**：单次验证和多目标轨迹互不干扰
- **基于 RViz/MoveIt**：可视化强，易于调试

---

*最后更新：2026-05-02*
*项目路径：`/home/wuqz/aubo_polish`*
