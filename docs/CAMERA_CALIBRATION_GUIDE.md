# 相机标定自动化采集系统使用指南

> 本文档介绍如何使用相机标定自动化采集系统，实现基于示教位置的多角度自动拍摄。

---

## 📋 功能概述

### 核心功能

1. **读取示教位置**：自动读取当前机器臂的关节角度和笛卡尔位姿
2. **生成标定位置**：基于示教位置自动生成 15 个不同角度和距离的标定位置
3. **自动执行轨迹**：依次移动到每个位置并触发相机拍照
4. **保存配置**：将所有位置信息保存到 YAML 配置文件

### 应用场景

- **相机标定**：多角度采集标定板图像
- **3D 重建**：多视角采集物体图像
- **质量检测**：不同角度检测工件表面
- **数据采集**：自动化图像数据集采集

---

## 🎯 标定位置说明

系统会自动生成 15 个标定位置（基于示教位置）：

| 编号 | 名称 | 描述 | 偏移量 |
|------|------|------|--------|
| 1 | calib_0001_center | 正对中心 | 无偏移 |
| 2 | calib_0002_left | 偏左 | Y +5cm |
| 3 | calib_0003_right | 偏右 | Y -5cm |
| 4 | calib_0004_x_plus_15 | X 轴旋转 +15° | Roll +15° |
| 5 | calib_0005_x_minus_15 | X 轴旋转 -15° | Roll -15° |
| 6 | calib_0006_x_plus_25 | X 轴旋转 +25° | Roll +25° |
| 7 | calib_0007_x_minus_25 | X 轴旋转 -25° | Roll -25° |
| 8 | calib_0008_y_plus_15 | Y 轴旋转 +15° | Pitch +15° |
| 9 | calib_0009_y_minus_15 | Y 轴旋转 -15° | Pitch -15° |
| 10 | calib_0010_y_plus_25 | Y 轴旋转 +25° | Pitch +25° |
| 11 | calib_0011_y_minus_25 | Y 轴旋转 -25° | Pitch -25° |
| 12 | calib_0012_z_plus_20 | Z 轴旋转 +20° | Yaw +20° |
| 13 | calib_0013_z_minus_20 | Z 轴旋转 -20° | Yaw -20° |
| 14 | calib_0014_near | 近一点 | X +7cm (500→430mm) |
| 15 | calib_0015_far | 远一点 | X -8cm (500→580mm) |

---

## 🚀 使用方法

### 方法 1：使用快速启动脚本（推荐）

**步骤 1 - 启动系统：**
```bash
# 终端 1：启动联动系统（实机或仿真）
cd ~/aubo_polish

# 实机模式
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    robot_ip:=192.168.1.10

# 或仿真模式
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    sim_only:=true
```

**步骤 2 - 示教初始位置：**
- 手动移动机器臂到初始位置
- 相机正对物体
- 距离约 500mm
- 确保工作空间内无障碍物

**步骤 3 - 运行标定脚本：**
```bash
# 终端 2：运行标定采集
./run_camera_calibration.sh
```

**步骤 4 - 确认并执行：**
- 脚本会显示当前位置信息
- 确认无误后输入 `y` 开始执行
- 系统会自动移动到 15 个位置并拍照

---

### 方法 2：直接运行脚本

```bash
cd ~/aubo_polish
source devel/setup.bash
rosrun aubo_planner camera_calibration_capture.py
```

---

## 📊 执行流程

### 1. 读取当前位置

脚本会自动读取并显示：
```
正在读取当前示教位置...
✓ 当前位置已读取：
  位置: x=0.4000, y=0.0000, z=0.5000
  姿态: roll=0.00°, pitch=90.00°, yaw=0.00°
  关节: ['0.00°', '-28.65°', '28.65°', '0.00°', '28.65°', '0.00°']
```

### 2. 生成标定位置

系统会基于当前位置生成 15 个标定位置：
```
正在生成标定位置...
✓ calib_0001_center: pos=(0.400, 0.000, 0.500), rpy=(0.0°, 90.0°, 0.0°)
✓ calib_0002_left: pos=(0.400, 0.050, 0.500), rpy=(0.0°, 90.0°, 0.0°)
...
✓ 已生成 15 个标定位置
```

### 3. 确认执行

```
请确认：
  1. 机器臂已手动示教到初始位置（相机正对物体，距离约 500mm）
  2. 相机已准备好拍照
  3. 工作空间内无障碍物

是否继续执行标定序列？(y/n):
```

### 4. 自动执行

系统会依次执行每个位置：
```
[1/15] calib_0001_center
  ✓ 规划成功
  ✓ 执行成功
  📷 触发相机拍照: calib_0001_center
  ✓ 完成

[2/15] calib_0002_left
  ✓ 规划成功
  ✓ 执行成功
  📷 触发相机拍照: calib_0002_left
  ✓ 完成
...
```

### 5. 保存配置

执行完成后，配置会自动保存：
```
✓ 配置已保存到: ~/aubo_polish/calibration_data/camera_calibration_20260502_143025.yaml
```

---

## 📁 输出文件

### 配置文件格式

保存位置：`~/aubo_polish/calibration_data/camera_calibration_YYYYMMDD_HHMMSS.yaml`

```yaml
timestamp: '20260502_143025'
base_pose:
  position: [0.4, 0.0, 0.5]
  orientation_quat: [0.0, 0.707, 0.0, 0.707]
base_joints: [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]
calibration_poses:
  - name: calib_0001_center
    position: [0.4, 0.0, 0.5]
    orientation_rpy: [0.0, 1.5708, 0.0]
    orientation_quat: [0.0, 0.707, 0.0, 0.707]
  - name: calib_0002_left
    position: [0.4, 0.05, 0.5]
    orientation_rpy: [0.0, 1.5708, 0.0]
    orientation_quat: [0.0, 0.707, 0.0, 0.707]
  # ... 更多位置
```

---

## 🔧 自定义配置

### 修改标定位置

编辑脚本中的 `calibration_configs` 列表：

```python
calibration_configs = [
    # (名称, x偏移, y偏移, z偏移, roll偏移, pitch偏移, yaw偏移)
    ("calib_0001_center", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    ("calib_0002_left", 0.0, 0.05, 0.0, 0.0, 0.0, 0.0),
    # 添加更多位置...
]
```

### 修改停留时间

修改 `capture_delay` 参数（默认 2 秒）：

```python
success = self.execute_calibration_sequence(
    capture_delay=3.0,  # 每个位置停留 3 秒
    save_config=True
)
```

---

## 📷 相机触发集成

### 方法 1：ROS 话题（默认）

脚本会发布相机触发消息到 `/camera/trigger` 话题：

```python
# 订阅触发消息
rostopic echo /camera/trigger
```

### 方法 2：ROS 服务

修改 `trigger_camera()` 方法，调用相机服务：

```python
def trigger_camera(self, pose_name):
    try:
        rospy.wait_for_service('/camera/capture', timeout=1.0)
        capture_service = rospy.ServiceProxy('/camera/capture', Trigger)
        response = capture_service()
        if response.success:
            print(f"  ✓ 相机拍照成功")
    except:
        print(f"  ⚠ 相机服务不可用")
```

### 方法 3：HTTP 请求

如果相机提供 HTTP API：

```python
import requests

def trigger_camera(self, pose_name):
    try:
        response = requests.post(
            'http://camera_ip:port/capture',
            json={'name': pose_name}
        )
        if response.status_code == 200:
            print(f"  ✓ 相机拍照成功")
    except:
        print(f"  ⚠ 相机连接失败")
```

---

## ⚠️ 注意事项

### 安全注意事项

1. **工作空间检查**：确保工作空间内无障碍物
2. **示教位置**：初始位置应在安全范围内
3. **急停准备**：随时准备按下急停按钮
4. **监控运动**：全程监控机器臂运动

### 使用建议

1. **首次测试**：建议先在仿真模式下测试
2. **小幅度运动**：标定位置的偏移量应保持在安全范围内
3. **相机焦距**：确保所有位置都在相机焦距范围内
4. **光照条件**：保持稳定的光照条件

---

## 🔍 故障排查

### 问题 1：规划失败

**现象**：某些位置规划失败

**原因**：目标位置超出工作空间或存在碰撞

**解决方案**：
- 调整示教位置
- 减小偏移量
- 检查工作空间限制

### 问题 2：执行失败

**现象**：规划成功但执行失败

**原因**：速度超限或关节限位

**解决方案**：
- 降低运动速度
- 检查关节限位配置
- 查看日志输出

### 问题 3：相机未触发

**现象**：机器臂运动正常，但相机未拍照

**原因**：相机触发接口未配置

**解决方案**：
- 检查相机连接
- 配置相机触发方法（ROS 服务或 HTTP API）
- 查看 `/camera/trigger` 话题

---

## 📚 相关文档

- **多目标轨迹指南**：`docs/MULTI_TARGET_TRAJECTORY_GUIDE.md`
- **坐标系定义**：`docs/MULTI_TARGET_TRAJECTORY_GUIDE.md`（含手眼标定）
- **故障排查**：`docs/TROUBLESHOOTING.md`
- **架构设计**：`docs/ARCHITECTURE.md`

---

## 💡 扩展功能

### 1. 添加更多标定位置

在 `calibration_configs` 中添加更多位置：

```python
("calib_0016_custom", 0.1, 0.1, 0.0, 0.0, 0.0, math.radians(10)),
```

### 2. 循环执行

修改脚本支持多次循环采集：

```python
for i in range(3):  # 循环 3 次
    self.execute_calibration_sequence()
```

### 3. 自动返回初始位置

在执行完成后返回示教位置：

```python
# 返回初始位置
self.move_group.set_joint_value_target(self.base_joints)
self.move_group.go(wait=True)
```

---

## 📊 总结

### 系统能力

- ✅ **能读取当前位姿**：支持关节角度和笛卡尔坐标
- ✅ **Unity/Gazebo 同步**：所有关节状态实时同步
- ✅ **自动化执行**：无需手动操作，全自动采集
- ✅ **配置保存**：所有位置信息自动保存

### 使用流程

1. 启动系统（实机或仿真）
2. 示教初始位置
3. 运行标定脚本
4. 确认并执行
5. 自动完成 15 个位置的采集

### 输出结果

- 15 张不同角度的图像
- 完整的位置配置文件（YAML）
- 执行日志和统计信息

---

*最后更新：2026-05-02*
*项目路径：`/home/wuqz/aubo_polish`*
