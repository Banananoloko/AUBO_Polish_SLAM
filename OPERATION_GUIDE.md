# AUBO E5 操作指南

> 实机测试、连续规划、避障测试和视觉扩展方案

---

## 快速参考

### 前置检查清单
- [ ] AUBO E5 机器人已上电
- [ ] 示教器急停按钮已释放
- [ ] 网络连接正常（机器人 IP: 192.168.1.10）
- [ ] 机器人周围 1 米内无障碍物和人员
- [ ] 示教器无报警信息（connect modbus successfully）

### 快速启动命令
```bash
# 1. 网络测试
ping 192.168.1.10

# 2. 系统验证
cd ~/aubo_polish
./system_tools.sh verify

# 3. 启动联动系统（选择一种）

# 3a. 实机 + Gazebo 镜像（默认）
source devel/setup.bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.1.10

# 3b. 仅 Gazebo 仿真
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch sim_only:=true

# 3c. 仅 Unity 仿真
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch use_unity:=true sim_only:=true

# 3d. 实机 + Unity 镜像
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.1.10 use_unity:=true
```

### 启动成功标志
等待终端显示以下信息（约 30-60 秒）：
```
✓ [startup] Robot powered on
✓ [startup] Switched to ROS control mode
✓ [startup] Initial position sync verified
✓ [safety_monitor] Safety monitor ready
✓ [linked_execution_action_server] ready
```

### 执行运动流程
1. **在 RViz 中拖动交互式标记**到目标位置
2. **点击 "Plan"** 按钮规划路径（观察橙色轨迹）
3. **检查终端**是否有安全警告
4. **点击 "Execute"** 按钮执行运动

### 常见问题速查

| 问题 | 快速解决 |
|------|---------|
| 启动超时 | 检查示教器急停是否释放 |
| 网络连接失败 | `ping 192.168.1.10` 测试连通性 |
| 大幅度运动警告 | 检查 RViz 位置是否与实机一致 |
| Execute 失败 | 查看 `/safety_monitor/warning` 话题 |
| Gazebo 不同步 | 检查 `/real/joint_states` 频率 (~10Hz) |

### 监控命令
```bash
# 安全状态
rostopic echo /safety_monitor/safe_to_execute

# 安全警告
rostopic echo /safety_monitor/warning

# 关节状态频率
rostopic hz /joint_states

# 诊断系统
./system_tools.sh diagnose
```

### 连续规划快速命令
```bash
# 启动仿真系统
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch sim_only:=true

# 关节空间连续运动
rosrun aubo_linked_execution continuous_motion_demo.py --mode joint

# 笛卡尔空间连续运动
rosrun aubo_linked_execution continuous_motion_demo.py --mode cartesian

# 循环执行
rosrun aubo_linked_execution continuous_motion_demo.py --mode joint --loop
```

### 避障测试快速命令
```bash
# 启动避障测试系统
roslaunch aubo_linked_execution test_obstacle_avoidance.launch

# 生成随机障碍物
rosservice call /obstacle_spawner/spawn_obstacles

# 清除所有障碍物
rosservice call /obstacle_spawner/clear_obstacles

# 自动化批量测试
rosrun aubo_linked_execution test_obstacle_avoidance.py
```

### 紧急停止
- **软件停止**: RViz 中点击 "Stop" 或按 Ctrl+C
- **硬件停止**: 按下示教器急停按钮

---

## 第一部分：实机测试基本操作

### 1. 系统检查

#### 1.1 硬件准备清单
- [ ] AUBO E5 机器人已上电
- [ ] 急停按钮已释放
- [ ] 示教器无报警信息—— connect modbus successfully
- [ ] 网络连接正常（机器人 IP 可 ping 通）IP: 192.168.1.10
- [ ] 工作站已安装 ROS Noetic

#### 1.2 软件环境验证
```bash
cd ~/aubo_polish
source devel/setup.bash

# 运行系统验证工具
./system_tools.sh verify
```

验证工具会自动检查：
- ROS 环境配置
- 核心包完整性
- Python 脚本权限
- AUBO SDK 库依赖
- 网络连接状态

---

### 2. 启动系统

#### 2.1 联动模式（实机 + Gazebo）

**适用场景**：实机执行 + 仿真镜像同步（shadow mode）

```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    robot_ip:=192.168.1.10
```

**启动过程**（约 30 秒）：
1. MoveIt 规划器启动
2. RViz 可视化界面打开
3. Gazebo 仿真环境启动（shadow 模式）
4. 实机自动上电并切换到 ROS 控制模式
5. 联动聚合层就绪

**观察终端输出**：
```
[aubo_robot_startup] Robot powered on (drives_powered=1).
[aubo_robot_startup] Initialization complete.
[linked_execution_action_server] ready at linked_execution_controller/follow_joint_trajectory
```

#### 2.2 仿真模式（仅 Gazebo）

**适用场景**：算法开发、路径规划测试

```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
    sim_only:=true
```

---

### 3. 基本操作流程

#### 3.1 在 RViz 中规划运动

**步骤 1：设置目标位姿**
- 在 RViz 左侧 "MotionPlanning" 面板
- 拖动交互式标记（Interactive Marker）到目标位置
- 或点击 "Random Valid Goal" 生成随机目标

**步骤 2：规划轨迹**
- 点击 "Plan" 按钮
- 观察规划结果（橙色轨迹预览）
- 如不满意，点击 "Plan" 重新规划

**步骤 3：执行运动**
- 点击 "Execute" 按钮
- **联动模式**：实机开始运动，Gazebo 同步镜像
- **仿真模式**：仅 Gazebo 运动

**步骤 4：观察执行状态**

终端输出示例：
```
[linked_execution_action_server] received goal
[linked_execution_action_server] real robot SUCCEEDED, waiting for Gazebo...
[LinkedExecutionMonitor] SUCCEEDED after 3.2s
[linked_execution_action_server] both real and Gazebo SUCCEEDED
```

RViz 状态栏显示 "SUCCEEDED"

#### 3.2 切换规划算法

在 RViz "MotionPlanning" 面板：
- **RRTConnect**（默认）：快速，适合简单场景
- **CHOMP**：优化平滑度，适合复杂场景
- **LERP**：线性插值，适合简单直线运动

#### 3.3 调整规划参数

```bash
# OMPL 规划器参数
gedit src/aubo_robot/aubo_e5_moveit_config/config/ompl_planning.yaml

# 关节限速限加速
gedit src/aubo_robot/aubo_e5_moveit_config/config/joint_limits.yaml
```

---

### 4. 高级功能

#### 4.1 Python 脚本控制

```python
#!/usr/bin/env python3
import rospy
import moveit_commander

rospy.init_node('aubo_script_control')
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("manipulator")

# 设置目标关节角度
joint_goal = [0.0, -0.5, 1.0, -0.5, 0.0, 0.0]
group.go(joint_goal, wait=True)
group.stop()

print("Motion completed!")
```

#### 4.2 录制与回放轨迹

```bash
# 录制执行过程
rosbag record -O my_trajectory.bag \
    /joint_states \
    /joint_path_command \
    /feedback_states

# 回放（仅用于分析，不会驱动实机）
rosbag play my_trajectory.bag
```

#### 4.3 诊断工具

```bash
# 运行诊断脚本
./system_tools.sh diagnose

# 查看所有话题
rostopic list

# 查看节点图
rqt_graph

# 监控 TF 树
rosrun tf view_frames
evince frames.pdf
```

---

### 5. 故障排查

#### 5.1 实机无法上电

**现象**：终端显示 `Timeout waiting for drives_powered`

**排查步骤**：
1. 检查示教器：急停是否释放？
2. 检查示教器：是否有报警信息？
3. 手动上电测试：
   ```bash
   # 发布上电命令
   rostopic pub /robot_control std_msgs/String "data: 'powerOn'" -1
   
   # 查看机器人状态
   rostopic echo /robot_status
   ```

#### 5.2 Execute 失败但实机已到位

**现象**：实机运动完成，但 RViz 显示 "ABORTED"

**原因**：Gazebo 未收敛到目标位置

**排查步骤**：
1. 检查 Gazebo RTF（Real-Time Factor）：
   - Gazebo 窗口左下角应显示 RTF ≈ 1.0
   - 如 RTF < 0.8，说明仿真过慢
2. 调整监视器参数：
   ```bash
   # 编辑 launch 文件
   gedit src/aubo_linked_execution/launch/aubo_e5_linked_execution.launch
   
   # 修改容差（第 96 行）
   <param name="joint_tolerance" value="0.05"/>  <!-- 放宽到 0.05 rad -->
   ```

#### 5.3 Gazebo 中机器人不动

**现象**：实机运动，但 Gazebo 中模型静止

**排查步骤**：
1. 检查镜像适配器：
   ```bash
   rostopic hz /real/joint_states
   # 应显示约 10 Hz
   ```
2. 检查 Gazebo 驱动：
   ```bash
   rosnode info /aubo_gazebo_driver
   # 确认 shadow 参数为 true
   ```
3. 重启 Gazebo：
   ```bash
   rosnode kill /gazebo
   # 系统会自动重启 Gazebo
   ```

#### 5.4 网络连接问题

**现象**：`aubo_driver` 无法连接实机

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

---

### 6. 安全注意事项

#### 6.1 操作规范
- ⚠️ **首次运行前**：确保机器人周围无障碍物和人员
- ⚠️ **规划前检查**：验证目标位姿在工作空间内
- ⚠️ **执行时监控**：手放在急停按钮附近
- ⚠️ **异常立即停止**：按下急停或在 RViz 中点击 "Stop"

#### 6.2 紧急停止

**方法一：硬件急停**
- 按下示教器或控制柜上的红色急停按钮

**方法二：软件停止**
- RViz 中点击 "Stop" 按钮
- 或发布话题：
  ```bash
  rostopic pub /trajectory_execution_event std_msgs/String "data: 'stop'" -1
  ```

**方法三：关闭系统**
- 按 `Ctrl+C` 终止 launch 进程
- 机器人会自动停止并保持当前位置

---

### 7. 关闭系统

#### 7.1 正常关闭
```bash
# 在 launch 终端按 Ctrl+C
# 等待所有节点优雅退出（约 5 秒）
```

#### 7.2 强制关闭
```bash
# 如系统无响应
rosnode kill -a
killall -9 gzserver gzclient
```

#### 7.3 实机断电
1. 在示教器上切换到手动模式
2. 按下控制柜电源按钮
3. 等待示教器屏幕熄灭

---

## 第二部分：连续规划与避障测试

### 1. 连续运动规划

连续运动规划允许机器人按预定义路径点序列自动执行运动，支持三种模式：关节空间、笛卡尔空间和预定义位置。

#### 1.1 快速开始

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

#### 1.2 配置路径点

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

#### 1.3 常见问题

**Q: 连续运动执行失败**
- 检查路径点是否超出工作空间或关节限位
- 在 RViz 中手动测试每个路径点
- 查看终端日志了解具体错误信息

**Q: 笛卡尔空间规划失败**
- 检查目标位置是否在工作空间内
- 尝试调整姿态（orientation）
- 使用关节空间模式作为替代方案

---

### 2. 避障测试系统

避障测试系统在 Gazebo 中动态生成随机障碍物，用于测试 MoveIt 的避障规划能力。

#### 2.1 快速开始

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

#### 2.2 配置工作空间

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

#### 2.3 常见问题

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

## 第三部分：视觉扩展方案

### 1. 系统架构设计

#### 1.1 感知-规划-执行分离架构

```
┌─────────────────────────────────────────┐
│ 感知层 (Perception)                     │
│ 相机驱动 → 点云处理 → 目标识别 → 位姿估计│
└──────────────┬──────────────────────────┘
               │ /detected_objects
                                   ↓
┌────────────────────────────────────────┐
│  任务规划层 (Task Planning)            │
│  抓取规划 → 碰撞检测 → 轨迹优化         │
└──────────────┬──────────────────────────┘
               │ /planned_trajectory
                                   ↓
┌─────────────────────────────────────────┐
│  运动执行层 (Motion Execution)         │
│  【当前已完成的联动链路】                                   │
│  RViz/MoveIt → 实机 + Gazebo 镜像      │
└────────────────────────────────────────┘
```

#### 1.2 设计原则
- **松耦合**：视觉模块通过 ROS 话题/服务与运动控制通信
- **可测试性**：每层可独立测试（录制 bag 文件回放）
- **可替换性**：相机/算法可更换而不影响运动控制
- **Gazebo 同步**：视觉目标可在 Gazebo 中可视化验证

---

### 2. 预留接口清单

#### 2.1 感知层接口

**相机数据话题**（mecheye_ros_interface 已有）
```yaml
# 彩色图像
/camera/color/image_raw          # sensor_msgs/Image
/camera/color/camera_info        # sensor_msgs/CameraInfo

# 深度图
/camera/depth/image_raw          # sensor_msgs/Image
/camera/depth/camera_info        # sensor_msgs/CameraInfo

# 点云
/camera/point_cloud/cloud        # sensor_msgs/PointCloud2

# 相机外参（需标定）
/camera_to_base_link             # geometry_msgs/TransformStamped (TF)
```

**目标检测输出话题**（待实现）
```yaml
# 检测到的物体位姿列表
/vision/detected_objects         # vision_msgs/Detection3DArray

# 分割后的点云
/vision/segmented_objects        # sensor_msgs/PointCloud2

# 可视化标记
/vision/detection_markers        # visualization_msgs/MarkerArray
```

#### 2.2 任务规划层接口

**抓取规划服务**（待实现）
```yaml
# 服务定义示例
/grasp_planner/compute_grasps
  Request:
    target_object: vision_msgs/Detection3D
    approach_direction: geometry_msgs/Vector3
  Response:
    grasp_poses: geometry_msgs/PoseArray
    success: bool
```

**场景更新服务**（MoveIt 已有，需对接）
```yaml
# 将检测到的物体添加到 MoveIt 规划场景
/planning_scene_interface/add_object
  - 使用 moveit_msgs/CollisionObject
  - 支持点云、网格、基本几何体

# 清除场景中的物体
/planning_scene_interface/clear_objects
```

#### 2.3 Gazebo 场景同步接口

**动态物体生成**（待实现）
```yaml
# 在 Gazebo 中生成检测到的物体模型
/gazebo/spawn_sdf_model          # gazebo_msgs/SpawnModel (已有)
/gazebo/delete_model             # gazebo_msgs/DeleteModel (已有)

# 自定义服务：批量同步场景
/vision/sync_scene_to_gazebo
  Request:
    objects: vision_msgs/Detection3DArray
  Response:
    success: bool
```

---

### 3. 集成步骤

#### 阶段一：相机标定与数据采集

**目标**：建立相机坐标系到机器人基座的 TF 变换

**步骤**：
1. 物理安装相机（建议固定安装，避免手眼标定复杂度）
2. 使用 `camera_calibration` 包标定相机内参
3. 使用 ArUco 标记或棋盘格标定相机外参
4. 发布静态 TF：`base_link → camera_link`
5. 验证：在 RViz 中叠加点云与机器人模型

**关键文件**：
```bash
# 新建包
src/aubo_vision/
├── launch/
│   ├── camera_bringup.launch       # 启动相机驱动
│   └── camera_calibration.launch   # 标定工具
├── config/
│   ├── camera_extrinsics.yaml      # 外参 TF
│   └── camera_intrinsics.yaml      # 内参（备份）
└── urdf/
    └── camera.urdf.xacro           # 相机模型（可选）
```

**修改点**：
```xml
<!-- 在 aubo_e5_linked_execution.launch 中添加 -->
<arg name="enable_camera" default="false"/>
<include file="$(find aubo_vision)/launch/camera_bringup.launch" 
         if="$(arg enable_camera)"/>
```

#### 阶段二：目标检测与位姿估计（3-5 天）

**目标**：从点云中识别目标物体并估计 6D 位姿

**技术选型**：
- **简单场景**：基于几何的分割（PCL RANSAC、欧氏聚类）
- **复杂场景**：深度学习（YOLO + PnP、Dope、6D-Pose）

**步骤**：
1. 实现点云预处理节点（降采样、滤波、平面分割）
2. 实现目标检测节点（输出 `/vision/detected_objects`）
3. 在 RViz 中可视化检测结果（MarkerArray）
4. 录制测试数据集（rosbag）用于离线调试

**关键文件**：
```bash
src/aubo_vision/
├── scripts/
│   ├── pointcloud_processor.py     # 点云预处理
│   ├── object_detector.py          # 目标检测
│   └── pose_estimator.py           # 位姿估计
└── launch/
    └── vision_pipeline.launch      # 视觉处理流水线
```

#### 阶段三：MoveIt 场景同步（2-3 天）

**目标**：将检测到的物体添加到 MoveIt 规划场景，避免碰撞

**步骤**：
1. 实现场景同步节点：
   - 订阅 `/vision/detected_objects`
   - 调用 `moveit::planning_interface::PlanningSceneInterface`
   - 添加碰撞物体（使用点云或简化几何体）
2. 在 RViz 的 Planning Scene 中验证物体显示
3. 测试规划器是否正确避障

**代码示例**：
```python
#!/usr/bin/env python3
import rospy
from moveit_commander import PlanningSceneInterface
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped

class SceneSynchronizer:
    def __init__(self):
        self.scene = PlanningSceneInterface()
        rospy.Subscriber('/vision/detected_objects', 
                         Detection3DArray, self.callback)
    
    def callback(self, msg):
        # 清除旧物体
        for obj_id in self.scene.get_known_object_names():
            if obj_id.startswith('detected_'):
                self.scene.remove_world_object(obj_id)
        
        # 添加新检测到的物体
        for i, detection in enumerate(msg.detections):
            obj_id = f'detected_object_{i}'
            pose = PoseStamped()
            pose.header = detection.header
            pose.pose = detection.bbox.center
            
            # 添加为盒子（简化）
            size = (detection.bbox.size.x, 
                    detection.bbox.size.y, 
                    detection.bbox.size.z)
            self.scene.add_box(obj_id, pose, size)
        
        rospy.loginfo(f'Added {len(msg.detections)} objects to scene')

if __name__ == '__main__':
    rospy.init_node('scene_synchronizer')
    SceneSynchronizer()
    rospy.spin()
```

#### 阶段四：Gazebo 场景同步（2-3 天）

**目标**：在 Gazebo 中生成检测到的物体，实现仿真环境与真实场景一致

**步骤**：
1. 实现 Gazebo 场景同步节点
2. 订阅 `/vision/detected_objects`
3. 调用 `/gazebo/spawn_sdf_model` 生成物体
4. 在 Gazebo 中验证物体位置和碰撞

**代码示例**：
```python
#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from vision_msgs.msg import Detection3DArray

class GazeboSceneSynchronizer:
    def __init__(self):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        
        rospy.Subscriber('/vision/detected_objects', 
                         Detection3DArray, self.callback)
        self.spawned_models = []
    
    def callback(self, msg):
        # 删除旧模型
        for model_name in self.spawned_models:
            try:
                self.delete_srv(model_name)
            except:
                pass
        self.spawned_models = []
        
        # 生成新模型
        for i, detection in enumerate(msg.detections):
            model_name = f'detected_object_{i}'
            sdf = self.generate_box_sdf(detection.bbox.size)
            
            try:
                self.spawn_srv(
                    model_name=model_name,
                    model_xml=sdf,
                    robot_namespace='',
                    initial_pose=detection.bbox.center,
                    reference_frame='world'
                )
                self.spawned_models.append(model_name)
            except Exception as e:
                rospy.logerr(f'Failed to spawn {model_name}: {e}')
    
    def generate_box_sdf(self, size):
        return f'''<?xml version="1.0"?>
        <sdf version="1.6">
          <model name="box">
            <static>true</static>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <box><size>{size.x} {size.y} {size.z}</size></box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box><size>{size.x} {size.y} {size.z}</size></box>
                </geometry>
              </visual>
            </link>
          </model>
        </sdf>'''

if __name__ == '__main__':
    rospy.init_node('gazebo_scene_synchronizer')
    GazeboSceneSynchronizer()
    rospy.spin()
```

#### 阶段五：抓取规划与执行（3-5 天）

**目标**：实现完整的视觉抓取流程

**步骤**：
1. 实现抓取规划器（基于物体位姿生成抓取点）
2. 集成 MoveIt 规划器（生成接近、抓取、撤离轨迹）
3. 实现夹爪控制（如有）
4. 完整流程测试

**完整流程**：
```
相机采集 → 目标检测 → 位姿估计 → 抓取规划 → 
场景更新 → 轨迹规划 → 实机执行 → Gazebo 镜像
```

---

### 4. 测试与验证

#### 4.1 单元测试

**相机标定验证**：
```bash
# 在 RViz 中显示点云和机器人模型
roslaunch aubo_vision camera_bringup.launch
# 检查点云是否与机器人坐标系对齐
```

**目标检测验证**：
```bash
# 录制测试数据
rosbag record /camera/point_cloud/cloud -O test_scene.bag

# 离线测试检测算法
rosbag play test_scene.bag
rosrun aubo_vision object_detector.py
# 在 RViz 中查看检测结果
```

**场景同步验证**：
```bash
# 启动完整系统
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch sim_only:=true
roslaunch aubo_vision vision_pipeline.launch

# 检查 MoveIt 场景中是否显示检测到的物体
# 检查 Gazebo 中是否生成对应模型
```

#### 4.2 集成测试

**完整抓取流程测试**：
1. 放置测试物体在相机视野内
2. 启动视觉系统，确认检测成功
3. 在 RViz 中规划抓取轨迹
4. 执行抓取，观察实机和 Gazebo 同步
5. 验证碰撞检测是否正常工作

---

### 5. 注意事项

#### 5.1 坐标系管理
- 确保所有坐标系通过 TF 正确连接
- 使用 `rosrun tf view_frames` 检查 TF 树完整性
- 相机外参标定精度直接影响抓取成功率

#### 5.2 性能优化
- 点云降采样以提高处理速度
- 目标检测频率不宜过高（建议 1-5 Hz）
- 场景更新采用增量式，避免全量刷新

#### 5.3 安全考虑
- 首次测试使用 sim_only 模式验证
- 实机测试时降低运动速度
- 设置工作空间限制，避免碰撞

---

### 6. 推荐工具和库

#### 6.1 点云处理
- **PCL (Point Cloud Library)**：点云滤波、分割、配准
- **Open3D**：现代化点云处理库，Python 友好

#### 6.2 目标检测
- **YOLO + Depth**：2D 检测 + 深度图获取 3D 位置
- **Dope**：深度学习 6D 位姿估计
- **PointNet/PointNet++**：直接从点云检测

#### 6.3 抓取规划
- **GPD (Grasp Pose Detection)**：基于点云的抓取点生成
- **GraspIt!**：抓取仿真和规划工具
- **MoveIt Grasps**：MoveIt 抓取规划插件

---

### 7. 参考资源

- **MoveIt 教程**：https://ros-planning.github.io/moveit_tutorials/
- **PCL 教程**：https://pcl.readthedocs.io/
- **ROS Perception**：http://wiki.ros.org/perception
- **Gazebo 模型库**：https://github.com/osrf/gazebo_models

---

**文档版本**：v1.0  
**最后更新**：2026-04-13  
**维护者**：AUBO Polish Project Team
