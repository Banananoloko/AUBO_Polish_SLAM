# AUBO E5 示教-回放系统使用指南

提供两套工具，按复杂度选用：

| 版本 | 脚本 | 适用场景 |
|------|------|---------|
| **完整版** | `teach_waypoints.py` + `playback_waypoints.py` | 循环/单步/反向执行、速度控制 |
| **极简版** | `simple_teach.py` + `simple_playback.py` | 快速记录回放，无需参数 |

快速启动（完整版）：`./run_teach_playback.sh`

---

## 一、完整版

### 1. 启动联动系统

```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.10.230
```

### 2. 示教（teach_waypoints.py）

```bash
rosrun aubo_linked_execution teach_waypoints.py
```

| 命令 | 说明 |
|------|------|
| `r [name]` | 记录当前位置 |
| `l` | 列出路径点 |
| `d <index>` | 删除路径点 |
| `u` | 撤销上一步 |
| `s <name>` | 保存到文件 |
| `load <name>` | 加载配置 |
| `status` | 当前机械臂状态 |
| `q` | 退出 |

```bash
teach> r home
teach> r pick
teach> r place
teach> s pick_place
```

### 3. 回放（playback_waypoints.py）

```bash
rosrun aubo_linked_execution playback_waypoints.py --config <name> --mode <mode>
```

**执行模式**：

```bash
# 单次执行
rosrun aubo_linked_execution playback_waypoints.py --config demo_path --mode single

# 循环执行（无限 / 指定次数）
rosrun aubo_linked_execution playback_waypoints.py --config demo_path --mode loop
rosrun aubo_linked_execution playback_waypoints.py --config demo_path --mode loop --loop-count 5

# 单步执行（每步手动确认）
rosrun aubo_linked_execution playback_waypoints.py --config demo_path --mode step

# 反向执行
rosrun aubo_linked_execution playback_waypoints.py --config demo_path --mode reverse
```

**常用参数**：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--velocity` | 速度比例 0.1-1.0 | 0.5 |
| `--pause` | 路径点间停留秒数 | 0.5 |
| `--loop-count` | 循环次数 | 无限 |
| `--planner` | 规划器 | RRTConnect |

### 4. 配置文件格式

保存路径：`~/aubo_polish/waypoints_config/<name>.yaml`

```yaml
name: demo_path
created_at: "2026-05-09 10:30:00"
waypoints:
  - name: point_1
    index: 0
    joints: [0.0, -0.5, 1.0, 0.0, 1.5, 0.0]     # 单位：rad
    cartesian:
      position: [0.5, 0.0, 0.3]                   # 单位：m
      orientation_rpy: [0.0, 0.0, 0.0]
      orientation_quat: [0.0, 0.0, 0.0, 1.0]
execution_params:
  velocity_scaling: 0.5
  goal_tolerance: 0.01
```

---

## 二、极简版

适合只需要"记录→回放"而不需要循环/参数调整的场景。代码量 < 200 行。

### 1. 示教（simple_teach.py）

```bash
rosrun aubo_linked_execution simple_teach.py
```

交互只有三个命令：`1` 记录 / `s` 保存 / `q` 退出。

```
> 1        # 手动移动机器臂到位置后输入
✓ 已记录路径点 1
> s
文件名> my_task
✓ 已保存 3 个路径点到 my_task.yaml
```

### 2. 回放（simple_playback.py）

```bash
rosrun aubo_linked_execution simple_playback.py \
    ~/aubo_polish/waypoints_config/my_task.yaml
```

顺序执行所有路径点，到最后一个后待机。

---

## 三、常见问题

**Q：如何手动移动机器臂示教？**  
拖动模式（示教器切换）或在 RViz 中拖动 Interactive Marker。

**Q：回放规划失败？**  
路径点可能超出工作空间（臂展 ≤ 0.784m，关节限位 ±3.05 rad）。完整版自动重试 3 次，失败后检查并调整路径点。

**Q：需要循环/单步/反向执行？**  
使用完整版 `playback_waypoints.py` 的 `--mode` 参数。极简版只支持顺序单次回放。

**Q：Unity 不同步？**  
确认启动时包含 `use_unity:=true`，Unity 已点击 Play。
