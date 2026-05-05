# Unity Shadow 模式抽搐问题修复

## 问题描述
实机到达位置长时间停止时，Unity 虽然跟踪到了，但会不断小幅度抽搐。

## 根本原因
1. **传感器噪声**：实机静止时，关节状态有微小噪声（±0.0001 rad）
2. **无位置死区**：所有位置变化都被转发到 Unity，包括噪声
3. **速度滤波不足**：`filter_alpha=0.3` 对噪声平滑效果不够强
4. **无速度阈值**：即使实机静止（速度≈0），仍持续发送指令

## 修复方案

### 1. 增加位置死区 (Position Deadzone)
- 默认值: `0.001 rad` (≈0.057°)
- 只有当位置变化超过死区时才发送指令
- 过滤掉传感器噪声引起的微小抖动

### 2. 增加速度阈值 (Velocity Threshold)
- 默认值: `0.005 rad/s` (≈0.286°/s)
- 当速度低于阈值时，视为静止，不发送指令
- 防止实机静止时持续发送指令

### 3. 改进速度滤波
- `filter_alpha` 从 `0.3` 增加到 `0.5`
- 更强的平滑效果，减少速度噪声

## 修改文件
- `scripts/unity_command_forwarder.py` - 添加死区和阈值逻辑
- `config/unity_shadow_tuning.yaml` - 调优参数配置

## 使用方法

### 默认参数（推荐）
```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.1.10
```

### 自定义参数
```bash
roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
  robot_ip:=192.168.1.10 \
  position_deadzone:=0.002 \
  velocity_threshold:=0.01 \
  shadow_filter_alpha:=0.7
```

## 调优建议

| 现象 | 调整参数 | 方向 |
|------|---------|------|
| Unity 仍然抽搐 | `position_deadzone` | 增加到 0.002 |
| Unity 响应太慢 | `position_deadzone` | 减小到 0.0005 |
| 需要更强平滑 | `shadow_filter_alpha` | 增加到 0.7 |
| 需要更快响应 | `shadow_filter_alpha` | 减小到 0.3 |
| 静止时仍有微动 | `velocity_threshold` | 增加到 0.01 |

## 技术细节

### 死区逻辑
```python
max_position_change = max(abs(c - l) for c, l in zip(current, last_published))
max_velocity = max(abs(v) for v in velocity_filter)

if max_position_change < position_deadzone and max_velocity < velocity_threshold:
    return  # 跳过发布
```

### 速度滤波（指数移动平均）
```python
velocity_filter = alpha * raw_velocity + (1 - alpha) * velocity_filter
```

## 测试验证
1. 启动联动模式
2. 示教实机到一个位置并停止
3. 观察 Unity 是否静止（无抽搐）
4. 移动实机，观察 Unity 是否平滑跟随
5. 如有问题，根据调优建议调整参数
