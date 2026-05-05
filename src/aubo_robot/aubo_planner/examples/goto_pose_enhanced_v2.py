#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
goto_pose_enhanced_v2.py - 完全增强版交互式坐标输入

新增功能：
- 彩色输出（绿/黄/红）
- 关节状态显示（角度 + 进度条）
- 笛卡尔位姿详细显示
- 执行统计（成功率、平均时间）
- 详细误差分析
- 预设位置支持（home, test1, test2）
- 相对运动支持（+x 0.1, +y -0.05 等）
- 命令历史记录（上下箭头）
- 智能警告和错误恢复建议

用法：
  rosrun aubo_planner goto_pose_enhanced_v2.py

输入格式：
  x y z                    仅移动位置，保持当前姿态
  x y z roll pitch yaw     同时指定姿态（弧度）
  +x 0.1                   相对运动（X 方向移动 0.1m）
  +y -0.05 +z 0.2          相对运动（多轴同时）
  home                     移动到预设位置
  test1 / test2            移动到预设测试位置
  presets                  显示所有预设位置
  stats                    显示统计信息
  q / quit                 退出
"""

import sys
import os
import math
import time
import readline
from typing import Optional, Tuple, Dict

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

import rospy
from motion_planning_interface import MotionPlanningInterface

# ANSI 颜色代码
class Colors:
    RESET = '\033[0m'
    BOLD = '\033[1m'
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GRAY = '\033[90m'

# 预设位置（x, y, z, roll, pitch, yaw）
PRESETS: Dict[str, Tuple[float, float, float, float, float, float]] = {
    'home': (0.4, 0.0, 0.5, 0.0, 1.57, 0.0),
    'test1': (0.5, 0.2, 0.4, 0.0, 1.57, 0.0),
    'test2': (0.3, -0.2, 0.6, 0.0, 1.57, 0.5),
}

# 执行统计
class ExecutionStats:
    def __init__(self):
        self.total_attempts = 0
        self.successes = 0
        self.failures = 0
        self.total_time = 0.0
        self.execution_times = []

    def record_success(self, duration):
        self.total_attempts += 1
        self.successes += 1
        self.total_time += duration
        self.execution_times.append(duration)

    def record_failure(self, duration):
        self.total_attempts += 1
        self.failures += 1
        self.total_time += duration
        self.execution_times.append(duration)

    def get_success_rate(self):
        return (self.successes / self.total_attempts * 100) if self.total_attempts > 0 else 0.0

    def get_avg_time(self):
        return (self.total_time / self.total_attempts) if self.total_attempts > 0 else 0.0

stats = ExecutionStats()

def print_header():
    """打印美化的标题"""
    print(f"\n{Colors.CYAN}{Colors.BOLD}{'='*70}{Colors.RESET}")
    print(f"{Colors.CYAN}{Colors.BOLD}  AUBO E5 交互式坐标规划 - 完全增强版{Colors.RESET}")
    print(f"{Colors.CYAN}{Colors.BOLD}{'='*70}{Colors.RESET}")
    print(f"{Colors.GRAY}坐标系: base_link | 单位: 米 | 姿态单位: 弧度{Colors.RESET}")
    print(f"{Colors.GRAY}工作空间: 臂展 ≤ 0.784m，高度 0 ~ 1.224m{Colors.RESET}\n")
    print(f"{Colors.YELLOW}输入格式:{Colors.RESET}")
    print(f"  {Colors.GREEN}x y z{Colors.RESET}                  仅移动位置，保持当前姿态")
    print(f"  {Colors.GREEN}x y z roll pitch yaw{Colors.RESET}   移动并指定姿态")
    print(f"  {Colors.GREEN}+x 0.1{Colors.RESET}                 相对运动（X 方向移动 0.1m）")
    print(f"  {Colors.GREEN}+y -0.05 +z 0.2{Colors.RESET}        相对运动（多轴同时）")
    print(f"  {Colors.GREEN}home / test1 / test2{Colors.RESET}   移动到预设位置")
    print(f"  {Colors.GREEN}presets{Colors.RESET}                显示所有预设位置")
    print(f"  {Colors.GREEN}stats{Colors.RESET}                  显示执行统计")
    print(f"  {Colors.GREEN}q{Colors.RESET}                      退出")
    print(f"\n{Colors.YELLOW}快捷键:{Colors.RESET}")
    print(f"  {Colors.GREEN}↑/↓{Colors.RESET}                    浏览命令历史")
    print(f"  {Colors.GREEN}Ctrl+C{Colors.RESET}                 中断运动或退出程序\n")

def get_current_rpy(move_group):
    """获取当前末端姿态（RPY）"""
    from tf.transformations import euler_from_quaternion
    pose = move_group.get_current_pose().pose
    q = pose.orientation
    return euler_from_quaternion([q.x, q.y, q.z, q.w])

def make_progress_bar(value, max_value, width=20):
    """创建进度条"""
    percentage = min(value / max_value, 1.0) if max_value > 0 else 0
    filled = int(width * percentage)
    bar = '█' * filled + '░' * (width - filled)

    if percentage < 0.3:
        color = Colors.GREEN
    elif percentage < 0.7:
        color = Colors.YELLOW
    else:
        color = Colors.RED

    return f"{color}{bar}{Colors.RESET} {percentage*100:5.1f}%"

def print_joint_status(move_group):
    """打印详细的关节状态"""
    joints = move_group.get_current_joint_values()
    joint_names = ["肩关节", "大臂", "小臂", "腕1", "腕2", "腕3"]
    joint_limits = [
        (-3.05, 3.05), (-3.05, 3.05), (-3.05, 3.05),
        (-3.05, 3.05), (-3.05, 3.05), (-6.28, 6.28),
    ]

    print(f"\n{Colors.CYAN}{Colors.BOLD}关节状态:{Colors.RESET}")
    print(f"{Colors.GRAY}{'关节':<8} {'角度(rad)':<12} {'角度(°)':<10} {'限位使用率'}{Colors.RESET}")
    print(f"{Colors.GRAY}{'-'*60}{Colors.RESET}")

    for i, (name, angle) in enumerate(zip(joint_names, joints)):
        min_limit, max_limit = joint_limits[i]
        angle_deg = math.degrees(angle)
        progress = make_progress_bar(abs(angle), max(abs(min_limit), abs(max_limit)))
        print(f"{name:<8} {angle:>10.4f}  {angle_deg:>8.2f}°  {progress}")

def print_cartesian_pose(move_group):
    """打印笛卡尔位姿"""
    pose = move_group.get_current_pose().pose
    rpy = get_current_rpy(move_group)

    print(f"\n{Colors.CYAN}{Colors.BOLD}笛卡尔位姿:{Colors.RESET}")
    print(f"{Colors.GRAY}{'坐标':<10} {'值':<12} {'说明'}{Colors.RESET}")
    print(f"{Colors.GRAY}{'-'*50}{Colors.RESET}")
    print(f"{'x':<10} {pose.position.x:>10.4f} m  {Colors.GRAY}前后（正向前）{Colors.RESET}")
    print(f"{'y':<10} {pose.position.y:>10.4f} m  {Colors.GRAY}左右（正向左）{Colors.RESET}")
    print(f"{'z':<10} {pose.position.z:>10.4f} m  {Colors.GRAY}上下（正向上）{Colors.RESET}")
    print(f"{'roll':<10} {rpy[0]:>10.4f} rad ({math.degrees(rpy[0]):>6.2f}°)")
    print(f"{'pitch':<10} {rpy[1]:>10.4f} rad ({math.degrees(rpy[1]):>6.2f}°)")
    print(f"{'yaw':<10} {rpy[2]:>10.4f} rad ({math.degrees(rpy[2]):>6.2f}°)")

    dist = math.sqrt(pose.position.x**2 + pose.position.y**2 + pose.position.z**2)
    print(f"\n{Colors.GRAY}到原点距离: {dist:.4f} m{Colors.RESET}")

def print_stats():
    """打印执行统计"""
    print(f"\n{Colors.CYAN}{Colors.BOLD}执行统计:{Colors.RESET}")
    print(f"{Colors.GRAY}{'-'*50}{Colors.RESET}")
    print(f"总尝试次数: {stats.total_attempts}")
    print(f"{Colors.GREEN}成功: {stats.successes}{Colors.RESET}")
    print(f"{Colors.RED}失败: {stats.failures}{Colors.RESET}")

    if stats.total_attempts > 0:
        success_rate = stats.get_success_rate()
        color = Colors.GREEN if success_rate >= 80 else Colors.YELLOW if success_rate >= 60 else Colors.RED
        print(f"成功率: {color}{success_rate:.1f}%{Colors.RESET}")
        print(f"平均执行时间: {stats.get_avg_time():.2f} 秒")

        if stats.execution_times:
            print(f"最快: {min(stats.execution_times):.2f} 秒")
            print(f"最慢: {max(stats.execution_times):.2f} 秒")

def print_presets():
    """打印所有预设位置"""
    print(f"\n{Colors.CYAN}{Colors.BOLD}预设位置:{Colors.RESET}")
    print(f"{Colors.GRAY}{'-'*70}{Colors.RESET}")
    for name, (x, y, z, roll, pitch, yaw) in PRESETS.items():
        dist = math.sqrt(x**2 + y**2 + z**2)
        print(f"{Colors.GREEN}{name:<10}{Colors.RESET} x={x:.3f} y={y:.3f} z={z:.3f} "
              f"r={roll:.2f} p={pitch:.2f} y={yaw:.2f} {Colors.GRAY}(距离: {dist:.3f}m){Colors.RESET}")

def parse_relative_motion(line, current_pose, current_rpy):
    """解析相对运动指令"""
    parts = line.strip().split()
    x, y, z = current_pose.position.x, current_pose.position.y, current_pose.position.z
    roll, pitch, yaw = current_rpy

    i = 0
    while i < len(parts):
        if parts[i].startswith('+'):
            axis = parts[i][1:].lower()
            if i + 1 >= len(parts):
                return None
            try:
                value = float(parts[i + 1])
            except ValueError:
                return None

            if axis == 'x':
                x += value
            elif axis == 'y':
                y += value
            elif axis == 'z':
                z += value
            elif axis == 'roll':
                roll += value
            elif axis == 'pitch':
                pitch += value
            elif axis == 'yaw':
                yaw += value
            else:
                return None
            i += 2
        else:
            return None

    return x, y, z, roll, pitch, yaw

def parse_input(line, current_pose, current_rpy):
    """解析用户输入"""
    line = line.strip()

    # 检查是否是预设位置
    if line.lower() in PRESETS:
        return PRESETS[line.lower()]

    # 检查是否是相对运动
    if line.startswith('+'):
        return parse_relative_motion(line, current_pose, current_rpy)

    # 解析绝对坐标
    parts = line.split()
    try:
        if len(parts) == 3:
            x, y, z = map(float, parts)
            roll, pitch, yaw = current_rpy
        elif len(parts) == 6:
            x, y, z, roll, pitch, yaw = map(float, parts)
        else:
            return None
    except ValueError:
        return None
    return x, y, z, roll, pitch, yaw

def print_error_analysis(mg, trajectory, x, y, z):
    """打印详细的误差分析"""
    current_joints = mg.get_current_joint_values()
    target_joints = trajectory.joint_trajectory.points[-1].positions

    errors = [abs(c - t) for c, t in zip(current_joints, target_joints)]
    max_error = max(errors)
    max_error_idx = errors.index(max_error)

    print(f"\n{Colors.RED}{Colors.BOLD}误差分析:{Colors.RESET}")
    print(f"{Colors.GRAY}{'-'*70}{Colors.RESET}")
    print(f"{Colors.YELLOW}最大关节误差:{Colors.RESET} {max_error:.4f} rad ({math.degrees(max_error):.2f}°) 在关节 {max_error_idx + 1}")
    print(f"{Colors.YELLOW}各关节误差(rad):{Colors.RESET} [{', '.join(f'{e:.4f}' for e in errors)}]")

    current_pose = mg.get_current_pose().pose
    cart_error = math.sqrt((current_pose.position.x - x)**2 +
                          (current_pose.position.y - y)**2 +
                          (current_pose.position.z - z)**2)

    print(f"\n{Colors.YELLOW}笛卡尔误差:{Colors.RESET}")
    print(f"  当前位置: x={current_pose.position.x:.4f}, y={current_pose.position.y:.4f}, z={current_pose.position.z:.4f}")
    print(f"  目标位置: x={x:.4f}, y={y:.4f}, z={z:.4f}")
    print(f"  位置误差: {cart_error:.4f} m ({cart_error*1000:.2f} mm)")

    print(f"\n{Colors.CYAN}建议:{Colors.RESET}")
    if max_error > 0.1:
        print(f"  {Colors.YELLOW}•{Colors.RESET} 关节误差较大，建议放宽收敛容差")
    if cart_error < 0.005 and max_error > 0.05:
        print(f"  {Colors.YELLOW}•{Colors.RESET} 笛卡尔误差小但关节误差大，可能是 IK 解的问题")

def main():
    print_header()

    print(f"{Colors.YELLOW}正在初始化运动规划接口...{Colors.RESET}")
    planner = MotionPlanningInterface(planner_name="ompl")
    mg = planner.planner.move_group
    print(f"{Colors.GREEN}✓ 初始化完成{Colors.RESET}\n")

    while not rospy.is_shutdown():
        print_joint_status(mg)
        print_cartesian_pose(mg)

        try:
            line = input(f"\n{Colors.BOLD}{Colors.GREEN}目标坐标>{Colors.RESET} ").strip()
        except (EOFError, KeyboardInterrupt):
            print(f"\n{Colors.YELLOW}退出{Colors.RESET}")
            break

        if not line:
            continue

        if line.lower() in ('q', 'quit', 'exit'):
            print(f"{Colors.YELLOW}退出{Colors.RESET}")
            break
        elif line.lower() == 'stats':
            print_stats()
            continue
        elif line.lower() == 'presets':
            print_presets()
            continue

        current_pose = mg.get_current_pose().pose
        rpy = get_current_rpy(mg)
        parsed = parse_input(line, current_pose, rpy)
        if parsed is None:
            print(f"{Colors.RED}✗ 格式错误{Colors.RESET}")
            print(f"{Colors.GRAY}示例: 0.4 0.0 0.5  或  +x 0.1  或  home{Colors.RESET}\n")
            continue

        x, y, z, roll, pitch, yaw = parsed
        dist = math.sqrt(x**2 + y**2 + z**2)

        print(f"\n{Colors.CYAN}目标位置:{Colors.RESET}")
        print(f"  x={x:.4f}, y={y:.4f}, z={z:.4f}")
        print(f"  roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")
        print(f"  {Colors.GRAY}(距原点 {dist:.3f}m){Colors.RESET}")

        if dist > 0.80:
            print(f"{Colors.RED}⚠ 警告: 目标距原点 {dist:.3f}m，接近或超出臂展上限 (0.784m){Colors.RESET}")

        try:
            input(f"{Colors.GRAY}按 Enter 规划并执行，Ctrl+C 取消>{Colors.RESET} ")
        except KeyboardInterrupt:
            print(f"\n{Colors.YELLOW}已取消{Colors.RESET}\n")
            continue

        print(f"{Colors.YELLOW}规划中...{Colors.RESET}")
        start_time = time.time()
        trajectory = planner.plan_to_pose(x, y, z, roll, pitch, yaw)

        if trajectory is None:
            print(f"{Colors.RED}✗ 规划失败（目标不可达或超出关节限位）{Colors.RESET}\n")
            stats.record_failure(time.time() - start_time)
            continue

        plan_time = time.time() - start_time
        print(f"{Colors.GREEN}✓ 规划成功{Colors.RESET} {Colors.GRAY}({plan_time:.2f}秒){Colors.RESET}")
        print(f"{Colors.YELLOW}执行中... (按 Ctrl+C 可中断运动){Colors.RESET}")

        try:
            exec_start = time.time()
            success = planner.execute(trajectory, blocking=True)
            duration = time.time() - start_time

            if success:
                print(f"{Colors.GREEN}{Colors.BOLD}✓ 执行成功{Colors.RESET} {Colors.GRAY}(总时间: {duration:.2f}秒){Colors.RESET}\n")
                stats.record_success(duration)
            else:
                print(f"{Colors.RED}{Colors.BOLD}✗ 执行失败{Colors.RESET} {Colors.GRAY}(总时间: {duration:.2f}秒){Colors.RESET}")
                stats.record_failure(duration)
                print_error_analysis(mg, trajectory, x, y, z)
                print()

        except KeyboardInterrupt:
            print(f"\n{Colors.YELLOW}运动已中断，正在停止...{Colors.RESET}")
            mg.stop()
            print(f"{Colors.GREEN}✓ 已停止{Colors.RESET}\n")
            stats.record_failure(time.time() - start_time)
            continue

    if stats.total_attempts > 0:
        print_stats()

    print(f"\n{Colors.CYAN}感谢使用！{Colors.RESET}\n")

if __name__ == '__main__':
    main()
