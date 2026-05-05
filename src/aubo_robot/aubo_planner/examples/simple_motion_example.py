#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
运动规划示例
===================
本示例通过三个测试动作验证自动路径规划功能，适合实机测试。

安全特性：
- 相对于当前位置的适度移动（10~20cm）
- 全速运行（100%）
- 执行前需要用户确认
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from motion_planning_interface import MotionPlanningInterface


def print_safety_warning():
    """打印安全警告并等待用户确认"""
    print("=" * 70)
    print("⚠️  警告：即将在实机上执行运动！")
    print("=" * 70)
    print("\n请确保：")
    print("  1. 机器人周围无障碍物和人员")
    print("  2. 急停按钮随时可以按下")
    print("  3. 已在仿真环境中测试过此程序")
    print("  4. 机器人处于安全的初始位置")
    print("\n本示例运动参数：")
    print("  - 移动距离：10~20cm（相对当前位置）")
    print("  - 速度缩放：100%（全速）")
    print("=" * 70)

    try:
        response = input("\n按 Enter 键继续，或 Ctrl+C 取消... ")
        return True
    except KeyboardInterrupt:
        print("\n\n已取消执行")
        return False


def test_small_cartesian_move(planner):
    """
    测试1：笛卡尔空间移动

    从当前位置向上移动 15cm，再向前移动 10cm
    """
    print("\n" + "=" * 70)
    print("测试 1: 笛卡尔空间移动（向上 15cm + 向前 10cm）")
    print("=" * 70)

    # 获取当前位置
    current_pose = planner.planner.move_group.get_current_pose().pose
    print(f"\n当前位置：")
    print(f"  x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")

    # 目标位置：向上 15cm，向前 10cm
    target_x = current_pose.position.x + 0.10  # 向前 10cm
    target_y = current_pose.position.y
    target_z = current_pose.position.z + 0.15  # 向上 15cm

    print(f"\n目标位置：")
    print(f"  x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}")
    print(f"  移动：向上 15cm，向前 10cm")

    # 规划
    print("\n正在规划...")
    trajectory = planner.plan_to_pose(
        x=target_x, y=target_y, z=target_z,
        roll=0, pitch=1.57, yaw=0  # 保持姿态不变
    )

    if trajectory:
        print("✓ 规划成功！")
        print("\n按 Enter 执行，或 Ctrl+C 跳过...")
        try:
            input()
            success = planner.execute(trajectory, blocking=True)
            print(f"\n{'✓ 执行成功' if success else '✗ 执行失败'}")
            return success
        except KeyboardInterrupt:
            print("\n已跳过执行")
            return False
    else:
        print("✗ 规划失败")
        return False


def test_joint_space_move(planner):
    """
    测试2：关节空间移动

    关节1（腰部旋转）移动 20 度，关节3（大臂）移动 15 度
    """
    print("\n" + "=" * 70)
    print("测试 2: 关节空间移动（腰部 20°，大臂 15°）")
    print("=" * 70)

    # 获取当前关节角度
    current_joints = planner.planner.move_group.get_current_joint_values()
    print(f"\n当前关节角度（弧度）：")
    for i, angle in enumerate(current_joints):
        print(f"  joint_{i+1}: {angle:.3f} rad ({angle * 57.3:.1f}°)")

    # 目标关节角度：关节1 旋转 20°，关节3 弯曲 15°，其余不动
    import math
    target_joints = list(current_joints)
    target_joints[0] += math.radians(20)   # 腰部旋转 20°
    target_joints[2] += math.radians(15)   # 大臂弯曲 15°

    print(f"\n目标关节角度（弧度）：")
    for i, angle in enumerate(target_joints):
        print(f"  joint_{i+1}: {angle:.3f} rad ({angle * 57.3:.1f}°)")
    print(f"  关节1（腰部）变化：+20°，关节3（大臂）变化：+15°")

    # 规划
    print("\n正在规划...")
    trajectory = planner.plan_to_joint_values(target_joints)

    if trajectory:
        print("✓ 规划成功！")
        print("\n按 Enter 执行，或 Ctrl+C 跳过...")
        try:
            input()
            success = planner.execute(trajectory, blocking=True)
            print(f"\n{'✓ 执行成功' if success else '✗ 执行失败'}")
            return success
        except KeyboardInterrupt:
            print("\n已跳过执行")
            return False
    else:
        print("✗ 规划失败")
        return False


def test_return_to_home(planner):
    """
    测试3：返回初始位置（安全）

    返回到预定义的安全 Home 位置
    这个测试安全因为：
    - Home 位置是经过验证的安全位置
    - 可以在测试后恢复到已知状态
    """
    print("\n" + "=" * 70)
    print("测试 3: 返回 Home 位置")
    print("=" * 70)

    # 使用 MoveIt 的命名目标
    print("\n正在规划返回 Home...")
    success = planner.planner.move_group.set_named_target("home")

    if success:
        print("✓ 规划成功！")
        print("\n按 Enter 执行，或 Ctrl+C 跳过...")
        try:
            input()
            success = planner.planner.move_group.go(wait=True)
            planner.planner.move_group.stop()
            planner.planner.move_group.clear_pose_targets()
            print(f"\n{'✓ 执行成功' if success else '✗ 执行失败'}")
            return success
        except KeyboardInterrupt:
            print("\n已跳过执行")
            return False
    else:
        print("✗ 规划失败（可能未定义 home 位置）")
        return False


def main():
    """主函数：执行安全的运动测试序列"""

    # 显示安全警告
    if not print_safety_warning():
        return

    # 初始化接口
    print("\n正在初始化运动规划接口...")
    planner = MotionPlanningInterface(planner_name="ompl")

    # 显示当前状态
    print("\n当前机器人状态：")
    current_pose = planner.planner.move_group.get_current_pose().pose
    print(f"  位置：x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
    current_joints = planner.planner.move_group.get_current_joint_values()
    print(f"  关节角度：{[f'{j:.2f}' for j in current_joints]}")

    # 执行测试序列
    print("\n" + "=" * 70)
    print("开始安全测试序列")
    print("=" * 70)
    print("\n提示：每个测试前都可以按 Ctrl+C 跳过")

    results = []

    # 测试 1：笛卡尔空间小幅度移动
    results.append(("笛卡尔空间移动（向上15cm+向前10cm）", test_small_cartesian_move(planner)))

    # 测试 2：关节空间移动
    results.append(("关节空间移动（腰部20°+大臂15°）", test_joint_space_move(planner)))

    # 测试 3：返回 Home
    results.append(("返回 Home 位置", test_return_to_home(planner)))

    # 显示测试结果摘要
    print("\n" + "=" * 70)
    print("测试结果摘要")
    print("=" * 70)
    for test_name, success in results:
        status = "✓ 成功" if success else "✗ 失败/跳过"
        print(f"  {test_name}: {status}")

    print("\n测试完成！")


if __name__ == '__main__':
    main()
