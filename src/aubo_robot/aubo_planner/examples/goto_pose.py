#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
goto_pose.py  —  交互式坐标输入，自动规划执行

用法：
  rosrun aubo_planner goto_pose.py

输入格式：
  x y z              保持当前末端姿态，只移动位置
  x y z roll pitch yaw   同时指定姿态（弧度）
  q / quit           退出

坐标系：base_link（底座中心为原点，X 向前，Y 向左，Z 向上），单位：米
"""

import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

import rospy
from motion_planning_interface import MotionPlanningInterface


def get_current_rpy(move_group):
    from tf.transformations import euler_from_quaternion
    pose = move_group.get_current_pose().pose
    q = pose.orientation
    return euler_from_quaternion([q.x, q.y, q.z, q.w])


def parse_input(line, current_rpy):
    parts = line.strip().split()
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


def print_current_pose(move_group):
    pose = move_group.get_current_pose().pose
    rpy = get_current_rpy(move_group)
    print("当前 TCP  x={:.4f}  y={:.4f}  z={:.4f}"
          "  roll={:.3f}  pitch={:.3f}  yaw={:.3f}".format(
              pose.position.x, pose.position.y, pose.position.z,
              rpy[0], rpy[1], rpy[2]))


def main():
    print("正在初始化运动规划接口，请稍候...")
    planner = MotionPlanningInterface(planner_name="ompl")
    mg = planner.planner.move_group

    print("\n=== AUBO E5 交互式坐标规划 ===")
    print("坐标系: base_link | 单位: 米 | 姿态单位: 弧度")
    print("工作空间: 臂展 ≤ 0.784m，高度 0 ~ 1.224m")
    print("输入格式:")
    print("  x y z                  仅移动位置，保持当前姿态")
    print("  x y z roll pitch yaw   移动并指定姿态")
    print("  q                      退出\n")

    while not rospy.is_shutdown():
        print_current_pose(mg)

        try:
            line = input("目标坐标> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n退出")
            break

        if not line:
            continue
        if line.lower() in ('q', 'quit', 'exit'):
            print("退出")
            break

        rpy = get_current_rpy(mg)
        parsed = parse_input(line, rpy)
        if parsed is None:
            print("格式错误。示例: 0.4 0.0 0.5  或  0.4 0.0 0.5 0 1.57 0\n")
            continue

        x, y, z, roll, pitch, yaw = parsed
        dist = math.sqrt(x**2 + y**2 + z**2)

        print("目标    x={:.4f}  y={:.4f}  z={:.4f}"
              "  roll={:.3f}  pitch={:.3f}  yaw={:.3f}  (距原点 {:.3f}m)".format(
                  x, y, z, roll, pitch, yaw, dist))

        if dist > 0.80:
            print("提示: 目标距原点 {:.3f}m，接近或超出臂展上限 (0.784m)，规划可能失败".format(dist))

        try:
            input("按 Enter 规划并执行，Ctrl+C 取消> ")
        except KeyboardInterrupt:
            print("\n已取消\n")
            continue

        print("规划中...")
        trajectory = planner.plan_to_pose(x, y, z, roll, pitch, yaw)
        if trajectory is None:
            print("规划失败（目标不可达或超出关节限位）\n")
            continue

        print("规划成功，执行中... (按 Ctrl+C 可中断运动)")
        try:
            success = planner.execute(trajectory, blocking=True)
            if success:
                print("执行成功\n")
            else:
                print("执行失败")
                # 获取当前关节位置和目标关节位置，计算误差
                current_joints = mg.get_current_joint_values()
                target_joints = trajectory.joint_trajectory.points[-1].positions

                # 计算每个关节的误差
                errors = [abs(c - t) for c, t in zip(current_joints, target_joints)]
                max_error = max(errors)
                max_error_idx = errors.index(max_error)

                print(f"  最大关节误差: {max_error:.4f} rad ({math.degrees(max_error):.2f}°) 在关节 {max_error_idx + 1}")
                print(f"  各关节误差(rad): [{', '.join(f'{e:.4f}' for e in errors)}]")

                # 获取当前笛卡尔位置
                current_pose = mg.get_current_pose().pose
                print(f"  当前笛卡尔位置: x={current_pose.position.x:.4f}, y={current_pose.position.y:.4f}, z={current_pose.position.z:.4f}")
                print(f"  目标笛卡尔位置: x={x:.4f}, y={y:.4f}, z={z:.4f}")
                cart_error = math.sqrt((current_pose.position.x - x)**2 +
                                      (current_pose.position.y - y)**2 +
                                      (current_pose.position.z - z)**2)
                print(f"  笛卡尔误差: {cart_error:.4f} m\n")
        except KeyboardInterrupt:
            print("\n运动已中断，正在停止...")
            mg.stop()
            print("已停止\n")
            continue


if __name__ == '__main__':
    main()
