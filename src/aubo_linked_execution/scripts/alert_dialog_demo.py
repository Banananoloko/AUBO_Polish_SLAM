#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
alert_dialog_demo.py - Alert Dialog System Demo

Demonstrates all types of dialogs and use cases
"""

import sys
import os

# 添加路径
sys.path.insert(0, os.path.dirname(__file__))

from alert_dialog import AlertDialog


def demo_info():
    """Demo info dialog"""
    print("\n=== Demo 1: Info Dialog ===")
    print("Purpose: System startup, status updates, normal information")

    AlertDialog.info(
        title="System Started Successfully",
        message="AUBO E5 linked execution system started successfully",
        details=(
            "Real Robot: Connected (192.168.10.230)\n"
            "Gazebo: Started\n"
            "Unity: Connected\n"
            "Safety Monitor: Running\n"
            "All components operational"
        )
    )


def demo_warning():
    """Demo warning dialog"""
    print("\n=== Demo 2: Warning Dialog ===")
    print("Purpose: Gazebo convergence slow, performance warnings")

    AlertDialog.warning(
        title="Gazebo Convergence Slow",
        message="Gazebo virtual robot convergence time exceeded expected, may affect execution efficiency",
        details=(
            "Expected convergence time: 3.0 seconds\n"
            "Actual convergence time: 5.2 seconds\n"
            "Gazebo RTF: 0.85 (below normal 1.0)\n"
            "\n"
            "Suggested actions:\n"
            "1. Check Gazebo physics engine settings\n"
            "2. Reduce simulation complexity\n"
            "3. Increase PID controller parameters"
        )
    )


def demo_error():
    """Demo error dialog"""
    print("\n=== Demo 3: Error Dialog (with sound) ===")
    print("Purpose: Gazebo convergence failed, execution failed")

    AlertDialog.error(
        title="Gazebo Convergence Failed",
        message="Gazebo virtual robot failed to converge to target position within timeout",
        details=(
            "Waypoint: 3/10\n"
            "Target joint angles: [0.3, -0.6, 0.7, 0.0, 0.5, 0.0]\n"
            "Current joint angles: [0.25, -0.57, 0.66, 0.01, 0.48, 0.0]\n"
            "\n"
            "Joint errors:\n"
            "  shoulder_joint: 0.05 rad (2.86°)\n"
            "  upperArm_joint: 0.03 rad (1.72°)\n"
            "  foreArm_joint: 0.04 rad (2.29°)\n"
            "\n"
            "Max error: 0.05 rad (2.86°)\n"
            "Tolerance: 0.03 rad (1.72°)\n"
            "Timeout: 8.0 seconds\n"
            "Error code: GOAL_TOLERANCE_VIOLATED"
        ),
        sound=True
    )


def demo_critical():
    """Demo critical error dialog"""
    print("\n=== Demo 4: Critical Error Dialog (with sound) ===")
    print("Purpose: Safety check failed, collision risk")

    AlertDialog.critical(
        title="Safety Check Failed - Large Motion Detected",
        message="Detected large difference between planned trajectory start and current position, collision risk!",
        details=(
            "Current position: [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]\n"
            "Planned start: [0.8, -0.3, 0.3, 0.0, 0.3, 0.0]\n"
            "\n"
            "Joint errors:\n"
            "  shoulder_joint: 0.8 rad (45.8°) ⚠️ EXCEEDED\n"
            "  upperArm_joint: 0.2 rad (11.5°)\n"
            "  foreArm_joint: 0.2 rad (11.5°)\n"
            "\n"
            "Max error: 0.8 rad (45.8°)\n"
            "Safety threshold: 0.5 rad (28.6°)\n"
            "\n"
            "⚠️ Suggested actions:\n"
            "1. Stop execution immediately\n"
            "2. Check robot current position\n"
            "3. Verify if robot was manually moved\n"
            "4. Re-plan trajectory"
        ),
        sound=True
    )


def demo_pose_difference():
    """Demo Gazebo pose difference dialog"""
    print("\n=== Demo 5: Gazebo Pose Difference Dialog ===")
    print("Purpose: Gazebo restarted after abort, large pose difference detected")

    AlertDialog.error(
        title="Gazebo Pose Difference Too Large",
        message="After Gazebo restart, detected large pose difference between real robot and Gazebo",
        details=(
            "Real robot position: [0.4, 0.0, 0.5]\n"
            "Gazebo position: [0.35, 0.05, 0.48]\n"
            "\n"
            "Position errors:\n"
            "  X axis: 0.05 m (5 cm)\n"
            "  Y axis: 0.05 m (5 cm)\n"
            "  Z axis: 0.02 m (2 cm)\n"
            "\n"
            "Total position error: 0.074 m (7.4 cm)\n"
            "Tolerance: 0.05 m (5 cm)\n"
            "\n"
            "⚠️ Suggested actions:\n"
            "1. Stop current execution\n"
            "2. Restart Gazebo simulation\n"
            "3. Verify real robot position\n"
            "4. Re-sync Gazebo with real robot"
        ),
        sound=True
    )


def main():
    """主函数"""
    print("=" * 60)
    print("AUBO E5 弹窗报警系统演示")
    print("=" * 60)
    print("\n本演示将展示 5 种不同的弹窗类型")
    print("请依次关闭每个弹窗以继续下一个演示\n")

    input("按 Enter 键开始演示...")

    # 演示 1：信息弹窗
    demo_info()
    input("\n按 Enter 键继续下一个演示...")

    # 演示 2：警告弹窗
    demo_warning()
    input("\n按 Enter 键继续下一个演示...")

    # 演示 3：错误弹窗
    demo_error()
    input("\n按 Enter 键继续下一个演示...")

    # 演示 4：严重错误弹窗
    demo_critical()
    input("\n按 Enter 键继续下一个演示...")

    # 演示 5：Gazebo 位姿差异弹窗
    demo_pose_difference()

    print("\n" + "=" * 60)
    print("演示完成！")
    print("=" * 60)
    print("\n弹窗报警系统特性:")
    print("  ✅ 4 种报警级别（INFO, WARNING, ERROR, CRITICAL）")
    print("  ✅ 线程安全，不阻塞 ROS 节点")
    print("  ✅ 支持声音报警")
    print("  ✅ 支持详细信息展示")
    print("  ✅ 支持自动关闭")
    print("  ✅ 置顶显示，居中对齐")
    print("\n使用方法:")
    print("  from alert_dialog import AlertDialog")
    print("  AlertDialog.error(title='错误', message='消息', details='详情')")


if __name__ == '__main__':
    main()
