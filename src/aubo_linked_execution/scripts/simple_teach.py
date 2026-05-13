#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simple_teach.py - 简化示教脚本

功能：
1. 等待输入"1"记录当前位姿
2. 循环记录任意多个路径点
3. 输入"s"保存到 YAML 文件
4. 输入"q"退出

使用方法：
1. 启动联动系统：roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.10.230
2. 运行示教脚本：rosrun aubo_linked_execution simple_teach.py
3. 手动移动机器臂到目标位置，输入"1"记录
4. 输入"s"保存，输入"q"退出
"""

import sys
import os
import yaml
from datetime import datetime

# 添加路径以导入 motion_planning_interface
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'aubo_robot', 'aubo_planner', 'scripts'))

import rospy
from motion_planning_interface import MotionPlanningInterface


class SimpleTeach:
    def __init__(self):
        # 不在这里初始化节点，让 MotionPlanningInterface 来初始化
        print("正在初始化运动规划接口...")
        self.planner = MotionPlanningInterface(planner_name="ompl")
        self.move_group = self.planner.planner.move_group

        self.waypoints = []
        self.config_dir = os.path.expanduser("~/aubo_polish/waypoints_config")

        # 创建配置目录
        if not os.path.exists(self.config_dir):
            os.makedirs(self.config_dir)

        print("✓ 初始化完成\n")

    def record_waypoint(self):
        """记录当前位姿"""
        try:
            # 读取关节角度
            joints = self.move_group.get_current_joint_values()

            # 读取笛卡尔位姿
            pose_stamped = self.move_group.get_current_pose()
            pose = pose_stamped.pose

            waypoint = {
                'index': len(self.waypoints),
                'joints': list(joints),
                'cartesian': {
                    'position': [pose.position.x, pose.position.y, pose.position.z],
                    'orientation_quat': [pose.orientation.x, pose.orientation.y,
                                        pose.orientation.z, pose.orientation.w]
                }
            }

            self.waypoints.append(waypoint)
            print(f"✓ 已记录路径点 {len(self.waypoints)}")

            return True
        except Exception as e:
            print(f"✗ 记录失败: {e}")
            return False

    def save_config(self, name):
        """保存配置到 YAML"""
        if not self.waypoints:
            print("✗ 暂无路径点，无法保存")
            return False

        try:
            config = {
                'name': name,
                'created_at': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                'waypoints': self.waypoints
            }

            filepath = os.path.join(self.config_dir, f"{name}.yaml")
            with open(filepath, 'w') as f:
                yaml.dump(config, f, default_flow_style=False, allow_unicode=True)

            print(f"✓ 已保存 {len(self.waypoints)} 个路径点到 {name}.yaml")
            print(f"  文件位置: {filepath}")

            return True
        except Exception as e:
            print(f"✗ 保存失败: {e}")
            return False

    def run(self):
        """主循环"""
        print("=" * 50)
        print("  AUBO E5 示教系统")
        print("=" * 50)
        print()
        print("手动移动机器臂到目标位置，然后输入\"1\"记录")
        print()
        print("命令：")
        print("  1 - 记录当前位置")
        print("  s - 保存到文件")
        print("  q - 退出")
        print()

        try:
            while not rospy.is_shutdown():
                try:
                    cmd = input("> ").strip()

                    if not cmd:
                        continue

                    if cmd == "1":
                        self.record_waypoint()

                    elif cmd == "s":
                        name = input("文件名> ").strip()
                        if name:
                            self.save_config(name)
                        else:
                            print("✗ 文件名不能为空")

                    elif cmd == "q":
                        if self.waypoints and input("有未保存的路径点，确认退出？(y/n): ").lower() != 'y':
                            continue
                        print("再见！")
                        break

                    else:
                        print(f"✗ 未知命令: {cmd}")
                        print("  输入 1 记录，s 保存，q 退出")

                except KeyboardInterrupt:
                    print("\n使用 'q' 命令退出")
                except Exception as e:
                    print(f"✗ 错误: {e}")

        except KeyboardInterrupt:
            print("\n再见！")


def main():
    try:
        teacher = SimpleTeach()
        teacher.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"✗ 初始化失败: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
