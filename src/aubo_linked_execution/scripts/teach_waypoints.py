#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teach_waypoints.py - 交互式路径点示教脚本

功能：
1. 实时显示当前机器臂位姿（关节角度 + 笛卡尔坐标）
2. 交互式记录路径点
3. 路径点管理（列表、删除、预览、撤销）
4. 保存/加载 YAML 配置文件

使用方法：
1. 启动联动系统：roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.10.230
2. 运行示教脚本：rosrun aubo_linked_execution teach_waypoints.py
3. 手动移动机器臂到目标位置，输入 'r' 记录
4. 输入 's <name>' 保存到文件
"""

import sys
import os
import math
import yaml
from datetime import datetime

# 添加路径以导入 motion_planning_interface
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'aubo_robot', 'aubo_planner', 'scripts'))

import rospy
from motion_planning_interface import MotionPlanningInterface
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import JointState

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
    MAGENTA = '\033[95m'


class TeachWaypoints:
    def __init__(self):
        rospy.init_node('teach_waypoints', anonymous=True)

        # 初始化运动规划接口
        print(f"{Colors.CYAN}正在初始化运动规划接口...{Colors.RESET}")
        self.planner = MotionPlanningInterface(planner_name="ompl")
        self.move_group = self.planner.planner.move_group

        # 路径点列表
        self.waypoints = []
        self.waypoint_history = []  # 用于撤销

        # 配置目录
        self.config_dir = os.path.expanduser("~/aubo_polish/waypoints_config")
        if not os.path.exists(self.config_dir):
            os.makedirs(self.config_dir)

        # 工作空间限制
        self.max_reach = 0.784  # 最大臂展 (m)
        self.joint_limits = 3.05  # 关节限位 (rad, ±175°)

        print(f"{Colors.GREEN}✓ 初始化完成{Colors.RESET}\n")

    def get_current_state(self):
        """获取当前机器臂状态"""
        # 关节角度
        joints = self.move_group.get_current_joint_values()

        # 笛卡尔位姿
        pose_stamped = self.move_group.get_current_pose()
        pose = pose_stamped.pose

        # 转换为 RPY
        q = pose.orientation
        rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])

        return {
            'joints': joints,
            'position': [pose.position.x, pose.position.y, pose.position.z],
            'orientation_quat': [q.x, q.y, q.z, q.w],
            'orientation_rpy': list(rpy)
        }

    def display_current_state(self):
        """显示当前状态"""
        state = self.get_current_state()

        print(f"\n{Colors.CYAN}{Colors.BOLD}{'='*70}{Colors.RESET}")
        print(f"{Colors.CYAN}{Colors.BOLD}  当前机器臂状态{Colors.RESET}")
        print(f"{Colors.CYAN}{Colors.BOLD}{'='*70}{Colors.RESET}\n")

        # 关节角度
        print(f"{Colors.YELLOW}关节角度：{Colors.RESET}")
        for i, (j_rad, j_deg) in enumerate(zip(state['joints'],
                                                [math.degrees(j) for j in state['joints']]), 1):
            # 进度条
            progress = int((j_rad + self.joint_limits) / (2 * self.joint_limits) * 20)
            bar = '█' * progress + '░' * (20 - progress)

            # 颜色（接近限位时变红）
            if abs(j_rad) > self.joint_limits * 0.9:
                color = Colors.RED
            elif abs(j_rad) > self.joint_limits * 0.7:
                color = Colors.YELLOW
            else:
                color = Colors.GREEN

            print(f"  关节 {i}: {color}{j_deg:7.2f}°{Colors.RESET} "
                  f"({j_rad:6.3f} rad) [{bar}]")

        # 笛卡尔位置
        print(f"\n{Colors.YELLOW}笛卡尔位置：{Colors.RESET}")
        x, y, z = state['position']
        reach = math.sqrt(x**2 + y**2 + z**2)

        # 工作空间警告
        if reach > self.max_reach:
            reach_color = Colors.RED
            reach_warning = " ⚠ 超出工作空间"
        elif reach > self.max_reach * 0.9:
            reach_color = Colors.YELLOW
            reach_warning = " ⚠ 接近边界"
        else:
            reach_color = Colors.GREEN
            reach_warning = ""

        print(f"  X: {x:7.4f} m")
        print(f"  Y: {y:7.4f} m")
        print(f"  Z: {z:7.4f} m")
        print(f"  臂展: {reach_color}{reach:.4f} m{Colors.RESET}{reach_warning}")

        # 姿态
        print(f"\n{Colors.YELLOW}姿态 (RPY)：{Colors.RESET}")
        roll, pitch, yaw = state['orientation_rpy']
        print(f"  Roll:  {math.degrees(roll):7.2f}° ({roll:6.3f} rad)")
        print(f"  Pitch: {math.degrees(pitch):7.2f}° ({pitch:6.3f} rad)")
        print(f"  Yaw:   {math.degrees(yaw):7.2f}° ({yaw:6.3f} rad)")

        print(f"\n{Colors.CYAN}{Colors.BOLD}{'='*70}{Colors.RESET}\n")

    def validate_waypoint(self, state):
        """验证路径点是否有效"""
        # 检查关节限位
        for i, j in enumerate(state['joints'], 1):
            if abs(j) > self.joint_limits:
                print(f"{Colors.RED}✗ 关节 {i} 超出限位: {math.degrees(j):.2f}° "
                      f"(限制: ±{math.degrees(self.joint_limits):.2f}°){Colors.RESET}")
                return False

        # 检查工作空间
        x, y, z = state['position']
        reach = math.sqrt(x**2 + y**2 + z**2)
        if reach > self.max_reach:
            print(f"{Colors.RED}✗ 超出工作空间: {reach:.4f} m (最大: {self.max_reach} m){Colors.RESET}")
            return False

        # 检查重复路径点
        for i, wp in enumerate(self.waypoints):
            dist = math.sqrt(sum((a - b)**2 for a, b in zip(state['position'], wp['cartesian']['position'])))
            if dist < 0.01:  # 1cm 阈值
                print(f"{Colors.YELLOW}⚠ 警告: 与路径点 {i+1} 非常接近 (距离: {dist*1000:.1f} mm){Colors.RESET}")

        return True

    def record_waypoint(self, name=None):
        """记录当前路径点"""
        state = self.get_current_state()

        # 验证
        if not self.validate_waypoint(state):
            return False

        # 生成名称
        if not name:
            name = f"point_{len(self.waypoints) + 1}"

        waypoint = {
            'name': name,
            'index': len(self.waypoints),
            'joints': state['joints'],
            'cartesian': {
                'position': state['position'],
                'orientation_rpy': state['orientation_rpy'],
                'orientation_quat': state['orientation_quat']
            },
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }

        # 保存到历史（用于撤销）
        self.waypoint_history.append(list(self.waypoints))

        self.waypoints.append(waypoint)

        print(f"{Colors.GREEN}✓ 已记录路径点 {len(self.waypoints)}: {name}{Colors.RESET}")
        print(f"  位置: ({state['position'][0]:.3f}, {state['position'][1]:.3f}, {state['position'][2]:.3f})")
        print(f"  时间: {waypoint['timestamp']}\n")

        return True

    def list_waypoints(self):
        """列出所有路径点"""
        if not self.waypoints:
            print(f"{Colors.YELLOW}暂无路径点{Colors.RESET}\n")
            return

        print(f"\n{Colors.CYAN}{Colors.BOLD}已记录的路径点 ({len(self.waypoints)} 个)：{Colors.RESET}\n")
        print(f"{Colors.GRAY}{'序号':<6} {'名称':<20} {'位置 (x, y, z)':<30} {'时间':<20}{Colors.RESET}")
        print(f"{Colors.GRAY}{'-'*80}{Colors.RESET}")

        for i, wp in enumerate(self.waypoints, 1):
            pos = wp['cartesian']['position']
            print(f"{i:<6} {wp['name']:<20} "
                  f"({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f})  "
                  f"{wp['timestamp']}")

        print()

    def delete_waypoint(self, index):
        """删除路径点"""
        if index < 1 or index > len(self.waypoints):
            print(f"{Colors.RED}✗ 无效的序号: {index}{Colors.RESET}\n")
            return False

        # 保存到历史
        self.waypoint_history.append(list(self.waypoints))

        wp = self.waypoints.pop(index - 1)

        # 更新索引
        for i, w in enumerate(self.waypoints):
            w['index'] = i

        print(f"{Colors.GREEN}✓ 已删除路径点 {index}: {wp['name']}{Colors.RESET}\n")
        return True

    def undo(self):
        """撤销上一次操作"""
        if not self.waypoint_history:
            print(f"{Colors.YELLOW}无可撤销的操作{Colors.RESET}\n")
            return False

        self.waypoints = self.waypoint_history.pop()
        print(f"{Colors.GREEN}✓ 已撤销上一次操作{Colors.RESET}\n")
        return True

    def clear_waypoints(self):
        """清空所有路径点"""
        if not self.waypoints:
            print(f"{Colors.YELLOW}暂无路径点{Colors.RESET}\n")
            return

        # 保存到历史
        self.waypoint_history.append(list(self.waypoints))

        count = len(self.waypoints)
        self.waypoints = []
        print(f"{Colors.GREEN}✓ 已清空 {count} 个路径点{Colors.RESET}\n")

    def save_config(self, name):
        """保存配置到 YAML 文件"""
        if not self.waypoints:
            print(f"{Colors.RED}✗ 暂无路径点，无法保存{Colors.RESET}\n")
            return False

        config_file = os.path.join(self.config_dir, f"{name}.yaml")

        config_data = {
            'name': name,
            'description': f"Waypoint sequence: {name}",
            'created_at': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'waypoints': self.waypoints,
            'execution_params': {
                'planner': 'RRTConnect',
                'velocity_scaling': 0.5,
                'acceleration_scaling': 0.5,
                'goal_tolerance': 0.01,
                'planning_time': 5.0
            }
        }

        with open(config_file, 'w') as f:
            yaml.dump(config_data, f, default_flow_style=False, allow_unicode=True)

        print(f"{Colors.GREEN}✓ 已保存配置到: {config_file}{Colors.RESET}")
        print(f"  路径点数量: {len(self.waypoints)}\n")

        return True

    def load_config(self, name):
        """加载配置文件"""
        config_file = os.path.join(self.config_dir, f"{name}.yaml")

        if not os.path.exists(config_file):
            print(f"{Colors.RED}✗ 配置文件不存在: {config_file}{Colors.RESET}\n")
            return False

        with open(config_file, 'r') as f:
            config_data = yaml.safe_load(f)

        # 保存到历史
        if self.waypoints:
            self.waypoint_history.append(list(self.waypoints))

        self.waypoints = config_data['waypoints']

        print(f"{Colors.GREEN}✓ 已加载配置: {name}{Colors.RESET}")
        print(f"  路径点数量: {len(self.waypoints)}")
        print(f"  创建时间: {config_data.get('created_at', 'Unknown')}\n")

        return True

    def list_configs(self):
        """列出所有配置文件"""
        configs = [f[:-5] for f in os.listdir(self.config_dir) if f.endswith('.yaml')]

        if not configs:
            print(f"{Colors.YELLOW}暂无保存的配置{Colors.RESET}\n")
            return

        print(f"\n{Colors.CYAN}可用的配置文件：{Colors.RESET}")
        for cfg in sorted(configs):
            print(f"  • {cfg}")
        print()

    def show_stats(self):
        """显示统计信息"""
        if not self.waypoints:
            print(f"{Colors.YELLOW}暂无路径点{Colors.RESET}\n")
            return

        print(f"\n{Colors.CYAN}{Colors.BOLD}统计信息：{Colors.RESET}")
        print(f"  路径点数量: {len(self.waypoints)}")

        # 计算总路径长度
        total_dist = 0
        for i in range(len(self.waypoints) - 1):
            p1 = self.waypoints[i]['cartesian']['position']
            p2 = self.waypoints[i+1]['cartesian']['position']
            dist = math.sqrt(sum((a - b)**2 for a, b in zip(p1, p2)))
            total_dist += dist

        print(f"  总路径长度: {total_dist:.3f} m")

        # 工作空间范围
        if self.waypoints:
            positions = [wp['cartesian']['position'] for wp in self.waypoints]
            x_vals = [p[0] for p in positions]
            y_vals = [p[1] for p in positions]
            z_vals = [p[2] for p in positions]

            print(f"  工作空间范围:")
            print(f"    X: [{min(x_vals):.3f}, {max(x_vals):.3f}] m")
            print(f"    Y: [{min(y_vals):.3f}, {max(y_vals):.3f}] m")
            print(f"    Z: [{min(z_vals):.3f}, {max(z_vals):.3f}] m")

        print()

    def show_help(self):
        """显示帮助信息"""
        print(f"\n{Colors.CYAN}{Colors.BOLD}可用命令：{Colors.RESET}\n")
        print(f"  {Colors.GREEN}record / r [name]{Colors.RESET}        记录当前位置")
        print(f"  {Colors.GREEN}list / l{Colors.RESET}                  列出已记录的路径点")
        print(f"  {Colors.GREEN}delete / d <index>{Colors.RESET}       删除指定路径点")
        print(f"  {Colors.GREEN}clear{Colors.RESET}                     清空所有路径点")
        print(f"  {Colors.GREEN}undo / u{Colors.RESET}                  撤销上一次操作")
        print(f"  {Colors.GREEN}save / s <name>{Colors.RESET}          保存到文件")
        print(f"  {Colors.GREEN}load <name>{Colors.RESET}              加载配置文件")
        print(f"  {Colors.GREEN}configs{Colors.RESET}                   列出所有配置文件")
        print(f"  {Colors.GREEN}stats{Colors.RESET}                     显示统计信息")
        print(f"  {Colors.GREEN}status{Colors.RESET}                    显示当前状态")
        print(f"  {Colors.GREEN}help / h / ?{Colors.RESET}              显示帮助")
        print(f"  {Colors.GREEN}quit / q / exit{Colors.RESET}          退出\n")

    def run(self):
        """主运行循环"""
        print(f"{Colors.CYAN}{Colors.BOLD}{'='*70}{Colors.RESET}")
        print(f"{Colors.CYAN}{Colors.BOLD}  路径点示教系统{Colors.RESET}")
        print(f"{Colors.CYAN}{Colors.BOLD}{'='*70}{Colors.RESET}\n")

        print(f"{Colors.YELLOW}提示：{Colors.RESET}")
        print(f"  1. 手动移动机器臂到目标位置")
        print(f"  2. 输入 'r' 记录当前位置")
        print(f"  3. 输入 's <name>' 保存到文件")
        print(f"  4. 输入 'help' 查看所有命令\n")

        # 显示当前状态
        self.display_current_state()

        try:
            while not rospy.is_shutdown():
                try:
                    cmd = input(f"{Colors.CYAN}teach>{Colors.RESET} ").strip()

                    if not cmd:
                        continue

                    parts = cmd.split()
                    command = parts[0].lower()
                    args = parts[1:] if len(parts) > 1 else []

                    # 记录路径点
                    if command in ['record', 'r']:
                        name = args[0] if args else None
                        self.record_waypoint(name)

                    # 列出路径点
                    elif command in ['list', 'l']:
                        self.list_waypoints()

                    # 删除路径点
                    elif command in ['delete', 'd']:
                        if not args:
                            print(f"{Colors.RED}✗ 请指定路径点序号{Colors.RESET}\n")
                        else:
                            try:
                                index = int(args[0])
                                self.delete_waypoint(index)
                            except ValueError:
                                print(f"{Colors.RED}✗ 无效的序号{Colors.RESET}\n")

                    # 清空路径点
                    elif command == 'clear':
                        response = input(f"{Colors.YELLOW}确认清空所有路径点？(y/n): {Colors.RESET}")
                        if response.lower() == 'y':
                            self.clear_waypoints()

                    # 撤销
                    elif command in ['undo', 'u']:
                        self.undo()

                    # 保存配置
                    elif command in ['save', 's']:
                        if not args:
                            print(f"{Colors.RED}✗ 请指定配置名称{Colors.RESET}\n")
                        else:
                            self.save_config(args[0])

                    # 加载配置
                    elif command == 'load':
                        if not args:
                            print(f"{Colors.RED}✗ 请指定配置名称{Colors.RESET}\n")
                        else:
                            self.load_config(args[0])

                    # 列出配置
                    elif command == 'configs':
                        self.list_configs()

                    # 统计信息
                    elif command == 'stats':
                        self.show_stats()

                    # 显示当前状态
                    elif command == 'status':
                        self.display_current_state()

                    # 帮助
                    elif command in ['help', 'h', '?']:
                        self.show_help()

                    # 退出
                    elif command in ['quit', 'q', 'exit']:
                        if self.waypoints and not self.waypoint_history:
                            response = input(f"{Colors.YELLOW}有未保存的路径点，确认退出？(y/n): {Colors.RESET}")
                            if response.lower() != 'y':
                                continue
                        print(f"{Colors.GREEN}再见！{Colors.RESET}")
                        break

                    else:
                        print(f"{Colors.RED}✗ 未知命令: {command}{Colors.RESET}")
                        print(f"  输入 'help' 查看可用命令\n")

                except KeyboardInterrupt:
                    print(f"\n{Colors.YELLOW}使用 'quit' 命令退出{Colors.RESET}\n")
                except Exception as e:
                    print(f"{Colors.RED}✗ 错误: {e}{Colors.RESET}\n")

        except KeyboardInterrupt:
            print(f"\n{Colors.GREEN}再见！{Colors.RESET}")


def main():
    try:
        teacher = TeachWaypoints()
        teacher.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"{Colors.RED}✗ 初始化失败: {e}{Colors.RESET}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
