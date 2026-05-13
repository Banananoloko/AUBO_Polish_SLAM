#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
playback_waypoints.py - 路径点回放脚本
功能：加载YAML配置、多种执行模式、监控与恢复、安全机制
"""
import sys, os, math, yaml, time, argparse
from datetime import datetime
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'aubo_robot', 'aubo_planner', 'scripts'))
import rospy
from motion_planning_interface import MotionPlanningInterface
from goals.joint_goal import JointGoal

class Colors:
    RESET,BOLD,RED,GREEN,YELLOW,BLUE,CYAN,GRAY,MAGENTA='\033[0m','\033[1m','\033[91m','\033[92m','\033[93m','\033[94m','\033[96m','\033[90m','\033[95m'

class PlaybackWaypoints:
    def __init__(self, config_file, args):
        rospy.init_node('playback_waypoints', anonymous=True)
        print(f"{Colors.CYAN}正在初始化运动规划接口...{Colors.RESET}")
        self.planner = MotionPlanningInterface(planner_name=args.planner)
        self.move_group = self.planner.planner.move_group
        self.config_file = config_file
        self.config_data = None
        self.waypoints = []
        self.args = args
        self.max_retries = 3
        self.timeout = 30.0
        self.stats = {'total':0,'success':0,'failed':0,'retries':0,'start_time':None,'end_time':None}
        self.load_config()
        self.apply_execution_params()
        print(f"{Colors.GREEN}✓ 初始化完成{Colors.RESET}\n")

    def load_config(self):
        """加载配置文件"""
        if not os.path.exists(self.config_file):
            raise FileNotFoundError(f"配置文件不存在: {self.config_file}")
        with open(self.config_file, 'r') as f:
            self.config_data = yaml.safe_load(f)
        self.waypoints = self.config_data.get('waypoints', [])
        if not self.waypoints:
            raise ValueError("配置文件中没有路径点")
        print(f"{Colors.GREEN}✓ 已加载配置: {self.config_data.get('name', 'Unknown')}{Colors.RESET}")
        print(f"  路径点数量: {len(self.waypoints)}")
        print(f"  创建时间: {self.config_data.get('created_at', 'Unknown')}\n")

    def apply_execution_params(self):
        """应用执行参数"""
        params = self.config_data.get('execution_params', {})
        velocity = self.args.velocity if self.args.velocity else params.get('velocity_scaling', 0.5)
        self.move_group.set_max_velocity_scaling_factor(velocity)
        self.move_group.set_max_acceleration_scaling_factor(params.get('acceleration_scaling', 0.5))
        self.move_group.set_goal_tolerance(self.args.tolerance if self.args.tolerance else params.get('goal_tolerance', 0.01))
        self.move_group.set_planning_time(params.get('planning_time', 5.0))

    def preview_waypoints(self):
        """显示路径点预览表格"""
        print(f"\n{Colors.CYAN}{Colors.BOLD}{'='*90}{Colors.RESET}")
        print(f"{Colors.CYAN}{Colors.BOLD}  路径点预览{Colors.RESET}")
        print(f"{Colors.CYAN}{Colors.BOLD}{'='*90}{Colors.RESET}\n")
        print(f"{Colors.GRAY}{'序号':<6} {'名称':<20} {'位置 (x, y, z)':<35} {'时间':<20}{Colors.RESET}")
        print(f"{Colors.GRAY}{'-'*90}{Colors.RESET}")
        for i, wp in enumerate(self.waypoints, 1):
            pos = wp['cartesian']['position']
            print(f"{i:<6} {wp['name']:<20} ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f})  {wp['timestamp']}")
        print()

    def execute_waypoint(self, waypoint, index, total):
        """执行单个路径点"""
        print(f"{Colors.CYAN}[{index}/{total}] 移动到: {waypoint['name']}{Colors.RESET}")
        start_time = time.time()
        for attempt in range(self.max_retries):
            try:
                goal = JointGoal(waypoint['joints'])
                trajectory = self.planner.planner.plan(goal)
                if trajectory and self.planner.validator.validate(trajectory):
                    success = self.move_group.execute(trajectory, wait=True)
                    self.move_group.stop()
                    if success:
                        elapsed = time.time() - start_time
                        print(f"{Colors.GREEN}✓ 完成 (耗时: {elapsed:.2f}s){Colors.RESET}")
                        if self.args.pause > 0:
                            time.sleep(self.args.pause)
                        return True
                if attempt < self.max_retries - 1:
                    self.stats['retries'] += 1
                    print(f"{Colors.YELLOW}⚠ 重试 {attempt+1}/{self.max_retries-1}{Colors.RESET}")
                    time.sleep(1.0)
            except Exception as e:
                print(f"{Colors.RED}✗ 错误: {e}{Colors.RESET}")
                if attempt < self.max_retries - 1:
                    time.sleep(1.0)
        return False

    def run_single(self):
        """单次执行模式"""
        self.preview_waypoints()
        self.stats['start_time'] = time.time()
        self.stats['total'] = len(self.waypoints)
        for i, wp in enumerate(self.waypoints, 1):
            if self.execute_waypoint(wp, i, len(self.waypoints)):
                self.stats['success'] += 1
            else:
                self.stats['failed'] += 1
                print(f"{Colors.RED}✗ 执行失败，停止{Colors.RESET}")
                break
        self.stats['end_time'] = time.time()
        self.print_stats()

    def run_loop(self):
        """循环执行模式"""
        self.preview_waypoints()
        loop_count = self.args.loop_count if self.args.loop_count else float('inf')
        current_loop = 0
        while current_loop < loop_count and not rospy.is_shutdown():
            current_loop += 1
            print(f"\n{Colors.MAGENTA}=== 循环 {current_loop}/{loop_count if loop_count != float('inf') else '∞'} ==={Colors.RESET}\n")
            self.stats['start_time'] = time.time()
            self.stats['total'] = len(self.waypoints)
            for i, wp in enumerate(self.waypoints, 1):
                if self.execute_waypoint(wp, i, len(self.waypoints)):
                    self.stats['success'] += 1
                else:
                    self.stats['failed'] += 1
                    print(f"{Colors.RED}✗ 执行失败，停止循环{Colors.RESET}")
                    return
            self.stats['end_time'] = time.time()
        self.print_stats()

    def run_step(self):
        """单步执行模式"""
        self.preview_waypoints()
        self.stats['start_time'] = time.time()
        self.stats['total'] = len(self.waypoints)
        for i, wp in enumerate(self.waypoints, 1):
            input(f"{Colors.YELLOW}按 Enter 继续到路径点 {i}/{len(self.waypoints)}...{Colors.RESET}")
            if self.execute_waypoint(wp, i, len(self.waypoints)):
                self.stats['success'] += 1
            else:
                self.stats['failed'] += 1
        self.stats['end_time'] = time.time()
        self.print_stats()

    def run_reverse(self):
        """反向执行模式"""
        self.waypoints = list(reversed(self.waypoints))
        self.run_single()

    def print_stats(self):
        """打印统计信息"""
        elapsed = self.stats['end_time'] - self.stats['start_time']
        print(f"\n{Colors.CYAN}{Colors.BOLD}{'='*60}{Colors.RESET}")
        print(f"{Colors.CYAN}{Colors.BOLD}  执行统计{Colors.RESET}")
        print(f"{Colors.CYAN}{Colors.BOLD}{'='*60}{Colors.RESET}")
        print(f"  总路径点: {self.stats['total']}")
        print(f"  {Colors.GREEN}成功: {self.stats['success']}{Colors.RESET}")
        print(f"  {Colors.RED}失败: {self.stats['failed']}{Colors.RESET}")
        print(f"  {Colors.YELLOW}重试次数: {self.stats['retries']}{Colors.RESET}")
        print(f"  总耗时: {elapsed:.2f}s")
        print(f"  平均耗时: {elapsed/self.stats['total']:.2f}s/点\n")

def main():
    parser = argparse.ArgumentParser(description='路径点回放脚本')
    parser.add_argument('--config', type=str, required=True, help='配置文件名（不含.yaml后缀）')
    parser.add_argument('--mode', type=str, default='single', choices=['single','loop','step','reverse'], help='执行模式')
    parser.add_argument('--velocity', type=float, help='速度比例 (0.1-1.0)')
    parser.add_argument('--planner', type=str, default='RRTConnect', help='规划器名称')
    parser.add_argument('--tolerance', type=float, help='目标容差 (rad)')
    parser.add_argument('--pause', type=float, default=0.5, help='路径点间停留时间 (s)')
    parser.add_argument('--loop-count', type=int, help='循环次数（loop模式）')
    args = parser.parse_args()
    config_dir = os.path.expanduser("~/aubo_polish/waypoints_config")
    config_file = os.path.join(config_dir, f"{args.config}.yaml")
    try:
        playback = PlaybackWaypoints(config_file, args)
        if args.mode == 'single':
            playback.run_single()
        elif args.mode == 'loop':
            playback.run_loop()
        elif args.mode == 'step':
            playback.run_step()
        elif args.mode == 'reverse':
            playback.run_reverse()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"{Colors.RED}✗ 错误: {e}{Colors.RESET}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
