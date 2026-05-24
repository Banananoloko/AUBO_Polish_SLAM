#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simple_playback.py - 简化回放脚本

功能：
1. 加载 YAML 配置文件
2. 顺序执行所有路径点
3. 到达最后一个后待机
4. Unity 自动同步跟踪

使用方法：
1. 启动联动系统：roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.10.230 use_unity:=true
2. 运行回放脚本：rosrun aubo_linked_execution simple_playback.py <config_file.yaml>
"""

import sys
import os
import yaml
import time
import logging
from datetime import datetime

# 添加路径以导入 motion_planning_interface
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'aubo_robot', 'aubo_planner', 'scripts'))

import rospy
from motion_planning_interface import MotionPlanningInterface


class SimplePlayback:
    def __init__(self, config_file, velocity=0.3, acceleration=0.3, debug=False):
        self.debug = debug

        # 先初始化 ROS（让 MotionPlanningInterface 完成初始化）
        print("正在初始化运动规划接口...")
        self.planner = MotionPlanningInterface(planner_name="ompl")
        self.move_group = self.planner.planner.move_group

        # 在 ROS 初始化之后配置日志系统
        self.setup_logging()
        self.log_info("初始化运动规划接口完成")

        # 设置速度和加速度限制（降低速度使运动更平滑）
        # 确保缩放因子不超过 0.3（留出更大安全裕度，避免驱动层速度超限）
        self.velocity_scaling = min(velocity, 0.3)
        self.acceleration_scaling = min(acceleration, 0.3)

        self.log_info(f"原始速度参数: velocity={velocity}, acceleration={acceleration}")
        self.log_info(f"安全限制后: velocity={self.velocity_scaling}, acceleration={self.acceleration_scaling}")

        self.move_group.set_max_velocity_scaling_factor(self.velocity_scaling)
        self.move_group.set_max_acceleration_scaling_factor(self.acceleration_scaling)

        # 平衡精度和可靠性的到达判定阈值
        # 0.02 rad ≈ 1.15°（比默认 0.01 宽松 2 倍，比之前 0.1 严格 5 倍）
        self.move_group.set_goal_joint_tolerance(0.02)
        self.move_group.set_goal_position_tolerance(0.01)  # 1cm
        self.move_group.set_goal_orientation_tolerance(0.02)  # 0.02 rad

        # 设置规划时间（增加到 15 秒，给规划器更多时间找到平滑轨迹）
        self.move_group.set_planning_time(15.0)

        # 设置执行超时（避免卡死）
        self.move_group.set_num_planning_attempts(5)  # 增加到 5 次规划尝试

        print(f"速度比例: {self.velocity_scaling}, 加速度比例: {self.acceleration_scaling}")
        print(f"到达阈值: 关节 0.02 rad (≈1.15°), 位置 0.01 m, 姿态 0.02 rad")
        print(f"规划尝试次数: 5, 规划时间: 15.0 秒")
        print(f"⚠ 注意: 速度已自动限制到 0.3 以避免驱动层速度超限")

        self.log_info(f"MoveIt 配置: 速度={self.velocity_scaling}, 加速度={self.acceleration_scaling}")
        self.log_info(f"到达阈值: 关节=0.02 rad, 位置=0.01 m, 姿态=0.02 rad")
        self.log_info(f"规划配置: 尝试次数=5, 规划时间=15.0 秒")

        # 加载配置
        print(f"正在加载配置文件: {config_file}")
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)

        self.waypoints = self.config['waypoints']

        self.log_info(f"加载了 {len(self.waypoints)} 个路径点")
        print("✓ 初始化完成\n")

    def setup_logging(self):
        """设置日志系统（在 ROS 初始化之后）"""
        # 创建日志目录
        log_dir = os.path.expanduser("~/aubo_polish/playback_logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)

        # 生成日志文件名（带时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(log_dir, f"playback_{timestamp}.log")

        # 获取根 logger 并清除所有现有处理器（清除 ROS 的处理器）
        root_logger = logging.getLogger()
        for handler in root_logger.handlers[:]:
            root_logger.removeHandler(handler)

        # 重新配置日志
        logging.basicConfig(
            level=logging.DEBUG if self.debug else logging.INFO,
            format='%(asctime)s [%(levelname)s] %(message)s',
            handlers=[
                logging.FileHandler(self.log_file),
                logging.StreamHandler() if self.debug else logging.NullHandler()
            ],
            force=True  # 强制重新配置
        )
        self.logger = logging.getLogger(__name__)

        print(f"日志文件: {self.log_file}")
        self.log_info("=" * 80)
        self.log_info("AUBO E5 回放系统 - Debug 模式")
        self.log_info("=" * 80)

    def log_info(self, msg):
        """记录信息日志"""
        self.logger.info(msg)

    def log_error(self, msg):
        """记录错误日志"""
        self.logger.error(msg)

    def log_debug(self, msg):
        """记录调试日志"""
        self.logger.debug(msg)

    def execute_waypoint(self, waypoint, index, total):
        """执行单个路径点"""
        print(f"[{index+1}/{total}] 移动到路径点 {index+1}...")
        self.log_info(f"=" * 60)
        self.log_info(f"路径点 {index+1}/{total}: {waypoint.get('name', f'point_{index}')}")
        self.log_info(f"目标关节角度: {waypoint['joints']}")

        start_time = time.time()

        try:
            # 记录当前位置
            current_joints = self.move_group.get_current_joint_values()
            self.log_debug(f"当前关节角度: {current_joints}")

            # 计算关节偏差
            joint_diff = [abs(t - c) for t, c in zip(waypoint['joints'], current_joints)]
            max_diff = max(joint_diff)
            self.log_info(f"最大关节偏差: {max_diff:.4f} rad ({max_diff * 57.3:.2f}°)")

            # 设置目标关节角度
            self.move_group.set_joint_value_target(waypoint['joints'])
            self.log_debug("已设置目标关节角度")

            # 使用 go() 方法：规划并执行（一步到位，避免卡顿）
            print("  规划并执行中...", end='', flush=True)
            self.log_info("开始规划并执行...")

            # 尝试执行，如果失败则重试（最多 2 次）
            max_retries = 2
            success = False

            for attempt in range(max_retries):
                self.log_info(f"尝试 {attempt + 1}/{max_retries}")
                attempt_start = time.time()

                result = self.move_group.go(wait=True)

                attempt_duration = time.time() - attempt_start
                self.log_info(f"尝试 {attempt + 1} 耗时: {attempt_duration:.2f} 秒")

                if result:
                    print("\r  ✓ 执行成功")
                    self.log_info("执行成功")
                    success = True
                    break
                else:
                    self.log_error(f"尝试 {attempt + 1} 失败")

                    # 记录失败时的状态
                    final_joints = self.move_group.get_current_joint_values()
                    final_diff = [abs(t - f) for t, f in zip(waypoint['joints'], final_joints)]
                    self.log_error(f"失败时关节偏差: {final_diff}")
                    self.log_error(f"最大偏差: {max(final_diff):.4f} rad ({max(final_diff) * 57.3:.2f}°)")

                    if attempt < max_retries - 1:
                        print(f"\r  ⚠ 执行失败，重试 {attempt + 1}/{max_retries}...", end='', flush=True)
                        self.log_info(f"等待 0.5 秒后重试...")
                        time.sleep(0.5)
                    else:
                        print("\r  ✗ 执行失败（已重试 2 次）")
                        self.log_error("所有重试均失败")

            if not success:
                total_duration = time.time() - start_time
                self.log_error(f"路径点 {index+1} 执行失败，总耗时: {total_duration:.2f} 秒")
                return False

            # 停止运动并清除目标
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            self.log_debug("已停止运动并清除目标")

            # 验证最终位置
            final_joints = self.move_group.get_current_joint_values()
            final_diff = [abs(t - f) for t, f in zip(waypoint['joints'], final_joints)]
            max_final_diff = max(final_diff)
            self.log_info(f"最终关节偏差: {max_final_diff:.4f} rad ({max_final_diff * 57.3:.2f}°)")

            if max_final_diff > 0.02:  # 超过阈值
                self.log_error(f"警告: 最终偏差 {max_final_diff:.4f} rad 超过阈值 0.02 rad")

            # 短暂停留，避免连续运动时的暴起
            time.sleep(0.2)

            total_duration = time.time() - start_time
            self.log_info(f"路径点 {index+1} 执行成功，总耗时: {total_duration:.2f} 秒")

            return True

        except Exception as e:
            print(f"\r  ✗ 错误: {e}")
            self.log_error(f"异常: {e}")
            import traceback
            tb = traceback.format_exc()
            self.log_error(f"堆栈跟踪:\n{tb}")
            return False

    def run(self):
        """主执行流程"""
        print("=" * 50)
        print("  AUBO E5 回放系统")
        print("=" * 50)
        print()
        print(f"配置文件: {self.config['name']}")
        print(f"路径点数量: {len(self.waypoints)}")
        print()

        self.log_info(f"开始执行回放，共 {len(self.waypoints)} 个路径点")
        start_time = time.time()

        success_count = 0
        failed_waypoints = []

        for i, waypoint in enumerate(self.waypoints):
            if self.execute_waypoint(waypoint, i, len(self.waypoints)):
                success_count += 1
                print()
            else:
                failed_waypoints.append(i + 1)
                self.log_error(f"路径点 {i+1} 执行失败")
                print(f"\n✗ 执行失败，停止在路径点 {i+1}")
                break

        total_duration = time.time() - start_time

        print("=" * 50)
        if success_count == len(self.waypoints):
            print(f"✓ 所有路径点执行完成 ({success_count}/{len(self.waypoints)})")
            self.log_info(f"所有路径点执行完成，总耗时: {total_duration:.2f} 秒")
        else:
            print(f"✗ 部分路径点执行失败 ({success_count}/{len(self.waypoints)})")
            self.log_error(f"执行失败，成功 {success_count}/{len(self.waypoints)} 个路径点")
            self.log_error(f"失败的路径点: {failed_waypoints}")

        print("待机中...")
        print("=" * 50)
        print(f"\n日志已保存到: {self.log_file}")

        self.log_info("=" * 80)
        self.log_info(f"回放结束，总耗时: {total_duration:.2f} 秒")
        self.log_info(f"成功: {success_count}/{len(self.waypoints)}")
        if failed_waypoints:
            self.log_info(f"失败的路径点: {failed_waypoints}")
        self.log_info("=" * 80)

        return success_count == len(self.waypoints)


def main():
    if len(sys.argv) < 2:
        print("用法: simple_playback.py <config_file.yaml> [velocity] [acceleration] [--debug]")
        print()
        print("参数:")
        print("  config_file.yaml  - 路径点配置文件")
        print("  velocity          - 速度比例 (0.1-1.0, 默认 0.3)")
        print("  acceleration      - 加速度比例 (0.1-1.0, 默认 0.3)")
        print("  --debug           - 启用 debug 模式（详细日志）")
        print()
        print("示例:")
        print("  rosrun aubo_linked_execution simple_playback.py ~/aubo_polish/waypoints_config/my_task.yaml")
        print("  rosrun aubo_linked_execution simple_playback.py ~/aubo_polish/waypoints_config/my_task.yaml 0.5 0.5")
        print("  rosrun aubo_linked_execution simple_playback.py ~/aubo_polish/waypoints_config/my_task.yaml 0.5 0.5 --debug")
        sys.exit(1)

    config_file = sys.argv[1]

    # 检查是否启用 debug 模式
    debug = '--debug' in sys.argv

    # 解析速度和加速度参数（排除 --debug）
    args = [arg for arg in sys.argv[2:] if arg != '--debug']
    velocity = float(args[0]) if len(args) > 0 else 0.3
    acceleration = float(args[1]) if len(args) > 1 else 0.3

    # 限制范围
    velocity = max(0.1, min(1.0, velocity))
    acceleration = max(0.1, min(1.0, acceleration))

    # 支持相对路径
    if not os.path.isabs(config_file):
        config_file = os.path.abspath(config_file)

    if not os.path.exists(config_file):
        print(f"✗ 配置文件不存在: {config_file}")
        sys.exit(1)

    try:
        playback = SimplePlayback(config_file, velocity, acceleration, debug)
        playback.run()

        # 保持节点运行（待机）
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"✗ 错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
