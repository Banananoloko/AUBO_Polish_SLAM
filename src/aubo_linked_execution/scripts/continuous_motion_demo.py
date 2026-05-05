#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
连续运动演示脚本
支持三种运动模式：关节空间、笛卡尔空间、预定义位置
"""

import sys
import rospy
import yaml
import argparse
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String


class ContinuousMotionDemo:
    """连续运动演示类"""

    def __init__(self, config_file=None):
        """初始化"""
        # 初始化 moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化节点
        rospy.init_node('continuous_motion_demo', anonymous=True)

        # 创建 MoveGroupCommander
        self.group = moveit_commander.MoveGroupCommander("manipulator_e5")
        self.robot = moveit_commander.RobotCommander()

        # 设置规划参数
        self.group.set_planning_time(5.0)
        self.group.set_num_planning_attempts(10)

        # 加载配置文件
        self.config = None
        if config_file:
            self.load_config(config_file)

        rospy.loginfo("Continuous Motion Demo initialized")
        rospy.loginfo("Planning frame: %s", self.group.get_planning_frame())
        rospy.loginfo("End effector: %s", self.group.get_end_effector_link())

    def load_config(self, config_file):
        """加载配置文件"""
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
            rospy.loginfo("Loaded config from: %s", config_file)
        except Exception as e:
            rospy.logerr("Failed to load config: %s", str(e))
            self.config = None

    def execute_joint_waypoints(self, waypoints=None):
        """
        执行关节空间连续运动

        Args:
            waypoints: 关节值列表，每个元素是6个关节角度的列表
        """
        if waypoints is None:
            if self.config and 'joint_waypoints' in self.config:
                waypoints = self.config['joint_waypoints']
            else:
                rospy.logerr("No joint waypoints provided")
                return False

        rospy.loginfo("Executing %d joint waypoints", len(waypoints))

        for i, joint_values in enumerate(waypoints):
            rospy.loginfo("Moving to waypoint %d/%d: %s", i+1, len(waypoints), joint_values)

            # 设置目标关节值
            self.group.set_joint_value_target(joint_values)

            # 规划并执行
            success = self.group.go(wait=True)

            # 停止运动
            self.group.stop()

            if not success:
                rospy.logwarn("Failed to reach waypoint %d", i+1)
                return False

            # 短暂停留
            rospy.sleep(0.5)

        rospy.loginfo("Joint waypoints execution completed")
        return True

    def execute_cartesian_path(self, poses=None):
        """
        执行笛卡尔空间连续运动

        Args:
            poses: 位姿列表，每个元素是包含 position 和 orientation 的字典
        """
        if poses is None:
            if self.config and 'cartesian_waypoints' in self.config:
                poses = self.config['cartesian_waypoints']
            else:
                rospy.logerr("No cartesian waypoints provided")
                return False

        rospy.loginfo("Executing cartesian path with %d waypoints", len(poses))

        # 构建 waypoints 列表
        waypoints = []

        # 添加当前位姿作为起点
        waypoints.append(self.group.get_current_pose().pose)

        # 添加目标位姿
        for pose_dict in poses:
            pose = geometry_msgs.msg.Pose()

            # 设置位置
            if 'position' in pose_dict:
                pose.position.x = pose_dict['position'].get('x', 0.0)
                pose.position.y = pose_dict['position'].get('y', 0.0)
                pose.position.z = pose_dict['position'].get('z', 0.0)

            # 设置姿态（如果没有提供，使用默认值）
            if 'orientation' in pose_dict:
                pose.orientation.x = pose_dict['orientation'].get('x', 0.0)
                pose.orientation.y = pose_dict['orientation'].get('y', 0.0)
                pose.orientation.z = pose_dict['orientation'].get('z', 0.0)
                pose.orientation.w = pose_dict['orientation'].get('w', 1.0)
            else:
                pose.orientation.w = 1.0

            waypoints.append(pose)

        # 计算笛卡尔路径
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,
            0.01,  # eef_step: 1cm
            0.0    # jump_threshold: 禁用跳跃检测
        )

        rospy.loginfo("Cartesian path computed: %.2f%% complete", fraction * 100)

        # 如果路径完成度 > 95%，执行
        if fraction > 0.95:
            self.group.execute(plan, wait=True)
            rospy.loginfo("Cartesian path execution completed")
            return True
        else:
            rospy.logwarn("Cartesian path only %.2f%% complete, not executing", fraction * 100)
            return False

    def execute_named_targets(self, target_names=None):
        """
        执行预定义位置连续运动

        Args:
            target_names: 预定义位置名称列表（如 "home", "ready"）
        """
        if target_names is None:
            if self.config and 'named_targets' in self.config:
                target_names = self.config['named_targets']
            else:
                rospy.logerr("No named targets provided")
                return False

        rospy.loginfo("Executing %d named targets", len(target_names))

        for i, name in enumerate(target_names):
            rospy.loginfo("Moving to named target %d/%d: %s", i+1, len(target_names), name)

            # 设置命名目标
            self.group.set_named_target(name)

            # 规划并执行
            success = self.group.go(wait=True)

            # 停止运动
            self.group.stop()

            if not success:
                rospy.logwarn("Failed to reach named target: %s", name)
                return False

            # 短暂停留
            rospy.sleep(0.5)

        rospy.loginfo("Named targets execution completed")
        return True

    def shutdown(self):
        """关闭"""
        moveit_commander.roscpp_shutdown()


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='Continuous Motion Demo')
    parser.add_argument('--mode', type=str, default='joint',
                       choices=['joint', 'cartesian', 'named'],
                       help='Motion mode: joint, cartesian, or named')
    parser.add_argument('--config', type=str,
                       default='/home/wuqz/aubo_polish/src/aubo_linked_execution/config/motion_waypoints.yaml',
                       help='Path to config file')
    parser.add_argument('--loop', action='store_true',
                       help='Loop execution continuously')

    args = parser.parse_args()

    demo = None
    try:
        # 创建演示对象
        demo = ContinuousMotionDemo(config_file=args.config)

        # 循环执行
        loop_count = 0
        while not rospy.is_shutdown():
            loop_count += 1
            rospy.loginfo("=== Execution loop %d ===", loop_count)

            # 根据模式执行
            if args.mode == 'joint':
                success = demo.execute_joint_waypoints()
            elif args.mode == 'cartesian':
                success = demo.execute_cartesian_path()
            elif args.mode == 'named':
                success = demo.execute_named_targets()
            else:
                rospy.logerr("Unknown mode: %s", args.mode)
                break

            if not success:
                rospy.logwarn("Execution failed, stopping")
                break

            # 如果不是循环模式，执行一次后退出
            if not args.loop:
                break

            rospy.sleep(1.0)

        rospy.loginfo("Continuous Motion Demo finished")

    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted by user")
    except Exception as e:
        rospy.logerr("Error: %s", str(e))
        import traceback
        traceback.print_exc()
    finally:
        # 确保资源清理
        if demo is not None:
            demo.shutdown()


if __name__ == '__main__':
    main()
