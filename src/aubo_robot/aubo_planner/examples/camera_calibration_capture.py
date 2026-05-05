#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_calibration_capture.py - 相机标定自动化采集脚本

功能：
1. 读取当前示教位置
2. 基于该位置生成 15 个标定位置（不同角度和距离）
3. 自动移动到每个位置并触发相机拍照
4. 保存所有位置信息到配置文件

使用方法：
1. 手动示教机器臂到初始位置（相机正对物体，距离 500mm）
2. 运行脚本：rosrun aubo_planner camera_calibration_capture.py
3. 脚本会自动执行 15 个位置的采集
"""

import sys
import os
import math
import time
import yaml
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

import rospy
from motion_planning_interface import MotionPlanningInterface
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose
from std_msgs.msg import String

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


class CameraCalibrationCapture:
    def __init__(self):
        rospy.init_node('camera_calibration_capture', anonymous=True)

        # 初始化运动规划接口
        self.planner = MotionPlanningInterface(planner_name="ompl")
        self.move_group = self.planner.planner.move_group

        # 相机触发发布器（可选）
        self.camera_trigger_pub = rospy.Publisher('/camera/trigger', String, queue_size=10)

        # 标定位置列表
        self.calibration_poses = []

        # 当前位置（示教位置）
        self.base_pose = None
        self.base_joints = None

        print(f"{Colors.CYAN}{Colors.BOLD}{'='*70}{Colors.RESET}")
        print(f"{Colors.CYAN}{Colors.BOLD}  相机标定自动化采集系统{Colors.RESET}")
        print(f"{Colors.CYAN}{Colors.BOLD}{'='*70}{Colors.RESET}\n")

    def read_current_pose(self):
        """读取当前示教位置"""
        print(f"{Colors.YELLOW}正在读取当前示教位置...{Colors.RESET}")

        # 读取关节角度
        self.base_joints = self.move_group.get_current_joint_values()

        # 读取笛卡尔位姿
        pose_stamped = self.move_group.get_current_pose()
        self.base_pose = pose_stamped.pose

        # 转换为 RPY
        q = self.base_pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        print(f"{Colors.GREEN}✓ 当前位置已读取：{Colors.RESET}")
        print(f"  位置: x={self.base_pose.position.x:.4f}, "
              f"y={self.base_pose.position.y:.4f}, "
              f"z={self.base_pose.position.z:.4f}")
        print(f"  姿态: roll={math.degrees(roll):.2f}°, "
              f"pitch={math.degrees(pitch):.2f}°, "
              f"yaw={math.degrees(yaw):.2f}°")
        print(f"  关节: {[f'{math.degrees(j):.2f}°' for j in self.base_joints]}\n")

        return True

    def generate_calibration_poses(self):
        """生成 15 个标定位置"""
        print(f"{Colors.YELLOW}正在生成标定位置...{Colors.RESET}\n")

        # 获取基准位置的 RPY
        q = self.base_pose.orientation
        base_roll, base_pitch, base_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # 定义标定位置（相对于基准位置的偏移）
        calibration_configs = [
            # (名称, x偏移, y偏移, z偏移, roll偏移, pitch偏移, yaw偏移)
            ("calib_0001_center", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),  # 正对中心
            ("calib_0002_left", 0.0, 0.05, 0.0, 0.0, 0.0, 0.0),   # 偏左 5cm
            ("calib_0003_right", 0.0, -0.05, 0.0, 0.0, 0.0, 0.0), # 偏右 5cm
            ("calib_0004_x_plus_15", 0.0, 0.0, 0.0, math.radians(15), 0.0, 0.0),   # X +15°
            ("calib_0005_x_minus_15", 0.0, 0.0, 0.0, math.radians(-15), 0.0, 0.0), # X -15°
            ("calib_0006_x_plus_25", 0.0, 0.0, 0.0, math.radians(25), 0.0, 0.0),   # X +25°
            ("calib_0007_x_minus_25", 0.0, 0.0, 0.0, math.radians(-25), 0.0, 0.0), # X -25°
            ("calib_0008_y_plus_15", 0.0, 0.0, 0.0, 0.0, math.radians(15), 0.0),   # Y +15°
            ("calib_0009_y_minus_15", 0.0, 0.0, 0.0, 0.0, math.radians(-15), 0.0), # Y -15°
            ("calib_0010_y_plus_25", 0.0, 0.0, 0.0, 0.0, math.radians(25), 0.0),   # Y +25°
            ("calib_0011_y_minus_25", 0.0, 0.0, 0.0, 0.0, math.radians(-25), 0.0), # Y -25°
            ("calib_0012_z_plus_20", 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(20)),   # Z +20°
            ("calib_0013_z_minus_20", 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-20)), # Z -20°
            ("calib_0014_near", 0.07, 0.0, 0.0, 0.0, 0.0, 0.0),   # 近一点 (500->430mm, 约 7cm)
            ("calib_0015_far", -0.08, 0.0, 0.0, 0.0, 0.0, 0.0),  # 远一点 (500->580mm, 约 8cm)
        ]

        for name, dx, dy, dz, droll, dpitch, dyaw in calibration_configs:
            # 计算新位置
            new_pose = Pose()
            new_pose.position.x = self.base_pose.position.x + dx
            new_pose.position.y = self.base_pose.position.y + dy
            new_pose.position.z = self.base_pose.position.z + dz

            # 计算新姿态
            new_roll = base_roll + droll
            new_pitch = base_pitch + dpitch
            new_yaw = base_yaw + dyaw

            # 转换为四元数
            q_new = quaternion_from_euler(new_roll, new_pitch, new_yaw)
            new_pose.orientation.x = q_new[0]
            new_pose.orientation.y = q_new[1]
            new_pose.orientation.z = q_new[2]
            new_pose.orientation.w = q_new[3]

            self.calibration_poses.append({
                'name': name,
                'pose': new_pose,
                'position': [new_pose.position.x, new_pose.position.y, new_pose.position.z],
                'orientation_rpy': [new_roll, new_pitch, new_yaw],
                'orientation_quat': [q_new[0], q_new[1], q_new[2], q_new[3]]
            })

            print(f"{Colors.GREEN}✓{Colors.RESET} {name}: "
                  f"pos=({new_pose.position.x:.3f}, {new_pose.position.y:.3f}, {new_pose.position.z:.3f}), "
                  f"rpy=({math.degrees(new_roll):.1f}°, {math.degrees(new_pitch):.1f}°, {math.degrees(new_yaw):.1f}°)")

        print(f"\n{Colors.GREEN}✓ 已生成 {len(self.calibration_poses)} 个标定位置{Colors.RESET}\n")
        return True

    def execute_calibration_sequence(self, capture_delay=2.0, save_config=True):
        """执行标定序列"""
        print(f"{Colors.CYAN}{Colors.BOLD}开始执行标定序列...{Colors.RESET}\n")

        success_count = 0
        failed_poses = []

        for i, calib_data in enumerate(self.calibration_poses, 1):
            name = calib_data['name']
            pose = calib_data['pose']

            print(f"{Colors.CYAN}[{i}/{len(self.calibration_poses)}] {name}{Colors.RESET}")

            # 设置目标位姿
            self.move_group.set_pose_target(pose)

            # 规划
            print(f"  {Colors.GRAY}规划中...{Colors.RESET}", end='', flush=True)
            plan = self.move_group.plan()

            # 检查规划结果（兼容不同 MoveIt 版本）
            if isinstance(plan, tuple):
                success, trajectory, planning_time, error_code = plan
            else:
                success = bool(plan.joint_trajectory.points)
                trajectory = plan

            if not success:
                print(f"\r  {Colors.RED}✗ 规划失败{Colors.RESET}")
                failed_poses.append(name)
                continue

            print(f"\r  {Colors.GREEN}✓ 规划成功{Colors.RESET}")

            # 执行
            print(f"  {Colors.GRAY}执行中...{Colors.RESET}", end='', flush=True)
            result = self.move_group.execute(trajectory, wait=True)

            if not result:
                print(f"\r  {Colors.RED}✗ 执行失败{Colors.RESET}")
                failed_poses.append(name)
                continue

            print(f"\r  {Colors.GREEN}✓ 执行成功{Colors.RESET}")

            # 停止运动
            self.move_group.stop()
            self.move_group.clear_pose_targets()

            # 等待稳定
            time.sleep(capture_delay)

            # 触发相机拍照
            self.trigger_camera(name)

            success_count += 1
            print(f"  {Colors.GREEN}✓ 完成{Colors.RESET}\n")

        # 打印总结
        print(f"{Colors.CYAN}{Colors.BOLD}{'='*70}{Colors.RESET}")
        print(f"{Colors.GREEN}✓ 标定序列执行完成{Colors.RESET}")
        print(f"  成功: {success_count}/{len(self.calibration_poses)}")
        if failed_poses:
            print(f"  {Colors.RED}失败: {', '.join(failed_poses)}{Colors.RESET}")
        print(f"{Colors.CYAN}{Colors.BOLD}{'='*70}{Colors.RESET}\n")

        # 保存配置
        if save_config:
            self.save_calibration_config()

        return success_count == len(self.calibration_poses)

    def trigger_camera(self, pose_name):
        """触发相机拍照"""
        print(f"  {Colors.YELLOW}📷 触发相机拍照: {pose_name}{Colors.RESET}")

        # 发布相机触发消息
        msg = String()
        msg.data = pose_name
        self.camera_trigger_pub.publish(msg)

        # 这里可以添加实际的相机触发代码
        # 例如：调用相机 ROS 服务、发送 HTTP 请求等

        # 示例：如果使用 ROS 服务
        # try:
        #     rospy.wait_for_service('/camera/capture', timeout=1.0)
        #     capture_service = rospy.ServiceProxy('/camera/capture', Trigger)
        #     response = capture_service()
        #     if response.success:
        #         print(f"  {Colors.GREEN}✓ 相机拍照成功{Colors.RESET}")
        # except:
        #     print(f"  {Colors.YELLOW}⚠ 相机服务不可用{Colors.RESET}")

    def save_calibration_config(self):
        """保存标定配置到 YAML 文件"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        config_dir = os.path.expanduser("~/aubo_polish/calibration_data")

        # 创建目录
        if not os.path.exists(config_dir):
            os.makedirs(config_dir)

        config_file = os.path.join(config_dir, f"camera_calibration_{timestamp}.yaml")

        # 准备配置数据
        config_data = {
            'timestamp': timestamp,
            'base_pose': {
                'position': [
                    self.base_pose.position.x,
                    self.base_pose.position.y,
                    self.base_pose.position.z
                ],
                'orientation_quat': [
                    self.base_pose.orientation.x,
                    self.base_pose.orientation.y,
                    self.base_pose.orientation.z,
                    self.base_pose.orientation.w
                ]
            },
            'base_joints': [float(j) for j in self.base_joints],
            'calibration_poses': []
        }

        for calib_data in self.calibration_poses:
            config_data['calibration_poses'].append({
                'name': calib_data['name'],
                'position': calib_data['position'],
                'orientation_rpy': calib_data['orientation_rpy'],
                'orientation_quat': calib_data['orientation_quat']
            })

        # 保存到文件
        with open(config_file, 'w') as f:
            yaml.dump(config_data, f, default_flow_style=False)

        print(f"{Colors.GREEN}✓ 配置已保存到: {config_file}{Colors.RESET}\n")

    def run(self):
        """主运行流程"""
        try:
            # 1. 读取当前示教位置
            if not self.read_current_pose():
                print(f"{Colors.RED}✗ 读取当前位置失败{Colors.RESET}")
                return False

            # 2. 确认是否继续
            print(f"{Colors.YELLOW}请确认：{Colors.RESET}")
            print(f"  1. 机器臂已手动示教到初始位置（相机正对物体，距离约 500mm）")
            print(f"  2. 相机已准备好拍照")
            print(f"  3. 工作空间内无障碍物\n")

            response = input(f"{Colors.CYAN}是否继续执行标定序列？(y/n): {Colors.RESET}")
            if response.lower() != 'y':
                print(f"{Colors.YELLOW}已取消{Colors.RESET}")
                return False

            # 3. 生成标定位置
            if not self.generate_calibration_poses():
                print(f"{Colors.RED}✗ 生成标定位置失败{Colors.RESET}")
                return False

            # 4. 执行标定序列
            success = self.execute_calibration_sequence(
                capture_delay=2.0,  # 每个位置停留 2 秒
                save_config=True    # 保存配置文件
            )

            if success:
                print(f"{Colors.GREEN}✓ 标定采集完成！{Colors.RESET}")
            else:
                print(f"{Colors.YELLOW}⚠ 标定采集部分完成（有失败的位置）{Colors.RESET}")

            return success

        except KeyboardInterrupt:
            print(f"\n{Colors.YELLOW}⚠ 用户中断{Colors.RESET}")
            self.move_group.stop()
            return False
        except Exception as e:
            print(f"\n{Colors.RED}✗ 错误: {e}{Colors.RESET}")
            self.move_group.stop()
            return False


def main():
    try:
        capture = CameraCalibrationCapture()
        capture.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
