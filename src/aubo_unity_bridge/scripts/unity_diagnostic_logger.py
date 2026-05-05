#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unity 仿真诊断日志记录器
实时记录关键状态信息，帮助定位收敛判定问题
"""

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from industrial_msgs.msg import RobotStatus
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryActionResult
import datetime
import os

class UnityDiagnosticLogger:
    def __init__(self):
        self.log_dir = os.path.expanduser("~/aubo_polish/diagnostic_logs")
        os.makedirs(self.log_dir, exist_ok=True)

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(self.log_dir, f"unity_diagnostic_{timestamp}.txt")

        self.goal_received = False
        self.goal_position = None
        self.start_time = None

        # 订阅关键话题
        rospy.Subscriber('/aubo_e5/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/feedback_states', JointTrajectoryPoint, self.feedback_callback)
        rospy.Subscriber('/robot_status', RobotStatus, self.robot_status_callback)
        rospy.Subscriber('/aubo_e5_controller/follow_joint_trajectory/goal',
                        FollowJointTrajectoryActionGoal, self.goal_callback)
        rospy.Subscriber('/aubo_e5_controller/follow_joint_trajectory/result',
                        FollowJointTrajectoryActionResult, self.result_callback)

        self.log("=== Unity 仿真诊断日志启动 ===")
        self.log(f"日志文件: {self.log_file}")
        self.log("")

        rospy.loginfo(f"诊断日志记录器已启动，日志文件: {self.log_file}")

    def log(self, message):
        """写入日志文件"""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with open(self.log_file, 'a') as f:
            f.write(f"[{timestamp}] {message}\n")

    def joint_states_callback(self, msg):
        """记录关节状态"""
        if not self.goal_received:
            return

        # 计算与目标的误差
        if self.goal_position and len(msg.position) == len(self.goal_position):
            errors = [abs(c - g) for c, g in zip(msg.position, self.goal_position)]
            max_error = max(errors)
            max_error_idx = errors.index(max_error)

            # 记录速度信息
            velocities = msg.velocity if msg.velocity else [0.0] * len(msg.position)
            max_vel = max(abs(v) for v in velocities)

            elapsed = (rospy.Time.now() - self.start_time).to_sec() if self.start_time else 0

            self.log(f"[JOINT_STATES] t={elapsed:.2f}s | "
                    f"max_error={max_error:.4f} rad ({max_error*57.3:.2f}°) @ joint_{max_error_idx} | "
                    f"max_vel={max_vel:.4f} rad/s | "
                    f"velocities={[f'{v:.4f}' for v in velocities]}")

    def feedback_callback(self, msg):
        """记录反馈状态"""
        if not self.goal_received:
            return

        elapsed = (rospy.Time.now() - self.start_time).to_sec() if self.start_time else 0

        # 计算与目标的误差
        if self.goal_position and len(msg.positions) == len(self.goal_position):
            errors = [abs(c - g) for c, g in zip(msg.positions, self.goal_position)]
            max_error = max(errors)

            velocities = msg.velocities if msg.velocities else [0.0] * len(msg.positions)
            max_vel = max(abs(v) for v in velocities)

            self.log(f"[FEEDBACK] t={elapsed:.2f}s | "
                    f"max_error={max_error:.4f} rad | "
                    f"max_vel={max_vel:.4f} rad/s")

    def robot_status_callback(self, msg):
        """记录机器人状态"""
        if not self.goal_received:
            return

        elapsed = (rospy.Time.now() - self.start_time).to_sec() if self.start_time else 0

        in_motion_str = "UNKNOWN"
        if msg.in_motion.val == -1:
            in_motion_str = "UNKNOWN"
        elif msg.in_motion.val == 0:
            in_motion_str = "FALSE"
        elif msg.in_motion.val == 1:
            in_motion_str = "TRUE"

        self.log(f"[ROBOT_STATUS] t={elapsed:.2f}s | "
                f"in_motion={in_motion_str} | "
                f"mode={msg.mode.val} | "
                f"e_stopped={msg.e_stopped.val}")

    def goal_callback(self, msg):
        """记录目标接收"""
        self.goal_received = True
        self.start_time = rospy.Time.now()

        trajectory = msg.goal.trajectory
        if trajectory.points:
            last_point = trajectory.points[-1]
            self.goal_position = list(last_point.positions)

            self.log("")
            self.log("=" * 80)
            self.log("[GOAL_RECEIVED] 新的轨迹目标")
            self.log(f"  关节数量: {len(self.goal_position)}")
            self.log(f"  目标位置: {[f'{p:.4f}' for p in self.goal_position]}")
            self.log(f"  轨迹点数: {len(trajectory.points)}")
            self.log(f"  预计时长: {last_point.time_from_start.to_sec():.2f}s")
            self.log("=" * 80)

    def result_callback(self, msg):
        """记录执行结果"""
        if not self.goal_received:
            return

        elapsed = (rospy.Time.now() - self.start_time).to_sec() if self.start_time else 0

        result_str = "UNKNOWN"
        if msg.status.status == 3:
            result_str = "SUCCEEDED"
        elif msg.status.status == 4:
            result_str = "ABORTED"
        elif msg.status.status == 5:
            result_str = "REJECTED"

        self.log("")
        self.log("=" * 80)
        self.log(f"[RESULT] 执行结果: {result_str}")
        self.log(f"  总耗时: {elapsed:.2f}s")
        self.log(f"  状态文本: {msg.status.text}")
        self.log(f"  错误代码: {msg.result.error_code}")
        self.log("=" * 80)
        self.log("")

        # 重置状态
        self.goal_received = False
        self.goal_position = None
        self.start_time = None

def main():
    rospy.init_node('unity_diagnostic_logger', anonymous=False)
    logger = UnityDiagnosticLogger()

    rospy.loginfo("诊断日志记录器运行中...")
    rospy.loginfo("按 Ctrl+C 停止")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("诊断日志记录器已停止")

if __name__ == '__main__':
    main()
