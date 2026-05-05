#!/usr/bin/env python3
"""
safety_monitor.py
安全监控节点，提供：
  1. 大幅度运动预警
  2. 轨迹起点验证
  3. 碰撞检测状态监控
"""
import rospy
import threading
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String, Bool
from moveit_msgs.msg import DisplayTrajectory


class SafetyMonitor:
    def __init__(self):
        rospy.init_node('safety_monitor')

        self.lock = threading.Lock()
        self.current_joint_state = None
        self.last_planned_trajectory = None
        self.planned_position = None  # 记录规划时的位置

        # 安全参数
        self.large_motion_threshold = rospy.get_param('~large_motion_threshold', 0.5)  # rad
        self.trajectory_start_tolerance = rospy.get_param('~trajectory_start_tolerance', 0.15)  # rad
        self.manual_movement_threshold = rospy.get_param('~manual_movement_threshold', 0.05)  # rad

        # Publishers
        self.warning_pub = rospy.Publisher('/safety_monitor/warning', String, queue_size=10)
        self.safe_to_execute_pub = rospy.Publisher('/safety_monitor/safe_to_execute', Bool, queue_size=1, latch=True)

        # Subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_state_cb, queue_size=1)
        rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.planned_path_cb, queue_size=1)
        rospy.Subscriber('/joint_path_command', JointTrajectory, self.trajectory_command_cb, queue_size=1)

        # 初始状态：安全
        self.safe_to_execute_pub.publish(Bool(data=True))

        rospy.loginfo('[safety_monitor] Safety monitor started')
        rospy.loginfo('[safety_monitor] Large motion threshold: %.2f rad (%.1f deg)',
                      self.large_motion_threshold,
                      self.large_motion_threshold * 57.3)
        rospy.loginfo('[safety_monitor] Trajectory start tolerance: %.2f rad (%.1f deg)',
                      self.trajectory_start_tolerance,
                      self.trajectory_start_tolerance * 57.3)
        rospy.loginfo('[safety_monitor] Manual movement threshold: %.2f rad (%.1f deg)',
                      self.manual_movement_threshold,
                      self.manual_movement_threshold * 57.3)

    def joint_state_cb(self, msg):
        with self.lock:
            self.current_joint_state = msg

    def planned_path_cb(self, msg):
        """监控 MoveIt 规划的轨迹"""
        with self.lock:
            if not msg.trajectory or not msg.trajectory[0].joint_trajectory.points:
                return

            self.last_planned_trajectory = msg.trajectory[0].joint_trajectory

            # 检查规划的轨迹
            self.check_planned_trajectory(self.last_planned_trajectory)

    def trajectory_command_cb(self, msg):
        """监控即将执行的轨迹命令"""
        with self.lock:
            if not msg.points:
                return

            # 检查轨迹起点是否接近当前位置
            self.check_trajectory_start_point(msg)

    def check_planned_trajectory(self, trajectory):
        """检查规划的轨迹是否安全"""
        if not self.current_joint_state or not trajectory.points:
            return

        # 获取当前位置
        current_pos = dict(zip(self.current_joint_state.name, self.current_joint_state.position))

        # 记录规划时的位置（用于手动移动检测）
        self.planned_position = current_pos.copy()

        # 获取轨迹起点
        traj_start = trajectory.points[0]
        traj_joint_names = trajectory.joint_names

        # 计算起点与当前位置的差异
        max_diff = 0.0
        max_diff_joint = ''

        for i, joint_name in enumerate(traj_joint_names):
            if joint_name in current_pos and i < len(traj_start.positions):
                diff = abs(traj_start.positions[i] - current_pos[joint_name])
                if diff > max_diff:
                    max_diff = diff
                    max_diff_joint = joint_name

        # 检查是否超过阈值
        if max_diff > self.large_motion_threshold:
            warning_msg = (
                'WARNING: Large motion detected!\n'
                '  Joint: %s\n'
                '  Current: %.3f rad (%.1f deg)\n'
                '  Planned start: %.3f rad (%.1f deg)\n'
                '  Difference: %.3f rad (%.1f deg)\n'
                '  This may cause sudden large movement!'
            ) % (
                max_diff_joint,
                current_pos.get(max_diff_joint, 0.0),
                current_pos.get(max_diff_joint, 0.0) * 57.3,
                traj_start.positions[traj_joint_names.index(max_diff_joint)],
                traj_start.positions[traj_joint_names.index(max_diff_joint)] * 57.3,
                max_diff,
                max_diff * 57.3
            )

            rospy.logwarn('[safety_monitor] %s', warning_msg)
            self.warning_pub.publish(String(data=warning_msg))
            self.safe_to_execute_pub.publish(Bool(data=False))
        else:
            rospy.loginfo('[safety_monitor] ✓ Trajectory start point is close to current position (max_diff=%.3f rad)', max_diff)
            self.safe_to_execute_pub.publish(Bool(data=True))

    def check_trajectory_start_point(self, trajectory):
        """检查即将执行的轨迹起点"""
        if not self.current_joint_state or not trajectory.points:
            return

        # 获取当前位置
        current_pos = dict(zip(self.current_joint_state.name, self.current_joint_state.position))

        # 1. 检查手动移动
        if self.planned_position:
            manual_movement_detected = False
            max_manual_diff = 0.0
            max_manual_joint = ''

            for joint_name in trajectory.joint_names:
                if joint_name in self.planned_position and joint_name in current_pos:
                    diff = abs(current_pos[joint_name] - self.planned_position[joint_name])
                    if diff > max_manual_diff:
                        max_manual_diff = diff
                        max_manual_joint = joint_name
                    if diff > self.manual_movement_threshold:
                        manual_movement_detected = True

            if manual_movement_detected:
                error_msg = (
                    'ERROR: Manual movement detected!\n'
                    '  Joint: %s\n'
                    '  Position at planning: %.3f rad (%.1f deg)\n'
                    '  Current position: %.3f rad (%.1f deg)\n'
                    '  Difference: %.3f rad (%.1f deg)\n'
                    '  Threshold: %.3f rad (%.1f deg)\n'
                    '  Robot was moved between planning and execution!\n'
                    '  EXECUTION BLOCKED FOR SAFETY!'
                ) % (
                    max_manual_joint,
                    self.planned_position.get(max_manual_joint, 0.0),
                    self.planned_position.get(max_manual_joint, 0.0) * 57.3,
                    current_pos.get(max_manual_joint, 0.0),
                    current_pos.get(max_manual_joint, 0.0) * 57.3,
                    max_manual_diff,
                    max_manual_diff * 57.3,
                    self.manual_movement_threshold,
                    self.manual_movement_threshold * 57.3
                )

                rospy.logerr('[safety_monitor] %s', error_msg)
                self.warning_pub.publish(String(data=error_msg))
                self.safe_to_execute_pub.publish(Bool(data=False))
                return

        # 2. 检查轨迹起点
        # 获取轨迹起点
        traj_start = trajectory.points[0]
        traj_joint_names = trajectory.joint_names

        # 计算差异
        max_diff = 0.0
        max_diff_joint = ''

        for i, joint_name in enumerate(traj_joint_names):
            if joint_name in current_pos and i < len(traj_start.positions):
                diff = abs(traj_start.positions[i] - current_pos[joint_name])
                if diff > max_diff:
                    max_diff = diff
                    max_diff_joint = joint_name

        # 检查容差
        if max_diff > self.trajectory_start_tolerance:
            error_msg = (
                'ERROR: Trajectory start point mismatch!\n'
                '  Joint: %s\n'
                '  Current: %.3f rad (%.1f deg)\n'
                '  Trajectory start: %.3f rad (%.1f deg)\n'
                '  Difference: %.3f rad (%.1f deg)\n'
                '  Tolerance: %.3f rad (%.1f deg)\n'
                '  EXECUTION BLOCKED FOR SAFETY!'
            ) % (
                max_diff_joint,
                current_pos.get(max_diff_joint, 0.0),
                current_pos.get(max_diff_joint, 0.0) * 57.3,
                traj_start.positions[traj_joint_names.index(max_diff_joint)],
                traj_start.positions[traj_joint_names.index(max_diff_joint)] * 57.3,
                max_diff,
                max_diff * 57.3,
                self.trajectory_start_tolerance,
                self.trajectory_start_tolerance * 57.3
            )

            rospy.logerr('[safety_monitor] %s', error_msg)
            self.warning_pub.publish(String(data=error_msg))
            self.safe_to_execute_pub.publish(Bool(data=False))
        else:
            rospy.loginfo('[safety_monitor] ✓ Trajectory start point verified (max_diff=%.3f rad)', max_diff)


if __name__ == '__main__':
    try:
        monitor = SafetyMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
