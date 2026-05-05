#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
unity_command_forwarder.py
===========================
监听 ROS 侧的关节指令，转发到 Unity 端通过 ROS-TCP-Endpoint。

支持两种工作模式：

1. NORMAL 模式（sim_only:=true，对标 aubo_gazebo_driver normal 模式）
   - 订阅 /joint_path_command （MoveIt 下发的轨迹）
   - 取最后一个 waypoint 作为目标位置发给 Unity
   - 或者订阅 6 个 /aubo_e5/<joint>_position_controller/command （单关节命令）

2. SHADOW 模式（sim_only:=false，对标 aubo_gazebo_driver shadow 模式）
   - 订阅 /real/joint_states （来自 joint_state_mirror_adapter）
   - 直接镜像实机关节状态发给 Unity，让 Unity 跟随实机

Unity 侧约定：
  - Unity 订阅 /unity/joint_command (sensor_msgs/JointState) 接收目标位置
  - Unity 内部用 ArticulationBody / ConfigurableJoint 做位置驱动
"""

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64

JOINT_NAMES = [
    'shoulder_joint',
    'upperArm_joint',
    'foreArm_joint',
    'wrist1_joint',
    'wrist2_joint',
    'wrist3_joint',
]


class UnityCommandForwarder:
    def __init__(self):
        self.mode = rospy.get_param('~mode', 'shadow')  # 'normal' | 'shadow'
        self.unity_cmd_topic = rospy.get_param('~unity_cmd_topic', '/unity/joint_command')

        # 速度限制和滤波参数
        self.max_joint_velocity = rospy.get_param('~max_joint_velocity', 0.5)
        self.filter_alpha = rospy.get_param('~shadow_filter_alpha', 0.5)  # 增加到 0.5 以更强平滑
        self.enable_velocity_limit = rospy.get_param('~enable_velocity_limit', True)

        # Shadow mode 死区和阈值参数
        self.position_deadzone = rospy.get_param('~position_deadzone', 0.001)  # 位置死区 (rad)
        self.velocity_threshold = rospy.get_param('~velocity_threshold', 0.005)  # 速度阈值 (rad/s)

        # Shadow mode 平滑滤波器
        self.last_shadow_position = None
        self.last_shadow_time = None
        self.velocity_filter = None
        self.last_published_position = None  # 上次发布的位置，用于死区判断

        self.pub = rospy.Publisher(self.unity_cmd_topic, JointState, queue_size=10)

        if self.mode == 'shadow':
            self._setup_shadow_mode()
        elif self.mode == 'normal':
            self._setup_normal_mode()
        else:
            rospy.logfatal("Unknown mode: %s (expected 'normal' or 'shadow')", self.mode)
            raise SystemExit(1)

        rospy.loginfo("[unity_command_forwarder] mode=%s, publishing to %s, "
                      "max_velocity=%.2f rad/s, filter_alpha=%.2f, velocity_limit=%s, "
                      "position_deadzone=%.4f rad, velocity_threshold=%.4f rad/s",
                      self.mode, self.unity_cmd_topic, self.max_joint_velocity,
                      self.filter_alpha, self.enable_velocity_limit,
                      self.position_deadzone, self.velocity_threshold)

    def _setup_shadow_mode(self):
        """Shadow 模式：直接镜像 /real/joint_states 给 Unity"""
        topic = rospy.get_param('~shadow_input', '/real/joint_states')
        self.sub = rospy.Subscriber(topic, JointState, self._forward_joint_state, queue_size=10)
        rospy.loginfo("[shadow] subscribing %s", topic)

    def _setup_normal_mode(self):
        """Normal 模式：订阅 MoveIt 下发的轨迹，取最后一点发给 Unity"""
        topic = rospy.get_param('~normal_input', '/joint_path_command')
        self.sub = rospy.Subscriber(topic, JointTrajectory, self._forward_trajectory, queue_size=10)
        rospy.loginfo("[normal] subscribing %s", topic)

    def _forward_joint_state(self, msg):
        """Shadow 模式：转发实机关节状态，添加平滑滤波、位置死区和速度阈值"""
        current_time = rospy.Time.now()
        current_position = list(msg.position)

        # 计算速度并应用平滑滤波
        if self.last_shadow_position is not None:
            dt = (current_time - self.last_shadow_time).to_sec()
            if dt > 0:
                # 计算瞬时速度
                raw_velocity = [(c - l) / dt
                               for c, l in zip(current_position, self.last_shadow_position)]

                # 低通滤波（指数移动平均）
                if self.velocity_filter is None:
                    self.velocity_filter = raw_velocity
                else:
                    self.velocity_filter = [
                        self.filter_alpha * v + (1 - self.filter_alpha) * vf
                        for v, vf in zip(raw_velocity, self.velocity_filter)
                    ]

        # 位置死区检查：如果位置变化和速度都很小，跳过发布
        if self.last_published_position is not None and self.velocity_filter is not None:
            max_position_change = max(abs(c - l)
                                     for c, l in zip(current_position, self.last_published_position))
            max_velocity = max(abs(v) for v in self.velocity_filter)

            # 如果位置变化小于死区且速度低于阈值，跳过发布（实机静止）
            if max_position_change < self.position_deadzone and max_velocity < self.velocity_threshold:
                # 更新历史但不发布
                self.last_shadow_position = current_position
                self.last_shadow_time = current_time
                return

        out = JointState()
        out.header.stamp = current_time
        out.name = list(msg.name)
        out.position = current_position

        # 发送平滑后的速度
        if self.velocity_filter is not None:
            out.velocity = self.velocity_filter

        self.pub.publish(out)

        # 更新历史
        self.last_shadow_position = current_position
        self.last_shadow_time = current_time
        self.last_published_position = current_position

    def _forward_trajectory(self, msg):
        """Normal 模式：转发轨迹最后一点，添加速度限制"""
        if not msg.points:
            return

        # 取最后一个 waypoint 作为最终目标
        last_point = msg.points[-1]
        out = JointState()
        out.header.stamp = rospy.Time.now()
        out.name = list(msg.joint_names)
        out.position = list(last_point.positions)

        # 添加速度限制（匹配实机）
        if self.enable_velocity_limit:
            if last_point.velocities:
                # 如果轨迹包含速度信息，限制最大速度
                out.velocity = [min(abs(v), self.max_joint_velocity) * (1 if v >= 0 else -1)
                               for v in last_point.velocities]
            else:
                # 默认速度限制
                out.velocity = [self.max_joint_velocity] * len(out.position)

        self.pub.publish(out)


def main():
    rospy.init_node('unity_command_forwarder', anonymous=False)
    UnityCommandForwarder()
    rospy.spin()


if __name__ == '__main__':
    main()
