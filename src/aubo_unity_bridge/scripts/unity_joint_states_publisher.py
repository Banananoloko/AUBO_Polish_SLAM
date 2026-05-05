#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
unity_joint_states_publisher.py
================================
订阅 Unity 端通过 ROS-TCP-Endpoint 发来的关节状态消息，
转发到 /aubo_e5/joint_states，供 robot_state_publisher 和 MoveIt 使用。

对标 Gazebo 的 joint_state_controller：发布 sensor_msgs/JointState 到 /aubo_e5/joint_states。

Unity 侧约定：
  - Unity 通过 ROS-TCP-Endpoint 发布到 /unity/joint_states (sensor_msgs/JointState)
  - 关节顺序需为 AUBO 规范顺序（与 JOINT_NAMES 一致）
  - 发布频率建议 50 Hz（与 Gazebo 一致）

本节点的作用是「话题转发 + 关节顺序校验 + 时间戳重打」。
"""

import rospy
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    'shoulder_joint',
    'upperArm_joint',
    'foreArm_joint',
    'wrist1_joint',
    'wrist2_joint',
    'wrist3_joint',
]


class UnityJointStatesBridge:
    def __init__(self):
        self.input_topic = rospy.get_param('~input_topic', '/unity/joint_states')
        self.output_topic = rospy.get_param('~output_topic', '/aubo_e5/joint_states')
        self.restamp = rospy.get_param('~restamp', True)

        self.pub = rospy.Publisher(self.output_topic, JointState, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, JointState, self.on_unity_state, queue_size=10)

        rospy.loginfo("[unity_joint_states_publisher] %s -> %s",
                      self.input_topic, self.output_topic)

    def on_unity_state(self, msg):
        if not msg.name:
            rospy.logwarn_throttle(5.0, "Unity JointState message has no joint names; dropping")
            return

        # 关节顺序映射：Unity 可能不按 AUBO 规范顺序，按名称重排
        try:
            indices = [msg.name.index(n) for n in JOINT_NAMES]
        except ValueError as e:
            rospy.logwarn_throttle(5.0, "Joint name mismatch from Unity: %s", e)
            return

        out = JointState()
        out.header.stamp = rospy.Time.now() if self.restamp else msg.header.stamp
        out.name = JOINT_NAMES
        out.position = [msg.position[i] for i in indices] if msg.position else []
        out.velocity = [msg.velocity[i] for i in indices] if len(msg.velocity) == len(msg.name) else [0.0] * 6
        out.effort = [msg.effort[i] for i in indices] if len(msg.effort) == len(msg.name) else [0.0] * 6

        self.pub.publish(out)


def main():
    rospy.init_node('unity_joint_states_publisher', anonymous=False)
    UnityJointStatesBridge()
    rospy.spin()


if __name__ == '__main__':
    main()
