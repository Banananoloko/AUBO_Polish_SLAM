#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fake_aubo_joint_states.py
=========================
临时假关节状态发布器，用于在 Unity 端尚未接通时验证 ROS 侧链路。
发布频率与 Gazebo joint_state_controller 一致（50Hz）。
启动 Unity 后，此节点应被关闭，由 unity_joint_states_publisher.py 取代。
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

HOME_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def main():
    rospy.init_node('fake_aubo_joint_states', anonymous=False)
    topic = rospy.get_param('~topic', '/aubo_e5/joint_states')
    rate_hz = rospy.get_param('~rate_hz', 50.0)

    pub = rospy.Publisher(topic, JointState, queue_size=10)
    rate = rospy.Rate(rate_hz)

    rospy.loginfo("[fake_aubo_joint_states] Publishing %s at %.1f Hz", topic, rate_hz)

    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = JOINT_NAMES
        msg.position = HOME_POSITION
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
