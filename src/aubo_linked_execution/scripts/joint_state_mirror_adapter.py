#!/usr/bin/env python
"""
joint_state_mirror_adapter.py

Subscribes to /joint_states (published by aubo_driver for the real robot),
reorders joints to a fixed canonical order, and republishes as /real/joint_states
so that aubo_gazebo_driver in shadow mode receives consistently-ordered data.
"""

import rospy
from sensor_msgs.msg import JointState

JOINT_ORDER = [
    'shoulder_joint',
    'upperArm_joint',
    'foreArm_joint',
    'wrist1_joint',
    'wrist2_joint',
    'wrist3_joint',
]


class JointStateMirrorAdapter(object):
    def __init__(self):
        self._pub = rospy.Publisher('/real/joint_states', JointState, queue_size=1)
        self._sub = rospy.Subscriber('/joint_states', JointState, self._cb, queue_size=1)

    def _cb(self, msg):
        if len(msg.name) != len(JOINT_ORDER):
            rospy.logwarn_throttle(5.0,
                'joint_state_mirror_adapter: received %d joints, expected %d — dropping',
                len(msg.name), len(JOINT_ORDER))
            return

        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        for joint in JOINT_ORDER:
            if joint not in name_to_idx:
                rospy.logwarn_throttle(5.0,
                    'joint_state_mirror_adapter: joint "%s" missing in /joint_states — dropping', joint)
                return

        indices = [name_to_idx[j] for j in JOINT_ORDER]

        out = JointState()
        out.header = msg.header
        out.name = JOINT_ORDER[:]
        out.position = [msg.position[i] for i in indices] if msg.position else []
        out.velocity = [msg.velocity[i] for i in indices] if msg.velocity else []
        out.effort   = [msg.effort[i]   for i in indices] if msg.effort   else []

        self._pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('joint_state_mirror_adapter')
    JointStateMirrorAdapter()
    rospy.spin()
