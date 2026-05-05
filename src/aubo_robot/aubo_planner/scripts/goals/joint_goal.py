#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from .motion_goal import MotionGoal

class JointGoal(MotionGoal):
    """关节空间目标"""
    def __init__(self, joint_values):
        if len(joint_values) != 6:
            raise ValueError("AUBO E5 requires 6 joint values")
        self.joint_values = joint_values

    def to_moveit_goal(self, move_group):
        move_group.set_joint_value_target(self.joint_values)
