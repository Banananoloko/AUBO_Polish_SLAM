#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
from .motion_goal import MotionGoal

class PoseGoal(MotionGoal):
    """笛卡尔空间目标"""
    def __init__(self, x, y, z, roll, pitch, yaw, frame_id="base_link"):
        self.pose = Pose()
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = z
        q = quaternion_from_euler(roll, pitch, yaw)
        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]
        self.frame_id = frame_id

    def to_moveit_goal(self, move_group):
        move_group.set_pose_target(self.pose)
