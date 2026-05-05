#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class AsyncExecutor:
    """异步执行器"""
    def __init__(self, action_name='/linked_execution_controller/follow_joint_trajectory'):
        self.client = actionlib.SimpleActionClient(action_name, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server(timeout=rospy.Duration(10.0))
        rospy.loginfo("Action server connected")

    def execute_async(self, trajectory, done_cb=None, feedback_cb=None):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory.joint_trajectory
        self.client.send_goal(goal, done_cb=done_cb, feedback_cb=feedback_cb)

    def execute_sync(self, trajectory, timeout=30.0):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory.joint_trajectory
        self.client.send_goal_and_wait(goal, execute_timeout=rospy.Duration(timeout))
        return self.client.get_state() == actionlib.GoalStatus.SUCCEEDED
