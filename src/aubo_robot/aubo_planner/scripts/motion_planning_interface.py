#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
from planner_manager import PlannerManager
from validators.trajectory_validator import TrajectoryValidator
from execution.async_executor import AsyncExecutor
from goals.pose_goal import PoseGoal
from goals.joint_goal import JointGoal

class MotionPlanningInterface:
    """运动规划统一接口"""
    def __init__(self, planner_name="auto"):
        rospy.init_node('motion_planning_interface', anonymous=True)
        moveit_commander.roscpp_initialize([])

        self.planner_manager = PlannerManager()
        self.planner = self.planner_manager.select_planner(planner_name)
        self.validator = TrajectoryValidator()
        self.executor = AsyncExecutor()

    def plan_to_pose(self, x, y, z, roll, pitch, yaw):
        """规划到笛卡尔目标位置"""
        goal = PoseGoal(x, y, z, roll, pitch, yaw)
        trajectory = self.planner.plan(goal)
        if trajectory and self.validator.validate(trajectory):
            return trajectory
        return None

    def plan_to_joint_values(self, joint_values):
        """规划到关节空间目标"""
        goal = JointGoal(joint_values)
        trajectory = self.planner.plan(goal)
        if trajectory and self.validator.validate(trajectory):
            return trajectory
        return None

    def execute(self, trajectory, blocking=False):
        """执行轨迹（走 MoveIt 标准执行管道，与手动规划路径一致）"""
        success = self.planner.move_group.execute(trajectory, wait=blocking)
        self.planner.move_group.stop()
        return success

    def plan_and_execute(self, goal, blocking=False):
        """规划并执行"""
        trajectory = self.planner.plan(goal)
        if trajectory and self.validator.validate(trajectory):
            return self.execute(trajectory, blocking)
        return False

    def set_planner(self, planner_name):
        """切换规划器"""
        self.planner = self.planner_manager.select_planner(planner_name)
