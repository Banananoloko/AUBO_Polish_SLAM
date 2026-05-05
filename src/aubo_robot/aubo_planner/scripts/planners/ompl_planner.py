#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import moveit_commander
from .base_planner import BasePlanner

class OMPLPlanner(BasePlanner):
    """MoveIt OMPL 规划器"""
    def __init__(self, algorithm="RRTConnect", planning_time=5.0):
        self.move_group = moveit_commander.MoveGroupCommander("manipulator_e5")
        self.move_group.set_planner_id(algorithm)
        self.move_group.set_planning_time(planning_time)
        self.move_group.set_num_planning_attempts(10)

    def plan(self, goal):
        goal.to_moveit_goal(self.move_group)
        plan = self.move_group.plan()
        return plan[1] if plan[0] else None

    def get_name(self):
        return "ompl"
