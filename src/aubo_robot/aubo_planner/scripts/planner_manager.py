#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class PlannerManager:
    """规划器管理器"""
    def __init__(self):
        self._planners = {}
        self._register_default_planners()

    def _register_default_planners(self):
        from planners.ompl_planner import OMPLPlanner
        self.register_planner(OMPLPlanner())

    def register_planner(self, planner):
        self._planners[planner.get_name()] = planner

    def select_planner(self, name_or_strategy):
        if name_or_strategy == "auto":
            return self._planners["ompl"]  # 默认使用 OMPL
        return self._planners.get(name_or_strategy)

    def get_available_planners(self):
        return list(self._planners.keys())
