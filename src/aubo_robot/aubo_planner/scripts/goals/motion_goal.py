#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from abc import ABC, abstractmethod

class MotionGoal(ABC):
    """运动目标抽象基类"""
    @abstractmethod
    def to_moveit_goal(self, move_group):
        """转换为 MoveIt 可识别的目标"""
        pass
