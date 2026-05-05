#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from abc import ABC, abstractmethod

class BasePlanner(ABC):
    """规划器抽象基类"""
    @abstractmethod
    def plan(self, goal):
        """规划到目标位置"""
        pass

    @abstractmethod
    def get_name(self):
        """返回规划器名称"""
        pass
