#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

class TrajectoryValidator:
    """轨迹验证器"""
    def __init__(self):
        # 关节限位（从 URDF 读取）
        self.joint_limits = {
            'shoulder_joint': (-3.05, 3.05),
            'upperArm_joint': (-3.05, 3.05),
            'foreArm_joint': (-3.05, 3.05),
            'wrist1_joint': (-3.05, 3.05),
            'wrist2_joint': (-3.05, 3.05),
            'wrist3_joint': (-3.05, 3.05)
        }
        # 速度限位
        self.velocity_limits = [3.14, 3.14, 3.14, 2.6, 2.6, 2.6]

    def validate(self, trajectory):
        if trajectory is None:
            return False

        self._check_joint_limits(trajectory)
        self._check_velocity_limits(trajectory)
        return True

    def _check_joint_limits(self, trajectory):
        for point in trajectory.joint_trajectory.points:
            for i, pos in enumerate(point.positions):
                joint_name = trajectory.joint_trajectory.joint_names[i]
                limits = self.joint_limits[joint_name]
                if pos < limits[0] or pos > limits[1]:
                    raise ValueError(f"Joint {joint_name} position {pos} exceeds limits {limits}")

    def _check_velocity_limits(self, trajectory):
        for point in trajectory.joint_trajectory.points:
            for i, vel in enumerate(point.velocities):
                if abs(vel) > self.velocity_limits[i]:
                    rospy.logwarn(f"Joint {i} velocity {vel} exceeds limit {self.velocity_limits[i]}")
