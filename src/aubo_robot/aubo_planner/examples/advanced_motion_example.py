#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from motion_planning_interface import MotionPlanningInterface
from goals.pose_goal import PoseGoal

def on_feedback(feedback):
    print(f"Execution progress: {feedback}")

def on_done(state, result):
    print(f"Execution completed with state: {state}")

def main():
    planner = MotionPlanningInterface()

    # 异步执行带回调
    goal = PoseGoal(0.5, 0.2, 0.3, 0, 1.57, 0)
    trajectory = planner.planner.plan(goal)

    if trajectory:
        print("Planning succeeded! Executing asynchronously...")
        planner.executor.execute_async(
            trajectory,
            done_cb=on_done,
            feedback_cb=on_feedback
        )

        # 主线程可以继续做其他事情
        import rospy
        rospy.spin()

if __name__ == '__main__':
    main()
