#!/bin/bash
# 快速启动增强版 goto_pose

cd ~/aubo_polish
source devel/setup.bash

echo "启动增强版 goto_pose..."
rosrun aubo_planner goto_pose_enhanced.py
