#!/bin/bash
# 相机标定快速启动脚本

cd ~/aubo_polish
source devel/setup.bash

echo "=========================================="
echo "相机标定自动化采集系统"
echo "=========================================="
echo ""
echo "使用说明："
echo "1. 确保系统已启动（实机或仿真）"
echo "2. 手动示教机器臂到初始位置"
echo "   - 相机正对物体"
echo "   - 距离约 500mm"
echo "3. 运行此脚本"
echo ""
read -p "按 Enter 键继续..."

rosrun aubo_planner camera_calibration_capture.py
