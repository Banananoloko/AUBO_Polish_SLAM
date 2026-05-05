#!/bin/bash
# P6 Shadow 模式启动脚本

cd ~/aubo_polish
source devel/setup.bash

echo "=========================================="
echo "启动 P6 Shadow 模式"
echo "实机 IP: 192.168.1.10"
echo "=========================================="
echo ""

roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.1.10 use_unity:=true
