#!/bin/bash
# 多目标轨迹快速启动脚本

cd ~/aubo_polish
source devel/setup.bash

echo "=========================================="
echo "AUBO E5 多目标轨迹执行"
echo "=========================================="
echo ""

# 显示可用的轨迹配置
echo "可用的轨迹配置："
echo "  1. square_motion.yaml   - 画正方形（20cm × 20cm）"
echo "  2. circle_motion.yaml   - 画圆形（半径 10cm）"
echo "  3. patrol_motion.yaml   - 多点巡航（4 个位置）"
echo "  4. motion_waypoints.yaml - 默认轨迹"
echo ""

# 读取用户选择
read -p "选择轨迹 (1-4) 或输入自定义配置文件名: " choice

case $choice in
    1)
        config="square_motion.yaml"
        ;;
    2)
        config="circle_motion.yaml"
        ;;
    3)
        config="patrol_motion.yaml"
        ;;
    4)
        config="motion_waypoints.yaml"
        ;;
    *)
        config="$choice"
        ;;
esac

# 询问是否循环执行
read -p "是否循环执行？(y/n): " loop_choice

if [ "$loop_choice" = "y" ]; then
    read -p "循环次数（留空表示无限循环）: " iterations
    if [ -z "$iterations" ]; then
        echo "启动无限循环模式..."
        rosrun aubo_linked_execution continuous_motion_demo.py \
            --mode cartesian \
            --config "$config" \
            --loop
    else
        echo "启动循环模式（$iterations 次）..."
        rosrun aubo_linked_execution continuous_motion_demo.py \
            --mode cartesian \
            --config "$config" \
            --loop \
            --iterations "$iterations"
    fi
else
    echo "启动单次执行模式..."
    rosrun aubo_linked_execution continuous_motion_demo.py \
        --mode cartesian \
        --config "$config"
fi
