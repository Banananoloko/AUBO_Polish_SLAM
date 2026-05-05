#!/bin/bash
# P6 快速诊断脚本 - 检查 MoveIt 规划失败的原因

echo "=========================================="
echo "P6 MoveIt 规划失败诊断"
echo "=========================================="
echo ""

# 1. 检查 joint_states 话题
echo "1. 检查 /joint_states 话题（MoveIt 需要）:"
timeout 2 rostopic hz /joint_states 2>/dev/null && echo "  ✓ /joint_states 正常" || echo "  ✗ /joint_states 不存在或无数据"
echo ""

# 2. 检查 feedback_states 话题
echo "2. 检查 /feedback_states 话题（动作服务器需要）:"
timeout 2 rostopic hz /feedback_states 2>/dev/null && echo "  ✓ /feedback_states 正常" || echo "  ✗ /feedback_states 不存在或无数据"
echo ""

# 3. 检查动作服务器连接
echo "3. 检查动作服务器连接:"
timeout 2 rostopic echo -n 1 /aubo_e5_controller/follow_joint_trajectory/status 2>/dev/null | grep -q "status_list" && echo "  ✓ 动作服务器可访问" || echo "  ✗ 动作服务器不可访问"
echo ""

# 4. 检查 MoveIt 订阅的话题
echo "4. 检查 MoveIt 订阅的话题:"
rosnode info /move_group 2>/dev/null | grep -A 10 "Subscriptions:" | grep joint_states
echo ""

# 5. 检查话题映射
echo "5. 检查话题映射:"
echo "  /aubo_e5/joint_states → /joint_states"
rostopic info /joint_states 2>/dev/null | grep -q "Publishers" && echo "  ✓ /joint_states 有发布者" || echo "  ✗ /joint_states 无发布者"
echo ""

# 6. 建议
echo "=========================================="
echo "诊断建议"
echo "=========================================="
echo ""

if ! timeout 2 rostopic hz /joint_states &>/dev/null; then
    echo "⚠️ 问题：/joint_states 话题不存在"
    echo ""
    echo "原因：MoveIt 默认订阅 /joint_states，但当前系统可能只有 /aubo_e5/joint_states"
    echo ""
    echo "解决方案："
    echo "  1. 检查 unity_joint_states_publisher 是否正确重映射"
    echo "  2. 或者在 MoveIt 配置中修改订阅的话题名"
    echo ""
fi

if ! timeout 2 rostopic hz /feedback_states &>/dev/null; then
    echo "⚠️ 问题：/feedback_states 话题不存在"
    echo ""
    echo "原因：aubo_robot_simulator 可能未正确发布 feedback_states"
    echo ""
    echo "解决方案："
    echo "  1. 检查 aubo_robot_simulator 是否在运行"
    echo "  2. 检查 feedback_states 话题映射"
    echo ""
fi

echo "=========================================="
echo "下一步操作"
echo "=========================================="
echo ""
echo "在新终端运行："
echo "  cd ~/aubo_polish"
echo "  source devel/setup.bash"
echo "  rosrun aubo_planner goto_pose.py"
echo ""
echo "然后输入目标坐标："
echo "  目标坐标> 0.4 0.0 0.5"
echo ""
