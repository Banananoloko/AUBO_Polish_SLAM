#!/bin/bash
# P6 Debug 脚本 - 实时监控和诊断工具
# 用途：在 P6 测试过程中实时监控系统状态

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "=========================================="
echo "P6 Shadow 模式实时监控"
echo "=========================================="
echo ""

# 检查 ROS 环境
if [ -z "$ROS_MASTER_URI" ]; then
    echo -e "${RED}错误: ROS 环境未加载${NC}"
    exit 1
fi

# 1. 节点存活检查
echo -e "${BLUE}=== 1. 节点存活检查 ===${NC}"
echo ""
echo "Unity 相关节点:"
rosnode list 2>/dev/null | grep -E 'unity' || echo "  无 Unity 节点"
echo ""
echo "AUBO 相关节点:"
rosnode list 2>/dev/null | grep -E 'aubo' || echo "  无 AUBO 节点"
echo ""
echo "MoveIt 相关节点:"
rosnode list 2>/dev/null | grep -E 'move_group' || echo "  无 MoveIt 节点"
echo ""

# 2. 话题频率检查
echo -e "${BLUE}=== 2. 话题频率检查 ===${NC}"
echo ""
echo "检查 /unity/joint_states (期望 ~50 Hz):"
timeout 3 rostopic hz /unity/joint_states 2>/dev/null || echo "  话题不存在或无数据"
echo ""
echo "检查 /aubo_e5/joint_states (期望 ~50 Hz):"
timeout 3 rostopic hz /aubo_e5/joint_states 2>/dev/null || echo "  话题不存在或无数据"
echo ""
echo "检查 /real/joint_states (期望 ~125 Hz):"
timeout 3 rostopic hz /real/joint_states 2>/dev/null || echo "  话题不存在或无数据"
echo ""

# 3. Shadow 模式数据流检查
echo -e "${BLUE}=== 3. Shadow 模式数据流检查 ===${NC}"
echo ""
echo "检查 /unity/joint_command (Unity 接收的指令):"
timeout 2 rostopic echo -n 1 /unity/joint_command 2>/dev/null || echo "  话题不存在或无数据"
echo ""

# 4. 延迟测量
echo -e "${BLUE}=== 4. 延迟测量 ===${NC}"
echo ""
echo "实机 → Unity 延迟估算:"
echo "  (通过时间戳对比)"
echo ""

# 获取实机关节状态时间戳
REAL_TIME=$(timeout 1 rostopic echo -n 1 /real/joint_states/header/stamp 2>/dev/null | grep -A 1 "secs:" | tail -1 | awk '{print $2}')
sleep 0.1
# 获取 Unity 关节状态时间戳
UNITY_TIME=$(timeout 1 rostopic echo -n 1 /unity/joint_states/header/stamp 2>/dev/null | grep -A 1 "secs:" | tail -1 | awk '{print $2}')

if [ -n "$REAL_TIME" ] && [ -n "$UNITY_TIME" ]; then
    DELAY=$((UNITY_TIME - REAL_TIME))
    if [ $DELAY -lt 0 ]; then
        DELAY=$((DELAY * -1))
    fi

    if [ $DELAY -lt 20 ]; then
        echo -e "  延迟: ${GREEN}${DELAY} ms (良好)${NC}"
    elif [ $DELAY -lt 50 ]; then
        echo -e "  延迟: ${YELLOW}${DELAY} ms (可接受)${NC}"
    else
        echo -e "  延迟: ${RED}${DELAY} ms (需要优化)${NC}"
    fi
else
    echo "  无法测量延迟（数据不足）"
fi
echo ""

# 5. TF 检查
echo -e "${BLUE}=== 5. TF 变换检查 ===${NC}"
echo ""
echo "检查 base_link → wrist3_Link 变换:"
timeout 2 rosrun tf tf_echo base_link wrist3_Link 2>/dev/null | head -5 || echo "  TF 不可用"
echo ""

# 6. 动作服务器状态
echo -e "${BLUE}=== 6. 动作服务器状态 ===${NC}"
echo ""
echo "检查 follow_joint_trajectory 动作服务器:"
timeout 2 rostopic echo -n 1 /aubo_e5_controller/follow_joint_trajectory/status 2>/dev/null || echo "  动作服务器不可用"
echo ""

# 7. 实时监控模式
echo -e "${BLUE}=== 7. 实时监控模式 ===${NC}"
echo ""
echo "按 Ctrl+C 退出实时监控"
echo ""
echo "监控内容:"
echo "  - 实机关节位置"
echo "  - Unity 关节位置"
echo "  - 位置误差"
echo ""

# 实时监控循环
while true; do
    # 获取实机关节位置
    REAL_POS=$(timeout 1 rostopic echo -n 1 /real/joint_states/position 2>/dev/null | tr -d '[]' | tr ',' ' ')

    # 获取 Unity 关节位置
    UNITY_POS=$(timeout 1 rostopic echo -n 1 /unity/joint_states/position 2>/dev/null | tr -d '[]' | tr ',' ' ')

    if [ -n "$REAL_POS" ] && [ -n "$UNITY_POS" ]; then
        echo -e "${GREEN}[$(date +%H:%M:%S)]${NC}"
        echo "  实机: $REAL_POS"
        echo "  Unity: $UNITY_POS"

        # 计算误差（简化版）
        REAL_ARRAY=($REAL_POS)
        UNITY_ARRAY=($UNITY_POS)

        if [ ${#REAL_ARRAY[@]} -eq 6 ] && [ ${#UNITY_ARRAY[@]} -eq 6 ]; then
            MAX_ERROR=0
            for i in {0..5}; do
                ERROR=$(echo "${REAL_ARRAY[$i]} - ${UNITY_ARRAY[$i]}" | bc -l | sed 's/-//')
                if (( $(echo "$ERROR > $MAX_ERROR" | bc -l) )); then
                    MAX_ERROR=$ERROR
                fi
            done

            # 转换为度
            MAX_ERROR_DEG=$(echo "$MAX_ERROR * 57.2958" | bc -l)

            if (( $(echo "$MAX_ERROR_DEG < 1.0" | bc -l) )); then
                echo -e "  最大误差: ${GREEN}${MAX_ERROR_DEG:0:5}° (优秀)${NC}"
            elif (( $(echo "$MAX_ERROR_DEG < 5.0" | bc -l) )); then
                echo -e "  最大误差: ${YELLOW}${MAX_ERROR_DEG:0:5}° (可接受)${NC}"
            else
                echo -e "  最大误差: ${RED}${MAX_ERROR_DEG:0:5}° (需要优化)${NC}"
            fi
        fi
    else
        echo -e "${RED}[$(date +%H:%M:%S)] 无数据${NC}"
    fi

    echo ""
    sleep 1
done
