#!/bin/bash
# AUBO E5 正方形轨迹演示 — 一键启动脚本
# 用法: ./run_square_demo.sh              # 仅仿真模式
#       ./run_square_demo.sh --real <ip>  # 实机+Gazebo镜像模式

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="sim"
ROBOT_IP=""

# 解析参数
while [[ $# -gt 0 ]]; do
    case "$1" in
        --real)
            MODE="real"
            ROBOT_IP="$2"
            shift 2
            ;;
        --help|-h)
            echo "AUBO E5 正方形轨迹演示 — 一键启动"
            echo ""
            echo "用法: ./run_square_demo.sh [选项]"
            echo ""
            echo "选项:"
            echo "  --real <ip>   实机+Gazebo镜像模式 (默认: 仅仿真)"
            echo "  --help        显示此帮助"
            echo ""
            echo "启动后请在另一终端运行:"
            echo "  rosrun aubo_linked_execution square_demo_control.py"
            exit 0
            ;;
        *)
            echo -e "${RED}[ERROR] 未知参数: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${CYAN}${BOLD}============================================${NC}"
echo -e "${CYAN}${BOLD}  AUBO E5 正方形轨迹演示 — 虚实同步控制${NC}"
echo -e "${CYAN}${BOLD}============================================${NC}"
echo ""

# ---- 环境检查 ----
echo -e "${CYAN}[CHECK] 检查环境...${NC}"

if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}[FAIL] ROS 环境未加载${NC}"
    echo "请先运行: source /opt/ros/noetic/setup.bash"
    exit 1
fi
echo -e "${GREEN}  [OK] ROS $ROS_DISTRO${NC}"

if [ ! -d "$SCRIPT_DIR/devel" ]; then
    echo -e "${RED}[FAIL] 工作空间未编译 (devel/ 不存在)${NC}"
    exit 1
fi
echo -e "${GREEN}  [OK] 工作空间已编译${NC}"

source "$SCRIPT_DIR/devel/setup.bash"

if ! rospack find aubo_linked_execution &>/dev/null; then
    echo -e "${RED}[FAIL] aubo_linked_execution 包未找到${NC}"
    exit 1
fi
echo -e "${GREEN}  [OK] aubo_linked_execution 包${NC}"

if ! rospack find aubo_gazebo &>/dev/null; then
    echo -e "${RED}[FAIL] aubo_gazebo 包未找到${NC}"
    exit 1
fi
echo -e "${GREEN}  [OK] aubo_gazebo 包${NC}"

# ---- 启动模式 ----
echo ""
if [ "$MODE" = "real" ]; then
    echo -e "${YELLOW}[MODE] 实机 + Gazebo 镜像模式${NC}"
    echo -e "${YELLOW}       机器人 IP: $ROBOT_IP${NC}"
    echo -e "${RED}  !! 请确保机器人周围无障碍物，急停已释放 !!${NC}"

    roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
        robot_ip:="$ROBOT_IP"
else
    echo -e "${GREEN}[MODE] 仅仿真模式 (Gazebo)${NC}"

    roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
        sim_only:=true
fi
