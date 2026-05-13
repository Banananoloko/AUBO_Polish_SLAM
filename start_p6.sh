#!/bin/bash
# P6 Shadow 模式统一启动脚本
# 集成环境检查、启动、测试坐标

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 显示帮助
show_help() {
    echo "=========================================="
    echo "  P6 Shadow 模式启动脚本"
    echo "=========================================="
    echo ""
    echo "用法: ./start_p6.sh [选项]"
    echo ""
    echo "选项:"
    echo "  --check     - 仅检查环境，不启动"
    echo "  --coords    - 显示测试坐标"
    echo "  --help      - 显示此帮助信息"
    echo ""
    echo "默认行为: 检查环境 → 启动 P6 Shadow 模式"
    echo ""
}

# 环境检查
check_environment() {
    echo -e "${BLUE}=== 环境检查 ===${NC}"
    echo ""

    # 1. ROS 环境
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${RED}[✗] ROS 环境未加载${NC}"
        echo "请运行: source devel/setup.bash"
        exit 1
    fi
    echo -e "${GREEN}[✓] ROS 版本: $ROS_DISTRO${NC}"

    # 2. 工作空间
    if [ ! -d "$SCRIPT_DIR/devel" ]; then
        echo -e "${RED}[✗] 工作空间未编译${NC}"
        exit 1
    fi
    echo -e "${GREEN}[✓] 工作空间已编译${NC}"

    # 3. 核心包
    if ! rospack find aubo_linked_execution &>/dev/null; then
        echo -e "${RED}[✗] aubo_linked_execution 包未找到${NC}"
        exit 1
    fi
    echo -e "${GREEN}[✓] aubo_linked_execution 包已安装${NC}"

    # 4. Unity 桥接包
    if ! rospack find aubo_unity_bridge &>/dev/null; then
        echo -e "${YELLOW}[!] aubo_unity_bridge 包未找到${NC}"
    else
        echo -e "${GREEN}[✓] aubo_unity_bridge 包已安装${NC}"
    fi

    echo ""
}

# 显示测试坐标
show_coordinates() {
    echo "=========================================="
    echo "  P6 测试坐标"
    echo "=========================================="
    echo ""
    echo "【基础测试点】"
    echo "  Home:   [0, 0, 0, 0, 0, 0]"
    echo "  Point1: [0.5, -0.3, 0.2, 0, 0, 0]"
    echo "  Point2: [-0.5, 0.3, -0.2, 0, 0, 0]"
    echo ""
    echo "【安全测试点】（小幅度运动）"
    echo "  Safe1:  [0.1, 0.1, 0.1, 0, 0, 0]"
    echo "  Safe2:  [-0.1, -0.1, -0.1, 0, 0, 0]"
    echo ""
    echo "【工作空间边界】"
    echo "  Max:    [1.5, 1.0, 1.0, 1.57, 1.57, 3.14]"
    echo "  Min:    [-1.5, -1.0, -1.0, -1.57, -1.57, -3.14]"
    echo ""
    echo "使用方法:"
    echo "  rosrun aubo_planner goto_pose_enhanced.py"
    echo "  然后输入坐标，例如: 0.5 -0.3 0.2 0 0 0"
    echo ""
}

# 启动 P6 Shadow 模式
start_p6() {
    echo "=========================================="
    echo "  启动 P6 Shadow 模式"
    echo "=========================================="
    echo ""
    echo "实机 IP: 192.168.10.230"
    echo "模式: Unity Shadow (实机镜像到 Unity)"
    echo ""
    echo "按 Ctrl+C 停止"
    echo ""

    cd "$SCRIPT_DIR"
    source devel/setup.bash

    roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
        robot_ip:=192.168.10.230 \
        use_unity:=true
}

# 主程序
case "${1:-}" in
    --check)
        check_environment
        echo -e "${GREEN}环境检查通过！${NC}"
        echo ""
        echo "启动命令: ./start_p6.sh"
        ;;
    --coords)
        show_coordinates
        ;;
    --help|-h)
        show_help
        ;;
    "")
        check_environment
        start_p6
        ;;
    *)
        echo -e "${RED}错误: 未知选项 '$1'${NC}"
        echo ""
        show_help
        exit 1
        ;;
esac
