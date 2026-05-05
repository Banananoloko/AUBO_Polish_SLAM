#!/bin/bash
# AUBO E5 系统管理工具
# 集成验证、诊断、修复功能

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 检查函数
check_pass() { echo -e "${GREEN}[✓]${NC} $1"; }
check_fail() { echo -e "${RED}[✗]${NC} $1"; }
check_warn() { echo -e "${YELLOW}[!]${NC} $1"; }
check_info() { echo -e "${BLUE}[i]${NC} $1"; }

# 显示帮助
show_help() {
    echo "AUBO E5 系统管理工具"
    echo ""
    echo "用法: ./system_tools.sh <命令>"
    echo ""
    echo "命令:"
    echo "  verify      - 验证系统完整性（环境、包、权限、依赖）"
    echo "  diagnose    - 诊断运行时状态（节点、话题、参数）"
    echo "  fix-rpath   - 修复 AUBO SDK 动态库依赖问题"
    echo "  help        - 显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  ./system_tools.sh verify"
    echo "  ./system_tools.sh diagnose"
    echo ""
}

# ============================================================
# 功能 1: 验证系统完整性
# ============================================================
verify_system() {
    echo "=========================================="
    echo "  AUBO E5 系统验证"
    echo "=========================================="
    echo ""

    # 1. 检查 ROS 环境
    echo "1. 检查 ROS 环境..."
    if [ -z "$ROS_DISTRO" ]; then
        check_fail "ROS 环境未加载，请运行: source devel/setup.bash"
        exit 1
    else
        check_pass "ROS 版本: $ROS_DISTRO"
    fi

    # 2. 检查工作空间
    echo ""
    echo "2. 检查工作空间..."
    if [ ! -d "$SCRIPT_DIR/devel" ] || [ ! -d "$SCRIPT_DIR/build" ]; then
        check_fail "工作空间未编译，请运行: catkin_make"
        exit 1
    else
        check_pass "工作空间已编译"
    fi

    # 3. 检查核心包
    echo ""
    echo "3. 检查核心包..."
    REQUIRED_PACKAGES=(
        "aubo_linked_execution"
        "aubo_driver"
        "aubo_controller"
        "aubo_gazebo"
        "aubo_e5_moveit_config"
        "aubo_msgs"
    )

    for pkg in "${REQUIRED_PACKAGES[@]}"; do
        if rospack find "$pkg" &> /dev/null; then
            check_pass "$pkg"
        else
            check_fail "$pkg 未找到"
            exit 1
        fi
    done

    # 4. 检查 Python 脚本权限
    echo ""
    echo "4. 检查 Python 脚本权限..."
    SCRIPTS=(
        "src/aubo_linked_execution/scripts/joint_state_mirror_adapter.py"
        "src/aubo_linked_execution/scripts/linked_execution_action_server.py"
        "src/aubo_linked_execution/scripts/linked_execution_monitor.py"
        "src/aubo_linked_execution/scripts/aubo_robot_startup.py"
    )

    for script in "${SCRIPTS[@]}"; do
        if [ -x "$SCRIPT_DIR/$script" ]; then
            check_pass "$(basename $script)"
        else
            check_warn "$(basename $script) 不可执行，正在修复..."
            chmod +x "$SCRIPT_DIR/$script"
            check_pass "已修复: $(basename $script)"
        fi
    done

    # 5. 检查 AUBO SDK 库
    echo ""
    echo "5. 检查 AUBO SDK 库..."
    DRIVER_BIN="$SCRIPT_DIR/devel/lib/aubo_driver/aubo_driver"
    if [ -f "$DRIVER_BIN" ]; then
        if ldd "$DRIVER_BIN" 2>&1 | grep -q "libauborobotcontroller.so"; then
            check_pass "AUBO SDK 库已链接"
            if ldd "$DRIVER_BIN" 2>&1 | grep -q "not found"; then
                check_warn "AUBO SDK 库依赖缺失，运行: ./system_tools.sh fix-rpath"
            else
                check_pass "AUBO SDK 库依赖完整"
            fi
        else
            check_warn "AUBO SDK 库未链接（仅使用仿真模式可忽略）"
        fi
    else
        check_warn "aubo_driver 未编译"
    fi

    # 6. 检查 MoveIt 配置
    echo ""
    echo "6. 检查 MoveIt 配置..."
    MOVEIT_CONFIGS=(
        "src/aubo_robot/aubo_e5_moveit_config/config/kinematics.yaml"
        "src/aubo_robot/aubo_e5_moveit_config/config/joint_limits.yaml"
        "src/aubo_linked_execution/config/linked_execution_controllers.yaml"
    )

    for config in "${MOVEIT_CONFIGS[@]}"; do
        if [ -f "$SCRIPT_DIR/$config" ]; then
            check_pass "$(basename $config)"
        else
            check_fail "$(basename $config) 缺失"
            exit 1
        fi
    done

    # 7. 检查网络
    echo ""
    echo "7. 检查机器人网络连接..."
    read -p "输入机器人 IP 地址（留空跳过）: " ROBOT_IP

    if [ -n "$ROBOT_IP" ]; then
        if ping -c 1 -W 2 "$ROBOT_IP" &> /dev/null; then
            check_pass "机器人 $ROBOT_IP 可达"
        else
            check_warn "机器人 $ROBOT_IP 无法 ping 通（请检查网络连接）"
        fi
    else
        check_warn "跳过网络检查"
    fi

    # 8. 生成启动命令
    echo ""
    echo "=========================================="
    echo "  验证完成！"
    echo "=========================================="
    echo ""
    echo "推荐启动命令："
    echo ""
    echo "【联动模式】（实机 + Gazebo）"
    if [ -n "$ROBOT_IP" ]; then
        echo "  roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=$ROBOT_IP"
    else
        echo "  roslaunch aubo_linked_execution aubo_e5_linked_execution.launch robot_ip:=192.168.1.10"
    fi
    echo ""
    echo "【仿真模式】（仅 Gazebo）"
    echo "  roslaunch aubo_linked_execution aubo_e5_linked_execution.launch sim_only:=true"
    echo ""
}

# ============================================================
# 功能 2: 诊断运行时状态
# ============================================================
diagnose_system() {
    echo "=========================================="
    echo "  AUBO E5 运行时诊断"
    echo "=========================================="
    echo ""

    # 检查 ROS Master
    if ! rostopic list &> /dev/null; then
        check_fail "ROS Master 未运行"
        echo ""
        echo "请先启动系统："
        echo "  roslaunch aubo_linked_execution aubo_e5_linked_execution.launch"
        exit 1
    fi

    check_pass "ROS Master 正在运行"
    echo ""

    # 1. 检查关键节点
    echo "1. 关键节点状态"
    echo "----------------------------------------"
    NODES=(
        "aubo_driver"
        "aubo_joint_trajectory_action"
        "aubo_robot_simulator"
        "linked_execution_action_server"
        "joint_state_mirror_adapter"
        "linked_execution_monitor"
        "aubo_gazebo_driver"
        "move_group"
    )

    for node in "${NODES[@]}"; do
        if rosnode list 2>/dev/null | grep -q "$node"; then
            check_pass "$node"
        else
            check_warn "$node 未运行"
        fi
    done

    # 2. 检查关键话题
    echo ""
    echo "2. 关键话题发布频率"
    echo "----------------------------------------"
    TOPICS=(
        "/joint_states"
        "/feedback_states"
        "/robot_status"
        "/real/joint_states"
        "/aubo_e5/joint_states"
    )

    for topic in "${TOPICS[@]}"; do
        if rostopic list 2>/dev/null | grep -q "^${topic}$"; then
            hz=$(timeout 2 rostopic hz "$topic" 2>/dev/null | grep "average rate" | awk '{print $3}')
            if [ -n "$hz" ]; then
                check_pass "$topic (${hz} Hz)"
            else
                check_warn "$topic (无数据)"
            fi
        else
            check_warn "$topic 不存在"
        fi
    done

    # 3. 检查 Action Servers
    echo ""
    echo "3. Action Servers"
    echo "----------------------------------------"
    ACTION_SERVERS=(
        "/linked_execution_controller/follow_joint_trajectory"
        "/aubo_e5_controller/follow_joint_trajectory"
    )

    for server in "${ACTION_SERVERS[@]}"; do
        if rostopic list 2>/dev/null | grep -q "${server}/goal"; then
            check_pass "$server"
        else
            check_warn "$server 未就绪"
        fi
    done

    # 4. 检查关键参数
    echo ""
    echo "4. 关键参数"
    echo "----------------------------------------"
    robot_name=$(rosparam get /robot_name 2>/dev/null || echo "NOT SET")
    check_info "/robot_name = $robot_name"

    robot_connected=$(rosparam get /aubo_driver/robot_connected 2>/dev/null || echo "NOT SET")
    if [ "$robot_connected" = "1" ]; then
        check_pass "/aubo_driver/robot_connected = 1"
    else
        check_warn "/aubo_driver/robot_connected = $robot_connected"
    fi

    # 5. 检查 TF 树
    echo ""
    echo "5. TF 树状态"
    echo "----------------------------------------"
    if rosrun tf tf_echo base_link tool0 &> /dev/null; then
        check_pass "TF 树完整 (base_link → tool0)"
    else
        check_warn "TF 树不完整或未发布"
    fi

    echo ""
    echo "=========================================="
    echo "  诊断完成"
    echo "=========================================="
    echo ""
    echo "详细信息："
    echo "  查看节点图: rqt_graph"
    echo "  查看 TF 树: rosrun tf view_frames && evince frames.pdf"
    echo "  监控话题: rostopic echo <topic_name>"
    echo ""
}

# ============================================================
# 功能 3: 修复 AUBO SDK RPATH
# ============================================================
fix_rpath() {
    echo "=========================================="
    echo "  修复 AUBO SDK 动态库依赖"
    echo "=========================================="
    echo ""

    # 检查 patchelf
    if ! command -v patchelf &> /dev/null; then
        check_fail "patchelf 未安装"
        echo ""
        echo "请运行: sudo apt install patchelf"
        exit 1
    fi

    LIB_DIR="$SCRIPT_DIR/src/aubo_robot/aubo_driver/lib/lib64"
    AUBOCTL_DIR="$LIB_DIR/aubocontroller"
    FULL_RPATH="$SCRIPT_DIR/src/aubo_robot/opt/lib:$LIB_DIR:$AUBOCTL_DIR:$LIB_DIR/config:$LIB_DIR/log4cplus"

    check_info "目标目录: $LIB_DIR"
    check_info "RPATH: $FULL_RPATH"
    echo ""

    # 修复所有 .so 文件
    find "$LIB_DIR" -name "*.so*" -type f 2>/dev/null | while read -r lib; do
        echo "处理: $(basename $lib)"
        patchelf --set-rpath "$FULL_RPATH" "$lib" 2>/dev/null || true
    done

    check_pass "RPATH 修复完成"
    echo ""
    echo "请重新编译工作空间:"
    echo "  cd $SCRIPT_DIR"
    echo "  catkin_make"
    echo ""
}

# ============================================================
# 主程序
# ============================================================
case "${1:-}" in
    verify)
        verify_system
        ;;
    diagnose)
        diagnose_system
        ;;
    fix-rpath)
        fix_rpath
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        echo "错误: 未知命令 '${1:-}'"
        echo ""
        show_help
        exit 1
        ;;
esac
