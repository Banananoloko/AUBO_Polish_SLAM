#!/bin/bash
# AUBO E5 正方形轨迹演示 — 一键启动脚本（扩展版）
# 用法: ./run_square_demo.sh              # 仅仿真模式
#       ./run_square_demo.sh --real <ip>  # 实机+Gazebo镜像模式
#       ./run_square_demo.sh --unity      # 使用 Unity 后端
#
# 功能: 自动启动 ROS 系统 + 图形 TUI 弹窗
#   输入和退出均由 square_demo_gui.py 负责。

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
AUTO_START_UI=true
UI_MODE="gui"
USE_UNITY=false
ROSLAUNCH_PID=""
MENU_ONLY=false
MENU_LOG=""
SYSTEM_LOG_FILE=""

# ---- 清理函数 ----
cleanup() {
    [ -z "${ROSLAUNCH_PID:-}" ] && return
    echo -e "${YELLOW}[CLEANUP] 关闭所有进程...${NC}"
    kill "$ROSLAUNCH_PID" 2>/dev/null
    wait "$ROSLAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

prepare_system_log() {
    local log_dir="$SCRIPT_DIR/logs"
    mkdir -p "$log_dir"
    SYSTEM_LOG_FILE="$log_dir/square_demo_system_$(date +%Y%m%d_%H%M%S).log"
    export AUBO_TUI_SYSTEM_LOG="$SYSTEM_LOG_FILE"
    echo -e "${CYAN}[LOG] ROS/RViz/MoveIt 启动日志: $SYSTEM_LOG_FILE${NC}"
}

run_control_action() {
    if ! rosrun aubo_linked_execution square_demo_control.py --action "$@"; then
        echo -e "${YELLOW}[WARN] 动作执行进程异常退出${NC}"
    fi
}

print_terminal_menu() {
    echo ""
    echo -e "${CYAN}${BOLD}========= 快捷菜单 =========${NC}"
    echo -e "${CYAN}[1]${NC} 执行正方形轨迹"
    echo -e "${CYAN}[2]${NC} 输入自定义目标位姿"
    echo -e "${CYAN}[3]${NC} 多路径点连续轨迹"
    echo -e "${CYAN}[4]${NC} 安全审查状态"
    echo -e "${CYAN}[5]${NC} 预设工件打磨测试"
    echo -e "${CYAN}[6]${NC} 轨迹生成测试"
    echo -e "${CYAN}[7]${NC} 介绍 README"
    echo -e "${CYAN}[q]${NC} 退出"
}

run_terminal_menu() {
    while true; do
        print_terminal_menu
        printf "> "
        read -r choice || break

        case "$choice" in
            1|4|5|6|7)
                run_control_action "$choice"
                ;;
            2)
                echo -e "${CYAN}[INPUT] 输入目标位姿:${NC}"
                echo -e "${CYAN}[INPUT]   仅位置:      x y z${NC}"
                echo -e "${CYAN}[INPUT]   位置 + RPY:  x y z roll pitch yaw (度)${NC}"
                printf "> "
                read -r pose_text || continue
                [ -z "$pose_text" ] && continue
                run_control_action 2 --pose "$pose_text"
                ;;
            3)
                echo -e "${CYAN}[INPUT] 输入路径点，空行或 done 结束:${NC}"
                echo -e "${CYAN}[INPUT]   每行: x y z  （连续模式锁定当前 RPY）${NC}"
                waypoint_args=()
                while true; do
                    printf "  wp> "
                    read -r wp_text || break
                    [ -z "$wp_text" ] && break
                    [ "$wp_text" = "done" ] && break
                    waypoint_args+=(--waypoint "$wp_text")
                done
                if [ ${#waypoint_args[@]} -eq 0 ]; then
                    echo -e "${YELLOW}[WARN] 未输入路径点${NC}"
                    continue
                fi
                echo -e "${CYAN}[INPUT] 循环次数 [默认 1]:${NC}"
                printf "> "
                read -r loops
                loops="${loops:-1}"
                if ! [[ "$loops" =~ ^[0-9]+$ ]]; then
                    loops=1
                fi
                run_control_action 3 "${waypoint_args[@]}" --loops "$loops"
                ;;
            q|Q)
                echo -e "${GREEN}[INFO] 退出${NC}"
                break
                ;;
            "")
                ;;
            *)
                echo -e "${YELLOW}[WARN] 未知选项: $choice${NC}"
                ;;
        esac
    done
}

run_logged_terminal_menu() {
    local log_file="$1"
    mkdir -p "$(dirname "$log_file")"
    echo -e "${CYAN}[LOG] 快捷菜单日志: $log_file${NC}"
    run_terminal_menu 2>&1 | tee -a "$log_file"
}

launch_terminal_menu_window() {
    local log_dir="$SCRIPT_DIR/logs"
    local log_file="$log_dir/square_demo_menu_$(date +%Y%m%d_%H%M%S).log"
    local marker="/tmp/aubo_square_demo_menu_$$.running"
    local cmd=""

    mkdir -p "$log_dir"
    echo "pending" > "$marker"

    printf -v cmd \
        'echo $$ > %q; cd %q && source %q && if command -v script >/dev/null 2>&1; then script -q -f %q -c %q; else %q --menu-only --menu-log %q; fi; status=$?; rm -f %q; echo; echo "[INFO] 快捷菜单已退出，日志: %s"; read -r -p "按 Enter 关闭窗口..." _; exit $status' \
        "$marker" \
        "$SCRIPT_DIR" \
        "$SCRIPT_DIR/devel/setup.bash" \
        "$log_file" \
        "$SCRIPT_DIR/run_square_demo.sh --menu-only" \
        "$SCRIPT_DIR/run_square_demo.sh" \
        "$log_file" \
        "$marker" \
        "$log_file"

    if command -v gnome-terminal &>/dev/null; then
        gnome-terminal --title="AUBO E5 快捷菜单" -- bash -lc "$cmd" &
    elif command -v x-terminal-emulator &>/dev/null; then
        x-terminal-emulator -T "AUBO E5 快捷菜单" -e bash -lc "$cmd" &
    elif command -v xterm &>/dev/null; then
        xterm -T "AUBO E5 快捷菜单" -e bash -lc "$cmd" &
    elif command -v konsole &>/dev/null; then
        konsole --new-tab -p tabtitle="AUBO E5 快捷菜单" -e bash -lc "$cmd" &
    else
        rm -f "$marker"
        echo -e "${YELLOW}[WARN] 未找到可用图形终端，回退到当前终端。${NC}"
        run_logged_terminal_menu "$log_file"
        return
    fi

    echo -e "${GREEN}[START] 快捷菜单已在新终端打开${NC}"
    echo -e "${CYAN}[LOG] 菜单日志: $log_file${NC}"

    local wait_start
    wait_start=$(date +%s)
    while [ -e "$marker" ]; do
        local menu_pid
        menu_pid="$(cat "$marker" 2>/dev/null || true)"
        if [[ "$menu_pid" =~ ^[0-9]+$ ]] && ! kill -0 "$menu_pid" 2>/dev/null; then
            rm -f "$marker"
            break
        fi
        if [ "$menu_pid" = "pending" ] && [ $(( $(date +%s) - wait_start )) -gt 8 ]; then
            echo -e "${RED}[ERROR] 新终端未能启动快捷菜单${NC}"
            rm -f "$marker"
            return 1
        fi
        if [ -n "${ROSLAUNCH_PID:-}" ] && ! kill -0 "$ROSLAUNCH_PID" 2>/dev/null; then
            echo -e "${RED}[ERROR] ROS 系统已退出${NC}"
            rm -f "$marker"
            return 1
        fi
        sleep 1
    done
}

launch_interaction() {
    if [ "$UI_MODE" = "gui" ]; then
        rosrun aubo_linked_execution square_demo_gui.py
    else
        echo -e "${YELLOW}[WARN] 当前默认入口是图形 TUI；终端菜单仅保留为调试入口。${NC}"
        launch_terminal_menu_window
    fi
}

# ---- 解析参数 ----
while [[ $# -gt 0 ]]; do
    case "$1" in
        --real)
            MODE="real"
            ROBOT_IP="$2"
            shift 2
            ;;
        --no-ui)
            AUTO_START_UI=false
            shift
            ;;
        --no-monitor)
            # 保留兼容旧命令；本脚本不再自动启动监控窗口。
            shift
            ;;
        --menu-only)
            MENU_ONLY=true
            shift
            ;;
        --menu-log)
            MENU_LOG="$2"
            shift 2
            ;;
        --gui)
            UI_MODE="gui"
            shift
            ;;
        --unity)
            USE_UNITY=true
            shift
            ;;
        --tui)
            # TUI 指图形 tkinter 控制界面；终端输入菜单暂不作为默认入口。
            UI_MODE="gui"
            shift
            ;;
        --terminal-menu)
            # 调试入口：保留旧终端菜单，但默认不会启动。
            UI_MODE="terminal"
            shift
            ;;
        --help|-h)
            echo "AUBO E5 正方形轨迹演示 — 一键启动（扩展版）"
            echo ""
            echo "用法: ./run_square_demo.sh [选项]"
            echo ""
            echo "选项:"
            echo "  --real <ip>    实机+Gazebo镜像模式 (默认: 仅仿真)"
            echo "  --no-ui        仅启动 ROS 系统，不自动启动交互界面"
            echo "  --no-monitor   兼容旧参数；当前脚本默认不启动监控窗口"
            echo "  --gui          启动图形 TUI (默认)"
            echo "  --unity        使用 Unity 后端；默认使用 Gazebo"
            echo "  --tui          兼容旧参数；等价于 --gui"
            echo "  --terminal-menu 调试入口：启动旧终端菜单"
            echo "  --help         显示此帮助"
            echo ""
            echo "图形 TUI 功能:"
            echo "  [1] 执行正方形轨迹 (20cm × 20cm, YZ 平面)"
            echo "  [2] 输入自定义目标位姿 (x y z [roll pitch yaw])"
            echo "  [3] 多路径点连续轨迹"
            echo "  [4] 安全审查状态"
            echo "  [5] 预设工件打磨测试 (3点笛卡尔轨迹)"
            echo "  [6] 轨迹生成测试"
            echo "  [7] 介绍 (README)"
            echo "  [退出系统] 关闭 TUI 并触发脚本清理 ROS 系统"
            echo ""
            echo "手动启动界面:"
            echo "  rosrun aubo_linked_execution square_demo_gui.py (图形 TUI)"
            exit 0
            ;;
        *)
            echo -e "${RED}[ERROR] 未知参数: $1${NC}"
            exit 1
            ;;
    esac
done

if [ "$MENU_ONLY" = true ]; then
    if [ -f "$SCRIPT_DIR/devel/setup.bash" ]; then
        source "$SCRIPT_DIR/devel/setup.bash"
    fi
    if [ -n "$MENU_LOG" ]; then
        run_logged_terminal_menu "$MENU_LOG"
    else
        run_terminal_menu
    fi
    exit 0
fi

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
    if [ "$USE_UNITY" = true ]; then
        echo -e "${YELLOW}  [WARN] aubo_gazebo 包未找到；Unity 后端可继续${NC}"
    else
        echo -e "${RED}[FAIL] aubo_gazebo 包未找到${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}  [OK] aubo_gazebo 包${NC}"
fi

if [ "$USE_UNITY" = true ]; then
    if ! rospack find aubo_unity_bridge &>/dev/null; then
        echo -e "${RED}[FAIL] aubo_unity_bridge 包未找到${NC}"
        exit 1
    fi
    echo -e "${GREEN}  [OK] aubo_unity_bridge 包${NC}"
fi

BACKEND_NAME="Gazebo"
if [ "$USE_UNITY" = true ]; then
    BACKEND_NAME="Unity"
fi

# ---- 启动模式 ----
echo ""
if [ "$MODE" = "real" ]; then
    echo -e "${YELLOW}[MODE] 实机 + ${BACKEND_NAME} 镜像模式${NC}"
    echo -e "${YELLOW}       机器人 IP: $ROBOT_IP${NC}"
    echo -e "${RED}  !! 请确保机器人周围无障碍物，急停已释放 !!${NC}"
    echo ""

    if [ "$AUTO_START_UI" = true ]; then
        echo -e "${CYAN}[INFO] 启动 ROS 系统 + 图形 TUI...${NC}"
        echo -e "${CYAN}       (使用 --no-ui 可仅启动 ROS 系统)${NC}"
        echo ""

        # 后台启动 roslaunch；输出写入日志，由图形 TUI 从文件头开始播放。
        prepare_system_log
        roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
            robot_ip:="$ROBOT_IP" \
            use_unity:="$USE_UNITY" >> "$SYSTEM_LOG_FILE" 2>&1 &
        ROSLAUNCH_PID=$!

        # 等待 ROS 系统就绪 (约 8-10 秒)
        echo -e "${CYAN}[WAIT] 等待 ROS 系统启动...${NC}"
        sleep 10

        # 检查 roslaunch 是否还在运行
        if ! kill -0 $ROSLAUNCH_PID 2>/dev/null; then
            echo -e "${RED}[ERROR] ROS 系统启动失败${NC}"
            exit 1
        fi

        # 启动交互入口
        echo -e "${GREEN}[START] 启动交互入口...${NC}"
        echo ""
        launch_interaction
    else
        roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
            robot_ip:="$ROBOT_IP" \
            use_unity:="$USE_UNITY"
    fi
else
    echo -e "${GREEN}[MODE] 仅仿真模式 (${BACKEND_NAME})${NC}"
    echo ""

    if [ "$AUTO_START_UI" = true ]; then
        echo -e "${CYAN}[INFO] 启动 ROS 系统 + 图形 TUI...${NC}"
        echo -e "${CYAN}       (使用 --no-ui 可仅启动 ROS 系统)${NC}"
        echo ""

        # 后台启动 roslaunch；输出写入日志，由图形 TUI 从文件头开始播放。
        prepare_system_log
        roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
            sim_only:=true \
            use_unity:="$USE_UNITY" >> "$SYSTEM_LOG_FILE" 2>&1 &
        ROSLAUNCH_PID=$!

        # 等待 ROS 系统就绪 (约 8-10 秒)
        echo -e "${CYAN}[WAIT] 等待 ROS 系统启动...${NC}"
        sleep 10

        # 检查 roslaunch 是否还在运行
        if ! kill -0 $ROSLAUNCH_PID 2>/dev/null; then
            echo -e "${RED}[ERROR] ROS 系统启动失败${NC}"
            exit 1
        fi

        # 启动交互入口
        echo -e "${GREEN}[START] 启动交互入口...${NC}"
        echo ""
        launch_interaction
    else
        roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \
            sim_only:=true \
            use_unity:="$USE_UNITY"
    fi
fi
