#!/bin/bash
# AUBO E5 示教-回放系统快速启动脚本

CYAN='\033[96m'
GREEN='\033[92m'
YELLOW='\033[93m'
RED='\033[91m'
RESET='\033[0m'
BOLD='\033[1m'

CONFIG_DIR="$HOME/aubo_polish/waypoints_config"

echo -e "${CYAN}${BOLD}================================${RESET}"
echo -e "${CYAN}${BOLD}  AUBO E5 示教-回放系统${RESET}"
echo -e "${CYAN}${BOLD}================================${RESET}\n"

# 检查配置目录
if [ ! -d "$CONFIG_DIR" ]; then
    mkdir -p "$CONFIG_DIR"
    echo -e "${GREEN}✓ 已创建配置目录: $CONFIG_DIR${RESET}\n"
fi

# 主菜单
echo -e "${YELLOW}请选择模式：${RESET}"
echo "  1) 示教模式 - 记录路径点"
echo "  2) 回放模式 - 执行路径点"
echo "  3) 查看配置文件"
echo "  4) 退出"
echo ""
read -p "请输入选项 [1-4]: " choice

case $choice in
    1)
        echo -e "\n${CYAN}启动示教模式...${RESET}"
        rosrun aubo_linked_execution teach_waypoints.py
        ;;
    2)
        # 列出可用配置
        echo -e "\n${CYAN}可用的配置文件：${RESET}"
        configs=($(ls -1 "$CONFIG_DIR"/*.yaml 2>/dev/null | xargs -n 1 basename | sed 's/.yaml//'))

        if [ ${#configs[@]} -eq 0 ]; then
            echo -e "${RED}✗ 没有找到配置文件${RESET}"
            echo -e "${YELLOW}请先使用示教模式创建配置${RESET}"
            exit 1
        fi

        for i in "${!configs[@]}"; do
            echo "  $((i+1))) ${configs[$i]}"
        done
        echo ""
        read -p "请选择配置 [1-${#configs[@]}]: " config_choice

        if [ "$config_choice" -lt 1 ] || [ "$config_choice" -gt ${#configs[@]} ]; then
            echo -e "${RED}✗ 无效的选项${RESET}"
            exit 1
        fi

        selected_config="${configs[$((config_choice-1))]}"

        # 执行模式
        echo -e "\n${YELLOW}执行模式：${RESET}"
        echo "  1) single - 单次执行"
        echo "  2) loop - 循环执行"
        echo "  3) step - 单步执行"
        echo "  4) reverse - 反向执行"
        echo ""
        read -p "请选择模式 [1-4]: " mode_choice

        case $mode_choice in
            1) mode="single" ;;
            2) mode="loop" ;;
            3) mode="step" ;;
            4) mode="reverse" ;;
            *) echo -e "${RED}✗ 无效的选项${RESET}"; exit 1 ;;
        esac

        # 可选参数
        echo -e "\n${YELLOW}可选参数（直接回车使用默认值）：${RESET}"
        read -p "速度比例 [0.1-1.0, 默认0.5]: " velocity
        read -p "路径点间停留时间 (秒, 默认0.5): " pause

        if [ "$mode" == "loop" ]; then
            read -p "循环次数 (默认无限): " loop_count
        fi

        # 构建命令
        cmd="rosrun aubo_linked_execution playback_waypoints.py --config $selected_config --mode $mode"
        [ -n "$velocity" ] && cmd="$cmd --velocity $velocity"
        [ -n "$pause" ] && cmd="$cmd --pause $pause"
        [ -n "$loop_count" ] && cmd="$cmd --loop-count $loop_count"

        echo -e "\n${CYAN}执行命令: $cmd${RESET}\n"
        eval $cmd
        ;;
    3)
        echo -e "\n${CYAN}配置文件列表：${RESET}"
        ls -lh "$CONFIG_DIR"/*.yaml 2>/dev/null || echo -e "${YELLOW}暂无配置文件${RESET}"
        ;;
    4)
        echo -e "${GREEN}再见！${RESET}"
        exit 0
        ;;
    *)
        echo -e "${RED}✗ 无效的选项${RESET}"
        exit 1
        ;;
esac
