#!/bin/bash
# Unity Shadow 模式抽搐修复测试脚本

echo "=========================================="
echo "Unity Shadow 模式抽搐修复测试"
echo "=========================================="
echo ""

# 检查修改的文件
echo "1. 检查修改的文件..."
if [ -f "/home/wuqz/aubo_polish/src/aubo_unity_bridge/scripts/unity_command_forwarder.py" ]; then
    echo "   ✓ unity_command_forwarder.py 存在"
else
    echo "   ✗ unity_command_forwarder.py 不存在"
    exit 1
fi

if [ -f "/home/wuqz/aubo_polish/src/aubo_unity_bridge/launch/unity_bridge.launch" ]; then
    echo "   ✓ unity_bridge.launch 存在"
else
    echo "   ✗ unity_bridge.launch 不存在"
    exit 1
fi

# 检查 Python 语法
echo ""
echo "2. 检查 Python 语法..."
python3 -m py_compile /home/wuqz/aubo_polish/src/aubo_unity_bridge/scripts/unity_command_forwarder.py
if [ $? -eq 0 ]; then
    echo "   ✓ Python 语法正确"
else
    echo "   ✗ Python 语法错误"
    exit 1
fi

# 检查 XML 语法
echo ""
echo "3. 检查 XML 语法..."
xmllint --noout /home/wuqz/aubo_polish/src/aubo_unity_bridge/launch/unity_bridge.launch
if [ $? -eq 0 ]; then
    echo "   ✓ XML 语法正确"
else
    echo "   ✗ XML 语法错误"
    exit 1
fi

# 检查关键参数是否存在
echo ""
echo "4. 检查关键参数..."
if grep -q "position_deadzone" /home/wuqz/aubo_polish/src/aubo_unity_bridge/scripts/unity_command_forwarder.py; then
    echo "   ✓ position_deadzone 参数已添加"
else
    echo "   ✗ position_deadzone 参数缺失"
    exit 1
fi

if grep -q "velocity_threshold" /home/wuqz/aubo_polish/src/aubo_unity_bridge/scripts/unity_command_forwarder.py; then
    echo "   ✓ velocity_threshold 参数已添加"
else
    echo "   ✗ velocity_threshold 参数缺失"
    exit 1
fi

if grep -q "last_published_position" /home/wuqz/aubo_polish/src/aubo_unity_bridge/scripts/unity_command_forwarder.py; then
    echo "   ✓ last_published_position 变量已添加"
else
    echo "   ✗ last_published_position 变量缺失"
    exit 1
fi

# 检查 launch 文件参数
echo ""
echo "5. 检查 launch 文件参数..."
if grep -q "position_deadzone" /home/wuqz/aubo_polish/src/aubo_unity_bridge/launch/unity_bridge.launch; then
    echo "   ✓ launch 文件包含 position_deadzone"
else
    echo "   ✗ launch 文件缺少 position_deadzone"
    exit 1
fi

if grep -q "velocity_threshold" /home/wuqz/aubo_polish/src/aubo_unity_bridge/launch/unity_bridge.launch; then
    echo "   ✓ launch 文件包含 velocity_threshold"
else
    echo "   ✗ launch 文件缺少 velocity_threshold"
    exit 1
fi

echo ""
echo "=========================================="
echo "✓ 所有检查通过！"
echo "=========================================="
echo ""
echo "使用方法："
echo "  默认参数（推荐）："
echo "    roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \\"
echo "      robot_ip:=192.168.1.10 use_unity:=true"
echo ""
echo "  自定义参数："
echo "    roslaunch aubo_linked_execution aubo_e5_linked_execution.launch \\"
echo "      robot_ip:=192.168.1.10 use_unity:=true \\"
echo "      position_deadzone:=0.002 \\"
echo "      velocity_threshold:=0.01 \\"
echo "      shadow_filter_alpha:=0.7"
echo ""
echo "参考文档："
echo "  - src/aubo_unity_bridge/UNITY_SHADOW_FIX.md"
echo "  - src/aubo_unity_bridge/config/unity_shadow_tuning.yaml"
echo ""
