#!/bin/bash

set -e

# ----------------------------
# Language selection
# ----------------------------
echo "🌍 Select language / 选择语言:"
echo "1) English (EN)"
echo "2) 中文 (CN)"
read -p "Please select language (1/2, default=1): " lang_choice

case "$lang_choice" in
2 | CN | CN)
    LANGUAGE="CN"
	export RYNNMOTION_LANG=zh
    ;;
*)
    LANGUAGE="EN"
	export RYNNMOTION_LANG=en
    ;;
esac

if [[ "$OSTYPE" == "msys"* ]] || [[ "$OSTYPE" == "win32"* ]]; then
    VIRTUAL_ENV="venv"
fi

# Language strings
if [[ "$LANGUAGE" == "CN" ]]; then
    MSG_CONFIGURING="🔧 正在配置 RynnLeRobot 系统..."
    MSG_VENV_ACTIVATED="✅ 虚拟环境已激活: $(basename $VIRTUAL_ENV)"
    MSG_VENV_NOT_FOUND="⚠️  未找到虚拟环境。请先运行 install_env.sh。"
    MSG_CONTINUE_WITHOUT_VENV="是否在不激活虚拟环境的情况下继续? (y/N): "
    MSG_MOTOR_PORT_DETECTION="是否继续进行电机端口检测? (y/n, 默认=y): "
    MSG_PROCEEDING_MOTOR="正在执行电机端口检测..."
    MSG_RUNNING_MOTOR_SCRIPT="正在运行电机端口检测脚本..."
    MSG_SCRIPT_NOT_FOUND="❌ 错误: 端口检测脚本未找到!"
    MSG_SKIPPING_MOTOR="跳过电机端口检测并继续设置..."
    MSG_SKIPPING_MOTOR_USER="跳过电机端口检测。"
    MSG_INVALID_INPUT_MOTOR="输入无效。跳过电机端口检测。"
    MSG_CAMERA_PORT_DETECTION="是否继续进行摄像头端口检测? (y/n, 默认=y): "
    MSG_PROCEEDING_CAMERA="正在执行摄像头端口检测..."
    MSG_RUNNING_CAMERA_SCRIPT="正在运行摄像头端口检测脚本..."
    MSG_CAMERA_SCRIPT_NOT_FOUND="❌ 错误: 摄像头检测脚本未找到!"
    MSG_SKIPPING_CAMERA="跳过摄像头端口检测并继续设置..."
    MSG_SKIPPING_CAMERA_USER="跳过摄像头端口检测。"
    MSG_INVALID_INPUT_CAMERA="输入无效。跳过摄像头端口检测。"
    MSG_COMPLETED="✅ 配置完成!"
    MSG_QUICK_START="🎯 快速启动命令 (激活后使用):"
    MSG_CALIBRATION="🔧 设备校准命令:"
    MSG_AUTO_CALIBRATE="# 自动校准机器人或遥操作设备 (从 configs/lerobot.yaml 读取配置):"
    MSG_CALIBRATE_SCRIPT="# 脚本会根据您的配置文件自动检测并校准"
    MSG_RUN_FOLLOWER="# 运行 so101 follower 模拟模式或真实模式"
    MSG_RUN_LEADER_FOLLOWER="# 运行 so101 leader & follower 遥操作模拟模式或真实模式"
    MSG_USING_JOINT_TELEOP="# 使用新的 joint-teleop 命令:"
    MSG_OR_DIRECTLY="# 或者直接使用 python 模块:"
    MSG_USING_UV="# 使用 uv 直接运行 (无需激活环境):"
    MSG_DEVELOPMENT="💡 开发相关命令:"
    MSG_FORMAT_CODE="# 格式化代码"
    MSG_RUN_TESTS="# 运行测试"
    MSG_TYPE_CHECKING="# 类型检查"
else
    MSG_CONFIGURING="🔧 Configuring RynnLeRobot system..."
    MSG_VENV_ACTIVATED="✅ Virtual environment activated: $(basename $VIRTUAL_ENV)"
    MSG_VENV_NOT_FOUND="⚠️  Virtual environment not found. Please run install_env.sh first."
    MSG_CONTINUE_WITHOUT_VENV="Continue without activating virtual environment? (y/N): "
    MSG_MOTOR_PORT_DETECTION="Would you like to proceed with motor port detection? (y/n, default=y): "
    MSG_PROCEEDING_MOTOR="Proceeding with motor port detection..."
    MSG_RUNNING_MOTOR_SCRIPT="Running Python script to detect motor serial ports..."
    MSG_SCRIPT_NOT_FOUND="❌ Error: Port detection script not found!"
    MSG_SKIPPING_MOTOR="Skipping port detection and continuing with setup..."
    MSG_SKIPPING_MOTOR_USER="Skipping motor port detection."
    MSG_INVALID_INPUT_MOTOR="Invalid input. Skipping motor port detection."
    MSG_CAMERA_PORT_DETECTION="Would you like to proceed with camera port detection? (y/n, default=y): "
    MSG_PROCEEDING_CAMERA="Proceeding with camera port detection..."
    MSG_RUNNING_CAMERA_SCRIPT="Running Python script to detect camera ports..."
    MSG_CAMERA_SCRIPT_NOT_FOUND="❌ Error: Camera detection script not found!"
    MSG_SKIPPING_CAMERA="Skipping camera detection and continuing with setup..."
    MSG_SKIPPING_CAMERA_USER="Skipping camera port detection."
    MSG_INVALID_INPUT_CAMERA="Invalid input. Skipping camera port detection."
    MSG_COMPLETED="✅ Configuration completed successfully!"
    MSG_QUICK_START="🎯 Quick start commands (after activation):"
    MSG_CALIBRATION="🔧 Device calibration commands:"
    MSG_AUTO_CALIBRATE="# Auto-calibrate robot or teleoperator (reads config from configs/lerobot.yaml):"
    MSG_CALIBRATE_SCRIPT="# The script automatically detects and calibrates based on your config file"
    MSG_RUN_FOLLOWER="# run so101 follower in sim mode or real mode"
    MSG_RUN_LEADER_FOLLOWER="# run so101 leader & follower teleoperation in sim mode or real mode"
    MSG_USING_JOINT_TELEOP="# Using the new joint-teleop command:"
    MSG_OR_DIRECTLY="# Or using python module directly:"
    MSG_USING_UV="# Using uv directly (without activating the environment):"
    MSG_DEVELOPMENT="💡 For development, you can also use:"
    MSG_FORMAT_CODE="# Format code"
    MSG_RUN_TESTS="# Run tests"
    MSG_TYPE_CHECKING="# Type checking"
fi

# ----------------------------
# Configuration script for RynnLeRobot
# ----------------------------
echo "$MSG_CONFIGURING"

#if [[ "$OSTYPE" == "darwin"* ]]; then
    VENV_DIR="venv"
#else
#    VENV_DIR=".venv"
#fi

if [[ "$OSTYPE" == "msys"* ]] || [[ "$OSTYPE" == "win32"* ]]; then
    echo "Windows detected. Skipping virtual environment activation."
else
    # Activate virtual environment
    if [[ -f "$VENV_DIR/bin/activate" ]]; then
        source "$VENV_DIR/bin/activate"
        echo "$MSG_VENV_ACTIVATED"
    else
        echo "$MSG_VENV_NOT_FOUND"
        read -p "$MSG_CONTINUE_WITHOUT_VENV" continue_without_venv
        if [[ ! "$continue_without_venv" =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# ----------------------------
# User Choice: Proceed or Exit for Port Detection
# ----------------------------
echo ""
read -p "$MSG_MOTOR_PORT_DETECTION" choice
case "$choice" in
"" | y | Y)
    echo "$MSG_PROCEEDING_MOTOR"

    # ----------------------------
    # Step 1: Run Python script to detect motor serial ports
    # ----------------------------
    echo "$MSG_RUNNING_MOTOR_SCRIPT"
    if [ ! -f "RynnLeRobot/scripts/find_motor_port.py" ]; then
        echo "$MSG_SCRIPT_NOT_FOUND"
        echo "$MSG_SKIPPING_MOTOR"
    else
        python -m RynnLeRobot.scripts.find_motor_port
    fi
    ;;
n | N)
    echo "$MSG_SKIPPING_MOTOR_USER"
    ;;
*)
    echo "$MSG_INVALID_INPUT_MOTOR"
    ;;
esac

# ----------------------------
# User Choice: Proceed or Exit for Camera Detection
# ----------------------------
echo ""
read -p "$MSG_CAMERA_PORT_DETECTION" choice
case "$choice" in
"" | y | Y)
    echo "$MSG_PROCEEDING_CAMERA"

    # ----------------------------
    # Step 2: Run Python script to detect camera ports
    # ----------------------------
    echo "$MSG_RUNNING_CAMERA_SCRIPT"
    if [ ! -f "RynnLeRobot/scripts/find_camera_port.py" ]; then
        echo "$MSG_CAMERA_SCRIPT_NOT_FOUND"
        echo "$MSG_SKIPPING_CAMERA"
    else
        python -m RynnLeRobot.scripts.find_camera_port
    fi
    ;;
n | N)
    echo "$MSG_SKIPPING_CAMERA_USER"
    ;;
*)
    echo "$MSG_INVALID_INPUT_CAMERA"
    ;;
esac

echo ""
echo "$MSG_COMPLETED"
echo ""
echo "$MSG_QUICK_START"
echo ""
echo "$MSG_CALIBRATION"
echo ""
echo "  $MSG_AUTO_CALIBRATE"
echo "  python -m RynnLeRobot.scripts.calibrate"
echo ""
echo "  $MSG_CALIBRATE_SCRIPT"
echo ""
echo "  $MSG_RUN_FOLLOWER"
echo "  python -m RynnLeRobot.controller.lerobot --mode sim --motion 2"
echo "  python -m RynnLeRobot.controller.lerobot --mode real"
echo ""
echo "  $MSG_RUN_LEADER_FOLLOWER"
echo ""
echo "  $MSG_USING_JOINT_TELEOP"
echo "  joint-teleop --mode sim"
echo "  joint-teleop --mode real"
echo ""
echo "  $MSG_OR_DIRECTLY"
echo "  python -m RynnLeRobot.controller.joint_teleop --mode sim"
echo "  python -m RynnLeRobot.controller.joint_teleop --mode real"
echo ""
echo "  $MSG_USING_UV"
echo "  uv run python -m RynnLeRobot.controller.lerobot --mode sim --motion 2"
echo ""
# echo "$MSG_DEVELOPMENT"
# echo "  uv run black .                    $MSG_FORMAT_CODE"
# echo "  uv run pytest tests/              $MSG_RUN_TESTS"
# echo "  uv run mypy RynnLeRobot/controller/  $MSG_TYPE_CHECKING"
# echo ""
