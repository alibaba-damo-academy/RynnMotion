#!/bin/bash

set -e

# ----------------------------
# Language Selection / 语言选择
# ----------------------------
echo "Select language / 选择语言:"
echo "  1. English"
echo "  2. 中文"
read -p "Choice/选择 (1/2, default=1): " lang_choice

if [ "$lang_choice" = "2" ]; then
    LANG_ZH=true
    export RYNNMOTION_LANG="zh"
else
    LANG_ZH=false
    export RYNNMOTION_LANG="en"
fi

# Bilingual message helper
msg() {
    if [ "$LANG_ZH" = true ]; then
        echo "$2"
    else
        echo "$1"
    fi
}

# ----------------------------
# Setup script for RynnLeRobot using uv
# ----------------------------
msg "🚀 Setting up RynnLeRobot package with uv..." "🚀 正在使用 uv 设置 RynnLeRobot 包..."

install_uv() {
    msg "🔧 Installing uv..." "🔧 正在安装 uv..."

    if [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS
        if command -v brew &> /dev/null; then
            msg "🍺 Installing uv using Homebrew..." "🍺 使用 Homebrew 安装 uv..."
            brew install uv
        else
            msg "🐍 Installing uv using Python pip..." "🐍 使用 Python pip 安装 uv..."
            pip install uv
        fi
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        msg "🌐 Installing uv using official installer..." "🌐 使用官方安装程序安装 uv..."
        curl -LsSf https://astral.sh/uv/install.sh | sh
        export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
    else
        msg "🐍 Installing uv using Python pip..." "🐍 使用 Python pip 安装 uv..."
        pip install uv
    fi

    # Check if installation was successful
    if ! command -v uv &> /dev/null; then
        msg "❌ Failed to install uv" "❌ uv 安装失败"
        exit 1
    fi

    msg "✅ uv installed successfully" "✅ uv 安装成功"
}

if ! command -v uv &> /dev/null; then
    msg "⚠️ uv is not installed." "⚠️ uv 未安装。"
    install_uv
else
    msg "✅ uv is already installed" "✅ uv 已安装"
fi

if [[ ! -f "setup_env.sh" ]]; then
    msg "❌ Error: Please run this script from robots/RynnLeRobot/ directory" "❌ 错误：请从 robots/RynnLeRobot/ 目录运行此脚本"
    msg "Current directory: $(pwd)" "当前目录：$(pwd)"
    exit 1
fi

VENV_DIR="venv"


if [[ "$OSTYPE" == "msys"* ]] || [[ "$OSTYPE" == "win32"* ]]; then
    # Windows Git Bash with conda venv
    msg "🔧 Setting up windows env for common modules..." "🔧 正在为公共模块设置..."
    mamba create -n venv python=3.13 -y

    eval "$(mamba.exe shell hook --shell bash)"

    mamba activate venv

else
    msg "📦 Creating virtual environment with uv..." "📦 正在使用 uv 创建虚拟环境..."
    uv venv $VENV_DIR --python 3.13
    source "$VENV_DIR/bin/activate"

    msg "✅ Virtual environment activated: $(basename $VIRTUAL_ENV)" "✅ 虚拟环境已激活：$(basename $VIRTUAL_ENV)"

    # Ensure proper PATH for uv venv
    export PATH="$VIRTUAL_ENV/bin:$PATH"
    msg "📍 Updated PATH to prioritize venv binaries" "📍 已更新 PATH 以优先使用 venv 二进制文件"

    msg "🔧 Setting up PYTHONPATH for common modules..." "🔧 正在为公共模块设置 PYTHONPATH..."
    export PYTHONPATH="$(pwd)/../../../:$PYTHONPATH"
    msg "✅ PYTHONPATH set to include common directory: $(pwd)/../../../" "✅ PYTHONPATH 已设置为包含公共目录：$(pwd)/../../../"

    echo "export PYTHONPATH=\"$(pwd)/../../../:\$PYTHONPATH\"" >> "$VENV_DIR/bin/activate"
    echo "export PATH=\"\$VIRTUAL_ENV/bin:\$PATH\"" >> "$VENV_DIR/bin/activate"
    msg "✅ Added PYTHONPATH and PATH to virtual environment activation script" "✅ 已将 PYTHONPATH 和 PATH 添加到虚拟环境激活脚本"

    msg "📦 Installing dependencies with uv..." "📦 正在使用 uv 安装依赖..."
    export UV_VIRTUAL_ENV="$(pwd)/$VENV_DIR"
fi

# ----------------------------
# User Choice: PyTorch Version (BEFORE installing other packages)
# ----------------------------
echo ""
msg "🔧 PyTorch Installation Options:" "🔧 PyTorch 安装选项："
msg "  1) CPU-only (lightweight, ~70MB)" "  1) 仅 CPU 版本（轻量级，约 70MB）"
msg "  2) CUDA (full version with GPU support, ~2GB)" "  2) CUDA 版本（支持 GPU，约 2GB）"
echo ""

if [ "$LANG_ZH" = true ]; then
    read -p "选择 PyTorch 版本 (1/2, 默认=1): " pytorch_choice
else
    read -p "Select PyTorch version (1/2, default=1): " pytorch_choice
fi

case "$pytorch_choice" in
    ""|1 )
        msg "🔧 Installing PyTorch CPU-only version..." "🔧 正在安装 PyTorch CPU 版本..."
        TORCH_INDEX_URL="https://download.pytorch.org/whl/cpu"
        ;;
    2 )
        msg "🔧 Installing PyTorch with CUDA support..." "🔧 正在安装支持 CUDA 的 PyTorch..."
        TORCH_INDEX_URL="https://download.pytorch.org/whl/cu121"
        ;;
    * )
        msg "Invalid input. Defaulting to CPU-only version..." "无效输入，默认安装 CPU 版本..."
        TORCH_INDEX_URL="https://download.pytorch.org/whl/cpu"
        ;;
esac

msg "🔧 Installing PyTorch and torchvision..." "🔧 正在安装 PyTorch 和 torchvision..."
uv pip install torch torchvision --index-url $TORCH_INDEX_URL

msg "🔧 Installing RynnMotion core package from main repo (skipping torch dependency)..." "🔧 正在从主仓库安装 RynnMotion 核心包（跳过 torch 依赖）..."
uv pip install -e ../../python/ --no-deps

msg "🔧 Installing remaining RynnMotion dependencies..." "🔧 正在安装其余 RynnMotion 依赖..."
uv pip install numpy scipy PyYAML matplotlib opencv-python tqdm mujoco fsspec pillow build lcm python-statemachine datasets huggingface_hub packaging mink loop-rate-limiters robot_descriptions ruckig

if [[ "$OSTYPE" == "msys"* ]] || [[ "$OSTYPE" == "win32"* ]]; then
    msg "⚠️ Windows detected. Skipping pin installation, Install by conda..." "⚠️ 检测到 Windows。跳过 pin 安装, 使用conda 安装..."
    conda install -c conda-forge pinocchio hpp-fcl eigenpy assimp -y
else
    uv pip install pin
fi

msg "🔧 Installing RynnLeRobot package and its dependencies..." "🔧 正在安装 RynnLeRobot 包及其依赖..."
uv pip install -e .

msg "✅ All dependencies installed via pyproject.toml" "✅ 所有依赖已通过 pyproject.toml 安装"

msg "🔧 Installing development tools..." "🔧 正在安装开发工具..."
uv pip install -e .[dev]

msg "✅ Verifying critical packages are installed..." "✅ 正在验证关键包是否已安装..."

python -c "import RynnMotion; print('✅ RynnMotion package installed successfully')" || {
    msg "❌ RynnMotion package installation failed, trying again with uv..." "❌ RynnMotion 包安装失败，正在使用 uv 重试..."
    uv pip install --force-reinstall -e ../../python/
}

python -c "import datasets; print('✅ datasets package installed successfully')" || {
    msg "❌ datasets package installation failed, trying again with uv..." "❌ datasets 包安装失败，正在使用 uv 重试..."
    uv pip install --force-reinstall datasets pyarrow
}

python -c "import torch; print('✅ torch package installed successfully')" || {
    msg "❌ torch package installation failed, trying again with uv..." "❌ torch 包安装失败，正在使用 uv 重试..."
    uv pip install torch torchvision --index-url $TORCH_INDEX_URL
}

msg "🔍 Final verification - checking pip location..." "🔍 最终验证 - 检查 pip 位置..."
msg "Python location: $(which python)" "Python 位置：$(which python)"
msg "Pip location: $(which pip)" "Pip 位置：$(which pip)"

# Fix pip path if it's not pointing to the venv
if [[ "$(which pip)" != *"$VENV_DIR"* ]]; then
    msg "⚠️  Pip is not pointing to venv, installing pip in venv..." "⚠️  Pip 未指向 venv，正在 venv 中安装 pip..."
    # Install pip directly in the uv venv
    uv pip install pip
    msg "✅ Installed pip in venv: $VIRTUAL_ENV/bin/pip" "✅ 已在 venv 中安装 pip：$VIRTUAL_ENV/bin/pip"

    if [[ "$OSTYPE" == "msys"* ]] || [[ "$OSTYPE" == "win32"* ]]; then
        msg "⚠️  Skipping pip wrapper creation for Windows..." "⚠️  跳过在 Windows 上创建 pip 包装器..."
    else
        # Verify the fix
        if [[ -f "$VIRTUAL_ENV/bin/pip" ]]; then
            msg "✅ Pip is now available in venv" "✅ Pip 现已在 venv 中可用"
        else
            msg "⚠️  Creating pip wrapper..." "⚠️  正在创建 pip 包装器..."
            echo '#!/bin/bash' > "$VIRTUAL_ENV/bin/pip"
            echo 'exec python -m pip "$@"' >> "$VIRTUAL_ENV/bin/pip"
            chmod +x "$VIRTUAL_ENV/bin/pip"
            msg "✅ Created pip wrapper in venv" "✅ 已在 venv 中创建 pip 包装器"
        fi
    fi
else
    msg "✅ Pip is correctly pointing to venv" "✅ Pip 已正确指向 venv"
fi

msg "🔧 Registering virtual environment as Jupyter kernel..." "🔧 正在将虚拟环境注册为 Jupyter 内核..."
python -m ipykernel install --user --name=RynnLeRobot --display-name "Python (RynnLeRobot)"

# ----------------------------
# User Choice: Proceed or Exit for Port Detection
# ----------------------------
echo ""
if [ "$LANG_ZH" = true ]; then
    read -p "是否继续进行电机端口检测？(y/n, 默认=y): " choice
else
    read -p "Would you like to proceed with motor port detection? (y/n, default=y): " choice
fi
case "$choice" in
    ""|y|Y )
        msg "Proceeding with motor port detection..." "正在进行电机端口检测..."

        # ----------------------------
        # Step 1: Run Python script to detect motor serial ports
        # ----------------------------
        msg "Running Python script to detect motor serial ports..." "正在运行 Python 脚本检测电机串口..."
        if [ ! -f "RynnLeRobot/scripts/find_motor_port.py" ]; then
            msg "❌ Error: Port detection script not found!" "❌ 错误：未找到端口检测脚本！"
            msg "Skipping port detection and continuing with setup..." "跳过端口检测，继续设置..."
        else
            python -m RynnLeRobot.scripts.find_motor_port
        fi
        ;;
    n|N )
        msg "Skipping motor port detection." "跳过电机端口检测。"
        ;;
    * )
        msg "Invalid input. Skipping motor port detection." "无效输入，跳过电机端口检测。"
        ;;
esac

# ----------------------------
# User Choice: Proceed or Exit for Camera Detection
# ----------------------------
echo ""
if [ "$LANG_ZH" = true ]; then
    read -p "是否继续进行摄像头端口检测？(y/n, 默认=y): " choice
else
    read -p "Would you like to proceed with camera port detection? (y/n, default=y): " choice
fi
case "$choice" in
    ""|y|Y )
        msg "Proceeding with camera port detection..." "正在进行摄像头端口检测..."

        # ----------------------------
        # Step 2: Run Python script to detect camera ports
        # ----------------------------
        msg "Running Python script to detect camera ports..." "正在运行 Python 脚本检测摄像头端口..."
        if [ ! -f "RynnLeRobot/scripts/find_camera_port.py" ]; then
            msg "❌ Error: Camera detection script not found!" "❌ 错误：未找到摄像头检测脚本！"
            msg "Skipping camera detection and continuing with setup..." "跳过摄像头检测，继续设置..."
        else
            python -m RynnLeRobot.scripts.find_camera_port
        fi
        ;;
    n|N )
        msg "Skipping camera port detection." "跳过摄像头端口检测。"
        ;;
    * )
        msg "Invalid input. Skipping camera port detection." "无效输入，跳过摄像头端口检测。"
        ;;
esac


echo ""
msg "✅ Setup completed successfully!" "✅ 设置成功完成！"
echo ""
msg "🔄 To activate the environment in the future:" "🔄 以后激活环境："
if [[ "$OSTYPE" == "msys"* ]] || [[ "$OSTYPE" == "win32"* ]]; then
    echo '  eval "$(mamba.exe shell hook --shell bash)"'
    echo "  mamba activate venv"
else
    echo "  source $VENV_DIR/bin/activate"
fi
echo ""
msg "🎯 Quick start commands (after activation):" "🎯 快速启动命令（激活后）："
echo ""
msg "🔧 Device calibration & detection:" "🔧 设备校准与检测："
msg "  calibrate                         # Calibrate follower/leader/second leader" \
    "  calibrate                         # 校准从臂/主臂/第二主臂"
msg "  find-motor-port                   # Detect motor serial ports" \
    "  find-motor-port                   # 检测电机串口"
msg "  find-camera-port                  # Detect camera ports" \
    "  find-camera-port                  # 检测摄像头端口"
echo ""
msg "🤖 SO101 Motion Control:" "🤖 SO101 运动控制："
msg "  so101-motion --mode sim           # Simulation mode" \
    "  so101-motion --mode sim           # 仿真模式"
msg "  so101-motion --mode sim --motion 2  # With motion pattern 2" \
    "  so101-motion --mode sim --motion 2  # 使用运动模式 2"
echo ""
msg "🎮 Teleoperation (Leader-Follower):" "🎮 主从遥操作："
msg "  joint-teleop --mode sim           # SO101 to SO101 (simulation)" \
    "  joint-teleop --mode sim           # SO101 主从遥操作（仿真）"
msg "  joint-teleop --mode real          # SO101 to SO101 (real hardware)" \
    "  joint-teleop --mode real          # SO101 主从遥操作（真实硬件）"
echo ""
msg "🤖 Heterogeneous Teleoperation:" "🤖 异构遥操作："
msg "  so101-teleop --robot fr3          # SO101 to dual FR3" \
    "  so101-teleop --robot fr3          # SO101 控制双臂 FR3"
msg "  so101-teleop --robot ur5e         # SO101 to dual UR5e" \
    "  so101-teleop --robot ur5e         # SO101 控制双臂 UR5e"
msg "  multi-teleop --dual -mn 24        # Multi-robot teleop" \
    "  multi-teleop --dual -mn 24        # 多机器人遥操作"
echo ""
msg "🎯 Other Commands:" "🎯 其他命令："
msg "  so101-pickplace                   # Pick and place demo" \
    "  so101-pickplace                   # 抓取放置演示"
msg "  lerobot-ui                        # LeRobot visualization UI" \
    "  lerobot-ui                        # LeRobot 可视化界面"
msg "  record                            # Record dataset" \
    "  record                            # 录制数据集"
echo ""
exit 0
