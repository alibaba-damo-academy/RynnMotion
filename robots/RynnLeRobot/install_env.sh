
#!/bin/bash

set -e

# ----------------------------
# Environment setup script for RynnLeRobot using uv (Non-interactive mode)
# ----------------------------
echo "🚀 Setting up RynnMotion-LeRobot/LeKiwi Motion Control System environment with uv..."

install_uv() {
    echo "🔧 Installing uv..."

    if [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS
        if command -v brew &> /dev/null; then
            echo "🍺 Installing uv using Homebrew..."
            brew install uv
        else
            echo "🐍 Installing uv using Python pip..."
            pip install uv
        fi
    elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
        echo "🌐 Installing uv using official installer..."
        curl -LsSf https://astral.sh/uv/install.sh | sh
        export PATH="$HOME/.local/bin:$HOME/.cargo/bin:$PATH"
    else
        echo "🐍 Installing uv using Python pip..."
        pip install uv
    fi

    # Check if installation was successful
    if ! command -v uv &> /dev/null; then
        echo "❌ Failed to install uv"
        exit 1
    fi

    echo "✅ uv installed successfully"
}

setup_system_deps() {
     # 只在 Linux GNU 系统运行
     if [[ "$OSTYPE" != linux-gnu* ]]; then
         return 0  # 非 Linux 跳过
     fi

     # 检查是否为 Debian/Ubuntu 系（有 apt）
     if ! command -v apt &>/dev/null; then
         echo "⚠️  当前系统不支持 apt（非 Debian/Ubuntu），跳过系统依赖安装"
         return 0
     fi

     echo "🔧 正在检查并安装系统依赖（内核头文件、编译工具等）..."

     # 定义所需包
     local required_packages=(
         linux-headers-generic
         build-essential
         python3-dev
         libevdev-dev
         curl
         git
         libpng-dev
     )

     # 检查哪些包未安装
     local missing_packages=()
     for pkg in "${required_packages[@]}"; do
         # 使用 dpkg 判断是否真正安装
         if ! dpkg -s "$pkg" >/dev/null 2>&1; then
             missing_packages+=("$pkg")
         fi
     done

     # 如果没有缺失的包，直接返回
     if [ ${#missing_packages[@]} -eq 0 ]; then
         echo "✅ 所有系统依赖已安装，跳过"
         return 0
     fi
     
     if ! sudo apt update; then
         echo "❌ apt update 失败，请检查网络或权限"
         return 1
     fi

     # 安装缺失的包
     echo "📦 正在安装缺失的系统包: ${missing_packages[*]}"
     if sudo apt -y install "${missing_packages[@]}"; then
         echo "✅ 系统依赖安装成功"
     else
         echo "❌ 系统依赖安装失败，请手动运行："
         echo "      sudo apt install ${missing_packages[*]}"
         return 1
     fi
}

# 调用函数
setup_system_deps || echo "⚠️ 系统依赖安装失败，可能影响后续构建"

if ! command -v uv &> /dev/null; then
    echo "⚠️ uv is not installed."
    install_uv
else
    echo "✅ uv is already installed"
fi

if [[ ! -f "install_env.sh" ]] && [[ ! -f "setup_env.sh" ]]; then
    echo "❌ Error: Please run this script from robots/RynnLeRobot/ directory"
    echo "Current directory: $(pwd)"
    exit 1
fi

#if [[ "$OSTYPE" == "darwin"* ]]; then
    VENV_DIR="venv"
#else
#    VENV_DIR=".venv"
#fi

echo "📦 Creating virtual environment with uv..."
uv venv $VENV_DIR --python 3.13

source "$VENV_DIR/bin/activate"
echo "✅ Virtual environment activated: $(basename $VIRTUAL_ENV)"

# Ensure proper PATH for uv venv
export PATH="$VIRTUAL_ENV/bin:$PATH"
echo "📍 Updated PATH to prioritize venv binaries"

echo "🔧 Setting up PYTHONPATH for common modules..."
export PYTHONPATH="$(pwd)/../../../:$PYTHONPATH"
echo "✅ PYTHONPATH set to include common directory: $(pwd)/../../../"

echo "export PYTHONPATH=\"$(pwd)/../../../:\$PYTHONPATH\"" >> "$VENV_DIR/bin/activate"
echo "export PATH=\"\$VIRTUAL_ENV/bin:\$PATH\"" >> "$VENV_DIR/bin/activate"
echo "✅ Added PYTHONPATH and PATH to virtual environment activation script"

echo "📦 Installing dependencies with uv..."
export UV_VIRTUAL_ENV="$(pwd)/$VENV_DIR"

# ----------------------------
# Automatic PyTorch Version Selection (Non-interactive mode)
# ----------------------------
echo ""
echo "🔧 PyTorch Installation Options:"
echo "  1) CPU-only (lightweight, ~200MB)"
echo "  2) CUDA (full version with GPU support, ~2GB)"
echo ""
echo "🔧 Automatically selecting default option (CPU-only)..."
pytorch_choice="1"

case "$pytorch_choice" in
    ""|1 )
        echo "🔧 Installing PyTorch CPU-only version..."
        TORCH_INDEX_URL="https://download.pytorch.org/whl/cpu"
        ;;
    2 )
        echo "🔧 Installing PyTorch with CUDA support..."
        TORCH_INDEX_URL="https://download.pytorch.org/whl/cu121"
        ;;
    * )
        echo "Invalid input. Defaulting to CPU-only version..."
        TORCH_INDEX_URL="https://download.pytorch.org/whl/cpu"
        ;;
esac

echo "🔧 Installing PyTorch and torchvision..."
uv pip install torch torchvision --index-url $TORCH_INDEX_URL

echo "🔧 Installing RynnMotion core package from main repo (skipping torch dependency)..."
uv pip install -e ../../python/ --no-deps

echo "🔧 Installing remaining RynnMotion dependencies..."
uv pip install numpy scipy PyYAML matplotlib opencv-python tqdm mujoco fsspec pin pillow build lcm python-statemachine datasets huggingface_hub packaging

echo "🔧 Installing RynnLeRobot package and its dependencies..."
uv pip install -e .

echo "✅ All dependencies installed via pyproject.toml"

echo "🔧 Installing development tools..."
uv pip install -e .[dev]

echo "✅ Verifying critical packages are installed..."

python -c "import RynnMotion; print('✅ RynnMotion package installed successfully')" || {
    echo "❌ RynnMotion package installation failed, trying again with uv..."
    uv pip install --force-reinstall -e ../../python/
}

python -c "import datasets; print('✅ datasets package installed successfully')" || {
    echo "❌ datasets package installation failed, trying again with uv..."
    uv pip install --force-reinstall datasets pyarrow
}

python -c "import torch; print('✅ torch package installed successfully')" || {
    echo "❌ torch package installation failed, trying again with uv..."
    uv pip install torch torchvision --index-url $TORCH_INDEX_URL
}

echo "🔍 Final verification - checking pip location..."
echo "Python location: $(which python)"
echo "Pip location: $(which pip)"

# Fix pip path if it's not pointing to the venv
if [[ "$(which pip)" != *"$VENV_DIR"* ]]; then
    echo "⚠️  Pip is not pointing to venv, installing pip in venv..."
    # Install pip directly in the uv venv
    uv pip install pip
    echo "✅ Installed pip in venv: $VIRTUAL_ENV/bin/pip"

    # Verify the fix
    if [[ -f "$VIRTUAL_ENV/bin/pip" ]]; then
        echo "✅ Pip is now available in venv"
    else
        echo "⚠️  Creating pip wrapper..."
        echo '#!/bin/bash' > "$VIRTUAL_ENV/bin/pip"
        echo 'exec python -m pip "$@"' >> "$VIRTUAL_ENV/bin/pip"
        chmod +x "$VIRTUAL_ENV/bin/pip"
        echo "✅ Created pip wrapper in venv"
    fi
else
    echo "✅ Pip is correctly pointing to venv"
fi

echo "🔧 Registering virtual environment as Jupyter kernel..."
python -m ipykernel install --user --name=RynnLeRobot --display-name "Python (RynnLeRobot)"

echo ""
echo "✅ Environment setup completed successfully!"
echo ""
echo "🔄 To activate the environment in the future:"
echo "  source $VENV_DIR/bin/activate"