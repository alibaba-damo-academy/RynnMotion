#!/bin/bash

set -e

# ----------------------------
# Step 0: Check Python 3.13 availability
# ----------------------------
echo "🚀 Setting up RynnMotion python environment..."
echo ""

# Check if Python 3.13 is available
PYTHON_CMD=""
if command -v python3.13 &> /dev/null; then
    PYTHON_CMD="python3.13"
    echo "✅ Found Python 3.13: $(python3.13 --version)"
elif command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version 2>&1 | sed -n 's/Python \([0-9]*\.[0-9]*\).*/\1/p')
    MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
    MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)

    if [ "$MAJOR" -eq 3 ] && [ "$MINOR" -ge 13 ]; then
        PYTHON_CMD="python3"
        echo "✅ Found Python $PYTHON_VERSION (meets >=3.13 requirement)"
    else
        echo "❌ Error: Python 3.13+ is required, but found Python $PYTHON_VERSION"
        echo ""
        echo "On Ubuntu 22.04, install Python 3.13 with:"
        echo "  sudo add-apt-repository ppa:deadsnakes/ppa"
        echo "  sudo apt update"
        echo "  sudo apt install python3.13 python3.13-venv python3.13-dev"
        echo ""
        echo "On macOS, install with Homebrew:"
        echo "  brew install python@3.13"
        exit 1
    fi
else
    echo "❌ Error: Python 3 not found"
    echo "Please install Python 3.13+ and try again"
    exit 1
fi

echo ""

# ----------------------------
# Step 1: Check and install uv
# ----------------------------
# Check if uv is installed
if ! command -v uv &> /dev/null; then
    echo "📦 uv not found. Installing uv via pip..."
    if command -v pip &> /dev/null; then
        pip install --user uv
    else
        $PYTHON_CMD -m ensurepip --user
        $PYTHON_CMD -m pip install --user uv
    fi

    # Add uv to PATH for current session if needed
    export PATH="$HOME/.local/bin:$PATH"

    if ! command -v uv &> /dev/null; then
        echo "❌ Error: uv installation failed"
        echo "Please install uv manually: https://github.com/astral-sh/uv"
        exit 1
    fi
fi

echo "✅ uv is available: $(uv --version)"
echo ""

# ----------------------------
# Step 2: Create and activate virtual environment
# ----------------------------
if [[ "$OSTYPE" == "darwin"* ]]; then
    VENV_DIR="venv"
else
    VENV_DIR=".venv"
fi

echo "📦 Creating virtual environment in $VENV_DIR using Python 3.13..."
uv venv $VENV_DIR --python 3.13
echo "✅ Virtual environment created"

ACTIVATE_SCRIPT="$VENV_DIR/bin/activate"
source "$ACTIVATE_SCRIPT"
echo "✅ Virtual environment activated: $(basename $VIRTUAL_ENV)"

# Verify Python version in venv
echo ""
echo "🔍 Verifying virtual environment Python version..."
VENV_PYTHON_VERSION=$(python --version 2>&1 | sed -n 's/Python \([0-9]*\.[0-9]*\).*/\1/p')
echo "   Virtual environment Python: $VENV_PYTHON_VERSION"

VENV_MAJOR=$(echo $VENV_PYTHON_VERSION | cut -d. -f1)
VENV_MINOR=$(echo $VENV_PYTHON_VERSION | cut -d. -f2)

if [ "$VENV_MAJOR" -ne 3 ] || [ "$VENV_MINOR" -lt 13 ]; then
    echo "❌ Error: Virtual environment has wrong Python version ($VENV_PYTHON_VERSION)"
    echo "   Expected: Python >=3.13"
    echo "   This may happen if 'uv' couldn't find Python 3.13"
    echo ""
    echo "Please ensure Python 3.13 is installed and accessible"
    exit 1
fi

echo "✅ Python version verified: $VENV_PYTHON_VERSION"
echo ""

read -p "Install PyTorch CPU-only version? (Y/n, default: Y): " pytorch_choice
case "$pytorch_choice" in
    ""|y|Y )
        echo "🔧 Installing PyTorch CPU-only version..."
        uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
        ;;
    n|N )
        echo "🔧 Installing PyTorch with CUDA support (if available)..."
        ;;
    * )
        echo "Invalid input. Defaulting to CPU-only PyTorch..."
        uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
        ;;
esac

echo "📦 Installing RynnMotion package with dev dependencies..."
uv pip install -e ".[dev]"

# Final verification
echo ""
echo "🔍 Final package verification..."
echo "   NumPy version: $(python -c 'import numpy; print(numpy.__version__)')"
echo "   Pinocchio version: $(python -c 'import pinocchio; print(pinocchio.__version__)' 2>/dev/null || echo 'Not installed or import failed')"
echo "   Python version: $(python --version)"
echo ""
echo "✅ Setup complete!"
