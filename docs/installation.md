# RynnMotion Installation Guide

This comprehensive guide covers all installation methods for RynnMotion across different platforms.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation Methods](#installation-methods)
  - [Native Installation (Recommended)](#native-installation-recommended)
  - [Docker Installation (Experimental)](#docker-installation-experimental)
  - [Python Package (Coming Soon)](#python-package-coming-soon)
- [Platform-Specific Instructions](#platform-specific-instructions)
  - [Ubuntu/Debian Linux](#ubuntudebian-linux)
  - [macOS](#macos)
  - [Windows (WSL2)](#windows-wsl2)
- [Verification](#verification)
- [Troubleshooting](#troubleshooting)
- [Manual Installation](#manual-installation)
- [Uninstallation](#uninstallation)

---

## Prerequisites

### System Requirements

**Minimum:**
- CPU: 4 cores (Intel/AMD x86_64 or Apple Silicon)
- RAM: 8 GB
- Disk: 10 GB free space
- GPU: Not required (optional for rendering acceleration)

**Recommended:**
- CPU: 8+ cores
- RAM: 16 GB
- Disk: 20 GB free space (SSD preferred)
- GPU: NVIDIA GPU with OpenGL 3.3+ support (for better visualization)

### Supported Platforms

| Platform | Status | Notes |
|----------|--------|-------|
| **Ubuntu 22.04+** | ✅ Fully Tested | Primary development platform |
| **Ubuntu 24.04** | ✅ Tested | Latest LTS release |
| **Debian 11+** | ⚠️ Should Work | Similar to Ubuntu |
| **macOS 13+ (Ventura)** | ⚠️ Experimental | Intel and Apple Silicon |
| **macOS 14+ (Sonoma)** | ⚠️ Experimental | Latest release |
| **Windows (WSL2)** | ⚠️ Untested | Via Ubuntu WSL2 |

### Required Tools

**All platforms:**
- Git (≥2.25)
- CMake (≥3.16)
- C++20 compiler:
  - GCC (≥11) on Linux
  - Clang (≥13) on macOS
- Python (≥3.8) for Python bindings

---

## Installation Methods

### Comparison Table

| Method | Setup Time | Status | Best For |
|--------|-----------|--------|----------|
| **Native** | 30-60 min | ✅ Production-Ready | Development, deployment, max performance |
| **Docker** | 15-30 min | ⚠️ Experimental | Quick evaluation, consistent environments |
| **Python Package** | 2 min | 📅 Coming Q2 2025 | Python-only workflows |

---

## Native Installation (Recommended)

### Quick Setup

```bash
# 1. Clone repository
git clone https://github.com/alibaba-damo-academy/RynnMotion.git
cd RynnMotion

# 2. Install dependencies (one-time, 30-60 min)
# Linux:
sudo ./scripts/setup_dependencies.sh

# macOS:
./scripts/setup_dependencies.sh  # NO sudo

# 3. Build RynnMotion (3-5 min)
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# 4. Install Python package (optional)
cd ../python
python3 -m venv venv
source venv/bin/activate
pip install -e ".[dev]"

# 5. Test installation
cd ../build
./mujocoExe fr3 1  # Should open MuJoCo viewer with FR3 robot
```

### What Gets Installed

The `setup_dependencies.sh` script installs:

**Core Libraries:**
- **Eigen** (≥3.3.0) - Linear algebra library
- **Boost** (≥1.65) - C++ utility libraries
- **yaml-cpp** - YAML parser for configuration files
- **nlohmann/json** - JSON library

**Physics & Kinematics:**
- **MuJoCo** (3.3.5) - Physics simulation engine
- **Pinocchio** (3.7.0) - Rigid body dynamics library
- **FCL** (Flexible Collision Library) - Collision detection

**Control & Optimization:**
- **qpOASES** - Quadratic programming solver
- **Ruckig** (v0.15.3) - Online trajectory generation
- **LCM** (Lightweight Communications and Marshalling)

**Visualization:**
- **OpenCV** - Computer vision library
- **GLEW**, **GLFW** - OpenGL utilities (Linux only)

**Installation Locations:**
- **Linux**: `/usr/local/include`, `/usr/local/lib`
- **macOS**: Homebrew prefix (usually `/opt/homebrew` or `/usr/local`)

### Build Options

#### Standard Build (Release)

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

#### Debug Build

```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)

# With GDB support
cmake -DCMAKE_BUILD_TYPE=Debug -DUSING_GDB=ON ..
make -j$(nproc)
```

#### Build with Tests

```bash
cmake -DBUILD_TESTS=ON ..
make -j$(nproc)

# Run tests
ctest
```

#### Build with ROS2 Support

```bash
cmake -DBUILD_ROS_MUJOCO=ON ..
make -j$(nproc)
```

#### Build Static Libraries

```bash
cmake -DBUILD_STATIC=ON ..
make -j$(nproc)
```

#### Custom Install Prefix

```bash
cmake -DCMAKE_INSTALL_PREFIX=/custom/path ..
make -j$(nproc)
sudo make install
```

---

## Docker Installation (Experimental)

> ⚠️ **Warning:** Docker setup has not been fully tested. For production use, please use native installation.

### Quick Setup

```bash
# 1. Clone repository
git clone https://github.com/alibaba-damo-academy/RynnMotion.git
cd RynnMotion

# 2. Build Docker image (15-30 min)
./scripts/docker-build.sh

# 3. Run container
./scripts/docker-run.sh

# 4. Inside container: build RynnMotion
mkdir build && cd build
cmake ..
make -j$(nproc)

# 5. Test
./mujocoExe fr3 1
```

### Full Docker Documentation

See **[Docker Setup Guide](DOCKER-SETUP.md)** for:
- GUI setup (X11 forwarding)
- Hardware access (USB devices, GPUs)
- Advanced configuration
- Docker Compose usage
- Troubleshooting

---

## Python Package (Coming Soon)

```bash
# Future release (Q2 2025)
pip install rynnmotion

# Test installation
python -c "from RynnMotion import RynnMuJoCo; print('✓ RynnMotion installed')"
```

---

## Platform-Specific Instructions

### Ubuntu/Debian Linux

#### Prerequisites

```bash
# Update package lists
sudo apt update

# Install build tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-dev \
    python3-pip \
    python3-venv

# Install OpenGL libraries (for MuJoCo viewer)
sudo apt install -y \
    libglew-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev
```

#### Run Installation Script

```bash
cd RynnMotion
sudo ./scripts/setup_dependencies.sh
```

The script will:
1. Install system packages via `apt`
2. Download and compile libraries from source
3. Install to `/usr/local/`

**Time:** 30-60 minutes depending on CPU

#### Troubleshooting

**Issue: `E: Unable to locate package`**

```bash
# Enable universe repository
sudo add-apt-repository universe
sudo apt update
```

**Issue: MuJoCo viewer doesn't open**

```bash
# Check OpenGL support
glxinfo | grep "OpenGL version"

# Should show version 3.3 or higher
# If not, install Mesa drivers:
sudo apt install mesa-utils
```

---

### macOS

#### Prerequisites

```bash
# Install Homebrew (if not already installed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install Xcode Command Line Tools
xcode-select --install

# Install basic dependencies via Homebrew
brew install cmake git python@3.11
```

#### Run Installation Script

```bash
cd RynnMotion
./scripts/setup_dependencies.sh  # NO sudo on macOS!
```

The script will:
1. Install some dependencies via Homebrew
2. Download and compile libraries from source
3. Install to Homebrew prefix (usually `/opt/homebrew` or `/usr/local`)

**Time:** 30-60 minutes depending on CPU

#### Apple Silicon (M1/M2/M3) Notes

**Rosetta 2 NOT required** - RynnMotion builds natively on ARM64.

If you encounter architecture issues:

```bash
# Check your architecture
uname -m  # Should show "arm64"

# Ensure you're using native Homebrew
which brew  # Should show /opt/homebrew/bin/brew

# If using x86_64 Homebrew, reinstall:
# 1. Uninstall old Homebrew
# 2. Reinstall from homebrew.sh
# 3. Re-run setup_dependencies.sh
```

#### Troubleshooting

**Issue: `xcrun: error: invalid active developer path`**

```bash
# Install Xcode Command Line Tools
xcode-select --install
```

**Issue: MuJoCo viewer doesn't open**

```bash
# Ensure XQuartz is NOT running (use native OpenGL)
pkill XQuartz

# Verify OpenGL support
system_profiler SPDisplaysDataType | grep OpenGL
```

**Issue: Python version conflicts**

```bash
# Use Homebrew Python
brew install python@3.11

# Create venv with specific Python
python3.11 -m venv venv
source venv/bin/activate
```

---

### Windows (WSL2)

> ⚠️ **Note:** Windows support is via WSL2 (Ubuntu). Native Windows builds are not currently supported.

#### Prerequisites

1. **Enable WSL2** (Windows 10 version 2004+ or Windows 11):

```powershell
# In PowerShell (Admin)
wsl --install
wsl --set-default-version 2
```

2. **Install Ubuntu** from Microsoft Store (Ubuntu 22.04 or 24.04)

3. **Update WSL2 Ubuntu:**

```bash
# Inside WSL2 terminal
sudo apt update && sudo apt upgrade -y
```

#### Follow Ubuntu Instructions

Once inside WSL2 Ubuntu terminal, follow the [Ubuntu installation steps](#ubuntudebian-linux) above.

#### GUI Support (X11)

To run MuJoCo viewer on Windows:

1. **Install VcXsrv** or **X410** (X server for Windows)

2. **Configure X server** to allow connections:
   - Launch VcXsrv with "Disable access control" enabled

3. **Set DISPLAY in WSL2:**

```bash
# Add to ~/.bashrc
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0

# Reload
source ~/.bashrc
```

4. **Test:**

```bash
cd RynnMotion/build
./mujocoExe fr3 1
```

---

## Verification

### Test C++ Installation

```bash
cd build

# 1. Check executables built
ls -lh mujocoExe
# Should show executable with size ~5-10 MB

# 2. Run simple simulation
./mujocoExe fr3 1

# Expected: MuJoCo viewer opens, FR3 robot loads, figure-8 trajectory
# Press Space to pause, Ctrl+Q to quit

# 3. Test other robots
./mujocoExe ur5e 1
./mujocoExe dual_fr3 pickplace
```

### Test Python Installation

```bash
cd python
source venv/bin/activate

# 1. Test import
python -c "from RynnMotion import RynnMuJoCo; print('✓ Python interface working')"

# 2. Test simulation
python -c "
from RynnMotion import RynnMuJoCo
sim = RynnMuJoCo('fr3', 1)
print(f'✓ Simulation created: {sim}')
"

# 3. Run example scripts
python scripts/pinkine_viewer.py --robot fr3
```

### Check Dependencies

```bash
# Verify installed libraries
pkg-config --modversion eigen3      # Should show ≥3.3.0
pkg-config --modversion pinocchio   # Should show 3.7.0

# Check MuJoCo
ls /usr/local/mujoco-3.3.5/  # Linux
ls /opt/homebrew/Cellar/mujoco/  # macOS (Homebrew)

# Check Python packages
pip list | grep RynnMotion
```

---

## Troubleshooting

### Build Errors

#### `CMake Error: Could not find Eigen3`

**Solution:**

```bash
# Linux
sudo apt install libeigen3-dev

# macOS
brew install eigen

# Or re-run dependency script
sudo ./scripts/setup_dependencies.sh  # Linux
./scripts/setup_dependencies.sh       # macOS
```

#### `undefined reference to pinocchio::...`

**Solution:**

```bash
# Check Pinocchio installation
pkg-config --cflags --libs pinocchio

# If not found, reinstall Pinocchio
cd /tmp
git clone --recursive https://github.com/stack-of-tasks/pinocchio
cd pinocchio && git checkout v3.7.0
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=OFF
make -j$(nproc)
sudo make install
```

#### `fatal error: mujoco/mujoco.h: No such file`

**Solution:**

```bash
# Linux: Check MuJoCo installation
ls /usr/local/include/mujoco/

# If missing, reinstall MuJoCo
cd /tmp
wget https://github.com/google-deepmind/mujoco/releases/download/3.3.5/mujoco-3.3.5-linux-x86_64.tar.gz
tar -xzf mujoco-3.3.5-linux-x86_64.tar.gz
sudo mv mujoco-3.3.5 /usr/local/
sudo ln -sf /usr/local/mujoco-3.3.5/include/mujoco /usr/local/include/
sudo ln -sf /usr/local/mujoco-3.3.5/lib/libmujoco.so.3.3.5 /usr/local/lib/

# macOS:
brew install mujoco
```

### Runtime Errors

#### MuJoCo Viewer Doesn't Open

**Linux:**

```bash
# Check DISPLAY
echo $DISPLAY  # Should show :0 or :1

# Allow X server connections
xhost +local:

# Install OpenGL libraries
sudo apt install libglew-dev libglfw3-dev mesa-utils

# Test OpenGL
glxinfo | grep "OpenGL version"
```

**macOS:**

```bash
# Ensure XQuartz is NOT running
pkill XQuartz

# MuJoCo uses native OpenGL on macOS
```

#### `error while loading shared libraries: libmujoco.so.3.3.5`

**Solution:**

```bash
# Linux: Update library cache
sudo ldconfig

# Or add to LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Make permanent: add to ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
```

#### Python Import Errors

**Solution:**

```bash
# Ensure you're in virtual environment
source venv/bin/activate

# Reinstall Python package
cd python
pip install -e ".[dev]" --force-reinstall

# Check Python version (needs ≥3.8)
python --version
```

### Performance Issues

#### Simulation Running Slowly

**Check CPU usage:**

```bash
# Monitor CPU while running
htop  # or top

# If low CPU usage, may be rendering-limited
```

**Disable rendering for faster simulation:**

```cpp
// In your code
sim.set_rendering(false);  // Headless mode
```

**Use Release build:**

```bash
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

---

## Manual Installation

If the automated script fails, you can install dependencies manually.

### Manual Dependency Installation

See the [setup_dependencies.sh](../scripts/setup_dependencies.sh) script for exact versions and build commands.

**General pattern:**

```bash
# Example: Installing library from source
cd /tmp
git clone <repository_url>
cd <library_name>
git checkout <version_tag>
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

**Key libraries to install (in order):**

1. Eigen (≥3.3.0)
2. Boost (≥1.65)
3. MuJoCo (3.3.5)
4. Pinocchio (3.7.0)
5. yaml-cpp
6. qpOASES
7. Ruckig (v0.15.3)
8. OpenCV
9. FCL
10. LCM
11. nlohmann/json

---

## Uninstallation

### Remove RynnMotion

```bash
# Remove build artifacts
cd RynnMotion
rm -rf build/

# Remove Python package
cd python
pip uninstall rynnmotion

# Remove virtual environment
rm -rf venv/

# Remove repository (if desired)
cd ..
rm -rf RynnMotion/
```

### Remove Dependencies

**⚠️ Warning:** Only do this if you're sure no other projects need these libraries.

```bash
# Linux: Remove installed libraries
sudo rm -rf /usr/local/mujoco-3.3.5
sudo rm -rf /usr/local/include/pinocchio
sudo rm -rf /usr/local/lib/libpinocchio*
# ... (repeat for other libraries)

# macOS: Use Homebrew
brew uninstall eigen boost mujoco opencv
```

---

## Next Steps

After successful installation:

1. **Run the Quick Start Tutorial:** [tutorials/quickstart.md](../tutorials/quickstart.md)
2. **Explore Examples:** [examples/](../examples/)
3. **Read Architecture Documentation:** [ARCHITECTURE.md](ARCHITECTURE.md)
4. **Join the Community:** [GitHub Discussions](https://github.com/alibaba-damo-academy/RynnMotion/discussions)

---

## Getting Help

If you encounter issues not covered in this guide:

1. **Check existing issues:** [GitHub Issues](https://github.com/alibaba-damo-academy/RynnMotion/issues)
2. **Search discussions:** [GitHub Discussions](https://github.com/alibaba-damo-academy/RynnMotion/discussions)
3. **Ask for help:** Open a new issue with:
   - Your OS and version
   - Output of `cmake ..` and `make`
   - Full error messages
   - Steps to reproduce

---

## License

Apache License 2.0 - See [LICENSE](../LICENSE) for details.
