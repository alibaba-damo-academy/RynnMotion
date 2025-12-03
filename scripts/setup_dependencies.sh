#!/bin/bash

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================================="
echo "RynnMotion - Cross-Platform C++ Dependency Installation"
echo "=========================================================="
echo ""
echo "Version Compatibility:"
echo "  - Eigen:      >= 3.3.0"
echo "  - Pinocchio:  v3.7.0"
echo "  - Boost:      >= 1.65"
echo "  - MuJoCo:     3.3.5"
echo ""
echo "Prerequisites:"
echo "  Linux:  Run with sudo"
echo "  macOS:  Run as regular user (not sudo)"
echo "=========================================================="
echo

FAST_SETUP="n"
if [[ "$FAST_SETUP" == "n" ]]; then
  echo "⚠️  Warning: Unverified feature: Enabling fast Setup may result in incomplete installation of dependencies"
  echo "Fast Setup skips re-downloading if dependencies are already present."
  read -p "Enable fast setup? (y/n, default: n): " -n 1 -r FAST_SETUP_INPUT
  echo
  if [[ $FAST_SETUP_INPUT =~ ^[Yy]$ ]]; then
    FAST_SETUP="y"
  else
    FAST_SETUP="n"
  fi
fi

echo "Fast Setup: $([ "$FAST_SETUP" == "y" ] && echo "Enabled" || echo "Disabled")"
echo

# Detect OS
OS_TYPE=""
if [[ "$(uname)" == "Darwin" ]]; then
    OS_TYPE="macos"
    echo "Detected: macOS"
elif [[ "$(uname)" == "Linux" ]]; then
    OS_TYPE="linux"
    echo "Detected: Linux"
else
    echo "Unsupported OS: $(uname)"
    exit 1
fi

echo ""
echo "=========================================================="
echo "Checking for Python (Optional)"
echo "=========================================================="

if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
    echo "✓ Found Python: $(python3 --version)"
else
    if [[ "$OS_TYPE" == "macos" ]]; then
        echo "⚠️  Warning: Python not found. Some Homebrew packages may require it."
        echo "   Install with: brew install python@3.10"
    else
        echo "✓ Python not required for C++ library installation"
    fi
fi
echo ""

check_eigen_conflicts() {
    echo ""
    echo "=========================================================="
    echo "Checking Eigen Installation"
    echo "=========================================================="

    local usr_local_eigen=""
    local system_eigen=""

    if [ -d "/usr/local/include/eigen3" ]; then
        usr_local_eigen=$(grep "EIGEN_WORLD_VERSION\|EIGEN_MAJOR_VERSION\|EIGEN_MINOR_VERSION" /usr/local/include/eigen3/Eigen/src/Core/util/Macros.h 2>/dev/null | grep "define" | awk '{print $3}' | tr '\n' '.' | sed 's/\.$//')
        if [ -n "$usr_local_eigen" ]; then
            echo "✗ ERROR: Found conflicting Eigen $usr_local_eigen in /usr/local/include/eigen3"
            echo ""
            echo "This script requires Eigen to be installed via system package manager only."
            echo "Conflicting installations in /usr/local can cause build failures."
            echo ""
            echo "Please remove the old installation:"
            if [[ "$OS_TYPE" == "linux" ]]; then
                echo "  sudo mv /usr/local/include/eigen3 /usr/local/include/eigen3.backup"
                echo "  sudo rm -rf /usr/local/share/eigen3"
                echo "  sudo rm -f /usr/local/share/pkgconfig/eigen3.pc"
            else
                echo "  sudo mv /usr/local/include/eigen3 /usr/local/include/eigen3.backup"
            fi
            echo ""
            exit 1
        fi
    fi

    if [[ "$OS_TYPE" == "linux" ]]; then
        if [ -d "/usr/include/eigen3" ]; then
            system_eigen=$(grep "EIGEN_WORLD_VERSION\|EIGEN_MAJOR_VERSION\|EIGEN_MINOR_VERSION" /usr/include/eigen3/Eigen/src/Core/util/Macros.h 2>/dev/null | grep "define" | awk '{print $3}' | tr '\n' '.' | sed 's/\.$//')
            if [ -n "$system_eigen" ]; then
                echo "✓ Found Eigen $system_eigen (via apt)"
            fi
        else
            echo "✗ ERROR: Eigen not found. Install via: apt-get install libeigen3-dev"
            exit 1
        fi
    elif [[ "$OS_TYPE" == "macos" ]]; then
        system_eigen=$(pkg-config --modversion eigen3 2>/dev/null || echo "")
        if [ -n "$system_eigen" ]; then
            echo "✓ Found Eigen $system_eigen (via Homebrew)"
        else
            echo "✗ ERROR: Eigen not found. Install via: brew install eigen"
            exit 1
        fi
    fi

    local major=$(echo "$system_eigen" | cut -d. -f1)
    local minor=$(echo "$system_eigen" | cut -d. -f2)

    if [ "$major" -lt 3 ] || ([ "$major" -eq 3 ] && [ "$minor" -lt 3 ]); then
        echo "✗ ERROR: Eigen version $system_eigen is too old"
        echo "  eigenpy v3.10.3 and Pinocchio v3.7.0 require Eigen >= 3.3.0"
        exit 1
    fi

    echo "✓ Eigen version $system_eigen is compatible"
    echo ""
}

init_platform() {
    if [[ "$OS_TYPE" == "linux" ]]; then
        init_linux
    elif [[ "$OS_TYPE" == "macos" ]]; then
        init_macos
    fi
}

init_linux() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "This script needs to be run with sudo privileges on Linux."
        exit 1
    fi

    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS=$NAME
        VER=$VERSION_ID
    else
        echo "Cannot detect OS. This script requires Ubuntu 20.04 or newer."
        exit 1
    fi

    if [[ "$OS" != *"Ubuntu"* ]]; then
        echo "This script is designed for Ubuntu. Detected OS: $OS"
        echo "You may need to modify this script for your distribution."
        read -p "Continue anyway? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi

    echo "Installing base build tools..."
    apt-get update || echo "apt-get update failed, but continuing script..."

    apt-get install -y build-essential cmake git

    echo "Installing basic system dependencies..."
    apt-get install -y libgflags-dev xorg-dev libgl1-mesa-dev libglib2.0-dev libccd-dev

    echo "Installing HDF5 and FFmpeg (for data recording)..."
    apt-get install -y libhdf5-dev libavcodec-dev libavformat-dev libswscale-dev libavutil-dev

    echo "Installing FCL-specific dependencies..."
    apt-get install -y libfcl-dev || echo "FCL not available via apt, will build from source"
    apt-get install -y liboctomap-dev libccd-dev libassimp-dev

    echo "Installing GLFW..."
    apt-get install -y libglfw3 libglfw3-dev
}

init_macos() {
    if [ "$(id -u)" -eq 0 ]; then
        echo "This script should NOT be run with sudo on macOS."
        echo "Please run it as a regular user."
        exit 1
    fi

    if ! command -v brew &> /dev/null; then
        echo "Homebrew is not installed. Please install Homebrew first:"
        echo '/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"'
        exit 1
    fi

    HOMEBREW_PREFIX=$(brew --prefix)
    echo "Using Homebrew prefix: $HOMEBREW_PREFIX"

    echo "Updating Homebrew..."
    brew update || echo "brew update failed, but continuing script..."

    echo "Installing basic build tools and dependencies..."
    brew install git pkg-config cmake ninja

    if brew list qt &> /dev/null && brew list qtbase &> /dev/null; then
        echo "Detected both qt and qtbase installed, unlinking qt to resolve conflicts..."
        brew unlink qt || echo "Warning: Failed to unlink qt"
        brew link qtbase || echo "Warning: Failed to link qtbase"
    fi

    echo "Installing GLFW..."
    brew install glfw
    echo "GLFW installed via Homebrew."

    echo "Installing HDF5 and FFmpeg (for data recording)..."
    brew install hdf5 ffmpeg

    echo ""
    echo "=========================================================="
    echo "Checking for macOS compatibility issues..."
    echo "=========================================================="

    CURRENT_SDK=$(xcrun --sdk macosx --show-sdk-path 2>/dev/null || echo "")
    if [ -n "$CURRENT_SDK" ]; then
        echo "Current macOS SDK: $CURRENT_SDK"
        SDK_VERSION=$(echo "$CURRENT_SDK" | grep -o "MacOSX[0-9]*\.[0-9]*" | sed 's/MacOSX//')
        echo "SDK version: $SDK_VERSION"
    else
        echo "Could not detect current SDK"
    fi

    echo ""
    echo "Checking Homebrew library versions..."
    NEEDS_REINSTALL=false
    REINSTALL_REASON=""

    if brew list protobuf &> /dev/null && brew list opencv &> /dev/null; then
        PROTOBUF_VERSION=$(brew list --versions protobuf | awk '{print $2}' | head -1)
        OPENCV_VERSION=$(brew list --versions opencv | awk '{print $2}' | head -1)
        echo "  - protobuf: $PROTOBUF_VERSION"
        echo "  - opencv: $OPENCV_VERSION"

        OPENCV_LIB_PATH="/opt/homebrew/Cellar/opencv/$OPENCV_VERSION/lib/libopencv_dnn.*.dylib"
        OPENCV_PROTOBUF_DEP=$(otool -L $OPENCV_LIB_PATH 2>/dev/null | grep protobuf | head -1 || echo "")

        if [ -n "$OPENCV_PROTOBUF_DEP" ]; then
            LINKED_PROTOBUF=$(echo "$OPENCV_PROTOBUF_DEP" | grep -o "libprotobuf\.[0-9]*\.[0-9]*\.[0-9]*\.dylib" || echo "")
            if [ -n "$LINKED_PROTOBUF" ]; then
                EXPECTED_PROTOBUF=$(ls /opt/homebrew/opt/protobuf/lib/libprotobuf.*.dylib 2>/dev/null | head -1 | xargs basename)
                if [ "$LINKED_PROTOBUF" != "$EXPECTED_PROTOBUF" ]; then
                    NEEDS_REINSTALL=true
                    REINSTALL_REASON="${REINSTALL_REASON}\n  - OpenCV linked to old protobuf ($LINKED_PROTOBUF vs $EXPECTED_PROTOBUF)"
                fi
            fi
        fi
    fi

    echo ""
    echo "Note: After macOS/Homebrew updates, packages may need reinstallation due to:"
    echo "  - macOS SDK updates (SDK path changes)"
    echo "  - Homebrew dependency upgrades (e.g., protobuf version mismatch)"
    echo ""
    echo "The following packages are known to cause issues:"
    echo "  - libccd (SDK-dependent, used by FCL)"
    echo "  - fcl (SDK-dependent, collision library)"
    echo "  - opencv (depends on protobuf, SDK-dependent)"
    echo ""

    if [ "$NEEDS_REINSTALL" = true ]; then
        echo "⚠️  Detected compatibility issues:"
        echo -e "$REINSTALL_REASON"
        echo ""
        echo "Reinstallation is STRONGLY recommended."
        echo ""
    fi

    read -p "Reinstall libccd, FCL, and OpenCV to fix compatibility? (recommended: y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo ""
        echo "Reinstalling packages for compatibility..."
        echo "This may take a few minutes..."

        if brew list qt &> /dev/null; then
            echo "  → Unlinking qt to prevent conflicts..."
            brew unlink qt || echo "Warning: Failed to unlink qt"
        fi

        if brew list qtbase &> /dev/null; then
            echo "  → Linking qtbase..."
            brew link qtbase || echo "Warning: qtbase already linked or conflicts exist"
        fi

        if brew list libccd &> /dev/null; then
            echo "  → Reinstalling libccd..."
            brew reinstall libccd || echo "Warning: libccd reinstall failed"
        fi

        if brew list fcl &> /dev/null; then
            echo "  → Reinstalling fcl..."
            brew reinstall fcl || echo "Warning: fcl reinstall failed"
        fi

        if brew list opencv &> /dev/null; then
            echo "  → Reinstalling opencv (against current protobuf)..."
            brew reinstall opencv || echo "Warning: opencv reinstall failed"
        fi

        echo "✓ Packages reinstalled for compatibility"
    else
        echo ""
        echo "⚠️  WARNING: Skipping compatibility fix."
        echo "   You may encounter errors like:"
        echo "   - 'No rule to make target /Library/.../MacOSXXX.sdk/usr/lib/libm.tbd'"
        echo "   - 'Library not loaded: .../libprotobuf.XX.X.X.dylib'"
        echo ""
        echo "   If you encounter issues, run:"
        echo "   brew unlink qt && brew link qtbase && brew reinstall libccd fcl opencv"
    fi
}

BUILD_DIR="$SCRIPT_DIR/deps_build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

check_pinocchio_installed() {
    if [[ "$OS_TYPE" == "linux" ]]; then
        if pkg-config --exists pinocchio 2>/dev/null; then
            INSTALLED_VERSION=$(pkg-config --modversion pinocchio 2>/dev/null || echo "")
            if [[ "$INSTALLED_VERSION" == "3.7.0" ]]; then
                echo "✓ Pinocchio v3.7.0 is already installed"
                return 0
            else
                echo "⚠️  Found Pinocchio $INSTALLED_VERSION, but need v3.7.0"
                return 1
            fi
        else
            echo "Pinocchio not found in system"
            return 1
        fi
    elif [[ "$OS_TYPE" == "macos" ]]; then
        if brew list pinocchio &> /dev/null; then
            INSTALLED_VERSION=$(brew list --versions pinocchio | awk '{print $2}')
            if [[ "$INSTALLED_VERSION" == "3.7.0" ]]; then
                echo "✓ Pinocchio v3.7.0 is already installed via Homebrew"
                return 0
            else
                echo "⚠️  Found Pinocchio $INSTALLED_VERSION via Homebrew, but need v3.7.0"
                return 1
            fi
        else
            echo "Pinocchio not found in Homebrew"
            return 1
        fi
    fi
}

check_pinocchio_source() {
    local pinocchio_dir="$BUILD_DIR/pinocchio"
    
    if [ ! -d "$pinocchio_dir" ]; then
        echo "Pinocchio source not found in deps_build"
        return 1
    fi

    cd "$pinocchio_dir"

    if [ ! -d ".git" ]; then
        echo "Pinocchio directory is not a git repository"
        cd "$BUILD_DIR"
        return 1
    fi

    local current_tag=$(git describe --tags --exact-match 2>/dev/null || echo "")
    
    if [[ "$current_tag" == "v3.7.0" ]]; then
        echo "✓ Pinocchio source in deps_build is already at v3.7.0"
        cd "$BUILD_DIR"
        return 0
    else
        echo "Pinocchio source in deps_build is not at v3.7.0 (current: $current_tag)"
        cd "$BUILD_DIR"
        return 1
    fi
}

install_from_github() {
    local repo_url=$1
    local dir_name=$2
    local branch=${3:-master}
    local build_options=${4:-""}
    local skip_download=${5:-"false"}

    echo "Installing $dir_name from $repo_url (branch: $branch)..."

    if [[ "$FAST_SETUP" == "y" && "$skip_download" == "true" && -d "$dir_name" ]]; then
        echo "Fast setup: Using existing $dir_name directory"
    else
        if [ ! -d "$dir_name" ]; then
            git clone --recursive "$repo_url" "$dir_name"
        fi

        cd "$dir_name"

        if [[ "$branch" =~ ^v[0-9]+\.[0-9]+\.[0-9]+ ]]; then
            echo "Checking out tag $branch..."
            git fetch --tags
            git checkout "$branch"
        else
            echo "Checking out branch $branch..."
            git checkout "$branch"
            git pull origin "$branch" || git pull
        fi

        if [ -f ".gitmodules" ]; then
            git submodule init
            git submodule update --recursive
        fi
    fi

    if [[ "$(basename "$PWD")" != "$dir_name" ]]; then
        cd "$dir_name"
    fi

    mkdir -p build
    cd build

    if [[ "$OS_TYPE" == "macos" ]]; then
        if [[ "$dir_name" == "pinocchio" ]]; then
            echo "Building Pinocchio with memory-conservative settings..."
            AVAILABLE_MEM_GB=$(sysctl hw.memsize | awk '{print int($2/1024/1024/1024)}')
            SAFE_JOBS=$(( AVAILABLE_MEM_GB / 4 ))
            if [ $SAFE_JOBS -lt 1 ]; then
                SAFE_JOBS=1
            fi
            if [ $SAFE_JOBS -gt 4 ]; then
                SAFE_JOBS=4
            fi
            echo "Using make -j$SAFE_JOBS for Pinocchio"
            cmake -DCMAKE_INSTALL_PREFIX="$HOMEBREW_PREFIX" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2" $build_options ..
            make -j$SAFE_JOBS
            sudo make install
        else
            cmake -DCMAKE_INSTALL_PREFIX="$HOMEBREW_PREFIX" $build_options ..
            local num_jobs=$(sysctl -n hw.ncpu)
            make -j$num_jobs
            sudo make install
        fi
    else
        if [[ "$dir_name" == "pinocchio" ]]; then
            echo "Building Pinocchio with memory-conservative settings..."
            AVAILABLE_MEM_GB=$(free -g | awk '/^Mem:/{print $7}')
            SAFE_JOBS=$(( AVAILABLE_MEM_GB / 4 ))
            if [ $SAFE_JOBS -lt 1 ]; then
                SAFE_JOBS=1
            fi
            if [ $SAFE_JOBS -gt 4 ]; then
                SAFE_JOBS=4
            fi
            echo "Using make -j$SAFE_JOBS for Pinocchio (based on $AVAILABLE_MEM_GB GB available memory)"

            cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O2" $build_options ..

            nice -n 10 make -j$SAFE_JOBS
            make install
        else
            cmake $build_options ..
            local num_jobs=$(($(nproc) / 2))
            if [ $num_jobs -lt 1 ]; then
                num_jobs=1
            fi
            echo "Building with $num_jobs parallel jobs to avoid memory issues..."
            make -j$num_jobs
            make install
        fi
    fi

    cd "$BUILD_DIR"
    echo "$dir_name installed successfully."
    echo
}

install_boost() {
    echo "Checking Boost installation..."
    if [[ "$OS_TYPE" == "linux" ]]; then
        if ! dpkg -l | grep -q libboost-all-dev; then
            echo "Installing Boost libraries..."
            apt-get install -y libboost-all-dev
            echo "Boost installed successfully."
        else
            echo "Boost is already installed."
        fi
    elif [[ "$OS_TYPE" == "macos" ]]; then
        if ! brew list boost &> /dev/null; then
            echo "Installing Boost libraries..."
            brew install boost
            echo "Boost installed successfully."
        else
            echo "Boost is already installed."
        fi
    fi
}

install_eigen() {
    if [[ "$OS_TYPE" == "linux" ]]; then
        echo "Installing Eigen3 via apt..."
        apt-get install -y libeigen3-dev
    elif [[ "$OS_TYPE" == "macos" ]]; then
        echo "Installing Eigen3 via Homebrew..."
        brew install eigen
    fi
}

install_pinocchio() {
    if [[ "$FAST_SETUP" == "y" ]]; then
        if check_pinocchio_installed; then
            echo "Skipping Pinocchio installation (already installed)"
            return 0
        fi
    fi

    if [[ "$OS_TYPE" == "linux" ]]; then
        echo "Installing Pinocchio v3.7.0 from source..."
        echo ""

        echo "Installing Pinocchio dependencies..."
        apt-get install -y \
            libboost-all-dev \
            liburdfdom-dev \
            libeigen3-dev \
            libtinyxml2-dev \
            libconsole-bridge-dev

        local skip_download="false"
        if [[ "$FAST_SETUP" == "y" ]]; then
            if check_pinocchio_source; then
                skip_download="true"
            fi
        fi

        echo "Building Pinocchio with memory-conservative settings..."

        AVAILABLE_MEM_GB=$(free -g | awk '/^Mem:/{print $7}')
        SAFE_JOBS=$(( AVAILABLE_MEM_GB / 4 ))
        if [ $SAFE_JOBS -lt 1 ]; then
            SAFE_JOBS=1
        fi
        if [ $SAFE_JOBS -gt 4 ]; then
            SAFE_JOBS=4
        fi

        echo "Using $SAFE_JOBS parallel jobs for Pinocchio (based on $AVAILABLE_MEM_GB GB available memory)"

        install_from_github "https://github.com/stack-of-tasks/pinocchio.git" "pinocchio" "v3.7.0" \
            "-DCMAKE_BUILD_TYPE=Release \
             -DBUILD_PYTHON_INTERFACE=OFF \
             -DBUILD_WITH_URDF_SUPPORT=ON \
             -DBUILD_WITH_COLLISION_SUPPORT=OFF \
             -DBUILD_TESTING=OFF \
             -DBUILD_ADVANCED_TESTING=OFF \
             -DBUILD_BENCHMARK=OFF \
             -DCMAKE_CXX_FLAGS=\"-O2\" \
             -DPINOCCHIO_JOBS=$SAFE_JOBS" \
            "$skip_download"

        ldconfig

        echo "✓ Pinocchio v3.7.0 installed successfully from source"

    elif [[ "$OS_TYPE" == "macos" ]]; then
        echo "Installing Pinocchio via Homebrew..."

        if brew list pinocchio &> /dev/null; then
            echo "✓ Pinocchio is already installed via Homebrew"
            CURRENT_VERSION=$(brew list --versions pinocchio | awk '{print $2}')
            echo "  Current version: $CURRENT_VERSION"

            echo "Checking for Pinocchio updates..."
            brew upgrade pinocchio || echo "Pinocchio is already up to date"
        else
            echo "Installing Pinocchio dependencies..."
            brew install boost eigen urdfdom tinyxml2 console_bridge

            brew install pinocchio
            echo "✓ Pinocchio installed via Homebrew (with URDF support)"
        fi

        SHELL_RC="$HOME/.zshrc"
        if [[ "$SHELL" == *"bash"* ]]; then
            SHELL_RC="$HOME/.bash_profile"
        fi

        if [ -d "$HOMEBREW_PREFIX/opt/pinocchio" ]; then
            if ! grep -q "PKG_CONFIG_PATH.*pinocchio" "$SHELL_RC" 2>/dev/null; then
                echo "export PKG_CONFIG_PATH=\"$HOMEBREW_PREFIX/opt/pinocchio/lib/pkgconfig:\$PKG_CONFIG_PATH\"" >> "$SHELL_RC"
                echo "export CMAKE_PREFIX_PATH=\"$HOMEBREW_PREFIX/opt/pinocchio:\$CMAKE_PREFIX_PATH\"" >> "$SHELL_RC"
                echo "✓ Added Pinocchio paths to $SHELL_RC"
            else
                echo "✓ Pinocchio environment already configured"
            fi
        fi

        echo "  Note: Run 'source $SHELL_RC' to update your environment"
    fi
}

install_fcl() {
    if [[ "$OS_TYPE" == "linux" ]]; then
        # Install FCL with more conservative build settings
        echo "Installing FCL from source..."
        install_from_github "https://github.com/flexible-collision-library/fcl.git" "fcl" "master" "-DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DFCL_WITH_OCTOMAP=ON"
    elif [[ "$OS_TYPE" == "macos" ]]; then
        echo "Installing FCL via Homebrew..."
        brew install fcl
        echo "FCL installed via Homebrew."
    fi
}

install_nlohmann_json() {
    if [[ "$OS_TYPE" == "linux" ]]; then
        echo "Installing nlohmann/json from source..."
        install_from_github "https://github.com/nlohmann/json.git" "json" "master" "-DBUILD_SHARED_LIBS=ON -DJSON_BuildTests=OFF"
    elif [[ "$OS_TYPE" == "macos" ]]; then
        echo "Installing nlohmann/json via Homebrew..."
        brew install nlohmann-json
        echo "nlohmann/json installed via Homebrew."
    fi
}

install_optional_package() {
    local package_name=$1
    local apt_package=$2
    local brew_package=$3
    local github_url=$4
    local github_options=${5:-""}

    if [[ "$OS_TYPE" == "linux" ]]; then
        apt-get install -y "$apt_package"
        echo "✓ $package_name installed from apt."
    elif [[ "$OS_TYPE" == "macos" ]]; then
        if brew list "$brew_package" &> /dev/null; then
            echo "✓ $package_name is already installed"
        else
            if [[ "$brew_package" == "opencv" ]]; then
                if brew list qt &> /dev/null && ! brew list qtbase &> /dev/null; then
                    echo "Unlinking qt to prevent conflicts with qtbase (opencv dependency)..."
                    brew unlink qt || echo "Warning: Failed to unlink qt"
                fi
            fi
            brew install "$brew_package" || echo "Warning: $package_name installation encountered issues"
            if [[ "$brew_package" == "opencv" ]] && brew list qtbase &> /dev/null; then
                brew link qtbase || echo "Note: qtbase link conflicts with qt, but opencv should work"
            fi
            echo "✓ $package_name installed from Homebrew."
        fi
    fi
}

install_mujoco_linux() {
    local MUJOCO_VERSION=""
    local MUJOCO_PERMANENT_DIR=""

    for version in "3.3.5" "3.3.4" "3.3.3" "3.3.2" "3.3.1" "3.3.0"; do
        if [ -d "/usr/local/mujoco-${version}" ]; then
            echo "✓ Found existing MuJoCo installation at: /usr/local/mujoco-${version}"
            MUJOCO_VERSION="$version"
            MUJOCO_PERMANENT_DIR="/usr/local/mujoco-${version}"
            break
        fi
    done

    if [ -n "$MUJOCO_PERMANENT_DIR" ]; then
        echo "✓ Using existing MuJoCo installation: $MUJOCO_PERMANENT_DIR"
        echo ""
        if [ "$MUJOCO_VERSION" != "3.3.5" ]; then
            echo "Note: You have MuJoCo $MUJOCO_VERSION, but the recommended version is 3.3.5."
            read -p "Would you like to install MuJoCo 3.3.5? (y/n) " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                echo "Continuing with existing MuJoCo $MUJOCO_VERSION installation."
                return 0
            fi
            echo "Proceeding to install MuJoCo 3.3.5..."
        else
            echo "You already have the recommended version (3.3.5)."
            return 0
        fi
    fi

    echo "MuJoCo 3.3.5 needs to be installed."
    echo ""
    echo "Choose installation method:"
    echo "1. Automatic download and install (recommended)"
    echo "2. Manual download and extract"
    echo ""
    read -p "Select option (1 or 2): " -n 1 -r
    echo

    if [[ $REPLY == "1" ]]; then
        echo "Downloading MuJoCo 3.3.5..."

        # Detect architecture
        ARCH=$(uname -m)
        if [[ "$ARCH" == "x86_64" ]]; then
            MUJOCO_ARCH="linux-x86_64"
        elif [[ "$ARCH" == "aarch64" ]]; then
            MUJOCO_ARCH="linux-aarch64"
        else
            echo "Warning: Unsupported architecture $ARCH, trying x86_64 version..."
            MUJOCO_ARCH="linux-x86_64"
        fi

        DOWNLOAD_URL="https://github.com/google-deepmind/mujoco/releases/download/3.3.5/mujoco-3.3.5-${MUJOCO_ARCH}.tar.gz"
        DOWNLOAD_FILE="/tmp/mujoco-3.3.5-${MUJOCO_ARCH}.tar.gz"
        EXTRACT_DIR="/tmp/mujoco-3.3.5-extract"

        if wget -O "$DOWNLOAD_FILE" "$DOWNLOAD_URL" || curl -L -o "$DOWNLOAD_FILE" "$DOWNLOAD_URL"; then
            echo "✓ Download completed successfully"
            echo "Extracting MuJoCo..."

            mkdir -p "$EXTRACT_DIR"
            if tar -xzf "$DOWNLOAD_FILE" -C "$EXTRACT_DIR" --strip-components=1; then
                echo "✓ Extraction completed successfully"

                echo "Installing MuJoCo 3.3.5 to /usr/local/mujoco-3.3.5..."
                mkdir -p /usr/local/mujoco-3.3.5
                cp -r "$EXTRACT_DIR"/* /usr/local/mujoco-3.3.5/

                if [ -f "/usr/local/mujoco-3.3.5/lib/libmujoco.so.3.3.5" ] && [ -d "/usr/local/mujoco-3.3.5/include" ]; then
                    echo "✓ MuJoCo 3.3.5 installed successfully!"
                    echo "  Library: /usr/local/mujoco-3.3.5/lib/"
                    echo "  Headers: /usr/local/mujoco-3.3.5/include/"

                    # Update library cache
                    ldconfig

                    # Cleanup
                    echo "Cleaning up..."
                    rm -rf "$EXTRACT_DIR"
                    rm -f "$DOWNLOAD_FILE"
                    echo "✓ Temporary files cleaned up"

                    echo "✓ MuJoCo is permanently installed and ready to use."
                    echo ""

                    # Configure library path automatically
                    read -p "Would you like to automatically add MuJoCo to your library path? (y/n) " -n 1 -r
                    echo
                    if [[ $REPLY =~ ^[Yy]$ ]]; then
                        # Check if the path is already in bashrc
                        if ! grep -q "LD_LIBRARY_PATH.*mujoco-3.3.5" ~/.bashrc 2>/dev/null; then
                            echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/mujoco-3.3.5/lib' >> ~/.bashrc
                            echo "✓ Added MuJoCo library path to ~/.bashrc"
                            echo "Please run: source ~/.bashrc"
                        else
                            echo "✓ MuJoCo library path already configured"
                        fi
                    else
                        echo "Add manually to your ~/.bashrc:"
                        echo "   echo 'export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/mujoco-3.3.5/lib' >> ~/.bashrc"
                        echo "   source ~/.bashrc"
                    fi
                else
                    echo "✗ Error: MuJoCo installation failed."
                    echo "Please check the download and try again."
                    return 1
                fi
            else
                echo "✗ Error: Failed to extract MuJoCo archive."
                return 1
            fi
        else
            echo "✗ Error: Download failed"
            echo "Please download manually from: $DOWNLOAD_URL"
            echo "Extract to /usr/local/mujoco-3.3.5"
            return 1
        fi
    elif [[ $REPLY == "2" ]]; then
        echo "Please follow these manual steps:"
        echo "1. Download MuJoCo 3.3.5 from https://github.com/google-deepmind/mujoco/releases"
        echo "   Direct link: https://github.com/google-deepmind/mujoco/releases/download/3.3.5/mujoco-3.3.5-linux-x86_64.tar.gz"
        echo "2. Extract: tar -xzf mujoco-3.3.5-linux-x86_64.tar.gz"
        echo "3. Move: sudo mv mujoco-3.3.5 /usr/local/mujoco-3.3.5"
        echo "4. Update library path:"
        echo "   echo 'export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/mujoco-3.3.5/lib' >> ~/.bashrc"
        echo "   source ~/.bashrc"
        echo "5. Run: sudo ldconfig"
    else
        echo "Invalid option. Skipping MuJoCo installation."
    fi
}

install_mujoco_macos() {
    local DMG_MOUNT_PATH="/Volumes/MuJoCo"
    local MUJOCO_VERSION=""
    local MUJOCO_PERMANENT_DIR=""

    # Check if MuJoCo is already installed
    for version in "3.3.5" "3.3.4" "3.3.3" "3.3.2" "3.3.1" "3.3.0"; do
        if [ -d "/usr/local/mujoco-${version}" ]; then
            echo "✓ Found existing MuJoCo installation at: /usr/local/mujoco-${version}"
            MUJOCO_VERSION="$version"
            MUJOCO_PERMANENT_DIR="/usr/local/mujoco-${version}"
            break
        fi
    done

    if [ -n "$MUJOCO_PERMANENT_DIR" ]; then
        echo "✓ Using existing MuJoCo installation: $MUJOCO_PERMANENT_DIR"
        echo ""
        if [ "$MUJOCO_VERSION" != "3.3.5" ]; then
            echo "Note: You have MuJoCo $MUJOCO_VERSION, but the recommended version is 3.3.5."
            read -p "Would you like to install MuJoCo 3.3.5? (y/n) " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                return 0
            fi
            echo "Proceeding to install MuJoCo 3.3.5..."
        else
            echo "You already have the recommended version (3.3.5)."
            return 0
        fi
    fi

    # Check if DMG is mounted
    if [ ! -d "$DMG_MOUNT_PATH" ]; then
        echo "MuJoCo 3.3.5 needs to be installed."
        echo ""
        echo "Choose installation method:"
        echo "1. Automatic download and install (recommended)"
        echo "2. Manual download and mount"
        echo ""
        read -p "Select option (1 or 2): " -n 1 -r
        echo

        if [[ $REPLY == "1" ]]; then
            echo "Downloading MuJoCo 3.3.5..."
            DOWNLOAD_URL="https://github.com/google-deepmind/mujoco/releases/download/3.3.5/mujoco-3.3.5-macos-universal2.dmg"
            DOWNLOAD_FILE="/tmp/mujoco-3.3.5-macos-universal2.dmg"

            if curl -L -o "$DOWNLOAD_FILE" "$DOWNLOAD_URL"; then
                echo "✓ Download completed successfully"
                echo "Mounting DMG..."
                hdiutil attach "$DOWNLOAD_FILE" -quiet
                if [ ! -d "$DMG_MOUNT_PATH" ]; then
                    echo "✗ Error: Failed to mount DMG"
                    exit 1
                fi
                echo "✓ DMG mounted successfully"
            else
                echo "✗ Error: Download failed"
                echo "Please download manually from: $DOWNLOAD_URL"
                exit 1
            fi
        elif [[ $REPLY == "2" ]]; then
            echo "Please follow these steps:"
            echo "1. Download MuJoCo 3.3.5 .dmg from https://github.com/google-deepmind/mujoco/releases"
            echo "   Direct link: https://github.com/google-deepmind/mujoco/releases/download/3.3.5/mujoco-3.3.5-macos-universal2.dmg"
            echo "2. Mount the .dmg file (double-click)"
            echo "3. Re-run this script to install it permanently"
            echo ""
            read -p "Have you mounted the MuJoCo DMG? (y/n) " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                exit 1
            fi
        else
            echo "Invalid option. Exiting."
            exit 1
        fi
    fi

    if [ -d "$DMG_MOUNT_PATH/mujoco.framework" ]; then
        for version in "3.3.5" "3.3.4" "3.3.3" "3.3.2" "3.3.1" "3.3.0"; do
            if [ -f "$DMG_MOUNT_PATH/mujoco.framework/Versions/A/libmujoco.${version}.dylib" ]; then
                MUJOCO_VERSION="$version"
                break
            fi
        done

        if [ -z "$MUJOCO_VERSION" ]; then
            MUJOCO_VERSION="3.3.5"
        fi

        MUJOCO_PERMANENT_DIR="/usr/local/mujoco-${MUJOCO_VERSION}"

        echo "Installing MuJoCo ${MUJOCO_VERSION} permanently to: $MUJOCO_PERMANENT_DIR"

        sudo mkdir -p "$MUJOCO_PERMANENT_DIR"
        sudo chown $(whoami):staff "$MUJOCO_PERMANENT_DIR"

        echo "Copying MuJoCo framework..."
        mkdir -p "$MUJOCO_PERMANENT_DIR/lib"
        mkdir -p "$MUJOCO_PERMANENT_DIR/include"

        cp "$DMG_MOUNT_PATH/mujoco.framework/Versions/A/"*.dylib "$MUJOCO_PERMANENT_DIR/lib/" 2>/dev/null || true
        cp -R "$DMG_MOUNT_PATH/mujoco.framework/Versions/A/Headers/"* "$MUJOCO_PERMANENT_DIR/include/" 2>/dev/null || true

        if [ -f "$MUJOCO_PERMANENT_DIR/lib/libmujoco.${MUJOCO_VERSION}.dylib" ] && [ -d "$MUJOCO_PERMANENT_DIR/include" ]; then
            echo "✓ MuJoCo ${MUJOCO_VERSION} installed successfully!"
            echo "  Library: $MUJOCO_PERMANENT_DIR/lib/libmujoco.${MUJOCO_VERSION}.dylib"
            echo "  Headers: $MUJOCO_PERMANENT_DIR/include/"
            echo ""

            # Unmount the DMG
            echo "Cleaning up..."
            hdiutil detach "$DMG_MOUNT_PATH" -quiet 2>/dev/null || true

            # Remove downloaded file if it exists
            if [ -f "/tmp/mujoco-3.3.5-macos-universal2.dmg" ]; then
                rm -f "/tmp/mujoco-3.3.5-macos-universal2.dmg"
                echo "✓ Downloaded DMG file cleaned up"
            fi

            echo "✓ MuJoCo is permanently installed and ready to use."
        else
            echo "✗ Error: MuJoCo installation failed."
            echo "Please check the DMG contents and try again."
            exit 1
        fi
    else
        echo "✗ Error: MuJoCo framework not found at $DMG_MOUNT_PATH/mujoco.framework"
        echo "Please ensure you have mounted the correct MuJoCo DMG."
        exit 1
    fi
}

# Main installation flow
main() {
    init_platform
    check_eigen_conflicts
    install_boost
    install_eigen
    install_pinocchio

    echo "Installing yaml-cpp..."
    install_from_github "https://github.com/jbeder/yaml-cpp.git" "yaml-cpp" "master" "-DBUILD_SHARED_LIBS=ON -DYAML_CPP_BUILD_TESTS=OFF"

    echo "Installing qpOASES..."
    if [[ "$OS_TYPE" == "macos" ]]; then
        install_from_github "https://github.com/coin-or/qpOASES.git" "qpOASES" "master" "-DBUILD_SHARED_LIBS=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DQPOASES_BUILD_EXAMPLES=OFF"
    else
        install_from_github "https://github.com/coin-or/qpOASES.git" "qpOASES" "master" "-DBUILD_SHARED_LIBS=ON -DQPOASES_BUILD_EXAMPLES=OFF"
    fi

    echo "Installing LCM (Lightweight Communications and Marshalling)..."
    if [[ "$OS_TYPE" == "macos" ]]; then
        if ! brew list lcm &> /dev/null; then
            echo "Installing LCM via Homebrew..."
            brew install glib lcm
            echo "✓ LCM installed via Homebrew"
        else
            echo "✓ LCM is already installed via Homebrew"
            LCM_VERSION=$(brew list --versions lcm | awk '{print $2}')
            echo "  Current version: $LCM_VERSION"
        fi
    else
        # Linux: Install from source (not available in apt)
        echo "Installing glib dependency..."
        apt-get install -y libglib2.0-dev
        echo "Building LCM from source..."
        install_from_github "https://github.com/lcm-proj/lcm.git" "lcm" "master" "-DLCM_ENABLE_TESTS=OFF"
    fi

    install_fcl

    install_nlohmann_json

    echo "Installing Ruckig..."
    install_from_github "https://github.com/pantor/ruckig.git" "ruckig" "v0.15.3" "-DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DBUILD_CLOUD_CLIENT=OFF -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF"

    install_optional_package "OpenCV" "libopencv-dev" "opencv" "https://github.com/opencv/opencv.git" "-DBUILD_SHARED_LIBS=ON"

    # Platform-specific post-installation
    if [[ "$OS_TYPE" == "linux" ]]; then
        ldconfig
    elif [[ "$OS_TYPE" == "macos" ]]; then
        echo "Creating Eigen symlinks if needed..."
        if [ -d "$HOMEBREW_PREFIX/include/eigen3" ] && [ ! -d "$HOMEBREW_PREFIX/include/Eigen" ]; then
            echo "Creating Eigen symlink in Homebrew prefix..."
            ln -sf "$HOMEBREW_PREFIX/include/eigen3/Eigen" "$HOMEBREW_PREFIX/include/Eigen"
        fi
        echo "Homebrew will handle library paths automatically."
    fi

    echo "=========================================================="
    echo "MuJoCo Installation:"
    echo "=========================================================="

    if [[ "$OS_TYPE" == "linux" ]]; then
        install_mujoco_linux
    elif [[ "$OS_TYPE" == "macos" ]]; then
        install_mujoco_macos
    fi

    # Platform-specific notes
    if [[ "$OS_TYPE" == "macos" ]]; then
        echo
        echo "Important Notes:"
        echo "- Your Homebrew prefix is: $HOMEBREW_PREFIX"
        echo "- Libraries are installed in: $HOMEBREW_PREFIX/lib"
        echo "- Headers are installed in: $HOMEBREW_PREFIX/include"
        echo "- Add to your shell profile:"
        echo "   export CMAKE_PREFIX_PATH=\"$HOMEBREW_PREFIX:\$CMAKE_PREFIX_PATH\""
        echo "   export PKG_CONFIG_PATH=\"$HOMEBREW_PREFIX/lib/pkgconfig:\$PKG_CONFIG_PATH\""
        echo
        echo "Next steps:"
        echo "1. Add the following to your ~/.zshrc (or ~/.bash_profile):"
        echo "   export CMAKE_PREFIX_PATH=\"$HOMEBREW_PREFIX:\$CMAKE_PREFIX_PATH\""
        echo "   export PKG_CONFIG_PATH=\"$HOMEBREW_PREFIX/lib/pkgconfig:\$PKG_CONFIG_PATH\""
        echo "2. Run 'source ~/.zshrc' to update your environment"
        echo "3. If you encounter library loading issues, check your DYLD_LIBRARY_PATH"
        echo
        echo "For building the project, you may need to use:"
        echo "   cmake -DCMAKE_PREFIX_PATH=\"$HOMEBREW_PREFIX\" .."
    fi

    echo
    echo "ROS2 Humble installation:"
    if [[ "$OS_TYPE" == "linux" ]]; then
        echo "Follow instructions at https://docs.ros.org/en/humble/Installation.html"
    elif [[ "$OS_TYPE" == "macos" ]]; then
        echo "Follow instructions at https://docs.ros.org/en/humble/Installation.html"
        echo "For macOS, you may need to use conda-forge:"
        echo "conda install ros-humble-desktop -c conda-forge"
    fi
    echo "=========================================================="

    # Clean up
    cd "$SCRIPT_DIR"
    echo "Do you want to remove the build directory for dependencies? (y/n)"
    read -p "" -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$BUILD_DIR"
        echo "Build directory removed."
    fi

    echo "Dependency installation completed!"
    if [[ "$OS_TYPE" == "linux" ]]; then
        echo "You may need to run 'source ~/.bashrc' to update your environment."
        echo "Run 'sudo ldconfig' if you encounter library loading issues."
    fi
}

# Run main installation
main
