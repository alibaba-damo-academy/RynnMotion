#!/bin/bash
# Build script for Hybrid Kinematics Comparison Test

# Set colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Building Hybrid Kinematics vs MuJoCo Engine Test${NC}"

# Check if MuJoCo environment variables are set (optional, we have defaults)
if [ -z "$MUJOCO_INCLUDE_DIR" ] || [ -z "$MUJOCO_LIB_DIR" ]; then
    echo -e "${YELLOW}MuJoCo environment variables not set, using auto-detection:${NC}"
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo "  Will check: /opt/homebrew/ (for Apple Silicon)"
        echo "  Will check: /usr/local/mujoco-{version}/ (manual install)"
    else
        echo "  MUJOCO_DIR=/usr/local/mujoco-3.3.0"
    fi
    echo ""
    echo "To override, set:"
    echo "  export MUJOCO_INCLUDE_DIR=/path/to/mujoco/include"
    echo "  export MUJOCO_LIB_DIR=/path/to/mujoco/lib"
    echo ""
fi

# Create build directory
mkdir -p build
cd build

# Configure with CMake
echo -e "${YELLOW}Configuring with CMake...${NC}"
cmake .. -DCMAKE_BUILD_TYPE=Debug

# Build the project
echo -e "${YELLOW}Building...${NC}"
make hybrid_main -j4

# Check if build was successful
if [ $? -eq 0 ]; then
    echo -e "${GREEN}Build successful!${NC}"
    echo -e "${GREEN}Executable created at: $(pwd)/hybrid_main${NC}"
    echo ""
    echo "To run the test:"
    echo "  cd build"
    echo "  ./hybrid_main                                      # Run LeRobot (robot 30), scenario 0 (5 steps), EE pose comparison"
    echo "  ./hybrid_main --scenario 1                         # Run LeRobot with scenario 1 (lerp 10s)"
    echo "  ./hybrid_main --robot 20 --scenario 0              # Run Franka FR3 (robot 20) with scenario 0"
    echo "  ./hybrid_main --robot 22 --render --scenario 1     # Run Piper (robot 22) with rendering"
    echo "  ./hybrid_main --jaco                               # Compare Jacobians instead of EE poses"
    echo "  ./hybrid_main --robot 23 --render --jaco           # Run RM75 (robot 23) with rendering, compare Jacobians"
    echo "  ./hybrid_main --rbdl                               # Enable RBDL comparison (default: MuJoCo vs Pinocchio only)"
    echo "  ./hybrid_main --help                               # Show all options and robot numbers"
else
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi
