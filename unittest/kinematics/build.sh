#!/bin/bash
# Standalone build script for Pinocchio+MuJoCo Kinematics Tests

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}Building Pinocchio+MuJoCo Kinematics Tests${NC}"

if [ -z "$MUJOCO_INCLUDE_DIR" ] || [ -z "$MUJOCO_LIB_DIR" ]; then
    echo -e "${YELLOW}MuJoCo environment variables not set, using auto-detection${NC}"
fi

mkdir -p build
cd build

echo -e "${YELLOW}Configuring with CMake...${NC}"
cmake .. -DCMAKE_BUILD_TYPE=Debug

echo -e "${YELLOW}Building...${NC}"
make -j4

if [ $? -eq 0 ]; then
    echo -e "${GREEN}Build successful!${NC}"
    echo ""
    echo "Usage:"
    echo "  ./pinMjTest           # Single EE test (default)"
    echo "  ./pinMjTest --multi   # Multi-site test"
    echo "  ./pinMjTest --help    # All options"
else
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi
