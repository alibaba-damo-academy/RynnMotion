#!/bin/bash
#
# convert.sh - Robot Package to RynnMotion MJCF Converter
#
# Usage:
#   ./convert.sh --urdf /path/to/package --output ../3.robot_arm/30.myrobot
#   ./convert.sh --menagerie /path/to/menagerie/robot --output ../3.robot_arm/31.robot
#   ./convert.sh --urdf /path/to/robot.urdf --output ../3.robot_arm/32.robot --dual
#
# See: python convert_robot.py --help for all options
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$SCRIPT_DIR"

if ! command -v python3 &> /dev/null; then
    echo "Error: python3 not found"
    exit 1
fi

if ! python3 -c "import mujoco" 2>/dev/null; then
    echo "Warning: mujoco package not installed"
    echo "Install with: pip install mujoco"
fi

python3 convert_robot.py "$@"
