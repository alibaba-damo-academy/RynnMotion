#!/bin/bash
# Script to fix MuJoCo library path on macOS

EXECUTABLE="$1"
MUJOCO_LIBRARY="$2"

if [[ "$OSTYPE" != "darwin"* ]]; then
    echo "MuJoCo path fix only needed on macOS"
    exit 0
fi

echo "Checking MuJoCo library path..."

if otool -L "$EXECUTABLE" | grep -q '@rpath/mujoco.framework'; then
    echo "Fixing MuJoCo library path..."
    for version in 3.3.5 3.3.4 3.3.3 3.3.2 3.3.1 3.3.0; do
        if install_name_tool -change "@rpath/mujoco.framework/Versions/A/libmujoco.$version.dylib" "$MUJOCO_LIBRARY" "$EXECUTABLE" 2>/dev/null; then
            echo "MuJoCo library path fixed (version $version)"
            break
        fi
    done
else
    echo "MuJoCo library path already correct"
fi