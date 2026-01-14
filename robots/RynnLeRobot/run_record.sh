#!/bin/bash

### LeRobot Dataset Recording Script
###
### Options:
###   --godview, -gv: Express poses in front camera frame (default: yes)
###                   Use -gv no for robot base frame poses

cd "$(dirname "$0")"

# Try to find and activate the virtual environment
# Match the same logic as setup_env.sh: venv on macOS, .venv on Linux
if [[ "$OSTYPE" == "darwin"* ]]; then
    VENV_DIR="venv"
else
    VENV_DIR=".venv"
fi

if [[ -f "$VENV_DIR/bin/activate" ]]; then
    source "$VENV_DIR/bin/activate"
    echo "✅ Activated $VENV_DIR virtual environment"
else
    echo "❌ Error: Virtual environment not found at $VENV_DIR!"
    echo "Please run setup_env.sh or setup_lerobot_uv.sh first"
    exit 1
fi

# Verify that datasets package is available
python -c "import datasets" 2>/dev/null || {
    echo "❌ Error: 'datasets' package not found in virtual environment"
    echo "Please run the setup script again:"
    echo "  ./setup_env.sh  OR  ./setup_lerobot_uv.sh"
    exit 1
}

# Generate timestamp folder: month_day_hour_min (e.g., 0904_2015)
TIMESTAMP=$(date "+%m%d_%H%M")
OUTPUT_DIR="outputs/${TIMESTAMP}"

# Setup data collection parameters
TASK="Pick up the cube and place it in the container"
REPO_ID="lerobot_demo/cube_pickup_${TIMESTAMP}"

echo "🎬 RynnLeRobot Data Collection"
echo "📅 Session: $TIMESTAMP"
echo "🎯 Task: $TASK"
echo "📁 Output: $OUTPUT_DIR/$REPO_ID"
echo ""

# Main recording command
python -m RynnLeRobot.scripts.record \
    --repo-id "$REPO_ID" \
    --task "$TASK" \
    --episodes 2 \
    --episode-time 5 \
    --reset-time 10 \
    --fps 30 \
    --root "$OUTPUT_DIR" \
    --config "configs/so101.yaml" \
    --display-data

echo ""
echo "✅ Recording completed!"
echo "📁 Data saved to: $OUTPUT_DIR/$REPO_ID"
