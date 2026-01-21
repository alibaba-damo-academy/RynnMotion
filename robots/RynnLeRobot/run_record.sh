#!/bin/bash

### LeRobot Dataset Recording Script
###
### Options:
###   --godview, -gv: Express poses in front camera frame (default: yes)
###                   Use -gv no for robot base frame poses

cd "$(dirname "$0")"

# Try to find and activate the virtual environment
# Match the same logic as setup_env.sh: venv on macOS, .venv on Linux
#if [[ "$OSTYPE" == "darwin"* ]]; then
    VENV_DIR="venv"
#else
#    VENV_DIR=".venv"
#fi

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
    echo "  bash setup_env.sh  OR  bash setup_lerobot_uv.sh"
    exit 1
}

export LANGUAGE="CN"
# export LANGUAGE=EN

# Generate timestamp folder: month_day_hour_min (e.g., 0904_2015)
# TIMESTAMP=$(date "+%m%d_%H%M%S")
# OUTPUT_DIR="outputs/${TIMESTAMP}"
OUTPUT_DIR="outputs"

# Setup data collection parameters
TASK="Pick up the cube and place it in the container"
REPO_ID="cube_pickup_1"

echo "🎬 RynnLeRobot Data Collection"
# echo "📅 Session: $TIMESTAMP"
echo "🎯 Task: $TASK"
echo "📁 Output: $OUTPUT_DIR/$REPO_ID"
echo ""

# Main recording command
python -m RynnLeRobot.scripts.record \
    --repo-id "$REPO_ID" \
    --task "$TASK" \
    --episodes 2 \
    --episode-time 30 \
    --reset-time 3 \
    --fps 30 \
    --root "$OUTPUT_DIR" \
    --config "configs/so101.yaml" \
    --show-webcam \
    --log-level INFO \
    --lang $LANGUAGE

# --show-display \

echo ""
echo "✅ Recording completed!"
echo "📁 Data saved to: $OUTPUT_DIR/$REPO_ID"
