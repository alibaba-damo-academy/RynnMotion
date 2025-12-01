#!/usr/bin/env python3
"""
Constants for RynnMotion dataset functionality.
Moved from RynnLeRobot.hardware.utils.constants to main RynnMotion package.
"""

from pathlib import Path
import os

# Default location for LeRobot datasets
HF_LEROBOT_HOME = Path(os.getenv("LEROBOT_HOME", Path.home() / ".cache" / "huggingface" / "lerobot"))

# Ensure the directory exists
HF_LEROBOT_HOME.mkdir(parents=True, exist_ok=True)