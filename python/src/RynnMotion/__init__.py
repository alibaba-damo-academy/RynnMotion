"""
RynnMotion: Comprehensive robotics framework for mobile manipulation and locomotion.

This package provides:
- base: Base interfaces and controllers for robot systems
- utils: Utilities for kinematics, trajectory planning, plotting, and more
"""

__version__ = "1.0.0"
__author__ = "RynnMotion Team"

# Make submodules easily accessible
from RynnMotion import utils
from RynnMotion import common

__all__ = ["utils", "common", "__version__"]
