"""
Manager module - Robot configuration management

Contains:
- RobotManager: Config-driven robot manager (from YAML or dict)
- MjcfParser: MJCF parsing utilities
"""

from .robot_manager import RobotManager

__all__ = [
    'RobotManager',
]
