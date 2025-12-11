"""
Manager module - Robot configuration management

Contains:
- RobotManager: Config-driven robot manager (from YAML or dict)
- MjcfParser: MJCF parsing utilities
"""

from .robot_manager import RobotManager

from RynnMotion.manager.fsm_base import (
    register_fsm_factory_func,
    fsm_factory,
    RobotStateMachine,
)

__all__ = [
    "RobotManager",
]
