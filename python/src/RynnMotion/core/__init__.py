"""
RynnMotion Core Module - Robot Interface Architecture

This module provides the foundational architecture for robot control:
- RobotInterfaceBase: Abstract base class for all robot interfaces
- MujocoRobotInterface: Simulation interface with multi-threaded rendering
- Factory pattern: robotinterface_factory for creating robot interfaces

Design Philosophy:
- Config-driven architecture (RobotManager with YAML/dict configs)
- Structured state management (RobotState dataclass)
- Factory registration pattern for extensibility
- Simple, Pythonic, and performant

Example:
    from RynnMotion.core import robotinterface_factory
    from RynnMotion.manager import RobotManager

    # Create robot manager from config
    config = {
        "robot_name": "fr3",
        "robot_control_freq": 100,
        "robot_mjcf": "path/to/model.xml",
        "pino_mjcf": "path/to/pinocchio.xml",
    }
    robot_manager = RobotManager(config, logger)

    # Create interface via factory
    robot_interface = robotinterface_factory("mujoco", robot_manager)
"""

__version__ = "1.0.0"

from RynnMotion.core.robotinterface_base import (
    register_robotinterface_factory_func,
    robotinterface_factory,
    RobotInterfaceBase,
)

from RynnMotion.core.mjrobot_interface import MujocoRobotInterface

__all__ = [
    "RobotInterfaceBase",
    "MujocoRobotInterface",
    "register_robotinterface_factory_func",
    "robotinterface_factory",
]
