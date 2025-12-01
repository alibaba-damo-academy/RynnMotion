"""
Common utilities and shared components for the robot motion project.

This package contains shared libraries, LCM types, and other common functionality.
"""

from RynnMotion.common.communicator_base import (
    register_communicator_factory_func,
    communicator_factory,
    CommunicatorBase,
)

from RynnMotion.common.lcm_communiactor import LCMCommunicator
from RynnMotion.common.mock_communiactor import MockCommunicator
