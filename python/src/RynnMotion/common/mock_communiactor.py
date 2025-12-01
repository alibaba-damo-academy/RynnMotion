"""
LCM Communication Handler

Handles all LCM communication for robot control including:
- Policy command reception (ACT commands)
- Feedback requests handling
- State and robot feedback publishing
"""

import time
import threading
import logging
import lcm
import numpy as np
import sys
from pathlib import Path

from RynnMotion.common.communicator_base import (
    CommunicatorBase,
    register_communicator_factory_func,
)

# Use absolute imports from RynnMotion package
from RynnMotion.common.lcm.lcmMotion.act_command import act_command
from RynnMotion.common.lcm.lcmMotion.act_request import act_request
from RynnMotion.common.lcm.lcmMotion.robot_feedback import robot_feedback
from RynnMotion.common.lcm.lcmMotion.state_feedback import state_feedback

from RynnMotion.common.data.basic_data import Pose


@register_communicator_factory_func("mock")
def mock_communicator_factory(robot_model, communicator_config: dict, logger=None):
    """
    lcm communicator class generator function, along with additional algo kwargs.

    Returns:
        lcmcommunicator_class: subclass of comunicator
        lcmcommunicator_kwargs (dict): dictionary of additional kwargs to pass to communicator
    """
    return MockCommunicator(robot_model, communicator_config, logger)


class MockCommunicator(CommunicatorBase):
    """
    Handles all LCM communication for robot control.

    Separates communication logic from the main controller,
    making the code more modular and maintainable.
    """

    def __init__(self, robot_model, communicator_config, logger=None):
        """
        Initialize LCM handler.

        Args:
            logger: Logger instance (optional)
        """
        super().__init__(
            robot_model=robot_model,
            communicator_config=communicator_config,
            logger=logger,
        )

        self.mock_config = communicator_config.get("mock_config", None)
        self.home_position = self.mock_config.get("mock_home_position", None)
        self.signal_amplitude = self.mock_config.get("mock_amplitude", None)
        self.signal_frequency = self.mock_config.get("mock_frequency", 0)
        self.block_time = self.mock_config.get("block_time", 0.1)

        self.logger.info(f"mock signal home position: {self.home_position}")
        self.logger.info(f"amplitude: {self.signal_amplitude}")
        self.logger.info(
            f"frequency: {self.signal_frequency}, block_time: {self.block_time}"
        )

        self.seq = -1
        self.signal_time = 0.0
        self.control_loop_time = 0.0
        self.gen_command_time = (
            self.command_step_dt * self.default_chunk + self.block_time
        )

    def generate_commands(self):
        """generate simulation trajectory."""
        for i in range(self.default_chunk):
            self.signal_time = self.signal_time + self.command_step_dt
            self.robot_command.trajectory[i].joint_pos = self.generate_sine_commands(
                self.signal_time
            ).tolist()

    def generate_sine_commands(self, signal_t):
        """generate simulation sine trajectory command."""
        trajectory_point = np.zeros(self.actuator_dofs)
        for i in range(self.actuator_dofs):
            trajectory_point[i] = self.home_position[i] + self.signal_amplitude[
                i
            ] * np.sin(2 * np.pi * self.signal_frequency * signal_t)
        return trajectory_point

    def connect(self):
        """Initialize communicator connection and subscribe to channels."""
        self.connected = True

    def disconnect(self):
        """Disconnect communicator."""
        self.connected = False
        self.logger.info("✓ communicator disconnected")

    def is_connected(self):
        """Check if communicator is connected."""
        return True

    def process_subscribe_command(self):
        """update signal, update every loop and it will count in inner loop."""
        new_command = False

        if self.control_loop_time >= self.gen_command_time:
            self.control_loop_time = 0
        if self.control_loop_time == 0:
            self.robot_command.chunk_size = self.default_chunk
            self.generate_commands()
            self.seq = self.seq + 1
            new_command = True
        self.control_loop_time = self.control_loop_time + self.control_dt
        return self.robot_command, new_command

    def process_publish_robot_state(self, robot_state):
        """Handle communicator feedback publishing."""
        pass
