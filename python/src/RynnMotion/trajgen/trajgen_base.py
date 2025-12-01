"""
trajectory generator base class and factory functions.
"""

from abc import ABC, abstractmethod
from typing import Optional
import numpy as np
from collections import OrderedDict
import logging

from RynnMotion.common.data.robot_state import RobotState

REGISTERED_TRAJECTORY_GENERATOR_FACTORY_FUNCS = OrderedDict()


def register_generator_factory_func(generator_name):
    """
    Function decorator to register algo generator functions that map generator class names.
    Each generator implements such a function, and decorates it with this decorator.

    Args:
        generator_name (str): the algorithm name to register the algorithm under
    """

    def decorator(factory_func):
        REGISTERED_TRAJECTORY_GENERATOR_FACTORY_FUNCS[generator_name] = factory_func

    return decorator


def generator_factory(
    generator_name, generator_config, robot_model, communicator=None, logger=None
):
    """
    Factory function for creating trajectory generator based on the generator name and config.

    Args:
        generator_name (str): the generator name

        config (BaseConfig instance): config object
    """

    #
    generator_class = REGISTERED_TRAJECTORY_GENERATOR_FACTORY_FUNCS[generator_name]

    # create algo instance
    return generator_class(generator_config, robot_model, communicator, logger)


class TrajectoryGeneratorBase(object):
    """
    trajectory generator base class.
    """

    def __init__(
        self, generator_config: dict, robot_model, communicator=None, logger=None
    ):
        """
        Initialize generator.

        Args:
            generator_config: Trajectory generator config
            robot_model: Robot model class
            communicator: Communicator class (optional)
        """
        self.logger = logger or logging.getLogger(__name__)

        self.workmode = 0
        self.robot_connected = False
        self.communicator_connected = False
        self.complete = True

        # self.robot_model = robot_model
        self._load_robot_model(robot_model)
        self.generator_config = generator_config

        self.act_dofs = self.robot_model.get_actuator_num()
        self.robot_command = RobotState()
        self.robot_command.num_joints = self.act_dofs

        self.ee_num = self.robot_model.get_ee_num()
        self.robot_command.num_end_effectors = self.ee_num

        self.logger.info("trajectory generator build successful")
        self.logger.info(f"act dofs: {self.act_dofs}")

    def _load_robot_model(self, robot_model):
        if robot_model is None:
            self.logger.error(
                "robot_model is None. This will cause initialization failure."
            )
            raise ValueError("robot_model cannot be None")

        self.logger.debug(
            f"Initializing TrajectoryGenerator with robot_model: {type(robot_model).__name__}"
        )
        # Check if required methods exist on robot_model
        required_methods = ["get_actuator_num", "get_ee_num"]
        for method in required_methods:
            if not hasattr(robot_model, method):
                self.logger.error(f"robot_model missing required method: {method}")
                raise AttributeError(f"robot_model must implement {method} method")

        self.robot_model = robot_model

    @abstractmethod
    def process_input_command(self, path_point=None):
        """
        process communicator input command

        path_point: path point to set if use thid module in one process.
        """
        pass

    @abstractmethod
    def reset_trajectory_state(self, robot_state: RobotState):
        """
        process robot state feedback.

        prepare trajectory: set robot state.
        """
        pass

    @abstractmethod
    def update_trajectory(self) -> (RobotState, bool):
        """
        update trajectory based on input command and robot state.

        Returns:
            robot_command: RobotState
        """
        self.complete = True
        return self.robot_command, self.complete
