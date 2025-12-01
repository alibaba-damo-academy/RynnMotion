#!/usr/bin/env python3
"""
Base Unified Controller - Abstract base class for all robot controllers

This module provides the common control logic and infrastructure that all robot
implementations (LeRobot, Lekiwi, etc.) can inherit from.
"""

from abc import ABC, abstractmethod
from collections import OrderedDict
import logging

from RynnMotion.common.data.robot_state import RobotState
from RynnMotion.common.data.robot_command import RobotCommand
from RynnMotion.manager.robot_manager import RobotManager

REGISTERED_COMMUNICATOR_FACTORY_FUNCS = OrderedDict()


def register_communicator_factory_func(communicator_type):
    """
    Function decorator to register algo generator functions that map generator class names.
    Each generator implements such a function, and decorates it with this decorator.

    Args:
        generator_name (str): the algorithm name to register the algorithm under
    """

    def decorator(factory_func):
        REGISTERED_COMMUNICATOR_FACTORY_FUNCS[communicator_type] = factory_func

    return decorator


def communicator_factory(
    communicator_type, robot_model, communicator_config, logger=None
):
    """
    Factory function for creating trajectory generator based on the generator name and config.

    Args:
        generator_name (str): the generator name

        config (BaseConfig instance): config object
    """
    generator_class = REGISTERED_COMMUNICATOR_FACTORY_FUNCS[communicator_type]

    # create algo instance
    return generator_class(
        robot_model=robot_model, communicator_config=communicator_config, logger=logger
    )


class CommunicatorBase(ABC):
    """
    Communicator - Abstract Base Class for out command Communication

    This class provides an interface for all robot controllers to communicate
    with the robot using LCM. It is intended to be used as a base class for
    all robot controllers.
    """

    def __init__(self, robot_model: RobotManager, communicator_config, logger):
        self.logger = logger or logging.getLogger(__name__)

        self.robot_model = robot_model
        self.robot_control_freq = robot_model.get_robot_control_freq()
        self.control_dt = 1.0 / self.robot_control_freq

        self.actuator_dofs = robot_model.get_actuator_num()
        self.joint_state_dofs = robot_model.get_joint_state_num()
        self.gripper_dofs = robot_model.get_gripper_num()
        self.ee_num = robot_model.get_ee_num()
        self.sites_num = robot_model.get_site_num()

        # optional/extended counts (may not exist on all models)
        self.num_ft_sensors = robot_model.get_ft_sensor_num()

        if self.actuator_dofs == 0:
            self.logger.info("robot_model has no actuator_dofs or it returns 0")

        self.default_chunk = communicator_config.get("default_chunk", 1)
        self.command_traj_freq = communicator_config.get("command_traj_freq", 10)
        self.command_step_dt = 1.0 / self.command_traj_freq
        self.control_step_dt = 1.0 / self.robot_control_freq

        self.robot_command = RobotCommand()
        self.robot_command.init_from_dofs(
            num_joints=int(self.actuator_dofs),
            num_grippers=int(self.gripper_dofs),
            num_end_effectors=int(self.ee_num),
            num_ft_sensors=int(self.num_ft_sensors),
            num_sites=int(self.sites_num),
            chunk_size=int(self.default_chunk),
        )

    def get_command_traj_freq(self):
        """Get command traj freq."""
        return self.command_traj_freq

    def get_command_traj_dt(self):
        """Get command dt."""
        return self.command_step_dt

    @abstractmethod
    def connect(self):
        """Initialize communicator."""
        pass

    @abstractmethod
    def process_publish_robot_state(self, robot_state: RobotState) -> None:
        """Publish message to channel."""
        pass

    @abstractmethod
    def process_subscribe_command(self) -> None:
        """Subscribe to channel with callback."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        pass
