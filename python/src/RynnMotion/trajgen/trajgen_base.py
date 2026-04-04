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
        return factory_func

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


class TrajectoryGeneratorBase(ABC):
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

        # Total actuator DOFs (arm + gripper)
        self.act_dofs = self.robot_model.get_actuator_num()

        # Arm/gripper DOF separation
        self.arm_dofs = self.robot_model.get_motion_dof()
        self.gripper_dofs = self.robot_model.get_action_dof()
        self.joint_indices = self.robot_model.get_joint_indices()
        self.ee_indices = self.robot_model.get_ee_indices()

        self.robot_command = RobotState()
        self.robot_command.num_joints = self.act_dofs

        self.ee_num = self.robot_model.get_ee_num()
        self.robot_command.num_end_effectors = self.ee_num

        self.robot_feedback = RobotState()

        self.logger.info("trajectory generator build successful")
        self.logger.info(
            f"act_dofs: {self.act_dofs} (arm: {self.arm_dofs}, gripper: {self.gripper_dofs})"
        )
        self.logger.info(f"joint_indices: {self.joint_indices}, ee_indices: {self.ee_indices}")

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

    # ========== Arm/gripper split helpers ==========

    def split_arm_gripper(self, full_pos):
        """Split full joint positions into arm and gripper components.

        Args:
            full_pos: Full joint positions list/array of length act_dofs.

        Returns:
            arm_pos: List of arm joint positions (length arm_dofs).
            gripper_pos: List of gripper joint positions (length gripper_dofs).
        """
        arm_pos = [full_pos[i] for i in self.joint_indices]
        gripper_pos = [full_pos[i] for i in self.ee_indices]
        return arm_pos, gripper_pos

    def merge_arm_gripper(self, arm_pos, gripper_pos, out=None):
        """Merge arm and gripper positions into full joint position array.

        Args:
            arm_pos: Arm joint positions (length arm_dofs).
            gripper_pos: Gripper joint positions (length gripper_dofs).
            out: Optional output list to write into (length act_dofs).
                 If None, returns a new list.

        Returns:
            full_pos: Merged joint positions (length act_dofs).
        """
        if out is None:
            out = [0.0] * self.act_dofs
        for i, idx in enumerate(self.joint_indices):
            out[idx] = arm_pos[i]
        for i, idx in enumerate(self.ee_indices):
            out[idx] = gripper_pos[i]
        return out

    def process_parameters_update(self, parameters=None):
        """
        process parameters update
        """
        pass

    @abstractmethod
    def process_input_command(self, latest_command, new_command_flag):
        """
        process communicator input command

        latest_command: path point to set if use thid module in one process.
        new_command_flag: True if command is newest.
        """
        pass

    def process_robot_state(self, robot_state: RobotState):
        """
        process robot state feedback.

        set robot state.
        use this function to get robot state feedback if trajectory generator is based on periodic feedback.
        """
        self.robot_feedback = robot_state.copy()

    @abstractmethod
    def reset_trajectory_state(self, robot_state: RobotState):
        """
        process robot state feedback.

        prepare trajectory: set robot state.
        reset once if trajectory generator need initial feedback.
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

    @abstractmethod
    def get_trajectory_command_without_update(self):
        """
        get trajectory input command.
        """
        return self.robot_command.copy()
