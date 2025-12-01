"""
sine wave trajectory generator for simulation motion.
"""

from abc import ABC, abstractmethod
from typing import Optional
import numpy as np
from RynnMotion.trajgen.trajgen_base import (
    TrajectoryGeneratorBase,
    register_generator_factory_func,
)

from RynnMotion.trajgen.interpolate.policy_interpolator import lerp


@register_generator_factory_func("sim_motion")
def generator_config_to_class(
    generator_config: dict, robot_model, communicator=None, logger=None
):
    """
    Maps algo config to the CQL algo class to instantiate, along with additional algo kwargs.

    Args:
        algo_config (Config instance): algo config

    Returns:
        algo_class: subclass of Algo
        algo_kwargs (dict): dictionary of additional kwargs to pass to algorithm
    """
    return SimMotionGenerator(generator_config, robot_model, communicator, logger)


class SimMotionGenerator(TrajectoryGeneratorBase):
    """
    sine wave trajectory generator.
    """

    SIGNAL_TYPES = ("sine", "cosine", "triangle")

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
        super().__init__(generator_config, robot_model, communicator, logger)

        self.update_dt = generator_config.get("timestep", 0.01)
        self.signal_time = 0.0
        self.init_joint_position = [0.0] * self.act_dofs

        gen_cfg = generator_config.get("generator", {})
        self.signal_type = gen_cfg.get("type", "sine").lower()
        if self.signal_type not in self.SIGNAL_TYPES:
            raise ValueError(f"signal_type must be one of {self.SIGNAL_TYPES}")
        self.durtime = gen_cfg.get("durtime", 3.0)
        self.home_position = gen_cfg.get("signal_home_position", None)
        self.signal_amplitude = gen_cfg.get("signal_amplitude", None)
        self.signal_frequency = gen_cfg.get("signal_frequency", 0)
        if len(self.home_position) != self.act_dofs:
            raise ValueError(
                f"home_position must be of length {self.act_dofs}, but got {len(self.home_position)}"
            )

        if len(self.signal_amplitude) != self.act_dofs:
            raise ValueError(
                f"signal_amplitude must be of length {self.act_dofs}, but got {len(self.signal_amplitude)}"
            )

        self.logger.info(f"✓simulation {self.signal_type} trajectory ready")

    def generate_sine_trajectory_point(self, signal_t):
        """
        generate sine simulation commands.
        """
        for i in range(self.act_dofs):
            self.robot_command.joint_pos[i] = self.home_position[
                i
            ] + self.signal_amplitude[i] * np.sin(
                2 * np.pi * self.signal_frequency * signal_t
            )

    def generate_cosine_trajectory_point(self, signal_t):
        """
        generate cosine simulation commands.
        """
        for i in range(self.act_dofs):
            self.robot_command.joint_pos[i] = self.home_position[
                i
            ] + self.signal_amplitude[i] * np.cos(
                2 * np.pi * self.signal_frequency * signal_t
            )

    def generate_triangle_trajectory_point(self, signal_t):
        """
        generate triangle simulation commands (triangle wave in [-1,1]).
        """
        phase = self.signal_frequency * signal_t
        frac = phase - np.floor(phase)  # fractional part in [0,1)
        tri = np.where(
            frac < 0.5,
            4.0 * frac - 1.0,  # rising edge: -1 -> +1
            -4.0 * frac + 3.0,  # falling edge: +1 -> -1
        )
        tri = tri + 1.0  # shift to [0,2]
        for i in range(self.act_dofs):
            self.robot_command.joint_pos[i] = (
                self.home_position[i] + self.signal_amplitude[i] * tri
            )

    def move_to_home_position(self, signal_t):
        for i in range(self.act_dofs):
            self.robot_command.joint_pos[i] = lerp(
                self.init_joint_position[i],
                self.home_position[i],
                signal_t,
                self.durtime,
            )

    def process_input_command(self, path_point=None):
        """
        input command is from simulation motion, communicator is always connected
        """
        self.communicator_connected = True

    def reset_trajectory_state(self, robot_state):
        """
        prepare trajectory: set robot state once.
        """
        if robot_state is None:
            return
        self.robot_connected = True
        if robot_state.num_joints == self.act_dofs:
            self.init_joint_position = robot_state.joint_pos
        else:
            self.init_joint_position = robot_state.joint_pos[: self.act_dofs]
            self.logger.warning(
                f"robot state num_joints {robot_state.num_joints} does not match act_dofs {self.act_dofs}, use first {self.act_dofs} joints"
            )
        self.logger.info(
            f"trajectory module get init joint position: {self.init_joint_position}"
        )

    def update_trajectory(self):
        """
        update simulation trajectory.

        Returns:
            traj_point
        """
        if not self.robot_connected:
            return None, True
        if self.signal_time < self.durtime:
            self.move_to_home_position(self.signal_time)
        else:
            t_s = self.signal_time - self.durtime
            if self.signal_type == "sine":
                self.generate_sine_trajectory_point(t_s)
            elif self.signal_type == "cosine":
                self.generate_cosine_trajectory_point(t_s)
            elif self.signal_type == "triangle":
                self.generate_triangle_trajectory_point(t_s)
        self.signal_time = self.signal_time + self.update_dt
        return self.robot_command, True
