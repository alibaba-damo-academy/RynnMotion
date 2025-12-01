"""
real-time interpolation trajectory generator.
"""

from abc import ABC, abstractmethod
from typing import Optional
import numpy as np
import math
import time
from RynnMotion.trajgen.trajgen_base import (
    TrajectoryGeneratorBase,
    register_generator_factory_func,
)
from RynnMotion.trajgen.interpolate.rt_trajgen import RealtimeTrajGenRuckig

from RynnMotion.common.data.robot_state import RobotState


@register_generator_factory_func("tg_ruckig")
def generator_config_to_class(generator_config: dict, robot_model, communicator=None, logger=None):
    """
    Maps algo config to the CQL algo class to instantiate, along with additional algo kwargs.

    Args:
        algo_config (Config instance): algo config

    Returns:
        algo_class: subclass of Algo
        algo_kwargs (dict): dictionary of additional kwargs to pass to algorithm
    """
    return RTInterpolateGenerator(generator_config, robot_model, communicator, logger)


class RTInterpolateGenerator(TrajectoryGeneratorBase):
    """
    real-time interpolation trajectory generator.
    """

    def __init__(self, generator_config, robot_model, communicator=None, logger=None):
        """
        Initialize generator.

        Args:
            generator_config: Trajectory generator config
            robot_model: Robot model class
            communicator: Communicator class (optional)
        """
        super().__init__(generator_config, robot_model, communicator, logger)
        self.communicator = communicator
        self.current_action_count = 0
        self.signal_chunk_size = 0

        self.command_traj_freq = self.communicator.get_command_traj_freq()
        self.command_traj_dt = self.communicator.get_command_traj_dt()
        self.controller_freq = self.robot_model.get_robot_control_freq()

        self.traj_index = -1

        self.load_robot_limits()
        self.init_rt_trajgen()

    def load_robot_limits(self):
        """Load robot limits."""
        self.gen_cfg = self.generator_config.get("generator", {})
        self.joint_limits_upper = self.gen_cfg.get("joint_limit_upper", None)
        self.joint_limits_lower = self.gen_cfg.get("joint_limit_lower", None)

        ratio = self.gen_cfg.get("vel_limit_ratio", 1.0)
        vel_limit = self.gen_cfg.get("joint_vel_limit", None)
        self.joint_vel_limit = np.array(vel_limit) * ratio
        acc_limit = self.gen_cfg.get("joint_acc_limit", None)
        self.joint_acc_limit = np.array(acc_limit) * ratio
        jerk_limit = self.gen_cfg.get("joint_jerk_limit", None)
        self.joint_jerk_limit = np.array(jerk_limit) * ratio

        self.logger.info(f"load robot limits config!")
        self.logger.info(f"velocity limit: {self.joint_vel_limit}")
        self.logger.info(f"acceleration limit: {self.joint_acc_limit}")
        self.logger.info(f"jerk limit: {self.joint_jerk_limit}")

    def init_rt_trajgen(self):
        """Initialize real-time trajectory generator."""
        self.first_get_fb = True
        self.rt_trajgen = RealtimeTrajGenRuckig(
            dof=self.act_dofs,
            input_freq=self.command_traj_freq,
            output_freq=self.controller_freq,
        )
        self.rt_trajgen.set_position_limits(
            position_upper_limits=self.joint_limits_upper,
            position_lower_limits=self.joint_limits_lower,
        )
        self.rt_trajgen.set_robot_ability(
            velocity_limits=self.joint_vel_limit,
            acceleration_limits=self.joint_acc_limit,
            jerk_limits=self.joint_jerk_limit,
        )
        self.logger.info(f"✓ real-time trajecory generation seccessfully!")

    def _get_comannd_by_workmode(self):
        """get joint command in workmode 0"""
        if self.signal_trajectory.workmode == 0:
            num_joint = self.signal_trajectory.trajectory[self.traj_index].num_joint
            if num_joint != self.act_dofs:
                self.current_joint_command = self.signal_trajectory.trajectory[self.traj_index].joint_pos
            else:
                raise ValueError(f"rt trajgen joint number is {num_joint}, not equal to model act dofs {self.act_dofs}")
        elif self.signal_trajectory.workmode == 1:
            num_ee = self.signal_trajectory.trajectory[self.traj_index].num_ee
            if num_ee != self.ee_num:
                cartesian_pose = self.signal_trajectory.trajectory[self.traj_index].ee_pose

            else:
                raise ValueError(f"rt trajgen end-effector number is {num_ee}, not equal to model ee_num {self.ee_num}")

    def process_input_command(self, path_point=None):
        """Get signal command."""
        latest_command, new_command = self.communicator.process_subscribe_command()
        if not new_command:
            return

        """reset input command state to reflesh new input command."""
        self.current_action_count = 0
        self.traj_index = -1

        self.signal_trajectory = latest_command.copy()
        self.signal_chunk_size = self.signal_trajectory.chunk_size
        self.communicator_connected = True

    def reset_trajectory_state(self, robot_state: RobotState):
        """
        reset trajectory state by robot state.

        Args:
            robot_state: robot state
        """
        if robot_state is None:
            return
        self.current_joint_command = robot_state.joint_pos
        self.rt_trajgen.reset_state(
            q_state=robot_state.joint_pos,
            q_target=robot_state.joint_pos,
        )
        self.robot_command.joint_pos = robot_state.joint_pos
        self.robot_connected = True
        self.logger.info(f"trajectory module get init joint position: {self.current_joint_command}")

    def update_trajectory(self):
        """
        update simulation trajectory.

        Returns:
            traj_point
        """
        if not self.robot_connected:
            return None, True

        if not self.communicator_connected:
            return self.robot_command, True

        lowloop_count = self.current_action_count * self.command_traj_freq / self.controller_freq
        traj_index = min(self.signal_chunk_size - 1, max(0, math.floor(lowloop_count)))
        if traj_index != self.traj_index:
            self.traj_index = traj_index
            self.current_joint_command = self.signal_trajectory.trajectory[self.traj_index].joint_pos
            self.rt_trajgen.set_input_target(np.array(self.current_joint_command))

            if traj_index == self.signal_chunk_size - 1:
                self.complete = True
            else:
                self.complete = False
        self.current_action_count += 1
        self.robot_command.joint_pos = self.rt_trajgen.update().tolist()
        return self.robot_command, self.complete
