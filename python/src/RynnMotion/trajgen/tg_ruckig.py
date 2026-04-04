"""
real-time interpolation trajectory generator.

Arm joints are planned via Ruckig (smooth, time-optimal interpolation).
Gripper joints use direct passthrough (no interpolation).
"""

from abc import ABC, abstractmethod
from typing import Optional, List
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
def generator_config_to_class(
    generator_config: dict, robot_model, communicator=None, logger=None
):
    """Factory function for RTInterpolateGenerator."""
    return RTInterpolateGenerator(generator_config, robot_model, communicator, logger)


class RTInterpolateGenerator(TrajectoryGeneratorBase):
    """
    Real-time interpolation trajectory generator.

    Arm joints are planned via Ruckig for smooth, time-optimal trajectories.
    Gripper joints are passed through directly without interpolation.
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

        # Arm planning state
        self.current_arm_command: List[float] = [0.0] * self.arm_dofs

        # Gripper passthrough state
        self.current_gripper_command: List[float] = [0.0] * self.gripper_dofs

        self.load_robot_limits()
        self.init_rt_trajgen()

    def load_robot_limits(self):
        """Load robot limits for arm joints only."""
        self.gen_cfg = self.generator_config.get("generator", {})
        joint_limits_upper = self.gen_cfg.get("joint_limit_upper", None)
        joint_limits_lower = self.gen_cfg.get("joint_limit_lower", None)

        ratio = self.gen_cfg.get("vel_limit_ratio", 1.0)
        vel_limit = np.array(self.gen_cfg.get("joint_vel_limit", []))
        acc_limit = np.array(self.gen_cfg.get("joint_acc_limit", []))
        jerk_limit = np.array(self.gen_cfg.get("joint_jerk_limit", []))

        # Trim to arm_dofs if config provides full act_dofs length
        if len(vel_limit) > self.arm_dofs:
            self.logger.info(
                f"Trimming joint limits from {len(vel_limit)} to arm_dofs={self.arm_dofs}"
            )
            vel_limit = vel_limit[: self.arm_dofs]
            acc_limit = acc_limit[: self.arm_dofs]
            jerk_limit = jerk_limit[: self.arm_dofs]
            if joint_limits_upper is not None:
                joint_limits_upper = joint_limits_upper[: self.arm_dofs]
            if joint_limits_lower is not None:
                joint_limits_lower = joint_limits_lower[: self.arm_dofs]

        self.joint_limits_upper = joint_limits_upper
        self.joint_limits_lower = joint_limits_lower
        self.joint_vel_limit = vel_limit * ratio
        self.joint_acc_limit = acc_limit * ratio
        self.joint_jerk_limit = jerk_limit * ratio

        self.low_ability_ratio = self.gen_cfg.get("low_ability_ratio", 1.0)

        self.logger.info(f"Arm joint limits loaded (arm_dofs={self.arm_dofs}):")
        self.logger.info(f"  velocity limit: {self.joint_vel_limit}")
        self.logger.info(f"  acceleration limit: {self.joint_acc_limit}")
        self.logger.info(f"  jerk limit: {self.joint_jerk_limit}")
        self.logger.info(f"  command traj freq: {self.command_traj_freq}")
        self.logger.info(f"  controller freq: {self.controller_freq}")
        self.logger.info(f"  low ability ratio: {self.low_ability_ratio}")

    def init_rt_trajgen(self):
        """Initialize real-time trajectory generator for arm joints only."""
        self.first_get_fb = True
        self.rt_trajgen = RealtimeTrajGenRuckig(
            dof=self.arm_dofs,
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
        self.logger.info(
            f"Ruckig trajectory generator initialized (arm_dofs={self.arm_dofs}), "
            f"gripper passthrough (gripper_dofs={self.gripper_dofs})"
        )

    def process_input_command(self, latest_command, new_command_flag):
        """Get signal command."""
        if not new_command_flag:
            return

        # Reset input command state for new input command
        self.current_action_count = 0
        self.traj_index = -1

        self.signal_trajectory = latest_command.copy()
        self.signal_chunk_size = self.signal_trajectory.chunk_size
        self.communicator_connected = True

    def reset_trajectory_state(self, robot_state: RobotState):
        """
        Reset trajectory state by robot state.

        Splits robot state into arm/gripper and resets Ruckig for arm only.

        Args:
            robot_state: robot state
        """
        if robot_state is None:
            return

        # Split into arm and gripper
        arm_pos, gripper_pos = self.split_arm_gripper(robot_state.joint_pos)
        self.current_arm_command = arm_pos
        self.current_gripper_command = gripper_pos

        # Reset Ruckig for arm joints only
        self.rt_trajgen.reset_state(
            q_state=arm_pos,
            q_target=arm_pos,
        )

        # Set full command (merged)
        self.robot_command.joint_pos = robot_state.joint_pos
        self.robot_connected = True
        self.logger.info(
            f"Trajectory reset — arm: {self.current_arm_command}, "
            f"gripper: {self.current_gripper_command}"
        )

    def set_trajgen_low_ability(self):
        """Set low robot ability for arm trajectory generator."""
        self.rt_trajgen.set_robot_ability(
            velocity_limits=self.joint_vel_limit * self.low_ability_ratio,
            acceleration_limits=self.joint_acc_limit * self.low_ability_ratio,
            jerk_limits=self.joint_jerk_limit,
        )

    def set_trajgen_norm_ability(self):
        """Set normal robot ability for arm trajectory generator."""
        self.rt_trajgen.set_robot_ability(
            velocity_limits=self.joint_vel_limit,
            acceleration_limits=self.joint_acc_limit,
            jerk_limits=self.joint_jerk_limit,
        )

    def process_parameters_update(self, parameters="run"):
        """Process parameters update for arm trajectory generator."""
        if parameters == "run":
            self.set_trajgen_norm_ability()
        else:
            self.set_trajgen_low_ability()

    def update_trajectory(self):
        """
        Update trajectory: arm via Ruckig, gripper via passthrough.

        Returns:
            (robot_command, complete): RobotState with merged arm+gripper, completion flag
        """
        if not self.robot_connected:
            return None, True

        if not self.communicator_connected:
            return self.robot_command, True

        lowloop_count = (
            self.current_action_count * self.command_traj_freq / self.controller_freq
        )
        traj_index = min(self.signal_chunk_size - 1, max(0, math.floor(lowloop_count)))

        if traj_index != self.traj_index:
            self.traj_index = traj_index

            # Split new target into arm/gripper
            full_pos = self.signal_trajectory.trajectory[self.traj_index].joint_pos
            arm_target, gripper_target = self.split_arm_gripper(full_pos)

            # Arm: feed to Ruckig
            self.current_arm_command = arm_target
            self.rt_trajgen.set_input_target(np.array(arm_target))

            # Gripper: direct passthrough
            self.current_gripper_command = gripper_target

            if traj_index == self.signal_chunk_size - 1:
                self.complete = True
            else:
                self.complete = False

        self.current_action_count += 1

        # Arm: Ruckig smooth output
        arm_result = self.rt_trajgen.update().tolist()

        # Merge arm (Ruckig) + gripper (passthrough) into full command
        self.merge_arm_gripper(
            arm_result, self.current_gripper_command, out=self.robot_command.joint_pos
        )
        return self.robot_command, self.complete

    def get_trajectory_command_without_update(self):
        """Get current trajectory command without updating."""
        return self.robot_command.copy()
