"""
sine wave trajectory generator for simulation motion.

Arm joints generate sine/cosine/triangle wave signals.
Gripper joints hold at home position (passthrough).
"""

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
    """Factory function for SimMotionGenerator."""
    return SimMotionGenerator(generator_config, robot_model, communicator, logger)


class SimMotionGenerator(TrajectoryGeneratorBase):
    """
    Sine wave trajectory generator.

    Arm joints receive sine/cosine/triangle wave signals.
    Gripper joints hold at home position.
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

        self.update_dt = 1.0 / self.robot_model.get_robot_control_freq()
        self.signal_time = 0.0

        # Initial positions (full act_dofs, split later)
        self.init_arm_position = [0.0] * self.arm_dofs
        self.init_gripper_position = [0.0] * self.gripper_dofs

        gen_cfg = generator_config.get("generator", {})
        self.signal_type = gen_cfg.get("type", "sine").lower()
        if self.signal_type not in self.SIGNAL_TYPES:
            raise ValueError(f"signal_type must be one of {self.SIGNAL_TYPES}")
        self.durtime = gen_cfg.get("durtime", 3.0)

        # Load home position and amplitude (full act_dofs or arm_dofs)
        home_position = gen_cfg.get("signal_home_position", None)
        signal_amplitude = gen_cfg.get("signal_amplitude", None)
        self.signal_frequency = gen_cfg.get("signal_frequency", 0)

        # Split into arm/gripper components
        if len(home_position) == self.act_dofs:
            self.arm_home_position, self.gripper_home_position = self.split_arm_gripper(
                home_position
            )
            self.arm_amplitude, _ = self.split_arm_gripper(signal_amplitude)
        elif len(home_position) == self.arm_dofs:
            # Config only provides arm DOFs
            self.arm_home_position = list(home_position)
            self.gripper_home_position = [0.0] * self.gripper_dofs
            self.arm_amplitude = list(signal_amplitude)
        else:
            raise ValueError(
                f"home_position length {len(home_position)} must be "
                f"act_dofs ({self.act_dofs}) or arm_dofs ({self.arm_dofs})"
            )

        if len(self.arm_amplitude) != self.arm_dofs:
            raise ValueError(
                f"arm signal_amplitude length {len(self.arm_amplitude)} != arm_dofs {self.arm_dofs}"
            )

        self.logger.info(
            f"Simulation {self.signal_type} trajectory: "
            f"arm_dofs={self.arm_dofs}, gripper_dofs={self.gripper_dofs}"
        )

    def _generate_arm_signal(self, signal_t) -> list:
        """Generate arm signal at time signal_t based on signal_type."""
        arm_pos = [0.0] * self.arm_dofs
        if self.signal_type == "sine":
            for i in range(self.arm_dofs):
                arm_pos[i] = self.arm_home_position[i] + self.arm_amplitude[
                    i
                ] * np.sin(2 * np.pi * self.signal_frequency * signal_t)
        elif self.signal_type == "cosine":
            for i in range(self.arm_dofs):
                arm_pos[i] = self.arm_home_position[i] + self.arm_amplitude[
                    i
                ] * np.cos(2 * np.pi * self.signal_frequency * signal_t)
        elif self.signal_type == "triangle":
            phase = self.signal_frequency * signal_t
            frac = phase - np.floor(phase)
            tri = np.where(frac < 0.5, 4.0 * frac - 1.0, -4.0 * frac + 3.0)
            tri = tri + 1.0
            for i in range(self.arm_dofs):
                arm_pos[i] = self.arm_home_position[i] + self.arm_amplitude[i] * tri
        return arm_pos

    def move_to_home_position(self, signal_t):
        """Lerp arm to home position, gripper holds at initial."""
        arm_pos = [
            lerp(self.init_arm_position[i], self.arm_home_position[i], signal_t, self.durtime)
            for i in range(self.arm_dofs)
        ]
        gripper_pos = self.gripper_home_position
        self.merge_arm_gripper(arm_pos, gripper_pos, out=self.robot_command.joint_pos)

    def process_input_command(self, latest_command, new_command_flag):
        """Input command is from simulation motion, communicator is always connected."""
        self.communicator_connected = True

    def reset_trajectory_state(self, robot_state):
        """Prepare trajectory: set robot state once."""
        if robot_state is None:
            return
        self.robot_connected = True

        # Split initial state into arm/gripper
        full_pos = robot_state.joint_pos[: self.act_dofs]
        self.init_arm_position, self.init_gripper_position = self.split_arm_gripper(
            full_pos
        )
        self.logger.info(
            f"Trajectory reset — arm init: {self.init_arm_position}, "
            f"gripper init: {self.init_gripper_position}"
        )

    def update_trajectory(self):
        """
        Update simulation trajectory.

        Arm: sine/cosine/triangle wave signal.
        Gripper: holds at home position.

        Returns:
            (robot_command, complete)
        """
        complete = False
        if not self.robot_connected:
            return None, True

        if self.signal_time < self.durtime:
            self.move_to_home_position(self.signal_time)
        else:
            complete = True
            t_s = self.signal_time - self.durtime
            arm_pos = self._generate_arm_signal(t_s)
            gripper_pos = self.gripper_home_position
            self.merge_arm_gripper(
                arm_pos, gripper_pos, out=self.robot_command.joint_pos
            )

        self.signal_time += self.update_dt
        return self.robot_command, complete

    def get_trajectory_command_without_update(self):
        """Get current trajectory command without updating."""
        return self.robot_command.copy()
