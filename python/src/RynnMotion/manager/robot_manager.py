"""Robot Manager for unified robot configuration access.

Python equivalent of motion/manager/robot_manager.cpp

Example:
    robot = RobotManager(config, logger)
    mdof = robot.get_motion_dof()
    joint_indices = robot.get_joint_indices()
"""

from abc import ABC, abstractmethod
from pathlib import Path
from typing import Dict, List, Tuple
import numpy as np
import logging

from RynnMotion.manager.mjcf_parser import MjcfParser, ActuatorMode
from RynnMotion.algorithms.pin_kine import PinKine


class RobotManager(ABC):
    """Unified robot configuration manager.

    Attributes:
        logger: Logger instance for debugging.
    """

    def __init__(self, robotmodel_config: dict, logger):
        """Initialize with robot configuration from YAML/dict.

        Args:
            robotmodel_config: Dict with robot_name, robot_mjcf, pino_mjcf, robot_control_freq.
            logger: Logger instance.

        Raises:
            ValueError: If robot MJCF parsing fails.
        """
        self.logger = logger or logging.getLogger(__name__)

        self._robot_name = robotmodel_config.get("robot_name", "")
        self._robot_control_freq = robotmodel_config.get("robot_control_freq", 100)
        self._robot_mjcf = robotmodel_config.get("robot_mjcf", "")
        self._pino_mjcf = robotmodel_config.get("pino_mjcf", "")

        self.logger.info("robot name: %s", self._robot_name)
        self.logger.info("robot mjcf: %s", self._robot_mjcf)
        self.logger.info("pino mjcf: %s", self._pino_mjcf)

        self._robot_state = self._load_robot_state()
        if self._robot_state is None:
            raise ValueError("Failed to load robot state from mjcf!")

        self._mdof: int = self._robot_state["mdof"]
        self._adof: int = self._robot_state["adof"]
        self._num_ee: int = self._robot_state["num_ee"]
        self._joint_indices: List[int] = self._robot_state["joint_indices"]
        self._ee_indices: List[int] = self._robot_state["ee_indices"]
        self._ee_joint_indices: List[int] = self._robot_state["ee_joint_indices"]
        self._ee_ranges: List[Tuple[float, float]] = self._robot_state["ee_ranges"]

        self._joint_limits = self._load_joint_limits()
        self._actuator_modes = self._load_actuator_modes()
        self._actuator_gains = self._load_actuator_gains()
        self._keyframes = self._load_keyframes()
        self._site_names = self._load_site_names()

        self.logger.info("mdof: %d, adof: %d, num_ee: %d", self._mdof, self._adof, self._num_ee)
        self.logger.info("joint_indices: %s", self._joint_indices)
        self.logger.info("ee_indices: %s", self._ee_indices)
        self.logger.info("ee_joint_indices: %s", self._ee_joint_indices)
        self.logger.info("Load robot manager successful!\n")

    def _load_robot_state(self) -> dict:
        """Load robot state from MJCF file"""
        if not self._robot_mjcf:
            self.logger.info("Robot mjcf is empty, load robot mjcf fail!")
            return None

        try:
            return MjcfParser.extract_robot_state_dim(self._robot_mjcf)
        except Exception as e:
            self.logger.error("Failed to extract robot state from %s: %s", self._robot_mjcf, e)
            return None

    def _load_joint_limits(self) -> dict:
        """Load joint limits from MJCF file"""
        if not self._robot_mjcf:
            return {"q_pos_min": np.array([]), "q_pos_max": np.array([]),
                    "q_vel_max": np.array([]), "q_torque_max": np.array([])}
        try:
            return MjcfParser.extract_joint_limits(self._robot_mjcf, self._joint_indices)
        except Exception as e:
            self.logger.warning("Failed to load joint limits: %s", e)
            return {"q_pos_min": np.zeros(self._mdof), "q_pos_max": np.zeros(self._mdof),
                    "q_vel_max": np.zeros(self._mdof), "q_torque_max": np.zeros(self._mdof)}

    def _load_actuator_modes(self) -> List[ActuatorMode]:
        """Load actuator modes from MJCF file"""
        if not self._robot_mjcf:
            return []
        try:
            return MjcfParser.detect_actuator_modes(self._robot_mjcf)
        except Exception as e:
            self.logger.warning("Failed to load actuator modes: %s", e)
            return []

    def _load_actuator_gains(self) -> dict:
        """Load actuator gains from MJCF file"""
        if not self._robot_mjcf:
            return {"sim_kp": np.array([]), "sim_kd": np.array([])}
        try:
            return MjcfParser.extract_actuator_gains(self._robot_mjcf)
        except Exception as e:
            self.logger.warning("Failed to load actuator gains: %s", e)
            return {"sim_kp": np.array([]), "sim_kd": np.array([])}

    def _load_keyframes(self) -> dict:
        """Load keyframes from robot MJCF (includes gripper joints)"""
        if not self._robot_mjcf:
            return {"keyframes": [], "names": [], "nq": 0}
        try:
            return MjcfParser.extract_keyframes(self._robot_mjcf)
        except Exception as e:
            self.logger.warning("Failed to load keyframes from %s: %s", self._robot_mjcf, e)
            return {"keyframes": [], "names": [], "nq": 0}

    def _load_site_names(self) -> List[str]:
        """Load site names from MJCF file"""
        if not self._robot_mjcf:
            return []
        try:
            return MjcfParser.extract_site_names(self._robot_mjcf)
        except Exception as e:
            self.logger.warning("Failed to load site names: %s", e)
            return []

    # ========== Core DOF methods (C++ aligned) ==========

    def get_motion_dof(self) -> int:
        """Get motion DOF (actuated joints excluding grippers)"""
        return self._mdof

    def get_action_dof(self) -> int:
        """Get action DOF per end-effector"""
        return self._adof

    def get_num_ee(self) -> int:
        """Get number of end-effectors"""
        return self._num_ee

    # ========== Index methods (C++ aligned) ==========

    def get_joint_indices(self) -> List[int]:
        """Get actuator indices for motion joints"""
        return self._joint_indices

    def get_ee_indices(self) -> List[int]:
        """Get actuator indices for grippers"""
        return self._ee_indices

    def get_ee_joint_indices(self) -> List[int]:
        """Get parent actuator ID for each gripper"""
        return self._ee_joint_indices

    def get_ee_ranges(self) -> List[Tuple[float, float]]:
        """Get control ranges for each end-effector"""
        return self._ee_ranges

    def normalize_ee_command(self, ee_index: int, normalized_cmd: float) -> float:
        """Convert normalized EE command [-1, 1] to actual control value."""
        if ee_index < 0 or ee_index >= len(self._ee_ranges):
            return 0.0
        min_range, max_range = self._ee_ranges[ee_index]
        return min_range + (normalized_cmd + 1.0) * 0.5 * (max_range - min_range)

    # ========== Joint limits (C++ aligned) ==========

    def get_joint_pos_min(self) -> np.ndarray:
        """Get joint position lower limits (mdof,)"""
        return self._joint_limits["q_pos_min"]

    def get_joint_pos_max(self) -> np.ndarray:
        """Get joint position upper limits (mdof,)"""
        return self._joint_limits["q_pos_max"]

    def get_joint_vel_max(self) -> np.ndarray:
        """Get joint velocity limits (mdof,)"""
        return self._joint_limits["q_vel_max"]

    def get_joint_torque_max(self) -> np.ndarray:
        """Get joint torque limits (mdof,)"""
        return self._joint_limits["q_torque_max"]

    # ========== Actuator modes and gains (C++ aligned) ==========

    def get_actuator_modes(self) -> List[ActuatorMode]:
        """Get actuator control modes for all actuators"""
        return self._actuator_modes

    def get_sim_kp(self) -> np.ndarray:
        """Get simulation position gains (nu,)"""
        return self._actuator_gains["sim_kp"]

    def get_sim_kd(self) -> np.ndarray:
        """Get simulation damping gains (nu,)"""
        return self._actuator_gains["sim_kd"]

    # ========== Site names ==========

    def get_site_names(self) -> List[str]:
        """Get all site names from MJCF"""
        return self._site_names

    def get_num_sites(self) -> int:
        """Get number of sites"""
        return len(self._site_names)

    # ========== Path getters ==========

    def get_robot_mjcf(self) -> str:
        """Get robot MJCF path"""
        return self._robot_mjcf

    def get_pino_mjcf(self) -> str:
        """Get Pinocchio MJCF path"""
        return self._pino_mjcf

    def get_robot_name(self) -> str:
        """Get robot name"""
        return self._robot_name

    def get_robot_control_freq(self) -> int:
        """Get robot control frequency"""
        return self._robot_control_freq

    # ========== Keyframe methods ==========

    def get_keyframe(self, index: int, motion_only: bool = True) -> np.ndarray:
        """Get keyframe by index (0=home, 1=standby1, 2=standby2, 3+=extra).

        Args:
            index: Keyframe index
            motion_only: If True, return only motion DOF (exclude gripper). Default True.
        """
        keyframes = self._keyframes.get("keyframes", [])
        nq = self._keyframes.get("nq", 0)
        if index < len(keyframes):
            kf = keyframes[index]
            if motion_only:
                return kf[:self._mdof]
            return kf
        return np.zeros(self._mdof if motion_only else nq)

    def get_q_standby(self, index: int = 0) -> np.ndarray:
        """Get standby keyframe (0=standby1, 1=standby2). Returns motion DOF only."""
        return self.get_keyframe(1 + index, motion_only=True)

    def get_q_home(self) -> np.ndarray:
        """Get home keyframe (keyframes[0]). Returns motion DOF only."""
        return self.get_keyframe(0, motion_only=True)

    def get_num_keyframes(self) -> int:
        """Get total number of keyframes"""
        return len(self._keyframes.get("keyframes", []))

    # ========== Sensor info ==========

    def get_ft_sensor_num(self) -> int:
        """Get number of force/torque sensors"""
        return self._robot_state.get("ft_sensor_num", 0)

    def get_ee_site_idx(self) -> List[int]:
        """Get site indices for end-effectors"""
        return self._robot_state.get("ee_site_idx", [])

    def get_ee_site_name(self) -> List[str]:
        """Get site names for end-effectors"""
        return self._robot_state.get("ee_site_name", [])

    # ========== Backward-compatible camelCase aliases ==========

    def getRobotName(self) -> str:
        """Alias for get_robot_name() (backward compatibility)"""
        return self.get_robot_name()

    def getRobotMJCF(self) -> str:
        """Alias for get_robot_mjcf() (backward compatibility)"""
        return self.get_robot_mjcf()

    def getPinoMJCF(self) -> str:
        """Alias for get_pino_mjcf() (backward compatibility)"""
        return self.get_pino_mjcf()

    def get_qHome(self) -> np.ndarray:
        """Alias for get_q_home() (backward compatibility)"""
        return self.get_q_home()

    def get_qStand(self) -> np.ndarray:
        """Alias for get_q_standby(0) (backward compatibility)"""
        return self.get_q_standby(0)

    def get_qStandby(self, index=0) -> np.ndarray:
        """Alias for get_q_standby() (backward compatibility)"""
        if isinstance(index, str):
            index = 1 if index == "right" else 0
        return self.get_q_standby(index)

    # ========== Interface compatibility aliases ==========

    def get_actuator_num(self) -> int:
        """Get total number of actuators (mdof + adof)"""
        return self._mdof + self._adof

    def get_ee_num(self) -> int:
        """Alias for get_num_ee() (interface compatibility)"""
        return self.get_num_ee()

    def get_gripper_num(self) -> int:
        """Get number of grippers (same as num_ee)"""
        return self._num_ee

    def get_site_num(self) -> int:
        """Alias for get_num_sites() (interface compatibility)"""
        return self.get_num_sites()

    def get_joint_state_num(self) -> int:
        """Get joint state dimension (same as mdof)"""
        return self._mdof

    def get_ee_site_index(self) -> List[int]:
        """Alias for get_ee_site_idx() (interface compatibility)"""
        return self.get_ee_site_idx()

    def get_gripper_joint_name(self) -> List[str]:
        """Get gripper joint names from MJCF"""
        return self._robot_state.get("gripper_joint_names", [])
