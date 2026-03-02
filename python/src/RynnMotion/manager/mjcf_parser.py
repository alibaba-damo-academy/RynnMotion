"""MJCF Parser for robot configuration extraction.

Python equivalent of motion/manager/mjcf_parser.cpp

Example:
    state = MjcfParser.extract_robot_state_dim("fr3_robot.xml")
    mdof = state['mdof']
    joint_indices = state['joint_indices']
"""

import re
from enum import Enum
from typing import List, Tuple

import numpy as np
import mujoco


class ActuatorMode(Enum):
    """MuJoCo actuator control modes (matches C++ rynn::ActuatorMode)"""

    GENERAL = 0
    POSITION = 1
    VELOCITY = 2
    TORQUE = 3
    INT_VELOCITY = 4
    DAMPER = 5
    CYLINDER = 6
    MUSCLE = 7
    ADHESION = 8
    PLUGIN = 9


# Gripper detection patterns (matches C++ mjcf_parser.cpp)
GRIPPER_PATTERNS = ["grip", "gripper", "finger", "hand"]


class MjcfParser:
    """MJCF file parser for extracting robot configuration."""

    @staticmethod
    def _extract_base_name(name: str) -> str:
        """Remove trailing numeric suffix from name."""
        if not name:
            return ""
        match = re.match(r"^(.+?)_\d+$", name)
        return match.group(1) if match else name

    @staticmethod
    def _is_gripper_actuator(mj_model, actuator_idx: int) -> bool:
        """Check if actuator is a gripper based on name patterns or tendon transmission."""
        name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_idx)
        if not name:
            return False

        base_name = MjcfParser._extract_base_name(name).lower()

        for pattern in GRIPPER_PATTERNS:
            if pattern in base_name or base_name == pattern:
                return True

        trntype = mj_model.actuator_trntype[actuator_idx]
        if trntype == mujoco.mjtTrn.mjTRN_TENDON:
            return True

        return False

    @staticmethod
    def _compute_ee_parent_joints(
        joint_indices: List[int], ee_indices: List[int]
    ) -> List[int]:
        """Compute parent joint actuator ID for each end-effector."""
        ee_joint_indices = []
        for ee_actuator_idx in ee_indices:
            parent_joint_id = -1
            for joint_idx in joint_indices:
                if joint_idx < ee_actuator_idx:
                    parent_joint_id = joint_idx
                else:
                    break
            if parent_joint_id == -1 and joint_indices:
                parent_joint_id = joint_indices[-1]
            ee_joint_indices.append(parent_joint_id)
        return ee_joint_indices

    @staticmethod
    def extract_robot_state_dim(mjcf_path: str) -> dict:
        """
        Extract robot state dimensions from MJCF file (C++ aligned)

        Matches C++ MjcfParser::detectEndEffectors() and computeEEParentJoints()

        Args:
            mjcf_path: Path to robot MJCF file (e.g., fr3_robot.xml)

        Returns:
            Dictionary with robot configuration:
            {
                'mdof': int,                      # Motion DOF (motion actuators count)
                'adof': int,                      # Action DOF per EE (1, hardcoded)
                'num_ee': int,                    # Number of end-effectors

                'joint_indices': List[int],       # Actuator indices for motion joints
                'ee_indices': List[int],          # Actuator indices for grippers
                'ee_joint_indices': List[int],    # Parent actuator ID for each gripper

                'ee_ranges': List[Tuple[float, float]],  # Control ranges for each EE

                'ft_sensor_num': int,
                'site_num': int,
                'ee_site_idx': List[int],
                'ee_site_name': List[str],
            }

        Example:
            >>> state = MjcfParser.extract_robot_state_dim("fr3_robot.xml")
            >>> mdof = state['mdof']              # 7 for FR3
            >>> num_ee = state['num_ee']          # 1 for single gripper
            >>> joint_indices = state['joint_indices']  # [0,1,2,3,4,5,6]
            >>> ee_indices = state['ee_indices']        # [7]
        """
        mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        nu = mj_model.nu
        site_num = mj_model.nsite

        ft_sensor_num = 0
        for i in range(mj_model.nsensor):
            if mj_model.sensor_type[i] == mujoco.mjtSensor.mjSENS_FORCE:
                ft_sensor_num += 1

        joint_indices = []
        ee_indices = []
        ee_ranges = []

        for i in range(nu):
            if MjcfParser._is_gripper_actuator(mj_model, i):
                ee_indices.append(i)
                min_range = mj_model.actuator_ctrlrange[i, 0]
                max_range = mj_model.actuator_ctrlrange[i, 1]
                ee_ranges.append((float(min_range), float(max_range)))
            else:
                joint_indices.append(i)

        ee_joint_indices = MjcfParser._compute_ee_parent_joints(
            joint_indices, ee_indices
        )

        mdof = len(joint_indices)
        adof = len(ee_indices)  # Each gripper is 1 DOF
        num_ee = len(ee_joint_indices)

        ee_site_idx = []
        ee_site_name = []
        for i in range(site_num):
            name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_SITE, i)
            if name and ("ee" in name.lower()):
                ee_site_idx.append(i)
                ee_site_name.append(name)

        return {
            "mdof": mdof,
            "adof": adof,
            "num_ee": num_ee,
            "joint_indices": joint_indices,
            "ee_indices": ee_indices,
            "ee_joint_indices": ee_joint_indices,
            "ee_ranges": ee_ranges,
            "ft_sensor_num": ft_sensor_num,
            "site_num": site_num,
            "ee_site_idx": ee_site_idx,
            "ee_site_name": ee_site_name,
        }

    @staticmethod
    def extractRobotStateDim(mjcf_path: str) -> dict:
        """DEPRECATED: Use extract_robot_state_dim() instead"""
        return MjcfParser.extract_robot_state_dim(mjcf_path)

    @staticmethod
    def extractKeyframes(mjcf_path: str) -> dict:
        """
        Extract keyframe configurations from MJCF file

        Extracts ALL keyframes and organizes them using universal indexing:
        - Index 0: "home" - Home/initial position
        - Index 1: "standby1" - First standby position
        - Index 2: "standby2" - Second standby position
        - Index 3+: Extra keyframes (e.g., "standby3", "rest", etc.)

        Args:
            mjcf_path: Path to MJCF file (e.g., fr3_pinocchio.xml)

        Returns:
            Dictionary with keyframe data:
            {
                'keyframes': list[np.ndarray],  # Indexed keyframes [q0, q1, q2, ...]
                'names': list[str],              # Keyframe names for reference
                'nq': int,                       # Number of position DOF
                'home': np.ndarray,              # Backward compat - same as keyframes[0]
                'stand': np.ndarray,             # Backward compat - same as keyframes[1]
            }

        Example:
            >>> kf = MjcfParser.extractKeyframes("fr3_pinocchio.xml")
            >>> q_home = kf['keyframes'][0]      # Index 0: home
            >>> q_standby1 = kf['keyframes'][1]  # Index 1: standby1
            >>> q_standby2 = kf['keyframes'][2]  # Index 2: standby2
        """
        model = mujoco.MjModel.from_xml_path(mjcf_path)
        data = mujoco.MjData(model)
        nq = model.nq

        UNIVERSAL_KEYFRAME_NAMES = ["home", "standby1", "standby2"]
        keyframes_list = []
        names_list = []

        for expected_name in UNIVERSAL_KEYFRAME_NAMES:
            key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, expected_name)
            if key_id != -1:
                mujoco.mj_resetDataKeyframe(model, data, key_id)
                keyframes_list.append(data.qpos.copy())
                names_list.append(expected_name)
            else:
                keyframes_list.append(np.zeros(nq))
                names_list.append(expected_name)

        for key_idx in range(model.nkey):
            key_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, key_idx)
            if key_name and key_name not in UNIVERSAL_KEYFRAME_NAMES:
                mujoco.mj_resetDataKeyframe(model, data, key_idx)
                keyframes_list.append(data.qpos.copy())
                names_list.append(key_name)

        return {
            "keyframes": keyframes_list,
            "names": names_list,
            "nq": nq,
            "home": keyframes_list[0] if len(keyframes_list) > 0 else np.zeros(nq),
            "stand": keyframes_list[1] if len(keyframes_list) > 1 else np.zeros(nq),
        }

    @staticmethod
    def extract_keyframes(mjcf_path: str) -> dict:
        """Snake_case alias for extractKeyframes (C++ style alignment)"""
        return MjcfParser.extractKeyframes(mjcf_path)

    @staticmethod
    def extract_joint_limits(mjcf_path: str, joint_indices: List[int] = None) -> dict:
        """
        Extract joint limits from MJCF file (C++ aligned)

        Matches C++ MjcfParser::extractJointLimits()

        Args:
            mjcf_path: Path to MJCF file
            joint_indices: Optional list of actuator indices for motion joints
                          If None, extracts for all actuators

        Returns:
            Dictionary with limit data:
            {
                'q_pos_min': np.ndarray,     # (mdof,) position lower limits
                'q_pos_max': np.ndarray,     # (mdof,) position upper limits
                'q_vel_max': np.ndarray,     # (mdof,) velocity limits
                'q_torque_max': np.ndarray,  # (mdof,) torque limits
            }
        """
        mj_model = mujoco.MjModel.from_xml_path(mjcf_path)

        if joint_indices is None:
            robot_state = MjcfParser.extract_robot_state_dim(mjcf_path)
            joint_indices = robot_state["joint_indices"]

        mdof = len(joint_indices)

        q_pos_min = np.zeros(mdof)
        q_pos_max = np.zeros(mdof)
        q_vel_max = np.zeros(mdof)
        q_torque_max = np.zeros(mdof)

        for i, act_idx in enumerate(joint_indices):
            jnt_id = mj_model.actuator_trnid[act_idx, 0]
            if jnt_id >= 0 and jnt_id < mj_model.njnt:
                q_pos_min[i] = mj_model.jnt_range[jnt_id, 0]
                q_pos_max[i] = mj_model.jnt_range[jnt_id, 1]

            q_vel_max[i] = abs(mj_model.actuator_ctrlrange[act_idx, 1])

            if mj_model.actuator_forcerange[act_idx, 1] > 0:
                q_torque_max[i] = mj_model.actuator_forcerange[act_idx, 1]
            else:
                q_torque_max[i] = 100.0

        return {
            "q_pos_min": q_pos_min,
            "q_pos_max": q_pos_max,
            "q_vel_max": q_vel_max,
            "q_torque_max": q_torque_max,
        }

    @staticmethod
    def extractJointLimits(mjcf_path: str) -> dict:
        """DEPRECATED: Use extract_joint_limits() instead"""
        return MjcfParser.extract_joint_limits(mjcf_path)

    @staticmethod
    def detect_actuator_modes(mjcf_path: str) -> List[ActuatorMode]:
        """
        Detect actuator control modes from MJCF file (C++ aligned)

        Matches C++ MjcfParser::detectActuatorModes()

        Args:
            mjcf_path: Path to MJCF file

        Returns:
            List of ActuatorMode for each actuator
        """
        mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        nu = mj_model.nu
        modes = []

        for i in range(nu):
            gaintype = mj_model.actuator_gaintype[i]
            biastype = mj_model.actuator_biastype[i]
            dyntype = mj_model.actuator_dyntype[i]

            if gaintype == mujoco.mjtGain.mjGAIN_FIXED:
                if biastype == mujoco.mjtBias.mjBIAS_NONE:
                    modes.append(ActuatorMode.TORQUE)
                elif biastype == mujoco.mjtBias.mjBIAS_AFFINE:
                    kp = mj_model.actuator_gainprm[i, 0]
                    kv = abs(mj_model.actuator_biasprm[i, 2])
                    if kp > 0 and kv > 0:
                        modes.append(ActuatorMode.POSITION)
                    elif kp == 0 and kv > 0:
                        modes.append(ActuatorMode.VELOCITY)
                    else:
                        modes.append(ActuatorMode.GENERAL)
                else:
                    modes.append(ActuatorMode.GENERAL)
            elif gaintype == mujoco.mjtGain.mjGAIN_AFFINE:
                modes.append(ActuatorMode.GENERAL)
            elif gaintype == mujoco.mjtGain.mjGAIN_MUSCLE:
                modes.append(ActuatorMode.MUSCLE)
            else:
                modes.append(ActuatorMode.GENERAL)

            if dyntype == mujoco.mjtDyn.mjDYN_INTEGRATOR:
                modes[-1] = ActuatorMode.INT_VELOCITY

        return modes

    @staticmethod
    def detectActuatorModes(mjcf_path: str) -> List[ActuatorMode]:
        """DEPRECATED: Use detect_actuator_modes() instead"""
        return MjcfParser.detect_actuator_modes(mjcf_path)

    @staticmethod
    def extract_actuator_gains(mjcf_path: str) -> dict:
        """
        Extract actuator PD gains from MJCF file (C++ aligned)

        Matches C++ MjcfParser::extractActuatorGains()

        Args:
            mjcf_path: Path to MJCF file

        Returns:
            Dictionary with gain data:
            {
                'sim_kp': np.ndarray,  # (nu,) position gains
                'sim_kd': np.ndarray,  # (nu,) damping gains
            }
        """
        mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        nu = mj_model.nu

        sim_kp = np.zeros(nu)
        sim_kd = np.zeros(nu)

        for i in range(nu):
            sim_kp[i] = mj_model.actuator_gainprm[i, 0]
            sim_kd[i] = abs(mj_model.actuator_biasprm[i, 2])

        return {
            "sim_kp": sim_kp,
            "sim_kd": sim_kd,
        }

    @staticmethod
    def extract_site_names(mjcf_path: str) -> List[str]:
        """
        Extract all site names from MJCF file

        Sites in MJCF are attachment points for:
        - End effectors (EE)
        - Sensors (cameras, force sensors)
        - Visual markers
        - Key reference points (shoulderSite, elbowSite, wristSite)

        Args:
            mjcf_path: Path to MJCF file

        Returns:
            List of site names

        Example:
            >>> sites = MjcfParser.extract_site_names("fr3_pinocchio.xml")
            >>> print(sites)
            ['EE', 'shoulderSite', 'elbowSite', 'wristSite']
        """
        mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        site_names = []
        for i in range(mj_model.nsite):
            name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_SITE, i)
            if name:
                site_names.append(name)
        return site_names

    @staticmethod
    def extractSiteNames(mjcf_path: str) -> List[str]:
        """DEPRECATED: Use extract_site_names() instead"""
        return MjcfParser.extract_site_names(mjcf_path)
