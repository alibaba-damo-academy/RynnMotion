#!/usr/bin/env python3
"""SO101 teleoperation robot configuration.

Defines joint direction mappings and scene paths for supported robots.
"""

import numpy as np
from typing import Dict, Any, Tuple


# Robot configuration dictionary
ROBOT_CONFIGS: Dict[str, Dict[str, Any]] = {
    "fr3": {
        "number": 20,
        "name": "FR3",
        "scene_path": "4.dual_arm/40.dual_fr3/scene/dual_fr3_scene.xml",
        "qpos_arm1": (0, 9),
        "ctrl_arm1": (0, 8),
        "qpos_arm2": (9, 18),
        "ctrl_arm2": (8, 16),
        "qDirection_left": np.array([-1, 1, 0, -1, 0, -1, -1]),
        "qDirection_right": np.array([-1, 1, 0, -1, 0, -1, -1]),
    },
    "ur5e": {
        "number": 21,
        "name": "UR5e",
        "scene_path": "4.dual_arm/41.dual_ur5e/scene/dual_ur5e_scene.xml",
        "qpos_arm1": (0, 8),
        "ctrl_arm1": (0, 7),
        "qpos_arm2": (8, 16),
        "ctrl_arm2": (7, 14),
        "qDirection_left": np.array([-1, -1, -1, -1, 0, -1]),
        "qDirection_right": np.array([-1, 1, 1, 1, 0, -1]),
    },
    "piper": {
        "number": 22,
        "name": "Piper",
        "scene_path": "4.dual_arm/42.dual_piper/scene/dual_piper_scene.xml",
        "qpos_arm1": (0, 8),
        "ctrl_arm1": (0, 7),
        "qpos_arm2": (8, 16),
        "ctrl_arm2": (7, 14),
        "qDirection_left": np.array([-1, 1, 1, 0, 1, -1]),
        "qDirection_right": np.array([-1, 1, 1, 0, 1, -1]),
    },
    "rm75": {
        "number": 23,
        "name": "RM75",
        "scene_path": "4.dual_arm/43.dual_rm75/scene/dual_rm75_scene.xml",
        "qpos_arm1": (0, 9),
        "ctrl_arm1": (0, 8),
        "qpos_arm2": (9, 18),
        "ctrl_arm2": (8, 16),
        "qDirection_left": np.array([-1, 1, 0, 1, 0, 1, -1]),
        "qDirection_right": np.array([-1, -1, 0, -1, 0, -1, -1]),
    },
}

# Create reverse lookup: number -> name
NUMBER_TO_NAME = {cfg["number"]: name for name, cfg in ROBOT_CONFIGS.items()}


def get_robot_config(robot_identifier: Any) -> Tuple[str, Dict[str, Any]]:
    """
    Get robot configuration by name or number.

    Args:
        robot_identifier: Either robot name (str) or number (int)

    Returns:
        Tuple of (robot_name, config_dict)

    Raises:
        ValueError: If robot not found

    Examples:
        >>> name, cfg = get_robot_config("fr3")
        >>> name, cfg = get_robot_config(20)  # Same as "fr3"
    """
    # Handle integer or string number
    if isinstance(robot_identifier, (int, str)) and str(robot_identifier).isdigit():
        robot_num = int(robot_identifier)
        if robot_num in NUMBER_TO_NAME:
            robot_name = NUMBER_TO_NAME[robot_num]
            return robot_name, ROBOT_CONFIGS[robot_name]
        else:
            valid_numbers = sorted(NUMBER_TO_NAME.keys())
            raise ValueError(f"Invalid robot number: {robot_num}. " f"Valid numbers: {valid_numbers}")

    # Handle robot name
    elif isinstance(robot_identifier, str):
        robot_name = robot_identifier.lower()
        if robot_name in ROBOT_CONFIGS:
            return robot_name, ROBOT_CONFIGS[robot_name]
        else:
            valid_names = sorted(ROBOT_CONFIGS.keys())
            raise ValueError(f"Invalid robot name: '{robot_identifier}'. " f"Valid names: {valid_names}")

    else:
        raise ValueError(f"Robot identifier must be name (str) or number (int), got {type(robot_identifier)}")


def qDirection_to_dirMat(qDirection: np.ndarray, m_dof: int) -> np.ndarray:
    """
    Convert qDirection vector to direction matrix.

    Args:
        qDirection: Direction vector indexed by slave joints
        m_dof: Master DOF

    Returns:
        dirMat: Direction matrix (m_dof × s_dof)
    """
    s_dof = len(qDirection)
    dirMat = np.zeros((m_dof, s_dof))
    master_idx = 0

    for slave_idx in range(s_dof):
        if qDirection[slave_idx] != 0:
            dirMat[master_idx, slave_idx] = qDirection[slave_idx]
            master_idx += 1

    return dirMat


def compute_slave_command(master_qCmd: np.ndarray, master_qStand: np.ndarray, slave_qStand: np.ndarray, qDirection: np.ndarray) -> np.ndarray:
    """
    Compute slave robot joint commands using matrix-based mapping.

    Formula: slave_qRef = slave_qStand + dirMat.T @ (master_qCmd - master_qStand)

    Args:
        master_qCmd: Master robot current joint positions
        master_qStand: Master robot standby position
        slave_qStand: Slave robot standby position
        qDirection: Direction vector for joint mapping

    Returns:
        Slave robot joint commands
    """
    m_dof = len(master_qCmd)
    s_dof = len(qDirection)

    dirMat = qDirection_to_dirMat(qDirection, m_dof)

    master_delta = master_qCmd - master_qStand[:m_dof]
    slave_delta = dirMat.T @ master_delta
    slave_qRef = slave_qStand[:s_dof] + slave_delta

    return slave_qRef


if __name__ == "__main__":
    # Self-test
    print("SO101 Teleoperation Robot Configurations")
    print("=" * 60)

    for name in sorted(ROBOT_CONFIGS.keys()):
        cfg = ROBOT_CONFIGS[name]
        print(f"\n{cfg['name']} (Robot #{cfg['number']})")
        print(f"  Scene: {cfg['scene_path']}")
        print(f"  Arm 1: qpos {cfg['qpos_arm1']}, ctrl {cfg['ctrl_arm1']}")
        print(f"  Arm 2: qpos {cfg['qpos_arm2']}, ctrl {cfg['ctrl_arm2']}")
        print(f"  qDirection_left: {cfg['qDirection_left']}")
        print(f"  qDirection_right: {cfg['qDirection_right']}")

    print("\n" + "=" * 60)
    print("\nLookup tests:")
    print(f"  get_robot_config('fr3'): {get_robot_config('fr3')[0]}")
    print(f"  get_robot_config(20): {get_robot_config(20)[0]}")
    print(f"  get_robot_config('21'): {get_robot_config('21')[0]}")
