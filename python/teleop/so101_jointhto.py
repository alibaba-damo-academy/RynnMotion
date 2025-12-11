#!/usr/bin/env python3
"""SO101 Joint Teleoperation - single process, no LCM

Master: SO101 via MuJoCo passive viewer (drag control)
Slave: Target robot with GLFW rendering

Usage:
    python so101_jointhto.py fr3
    python so101_jointhto.py piper -v
    python so101_jointhto.py rm75 --side right

For EE-based teleoperation with OSC + Differential IK, see so101_eehto.py
"""

import argparse
import numpy as np

from RynnMotion.algorithms import PinKine
from RynnMotion.utils.teleop_utils import TeleopAppBase, resolve_robot_number


class JointHTOApp(TeleopAppBase):
    """Joint-level heterogeneous teleoperation application."""

    def __init__(
        self,
        master_robot_number: int,
        slave_robot_number: int,
        arm_side: str = "left",
        verbose: bool = False,
    ):
        super().__init__(
            master_robot_number=master_robot_number,
            slave_robot_number=slave_robot_number,
            arm_side=arm_side,
            verbose=verbose,
            window_title="Heterogeneous Teleop (Joint Mapping) - Slave Robot",
        )

    def init_teleop(self):
        """Initialize PinKine for joint mapping."""
        master_pin_mjcf = self.master_robot_manager.getPinoMJCF()
        slave_pin_mjcf = self.slave_robot_manager.getPinoMJCF()

        self.master_pinkine = PinKine(master_pin_mjcf, self.site_names)
        self.slave_pinkine = PinKine(slave_pin_mjcf, self.site_names)

        if self.verbose:
            print("✅ PinKine initialized for joint mapping")

    def run_teleop(self):
        """Run joint-to-joint teleoperation with robot-specific mapping."""
        with self.simulation_mutex:
            master_joint_positions = self.master_data.qpos.copy()
            master_gripper_cmd = self.master_data.ctrl[-1]

        m_dof = self.master_pinkine.pin_model.nq
        master_motion_joints = master_joint_positions[:m_dof]

        slave_joint_command = np.zeros(self.nctrl)
        s_dof = self.slave_pinkine.pin_model.nq

        # Robot-specific joint mapping
        if self.master_robot_number == 24 and self.slave_robot_number == 22:
            slave_joint_command[:s_dof] = self._map_so101_piper(master_motion_joints)
        elif self.master_robot_number == 24 and self.slave_robot_number == 20:
            slave_joint_command[:s_dof] = self._map_so101_fr3(master_motion_joints)
        elif self.master_robot_number == 24 and self.slave_robot_number == 21:
            slave_joint_command[:s_dof] = self._map_so101_ur5e(master_motion_joints)
        elif self.master_robot_number == 24 and self.slave_robot_number == 23:
            slave_joint_command[:s_dof] = self._map_so101_rm75(master_motion_joints)
        else:
            if self.verbose:
                print(f"⚠️  Unsupported: master #{self.master_robot_number} → slave #{self.slave_robot_number}")
            slave_joint_command[:min(m_dof, s_dof)] = master_motion_joints[:min(m_dof, s_dof)]

        # Gripper mapping
        slave_joint_command[self.nctrl - 1] = self.map_gripper_command(master_gripper_cmd)

        with self.simulation_mutex:
            self.data.ctrl[:] = slave_joint_command

    def _qdir_to_matrix(self, q_direction: np.ndarray, m_dof: int) -> np.ndarray:
        """Convert direction vector to mapping matrix."""
        s_dof = len(q_direction)
        dir_mat = np.zeros((m_dof, s_dof))
        master_idx = 0
        for slave_idx in range(s_dof):
            if q_direction[slave_idx] != 0:
                dir_mat[master_idx, slave_idx] = q_direction[slave_idx]
                master_idx += 1
        return dir_mat

    def _apply_mapping(self, master_q: np.ndarray, q_direction: np.ndarray) -> np.ndarray:
        """Apply joint mapping with direction vector."""
        master_qStand = self.master_robot_manager.get_qStandby(self.arm_side)
        slave_qStand = self.slave_robot_manager.get_qStandby(self.arm_side)

        m_dof = self.master_pinkine.pin_model.nq
        s_dof = self.slave_pinkine.pin_model.nq

        dir_mat = self._qdir_to_matrix(q_direction, m_dof)
        master_delta = master_q - master_qStand[:m_dof]
        slave_delta = dir_mat.T @ master_delta

        return slave_qStand[:s_dof] + slave_delta

    def _map_so101_piper(self, master_q: np.ndarray) -> np.ndarray:
        """SO101 → Piper joint mapping."""
        return self._apply_mapping(master_q, np.array([-1, 1, 1, 0, 1, -1]))

    def _map_so101_fr3(self, master_q: np.ndarray) -> np.ndarray:
        """SO101 → FR3 joint mapping."""
        return self._apply_mapping(master_q, np.array([-1, 1, 0, -1, 0, -1, -1]))

    def _map_so101_ur5e(self, master_q: np.ndarray) -> np.ndarray:
        """SO101 → UR5e joint mapping (arm_side dependent)."""
        if self.arm_side == "left":
            q_dir = np.array([-1, -1, -1, -1, 0, -1])
        else:
            q_dir = np.array([-1, 1, 1, 1, 0, -1])
        return self._apply_mapping(master_q, q_dir)

    def _map_so101_rm75(self, master_q: np.ndarray) -> np.ndarray:
        """SO101 → RM75 joint mapping (arm_side dependent)."""
        if self.arm_side == "left":
            q_dir = np.array([-1, 1, 0, 1, 0, 1, -1])
        else:
            q_dir = np.array([-1, -1, 0, -1, 0, -1, -1])
        return self._apply_mapping(master_q, q_dir)


def main():
    parser = argparse.ArgumentParser(
        description="SO101 Joint Teleoperation",
        epilog="Supported targets: fr3, ur5e, piper, rm75",
    )
    parser.add_argument("robot", type=str, help="Target robot (fr3, ur5e, piper, rm75)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Enable verbose output")
    parser.add_argument("--side", type=str, choices=["left", "right"], default="left",
                        help="Arm side for mapping (default: left)")

    args = parser.parse_args()
    slave_number = resolve_robot_number(args.robot)

    app = JointHTOApp(
        master_robot_number=24,
        slave_robot_number=slave_number,
        arm_side=args.side,
        verbose=args.verbose,
    )
    app.run()


if __name__ == "__main__":
    main()
