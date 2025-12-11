#!/usr/bin/env python3
"""SO101 End-Effector Teleoperation - single process, no LCM

Master: SO101 via MuJoCo passive viewer (drag control)
Slave: Target robot with OSC + Differential IK control

Usage:
    python so101_eehto.py fr3
    python so101_eehto.py piper -v
    python so101_eehto.py rm75

For joint-level teleoperation, see so101_jointhto.py
"""

import argparse
import numpy as np

from RynnMotion.algorithms import PoseMapper, PinKine
from RynnMotion.algorithms.osc import OSCtrl
from RynnMotion.utils.teleop_utils import TeleopAppBase, resolve_robot_number


class EEHTOApp(TeleopAppBase):
    """End-effector based heterogeneous teleoperation application."""

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
            window_title="Heterogeneous Teleop (EE-based) - Slave Robot",
        )

    def init_teleop(self):
        """Initialize PinKine, PoseMapper, OSC controller for EE-based teleop."""
        master_pin_mjcf = self.master_robot_manager.getPinoMJCF()
        slave_pin_mjcf = self.slave_robot_manager.getPinoMJCF()

        self.master_pinkine = PinKine(master_pin_mjcf, self.site_names)
        self.slave_pinkine = PinKine(slave_pin_mjcf, self.site_names)

        master_qStand = self.master_robot_manager.get_qStandby(self.arm_side)
        self.master_armlength_config = self._load_arm_length(self.master_pinkine, master_qStand)

        slave_qStand = self.slave_robot_manager.get_qStandby(self.arm_side)
        self.slave_armlength_config = self._load_arm_length(self.slave_pinkine, slave_qStand)

        if self.verbose:
            self._print_arm_lengths("Master", self.master_armlength_config)
            self._print_arm_lengths("Slave", self.slave_armlength_config)

        self.pose_mapper = PoseMapper(self.master_armlength_config, self.slave_armlength_config)
        self.osc_ctrl = OSCtrl(dt=self.ctrl_dt)

        if self.verbose:
            self._print_ee_poses(master_qStand, slave_qStand)
            print("PoseMapper, OSC initialized for EE-based teleop")

    def _load_arm_length(self, pinkine: PinKine, q_config: np.ndarray) -> dict:
        """Compute arm segment lengths from site positions.

        l0: base (origin) to shoulderSite
        l1: shoulderSite to elbowSite
        l2: elbowSite to wristSite
        l3: wristSite to EE
        """
        pinkine.update(q_config)

        base_pos = np.zeros(3)
        shoulder_pos = pinkine.getSitePosByName("shoulderSite")
        elbow_pos = pinkine.getSitePosByName("elbowSite")
        wrist_pos = pinkine.getSitePosByName("wristSite")
        ee_pos = pinkine.getSitePosByName("EE")

        return {
            "l0": shoulder_pos - base_pos,
            "l1": elbow_pos - shoulder_pos,
            "l2": wrist_pos - elbow_pos,
            "l3": ee_pos - wrist_pos,
        }

    def _print_arm_lengths(self, name: str, config: dict):
        """Print arm segment lengths."""
        print(f"\n{name} robot arm lengths (at qStand):")
        for key in ["l0", "l1", "l2", "l3"]:
            vec = config[key]
            length = np.linalg.norm(vec)
            print(f"  {key}: [{vec[0]:.4f}, {vec[1]:.4f}, {vec[2]:.4f}] (|{key}|={length:.4f}m)")

    def _print_ee_poses(self, master_qStand: np.ndarray, slave_qStand: np.ndarray):
        """Print EE poses at qStand for debugging."""
        self.master_pinkine.update(master_qStand)
        m_pos = self.master_pinkine.getSiteSE3ByName("EE").translation
        m_quat = self.master_pinkine.getSiteQuatByName("EE")

        self.slave_pinkine.update(slave_qStand)
        s_pos = self.slave_pinkine.getSiteSE3ByName("EE").translation
        s_quat = self.slave_pinkine.getSiteQuatByName("EE")

        print(f"\n=== EE Pose at qStand ===")
        print(f"Master EE: pos=[{m_pos[0]:.4f}, {m_pos[1]:.4f}, {m_pos[2]:.4f}], quat(xyzw)=[{m_quat[0]:.4f}, {m_quat[1]:.4f}, {m_quat[2]:.4f}, {m_quat[3]:.4f}]")
        print(f"Slave EE:  pos=[{s_pos[0]:.4f}, {s_pos[1]:.4f}, {s_pos[2]:.4f}], quat(xyzw)=[{s_quat[0]:.4f}, {s_quat[1]:.4f}, {s_quat[2]:.4f}, {s_quat[3]:.4f}]")

    def run_teleop(self):
        """Run EE-based teleoperation: FK on master, pose mapping, OSC + Diff IK."""
        with self.simulation_mutex:
            master_joint_positions = self.master_data.qpos.copy()
            master_gripper_cmd = self.master_data.ctrl[-1]
            slave_joint_positions = self.data.qpos.copy()

        # Step 1: Master FK
        m_dof = self.master_pinkine.pin_model.nq
        self.master_pinkine.update(master_joint_positions[:m_dof])
        m_eePose = self.master_pinkine.getSiteSE3ByName("EE")

        # Step 2: Slave FK (feedback)
        s_dof = self.slave_pinkine.pin_model.nq
        self.slave_pinkine.update(slave_joint_positions[:s_dof])
        s_eePoseFb = self.slave_pinkine.getSiteSE3ByName("EE")

        # Step 3: Pose mapping (master → slave workspace)
        s_eePoseRef = self.pose_mapper.pose_mapping(
            m_eePose, self.master_pinkine, self.slave_pinkine
        )

        # Step 4: OSC controller
        self.osc_ctrl.update(s_eePoseFb, s_eePoseRef)
        eeVel = self.osc_ctrl.get_eeVel()

        # Step 5: Differential IK
        s_jaco = self.slave_pinkine.getSiteJacoByName("EE")
        inverse_jaco = np.linalg.pinv(s_jaco.T @ s_jaco + 0.01 * np.eye(s_jaco.shape[1])) @ s_jaco.T
        qdot = inverse_jaco @ eeVel

        # Step 6: Integrate to get joint commands
        slave_joint_command = np.zeros(self.nctrl)
        slave_joint_command[:s_dof] = slave_joint_positions[:s_dof] + qdot * self.ctrl_dt

        # Step 7: Gripper mapping
        slave_joint_command[self.nctrl - 1] = self.map_gripper_command(master_gripper_cmd)

        with self.simulation_mutex:
            self.data.ctrl[:] = slave_joint_command


def main():
    parser = argparse.ArgumentParser(
        description="SO101 End-Effector Teleoperation",
        epilog="Supported targets: fr3, ur5e, piper, rm75",
    )
    parser.add_argument("robot", type=str, help="Target robot (fr3, ur5e, piper, rm75)")
    parser.add_argument("--verbose", "-v", action="store_true", help="Enable verbose output")
    parser.add_argument("--side", type=str, choices=["left", "right"], default="left",
                        help="Arm side for mapping (default: left)")

    args = parser.parse_args()
    slave_number = resolve_robot_number(args.robot)

    app = EEHTOApp(
        master_robot_number=24,
        slave_robot_number=slave_number,
        arm_side=args.side,
        verbose=args.verbose,
    )
    app.run()


if __name__ == "__main__":
    main()
