"""
PoseMapper - Maps poses between master and slave robots for teleoperation

Provides workspace scaling and orientation bias compensation between robots
"""

import numpy as np
import pinocchio as pin


class PoseMapper:
    """
    Maps end-effector poses from master robot to slave robot workspace

    Features:
    - Position scaling based on arm length ratios
    - Orientation bias compensation between robot bases
    """

    def __init__(self, arm1L: dict, arm2L: dict):
        """
        Initialize PoseMapper

        Args:
            arm1L: Master robot arm length config (l0, l1, l2, l3)
            arm2L: Slave robot arm length config (l0, l1, l2, l3)
        """
        self.arm1L = arm1L
        self.arm2L = arm2L

        master_arm_length = np.linalg.norm(arm1L["l1"]) + np.linalg.norm(arm1L["l2"])
        slave_arm_length = np.linalg.norm(arm2L["l1"]) + np.linalg.norm(arm2L["l2"])
        self.alpha = slave_arm_length / master_arm_length
        # print(f"PoseMapper alpha (slave/master arm length ratio): {self.alpha:.4f}")

    def pose_mapping(self, m_eePose, master_pinkine, slave_pinkine):
        """
        Map master robot pose to slave robot workspace

        Args:
            m_eePose: Master robot end-effector pose (pin.SE3)
            master_pinkine: Master PinKine instance for site positions
            slave_pinkine: Slave PinKine instance for site positions

        Returns:
            Slave robot end-effector pose (pin.SE3)
        """

        m_eeRot = m_eePose.rotation

        m_wristPos = master_pinkine.getSitePosByName("wristSite")
        m_shoulderPos = master_pinkine.getSitePosByName("shoulderSite")
        m_eePos = master_pinkine.getSitePosByName("EE")

        # print(
        #     f"\n Pose Mapping Debug Info: \n"
        #     f"\n  Master wrist position: pos=[{m_wristPos[0]:.4f}, {m_wristPos[1]:.4f}, {m_wristPos[2]:.4f}]"
        #     f"\n  Master shoulder position: pos=[{m_shoulderPos[0]:.4f}, {m_shoulderPos[1]:.4f}, {m_shoulderPos[2]:.4f}]"
        #     f"\n  Master ee position: pos=[{m_eePos[0]:.4f}, {m_eePos[1]:.4f}, {m_eePos[2]:.4f}]"
        # )

        m_shoulderVec = m_wristPos - m_shoulderPos
        s_shoulderVec = self.alpha * m_shoulderVec

        s_shoulderPos = slave_pinkine.getSitePosByName("shoulderSite")
        s_elbowPos = slave_pinkine.getSitePosByName("elbowSite")
        s_wristPos = slave_pinkine.getSitePosByName("wristSite")
        s_eePos = slave_pinkine.getSitePosByName("EE")
        # print(
        #     f"\n  Slave shoulder position (feedback): pos=[{s_shoulderPos[0]:.4f}, {s_shoulderPos[1]:.4f}, {s_shoulderPos[2]:.4f}]"
        #     f"\n  Slave elbow position (feedback): pos=[{s_elbowPos[0]:.4f}, {s_elbowPos[1]:.4f}, {s_elbowPos[2]:.4f}]"
        #     f"\n  Slave wrist position (feedback): pos=[{s_wristPos[0]:.4f}, {s_wristPos[1]:.4f}, {s_wristPos[2]:.4f}]"
        #     f"\n  Slave ee position (feedback): pos=[{s_eePos[0]:.4f}, {s_eePos[1]:.4f}, {s_eePos[2]:.4f}]"
        # )

        s_wristPosRef = s_shoulderPos + s_shoulderVec
        # print(
        #     f"\n  Slave wrist position reference: pos=[{s_wristPosRef[0]:.4f}, {s_wristPosRef[1]:.4f}, {s_wristPosRef[2]:.4f}]"
        # )

        m_eeVec = m_eePos - m_wristPos
        m_eeVec_norm = np.linalg.norm(m_eeVec)
        m_eeVecNorm = m_eeVec / m_eeVec_norm
        # print(
        #     f"\n  Master EE vector (from wrist): vec=[{m_eeVec[0]:.4f}, {m_eeVec[1]:.4f}, {m_eeVec[2]:.4f}], norm={m_eeVec_norm:.4f}"
        # )

        l3_direction_sign = np.sign(np.dot(self.arm2L["l3"], m_eeVecNorm))
        s_eePos = s_wristPosRef + l3_direction_sign * m_eeVecNorm * np.linalg.norm(
            self.arm2L["l3"]
        )
        # print(f"  Slave l3 direction sign: {l3_direction_sign:.0f}")

        slave_pose = pin.SE3(m_eeRot, s_eePos)

        return slave_pose
