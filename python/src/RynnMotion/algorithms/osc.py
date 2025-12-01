"""
Operational Space Control module - Workspace control for master-slave teleoperation

Provides:
- OSCtrl: Operational Space controller for SE3 pose tracking

Key sites in xxx_pinocchio.xml:
- EE: End effector site for runtime tracking and control (master ref, slave feedback)
- shoulderSite, elbowSite, wristSite: Key sites for arm configuration pose mapping
"""

import numpy as np
import pinocchio as pin


class OSCtrl:
    """
    SE3 workspace controller for end-effector pose tracking

    Computes 6D error (position + orientation) and converts to velocity command
    """

    def __init__(self, dt: float = 0.01):
        """
        Initialize workspace controller

        Args:
            dt: Control timestep for error to velocity conversion
        """
        self.dt = dt
        self.eeVel = np.zeros(6)

    def update(self, poseFb: pin.SE3, poseRef: pin.SE3):
        """
        Update controller with current feedback and reference poses

        Args:
            poseFb: Current end-effector pose feedback (SE3)
            poseRef: Desired end-effector pose reference (SE3)
        """
        posError = poseRef.translation - poseFb.translation

        quat_fb = pin.Quaternion(poseFb.rotation)
        quat_ref = pin.Quaternion(poseRef.rotation)
        quat_err = quat_ref * quat_fb.inverse()

        angle_axis = pin.AngleAxis(quat_err)
        orientError = angle_axis.angle * angle_axis.axis

        error = np.zeros(6)
        error[:3] = posError
        error[3:] = orientError

        if self.dt <= 1e-10:
            self.eeVel = np.zeros(6)
        else:
            self.eeVel = error / self.dt

    def get_eeVel(self) -> np.ndarray:
        """
        Get computed end-effector velocity command

        Returns:
            6D velocity [linear_vel (3), angular_vel (3)]
        """
        return self.eeVel
