"""Inverse kinematics solver using Pinocchio."""

import numpy as np
import pinocchio as pin


class IKSolver:
    def __init__(
        self,
        MJCF_path,
        ee_frame_name,
        pin_q_init=None,
        pos_bias=None,
    ):
        self.pin_model, self.collision_model, self.visual_model = (
            pin.buildModelsFromMJCF(MJCF_path)
        )
        self.pin_data = self.pin_model.createData()
        self.ee_frame_name = ee_frame_name
        self.ee_frame_id = self.pin_model.getFrameId(self.ee_frame_name)

        self.pin_q_init = (
            pin_q_init if pin_q_init is not None else pin.neutral(self.pin_model)
        )
        self.pin_q_last = self.pin_q_init.copy()
        self.pos_bias = pos_bias if pos_bias is not None else np.zeros(3)

        self.q_min = self.pin_model.lowerPositionLimit
        self.q_max = self.pin_model.upperPositionLimit

    def apply_joint_constraints(self, q):
        """Clamps the joint position vector q within the robot's joint limits."""
        return np.clip(q, self.q_min, self.q_max)

    def compute_desired_se3(self, target_pos, target_orientation):
        desired_SE3 = pin.SE3(target_orientation, np.array(target_pos))
        return desired_SE3

    def compute_ik(self, target_pos, target_quat):
        max_iter = 2000
        eps = 5 * 1e-3
        alpha = np.diag([1.0, 1.0, 1.0, 0.2, 0.2, 0.2])
        err_min = 10000

        q = self.pin_q_last.copy()
        q_err_min = q
        for i in range(max_iter):
            q = self.apply_joint_constraints(q)

            pin.forwardKinematics(self.pin_model, self.pin_data, q)
            pin.updateFramePlacements(self.pin_model, self.pin_data)

            current_SE3 = self.pin_data.oMf[self.ee_frame_id]
            current_SE3.translation += current_SE3.rotation @ (-1 * self.pos_bias)

            desired_SE3 = self.compute_desired_se3(target_pos, target_quat)
            error_SE3 = current_SE3.actInv(desired_SE3)
            err = pin.log6(error_SE3).vector

            if np.linalg.norm(err) < eps:
                break

            if np.linalg.norm(err) < err_min:
                err_min = np.linalg.norm(err)
                q_err_min = q.copy()

            J = pin.computeFrameJacobian(
                self.pin_model,
                self.pin_data,
                q,
                self.ee_frame_id,
                pin.LOCAL,
            )

            inverse_jacobian = np.linalg.pinv(J.T @ J + np.eye(J.shape[1])) @ J.T

            v = inverse_jacobian @ alpha @ err

            q = pin.integrate(self.pin_model, q, v)

            if i == max_iter - 1:
                q = q_err_min

        self.pin_q_last = q.copy()
        return q
