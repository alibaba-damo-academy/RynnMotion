"""
Differential IK using QP solver

Solves differential inverse kinematics using quadratic programming to find
joint velocities that track desired end-effector velocities
"""

import numpy as np
from scipy.optimize import minimize


class DiffIKQP:
    """
    Differential IK solver using quadratic programming

    Solves: min ||J*qdot - xdot_des||^2 + regularization
    """

    def __init__(self, nq: int):
        """
        Initialize differential IK QP solver

        Args:
            nq: Number of joint DOF
        """
        self.nq = nq
        self.qp_solved = False
        self.qdCmd = np.zeros(nq)
        self.qdCmdPrev = np.zeros(nq)

    def update(self, jaco: np.ndarray, xdotDes: np.ndarray, dt: float) -> bool:
        """
        Update and solve differential IK QP

        Args:
            jaco: Jacobian matrix (6 x nq)
            xdotDes: Desired end-effector velocity (6,)
            dt: Control timestep

        Returns:
            True if QP solved successfully
        """
        self._solveDiffQP(jaco, xdotDes, dt)
        return self.qp_solved

    def get_qdCmd(self) -> np.ndarray:
        """
        Get commanded joint velocities

        Returns:
            Joint velocities (nq,)
        """
        return self.qdCmd

    def _solveDiffQP(self, jaco: np.ndarray, xdotDes: np.ndarray, dt: float):
        """
        Solve QP: min ||J*qdot - xdot_des||^2 + 0.0009*||qdot||^2

        Args:
            jaco: Jacobian matrix (6 x nq)
            xdotDes: Desired end-effector velocity (6,)
            dt: Control timestep
        """
        hessian = jaco.T @ jaco + 0.0009 * np.eye(self.nq)
        grad = -jaco.T @ xdotDes

        def objective(qdot):
            return 0.5 * qdot.T @ hessian @ qdot + grad.T @ qdot

        def gradient(qdot):
            return hessian @ qdot + grad

        x0 = self.qdCmdPrev if self.qp_solved else np.zeros(self.nq)

        result = minimize(objective, x0, method="SLSQP", jac=gradient, options={"ftol": 1e-6, "disp": False})

        if result.success:
            self.qdCmd = result.x
            self.qdCmdPrev = self.qdCmd
            self.qp_solved = True
        else:
            self.qdCmd = self.qdCmdPrev
            self.qp_solved = False
