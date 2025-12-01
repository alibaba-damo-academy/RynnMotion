"""
Algorithms module - Robot control and kinematics algorithms

Contains:
- PoseMapper: Master-slave robot pose mapping for teleoperation
- OSCtrl: Operational Space controller for SE3 pose tracking
- DiffIKQP: Differential IK solver using quadratic programming
- PinKine: Forward kinematics and Jacobian computation
"""

from .pose_mapper import PoseMapper
from .osc import OSCtrl
from .diff_ik import DiffIKQP
from .pin_kine import PinKine

__all__ = [
    'PoseMapper',
    'OSCtrl',
    'DiffIKQP',
    'PinKine',
]
