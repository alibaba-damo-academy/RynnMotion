"""
Algorithms module - Robot control and kinematics algorithms

Contains:
- PoseMapper: Master-slave robot pose mapping for teleoperation
- OSCtrl: Operational Space controller for SE3 pose tracking
- DiffIKQP: Differential IK solver using quadratic programming
- PinKine: Forward kinematics and Jacobian computation
- Transform: SE3 rigid body transformation
- FrameTransformer: Multi-frame coordinate transform manager
- PolicyInterpolator: Trajectory interpolation for policy execution
- mink_bridge: Bridge utilities for mink IK integration
"""

from .pose_mapper import PoseMapper
from .osc import OSCtrl
from .diff_ik import DiffIKQP
from .pin_kine import PinKine
from .orient import (
    Transform,
    FrameTransformer,
    pos_quat_to_matrix,
    matrix_to_pos_quat,
    quat_xyzw_to_wxyz,
    quat_wxyz_to_xyzw,
    invert_transform,
    compose_transforms,
)
from .policy_interpolator import PolicyInterpolator, lerp
from .mink_bridge import (
    configuration_from_model,
    configuration_from_interface,
    sync_interface_to_config,
    sync_mj_data_to_config,
    apply_config_to_interface,
    apply_config_to_mj_data,
    get_joint_positions_from_config,
    create_frame_task,
    create_posture_task,
    create_standard_limits,
    solve_ik_step,
)

__all__ = [
    'PoseMapper',
    'OSCtrl',
    'DiffIKQP',
    'PinKine',
    'Transform',
    'FrameTransformer',
    'pos_quat_to_matrix',
    'matrix_to_pos_quat',
    'quat_xyzw_to_wxyz',
    'quat_wxyz_to_xyzw',
    'invert_transform',
    'compose_transforms',
    'PolicyInterpolator',
    'lerp',
    # mink bridge utilities
    'configuration_from_model',
    'configuration_from_interface',
    'sync_interface_to_config',
    'sync_mj_data_to_config',
    'apply_config_to_interface',
    'apply_config_to_mj_data',
    'get_joint_positions_from_config',
    'create_frame_task',
    'create_posture_task',
    'create_standard_limits',
    'solve_ik_step',
]
