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

# Conditional mink imports - only available when mink is installed
_MINK_AVAILABLE = False
try:
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
    _MINK_AVAILABLE = True
except ImportError:
    # mink not installed - these functions won't be available
    pass

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
]

# Add mink exports only when mink is available
if _MINK_AVAILABLE:
    __all__.extend([
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
    ])
