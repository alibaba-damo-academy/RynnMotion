"""
Orientation and Coordinate Frame Transform Module

Provides utilities for SE3 rigid body transformations and coordinate frame management.
Uses SciPy + NumPy backend for lightweight, Pinocchio-free operations.

Quaternion convention:
- Internal: (x, y, z, w) - SciPy/Pinocchio convention
- MuJoCo: (w, x, y, z) - use .quat_wxyz property

Example:
    from RynnMotion.algorithms.orient import Transform, FrameTransformer

    # Create transforms
    T1 = Transform.from_pos_quat([1, 0, 0], [0, 0, 0, 1])
    T2 = Transform.from_pos_euler([0, 0, 1], [0, 0, 1.57])

    # Compose transforms
    T3 = T1 @ T2

    # Multi-frame management
    ft = FrameTransformer(base_frame="robot_base")
    ft.set_frame_from_pos_quat("ee", ee_pos, ee_quat)
    ft.set_frame_from_pos_quat("camera_0", cam_pos, cam_quat)
    poses_in_cam0 = ft.transform_all_to_frame("camera_0")
"""

from __future__ import annotations

from typing import Dict, List, Tuple, Union

import numpy as np
from scipy.spatial.transform import Rotation as R


# =============================================================================
# Utility Functions
# =============================================================================


def quat_xyzw_to_wxyz(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion from (x,y,z,w) to (w,x,y,z) for MuJoCo.

    Args:
        quat: Quaternion in (x, y, z, w) format, shape (4,) or (N, 4)

    Returns:
        Quaternion in (w, x, y, z) format
    """
    quat = np.asarray(quat)
    if quat.ndim == 1:
        return np.array([quat[3], quat[0], quat[1], quat[2]])
    else:
        return np.column_stack([quat[:, 3], quat[:, 0], quat[:, 1], quat[:, 2]])


def quat_wxyz_to_xyzw(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion from (w,x,y,z) to (x,y,z,w) for SciPy/Pinocchio.

    Args:
        quat: Quaternion in (w, x, y, z) format, shape (4,) or (N, 4)

    Returns:
        Quaternion in (x, y, z, w) format
    """
    quat = np.asarray(quat)
    if quat.ndim == 1:
        return np.array([quat[1], quat[2], quat[3], quat[0]])
    else:
        return np.column_stack([quat[:, 1], quat[:, 2], quat[:, 3], quat[:, 0]])


def pos_quat_to_matrix(pos: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """Convert position + quaternion to 4x4 homogeneous transformation matrix.

    Args:
        pos: Position vector, shape (3,)
        quat: Quaternion in (x, y, z, w) format, shape (4,)

    Returns:
        4x4 homogeneous transformation matrix
    """
    pos = np.asarray(pos)
    quat = np.asarray(quat)

    rot = R.from_quat(quat)  # scipy uses (x, y, z, w)
    rotmat = rot.as_matrix()

    T = np.eye(4)
    T[:3, :3] = rotmat
    T[:3, 3] = pos
    return T


def matrix_to_pos_quat(matrix: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Convert 4x4 homogeneous matrix to position + quaternion.

    Args:
        matrix: 4x4 homogeneous transformation matrix

    Returns:
        Tuple of (position (3,), quaternion (x,y,z,w) (4,))
    """
    matrix = np.asarray(matrix)
    pos = matrix[:3, 3].copy()
    rotmat = matrix[:3, :3]
    rot = R.from_matrix(rotmat)
    quat = rot.as_quat()  # (x, y, z, w)
    return pos, quat


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Efficiently invert a 4x4 homogeneous transformation matrix.

    For rigid body transforms: T^-1 = [R^T | -R^T @ p]
                                       [0   |    1    ]

    Args:
        T: 4x4 homogeneous transformation matrix

    Returns:
        4x4 inverse transformation matrix
    """
    T = np.asarray(T)
    R_mat = T[:3, :3]
    p = T[:3, 3]

    T_inv = np.eye(4)
    T_inv[:3, :3] = R_mat.T
    T_inv[:3, 3] = -R_mat.T @ p
    return T_inv


def compose_transforms(*transforms: np.ndarray) -> np.ndarray:
    """Compose multiple 4x4 homogeneous transforms.

    Args:
        *transforms: Variable number of 4x4 transformation matrices

    Returns:
        Composed 4x4 transformation matrix: T1 @ T2 @ ... @ Tn
    """
    if len(transforms) == 0:
        return np.eye(4)

    result = np.asarray(transforms[0])
    for T in transforms[1:]:
        result = result @ np.asarray(T)
    return result


# =============================================================================
# Transform Class
# =============================================================================


class Transform:
    """SE3 rigid body transformation using 4x4 homogeneous matrix.

    Represents a rigid body transformation in 3D space, combining rotation
    and translation. Internally stores a 4x4 homogeneous transformation matrix.

    Attributes:
        _matrix: 4x4 homogeneous transformation matrix

    Example:
        # Create from position and quaternion
        T = Transform.from_pos_quat([1, 0, 0], [0, 0, 0, 1])

        # Access properties
        print(T.pos)        # [1, 0, 0]
        print(T.quat)       # [0, 0, 0, 1] (x,y,z,w)
        print(T.quat_wxyz)  # [1, 0, 0, 0] (w,x,y,z) for MuJoCo

        # Compose transforms
        T3 = T1 @ T2

        # Invert
        T_inv = T.inverse()

        # Transform points
        p_transformed = T.apply([1, 2, 3])
    """

    def __init__(self, matrix: np.ndarray = None):
        """Initialize Transform from 4x4 homogeneous matrix.

        Args:
            matrix: 4x4 homogeneous transformation matrix. If None, identity.
        """
        if matrix is None:
            self._matrix = np.eye(4)
        else:
            self._matrix = np.asarray(matrix, dtype=np.float64).copy()

    @classmethod
    def from_pos_quat(cls, pos: np.ndarray, quat: np.ndarray) -> Transform:
        """Create Transform from position and quaternion.

        Args:
            pos: Position vector, shape (3,)
            quat: Quaternion in (x, y, z, w) format, shape (4,)

        Returns:
            Transform instance
        """
        matrix = pos_quat_to_matrix(pos, quat)
        return cls(matrix)

    @classmethod
    def from_pos_quat_wxyz(cls, pos: np.ndarray, quat_wxyz: np.ndarray) -> Transform:
        """Create Transform from position and MuJoCo-style quaternion.

        Args:
            pos: Position vector, shape (3,)
            quat_wxyz: Quaternion in (w, x, y, z) format, shape (4,)

        Returns:
            Transform instance
        """
        quat = quat_wxyz_to_xyzw(quat_wxyz)
        return cls.from_pos_quat(pos, quat)

    @classmethod
    def from_pos_euler(
        cls, pos: np.ndarray, euler: np.ndarray, seq: str = "xyz"
    ) -> Transform:
        """Create Transform from position and Euler angles.

        Args:
            pos: Position vector, shape (3,)
            euler: Euler angles in radians, shape (3,)
            seq: Euler sequence (default: "xyz")

        Returns:
            Transform instance
        """
        pos = np.asarray(pos)
        euler = np.asarray(euler)

        rot = R.from_euler(seq, euler)
        rotmat = rot.as_matrix()

        matrix = np.eye(4)
        matrix[:3, :3] = rotmat
        matrix[:3, 3] = pos
        return cls(matrix)

    @classmethod
    def from_pos_rotmat(cls, pos: np.ndarray, rotmat: np.ndarray) -> Transform:
        """Create Transform from position and 3x3 rotation matrix.

        Args:
            pos: Position vector, shape (3,)
            rotmat: 3x3 rotation matrix

        Returns:
            Transform instance
        """
        pos = np.asarray(pos)
        rotmat = np.asarray(rotmat)

        matrix = np.eye(4)
        matrix[:3, :3] = rotmat
        matrix[:3, 3] = pos
        return cls(matrix)

    @classmethod
    def identity(cls) -> Transform:
        """Create identity transform.

        Returns:
            Identity Transform instance
        """
        return cls(np.eye(4))

    @property
    def matrix(self) -> np.ndarray:
        """Return 4x4 homogeneous transformation matrix."""
        return self._matrix.copy()

    @property
    def pos(self) -> np.ndarray:
        """Return translation vector (3,)."""
        return self._matrix[:3, 3].copy()

    @property
    def quat(self) -> np.ndarray:
        """Return quaternion (x,y,z,w) - SciPy/Pinocchio convention."""
        rot = R.from_matrix(self._matrix[:3, :3])
        return rot.as_quat()

    @property
    def quat_wxyz(self) -> np.ndarray:
        """Return quaternion (w,x,y,z) - MuJoCo convention."""
        return quat_xyzw_to_wxyz(self.quat)

    @property
    def rotmat(self) -> np.ndarray:
        """Return 3x3 rotation matrix."""
        return self._matrix[:3, :3].copy()

    def euler(self, seq: str = "xyz") -> np.ndarray:
        """Return Euler angles in specified sequence.

        Args:
            seq: Euler sequence (default: "xyz")

        Returns:
            Euler angles in radians, shape (3,)
        """
        rot = R.from_matrix(self._matrix[:3, :3])
        return rot.as_euler(seq)

    def inverse(self) -> Transform:
        """Return inverse transform.

        Returns:
            Inverse Transform instance
        """
        return Transform(invert_transform(self._matrix))

    def __matmul__(self, other: Transform) -> Transform:
        """Compose transforms: T_AC = T_AB @ T_BC.

        Args:
            other: Transform to compose with

        Returns:
            Composed Transform instance
        """
        if isinstance(other, Transform):
            return Transform(self._matrix @ other._matrix)
        raise TypeError(f"Cannot compose Transform with {type(other)}")

    def apply(self, point: np.ndarray) -> np.ndarray:
        """Transform a point or array of points.

        Args:
            point: Point (3,) or points (N, 3)

        Returns:
            Transformed point(s) with same shape as input
        """
        point = np.asarray(point)
        if point.ndim == 1:
            # Single point (3,)
            return self._matrix[:3, :3] @ point + self._matrix[:3, 3]
        else:
            # Multiple points (N, 3)
            return (self._matrix[:3, :3] @ point.T).T + self._matrix[:3, 3]

    def __repr__(self) -> str:
        pos = self.pos
        quat = self.quat
        return f"Transform(pos={pos}, quat={quat})"

    def __str__(self) -> str:
        return self.__repr__()


# =============================================================================
# FrameTransformer Class
# =============================================================================


class FrameTransformer:
    """Manages coordinate frame transforms for multi-site robotics.

    Stores poses of multiple frames relative to a base frame and provides
    utilities for transforming between any two frames.

    Example:
        ft = FrameTransformer(base_frame="robot_base")

        # Register frames (poses in base frame)
        ft.set_frame_from_pos_quat("ee_left", ee_pos, ee_quat)
        ft.set_frame_from_pos_quat("camera_0", cam_pos, cam_quat)

        # Get transform between frames
        T_cam0_ee = ft.get_transform(from_frame="ee_left", to_frame="camera_0")

        # Transform all to camera_0 frame
        all_in_cam0 = ft.transform_all_to_frame("camera_0")
    """

    def __init__(self, base_frame: str = "base"):
        """Initialize FrameTransformer.

        Args:
            base_frame: Name of the base/world frame
        """
        self._base_frame = base_frame
        self._frames: Dict[str, Transform] = {}
        # Base frame is identity by definition
        self._frames[base_frame] = Transform.identity()

    @property
    def base_frame(self) -> str:
        """Return name of base frame."""
        return self._base_frame

    @property
    def frame_names(self) -> List[str]:
        """List all registered frame names."""
        return list(self._frames.keys())

    def set_frame(self, name: str, T_base_frame: Transform) -> None:
        """Register a frame's pose in base frame.

        Args:
            name: Frame name
            T_base_frame: Transform from base to this frame
        """
        self._frames[name] = T_base_frame

    def set_frame_from_pos_quat(
        self, name: str, pos: np.ndarray, quat: np.ndarray
    ) -> None:
        """Register a frame from position and quaternion (x,y,z,w).

        Args:
            name: Frame name
            pos: Position in base frame, shape (3,)
            quat: Quaternion in (x, y, z, w) format, shape (4,)
        """
        T = Transform.from_pos_quat(pos, quat)
        self.set_frame(name, T)

    def set_frame_from_pos_quat_wxyz(
        self, name: str, pos: np.ndarray, quat_wxyz: np.ndarray
    ) -> None:
        """Register a frame from position and MuJoCo quaternion (w,x,y,z).

        Args:
            name: Frame name
            pos: Position in base frame, shape (3,)
            quat_wxyz: Quaternion in (w, x, y, z) format, shape (4,)
        """
        T = Transform.from_pos_quat_wxyz(pos, quat_wxyz)
        self.set_frame(name, T)

    def set_frame_from_matrix(self, name: str, matrix: np.ndarray) -> None:
        """Register a frame from 4x4 homogeneous matrix.

        Args:
            name: Frame name
            matrix: 4x4 homogeneous transformation matrix
        """
        self.set_frame(name, Transform(matrix))

    def get_frame(self, name: str) -> Transform:
        """Get transform from base frame to specified frame.

        Args:
            name: Frame name

        Returns:
            Transform from base to frame (T_base_frame)

        Raises:
            KeyError: If frame not found
        """
        if name not in self._frames:
            raise KeyError(f"Frame '{name}' not found. Available: {self.frame_names}")
        return self._frames[name]

    def get_transform(self, from_frame: str, to_frame: str) -> Transform:
        """Get transform from one frame to another.

        Computes T_to_from such that: p_to = T_to_from @ p_from

        Args:
            from_frame: Source frame name
            to_frame: Target frame name

        Returns:
            Transform from source to target frame
        """
        T_base_from = self.get_frame(from_frame)
        T_base_to = self.get_frame(to_frame)

        # T_to_from = T_to_base @ T_base_from = inv(T_base_to) @ T_base_from
        T_to_from = T_base_to.inverse() @ T_base_from
        return T_to_from

    def transform_pose(
        self, pose: Transform, from_frame: str, to_frame: str
    ) -> Transform:
        """Transform a pose from one frame to another.

        Args:
            pose: Pose to transform (expressed in from_frame)
            from_frame: Current frame of the pose
            to_frame: Target frame for the pose

        Returns:
            Pose expressed in to_frame
        """
        T_to_from = self.get_transform(from_frame, to_frame)
        return T_to_from @ pose

    def transform_all_to_frame(self, reference_frame: str) -> Dict[str, Transform]:
        """Transform all registered frames to a reference frame.

        Args:
            reference_frame: Target reference frame

        Returns:
            Dict mapping frame names to their transforms in reference frame
        """
        T_base_ref = self.get_frame(reference_frame)
        T_ref_base = T_base_ref.inverse()

        result = {}
        for name, T_base_frame in self._frames.items():
            # T_ref_frame = T_ref_base @ T_base_frame
            result[name] = T_ref_base @ T_base_frame

        return result

    def clear(self) -> None:
        """Clear all frames except base frame."""
        self._frames = {self._base_frame: Transform.identity()}

    def __repr__(self) -> str:
        return f"FrameTransformer(base='{self._base_frame}', frames={self.frame_names})"
