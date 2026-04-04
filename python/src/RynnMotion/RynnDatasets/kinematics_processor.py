"""
Kinematics processing for dataset recording.

Computes derived kinematic features (EE poses, god-view transforms)
from raw joint readings using Pinocchio FK.
"""

from abc import ABC, abstractmethod

import numpy as np

from RynnMotion.RynnDatasets.sources import SensorReading

_IDENTITY_POSE = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)

_POSE_FEATURE = {
    "dtype": "float32",
    "shape": (7,),
    "names": ["x", "y", "z", "qx", "qy", "qz", "qw"],
}


class IKinematicsProcessor(ABC):
    """Abstract base for kinematics processing."""

    @abstractmethod
    def process(self, frame: dict[str, SensorReading]) -> dict[str, np.ndarray]:
        """Compute derived kinematic features from sensor readings.

        Args:
            frame: Dict mapping source name to SensorReading.
                Must contain joint position data.

        Returns:
            Dict mapping feature keys to computed numpy arrays.
            E.g. {"observation.end_effector_pose": np.array([x,y,z,qx,qy,qz,qw])}
        """
        ...

    @abstractmethod
    def get_features(self) -> dict[str, dict]:
        """Return dataset feature descriptors for computed features."""
        ...


class PinocchioKinematicsProcessor(IKinematicsProcessor):
    """Computes FK-derived features using Pinocchio.

    Uses PinKine for forward kinematics and FrameTransformer for
    god-view (camera-relative) pose computation.

    Output feature keys:
        - ``observation.end_effector_pose`` (single EE) or
          ``observation.{ee_name}_pose`` (multiple EEs)
        - ``observation.camera_poses.{display_name}`` for each camera site
        - ``action.end_effector_pose`` / ``action.camera_poses.{display_name}``
          when ``action_joint_source_key`` is set

    Args:
        mjcf_path: Path to MJCF model file.
        site_names: List of Pinocchio site names to track (EE + cameras).
        joint_source_key: The data key in SensorReading that contains
            observation joint positions (e.g. "observation.state").
        action_joint_source_key: If set, also run FK on this key's joint
            positions and output action-prefix pose features.
        godview: If True, transform poses to reference camera frame.
        reference_camera: Site name of the reference camera for god-view.
            The reference camera will have identity pose in god-view mode.
    """

    def __init__(
        self,
        mjcf_path: str,
        site_names: list[str],
        joint_source_key: str = "observation.state",
        action_joint_source_key: str | None = None,
        godview: bool = True,
        reference_camera: str = "camera_front",
    ):
        from RynnMotion.algorithms.pin_kine import PinKine
        from RynnMotion.algorithms.orient import Transform, FrameTransformer

        self.mjcf_path = mjcf_path
        self.site_names = site_names
        self.joint_source_key = joint_source_key
        self.action_joint_source_key = action_joint_source_key
        self.godview = godview
        self.reference_camera = reference_camera

        self._Transform = Transform
        self._FrameTransformer = FrameTransformer

        self.kine = PinKine(mjcf_path, site_names=site_names)

        # Identify EE sites and camera sites
        self._ee_site_names = [
            name for name in site_names
            if name.startswith("EE") or name.startswith("ee") or "_EE" in name or "_ee" in name
        ]
        self._camera_site_names = [
            name for name in site_names
            if name.startswith("camera") or name.startswith("cam")
        ]

    def process(self, frame: dict[str, SensorReading]) -> dict[str, np.ndarray]:
        result = {}

        # Observation FK
        q_obs = self._extract_joint_positions(frame, self.joint_source_key)
        if q_obs is not None:
            self.kine.update(q_obs)
            if self.godview and self.reference_camera in self.site_names:
                result.update(self._compute_godview_poses("observation"))
            else:
                result.update(self._compute_world_poses("observation"))

        # Action FK (optional)
        if self.action_joint_source_key is not None:
            q_action = self._extract_joint_positions(frame, self.action_joint_source_key)
            if q_action is not None:
                self.kine.update(q_action)
                if self.godview and self.reference_camera in self.site_names:
                    result.update(self._compute_godview_poses("action"))
                else:
                    result.update(self._compute_world_poses("action"))

        return result

    def get_features(self) -> dict[str, dict]:
        features = {}
        prefixes = ["observation"]
        if self.action_joint_source_key is not None:
            prefixes.append("action")

        for prefix in prefixes:
            for key in self._ee_feature_keys(prefix):
                features[key] = dict(_POSE_FEATURE)
            for key in self._camera_feature_keys(prefix):
                features[key] = dict(_POSE_FEATURE)

        return features

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _ee_feature_keys(self, prefix: str) -> list[str]:
        """Return EE feature key(s) for the given prefix."""
        if len(self._ee_site_names) == 1:
            return [f"{prefix}.end_effector_pose"]
        return [f"{prefix}.{name}_pose" for name in self._ee_site_names]

    def _camera_feature_keys(self, prefix: str) -> list[str]:
        """Return camera pose feature keys for the given prefix."""
        return [
            f"{prefix}.camera_poses.{self._camera_display_name(name)}"
            for name in self._camera_site_names
        ]

    @staticmethod
    def _camera_display_name(site_name: str) -> str:
        """Strip 'camera_' or 'cam_' prefix for the feature key display name.

        Examples:
            "camera_front" -> "front"
            "cam_wrist"    -> "wrist"
            "d435_main"    -> "d435_main"
        """
        for prefix in ("camera_", "cam_"):
            if site_name.startswith(prefix):
                return site_name[len(prefix):]
        return site_name

    def _extract_joint_positions(
        self, frame: dict[str, SensorReading], source_key: str
    ) -> np.ndarray | None:
        """Find joint position array matching source_key from frame readings.

        Automatically slices to model DOF if the array is longer
        (e.g. action array includes gripper joints).
        """
        nq = self.kine.pin_model.nq
        for reading in frame.values():
            for key, val in reading.data.items():
                if key == source_key and isinstance(val, np.ndarray):
                    return val[:nq] if len(val) > nq else val
        return None

    def _compute_world_poses(self, prefix: str) -> dict[str, np.ndarray]:
        """Compute EE and camera poses in world frame."""
        result = {}
        ee_keys = self._ee_feature_keys(prefix)

        for i, ee_name in enumerate(self._ee_site_names):
            idx = self.kine.getSiteIndex(ee_name)
            pos = self.kine.getSitePos(idx)
            quat = self.kine.getSiteQuat(idx)  # (x, y, z, w)
            result[ee_keys[i]] = np.concatenate([pos, quat]).astype(np.float32)

        for cam_name in self._camera_site_names:
            display = self._camera_display_name(cam_name)
            idx = self.kine.getSiteIndex(cam_name)
            pos = self.kine.getSitePos(idx)
            quat = self.kine.getSiteQuat(idx)
            result[f"{prefix}.camera_poses.{display}"] = np.concatenate([pos, quat]).astype(np.float32)

        return result

    def _compute_godview_poses(self, prefix: str) -> dict[str, np.ndarray]:
        """Compute EE and camera poses relative to reference camera frame (god-view).

        The reference camera has identity pose. All other sites are expressed
        in the reference camera's coordinate frame.
        """
        ft = self._FrameTransformer(base_frame="world")

        # Register all sites in world frame
        for site_name in self.site_names:
            idx = self.kine.getSiteIndex(site_name)
            pos = self.kine.getSitePos(idx)
            quat = self.kine.getSiteQuat(idx)
            T = self._Transform.from_pos_quat(pos, quat)
            ft.set_frame(site_name, T)

        result = {}
        ee_keys = self._ee_feature_keys(prefix)

        # EE poses in reference camera frame
        for i, ee_name in enumerate(self._ee_site_names):
            T_cam_ee = ft.get_transform(from_frame=ee_name, to_frame=self.reference_camera)
            result[ee_keys[i]] = np.concatenate([
                T_cam_ee.pos.astype(np.float32),
                T_cam_ee.quat.astype(np.float32),
            ])

        # Camera poses: reference = identity, others relative to reference
        for cam_name in self._camera_site_names:
            display = self._camera_display_name(cam_name)
            if cam_name == self.reference_camera:
                result[f"{prefix}.camera_poses.{display}"] = _IDENTITY_POSE.copy()
            else:
                T_ref_cam = ft.get_transform(from_frame=cam_name, to_frame=self.reference_camera)
                result[f"{prefix}.camera_poses.{display}"] = np.concatenate([
                    T_ref_cam.pos.astype(np.float32),
                    T_ref_cam.quat.astype(np.float32),
                ])

        return result
