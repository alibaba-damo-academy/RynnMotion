"""
RobotContext: Bridge between MJCF parsing and dataset feature generation.

Replaces hard-coded feature lists (get_robot_features()) with auto-detected
features from MJCF model files. Mirrors the C++ MjcfParser pipeline.

Example:
    # From MJCF files directly
    ctx = RobotContext.from_mjcf("fr3_robot.xml", "fr3_pinocchio.xml")
    features = ctx.get_hw_features(camera_configs={"front": (480, 640, 3)})

    # From existing RobotManager
    ctx = RobotContext.from_robot_manager(robot_manager)
    features = ctx.get_hw_features()
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

from RynnMotion.manager.mjcf_parser import MjcfParser, SensorInfo


@dataclass
class RobotContext:
    """Auto-detected robot context for dataset feature generation.

    Attributes:
        robot_name: Human-readable robot identifier.
        mdof: Motion DOF (actuated joints excluding grippers).
        adof: Action DOF (gripper actuators).
        num_ee: Number of end-effectors.
        joint_names: Ordered list of joint actuator names.
        ee_names: Ordered list of EE actuator names.
        camera_names: Camera names from MJCF.
        sensors: Dict mapping sensor category to list of SensorInfo.
        ee_site_names: Site names for end-effectors.
    """

    robot_name: str = ""
    mdof: int = 0
    adof: int = 0
    num_ee: int = 0
    joint_names: List[str] = field(default_factory=list)
    ee_names: List[str] = field(default_factory=list)
    camera_names: List[str] = field(default_factory=list)
    sensors: Dict[str, List[SensorInfo]] = field(default_factory=dict)
    ee_site_names: List[str] = field(default_factory=list)
    is_dex_hand: bool = False

    @classmethod
    def from_mjcf(
        cls,
        robot_mjcf: str,
        pino_mjcf: str | None = None,
        robot_name: str = "",
    ) -> RobotContext:
        """Create RobotContext by parsing MJCF files.

        Args:
            robot_mjcf: Path to robot MJCF file (full model with actuators).
            pino_mjcf: Path to Pinocchio MJCF file (optional, for future use).
            robot_name: Robot identifier string.
        """
        state = MjcfParser.extract_robot_state_dim(robot_mjcf)
        actuator_names = MjcfParser.extract_actuator_names(robot_mjcf)
        sensors = MjcfParser.detect_sensors(robot_mjcf)
        cameras = MjcfParser.detect_cameras(robot_mjcf)

        # Check if standalone dex hand (all actuators are gripper, none are joints)
        mdof = state["mdof"]
        adof = state["adof"]
        is_dex_hand = False
        if mdof == 0 and len(state["ee_indices"]) > 5:
            # Standalone dex hand: reclassify EE as joints
            is_dex_hand = True
            mdof = len(state["ee_indices"])
            adof = 0
            joint_names = actuator_names["ee_names"]
            ee_names = []
        else:
            joint_names = actuator_names["joint_names"]
            ee_names = actuator_names["ee_names"]

        return cls(
            robot_name=robot_name,
            mdof=mdof,
            adof=adof,
            num_ee=state["num_ee"],
            joint_names=joint_names,
            ee_names=ee_names,
            camera_names=cameras,
            sensors=sensors,
            ee_site_names=state.get("ee_site_name", []),
            is_dex_hand=is_dex_hand,
        )

    @classmethod
    def from_robot_manager(cls, rm) -> RobotContext:
        """Create RobotContext from an existing RobotManager instance.

        Args:
            rm: RobotManager instance (Python or duck-typed equivalent).
        """
        return cls(
            robot_name=rm.get_robot_name(),
            mdof=rm.get_motion_dof(),
            adof=rm.get_action_dof(),
            num_ee=rm.get_num_ee(),
            joint_names=rm.get_joint_names(),
            ee_names=rm.get_ee_names(),
            camera_names=rm.get_camera_names(),
            sensors=rm.get_sensors(),
            ee_site_names=rm.get_ee_site_name(),
        )

    def get_hw_features(
        self,
        camera_configs: Dict[str, Tuple[int, int, int]] | None = None,
        use_videos: bool = True,
    ) -> Dict[str, Dict[str, Any]]:
        """Generate dataset feature dict from auto-detected robot context.

        Drop-in replacement for hard-coded get_robot_features().
        Produces features compatible with RynnDataset and LeRobot format.

        Args:
            camera_configs: Optional dict mapping camera name to (H, W, C) resolution.
                If None, cameras are detected but not included in features
                (they need resolution info from the actual hardware).
            use_videos: Whether camera features use "video" or "image" dtype.

        Returns:
            Feature dict mapping keys like "observation.state", "action",
            "observation.images.front" to feature descriptors with
            dtype, shape, and names.
        """
        features: Dict[str, Dict[str, Any]] = {}

        # observation.state: joint positions (mdof)
        if self.joint_names:
            features["observation.state"] = {
                "dtype": "float32",
                "shape": (self.mdof,),
                "names": list(self.joint_names),
            }

        # action: joint positions + gripper commands (mdof + adof)
        action_names = list(self.joint_names) + list(self.ee_names)
        if action_names:
            features["action"] = {
                "dtype": "float32",
                "shape": (len(action_names),),
                "names": action_names,
            }

        # observation.state.gripper: gripper state (adof)
        if self.ee_names:
            features["observation.state.gripper"] = {
                "dtype": "float32",
                "shape": (len(self.ee_names),),
                "names": list(self.ee_names),
            }

        # Cameras: observation.images.<camera_name>
        effective_configs = camera_configs or {}
        if not effective_configs and self.camera_names:
            effective_configs = {cam: (480, 640, 3) for cam in self.camera_names}
        for cam_name, resolution in effective_configs.items():
            features[f"observation.images.{cam_name}"] = {
                "dtype": "video" if use_videos else "image",
                "shape": resolution,
                "names": ["height", "width", "channels"],
            }

        # Force/torque sensors
        for sensor_info in self.sensors.get("force", []):
            features[f"observation.sensor.force.{sensor_info.name}"] = {
                "dtype": "float32",
                "shape": (sensor_info.dim,),
                "names": [f"f{i}" for i in range(sensor_info.dim)],
            }

        for sensor_info in self.sensors.get("torque", []):
            features[f"observation.sensor.torque.{sensor_info.name}"] = {
                "dtype": "float32",
                "shape": (sensor_info.dim,),
                "names": [f"t{i}" for i in range(sensor_info.dim)],
            }

        # IMU sensors (gyro + accel)
        for sensor_info in self.sensors.get("gyro", []):
            features[f"observation.sensor.gyro.{sensor_info.name}"] = {
                "dtype": "float32",
                "shape": (sensor_info.dim,),
                "names": ["wx", "wy", "wz"][:sensor_info.dim],
            }

        for sensor_info in self.sensors.get("accel", []):
            features[f"observation.sensor.accel.{sensor_info.name}"] = {
                "dtype": "float32",
                "shape": (sensor_info.dim,),
                "names": ["ax", "ay", "az"][:sensor_info.dim],
            }

        # Rangefinder sensors
        for sensor_info in self.sensors.get("rangefinder", []):
            features[f"observation.sensor.rangefinder.{sensor_info.name}"] = {
                "dtype": "float32",
                "shape": (sensor_info.dim,),
                "names": ["distance"],
            }

        # Touch sensors
        for sensor_info in self.sensors.get("touch", []):
            features[f"observation.sensor.touch.{sensor_info.name}"] = {
                "dtype": "float32",
                "shape": (sensor_info.dim,),
                "names": ["force"],
            }

        return features

    def _get_ee_pose_features(
        self, prefix: str
    ) -> Dict[str, Dict[str, Any]]:
        """Generate 7D EE pose features for observation or action prefix."""
        features: Dict[str, Dict[str, Any]] = {}
        if self.num_ee <= 0:
            return features

        pose_names = ["x", "y", "z", "qx", "qy", "qz", "qw"]

        if self.num_ee == 1:
            features[f"{prefix}.end_effector_pose"] = {
                "dtype": "float32",
                "shape": (7,),
                "names": pose_names,
            }
        else:
            for i in range(self.num_ee):
                ee_name = self.ee_names[i] if i < len(self.ee_names) else f"ee{i}"
                features[f"{prefix}.{ee_name}_pose"] = {
                    "dtype": "float32",
                    "shape": (7,),
                    "names": pose_names,
                }

        return features

    def get_recording_features(
        self,
        camera_configs: Dict[str, Tuple[int, int, int]] | None = None,
        use_videos: bool = True,
    ) -> Dict[str, Dict[str, Any]]:
        """Generate full recording feature dict including velocity, torque, EE poses, metadata.

        Produces features identical to C++ RobotContext::getRecordingFeatures().

        Args:
            camera_configs: Optional dict mapping camera name to (H, W, C) resolution.
            use_videos: Whether camera features use "video" or "image" dtype.

        Returns:
            Feature dict with all recording columns.
        """
        features: Dict[str, Dict[str, Any]] = {}

        # Joint state features
        features["observation.state"] = {
            "dtype": "float32",
            "shape": (self.mdof,),
            "names": list(self.joint_names),
        }
        features["observation.velocity"] = {
            "dtype": "float32",
            "shape": (self.mdof,),
            "names": list(self.joint_names),
        }
        features["observation.torque"] = {
            "dtype": "float32",
            "shape": (self.mdof,),
            "names": list(self.joint_names),
        }

        # Action: joints + grippers concatenated
        action_names = list(self.joint_names) + list(self.ee_names)
        features["action"] = {
            "dtype": "float32",
            "shape": (len(action_names),),
            "names": action_names,
        }
        features["action.velocity"] = {
            "dtype": "float32",
            "shape": (self.mdof,),
            "names": list(self.joint_names),
        }
        features["action.torque"] = {
            "dtype": "float32",
            "shape": (self.mdof,),
            "names": list(self.joint_names),
        }

        # Gripper
        if self.ee_names:
            features["observation.state.gripper"] = {
                "dtype": "float32",
                "shape": (len(self.ee_names),),
                "names": list(self.ee_names),
            }

        # EE poses (observation + action)
        features.update(self._get_ee_pose_features("observation"))
        features.update(self._get_ee_pose_features("action"))

        # Cameras
        effective_configs = camera_configs or {}
        if not effective_configs and self.camera_names:
            effective_configs = {cam: (480, 640, 3) for cam in self.camera_names}
        for cam_name, resolution in effective_configs.items():
            features[f"observation.images.{cam_name}"] = {
                "dtype": "video" if use_videos else "image",
                "shape": resolution,
                "names": ["height", "width", "channels"],
            }

        # Camera poses
        pose_names = ["x", "y", "z", "qx", "qy", "qz", "qw"]
        for cam_name in effective_configs:
            features[f"observation.camera_poses.{cam_name}"] = {
                "dtype": "float32",
                "shape": (7,),
                "names": pose_names,
            }
            features[f"action.camera_poses.{cam_name}"] = {
                "dtype": "float32",
                "shape": (7,),
                "names": pose_names,
            }

        # Sensors
        for sensor_info in self.sensors.get("force", []):
            features[f"observation.sensor.force.{sensor_info.name}"] = {
                "dtype": "float32",
                "shape": (sensor_info.dim,),
                "names": [f"f{i}" for i in range(sensor_info.dim)],
            }
        for sensor_info in self.sensors.get("torque", []):
            features[f"observation.sensor.torque.{sensor_info.name}"] = {
                "dtype": "float32",
                "shape": (sensor_info.dim,),
                "names": [f"t{i}" for i in range(sensor_info.dim)],
            }
        for sensor_info in self.sensors.get("gyro", []):
            features[f"observation.sensor.gyro.{sensor_info.name}"] = {
                "dtype": "float32",
                "shape": (sensor_info.dim,),
                "names": ["wx", "wy", "wz"][: sensor_info.dim],
            }
        for sensor_info in self.sensors.get("accel", []):
            features[f"observation.sensor.accel.{sensor_info.name}"] = {
                "dtype": "float32",
                "shape": (sensor_info.dim,),
                "names": ["ax", "ay", "az"][: sensor_info.dim],
            }

        # Rangefinder sensors
        for sensor_info in self.sensors.get("rangefinder", []):
            features[f"observation.sensor.rangefinder.{sensor_info.name}"] = {
                "dtype": "float32",
                "shape": (sensor_info.dim,),
                "names": ["distance"],
            }

        # Touch sensors
        for sensor_info in self.sensors.get("touch", []):
            features[f"observation.sensor.touch.{sensor_info.name}"] = {
                "dtype": "float32",
                "shape": (sensor_info.dim,),
                "names": ["force"],
            }

        # Metadata
        features["timestamp"] = {"dtype": "float64", "shape": (1,), "names": []}
        features["frame_index"] = {"dtype": "int64", "shape": (1,), "names": []}
        features["episode_index"] = {"dtype": "int64", "shape": (1,), "names": []}
        features["index"] = {"dtype": "int64", "shape": (1,), "names": []}
        features["task_index"] = {"dtype": "int64", "shape": (1,), "names": []}

        return features

    def to_info_context(self) -> Dict[str, Any]:
        """Export metadata dict suitable for dataset info.json.

        Returns:
            Dict with robot context metadata including DOF counts,
            joint/EE names, camera names, and sensor summary.
        """
        sensor_summary = {}
        for category, info_list in self.sensors.items():
            if info_list:
                sensor_summary[category] = [
                    {"name": s.name, "dim": s.dim} for s in info_list
                ]

        return {
            "robot_name": self.robot_name,
            "mdof": self.mdof,
            "adof": self.adof,
            "num_ee": self.num_ee,
            "joint_names": list(self.joint_names),
            "ee_names": list(self.ee_names),
            "camera_names": list(self.camera_names),
            "ee_site_names": list(self.ee_site_names),
            "sensors": sensor_summary,
        }

    @property
    def total_action_dim(self) -> int:
        """Total action dimension (joints + grippers)."""
        return self.mdof + self.adof

    @property
    def has_sensors(self) -> bool:
        """Whether any sensors were detected."""
        return any(len(v) > 0 for v in self.sensors.values())

    @property
    def has_cameras(self) -> bool:
        """Whether any cameras were detected."""
        return len(self.camera_names) > 0
