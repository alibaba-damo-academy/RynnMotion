"""Sensor source registry - creates ISensorSource instances from YAML config."""

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from . import ISensorSource, SensorType

logger = logging.getLogger(__name__)


@dataclass
class SourceConfig:
    """Configuration for a single sensor source."""
    name: str
    source: str  # "direct", "mujoco", "ros2"
    sensor_type: str  # matches SensorType enum values
    params: dict[str, Any]


@dataclass
class PipelineConfig:
    """Top-level pipeline configuration."""
    robot_type: str | None
    fps: int
    sources: list[SourceConfig]
    kinematics: dict[str, Any] | None
    alignment_mode: str  # "sync" or "async"
    repo_id: str | None
    use_videos: bool


class SensorSourceRegistry:
    """Creates ISensorSource instances from YAML configuration.

    YAML format:
    ```yaml
    robot_type: so101
    fps: 30
    alignment_mode: sync
    repo_id: user/dataset_name
    use_videos: true

    sources:
      - name: arm_joints
        source: direct
        sensor_type: joint
        params:
          joint_names: [shoulder_pan, shoulder_lift, elbow, ...]
          prefix: observation
          suffix: arm

      - name: camera_front
        source: direct
        sensor_type: camera
        params:
          camera_index: 0
          resolution: [480, 640, 3]
          fps: 30

      - name: joint_states
        source: ros2
        sensor_type: joint
        params:
          topic: /joint_states
          msg_type: sensor_msgs.msg.JointState
          adapter: GenericAdapter
          mappings:
            - field: position
              out_key: observation.state

    kinematics:
      mjcf_path: models/3.robot_arm/so101/so101.xml
      site_names: [EE_left, EE_right]
      godview: true
      reference_camera: camera_front
    ```
    """

    @staticmethod
    def from_yaml(path: str | Path) -> PipelineConfig:
        """Load pipeline configuration from YAML file."""
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f)

        if data is None:
            raise ValueError(f"Empty YAML configuration at {path}")

        sources = []
        for src in data.get("sources", []):
            sources.append(SourceConfig(
                name=src["name"],
                source=src.get("source", "direct"),
                sensor_type=src["sensor_type"],
                params=src.get("params", {}),
            ))

        kinematics = data.get("kinematics")

        return PipelineConfig(
            robot_type=data.get("robot_type"),
            fps=data.get("fps", 30),
            sources=sources,
            kinematics=kinematics,
            alignment_mode=data.get("alignment_mode", "sync"),
            repo_id=data.get("repo_id"),
            use_videos=data.get("use_videos", True),
        )

    @staticmethod
    def create_sources(config: PipelineConfig, **context) -> list[ISensorSource]:
        """Create ISensorSource instances from pipeline config.

        Args:
            config: Pipeline configuration.
            **context: Additional context for source creation, e.g.:
                - mj_model, mj_data: For MuJoCo sources.
                - reader_fns: Dict mapping source name to callable for direct joint sources.
                - camera_objects: Dict mapping source name to camera object/index.

        Returns:
            List of configured ISensorSource instances.
        """
        sources = []
        for src_cfg in config.sources:
            source = SensorSourceRegistry._create_source(src_cfg, **context)
            if source is not None:
                sources.append(source)
        return sources

    @staticmethod
    def _create_source(cfg: SourceConfig, **context) -> ISensorSource | None:
        sensor_type = SensorType(cfg.sensor_type)

        if cfg.source == "direct":
            return SensorSourceRegistry._create_direct_source(cfg, sensor_type, **context)
        elif cfg.source == "mujoco":
            return SensorSourceRegistry._create_mujoco_source(cfg, sensor_type, **context)
        elif cfg.source == "ros2":
            return SensorSourceRegistry._create_ros2_source(cfg, sensor_type)
        else:
            logger.warning(f"Unknown source type '{cfg.source}' for '{cfg.name}'")
            return None

    @staticmethod
    def _create_direct_source(
        cfg: SourceConfig, sensor_type: SensorType, **context
    ) -> ISensorSource | None:
        params = cfg.params

        if sensor_type == SensorType.JOINT:
            from .joint_source import DirectJointSensorSource

            reader_fns = context.get("reader_fns", {})
            reader_fn = reader_fns.get(cfg.name)
            if reader_fn is None:
                logger.warning(f"No reader_fn provided for direct joint source '{cfg.name}'")
                return None
            return DirectJointSensorSource(
                name=cfg.name,
                joint_names=params["joint_names"],
                reader_fn=reader_fn,
                prefix=params.get("prefix", "observation"),
                suffix=params.get("suffix"),
            )

        elif sensor_type == SensorType.CAMERA:
            from .camera_source import DirectCameraSensorSource

            camera_objects = context.get("camera_objects", {})
            camera = camera_objects.get(cfg.name, params.get("camera_index", 0))
            resolution = tuple(params.get("resolution", (480, 640, 3)))
            return DirectCameraSensorSource(
                name=cfg.name,
                camera=camera,
                resolution=resolution,
                fps=params.get("fps", 30),
                prefix=params.get("prefix", "observation"),
                use_video=params.get("use_video", True),
            )

        elif sensor_type == SensorType.WRENCH:
            from .wrench_source import DirectWrenchSensorSource

            reader_fns = context.get("reader_fns", {})
            reader_fn = reader_fns.get(cfg.name)
            if reader_fn is None:
                return None
            return DirectWrenchSensorSource(
                name=cfg.name,
                reader_fn=reader_fn,
                prefix=params.get("prefix", "observation"),
                suffix=params.get("suffix"),
            )

        elif sensor_type == SensorType.IMU:
            from .imu_source import DirectIMUSensorSource

            reader_fns = context.get("reader_fns", {})
            reader_fn = reader_fns.get(cfg.name)
            if reader_fn is None:
                return None
            return DirectIMUSensorSource(
                name=cfg.name,
                reader_fn=reader_fn,
                num_dims=params.get("num_dims", 10),
                prefix=params.get("prefix", "observation"),
                suffix=params.get("suffix"),
            )

        elif sensor_type == SensorType.DEPTH:
            from .depth_source import DirectDepthSensorSource

            camera_objects = context.get("camera_objects", {})
            camera = camera_objects.get(cfg.name, params.get("camera_index", 0))
            resolution = tuple(params.get("resolution", (480, 640, 1)))
            return DirectDepthSensorSource(
                name=cfg.name,
                camera=camera,
                resolution=resolution,
                prefix=params.get("prefix", "observation"),
                use_video=params.get("use_video", True),
            )

        elif sensor_type == SensorType.TACTILE:
            from .tactile_source import DirectTactileSensorSource

            reader_fns = context.get("reader_fns", {})
            reader_fn = reader_fns.get(cfg.name)
            if reader_fn is None:
                return None
            return DirectTactileSensorSource(
                name=cfg.name,
                reader_fn=reader_fn,
                num_dims=params.get("num_dims", 16),
                prefix=params.get("prefix", "observation"),
                suffix=params.get("suffix"),
            )

        logger.warning(f"Unsupported direct sensor type '{sensor_type}' for '{cfg.name}'")
        return None

    @staticmethod
    def _create_mujoco_source(
        cfg: SourceConfig, sensor_type: SensorType, **context
    ) -> ISensorSource | None:
        from .mujoco_source import MuJoCoSensorSource

        mj_model = context.get("mj_model")
        mj_data = context.get("mj_data")
        if mj_model is None or mj_data is None:
            logger.warning(f"mj_model/mj_data not provided for MuJoCo source '{cfg.name}'")
            return None

        params = cfg.params
        return MuJoCoSensorSource(
            name=cfg.name,
            mj_model=mj_model,
            mj_data=mj_data,
            sensor_names=params["sensor_names"],
            sensor_type=sensor_type,
            read_mode=params.get("read_mode", "joint"),
            prefix=params.get("prefix", "observation"),
            suffix=params.get("suffix"),
        )

    @staticmethod
    def _create_ros2_source(cfg: SourceConfig, sensor_type: SensorType) -> ISensorSource | None:
        from .ros2_source import Ros2SensorSource

        params = cfg.params
        mappings_raw = params.get("mappings", [])

        return Ros2SensorSource(
            name=cfg.name,
            topic=params["topic"],
            msg_type_str=params["msg_type"],
            adapter_name=params.get("adapter", "GenericAdapter"),
            mappings_raw=mappings_raw,
            sensor_type=sensor_type,
            prefix=params.get("prefix", "observation"),
        )
