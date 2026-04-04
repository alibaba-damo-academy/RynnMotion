"""
Unified recording pipeline orchestrator.

RynnRecorder ties together sensor sources, temporal alignment,
kinematics processing, and RynnDataset storage into a single API.
"""

import logging
import time
from pathlib import Path
from typing import Any

from RynnMotion.RynnDatasets.sources import ISensorSource, SensorReading
from RynnMotion.RynnDatasets.sources.registry import SensorSourceRegistry
from RynnMotion.RynnDatasets.alignment import TemporalAligner
from RynnMotion.RynnDatasets.kinematics_processor import IKinematicsProcessor, PinocchioKinematicsProcessor

logger = logging.getLogger(__name__)


class RynnRecorder:
    """Unified recording pipeline orchestrating sources -> alignment -> kinematics -> storage.

    Can be initialized from:
    1. YAML config file (new unified format)
    2. Programmatic arguments (backward compatible for satellite scripts)

    Example (YAML):
        recorder = RynnRecorder(config_path="pipeline.yaml")
        recorder.setup()
        recorder.record_dataset(task="pick and place", num_episodes=10, episode_time_s=30)
        recorder.cleanup()

    Example (programmatic):
        recorder = RynnRecorder(
            repo_id="user/my_dataset",
            fps=30,
            robot_type="so101",
            sources=[joint_source, camera_source],
        )
        recorder.setup()
        recorder.record_dataset(task="pick and place", num_episodes=10, episode_time_s=30)
        recorder.cleanup()
    """

    def __init__(
        self,
        config_path: str | Path | None = None,
        repo_id: str | None = None,
        fps: int = 30,
        robot_type: str | None = None,
        sources: list[ISensorSource] | None = None,
        kinematics: IKinematicsProcessor | None = None,
        alignment_mode: str = "sync",
        use_videos: bool = True,
        root: str | Path | None = None,
        image_writer_processes: int = 0,
        image_writer_threads: int = 4,
        batch_encoding_size: int = 1,
        vcodec: str = "libsvtav1",
        streaming_encoding: bool = False,
        encoder_threads: int | None = None,
        **context,
    ):
        self.config_path = config_path
        self.repo_id = repo_id
        self.fps = fps
        self.robot_type = robot_type
        self.alignment_mode = alignment_mode
        self.use_videos = use_videos
        self.root = root
        self.image_writer_processes = image_writer_processes
        self.image_writer_threads = image_writer_threads
        self.batch_encoding_size = batch_encoding_size
        self.vcodec = vcodec
        self.streaming_encoding = streaming_encoding
        self.encoder_threads = encoder_threads
        self._context = context

        # Set from config or args
        self._sources: list[ISensorSource] = sources or []
        self._kinematics: IKinematicsProcessor | None = kinematics
        self._aligner: TemporalAligner | None = None
        self._dataset = None
        self._is_setup = False

    def setup(self) -> None:
        """Initialize all components: sources, aligner, kinematics, dataset."""
        if self._is_setup:
            logger.warning("RynnRecorder already set up")
            return

        # Load from YAML if provided
        if self.config_path is not None:
            self._load_from_config()

        # Connect all sources
        for source in self._sources:
            source.connect()
            logger.info(f"Connected source: {source.name} ({source.sensor_type.value})")

        # Build features from all sources
        features = self._collect_features()

        # Create aligner
        source_names = [s.name for s in self._sources]
        self._aligner = TemporalAligner(
            source_names=source_names,
            mode=self.alignment_mode,
            fps=self.fps,
        )

        # Create dataset
        from RynnMotion.RynnDatasets.rynn_dataset import RynnDataset

        self._dataset = RynnDataset.create(
            repo_id=self.repo_id,
            fps=self.fps,
            features=features,
            root=self.root,
            robot_type=self.robot_type,
            use_videos=self.use_videos,
            image_writer_processes=self.image_writer_processes,
            image_writer_threads=self.image_writer_threads,
            batch_encoding_size=self.batch_encoding_size,
            vcodec=self.vcodec,
            streaming_encoding=self.streaming_encoding,
            encoder_threads=self.encoder_threads,
        )

        self._is_setup = True
        logger.info(
            f"RynnRecorder setup complete: {len(self._sources)} sources, "
            f"mode={self.alignment_mode}, fps={self.fps}"
        )

    def _load_from_config(self) -> None:
        """Load configuration from YAML and create sources."""
        config = SensorSourceRegistry.from_yaml(self.config_path)

        # Override with config values if not set programmatically
        if self.repo_id is None:
            self.repo_id = config.repo_id
        if self.robot_type is None:
            self.robot_type = config.robot_type
        self.fps = config.fps
        self.alignment_mode = config.alignment_mode
        self.use_videos = config.use_videos

        # Create sources from config
        if not self._sources:
            self._sources = SensorSourceRegistry.create_sources(config, **self._context)

        # Create kinematics processor from config
        if self._kinematics is None and config.kinematics:
            kine_cfg = config.kinematics
            self._kinematics = PinocchioKinematicsProcessor(
                mjcf_path=kine_cfg["mjcf_path"],
                site_names=kine_cfg.get("site_names", []),
                joint_source_key=kine_cfg.get("joint_source_key", "observation.state"),
                action_joint_source_key=kine_cfg.get("action_joint_source_key"),
                godview=kine_cfg.get("godview", True),
                reference_camera=kine_cfg.get("reference_camera", "camera_front"),
            )

    def _collect_features(self) -> dict[str, dict]:
        """Collect dataset features from all sources and kinematics."""
        features = {}
        for source in self._sources:
            features.update(source.get_features())

        if self._kinematics is not None:
            features.update(self._kinematics.get_features())

        return features

    def record_frame(self) -> dict[str, Any] | None:
        """Read all sources, align, process kinematics, return frame dict.

        Returns:
            Frame dict ready for dataset.add_frame(), or None if alignment fails.
        """
        if self.alignment_mode == "sync":
            return self._record_frame_sync()
        else:
            return self._record_frame_async()

    def _record_frame_sync(self) -> dict[str, Any]:
        """Synchronous frame: read all sources in lock-step."""
        frame_data: dict[str, SensorReading] = {}
        merged: dict[str, Any] = {}

        for source in self._sources:
            reading = source.read()
            frame_data[source.name] = reading
            merged.update(reading.data)

        # Process kinematics
        if self._kinematics is not None:
            kine_results = self._kinematics.process(frame_data)
            merged.update(kine_results)

        return merged

    def _record_frame_async(self) -> dict[str, Any] | None:
        """Async frame: read sources, push to aligner, get aligned frame."""
        # Read and push all sources
        for source in self._sources:
            reading = source.read()
            self._aligner.push(reading)

            # Spin ROS2 nodes if applicable
            if hasattr(source, "spin_once"):
                source.spin_once(timeout_sec=0.001)

        # Get aligned frame
        aligned = self._aligner.get_aligned_frame()
        if aligned is None:
            return None

        # Merge all data
        merged: dict[str, Any] = {}
        for reading in aligned.values():
            merged.update(reading.data)

        # Process kinematics
        if self._kinematics is not None:
            kine_results = self._kinematics.process(aligned)
            merged.update(kine_results)

        return merged

    def record_episode(
        self,
        task: str,
        episode_length_s: float,
        warmup_s: float = 0.0,
        events: dict | None = None,
    ) -> bool:
        """Record a single episode.

        Args:
            task: Task description string.
            episode_length_s: Maximum episode duration in seconds.
            warmup_s: Warmup time before recording starts.
            events: Optional keyboard events dict for episode control.
                Expected keys: "exit_early", "rerecord_episode", "stop_recording".

        Returns:
            True if episode was saved, False if discarded/interrupted.
        """
        if not self._is_setup:
            raise RuntimeError("Call setup() before record_episode()")

        target_interval = 1.0 / self.fps
        max_frames = int(episode_length_s * self.fps)

        # Warmup
        if warmup_s > 0:
            logger.info(f"Warming up for {warmup_s}s...")
            time.sleep(warmup_s)

        logger.info(f"Recording episode (max {episode_length_s}s, {max_frames} frames)...")

        frame_count = 0
        t_start = time.perf_counter()

        for _ in range(max_frames):
            t_frame_start = time.perf_counter()

            # Check events
            if events:
                if events.get("stop_recording"):
                    logger.info("Stop recording requested")
                    return False
                if events.get("rerecord_episode"):
                    logger.info("Re-record requested, discarding episode")
                    self._dataset.clear_episode_buffer()
                    events["rerecord_episode"] = False
                    return False
                if events.get("exit_early"):
                    logger.info("Exit early requested")
                    events["exit_early"] = False
                    break

            # Record frame
            frame = self.record_frame()
            if frame is not None:
                frame["task"] = task
                frame["timestamp"] = time.perf_counter() - t_start
                self._dataset.add_frame(frame)
                frame_count += 1

            # FPS control
            elapsed = time.perf_counter() - t_frame_start
            sleep_time = target_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        if frame_count == 0:
            logger.warning("No frames recorded in episode")
            return False

        # Save episode
        self._dataset.save_episode()
        logger.info(f"Episode saved: {frame_count} frames")
        return True

    def record_dataset(
        self,
        task: str,
        num_episodes: int,
        episode_time_s: float,
        warmup_s: float = 0.0,
        reset_time_s: float = 5.0,
        push_to_hub: bool = False,
        events: dict | None = None,
    ) -> None:
        """Record a full dataset with multiple episodes.

        Args:
            task: Task description.
            num_episodes: Number of episodes to record.
            episode_time_s: Duration per episode in seconds.
            warmup_s: Warmup before first episode.
            reset_time_s: Time between episodes for environment reset.
            push_to_hub: Whether to push to HuggingFace Hub after recording.
            events: Optional keyboard events dict.
        """
        if not self._is_setup:
            raise RuntimeError("Call setup() before record_dataset()")

        logger.info(
            f"Recording dataset: {num_episodes} episodes, "
            f"{episode_time_s}s each, task='{task}'"
        )

        episodes_recorded = 0
        for ep_idx in range(num_episodes):
            logger.info(f"\n--- Episode {ep_idx + 1}/{num_episodes} ---")

            if events and events.get("stop_recording"):
                logger.info("Recording stopped by user")
                break

            success = self.record_episode(
                task=task,
                episode_length_s=episode_time_s,
                warmup_s=warmup_s if ep_idx == 0 else 0,
                events=events,
            )

            if success:
                episodes_recorded += 1

            # Reset time between episodes
            if ep_idx < num_episodes - 1 and reset_time_s > 0:
                logger.info(f"Reset environment ({reset_time_s}s)...")
                time.sleep(reset_time_s)

        logger.info(
            f"Dataset recording complete: {episodes_recorded}/{num_episodes} episodes"
        )

        if push_to_hub and episodes_recorded > 0:
            logger.info("Pushing dataset to HuggingFace Hub...")
            self._dataset.push_to_hub()
            logger.info("Push complete")

    def cleanup(self) -> None:
        """Disconnect all sources, stop aligner, finalize and clean up dataset."""
        for source in self._sources:
            try:
                source.disconnect()
            except Exception as e:
                logger.warning(f"Error disconnecting source '{source.name}': {e}")

        if self._aligner is not None:
            self._aligner.reset()

        if self._dataset is not None:
            self._dataset.finalize()
            self._dataset.stop_image_writer()

        self._is_setup = False
        logger.info("RynnRecorder cleaned up")

    @property
    def dataset(self):
        """Access the underlying RynnDataset instance."""
        return self._dataset

    @property
    def sources(self) -> list[ISensorSource]:
        """Access the list of sensor sources."""
        return self._sources
