"""Tests for the unified sensor-source recording pipeline.

Exercises sources, alignment, hw_to_dataset_features utils,
SensorSourceRegistry, and RynnRecorder with mock objects.
"""

import time
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from RynnMotion.RynnDatasets.sources import ISensorSource, SensorReading, SensorType
from RynnMotion.RynnDatasets.sources.joint_source import DirectJointSensorSource
from RynnMotion.RynnDatasets.sources.camera_source import DirectCameraSensorSource
from RynnMotion.RynnDatasets.sources.wrench_source import DirectWrenchSensorSource
from RynnMotion.RynnDatasets.sources.imu_source import DirectIMUSensorSource
from RynnMotion.RynnDatasets.sources.tactile_source import DirectTactileSensorSource
from RynnMotion.RynnDatasets.sources.depth_source import DirectDepthSensorSource
from RynnMotion.RynnDatasets.sources.registry import SensorSourceRegistry
from RynnMotion.RynnDatasets.alignment import TemporalAligner
from RynnMotion.RynnDatasets.utils import hw_to_dataset_features


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def mock_joint_reader():
    """Callable returning a fixed 3-DOF joint array."""
    return lambda: np.array([0.1, 0.2, 0.3], dtype=np.float32)


@pytest.fixture
def mock_camera():
    """Object with .read() returning a 480x640x3 image."""
    cam = MagicMock()
    cam.read.return_value = np.zeros((480, 640, 3), dtype=np.uint8)
    return cam


@pytest.fixture
def mock_depth_camera():
    """Object with .read() returning a 480x640 depth image."""
    cam = MagicMock()
    cam.read.return_value = np.zeros((480, 640), dtype=np.float32)
    return cam


@pytest.fixture
def sample_yaml_config(tmp_path):
    """Write a minimal YAML pipeline config and return its path."""
    yaml_text = """\
robot_type: mock_bot
fps: 10
alignment_mode: sync
repo_id: test/yaml_dataset
use_videos: false

sources:
  - name: arm_joints
    source: direct
    sensor_type: joint
    params:
      joint_names: [j1, j2, j3]
      prefix: observation

  - name: camera_front
    source: direct
    sensor_type: camera
    params:
      resolution: [480, 640, 3]
      fps: 10
"""
    p = tmp_path / "pipeline.yaml"
    p.write_text(yaml_text)
    return p


# ===========================================================================
# 1. DirectJointSensorSource tests
# ===========================================================================

class TestDirectJointSensorSource:
    def test_read(self, mock_joint_reader):
        src = DirectJointSensorSource(
            "arm", ["j1", "j2", "j3"], mock_joint_reader,
        )
        reading = src.read()
        assert isinstance(reading, SensorReading)
        assert reading.name == "arm"
        assert "observation.state" in reading.data
        np.testing.assert_array_equal(
            reading.data["observation.state"],
            np.array([0.1, 0.2, 0.3], dtype=np.float32),
        )
        assert reading.timestamp > 0

    def test_features(self, mock_joint_reader):
        src = DirectJointSensorSource(
            "arm", ["j1", "j2", "j3"], mock_joint_reader,
        )
        feats = src.get_features()
        assert "observation.state" in feats
        ft = feats["observation.state"]
        assert ft["dtype"] == "float32"
        assert ft["shape"] == (3,)
        assert ft["names"] == ["j1", "j2", "j3"]

    def test_with_suffix(self, mock_joint_reader):
        src = DirectJointSensorSource(
            "arm", ["j1", "j2"], mock_joint_reader, suffix="arm",
        )
        reading = src.read()
        assert "observation.state.arm" in reading.data
        feats = src.get_features()
        assert "observation.state.arm" in feats

    def test_action_prefix(self, mock_joint_reader):
        src = DirectJointSensorSource(
            "act", ["j1", "j2", "j3"], mock_joint_reader, prefix="action",
        )
        reading = src.read()
        assert "action" in reading.data
        feats = src.get_features()
        assert "action" in feats

    def test_action_prefix_with_suffix(self, mock_joint_reader):
        src = DirectJointSensorSource(
            "act", ["j1", "j2"], mock_joint_reader,
            prefix="action", suffix="gripper",
        )
        reading = src.read()
        assert "action.gripper" in reading.data

    def test_dtype_coercion(self):
        """Non-float32 input should be coerced to float32."""
        src = DirectJointSensorSource(
            "arm", ["j1"], lambda: np.array([1.0], dtype=np.float64),
        )
        reading = src.read()
        assert reading.data["observation.state"].dtype == np.float32


# ===========================================================================
# 2. DirectCameraSensorSource tests
# ===========================================================================

class TestDirectCameraSensorSource:
    def test_read_mock(self, mock_camera):
        src = DirectCameraSensorSource(
            "front", mock_camera, resolution=(480, 640, 3),
        )
        src.connect()
        reading = src.read()
        assert isinstance(reading, SensorReading)
        assert "observation.images.front" in reading.data
        assert reading.data["observation.images.front"].shape == (480, 640, 3)

    def test_features(self, mock_camera):
        src = DirectCameraSensorSource(
            "front", mock_camera, resolution=(480, 640, 3), use_video=True,
        )
        feats = src.get_features()
        ft = feats["observation.images.front"]
        assert ft["dtype"] == "video"
        assert ft["shape"] == (480, 640, 3)

    def test_not_connected_raises(self, mock_camera):
        src = DirectCameraSensorSource("front", mock_camera)
        with pytest.raises(RuntimeError, match="not connected"):
            src.read()


# ===========================================================================
# 3. DirectWrenchSensorSource tests
# ===========================================================================

class TestDirectWrenchSensorSource:
    def test_read_and_features(self):
        reader = lambda: np.array([1, 2, 3, 4, 5, 6], dtype=np.float32)
        src = DirectWrenchSensorSource("wrist", reader)
        src.connect()
        reading = src.read()
        assert "observation.state.wrench" in reading.data
        assert reading.data["observation.state.wrench"].shape == (6,)

        feats = src.get_features()
        ft = feats["observation.state.wrench"]
        assert ft["shape"] == (6,)
        assert ft["names"] == ["fx", "fy", "fz", "tx", "ty", "tz"]

    def test_with_suffix(self):
        reader = lambda: np.zeros(6, dtype=np.float32)
        src = DirectWrenchSensorSource("w", reader, suffix="left")
        feats = src.get_features()
        assert "observation.state.wrench.left" in feats


# ===========================================================================
# 4. DirectIMUSensorSource tests
# ===========================================================================

class TestDirectIMUSensorSource:
    @pytest.mark.parametrize("ndims,expected_names", [
        (6, ["ax", "ay", "az", "gx", "gy", "gz"]),
        (9, ["ax", "ay", "az", "gx", "gy", "gz", "qx", "qy", "qz"]),
        (10, ["ax", "ay", "az", "gx", "gy", "gz", "qw", "qx", "qy", "qz"]),
    ])
    def test_imu_dimensions(self, ndims, expected_names):
        reader = lambda: np.zeros(ndims, dtype=np.float32)
        src = DirectIMUSensorSource("body", reader, num_dims=ndims)
        feats = src.get_features()
        key = "observation.state.imu"
        assert key in feats
        assert feats[key]["shape"] == (ndims,)
        assert feats[key]["names"] == expected_names

    def test_read(self):
        reader = lambda: np.ones(10, dtype=np.float32)
        src = DirectIMUSensorSource("body", reader)
        reading = src.read()
        assert "observation.state.imu" in reading.data
        assert reading.data["observation.state.imu"].shape == (10,)


# ===========================================================================
# 5. DirectTactileSensorSource tests
# ===========================================================================

class TestDirectTactileSensorSource:
    def test_read_and_features(self):
        reader = lambda: np.zeros(16, dtype=np.float32)
        src = DirectTactileSensorSource("finger", reader, num_dims=16)
        reading = src.read()
        assert "observation.state.tactile" in reading.data
        assert reading.data["observation.state.tactile"].shape == (16,)

        feats = src.get_features()
        ft = feats["observation.state.tactile"]
        assert ft["shape"] == (16,)
        assert ft["names"][0] == "taxel_0"
        assert ft["names"][-1] == "taxel_15"

    def test_custom_dims(self):
        reader = lambda: np.zeros(8, dtype=np.float32)
        src = DirectTactileSensorSource("pad", reader, num_dims=8)
        feats = src.get_features()
        assert feats["observation.state.tactile"]["shape"] == (8,)


# ===========================================================================
# 6. DirectDepthSensorSource tests
# ===========================================================================

class TestDirectDepthSensorSource:
    def test_read_2d_input(self, mock_depth_camera):
        src = DirectDepthSensorSource(
            "front", mock_depth_camera, resolution=(480, 640, 1),
        )
        src.connect()
        reading = src.read()
        key = "observation.images.depth_front"
        assert key in reading.data
        assert reading.data[key].shape == (480, 640, 1)

    def test_features(self, mock_depth_camera):
        src = DirectDepthSensorSource(
            "front", mock_depth_camera, resolution=(480, 640, 1),
        )
        feats = src.get_features()
        assert "observation.images.depth_front" in feats


# ===========================================================================
# 7. TemporalAligner tests
# ===========================================================================

class TestTemporalAligner:
    def test_sync_passthrough(self):
        aligner = TemporalAligner(["a", "b"], mode="sync")
        # push is a no-op in sync
        aligner.push(SensorReading("a", 1.0, {"x": 1}))
        assert aligner.get_aligned_frame() is None

    def test_async_alignment(self):
        aligner = TemporalAligner(["a", "b"], mode="async")
        # Source 'a' latest = 2.0, source 'b' latest = 1.5
        # frame_time = min(2.0, 1.5) = 1.5
        aligner.push(SensorReading("a", 1.0, {"x": 10}))
        aligner.push(SensorReading("a", 2.0, {"x": 20}))
        aligner.push(SensorReading("b", 1.5, {"y": 15}))

        frame = aligner.get_aligned_frame()
        assert frame is not None
        assert "a" in frame and "b" in frame
        # 'a' reading at t=1.0 is < 1.5 so popped; returns t=2.0
        assert frame["a"].data == {"x": 20}
        assert frame["b"].data == {"y": 15}

    def test_async_missing_source(self):
        aligner = TemporalAligner(["a", "b"], mode="async")
        aligner.push(SensorReading("a", 1.0, {"x": 1}))
        # 'b' has no data
        assert aligner.get_aligned_frame() is None

    def test_reset(self):
        aligner = TemporalAligner(["a"], mode="async")
        aligner.push(SensorReading("a", 1.0, {"x": 1}))
        aligner.reset()
        assert aligner.get_aligned_frame() is None

    def test_invalid_mode(self):
        with pytest.raises(ValueError, match="mode must be"):
            TemporalAligner(["a"], mode="invalid")


# ===========================================================================
# 8. hw_to_dataset_features tests
# ===========================================================================

class TestHwToDatasetFeatures:
    def test_wrench(self):
        hw = {"wrist_ft": {"wrench": 6}}
        feats = hw_to_dataset_features(hw, "observation")
        key = "observation.state.wrench.wrist_ft"
        assert key in feats
        assert feats[key]["shape"] == (6,)
        assert feats[key]["names"] == ["fx", "fy", "fz", "tx", "ty", "tz"]

    def test_imu(self):
        hw = {"body_imu": {"imu": 10}}
        feats = hw_to_dataset_features(hw, "observation")
        key = "observation.state.imu.body_imu"
        assert key in feats
        assert feats[key]["shape"] == (10,)

    def test_depth(self):
        hw = {"front": {"depth": [480, 640, 1]}}
        feats = hw_to_dataset_features(hw, "observation")
        key = "observation.images.depth_front"
        assert key in feats
        assert feats[key]["shape"] == (480, 640, 1)

    def test_tactile(self):
        hw = {"finger": {"tactile": 16}}
        feats = hw_to_dataset_features(hw, "observation")
        key = "observation.state.tactile.finger"
        assert key in feats
        assert feats[key]["shape"] == (16,)

    def test_backward_compat_float(self):
        hw = {"shoulder_pan": float, "elbow": float}
        feats = hw_to_dataset_features(hw, "observation")
        assert "observation.state" in feats
        ft = feats["observation.state"]
        assert ft["dtype"] == "float32"
        assert ft["shape"] == (2,)

    def test_backward_compat_tuple(self):
        hw = {"cam_front": (480, 640, 3)}
        feats = hw_to_dataset_features(hw, "observation", use_video=True)
        assert "observation.images.cam_front" in feats
        assert feats["observation.images.cam_front"]["dtype"] == "video"

    def test_backward_compat_list(self):
        hw = {"ee_pose": [7]}
        feats = hw_to_dataset_features(hw, "observation")
        key = "observation.ee_pose"
        assert key in feats
        assert feats[key]["shape"] == (7,)

    def test_action_prefix(self):
        hw = {"j1": float, "j2": float}
        feats = hw_to_dataset_features(hw, "action")
        assert "action" in feats
        assert feats["action"]["shape"] == (2,)


# ===========================================================================
# 9. SensorSourceRegistry tests
# ===========================================================================

class TestSensorSourceRegistry:
    def test_from_yaml(self, sample_yaml_config):
        config = SensorSourceRegistry.from_yaml(sample_yaml_config)
        assert config.robot_type == "mock_bot"
        assert config.fps == 10
        assert config.alignment_mode == "sync"
        assert config.repo_id == "test/yaml_dataset"
        assert config.use_videos is False
        assert len(config.sources) == 2
        assert config.sources[0].name == "arm_joints"
        assert config.sources[0].sensor_type == "joint"
        assert config.sources[1].name == "camera_front"
        assert config.sources[1].sensor_type == "camera"

    def test_create_sources_with_context(self, sample_yaml_config, mock_joint_reader, mock_camera):
        config = SensorSourceRegistry.from_yaml(sample_yaml_config)
        sources = SensorSourceRegistry.create_sources(
            config,
            reader_fns={"arm_joints": mock_joint_reader},
            camera_objects={"camera_front": mock_camera},
        )
        assert len(sources) == 2
        assert isinstance(sources[0], DirectJointSensorSource)
        assert isinstance(sources[1], DirectCameraSensorSource)

    def test_missing_reader_fn_skips_source(self, sample_yaml_config, mock_camera):
        config = SensorSourceRegistry.from_yaml(sample_yaml_config)
        sources = SensorSourceRegistry.create_sources(
            config,
            reader_fns={},  # no reader for arm_joints
            camera_objects={"camera_front": mock_camera},
        )
        # joint source skipped, only camera remains
        assert len(sources) == 1
        assert isinstance(sources[0], DirectCameraSensorSource)


# ===========================================================================
# 10. RynnRecorder tests (with mocked RynnDataset)
# ===========================================================================

class TestRynnRecorder:
    def _make_sources(self):
        joint_src = DirectJointSensorSource(
            "arm", ["j1", "j2", "j3"],
            lambda: np.array([0.1, 0.2, 0.3], dtype=np.float32),
        )

        cam = MagicMock()
        cam.read.return_value = np.zeros((480, 640, 3), dtype=np.uint8)
        cam_src = DirectCameraSensorSource(
            "front", cam, resolution=(480, 640, 3),
        )

        return [joint_src, cam_src]

    def _make_mock_dataset(self):
        ds = MagicMock()
        ds.features = {}
        ds.meta = MagicMock()
        ds.meta.total_episodes = 0
        ds.meta.total_frames = 0
        ds.meta.video_keys = []
        ds.meta.camera_keys = []
        return ds

    def test_setup_with_mock_sources(self):
        from RynnMotion.RynnDatasets.recorder import RynnRecorder

        sources = self._make_sources()
        recorder = RynnRecorder(
            repo_id="test/mock",
            fps=10,
            robot_type="mock",
            sources=sources,
        )

        mock_ds = self._make_mock_dataset()
        with patch(
            "RynnMotion.RynnDatasets.rynn_dataset.RynnDataset.create",
            return_value=mock_ds,
        ):
            recorder.setup()

        assert recorder._is_setup
        assert recorder._aligner is not None
        assert len(recorder._sources) == 2

    def test_record_frame_sync(self):
        from RynnMotion.RynnDatasets.recorder import RynnRecorder

        sources = self._make_sources()
        recorder = RynnRecorder(
            repo_id="test/mock",
            fps=10,
            robot_type="mock",
            sources=sources,
            alignment_mode="sync",
        )

        mock_ds = self._make_mock_dataset()
        with patch(
            "RynnMotion.RynnDatasets.rynn_dataset.RynnDataset.create",
            return_value=mock_ds,
        ):
            recorder.setup()

        frame = recorder.record_frame()
        assert frame is not None
        assert "observation.state" in frame
        assert "observation.images.front" in frame
        np.testing.assert_array_equal(
            frame["observation.state"],
            np.array([0.1, 0.2, 0.3], dtype=np.float32),
        )

    def test_record_frame_not_setup_raises(self):
        from RynnMotion.RynnDatasets.recorder import RynnRecorder

        recorder = RynnRecorder(repo_id="test/mock", fps=10)
        with pytest.raises(RuntimeError, match="setup"):
            recorder.record_episode(task="test", episode_length_s=1.0)

    def test_record_episode_with_mock_dataset(self):
        from RynnMotion.RynnDatasets.recorder import RynnRecorder

        sources = self._make_sources()
        recorder = RynnRecorder(
            repo_id="test/mock",
            fps=10,
            robot_type="mock",
            sources=sources,
        )

        mock_ds = self._make_mock_dataset()
        with patch(
            "RynnMotion.RynnDatasets.rynn_dataset.RynnDataset.create",
            return_value=mock_ds,
        ):
            recorder.setup()

        success = recorder.record_episode(task="test task", episode_length_s=0.3)
        assert success
        # At 10fps for 0.3s => 3 frames
        assert mock_ds.add_frame.call_count == 3
        mock_ds.save_episode.assert_called_once()

    def test_cleanup(self):
        from RynnMotion.RynnDatasets.recorder import RynnRecorder

        sources = self._make_sources()
        recorder = RynnRecorder(
            repo_id="test/mock",
            fps=10,
            robot_type="mock",
            sources=sources,
        )

        mock_ds = self._make_mock_dataset()
        with patch(
            "RynnMotion.RynnDatasets.rynn_dataset.RynnDataset.create",
            return_value=mock_ds,
        ):
            recorder.setup()

        recorder.cleanup()
        assert not recorder._is_setup
        mock_ds.stop_image_writer.assert_called_once()
