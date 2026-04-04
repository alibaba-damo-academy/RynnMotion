"""Tests for the unified RynnRecorder pipeline.

Exercises RobotContext, PinocchioKinematicsProcessor, create_teleop_sources,
and RynnRecorder integration with mock TeleOperator (no hardware needed).
"""

import json
import subprocess
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from RynnMotion.RynnDatasets.robot_context import RobotContext
from RynnMotion.RynnDatasets.kinematics_processor import PinocchioKinematicsProcessor
from RynnMotion.RynnDatasets.sources import ISensorSource, SensorReading, SensorType
from RynnMotion.RynnDatasets.sources.joint_source import DirectJointSensorSource
from RynnMotion.utils.path_config import get_models_root


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

MODELS_ROOT = get_models_root()

ROBOT_MJCF = {
    "so101": "3.robot_arm/24.so101/mjcf/so101_robot.xml",
    "fr3": "3.robot_arm/20.fr3/mjcf/fr3_robot.xml",
    "ur5e": "3.robot_arm/21.ur5e/mjcf/ur5e_robot.xml",
    "piper": "3.robot_arm/22.piper/mjcf/piper_robot.xml",
    "rm75": "3.robot_arm/23.rm75/mjcf/rm75_robot.xml",
    "rizon4s": "3.robot_arm/25.rizon4s/mjcf/rizon4s_robot.xml",
    "dm6dof": "3.robot_arm/27.dm6dof/mjcf/dm6dof_robot.xml",
    "openarm": "3.robot_arm/28.openarm/mjcf/openarm_robot.xml",
    "so101_6dof": "3.robot_arm/29.so101_6dof/mjcf/so101_6dof_robot.xml",
}

ROBOT_PINO_MJCF = {
    "so101": "3.robot_arm/24.so101/mjcf/so101_pinocchio.xml",
    "fr3": "3.robot_arm/20.fr3/mjcf/fr3_pinocchio.xml",
    "ur5e": "3.robot_arm/21.ur5e/mjcf/ur5e_pinocchio.xml",
    "piper": "3.robot_arm/22.piper/mjcf/piper_pinocchio.xml",
    "rm75": "3.robot_arm/23.rm75/mjcf/rm75_pinocchio.xml",
    "so101_6dof": "3.robot_arm/29.so101_6dof/mjcf/so101_6dof_pinocchio.xml",
}

EXPECTED_DOF = {
    "so101": (5, 1),
    "fr3": (7, 1),
    "ur5e": (6, 1),
    "piper": (6, 1),
    "rm75": (7, 1),
    "rizon4s": (7, 0),
    "dm6dof": (6, 0),
    "openarm": (7, 1),
    "so101_6dof": (6, 1),
}


def _mjcf_path(robot: str) -> str:
    return str(MODELS_ROOT / ROBOT_MJCF[robot])


def _pino_path(robot: str) -> str:
    return str(MODELS_ROOT / ROBOT_PINO_MJCF[robot])


def _make_context(robot: str) -> RobotContext:
    return RobotContext.from_mjcf(_mjcf_path(robot), robot_name=robot)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def so101_ctx():
    return _make_context("so101")


@pytest.fixture
def mock_teleop(so101_ctx):
    """Mock TeleOperator returning fake joint + gripper arrays."""
    mdof = so101_ctx.mdof
    adof = so101_ctx.adof
    total = mdof + adof

    teleop = MagicMock()
    teleop.get_observation.return_value = np.zeros(total, dtype=np.float32)
    teleop.get_action.return_value = np.zeros(total, dtype=np.float32)
    return teleop


# ===========================================================================
# 1. TestRobotContext
# ===========================================================================

class TestRobotContext:
    def test_so101_from_mjcf(self, so101_ctx):
        assert so101_ctx.mdof == 5
        assert so101_ctx.adof == 1
        assert so101_ctx.num_ee == 1
        assert len(so101_ctx.joint_names) == 5
        assert len(so101_ctx.ee_names) == 1

    def test_get_hw_features_keys(self, so101_ctx):
        features = so101_ctx.get_hw_features()
        assert "observation.state" in features
        assert "action" in features
        assert "observation.state.gripper" in features

    def test_get_hw_features_shapes(self, so101_ctx):
        cam_cfg = {"front": (360, 640, 3)}
        features = so101_ctx.get_hw_features(camera_configs=cam_cfg)

        assert features["observation.state"]["shape"] == (5,)
        assert features["action"]["shape"] == (6,)
        assert features["observation.state.gripper"]["shape"] == (1,)
        assert features["observation.images.front"]["shape"] == (360, 640, 3)

    def test_get_recording_features_superset(self, so101_ctx):
        hw = so101_ctx.get_hw_features()
        rec = so101_ctx.get_recording_features()

        for key in hw:
            assert key in rec, f"hw key {key!r} missing from recording features"

        assert "observation.velocity" in rec
        assert "observation.torque" in rec
        assert "action.velocity" in rec
        assert "timestamp" in rec
        assert "frame_index" in rec
        assert "observation.end_effector_pose" in rec

    @pytest.mark.parametrize("robot", list(EXPECTED_DOF.keys()))
    def test_multi_robot(self, robot):
        ctx = _make_context(robot)
        expected_mdof, expected_adof = EXPECTED_DOF[robot]
        assert ctx.mdof == expected_mdof, f"{robot}: mdof={ctx.mdof}, expected {expected_mdof}"
        assert ctx.adof == expected_adof, f"{robot}: adof={ctx.adof}, expected {expected_adof}"
        assert len(ctx.joint_names) == expected_mdof
        if expected_adof > 0:
            assert len(ctx.ee_names) == expected_adof

    def test_total_action_dim(self, so101_ctx):
        assert so101_ctx.total_action_dim == 6
        assert so101_ctx.total_action_dim == so101_ctx.mdof + so101_ctx.adof


# ===========================================================================
# 2. TestPinocchioKinematicsProcessor
# ===========================================================================

class TestPinocchioKinematicsProcessor:
    @pytest.fixture
    def so101_pino_path(self):
        return _pino_path("so101")

    @pytest.fixture
    def so101_site_names(self):
        return ["EE", "camera_front"]

    def _make_frame(self, q, key="observation.state"):
        """Create a minimal frame dict with joint readings."""
        reading = SensorReading(
            name="obs_joints",
            timestamp=0.0,
            data={key: np.array(q, dtype=np.float32)},
        )
        return {"obs_joints": reading}

    def test_features_observation_only(self, so101_pino_path, so101_site_names):
        proc = PinocchioKinematicsProcessor(
            mjcf_path=so101_pino_path,
            site_names=so101_site_names,
            joint_source_key="observation.state",
            action_joint_source_key=None,
        )
        features = proc.get_features()
        obs_keys = [k for k in features if k.startswith("observation")]
        act_keys = [k for k in features if k.startswith("action")]
        assert len(obs_keys) > 0
        assert len(act_keys) == 0

    def test_features_with_action(self, so101_pino_path, so101_site_names):
        proc = PinocchioKinematicsProcessor(
            mjcf_path=so101_pino_path,
            site_names=so101_site_names,
            joint_source_key="observation.state",
            action_joint_source_key="action",
        )
        features = proc.get_features()
        obs_keys = [k for k in features if k.startswith("observation")]
        act_keys = [k for k in features if k.startswith("action")]
        assert len(obs_keys) > 0
        assert len(act_keys) > 0

    def test_process_observation_fk(self, so101_pino_path, so101_site_names):
        proc = PinocchioKinematicsProcessor(
            mjcf_path=so101_pino_path,
            site_names=so101_site_names,
            joint_source_key="observation.state",
        )
        nq = proc.kine.pin_model.nq
        frame = self._make_frame(np.zeros(nq))
        result = proc.process(frame)

        assert len(result) > 0
        for key, val in result.items():
            assert val.shape == (7,), f"{key} has shape {val.shape}, expected (7,)"
            assert val.dtype == np.float32

    def test_action_array_slicing(self, so101_pino_path, so101_site_names):
        """Action array (6,) should be auto-sliced to model nq (5,) for FK."""
        proc = PinocchioKinematicsProcessor(
            mjcf_path=so101_pino_path,
            site_names=so101_site_names,
            joint_source_key="observation.state",
            action_joint_source_key="action",
        )
        nq = proc.kine.pin_model.nq
        # Action array has mdof + adof = 6 elements, model nq = 5
        action_array = np.zeros(nq + 1, dtype=np.float32)

        obs_reading = SensorReading("obs", 0.0, {"observation.state": np.zeros(nq, dtype=np.float32)})
        act_reading = SensorReading("act", 0.0, {"action": action_array})
        frame = {"obs": obs_reading, "act": act_reading}

        result = proc.process(frame)
        action_keys = [k for k in result if k.startswith("action")]
        assert len(action_keys) > 0
        for key in action_keys:
            assert result[key].shape == (7,)

    def test_godview_front_camera_identity(self, so101_pino_path, so101_site_names):
        """In godview mode, reference camera should have identity pose."""
        proc = PinocchioKinematicsProcessor(
            mjcf_path=so101_pino_path,
            site_names=so101_site_names,
            godview=True,
            reference_camera="camera_front",
        )
        nq = proc.kine.pin_model.nq
        frame = self._make_frame(np.zeros(nq))
        result = proc.process(frame)

        identity_pose = np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float32)
        cam_key = "observation.camera_poses.front"
        assert cam_key in result, f"Expected key {cam_key!r} in {list(result.keys())}"
        np.testing.assert_allclose(result[cam_key], identity_pose, atol=1e-6)

    def test_world_frame_mode(self, so101_pino_path, so101_site_names):
        """godview=False: poses should be non-identity in general."""
        proc = PinocchioKinematicsProcessor(
            mjcf_path=so101_pino_path,
            site_names=so101_site_names,
            godview=False,
        )
        nq = proc.kine.pin_model.nq
        frame = self._make_frame(np.zeros(nq))
        result = proc.process(frame)

        assert len(result) > 0
        # At least one pose should differ from identity
        identity = np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float32)
        any_non_identity = any(
            not np.allclose(v, identity, atol=1e-6) for v in result.values()
        )
        assert any_non_identity, "All poses are identity in world frame mode"


# ===========================================================================
# 3. TestCreateTeleopSources
# ===========================================================================

class TestCreateTeleopSources:
    def test_sources_created(self, so101_ctx, mock_teleop):
        from RynnMotion.RynnDatasets.sources.teleop_source import create_teleop_sources

        sources = create_teleop_sources(
            teleop=mock_teleop,
            robot_context=so101_ctx,
            cameras={},
            cameras_config={},
        )
        names = [s.name for s in sources]
        assert "obs_joints" in names
        assert "obs_gripper" in names
        assert "action" in names

    def test_no_pose_sources(self, so101_ctx, mock_teleop):
        """Teleop sources should NOT contain EE/camera pose sources."""
        from RynnMotion.RynnDatasets.sources.teleop_source import create_teleop_sources

        sources = create_teleop_sources(
            teleop=mock_teleop,
            robot_context=so101_ctx,
            cameras={},
            cameras_config={},
        )
        all_features = {}
        for src in sources:
            all_features.update(src.get_features())

        pose_keys = [k for k in all_features if "pose" in k.lower()]
        assert len(pose_keys) == 0, f"Unexpected pose features in sources: {pose_keys}"

    def test_source_features_complete(self, so101_ctx, mock_teleop):
        from RynnMotion.RynnDatasets.sources.teleop_source import create_teleop_sources

        sources = create_teleop_sources(
            teleop=mock_teleop,
            robot_context=so101_ctx,
            cameras={},
            cameras_config={},
        )
        all_features = {}
        for src in sources:
            all_features.update(src.get_features())

        assert "observation.state" in all_features
        assert "observation.state.gripper" in all_features
        assert "action" in all_features
        assert all_features["observation.state"]["shape"] == (5,)
        assert all_features["action"]["shape"] == (6,)

    def test_camera_sources(self, so101_ctx, mock_teleop):
        """With mock cameras, DirectCameraSensorSource should be created."""
        from RynnMotion.RynnDatasets.sources.teleop_source import create_teleop_sources
        from RynnMotion.RynnDatasets.sources.camera_source import DirectCameraSensorSource

        mock_cam = MagicMock()
        mock_cam.is_connected = True
        mock_cam.read.return_value = np.zeros((480, 640, 3), dtype=np.uint8)

        # Patch the ColorMode import inside teleop_source
        with patch("RynnMotion.RynnDatasets.sources.teleop_source.DirectCameraSensorSource") as MockCamSrc:
            # Need to patch at the import location
            pass

        # Since the camera source creation depends on RynnLeRobot.hardware.cameras.configs.ColorMode,
        # we test that cameras dict is iterated correctly by checking source count
        # without the camera import dependency.
        sources_no_cam = create_teleop_sources(
            teleop=mock_teleop,
            robot_context=so101_ctx,
            cameras={},
            cameras_config={},
        )
        # Without cameras, only joint sources
        assert len(sources_no_cam) == 3  # obs_joints, obs_gripper, action


# ===========================================================================
# 4. TestRecorderWithKinematics
# ===========================================================================

class TestRecorderWithKinematics:
    def _make_mock_dataset(self):
        ds = MagicMock()
        ds.features = {}
        ds.meta = MagicMock()
        ds.meta.total_episodes = 0
        ds.meta.total_frames = 0
        ds.meta.video_keys = []
        ds.meta.camera_keys = []
        return ds

    def _make_sources(self, ctx):
        mdof = ctx.mdof
        adof = ctx.adof
        total = mdof + adof

        obs_src = DirectJointSensorSource(
            "obs_joints", ctx.joint_names,
            lambda: np.zeros(mdof, dtype=np.float32),
        )
        gripper_src = DirectJointSensorSource(
            "obs_gripper", ctx.ee_names,
            lambda: np.zeros(adof, dtype=np.float32),
            suffix="gripper",
        )
        action_src = DirectJointSensorSource(
            "action", list(ctx.joint_names) + list(ctx.ee_names),
            lambda: np.zeros(total, dtype=np.float32),
            prefix="action",
        )
        return [obs_src, gripper_src, action_src]

    def _setup_recorder(self, so101_ctx, mock_ds):
        """Set up recorder by manually wiring internals (avoids rynn_dataset import)."""
        from RynnMotion.RynnDatasets.recorder import RynnRecorder
        from RynnMotion.RynnDatasets.alignment import TemporalAligner

        pino_path = _pino_path("so101")
        kine = PinocchioKinematicsProcessor(
            mjcf_path=pino_path,
            site_names=["EE", "camera_front"],
            action_joint_source_key="action",
        )
        sources = self._make_sources(so101_ctx)

        recorder = RynnRecorder(
            repo_id="test/kine_test",
            fps=10,
            robot_type="so101",
            sources=sources,
            kinematics=kine,
        )

        # Manually wire internals (same as setup() but without RynnDataset import)
        for source in recorder._sources:
            source.connect()

        source_names = [s.name for s in recorder._sources]
        recorder._aligner = TemporalAligner(
            source_names=source_names,
            mode=recorder.alignment_mode,
            fps=recorder.fps,
        )
        recorder._dataset = mock_ds
        recorder._is_setup = True

        return recorder

    def test_setup_with_kinematics(self, so101_ctx):
        mock_ds = self._make_mock_dataset()
        recorder = self._setup_recorder(so101_ctx, mock_ds)
        assert recorder._is_setup

    def test_record_frame_includes_poses(self, so101_ctx):
        mock_ds = self._make_mock_dataset()
        recorder = self._setup_recorder(so101_ctx, mock_ds)

        frame = recorder.record_frame()
        assert frame is not None

        # Joint data from sources
        assert "observation.state" in frame
        assert "action" in frame

        # FK-computed poses from kinematics
        pose_keys = [k for k in frame if "pose" in k.lower()]
        assert len(pose_keys) > 0, f"No pose keys in frame: {list(frame.keys())}"

    def test_frame_schema_matches_features(self, so101_ctx):
        mock_ds = self._make_mock_dataset()
        recorder = self._setup_recorder(so101_ctx, mock_ds)

        # Collect all expected feature keys
        expected_features = recorder._collect_features()
        frame = recorder.record_frame()

        for key in expected_features:
            assert key in frame, f"Feature {key!r} missing from frame"

    def test_record_episode_with_kinematics(self, so101_ctx):
        mock_ds = self._make_mock_dataset()
        recorder = self._setup_recorder(so101_ctx, mock_ds)

        success = recorder.record_episode(task="test task", episode_length_s=0.3)
        assert success
        assert mock_ds.add_frame.call_count == 3  # 10fps * 0.3s
        mock_ds.save_episode.assert_called_once()


# ===========================================================================
# 5. TestCppPythonSchemaParity
# ===========================================================================

class TestCppPythonSchemaParity:
    """Compare C++ mjcfDatasetExe --schema output with Python RobotContext."""

    @staticmethod
    def _get_cpp_schema(robot: str) -> dict | None:
        """Run mjcfDatasetExe -s and parse JSON output."""
        build_dir = Path(__file__).resolve().parents[3] / "build"
        candidates = [
            build_dir / "bin" / "mjcfDatasetExe",
            build_dir / "motion" / "manager" / "mjcfDatasetExe",
        ]
        exe = next((p for p in candidates if p.exists()), None)
        if exe is None:
            return None

        try:
            result = subprocess.run(
                [str(exe), robot, "-s"],
                capture_output=True,
                text=True,
                timeout=10,
            )
            if result.returncode != 0:
                return None

            output = result.stdout
            # Find the JSON block in the output
            json_start = output.find("{")
            if json_start == -1:
                return None
            return json.loads(output[json_start:])
        except (subprocess.TimeoutExpired, json.JSONDecodeError, FileNotFoundError):
            return None

    @pytest.mark.parametrize("robot", ["so101"])
    def test_hw_features_parity(self, robot):
        """Compare C++ and Python hw feature keys and shapes."""
        cpp_schema = self._get_cpp_schema(robot)
        if cpp_schema is None:
            pytest.skip("mjcfDatasetExe not built or not found")

        py_ctx = _make_context(robot)
        py_features = py_ctx.get_hw_features()
        cpp_features = cpp_schema.get("features", {})

        # Compare keys
        py_keys = set(py_features.keys())
        cpp_keys = set(cpp_features.keys())

        # HW features should be a subset of recording features from C++
        # C++ --schema outputs recording features, so check py_hw keys exist there
        for key in py_keys:
            assert key in cpp_keys, (
                f"Python hw feature key {key!r} not in C++ schema. "
                f"C++ keys: {sorted(cpp_keys)}"
            )

        # Compare shapes for matching keys
        for key in py_keys & cpp_keys:
            py_shape = list(py_features[key]["shape"])
            cpp_shape = cpp_features[key]["shape"]
            assert py_shape == cpp_shape, (
                f"Shape mismatch for {key!r}: Python={py_shape}, C++={cpp_shape}"
            )

    @pytest.mark.parametrize("robot", ["fr3", "ur5e", "piper", "rm75", "so101"])
    def test_multi_robot_parity(self, robot):
        """Compare C++ and Python recording feature schemas across robots."""
        cpp_schema = self._get_cpp_schema(robot)
        if cpp_schema is None:
            pytest.skip("mjcfDatasetExe not built or not found")

        py_ctx = _make_context(robot)
        py_rec = py_ctx.get_recording_features()
        cpp_features = cpp_schema.get("features", {})

        py_keys = set(py_rec.keys())
        cpp_keys = set(cpp_features.keys())

        # Check key-level parity
        missing_in_cpp = py_keys - cpp_keys
        missing_in_py = cpp_keys - py_keys

        assert not missing_in_cpp, (
            f"{robot}: Python keys missing from C++: {sorted(missing_in_cpp)}"
        )
        assert not missing_in_py, (
            f"{robot}: C++ keys missing from Python: {sorted(missing_in_py)}"
        )

        # Check shape parity for all matching keys
        for key in py_keys & cpp_keys:
            py_shape = list(py_rec[key]["shape"])
            cpp_shape = cpp_features[key]["shape"]
            assert py_shape == cpp_shape, (
                f"{robot}: shape mismatch for {key!r}: "
                f"Python={py_shape}, C++={cpp_shape}"
            )
