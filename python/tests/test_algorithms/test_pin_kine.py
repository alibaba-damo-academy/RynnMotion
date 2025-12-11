"""
Pytest integration of test_kine.py

Tests forward kinematics (FK) for multiple robots comparing Pinocchio vs MuJoCo.
Integrated with PRobotManager for path resolution.
"""

import numpy as np
import pytest
import mujoco

from RynnMotion.algorithms.pin_kine import PinKine
from RynnMotion.utils.path_config import get_models_root


# ========== Fixtures ==========

@pytest.fixture
def tolerance():
    """Tolerance for FK comparison tests"""
    return {
        'position': 1e-3,  # 1mm
        'quaternion': 1e-3
    }


@pytest.fixture(params=[
    ("fr3", 20, "../models/3.robot_arm/20.fr3/mjcf/fr3_pinocchio.xml"),
    ("ur5e", 21, "../models/3.robot_arm/21.ur5e/mjcf/ur5e_pinocchio.xml"),
    ("piper", 22, "../models/3.robot_arm/22.piper/mjcf/piper_pinocchio.xml"),
    ("rm75", 23, "../models/3.robot_arm/23.rm75/mjcf/rm75_pinocchio.xml"),
    ("so101", 24, "../models/3.robot_arm/24.so101/mjcf/so101_pinocchio.xml"),
])
def robot_config(request):
    """Fixture providing robot configuration for all supported robots"""
    robot_name, robot_number, pino_mjcf_path = request.param
    models_root = get_models_root()
    pino_mjcf = str(models_root / pino_mjcf_path.lstrip("../models/"))

    return {
        'name': robot_name,
        'number': robot_number,
        'pino_mjcf': pino_mjcf,
        'ee_site_name': 'EE',
    }


@pytest.fixture
def test_configurations():
    """Test joint configurations for each robot"""
    return {
        'fr3': [
            np.array([0, 0, 0, -1.57079, 0, 1.57079, 0.7853]),  # stand
            np.array([np.pi*0.25, np.pi*0.25, np.pi*0.25, -np.pi*0.25,
                     np.pi*0.25, np.pi*0.25, np.pi*0.25]),
        ],
        'ur5e': [
            np.array([0, -1.57079, 0, -1.57079, 0, 0]),  # home
            np.array([0, -1.57079, 1.57079, -3.14, -1.57079, 0]),
        ],
        'piper': [
            np.array([0, 0, 0, 0, 0, 0]),  # home
            np.array([0.5, 1.5, -1.0, 0.5, -0.6, 1.0]),
        ],
        'rm75': [
            np.array([0, 0, 0, 0, 0, 0, 0]),  # home
            np.array([0.735522, 0.388828, -0.9801, 1.74523, -0.419516, -0.502513, 0.699497]),
            np.array([0.5, -0.3, 0.8, -1.2, 0.6, -0.8, 1.0]),
        ],
        'so101': [
            np.array([0, 0, 0, 0, 0]),  # home
            np.array([0, 0, 0, 1.57079, -1.57079]),  # stand
            np.array([-0.0006442706799134612, -1.743825912475586, 1.7453292608261108,
                     0.8737468719482422, 0.08311091363430023]),
        ],
    }


# ========== Single-Site FK Tests ==========

class TestSingleSiteFK:
    """Test single end-effector site forward kinematics"""

    def test_initialization(self, robot_config):
        """Test that PinKine initializes correctly for each robot"""
        pino_mjcf = robot_config['pino_mjcf']
        ee_site_name = robot_config['ee_site_name']

        # Should not raise
        pin_kine = PinKine(pino_mjcf, site_names=[ee_site_name])

        # Basic checks
        assert pin_kine.pin_model.nq > 0, "Robot should have at least 1 DOF"
        assert pin_kine.getNumSites() == 1, "Should have 1 site"
        assert pin_kine.getSiteName(0) == ee_site_name
        assert hasattr(pin_kine, 'pin_model')
        assert hasattr(pin_kine, 'pin_data')

    def test_fk_comparison(self, robot_config, test_configurations, tolerance):
        """
        Test that Pinocchio FK matches MuJoCo FK.
        Core validation - ensures Pinocchio computes same pose as MuJoCo.
        """
        robot_name = robot_config['name']
        pino_mjcf = robot_config['pino_mjcf']
        ee_site_name = robot_config['ee_site_name']

        # Load models
        pin_kine = PinKine(pino_mjcf, site_names=[ee_site_name])
        mj_model = mujoco.MjModel.from_xml_path(pino_mjcf)
        mj_data = mujoco.MjData(mj_model)

        # Get test configurations for this robot
        configs = test_configurations[robot_name]

        for config_idx, q_test in enumerate(configs):
            # Update MuJoCo
            mj_data.qpos[:] = q_test
            mujoco.mj_fwdPosition(mj_model, mj_data)

            # Update Pinocchio
            pin_kine.update(q_test)
            pin_pos, pin_quat = pin_kine.getEEPose()

            # Get MuJoCo results
            mj_pos = mj_data.site(ee_site_name).xpos.copy()
            mj_ori = mj_data.site(ee_site_name).xmat.copy()
            mj_quat_wxyz = np.zeros(4)
            mujoco.mju_mat2Quat(mj_quat_wxyz, mj_ori)

            # Convert MuJoCo quat from (w, x, y, z) to (x, y, z, w) for comparison
            mj_quat_xyzw = np.array([mj_quat_wxyz[1], mj_quat_wxyz[2], mj_quat_wxyz[3], mj_quat_wxyz[0]])

            # Compare
            pos_diff = np.linalg.norm(pin_pos - mj_pos)
            quat_diff = np.linalg.norm(pin_quat - mj_quat_xyzw)

            assert pos_diff < tolerance['position'], \
                f"{robot_name} config {config_idx}: Position error {pos_diff:.6f}m exceeds tolerance"
            assert quat_diff < tolerance['quaternion'], \
                f"{robot_name} config {config_idx}: Quaternion error {quat_diff:.6f} exceeds tolerance"

    def test_zero_configuration(self, robot_config):
        """Test FK at zero configuration (sanity check)"""
        pino_mjcf = robot_config['pino_mjcf']
        ee_site_name = robot_config['ee_site_name']

        pin_kine = PinKine(pino_mjcf, site_names=[ee_site_name])

        # Zero configuration
        q_zero = np.zeros(pin_kine.pin_model.nq)
        pin_kine.update(q_zero)
        pin_pos, pin_quat = pin_kine.getEEPose()

        # Should return valid pose (not NaN)
        assert not np.any(np.isnan(pin_pos)), "Position contains NaN"
        assert not np.any(np.isnan(pin_quat)), "Quaternion contains NaN"

        # Quaternion should be normalized
        quat_norm = np.linalg.norm(pin_quat)
        assert abs(quat_norm - 1.0) < 1e-6, f"Quaternion not normalized: {quat_norm}"


# ========== Multi-Site FK Tests ==========

@pytest.fixture
def multisite_configurations():
    """Multi-site configurations for each robot"""
    return {
        'fr3': {
            'sites': ["shoulderSite", "elbowSite", "wristSite", "EE"],
            'configs': [
                np.array([0, 0, 0, -1.57079, 0, 1.57079, 0.7853]),
                np.array([np.pi*0.25, np.pi*0.25, np.pi*0.25, -np.pi*0.25,
                         np.pi*0.25, np.pi*0.25, np.pi*0.25]),
            ]
        },
        'ur5e': {
            'sites': ["shoulderSite", "elbowSite", "wristSite", "EE"],
            'configs': [
                np.array([0, -1.57079, 0, -1.57079, 0, 0]),
                np.array([0, -1.57079, 1.57079, -3.14, -1.57079, 0]),
            ]
        },
        'piper': {
            'sites': ["shoulderSite", "elbowSite", "wristSite", "EE"],
            'configs': [
                np.array([0, 0, 0, 0, 0, 0]),
                np.array([0.5, 1.5, -1.0, 0.5, -0.6, 1.0]),
            ]
        },
        'rm75': {
            'sites': ["shoulderSite", "elbowSite", "wristSite", "EE"],
            'configs': [
                np.array([0, 0, 0, 0, 0, 0, 0]),
                np.array([0.735522, 0.388828, -0.9801, 1.74523, -0.419516, -0.502513, 0.699497]),
            ]
        },
        'so101': {
            'sites': ["shoulderSite", "elbowSite", "wristSite", "EE", "camera_wrist", "camera_front"],
            'configs': [
                np.array([0, 0, 0, 0, 0]),
                np.array([0, 0, 0, 1.57079, -1.57079]),
                np.array([-0.0006442706799134612, -1.743825912475586, 1.7453292608261108,
                         0.8737468719482422, 0.08311091363430023]),
            ]
        },
    }


class TestMultiSiteFK:
    """Test multi-site forward kinematics (shoulder, elbow, wrist, EE)"""

    def test_multisite_initialization(self, robot_config, multisite_configurations):
        """Test that PinKine initializes with multiple sites"""
        robot_name = robot_config['name']
        pino_mjcf = robot_config['pino_mjcf']

        if robot_name not in multisite_configurations:
            pytest.skip(f"No multi-site config for {robot_name}")

        site_names = multisite_configurations[robot_name]['sites']

        # Should not raise
        pin_kine = PinKine(pino_mjcf, site_names=site_names)

        # Check number of sites
        assert pin_kine.getNumSites() == len(site_names)

        # Check site names
        for idx, expected_name in enumerate(site_names):
            assert pin_kine.getSiteName(idx) == expected_name

    def test_multisite_fk_comparison(self, robot_config, multisite_configurations, tolerance):
        """Test that multi-site FK matches between Pinocchio and MuJoCo"""
        robot_name = robot_config['name']
        pino_mjcf = robot_config['pino_mjcf']

        if robot_name not in multisite_configurations:
            pytest.skip(f"No multi-site config for {robot_name}")

        config_data = multisite_configurations[robot_name]
        site_names = config_data['sites']
        test_configs = config_data['configs']

        # Load models
        pin_kine = PinKine(pino_mjcf, site_names=site_names)
        mj_model = mujoco.MjModel.from_xml_path(pino_mjcf)
        mj_data = mujoco.MjData(mj_model)

        for config_idx, q_test in enumerate(test_configs):
            # Update models
            mj_data.qpos[:] = q_test
            mujoco.mj_fwdPosition(mj_model, mj_data)
            pin_kine.update(q_test)

            # Test each site
            for site_idx, site_name in enumerate(site_names):
                # Get Pinocchio results
                pin_pos = pin_kine.getSitePos(site_idx)
                pin_quat = pin_kine.getSiteQuat(site_idx)

                # Get MuJoCo results
                mj_pos = mj_data.site(site_name).xpos.copy()
                mj_ori = mj_data.site(site_name).xmat.copy()
                mj_quat_wxyz = np.zeros(4)
                mujoco.mju_mat2Quat(mj_quat_wxyz, mj_ori)

                # Convert MuJoCo quat from (w, x, y, z) to (x, y, z, w) for comparison
                mj_quat_xyzw = np.array([mj_quat_wxyz[1], mj_quat_wxyz[2], mj_quat_wxyz[3], mj_quat_wxyz[0]])

                # Compare
                pos_diff = np.linalg.norm(pin_pos - mj_pos)
                quat_diff = np.linalg.norm(pin_quat - mj_quat_xyzw)

                assert pos_diff < tolerance['position'], \
                    f"{robot_name} config {config_idx} site {site_name}: " \
                    f"Position error {pos_diff:.6f}m exceeds tolerance"
                assert quat_diff < tolerance['quaternion'], \
                    f"{robot_name} config {config_idx} site {site_name}: " \
                    f"Quaternion error {quat_diff:.6f} exceeds tolerance"


# ========== DOF Tests ==========

@pytest.mark.parametrize("robot_name,expected_dof", [
    ("fr3", 7),
    ("ur5e", 6),
    ("piper", 6),
    ("rm75", 7),
    ("so101", 5),
])
def test_robot_dof_count(robot_name, expected_dof):
    """Test that robots have expected DOF counts"""
    robot_manager = PRobotManager(robot_name)
    pino_mjcf = robot_manager.getPinoMJCF()

    pin_kine = PinKine(pino_mjcf, site_names=["EE"])

    assert pin_kine.pin_model.nq == expected_dof, \
        f"Robot {robot_name} should have {expected_dof} DOF, got {pin_kine.pin_model.nq}"


# ========== Utility for manual testing ==========

if __name__ == "__main__":
    """Run with: python -m pytest tests/test_pinkine_fk.py -v"""
    pytest.main([__file__, "-v", "-s"])
