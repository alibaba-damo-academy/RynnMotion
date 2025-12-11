import numpy as np
import sys
import time
import argparse
import mujoco
import mujoco.viewer
from pathlib import Path
from RynnMotion.algorithms.pin_kine import PinKine

# Get fixtures directory path
FIXTURES_DIR = Path(__file__).parent / "fixtures"


def run_EEPose_Comparison(model_path, ee_site_name, test_configs, robot_name):
    """
    Run forward kinematics comparison test (Pinocchio vs MuJoCo) without viewer.

    Args:
        model_path: Path to the robot MJCF/XML model
        ee_site_name: Name of the end-effector site (string or list)
        test_configs: List of joint configuration arrays to test
        robot_name: Name of the robot for display

    Returns:
        bool: True if all tests pass, False otherwise
    """
    print(f"Testing {robot_name} Forward Kinematics")
    print("=" * 50)

    try:
        # Convert site name to list if it's a string
        site_names_list = [ee_site_name] if isinstance(ee_site_name, str) else ee_site_name
        pinKine = PinKine(model_path, site_names=site_names_list)
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)

        print(f"✓ Successfully loaded model: {model_path}")
        print(f"✓ End-effector site: {ee_site_name}")
        print(f"✓ Number of DOF: {pinKine.pin_model.nq}")
        print()

        all_passed = True

        for i, q_test in enumerate(test_configs):
            print(f"\n--- Test Configuration {i+1} ---")
            print(f"Joint angles (rad): {np.round(q_test, 5)}")

            # Set MuJoCo configuration
            data.qpos[:] = q_test
            mujoco.mj_fwdPosition(model, data)

            # Update Pinocchio kinematics
            pinKine.update(q_test)
            pin_pos, pin_quat = pinKine.getEEPose()

            # Get MuJoCo end-effector pose
            mujoco_pos = data.site(ee_site_name).xpos
            mujoco_ori = data.site(ee_site_name).xmat
            mujoco_quat_wxyz = np.zeros(4)
            mujoco.mju_mat2Quat(mujoco_quat_wxyz, mujoco_ori)
            mujoco_quat = np.array(
                [
                    mujoco_quat_wxyz[1],
                    mujoco_quat_wxyz[2],
                    mujoco_quat_wxyz[3],
                    mujoco_quat_wxyz[0],
                ]
            )

            # Round for cleaner display
            pin_pos_rounded = np.round(pin_pos, 4)
            pin_quat_rounded = np.round(pin_quat, 4)
            mujoco_pos_rounded = np.round(mujoco_pos, 4)
            mujoco_quat_rounded = np.round(mujoco_quat, 4)

            print(f"[Pinocchio] Position: {pin_pos_rounded}")
            print(f"[MuJoCo]    Position: {mujoco_pos_rounded}")
            print(f"[Pinocchio] Quaternion: {pin_quat_rounded}")
            print(f"[MuJoCo]    Quaternion: {mujoco_quat_rounded}")

            # Check if results are close
            pos_diff = np.linalg.norm(pin_pos - mujoco_pos)
            quat_diff = np.linalg.norm(pin_quat - mujoco_quat)

            if pos_diff < 0.001 and quat_diff < 0.001:
                print(f"✓ PASS - Position diff: {pos_diff:.6f}, Quaternion diff: {quat_diff:.6f}")
            else:
                print(f"❌ FAIL - Position diff: {pos_diff:.6f}, Quaternion diff: {quat_diff:.6f}")
                all_passed = False

        return all_passed

    except Exception as e:
        print(f"❌ ERROR: {str(e)}")
        import traceback

        traceback.print_exc()
        return False


def run_viewer_test(model_path, ee_site_name, robot_name):
    """
    Run interactive forward kinematics test with MuJoCo viewer.

    Args:
        model_path: Path to the robot MJCF/XML model
        ee_site_name: Name of the end-effector site (string or list)
        robot_name: Name of the robot for display

    Returns:
        bool: True if test completes successfully, False otherwise
    """
    print(f"Testing {robot_name} Forward Kinematics (Interactive Viewer)")
    print("=" * 50)

    try:
        # Convert site name to list if it's a string
        site_names_list = [ee_site_name] if isinstance(ee_site_name, str) else ee_site_name
        pinKine = PinKine(model_path, site_names=site_names_list)
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)

        print(f"✓ Successfully loaded model: {model_path}")
        print(f"✓ End-effector site: {ee_site_name}")
        print(f"✓ Number of DOF: {pinKine.pin_model.nq}")
        print("\n🎮 Launching interactive viewer... (Ctrl+C to exit)")
        print()

        with mujoco.viewer.launch_passive(model, data) as viewer:
            try:
                current_time = time.time()
                while viewer.is_running():
                    mujoco.mj_step(model, data)
                    current_q = data.qpos[:]
                    pinKine.update(current_q)

                    if time.time() - current_time > 1.0:
                        current_time = time.time()

                        pin_pos, pin_quat = pinKine.getEEPose()

                        mujoco_pos = data.site(pinKine.ee_site_name).xpos
                        mujoco_ori = data.site(pinKine.ee_site_name).xmat
                        mujoco_quat_wxyz = np.zeros(4)
                        mujoco.mju_mat2Quat(mujoco_quat_wxyz, mujoco_ori)
                        mujoco_quat = np.array(
                            [
                                mujoco_quat_wxyz[1],
                                mujoco_quat_wxyz[2],
                                mujoco_quat_wxyz[3],
                                mujoco_quat_wxyz[0],
                            ]
                        )

                        pin_pos_rounded = np.round(pin_pos, 3)
                        pin_quat_rounded = np.round(pin_quat, 3)
                        mujoco_pos_rounded = np.round(mujoco_pos, 3)
                        mujoco_quat_rounded = np.round(mujoco_quat, 3)

                        print(f"\n--- Pose at q = {np.round(current_q, 3)} (rad) ---")
                        print(f"[Pinocchio] Position: {pin_pos_rounded}")
                        print(f"[MuJoCo]    Position: {mujoco_pos_rounded}")
                        print(f"[Pinocchio] Quaternion: {pin_quat_rounded}")
                        print(f"[MuJoCo]    Quaternion: {mujoco_quat_rounded}")

                    viewer.sync()
            except KeyboardInterrupt:
                print("\nReceived keyboard interrupt, shutting down...")
                viewer.close()

        return True

    except Exception as e:
        print(f"❌ ERROR: {str(e)}")
        import traceback

        traceback.print_exc()
        return False


def test_fr3_forward_kinematics(render_mode=False):
    """Test forward kinematics for FR3 robot"""
    model_path = str(FIXTURES_DIR / "fr3_pinocchio.xml")
    ee_site_name = "EE"
    robot_name = "FR3"

    if render_mode:
        result = run_viewer_test(model_path, ee_site_name, robot_name)
        assert result, f"{robot_name} viewer test failed"
    else:
        test_configs = [
            # FR3 Pinocchio has 7 DOF (no gripper)
            np.array(
                [
                    np.pi * 0.25,
                    np.pi * 0.25,
                    np.pi * 0.25,
                    -np.pi * 0.25,
                    np.pi * 0.25,
                    np.pi * 0.25,
                    np.pi * 0.25,
                ]
            ),
        ]
        result = run_EEPose_Comparison(model_path, ee_site_name, test_configs, robot_name)
        assert result, f"{robot_name} FK comparison test failed"


def test_ur5e_forward_kinematics(render_mode=False):
    """Test forward kinematics for UR5e robot"""
    model_path = str(FIXTURES_DIR / "ur5e_pinocchio.xml")
    ee_site_name = "EE"
    robot_name = "UR5e"

    if render_mode:
        result = run_viewer_test(model_path, ee_site_name, robot_name)
        assert result, f"{robot_name} viewer test failed"
    else:
        test_configs = [
            np.array([0, -1.57079, 0, -1.57079, 0, 0]),  # home pose
            np.array([0, -1.57079, 1.57079, -1.57079, -1.57079, 3.1415926]),
        ]
        result = run_EEPose_Comparison(model_path, ee_site_name, test_configs, robot_name)
        assert result, f"{robot_name} FK comparison test failed"


def test_piper_forward_kinematics(render_mode=False):
    """Test forward kinematics for Piper robot"""
    model_path = str(FIXTURES_DIR / "piper_pinocchio.xml")
    ee_site_name = "EE"
    robot_name = "Piper"

    if render_mode:
        result = run_viewer_test(model_path, ee_site_name, robot_name)
        assert result, f"{robot_name} viewer test failed"
    else:
        test_configs = [
            np.array([0, 0, 0, 0, 0, 0]),  # home pose
        ]
        result = run_EEPose_Comparison(model_path, ee_site_name, test_configs, robot_name)
        assert result, f"{robot_name} FK comparison test failed"


def test_rm75_forward_kinematics(render_mode=False):
    """Test forward kinematics for RM75 robot"""
    model_path = str(FIXTURES_DIR / "rm75_pinocchio.xml")
    ee_site_name = "EE"
    robot_name = "RM75"

    if render_mode:
        result = run_viewer_test(model_path, ee_site_name, robot_name)
        assert result, f"{robot_name} viewer test failed"
    else:
        test_configs = [
            np.array([0, 0, 0, 0, 0, 0, 0]),  # home pose
            np.array([0.735522, 0.388828, -0.9801, 1.74523, -0.419516, -0.502513, 0.699497]),  # stand pose
            np.array([0.5, -0.3, 0.8, -1.2, 0.6, -0.8, 1.0]),  # custom pose
        ]
        result = run_EEPose_Comparison(model_path, ee_site_name, test_configs, robot_name)
        assert result, f"{robot_name} FK comparison test failed"


def test_lerobot_forward_kinematics(render_mode=False):
    """Test forward kinematics for lerobot"""
    model_path = str(FIXTURES_DIR / "so101_pinocchio.xml")
    ee_site_name = "EE"
    robot_name = "LeRobot"

    if render_mode:
        result = run_viewer_test(model_path, ee_site_name, robot_name)
        assert result, f"{robot_name} viewer test failed"
    else:
        test_configs = [
            np.array([0, 0, 0, 0, 0]),  # home pose
            np.array([0, 0, 0, 1.57079, -1.57079]),  # stand pose
        ]
        result = run_EEPose_Comparison(model_path, ee_site_name, test_configs, robot_name)
        assert result, f"{robot_name} FK comparison test failed"


def test_dual_fr3_forward_kinematics(render_mode=False):
    """Test forward kinematics for Dual FR3 robot (left arm)"""
    model_path = str(FIXTURES_DIR / "dual_fr3_pinocchio.xml")
    ee_site_name = "EE_left"
    robot_name = "Dual FR3"

    if render_mode:
        result = run_viewer_test(model_path, ee_site_name, robot_name)
        assert result, f"{robot_name} viewer test failed"
    else:
        test_configs = [
            # Dual FR3 has 14 DOF (7 left + 7 right)
            # Home: all zeros
            np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
            # Standby: both arms at standby1 position
            np.array([0, 0, 0, -1.57079, 0, 1.57079, 0.7853,
                     0, 0, 0, -1.57079, 0, 1.57079, 0.7853]),
        ]
        result = run_EEPose_Comparison(model_path, ee_site_name, test_configs, robot_name)
        assert result, f"{robot_name} FK comparison test failed"
        return True


# ========== Multi-Site Tests ==========


def run_multisite_comparison(model_path, site_names, test_configs, robot_name):
    """
    Run multi-site forward kinematics comparison test.

    Args:
        model_path: Path to the robot MJCF/XML model
        site_names: List of site names to test
        test_configs: List of joint configuration arrays to test
        robot_name: Name of the robot for display

    Returns:
        bool: True if all tests pass, False otherwise
    """
    print(f"Testing {robot_name} Multi-Site Forward Kinematics")
    print("=" * 50)

    try:
        pinKine = PinKine(model_path, site_names=site_names)
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)

        print(f"✓ Successfully loaded model: {model_path}")
        print(f"✓ Number of sites: {pinKine.getNumSites()}")
        for i in range(pinKine.getNumSites()):
            print(f"  Site {i}: {pinKine.getSiteName(i)}")
        print(f"✓ Number of DOF: {pinKine.pin_model.nq}")
        print()

        all_passed = True

        for config_idx, q_test in enumerate(test_configs):
            print(f"\n{'='*60}")
            print(f"Test Configuration {config_idx+1}")
            print(f"{'='*60}")
            print(f"Joint angles (rad): {np.round(q_test, 5)}")
            print()

            # Set MuJoCo configuration
            data.qpos[:] = q_test
            mujoco.mj_fwdPosition(model, data)

            # Update Pinocchio kinematics
            pinKine.update(q_test)

            # Test each site
            for site_idx, site_name in enumerate(site_names):
                print(f"\n--- Site: {site_name} ---")

                # Get Pinocchio results
                pin_pos = pinKine.getSitePos(site_idx)
                pin_quat = pinKine.getSiteQuat(site_idx)

                # Get MuJoCo results
                mujoco_pos = data.site(site_name).xpos
                mujoco_ori = data.site(site_name).xmat
                mujoco_quat_wxyz = np.zeros(4)
                mujoco.mju_mat2Quat(mujoco_quat_wxyz, mujoco_ori)
                mujoco_quat = np.array(
                    [
                        mujoco_quat_wxyz[1],
                        mujoco_quat_wxyz[2],
                        mujoco_quat_wxyz[3],
                        mujoco_quat_wxyz[0],
                    ]
                )

                # Round for cleaner display
                pin_pos_rounded = np.round(pin_pos, 4)
                pin_quat_rounded = np.round(pin_quat, 4)
                mujoco_pos_rounded = np.round(mujoco_pos, 4)
                mujoco_quat_rounded = np.round(mujoco_quat, 4)

                print(f"[Pinocchio] Position: {pin_pos_rounded}")
                print(f"[MuJoCo]    Position: {mujoco_pos_rounded}")
                print(f"[Pinocchio] Quaternion: {pin_quat_rounded}")
                print(f"[MuJoCo]    Quaternion: {mujoco_quat_rounded}")

                # Check if results are close
                pos_diff = np.linalg.norm(pin_pos - mujoco_pos)
                quat_diff = np.linalg.norm(pin_quat - mujoco_quat)

                if pos_diff < 0.001 and quat_diff < 0.001:
                    print(f"✓ PASS - Position diff: {pos_diff:.6f}, Quaternion diff: {quat_diff:.6f}")
                else:
                    print(f"❌ FAIL - Position diff: {pos_diff:.6f}, Quaternion diff: {quat_diff:.6f}")
                    all_passed = False

        return all_passed

    except Exception as e:
        print(f"❌ ERROR: {str(e)}")
        import traceback

        traceback.print_exc()
        return False


def run_multisite_viewer(model_path, site_names, robot_name):
    """
    Run interactive multi-site forward kinematics test with MuJoCo viewer.

    Args:
        model_path: Path to the robot MJCF/XML model
        site_names: List of site names to test
        robot_name: Name of the robot for display

    Returns:
        bool: True if test completes successfully, False otherwise
    """
    print(f"Testing {robot_name} Multi-Site Forward Kinematics (Interactive Viewer)")
    print("=" * 50)

    try:
        pinKine = PinKine(model_path, site_names=site_names)
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)

        print(f"✓ Successfully loaded model: {model_path}")
        print(f"✓ Number of sites: {pinKine.getNumSites()}")
        for i in range(pinKine.getNumSites()):
            print(f"  Site {i}: {pinKine.getSiteName(i)}")
        print(f"✓ Number of DOF: {pinKine.pin_model.nq}")
        print("\n🎮 Launching interactive viewer... (Ctrl+C to exit)")
        print()

        with mujoco.viewer.launch_passive(model, data) as viewer:
            try:
                current_time = time.time()
                while viewer.is_running():
                    mujoco.mj_step(model, data)
                    current_q = data.qpos[:]
                    pinKine.update(current_q)

                    if time.time() - current_time > 1.0:
                        current_time = time.time()

                        print(f"\n{'='*60}")
                        print(f"Pose at q = {np.round(current_q, 3)} (rad)")
                        print(f"{'='*60}")

                        for site_idx, site_name in enumerate(site_names):
                            pin_pos = pinKine.getSitePos(site_idx)
                            pin_quat = pinKine.getSiteQuat(site_idx)

                            mujoco_pos = data.site(site_name).xpos
                            mujoco_ori = data.site(site_name).xmat
                            mujoco_quat_wxyz = np.zeros(4)
                            mujoco.mju_mat2Quat(mujoco_quat_wxyz, mujoco_ori)
                            mujoco_quat = np.array(
                                [
                                    mujoco_quat_wxyz[1],
                                    mujoco_quat_wxyz[2],
                                    mujoco_quat_wxyz[3],
                                    mujoco_quat_wxyz[0],
                                ]
                            )

                            pin_pos_rounded = np.round(pin_pos, 3)
                            mujoco_pos_rounded = np.round(mujoco_pos, 3)

                            print(f"\n--- {site_name} ---")
                            print(f"[Pinocchio] Position: {pin_pos_rounded}")
                            print(f"[MuJoCo]    Position: {mujoco_pos_rounded}")

                    viewer.sync()
            except KeyboardInterrupt:
                print("\nReceived keyboard interrupt, shutting down...")
                viewer.close()

        return True

    except Exception as e:
        print(f"❌ ERROR: {str(e)}")
        import traceback

        traceback.print_exc()
        return False


def test_fr3_multisite(render_mode=False):
    """Test multi-site forward kinematics for FR3 robot"""
    model_path = str(FIXTURES_DIR / "fr3_pinocchio.xml")
    site_names = ["shoulderSite", "elbowSite", "wristSite", "EE"]
    robot_name = "FR3 (Multi-Site)"

    if render_mode:
        result = run_multisite_viewer(model_path, site_names, robot_name)
        assert result, f"{robot_name} viewer test failed"
    else:
        test_configs = [
            np.array([0, 0, 0, -1.57079, 0, 1.57079, 0.7853]),  # stand pose (7 DOF only)
            np.array(
                [
                    np.pi * 0.25,
                    np.pi * 0.25,
                    np.pi * 0.25,
                    -np.pi * 0.25,
                    np.pi * 0.25,
                    np.pi * 0.25,
                    np.pi * 0.25,
                ]
            ),
        ]
        result = run_multisite_comparison(model_path, site_names, test_configs, robot_name)
        assert result, f"{robot_name} multi-site FK comparison test failed"


def test_ur5e_multisite(render_mode=False):
    """Test multi-site forward kinematics for UR5e robot"""
    model_path = str(FIXTURES_DIR / "ur5e_pinocchio.xml")
    site_names = ["shoulderSite", "elbowSite", "wristSite", "EE"]
    robot_name = "UR5e (Multi-Site)"

    if render_mode:
        result = run_multisite_viewer(model_path, site_names, robot_name)
        assert result, f"{robot_name} viewer test failed"
    else:
        test_configs = [
            np.array([0, -1.57079, 0, -1.57079, 0, 0]),  # home pose
            np.array([0, -1.57079, 1.57079, -1.57079, -1.57079, 3.1415926]),
        ]
        result = run_multisite_comparison(model_path, site_names, test_configs, robot_name)
        assert result, f"{robot_name} multi-site FK comparison test failed"


def test_piper_multisite(render_mode=False):
    """Test multi-site forward kinematics for Piper robot"""
    model_path = str(FIXTURES_DIR / "piper_pinocchio.xml")
    site_names = ["shoulderSite", "elbowSite", "wristSite", "EE"]
    robot_name = "Piper (Multi-Site)"

    if render_mode:
        result = run_multisite_viewer(model_path, site_names, robot_name)
        assert result, f"{robot_name} viewer test failed"
    else:
        test_configs = [
            np.array([0, 0, 0, 0, 0, 0]),  # home pose
            np.array([0.5, 1.5, -1.0, 0.5, -0.6, 1.0]),  # custom pose
        ]
        result = run_multisite_comparison(model_path, site_names, test_configs, robot_name)
        assert result, f"{robot_name} multi-site FK comparison test failed"


def test_rm75_multisite(render_mode=False):
    """Test multi-site forward kinematics for RM75 robot"""
    model_path = str(FIXTURES_DIR / "rm75_pinocchio.xml")
    site_names = ["shoulderSite", "elbowSite", "wristSite", "EE"]
    robot_name = "RM75 (Multi-Site)"

    if render_mode:
        result = run_multisite_viewer(model_path, site_names, robot_name)
        assert result, f"{robot_name} viewer test failed"
    else:
        test_configs = [
            np.array([0, 0, 0, 0, 0, 0, 0]),  # home pose
            np.array([0.735522, 0.388828, -0.9801, 1.74523, -0.419516, -0.502513, 0.699497]),  # stand pose
        ]
        result = run_multisite_comparison(model_path, site_names, test_configs, robot_name)
        assert result, f"{robot_name} multi-site FK comparison test failed"


def test_so101_multisite(render_mode=False):
    """Test multi-site forward kinematics for SO101 robot"""
    model_path = str(FIXTURES_DIR / "so101_pinocchio.xml")
    site_names = [
        "shoulderSite",
        "elbowSite",
        "wristSite",
        "EE",
        "camera_wrist",
        "camera_front",
    ]
    robot_name = "SO101 (Multi-Site)"

    if render_mode:
        result = run_multisite_viewer(model_path, site_names, robot_name)
        assert result, f"{robot_name} viewer test failed"
    else:
        test_configs = [
            np.array([0, 0, 0, 0, 0]),  # home pose
            np.array([0, 0, 0, 1.57079, -1.57079]),  # stand pose
        ]
        result = run_multisite_comparison(model_path, site_names, test_configs, robot_name)
        assert result, f"{robot_name} multi-site FK comparison test failed"
    return result


def test_dual_fr3_multisite(render_mode=False):
    """Test multi-site forward kinematics for Dual FR3 robot"""
    model_path = str(FIXTURES_DIR / "dual_fr3_pinocchio.xml")
    site_names = ["EE_left", "EE_right"]
    robot_name = "Dual FR3 (Multi-Site)"

    if render_mode:
        result = run_multisite_viewer(model_path, site_names, robot_name)
        assert result, f"{robot_name} viewer test failed"
    else:
        test_configs = [
            # Dual FR3 has 14 DOF (7 left + 7 right)
            np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]),  # home pose
            np.array([0, 0, 0, -1.57079, 0, 1.57079, 0.7853,
                     0, 0, 0, -1.57079, 0, 1.57079, 0.7853]),  # standby pose
        ]
        result = run_multisite_comparison(model_path, site_names, test_configs, robot_name)
        assert result, f"{robot_name} multi-site FK comparison test failed"
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Robot Kinematics Test Suite")
    parser.add_argument(
        "-v",
        "--viewer",
        "--view",
        action="store_true",
        help="Launch interactive MuJoCo viewer",
    )
    parser.add_argument(
        "-r",
        "--robot",
        type=int,
        default=1,
        choices=[1, 2, 3, 4, 5, 6],
        help="Select robot: 1=FR3, 2=UR5e, 3=Piper, 4=RM75, 5=SO101, 6=Dual_FR3 (default: 1)",
    )
    parser.add_argument(
        "-m",
        "--multisite",
        action="store_true",
        help="Test multi-site kinematics (shoulder, elbow, wrist, EE)",
    )
    args = parser.parse_args()

    print("Robot Kinematics Test Suite")
    print("=" * 60)

    # Single-site tests (legacy)
    single_site_tests = {
        1: ("FR3", test_fr3_forward_kinematics),
        2: ("UR5e", test_ur5e_forward_kinematics),
        3: ("Piper", test_piper_forward_kinematics),
        4: ("RM75", test_rm75_forward_kinematics),
        5: ("SO101", test_lerobot_forward_kinematics),
        6: ("Dual_FR3", test_dual_fr3_forward_kinematics),
    }

    # Multi-site tests (new)
    multi_site_tests = {
        1: ("FR3", test_fr3_multisite),
        2: ("UR5e", test_ur5e_multisite),
        3: ("Piper", test_piper_multisite),
        4: ("RM75", test_rm75_multisite),
        5: ("SO101", test_so101_multisite),
        6: ("Dual_FR3", test_dual_fr3_multisite),
    }

    # Select test mode
    if args.multisite:
        robot_tests = multi_site_tests
        test_mode = "Multi-Site"
    else:
        robot_tests = single_site_tests
        test_mode = "Single-Site"

    robot_name, test_func = robot_tests[args.robot]

    if args.viewer:
        print(f"🎮 Viewer mode enabled - launching interactive viewer for {robot_name}")
    else:
        print(f"📊 Comparison mode - running automated test for {robot_name}")

    print(f"📍 Test mode: {test_mode}")
    print()

    print(f"\n🤖 Testing {robot_name} Robot...")
    success = test_func(render_mode=args.viewer)

    print("\n" + "=" * 60)
    if success:
        print("🎉 TEST PASSED!")
    else:
        print("❌ TEST FAILED!")
        sys.exit(1)
