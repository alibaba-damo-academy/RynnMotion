#!/usr/bin/env python3
"""SO101 floating end-effector teleoperation.

Controls floating grippers in MuJoCo using SO101 master arm EE pose.

Usage:
    $ so101-floating-ee --gripper robotiq
    $ so101-floating-ee --gripper umi
    $ so101-floating-ee --gripper openarm
"""

import argparse
import mujoco
import numpy as np
import os
import platform
import time
import yaml
from scipy.spatial.transform import Rotation

from RynnMotion.utils.path_config import get_models_root
from RynnMotion.algorithms import PinKine
from RynnLeRobot.interface.so101_interface import create_robot_interface


# Gripper configuration
GRIPPER_CONFIG = {
    "robotiq": {
        "scene_path": "2.end_effector/11.robotiq/scene/scene.xml",
        "actuator_prefix": "2f85",
        "gripper_actuator": "gripper",
    },
    "umi": {
        "scene_path": "2.end_effector/12.umi/scene/scene.xml",
        "actuator_prefix": "umi",
        "gripper_actuator": "gripper",
    },
    "openarm": {
        "scene_path": "2.end_effector/13.openarm_gripper/scene/scene.xml",
        "actuator_prefix": "gripper",
        "gripper_actuator": "gripper",
    },
}


class SO101FloatingEE:
    """Control floating grippers with SO101 master arm EE pose."""

    def __init__(self, gripper_type="robotiq", use_viewer=None, config_path="configs/so101.yaml"):
        """
        Initialize SO101 floating EE teleoperation.

        Args:
            gripper_type: Gripper type ("robotiq", "umi", or "openarm")
            use_viewer: If None, auto-detect platform. If True, use viewer mode.
            config_path: Path to so101 configuration file
        """
        if gripper_type not in GRIPPER_CONFIG:
            raise ValueError(f"Unknown gripper type: {gripper_type}. "
                           f"Supported: {list(GRIPPER_CONFIG.keys())}")

        self.gripper_type = gripper_type
        self.gripper_config = GRIPPER_CONFIG[gripper_type]
        self.config_path = config_path

        # Determine rendering mode
        if use_viewer is None:
            self.use_viewer = platform.system() == "Darwin"
        else:
            self.use_viewer = use_viewer

        print(f"Initializing SO101 Floating EE with {gripper_type} gripper")

        # Control parameters
        self.control_freq = 100.0
        self.control_dt = 1.0 / self.control_freq

        # MuJoCo state
        self.model = None
        self.data = None

        # Actuator indices (will be populated after scene load)
        self.actuator_indices = {}  # arm_idx -> {pos, rot, gripper}

        # Initialize system
        self._init_scene()
        self._get_master_info()
        self._init_master_interface()
        self._init_kinematics()
        self._get_actuator_indices()
        self._init_ee_home_poses()

        # Control timing (needed for both viewer and GLFW modes)
        self.last_control_time = time.time()

    def _init_scene(self):
        """Initialize MuJoCo scene."""
        models_root = get_models_root()
        scene_mjcf = str(models_root / self.gripper_config["scene_path"])

        if not os.path.exists(scene_mjcf):
            raise FileNotFoundError(f"Scene file not found: {scene_mjcf}")

        print(f"Loading scene from: {scene_mjcf}")

        self.model = mujoco.MjModel.from_xml_path(scene_mjcf)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        print(f"Scene loaded: {self.model.nu} actuators, {self.model.nq} qpos")

    def _get_master_info(self):
        """Load master robot configuration from YAML."""
        try:
            with open(self.config_path, "r") as f:
                self.master_config = yaml.safe_load(f)
        except Exception as e:
            print(f"Warning: Could not load {self.config_path}: {e}")
            print("Using default port settings")
            self.master_config = {}

        teleop_config = self.master_config.get("leader", {})
        self.master_port = teleop_config.get("port", "/dev/ttyACM1")
        self.master_port_2 = teleop_config.get("port_2", "/dev/ttyACM2")
        self.master_calibration_dir = teleop_config.get("calibration_dir")
        self.master_calibration_dir_2 = teleop_config.get("calibration_dir_2")

    def _init_master_interface(self):
        """Initialize SO101 master robot interfaces."""
        # Master arm 1
        self.master_interface = create_robot_interface(
            name="so101",
            mode="teleop",
            config_path=self.config_path,
            port=self.master_port,
            calibration_dir=self.master_calibration_dir,
        )
        self.master_interface.init()
        self.master_interface.disable_robot_torque()
        print(f"Master arm 1 initialized on {self.master_port}")

        # Master arm 2
        self.master2_interface = create_robot_interface(
            name="so101",
            mode="teleop",
            config_path=self.config_path,
            port=self.master_port_2,
            calibration_dir=self.master_calibration_dir_2,
        )
        self.master2_interface.init()
        self.master2_interface.disable_robot_torque()
        print(f"Master arm 2 initialized on {self.master_port_2}")

    def _init_kinematics(self):
        """Initialize PinKine for SO101 FK computation."""
        models_root = get_models_root()
        so101_pin_mjcf = str(models_root / "3.robot_arm/24.so101/mjcf/so101_pinocchio.xml")

        self.site_names = ["shoulderSite", "elbowSite", "wristSite", "EE"]

        self.master_pin_kine = PinKine(so101_pin_mjcf, self.site_names)
        self.master2_pin_kine = PinKine(so101_pin_mjcf, self.site_names)

        print("SO101 kinematics initialized")

    def _get_actuator_indices(self):
        """Get actuator indices for each gripper."""
        prefix = self.gripper_config["actuator_prefix"]

        # For replicate count=2, actuators are named with _0 and _1 suffix
        for arm_idx in range(2):
            suffix = f"_{arm_idx}"

            pos_actuators = []
            rot_actuators = []
            gripper_actuator = None

            for i in range(self.model.nu):
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                if name is None:
                    continue

                # Check for position actuators (qx, qy, qz)
                if name == f"{prefix}_qx{suffix}":
                    pos_actuators.append(("x", i))
                elif name == f"{prefix}_qy{suffix}":
                    pos_actuators.append(("y", i))
                elif name == f"{prefix}_qz{suffix}":
                    pos_actuators.append(("z", i))
                # Check for rotation actuators (qrx, qry, qrz)
                elif name == f"{prefix}_qrx{suffix}":
                    rot_actuators.append(("rx", i))
                elif name == f"{prefix}_qry{suffix}":
                    rot_actuators.append(("ry", i))
                elif name == f"{prefix}_qrz{suffix}":
                    rot_actuators.append(("rz", i))
                # Check for gripper actuator
                elif name == f"{self.gripper_config['gripper_actuator']}{suffix}":
                    gripper_actuator = i

            # Sort by axis order
            pos_actuators.sort(key=lambda x: x[0])
            rot_actuators.sort(key=lambda x: x[0])

            self.actuator_indices[arm_idx] = {
                "pos": [idx for _, idx in pos_actuators],
                "rot": [idx for _, idx in rot_actuators],
                "gripper": gripper_actuator,
            }

            print(f"Arm {arm_idx} actuators: pos={self.actuator_indices[arm_idx]['pos']}, "
                  f"rot={self.actuator_indices[arm_idx]['rot']}, "
                  f"gripper={self.actuator_indices[arm_idx]['gripper']}")

    def _init_ee_home_poses(self):
        """Capture home EE poses for relative tracking."""
        self.ee_home_pos = {}
        self.ee_home_quat = {}

        # Gripper scene centers (replicate offset="0 -0.3 0" in y)
        self.gripper_center = {
            0: np.array([0.0, 0.0, 0.6]),     # First gripper
            1: np.array([0.0, -0.3, 0.6]),    # Second gripper (offset by -0.3 in y)
        }
        self.position_scale = 1.0  # Can scale motion if needed

        for arm_idx in range(2):
            pos, quat, _ = self._get_ee_pose_raw(arm_idx)
            self.ee_home_pos[arm_idx] = pos.copy()
            self.ee_home_quat[arm_idx] = quat.copy()

        print(f"Home poses captured: arm0={self.ee_home_pos[0]}, arm1={self.ee_home_pos[1]}")

    def _get_ee_pose_raw(self, arm_idx):
        """
        Get raw EE pose from master arm (in SO101 base frame).

        Args:
            arm_idx: 0 for arm 1, 1 for arm 2

        Returns:
            tuple: (position, quaternion, gripper_value)
        """
        if arm_idx == 0:
            interface = self.master_interface
            pin_kine = self.master_pin_kine
        else:
            interface = self.master2_interface
            pin_kine = self.master2_pin_kine

        # Get joint positions from hardware
        full_positions = interface.get_joint_positions()

        # Motion DOF (5 for SO101, excluding gripper)
        motion_dof = pin_kine.pin_model.nq
        motion_positions = full_positions[:motion_dof]
        gripper_position = full_positions[-1]  # Last joint is gripper (0-1)

        # Update FK
        pin_kine.update(motion_positions)

        # Get EE pose in SO101 base frame
        ee_pos = pin_kine.getSitePosByName("EE")
        ee_quat = pin_kine.getSiteQuatByName("EE")  # (x, y, z, w)

        return ee_pos, ee_quat, gripper_position

    def _get_observation_from_master(self, arm_idx):
        """
        Get EE pose mapped to gripper scene coordinates.

        Args:
            arm_idx: 0 for arm 1, 1 for arm 2

        Returns:
            tuple: (position, euler_angles, gripper_value)
        """
        pos, quat, gripper = self._get_ee_pose_raw(arm_idx)

        # Compute delta from home position
        delta_pos = pos - self.ee_home_pos[arm_idx]

        # Compute relative rotation
        home_rot = Rotation.from_quat(self.ee_home_quat[arm_idx])
        curr_rot = Rotation.from_quat(quat)
        rel_rot = curr_rot*home_rot.inv()
        eeRPY = rel_rot.as_euler("XYZ")  # Intrinsic xyz matches hinge joints

        # Map to gripper scene: center + scaled delta
        eePos = self.gripper_center[arm_idx] + delta_pos * self.position_scale

        return eePos, eeRPY, gripper

    def _set_action_floating_ee(self, arm_idx, pos, euler, gripper_val):
        """
        Set floating gripper pose in MuJoCo.

        Args:
            arm_idx: 0 or 1
            pos: (x, y, z) position
            euler: (rx, ry, rz) euler angles in radians
            gripper_val: 0-1 gripper value (0=closed, 1=open)
        """
        indices = self.actuator_indices[arm_idx]

        # Set position actuators
        for i, val in enumerate(pos):
            if i < len(indices["pos"]):
                self.data.ctrl[indices["pos"][i]] = val

        # Set rotation actuators
        for i, val in enumerate(euler):
            if i < len(indices["rot"]):
                self.data.ctrl[indices["rot"][i]] = val

        # Set gripper actuator
        if indices["gripper"] is not None:
            self.data.ctrl[indices["gripper"]] = gripper_val

    def _control_callback(self, model, data):
        """Control callback - read master pose and update floating gripper."""
        current_time = time.time()
        if current_time - self.last_control_time < self.control_dt:
            return

        self.last_control_time = current_time

        # Get EE pose from both master arms and apply to grippers
        for arm_idx in range(2):
            try:
                pos, euler, gripper = self._get_observation_from_master(arm_idx)
                self._set_action_floating_ee(arm_idx, pos, euler, gripper)
            except Exception as e:
                print(f"Error reading arm {arm_idx}: {e}")

    def _run_viewer(self):
        """Main loop for viewer mode (macOS)."""
        try:
            print("=" * 60)
            print(f"SO101 Floating EE - {self.gripper_type.upper()} Gripper")
            print("=" * 60)
            print("Master: Dual SO101 arms")
            print("Slave: Floating grippers in MuJoCo")
            print("Mode: Viewer (macOS)")
            print("=" * 60)
            print("\nControls:")
            print("  ESC - Exit (close window)")
            print("\nTeleoperation starts automatically...")
            print()

            import mujoco.viewer

            with mujoco.viewer.launch_passive(
                self.model,
                self.data,
                show_left_ui=False,
                show_right_ui=True
            ) as viewer:
                # Main control loop
                while viewer.is_running():
                    step_start = time.time()

                    # Call control logic
                    self._control_callback(self.model, self.data)

                    # Step simulation
                    mujoco.mj_step(self.model, self.data)

                    # Sync viewer
                    viewer.sync()

                    # Timing
                    elapsed = time.time() - step_start
                    sleep_time = self.model.opt.timestep - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)

            print("Application shut down successfully")

        except Exception as e:
            print(f"Error in main loop: {e}")
            import traceback
            traceback.print_exc()

    def run(self):
        """Run teleoperation application."""
        if self.use_viewer:
            self._run_viewer()
        else:
            # TODO: Add GLFW mode for Linux if needed
            print("GLFW mode not yet implemented. Using viewer mode.")
            self._run_viewer()


def main():
    """Entry point for so101-floating-ee command."""
    parser = argparse.ArgumentParser(
        description="SO101 Floating End-Effector Teleoperation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  so101-floating-ee --gripper robotiq   # Robotiq 2F85 gripper
  so101-floating-ee --gripper umi       # UMI gripper
  so101-floating-ee --gripper openarm   # OpenArm gripper

Description:
  Control floating grippers in MuJoCo using SO101 master arm EE pose.
  Reads end-effector position and orientation from two SO101 master arms
  and maps them to floating grippers in the simulation.
        """
    )

    parser.add_argument(
        "--gripper",
        type=str,
        default="robotiq",
        choices=["robotiq", "umi", "openarm"],
        help="Gripper type to use. Default: robotiq"
    )

    parser.add_argument(
        "--mode",
        type=str,
        choices=["auto", "viewer"],
        default="auto",
        help="Rendering mode. Default: auto (viewer on macOS)"
    )

    parser.add_argument(
        "--config",
        type=str,
        default="configs/so101.yaml",
        help="Path to so101 config file. Default: configs/so101.yaml"
    )

    args = parser.parse_args()

    # Determine use_viewer from mode
    if args.mode == "auto":
        use_viewer = None  # Auto-detect
    else:
        use_viewer = True

    try:
        app = SO101FloatingEE(
            gripper_type=args.gripper,
            use_viewer=use_viewer,
            config_path=args.config
        )
        app.run()
    except ValueError as e:
        print(f"Error: {e}")
        parser.print_help()
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        return 130

    return 0


if __name__ == "__main__":
    exit(main())
