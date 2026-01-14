#!/usr/bin/env python3
"""Joint teleoperation for LeRobot SO101.

Usage:
    $ joint-teleop
    $ joint-teleop --mode real --ctrlfreq 100
"""

import os
import sys
import argparse
import time
import threading

import logging
import yaml
import numpy as np

from RynnMotion.core.interface_base import robotinterface_factory
from RynnMotion.manager.robot_manager import RobotManager
from RynnMotion.common.data.robot_state import RobotState
from RynnMotion.utils.leader_reader import LeaderDataReader
from RynnLeRobot.interface.so101_interface import create_robot_interface
from RynnMotion.algorithms.pin_kine import PinKine
from RynnMotion.utils.path_config import get_models_root


class TeleOperator:
    """Single leader to single follower teleoperation system.

    Attributes:
        mode: Operation mode ('sim' or 'real').
        frequency: Control loop frequency in Hz.
    """

    def __init__(
        self,
        mode: str = "sim",
        frequency: int = 100,
        config_path: str = "configs/config.yaml",
    ):
        self.mode = mode
        self.frequency = max(30, min(frequency, 250))
        self.timestep = 1.0 / self.frequency
        self.config_path = config_path
        self.config = self._load_config(config_path)

        self._init_logging()

        self.leader_interface = None
        self.follower_interface = None
        self.leader_reader = None

        self.current_joint_positions = None
        self.command_joint_positions = None
        self.data_lock = threading.RLock()
        self.cached_observation = None
        self.cached_action = None
        self.last_observation_time = 0
        self.last_action_time = 0
        self.is_shutting_down = False

        self.leader_pin_kine = None
        self.follower_pin_kine = None

        self.cached_ee_pose_obs = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.cached_ee_pose_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)

        self.cached_camera_poses_obs = {
            'camera_wrist': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32),
            'camera_front': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        }
        self.cached_camera_poses_action = {
            'camera_wrist': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32),
            'camera_front': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        }

        self.robot_model = None
        self._init_robot_model()
        self._init_interfaces()
        self._init_kinematics()

    def _load_config(self, config_path):
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
            return config if config else {}
        except FileNotFoundError:
            return {}

    def _init_logging(self):
        from datetime import datetime

        log_dir = os.path.expanduser(
            self.config.get("logging", {}).get("log_dir", "~/logs/lerobot")
        )
        log_file = self.config.get("logging", {}).get("log_file", "teleop_simulation.log")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        log_path = os.path.join(log_dir, f"{log_file.replace('.log', '')}_{timestamp}.log")

        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s [%(levelname)s] %(message)s",
            handlers=[logging.FileHandler(log_path)],
        )
        self.logger = logging.getLogger(__name__)

    def get_scene_config(self) -> dict:
        return {
            "scene_path": "3.robot_arm/24.so101/scene/scene.xml",
            "camera_distance": 1.0,
            "camera_azimuth": 45.0,
            "camera_elevation": -20.0,
            "camera_lookat": [0.0, -0.2, 0.2],
        }

    def _init_robot_model(self):
        models_root = get_models_root()
        scene_config = self.get_scene_config()

        robotmodel_config = {
            "robot_name": "so101",
            "robot_control_freq": self.frequency,
            "robot_mjcf": str(models_root / scene_config["scene_path"]),
            "pino_mjcf": str(models_root / "3.robot_arm/24.so101/mjcf/so101_pinocchio.xml"),
        }
        self.robot_model = RobotManager(robotmodel_config, self.logger)

    def _init_interfaces(self):
        if self.mode == "sim":
            self.follower_interface = robotinterface_factory(
                "mujoco_sim_robot",
                self.robot_model,
                {"timestep": self.timestep},
            )
        elif self.mode == "real":
            self.follower_interface = create_robot_interface(
                name="lerobot",
                mode="real",
                config_path=self.config_path,
            )

        time.sleep(0.2)  # Avoid port conflicts
        self.leader_interface = create_robot_interface(
            name="lerobot",
            mode="teleop",
            config_path=self.config_path,
        )

        self.follower_interface.connect()
        self.leader_interface.connect()
        self.leader_interface.disable_robot_torque()

        if self.mode == "sim":
            self.follower_interface.set_camera(
                distance=1.0,
                azimuth=45.0,
                elevation=-20.0,
                lookat=[0.0, -0.2, 0.2],
            )

        if hasattr(self.leader_interface, "robot") and hasattr(self.leader_interface.robot, "bus"):
            self.leader_reader = LeaderDataReader(self.leader_interface, target_frequency=200)
        else:
            self.leader_reader = None

        if self.leader_reader is not None:
            time.sleep(0.2)
            self.initial_joint_positions = self.leader_reader.get_latest_data()
            if self.initial_joint_positions is None:
                self.initial_joint_positions = self.leader_interface.get_joint_positions()
        else:
            self.initial_joint_positions = self.leader_interface.get_joint_positions()

    def _init_kinematics(self):
        try:
            mjcf_path = self.robot_model.get_pino_mjcf()
            site_names = ["EE", "camera_wrist", "camera_front"]

            self.leader_pin_kine = PinKine(mjcf_path, site_names=site_names)
            self.follower_pin_kine = PinKine(mjcf_path, site_names=site_names)
        except Exception:
            pass  # Allow teleoperation to continue without FK

    def get_leader_commands(self):
        if self.is_shutting_down:
            return

        try:
            if self.leader_reader is not None:
                self.command_joint_positions = self.leader_reader.get_latest_data()
            else:
                self.command_joint_positions = self.leader_interface.get_joint_positions()
        except Exception:
            return

        if not self.is_shutting_down:
            self._cache_data_for_recording()

    def _cache_data_for_recording(self):
        with self.data_lock:
            current_time = time.time()

            try:
                if self.follower_interface and self.follower_interface.is_connected:
                    positions = self.follower_interface.get_joint_positions()
                    self.cached_observation = positions.copy() if positions is not None else None
                    self.last_observation_time = current_time

                    if positions is not None and self.follower_pin_kine is not None:
                        self._update_follower_fk(positions)
            except Exception:
                pass

            try:
                if self.command_joint_positions is not None:
                    if isinstance(self.command_joint_positions, np.ndarray):
                        self.cached_action = self.command_joint_positions.copy()
                    else:
                        self.cached_action = np.array(self.command_joint_positions)
                    self.last_action_time = current_time

                    if self.leader_pin_kine is not None:
                        self._update_leader_fk()
            except Exception:
                pass

    def _update_follower_fk(self, positions):
        try:
            self.follower_pin_kine.update(positions[:5])

            ee_pos = self.follower_pin_kine.getSitePos(0)
            ee_quat = self.follower_pin_kine.getSiteQuat(0)
            self.cached_ee_pose_obs = np.concatenate([ee_pos, ee_quat]).astype(np.float32)

            for cam_idx, cam_name in enumerate(['camera_wrist', 'camera_front'], start=1):
                cam_pos = self.follower_pin_kine.getSitePos(cam_idx)
                cam_quat = self.follower_pin_kine.getSiteQuat(cam_idx)
                self.cached_camera_poses_obs[cam_name] = np.concatenate([cam_pos, cam_quat]).astype(np.float32)
        except Exception:
            pass

    def _update_leader_fk(self):
        try:
            cmd_joints = self.command_joint_positions[:5] if len(self.command_joint_positions) >= 5 else self.command_joint_positions
            self.leader_pin_kine.update(cmd_joints)

            ee_pos = self.leader_pin_kine.getSitePos(0)
            ee_quat = self.leader_pin_kine.getSiteQuat(0)
            self.cached_ee_pose_action = np.concatenate([ee_pos, ee_quat]).astype(np.float32)

            for cam_idx, cam_name in enumerate(['camera_wrist', 'camera_front'], start=1):
                cam_pos = self.leader_pin_kine.getSitePos(cam_idx)
                cam_quat = self.leader_pin_kine.getSiteQuat(cam_idx)
                self.cached_camera_poses_action[cam_name] = np.concatenate([cam_pos, cam_quat]).astype(np.float32)
        except Exception:
            pass

    def send_follower_commands(self):
        if self.command_joint_positions is not None:
            robot_command = RobotState()
            robot_command.num_joints = len(self.command_joint_positions)
            for i, pos in enumerate(self.command_joint_positions):
                robot_command.joint_pos[i] = pos
            self.follower_interface.set_robot_command(robot_command)

            # For SO101 in sim mode: also send gripper command directly
            if self.mode == "sim" and len(self.command_joint_positions) > self.follower_interface.mdof:
                gripper_idx = self.follower_interface.mdof
                self.follower_interface.mjData.ctrl[gripper_idx] = self.command_joint_positions[5]


    def run(self):
        self.logger.info(f"Teleoperation Controller: mode={self.mode}, freq={self.frequency}Hz")

        try:
            while True:
                if self.mode == "sim" and not self.follower_interface.is_viewer_alive():
                    break

                precall_wall_time = time.time()

                self.get_leader_commands()
                self.send_follower_commands()

                self.leader_interface.step()
                self.follower_interface.step()

                sleep_time = self.timestep - (time.time() - precall_wall_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def cleanup(self):
        self.is_shutting_down = True
        time.sleep(0.1)

        if hasattr(self, "leader_reader") and self.leader_reader:
            try:
                self.leader_reader.stop()
            except Exception:
                pass

        if hasattr(self, "leader_interface") and self.leader_interface:
            try:
                if hasattr(self.leader_interface, "is_connected") and self.leader_interface.is_connected:
                    self.leader_interface.disconnect()
            except Exception:
                pass

        if hasattr(self, "follower_interface") and self.follower_interface:
            try:
                if hasattr(self.follower_interface, "is_connected") and self.follower_interface.is_connected:
                    self.follower_interface.disconnect()
            except Exception:
                pass

    def get_observation(self):
        with self.data_lock:
            if self.cached_observation is not None and time.time() - self.last_observation_time < 1.0:
                return self.cached_observation.copy()
            return np.zeros(6)

    def get_action(self):
        with self.data_lock:
            if self.cached_action is not None and time.time() - self.last_action_time < 1.0:
                return self.cached_action.copy()
            return np.zeros(6)

    def get_follower_interface(self):
        return self.follower_interface

    def get_leader_interface(self):
        return self.leader_interface

    def get_eePose_observation(self) -> np.ndarray:
        """Return follower EE pose [x, y, z, qx, qy, qz, qw]."""
        with self.data_lock:
            return self.cached_ee_pose_obs.copy()

    def get_eePose_action(self) -> np.ndarray:
        """Return leader EE pose [x, y, z, qx, qy, qz, qw]."""
        with self.data_lock:
            return self.cached_ee_pose_action.copy()

    def get_cameraPoses_observation(self) -> dict:
        """Return follower camera poses dict."""
        with self.data_lock:
            return {name: pose.copy() for name, pose in self.cached_camera_poses_obs.items()}

    def get_cameraPoses_action(self) -> dict:
        """Return leader camera poses dict."""
        with self.data_lock:
            return {name: pose.copy() for name, pose in self.cached_camera_poses_action.items()}


def main():
    parser = argparse.ArgumentParser(description="Joint Teleoperation Controller")
    parser.add_argument("--mode", choices=["sim", "real"], default="sim")
    parser.add_argument("--ctrlfreq", type=int, default=100)
    parser.add_argument("--config", default="configs/so101.yaml")
    args = parser.parse_args()

    teleoperator = TeleOperator(
        mode=args.mode,
        frequency=args.ctrlfreq,
        config_path=args.config,
    )
    teleoperator.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
