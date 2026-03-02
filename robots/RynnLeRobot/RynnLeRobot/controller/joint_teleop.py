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

from RynnLeRobot.scripts.find_camera_port import find_all_opencv_cameras
from RynnLeRobot.hardware.cameras.opencv.camera_opencv import OpenCVCamera
from RynnLeRobot.hardware.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from RynnLeRobot.hardware.cameras.configs import ColorMode
from RynnLeRobot.utils.visualization_utils import display_imgs, is_headless
from RynnLeRobot.tools.web_streamer import WebServer


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
        recording: bool = False,
        show_display: bool = False,  # 新增参数：控制画面显示
        show_webcam: bool = False,  # 新增参数：控制画面显示
        log_level: str = "INFO",
    ):
        self.mode = mode
        self.recording = recording
        self.frequency = max(30, min(frequency, 250))
        self.timestep = 1.0 / self.frequency
        self.config_path = config_path
        self.show_display = show_display  # 保存显示控制参数
        self.show_webcam = show_webcam  # 保存显示控制参数
        self.webcam_server = None
        self.log_level = log_level
        self.config = self._load_config(config_path)

        self.init_logging(log_level)

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

        # 添加摄像头相关属性
        self.cameras = {}
        self.cameras_config = self.config.get("cameras", {})

        # 只有在real模式且需要显示时才初始化摄像头
        if self.mode == "real":
            if self.show_display or self.show_webcam:
                self.setup_cameras()
            if self.show_webcam:
                self.webcam_server = WebServer()
                self.webcam_server.start()

    def setup_cameras(self):
        """Initialize cameras based on configuration."""
        if not self.cameras_config:
            self.logger.warning("⚠️ No camera configuration found")
            return

        for camera_name, cam_config in self.cameras_config.items():
            try:
                opencv_config = OpenCVCameraConfig(
                    index_or_path=cam_config["index_or_path"],
                    width=cam_config.get("width", 640),
                    height=cam_config.get("height", 360),
                    fps=cam_config.get("fps", 30),
                    color_mode=(
                        ColorMode.RGB
                        if cam_config.get("color_mode", "rgb").lower() == "rgb"
                        else ColorMode.BGR
                    ),
                )

                camera = OpenCVCamera(opencv_config)
                camera.connect(warmup=True)
                self.cameras[camera_name] = camera
                self.logger.info(
                    f"✅ {camera_name} camera connected on port {cam_config['index_or_path']}"
                )

            except Exception as e:
                self.logger.error(f"❌ Failed to setup {camera_name} camera: {e}")
                self.cameras[camera_name] = None

    def disconnect_cameras(self):
        """Disconnect all cameras."""
        for camera_name, camera in self.cameras.items():
            if camera and camera.is_connected:
                try:
                    camera.disconnect()
                    self.logger.info(f"🔌 {camera_name} camera disconnected")
                except Exception as e:
                    self.logger.error(
                        f"❌ Error disconnecting {camera_name} camera: {e}"
                    )

    def get_record_observations(self):
        """Get current camera observations for recording."""
        with self.data_lock:
            if hasattr(self, "cached_camera_obs") and self.cached_camera_obs:
                return self.cached_camera_obs.copy()
            # Return empty dict if no cached data
            return {}

    def get_camera_observations(self):
        """Get camera observations."""
        observation = {}
        for camera_name, camera in self.cameras.items():
            if camera and camera.is_connected:
                try:
                    frame = camera.read(color_mode=ColorMode.RGB)
                    observation[f"{camera_name}_image"] = frame
                except Exception as e:
                    self.logger.warning(
                        f"⚠️ Error reading from {camera_name} camera: {e}"
                    )
                    observation[f"{camera_name}_image"] = np.zeros(
                        (
                            cam_config.get("height", 480),
                            cam_config.get("width", 640),
                            3,
                        ),
                        dtype=np.uint8,
                    )
            else:
                observation[f"{camera_name}_image"] = np.zeros(
                    (cam_config.get("height", 480), cam_config.get("width", 640), 3),
                    dtype=np.uint8,
                )
        return observation
    def _load_config(self, config_path):
        """Load configuration from YAML file."""
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
            return config if config else {}
        except FileNotFoundError:
            return {}

    def init_logging(self, log_level="INFO"):
        """Setup logging configuration."""
        from datetime import datetime

        log_dir = os.path.expanduser(
            self.config.get("logging", {}).get("log_dir", "~/logs/lerobot")
        )
        log_file = self.config.get("logging", {}).get("log_file", "teleop_simulation.log")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        log_path = os.path.join(log_dir, f"{log_file.replace('.log', '')}_{timestamp}.log")

        logging.basicConfig(
            level=getattr(logging, log_level.upper()),  # 设置一个全局的最低日志级别
            format="%(asctime)s [%(levelname)s] %(message)s",
            handlers=[
                # 处理器1：写入文件
                # Handler 1: Writes to a file
                logging.FileHandler(log_path),
                # 处理器2：输出到终端（控制台）
                # Handler 2: Outputs to the terminal (console)
                logging.StreamHandler(sys.stdout),  # 或者直接用 logging.StreamHandler()
            ],
            # force=True 可以在多次调用时重新配置，避免 "already configured" 的警告
            # force=True allows reconfiguration on multiple calls, avoiding "already configured" warnings
            force=True,
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

            # 获取摄像头观测数据（仅在real模式下）
            if self.mode == "real" and self.cameras and not self.recording:
                try:
                    print("self.cached_camera_obs = self.get_camera_observations()")
                    self.cached_camera_obs = self.get_camera_observations()
                except Exception as e:
                    if not self.is_shutting_down:
                        self.logger.debug(f"Error caching camera observations: {e}")
            if self.recording and self.cameras:
                try:
                    self.cached_camera_obs = self.get_record_observations()
                except Exception as e:
                    if not self.is_shutting_down:
                        self.logger.debug(f"Error caching camera observations: {e}")

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

    def show_commands(self, prefix="", angle="degree"):
        if self.command_joint_positions is not None:
            joint_values = self.command_joint_positions
            if angle == "degree":
                joint_values = [x / 3.1415926 * 180 for x in joint_values[:-1]] + [
                    joint_values[-1]
                ]
            joint_values = f"[{', '.join([f'{x:6.2f}' for x in joint_values])}]"
            self.logger.info(f"{prefix} ({angle}): {joint_values}")

    def run(self):
        """Run the dual-interface teleoperation controller"""
        self.logger.info("=" * 60)
        self.logger.info(f"Teleoperation Controller: mode={self.mode}, freq={self.frequency}Hz")
        self.logger.info(
            f"Show Display: {self.show_display}; Show Web Camera: {self.show_webcam}"
        )  # 显示当前显示状态
        self.logger.info("Press Ctrl+C or close viewer window to stop")
        self.logger.info("=" * 60)
        frame_count = 0

        try:
            while True:
                if self.mode == "sim" and not self.follower_interface.is_viewer_alive():
                    break

                precall_wall_time = time.time()

                self.get_leader_commands()

                # 只有在启用显示时才显示图像
                if (
                    self.mode == "real"
                    and (self.show_display or self.show_webcam)
                    and getattr(self, "cached_camera_obs", False)
                ):
                    try:
                        display_imgs(
                            self.cached_camera_obs, self.show_display, self.show_webcam
                        )
                    except Exception as e:
                        self.logger.debug(f"Error displaying images: {e}")

                self.send_follower_commands()
                if not self.recording and frame_count % (self.frequency // 2) == 0:
                    self.show_commands(prefix="follower")

                self.leader_interface.step()
                self.follower_interface.step()

                sleep_time = self.timestep - (time.time() - precall_wall_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -0.001 and frame_count % (self.frequency // 2) == 0:
                    self.logger.warning(
                        f"Control loop running behind by {-sleep_time:.3f}s"
                    )
                frame_count += 1
                if frame_count > self.frequency:
                    frame_count = 0

        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def cleanup(self):
        self.is_shutting_down = True
        time.sleep(0.1)

        # 断开摄像头连接
        if self.mode == "real":
            try:
                self.disconnect_cameras()
                # 关闭OpenCV窗口（仅在启用显示时）
                if self.show_display:
                    import cv2

                    cv2.destroyAllWindows()
                if self.show_webcam:
                    self.webcam_server.stop()
            except Exception as e:
                self.logger.warning(f"Warning during camera cleanup: {e}")

        # Clean up leader data reader thread
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
            if (
                self.cached_observation is not None
                and time.time() - self.last_observation_time < 1.0
            ):
                return self.cached_observation.copy()
            return np.zeros(6)

    def get_action(self):
        with self.data_lock:
            if (
                self.cached_action is not None
                and time.time() - self.last_action_time < 1.0
            ):
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
        # 新增参数：控制画面显示
    parser.add_argument(
        "--show-display",
        action="store_true",
        default=False,
        help="Show camera display window (default: False)",
    )
    parser.add_argument(
        "--show-webcam",
        action="store_true",
        default=False,
        help="Show camera on web browser (default: False)",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["INFO", "DEBUG", "WARNING", "ERROR"],
        help="log level",
    )
    # parser.add_argument("--lang", default="EN", choices=["EN", "CN"], help="Language")

    args = parser.parse_args()

    if is_headless():
        logging.warning("Running in headless mode. Set args.show_display=False.")
        args.show_display = False

    print(args)

    teleoperator = TeleOperator(
        mode=args.mode,
        frequency=args.ctrlfreq,
        config_path=args.config,
        show_display=args.show_display,  # 传递显示控制参数
        show_webcam=args.show_webcam,  # 传递显示控制参数
        log_level=args.log_level,
    )
    teleoperator.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
