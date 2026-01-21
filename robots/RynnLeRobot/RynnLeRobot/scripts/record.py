#!/usr/bin/env python3
# Copyright 2025 Alibaba Damo Academy. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""LeRobot Dataset Recorder.

Records datasets by importing and running teleoperation directly.

Usage:
    $ record --repo-id "user/dataset" --task "Pick cube" --episodes 5
    $ record --repo-id "user/dataset" --task "Pick cube" -gv no

Options:
    --godview, -gv: Express poses in front camera frame (default: yes).
                    Set to 'no' to keep poses in robot base frame.
"""

import argparse
import logging
import time
import os
import sys
import threading
import json
import yaml
import numpy as np

from RynnLeRobot.controller.joint_teleop import TeleOperator
from RynnMotion.RynnDatasets.rynn_dataset import RynnDataset
from RynnMotion.RynnDatasets.utils import build_dataset_frame, hw_to_dataset_features
from RynnMotion.RynnDatasets.video_utils import VideoEncodingManager
from RynnMotion.RynnDatasets.image_writer import safe_stop_image_writer
from RynnMotion.algorithms.orient import Transform
from RynnLeRobot.scripts.find_camera_port import find_cameras, find_all_opencv_cameras
from RynnLeRobot.hardware.cameras.opencv.camera_opencv import OpenCVCamera
from RynnLeRobot.hardware.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from RynnLeRobot.hardware.cameras.configs import ColorMode
from RynnLeRobot.utils.keyboard_listener import (
    init_keyboard_listener,
    cleanup_keyboard_listener,
)
from RynnLeRobot.utils.visualization_utils import display_imgs, is_headless
from RynnLeRobot.tools.web_streamer import WebServer

logger = logging.getLogger(__name__)


class RynnLeRobotRecorder:
    """Recorder that imports and manages teleoperation directly.

    Attributes:
        godview: When True, transforms poses to front camera frame.
    """

    def __init__(
        self,
        repo_id: str,
        fps: int = 30,
        root: str = "./outputs",
        config_path: str = "configs/so101.yaml",
        cameras_config: dict = None,
        show_display: bool = False,
        show_webcam: bool = False,
        log_level: str = "INFO",
        godview: bool = True
    ):
        self.repo_id = repo_id
        self.fps = fps
        self.root = root
        self.config_path = config_path
        self.godview = godview

        self.config = self.load_config()
        self.cameras_config = cameras_config or self.config.get("cameras", {})
        self.cameras = {}
        self.teleop = None
        self.dataset = None
        self.recording = False
        self.teleop_thread = None
        self.show_display = show_display
        self.windows_created = False
        self.show_webcam = show_webcam
        self.webcam_server = None
        self.log_level = log_level

        logger.info(f"📋 Recorder initialized with config: {config_path}")
        logger.info(f"🎥 Camera configuration: {list(self.cameras_config.keys())}")

    def load_config(self):
        """Load configuration from YAML file."""
        try:
            with open(self.config_path, "r") as f:
                config = yaml.safe_load(f)
            return config
        except FileNotFoundError:
            logger.warning(f"Config file {self.config_path} not found, using defaults")
            return {}

    def detect_and_update_camera_ports(self):
        """Auto-detect camera ports using hardcoded assignment (last two cameras)."""
        logger.info("🔍 Auto-detecting camera ports using hardcoded assignment...")
        try:
            detected_cameras = find_all_opencv_cameras()

            if len(detected_cameras) >= 2:
                detected_cameras.sort(key=lambda cam: str(cam["id"]))
                front_camera_id = detected_cameras[-2]["id"]
                wrist_camera_id = detected_cameras[-1]["id"]

                if "front" in self.cameras_config:
                    self.cameras_config["front"]["index_or_path"] = front_camera_id
                    logger.info(f"📷 front camera -> port {front_camera_id}")

                if "wrist" in self.cameras_config:
                    self.cameras_config["wrist"]["index_or_path"] = wrist_camera_id
                    logger.info(f"📷 wrist camera -> port {wrist_camera_id}")

                self.save_camera_config_to_yaml()

                logger.info(
                    "✅ Using last two cameras (works on Ubuntu laptops - skips built-in cameras)"
                )
            else:
                logger.warning(
                    f"⚠️ Only {len(detected_cameras)} cameras detected, expected at least 2"
                )
        except Exception as e:
            logger.warning(f"⚠️ Camera auto-detection failed: {e}")

    def save_camera_config_to_yaml(self):
        """Save updated camera configuration back to YAML file."""
        try:
            with open(self.config_path, "r") as f:
                config = yaml.safe_load(f)
            config["cameras"] = self.cameras_config
            with open(self.config_path, "w") as f:
                yaml.safe_dump(config, f, default_flow_style=False)
            logger.info(f"💾 Updated camera configuration in {self.config_path}")
        except Exception as e:
            logger.error(f"❌ Failed to save camera config: {e}")

    def setup_cameras(self):
        """Initialize cameras based on configuration."""
        if not self.cameras_config:
            logger.warning("⚠️ No camera configuration found")
            return

        for camera_name, cam_config in self.cameras_config.items():
            try:
                # 检查设备是否存在且可访问
                device_path = cam_config["index_or_path"]
                if isinstance(device_path, str) and device_path.startswith("/dev/"):
                    import os

                    if not os.path.exists(device_path):
                        logger.warning(f"⚠️ Camera device {device_path} does not exist")
                        self.cameras[camera_name] = None
                        continue

                opencv_config = OpenCVCameraConfig(
                    index_or_path=cam_config["index_or_path"],
                    width=cam_config.get("width", 640),
                    height=cam_config.get("height", 480),
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
                logger.info(
                    f"✅ {camera_name} camera connected on port {cam_config['index_or_path']}"
                )

            except Exception as e:
                logger.error(f"❌ Failed to setup {camera_name} camera: {e}")
                self.cameras[camera_name] = None

    def reset_camera_devices(self):
        """尝试重置摄像头设备"""
        try:
            import cv2
            import os

            # 尝试释放可能被占用的设备
            for camera_name, cam_config in self.cameras_config.items():
                device_path = cam_config["index_or_path"]
                if isinstance(device_path, int):
                    # 对于索引设备，创建并立即释放一个capture对象
                    cap = cv2.VideoCapture(device_path)
                    if cap.isOpened():
                        cap.release()
                elif isinstance(device_path, str) and device_path.startswith("/dev/"):
                    # 检查设备权限
                    if os.path.exists(device_path) and not os.access(
                        device_path, os.R_OK
                    ):
                        logger.warning(
                            f"⚠️ No read access to camera device {device_path}"
                        )

            logger.info("🔄 Camera devices reset attempt completed")
        except Exception as e:
            logger.warning(f"⚠️ Error during camera device reset: {e}")

    def disconnect_cameras(self):
        """Disconnect all cameras."""
        for camera_name, camera in self.cameras.items():
            if camera and camera.is_connected:
                try:
                    camera.disconnect()
                    logger.info(f"🔌 {camera_name} camera disconnected")
                except Exception as e:
                    logger.error(f"❌ Error disconnecting {camera_name} camera: {e}")

    def get_robot_features(self):
        """Get robot features for dataset creation."""
        joint_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        ]
        motor_features = {f"{joint}.pos": float for joint in joint_names}
        # camera_features = {
        #     "front": (480, 640, 3),
        #     "wrist": (480, 640, 3)
        # }
        camera_features = {}
        for camera_name, cam_config in self.cameras_config.items():
            height = cam_config.get("height", 480)  # 仅在配置缺失时使用默认值
            width = cam_config.get("width", 640)  # 仅在配置缺失时使用默认值
            camera_features[camera_name] = (height, width, 3)

        # NEW: Add end-effector pose features (list format for pose detection)
        ee_pose_features = {"end_effector_pose": [7]}  # [x, y, z, qx, qy, qz, qw]

        # NEW: Add camera pose features (list format per camera)
        camera_pose_features = {}
        for camera_name in self.cameras_config.keys():
            camera_pose_features[f"camera_poses.{camera_name}"] = [
                7
            ]  # [x, y, z, qx, qy, qz, qw]

        return {
            "action_features": {**motor_features, **ee_pose_features},
            "observation_features": {
                **motor_features,
                **camera_features,
                **ee_pose_features,
                **camera_pose_features,
            },
        }

    def setup_dataset(self):
        """Initialize the dataset following source repo pattern."""
        features = self.get_robot_features()

        action_features = hw_to_dataset_features(
            features["action_features"], "action", use_video=True
        )
        obs_features = hw_to_dataset_features(
            features["observation_features"], "observation", use_video=True
        )
        dataset_features = {**action_features, **obs_features}

        self.dataset = RynnDataset.create(
            repo_id=self.repo_id,
            fps=self.fps,
            root=self.root,
            robot_type="so101_follower",  # Based on teleop config
            features=dataset_features,
            use_videos=True,
            image_writer_processes=0,
            image_writer_threads=8,
            batch_encoding_size=1,
        )
        logger.info(f"✅ Dataset initialized: {self.repo_id}")

    def start_teleoperation_thread(self):
        """Start teleoperation in a separate thread."""
        self.teleop = TeleOperator(
            mode="real",
            frequency=30,
            config_path=self.config_path,
            recording=True,
            log_level=self.log_level,
        )

        def run_teleop():
            try:
                self.teleop.run()
            except Exception as e:
                logger.error(f"❌ Teleoperation error: {e}")

        self.teleop_thread = threading.Thread(target=run_teleop, daemon=True)
        self.teleop_thread.start()

        time.sleep(2.0)
        logger.info("✅ Teleoperation thread started")

    def stop_teleoperation(self):
        """Stop the teleoperation thread."""
        if self.teleop:
            self.teleop.cleanup()
        logger.info("🛑 Teleoperation stopped")

    def get_robot_observation(self):
        """Get robot observation from teleoperator (follower joint positions in rad)."""
        observation = {}

        try:
            if self.teleop:
                joint_observation = self.teleop.get_observation()
                # Convert numpy array to joint dictionary
                if isinstance(joint_observation, np.ndarray):
                    joint_names = [
                        "shoulder_pan",
                        "shoulder_lift",
                        "elbow_flex",
                        "wrist_flex",
                        "wrist_roll",
                        "gripper",
                    ]
                    for i, joint_name in enumerate(joint_names):
                        if i < len(joint_observation):
                            observation[f"{joint_name}.pos"] = float(
                                joint_observation[i]
                            )
                        else:
                            observation[f"{joint_name}.pos"] = 0.0
                else:
                    # Fallback for dictionary format (backward compatibility)
                    observation.update(joint_observation)

                ee_pose_obs = self.teleop.get_eePose_observation()
                observation["end_effector_pose"] = ee_pose_obs.astype(
                    np.float32
                )  # Convert to float32

            for camera_name, camera in self.cameras.items():
                if camera and camera.is_connected:
                    try:
                        frame = camera.read(color_mode=ColorMode.RGB)
                        # 同时添加两种键名格式以兼容display_imgs
                        observation[camera_name] = frame
                        observation[f"{camera_name}_image"] = frame  # 关键：添加带_image后缀的键
                    except Exception as e:
                        logger.warning(f"⚠️ Error reading from {camera_name} camera: {e}")
                        # 两种格式都设置默认值
                        default_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                        observation[camera_name] = default_frame
                        observation[f"{camera_name}_image"] = default_frame
                else:
                    # 两种格式都设置默认值
                    default_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    observation[camera_name] = default_frame
                    observation[f"{camera_name}_image"] = default_frame

            # Get camera poses from TeleOperator FK (computed via Pinocchio)
            try:
                camera_poses_obs = self.teleop.get_cameraPoses_observation()
                # Map camera names: 'front' -> 'camera_front', 'wrist' -> 'camera_wrist'
                for camera_name in self.cameras_config.keys():
                    if camera_name == 'front':
                        observation[f"camera_poses.{camera_name}"] = camera_poses_obs.get('camera_front', np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32))
                    elif camera_name == 'wrist':
                        observation[f"camera_poses.{camera_name}"] = camera_poses_obs.get('camera_wrist', np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32))
                    else:
                        # Default for unknown cameras
                        observation[f"camera_poses.{camera_name}"] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
            except Exception as e:
                logger.warning(f"⚠️ Error getting camera poses from FK: {e}")
                # Fallback to default values
                for camera_name in self.cameras_config.keys():
                    observation[f"camera_poses.{camera_name}"] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)

        except Exception as e:
            logger.warning(f"⚠️ Error getting observation: {e}")
            joint_names = [
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
                "gripper",
            ]
            for joint in joint_names:
                observation[f"{joint}.pos"] = 0.0

            # NEW: Add fallback for end-effector pose (numpy array)
            observation["end_effector_pose"] = np.array(
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32
            )

            for camera_name in self.cameras_config.keys():
                observation[camera_name] = np.zeros(
                    (cam_config.get("height", 480), cam_config.get("width", 640), 3),
                    dtype=np.uint8,
                )
                # 添加带'image'关键字的键以兼容display_imgs
                observation[f"{camera_name}_image"] = np.zeros(
                    (cam_config.get("height", 480), cam_config.get("width", 640), 3),
                    dtype=np.uint8,
                )
                # NEW: Add fallback for camera poses (numpy array)
                observation[f"camera_poses.{camera_name}"] = np.array(
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32
                )

        return observation

    def get_robot_action(self):
        """Get robot action from teleoperator (leader joint commands in rad)."""
        action = {}

        try:
            if self.teleop:
                joint_action = self.teleop.get_action()
                # Convert numpy array to joint dictionary
                if isinstance(joint_action, np.ndarray):
                    joint_names = [
                        "shoulder_pan",
                        "shoulder_lift",
                        "elbow_flex",
                        "wrist_flex",
                        "wrist_roll",
                        "gripper",
                    ]
                    for i, joint_name in enumerate(joint_names):
                        if i < len(joint_action):
                            action[f"{joint_name}.pos"] = float(joint_action[i])
                        else:
                            action[f"{joint_name}.pos"] = 0.0
                else:
                    # Fallback for dictionary format (backward compatibility)
                    action = joint_action

                # NEW: Add end-effector pose action (numpy array)
                ee_pose_action = self.teleop.get_eePose_action()
                action["end_effector_pose"] = ee_pose_action.astype(
                    np.float32
                )  # Convert to float32

                # Get camera poses from TeleOperator FK (computed via Pinocchio)
                try:
                    camera_poses_action = self.teleop.get_cameraPoses_action()
                    # Map camera names: 'front' -> 'camera_front', 'wrist' -> 'camera_wrist'
                    for camera_name in self.cameras_config.keys():
                        if camera_name == 'front':
                            action[f"camera_poses.{camera_name}"] = camera_poses_action.get('camera_front', np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32))
                        elif camera_name == 'wrist':
                            action[f"camera_poses.{camera_name}"] = camera_poses_action.get('camera_wrist', np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32))
                        else:
                            # Default for unknown cameras
                            action[f"camera_poses.{camera_name}"] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
                except Exception as e:
                    logger.warning(f"⚠️ Error getting action camera poses from FK: {e}")
                    # Fallback to default values
                    for camera_name in self.cameras_config.keys():
                        action[f"camera_poses.{camera_name}"] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)

        except Exception as e:
            logger.warning(f"⚠️ Error getting action: {e}")
            joint_names = [
                "shoulder_pan",
                "shoulder_lift",
                "elbow_flex",
                "wrist_flex",
                "wrist_roll",
                "gripper",
            ]
            for joint in joint_names:
                action[f"{joint}.pos"] = 0.0

            action["end_effector_pose"] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        return action

    def transform_poses_to_front_camera_frame(
        self,
        observation: dict,
        action: dict
    ) -> tuple[dict, dict]:
        """Transform all poses from base frame to front camera frame.

        Args:
            observation: Observation dict with poses in base frame.
            action: Action dict with poses in base frame.

        Returns:
            Updated (observation, action) dicts with poses in front camera frame.
        """
        front_pose = observation["camera_poses.front"]
        T_base_front = Transform.from_pos_quat(front_pose[:3], front_pose[3:])
        T_front_base = T_base_front.inverse()

        ee_obs = observation["end_effector_pose"]
        T_base_ee_obs = Transform.from_pos_quat(ee_obs[:3], ee_obs[3:])
        T_front_ee_obs = T_front_base @ T_base_ee_obs
        observation["end_effector_pose"] = np.concatenate(
            [T_front_ee_obs.pos, T_front_ee_obs.quat]
        ).astype(np.float32)

        wrist_pose = observation["camera_poses.wrist"]
        T_base_wrist = Transform.from_pos_quat(wrist_pose[:3], wrist_pose[3:])
        T_front_wrist = T_front_base @ T_base_wrist
        observation["camera_poses.wrist"] = np.concatenate(
            [T_front_wrist.pos, T_front_wrist.quat]
        ).astype(np.float32)

        observation["camera_poses.front"] = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32
        )

        ee_action = action["end_effector_pose"]
        T_base_ee_action = Transform.from_pos_quat(ee_action[:3], ee_action[3:])
        T_front_ee_action = T_front_base @ T_base_ee_action
        action["end_effector_pose"] = np.concatenate(
            [T_front_ee_action.pos, T_front_ee_action.quat]
        ).astype(np.float32)

        return observation, action

    @safe_stop_image_writer
    def record_episode(self, task: str, episode_length_s: float, events: dict = None):
        """Record a single episode following source repo pattern."""
        episode_frames = int(episode_length_s * self.fps)
        logger.info(
            f"📹 Recording episode: {episode_frames} frames ({episode_length_s}s)"
        )
        self.recording = True
        frame_count = 0
        start_time = time.perf_counter()

        try:
            while frame_count < episode_frames and self.recording:
                frame_start = time.perf_counter()

                # 检查键盘事件
                if events:
                    if events["stop_recording"]:  # ESC键 - 停止整个录制会话
                        events["stop_recording"] = False
                        events["exit_early"] = False
                        logger.info("🛑 Stopping entire recording session")
                        break

                    if events["rerecord_episode"]:  # 左箭头键 - 重新录制当前回合
                        events["rerecord_episode"] = False
                        events["exit_early"] = False
                        logger.info("🔄 Re-recording current episode")
                        # 重置录制状态并重新开始当前回合
                        self.dataset.clear_episode_buffer()
                        frame_count = 0
                        start_time = time.perf_counter()
                        continue

                    if events["exit_early"]:  # 右箭头键 - 提前结束当前回合但保存数据
                        events["exit_early"] = False
                        logger.info("⏩ Exiting episode early but saving data")
                        break

                observation = self.get_robot_observation()
                action = self.get_robot_action()

                if self.godview:
                    observation, action = self.transform_poses_to_front_camera_frame(observation, action)

                observation_frame = build_dataset_frame(
                    self.dataset.features, observation, prefix="observation"
                )
                action_frame = build_dataset_frame(
                    self.dataset.features, action, prefix="action"
                )

                frame = {**observation_frame, **action_frame}
                self.dataset.add_frame(frame, task=task)

                # 创建一个新字典，将numpy数组转换为适合display_imgs的格式
                display_obs = {
                    k: v for k, v in observation.items() if isinstance(v, np.ndarray)
                }
                try:
                    display_imgs(display_obs, self.show_display, self.show_webcam)
                except Exception as e:
                    self.logger.debug(f"Error displaying images: {e}")

                if self.show_display and self.windows_created:
                    # 定期刷新窗口以保持焦点
                    import cv2

                    if frame_count % 5 == 0:  # 每5帧检查一次焦点
                        cv2.waitKey(1)

                frame_count += 1

                dt_s = time.perf_counter() - frame_start
                sleep_time = (1.0 / self.fps) - dt_s
                if sleep_time > 0:
                    time.sleep(sleep_time)

                if frame_count % (self.fps * 1) == 0:  # Every 1 seconds
                    state_arr = frame.get("observation.state", np.array([]))
                    action_arr = frame.get("action", np.array([]))
                    #self.show_commands(state_arr, " state", "degree")
                    #self.show_commands(action_arr, "action", "degree")

                if frame_count % (self.fps * 5) == 0:  # Every 5 seconds
                    elapsed = time.perf_counter() - start_time
                    logger.info(
                        f"  📊 Frame {frame_count}/{episode_frames} ({elapsed:.1f}s)"
                    )

        except KeyboardInterrupt:
            logger.info("⚠️ Episode recording interrupted")
            self.recording = False

        # 重置事件标志
        if events:
            events["exit_early"] = False
            events["rerecord_episode"] = False
            events["stop_recording"] = False
            events["continue"] = False

        if frame_count > 0:
            self.dataset.save_episode()
            total_time = time.perf_counter() - start_time
            logger.info(f"✅ Episode saved: {frame_count} frames in {total_time:.2f}s")
        else:
            self.dataset.clear_episode_buffer()
            logger.info("⚠️ Episode discarded: No frames recorded")

        return frame_count > 0  # Return whether episode was recorded

    def record_dataset(
        self,
        task: str,
        num_episodes: int,
        episode_time_s: float,
        reset_time_s: float = 10.0,
        push_to_hub: bool = False,
    ):
        """Record a complete dataset."""
        logger.info(f"🎬 Starting dataset recording")
        logger.info(f"   Task: {task}")
        logger.info(f"   Episodes: {num_episodes}")
        logger.info(f"   Episode length: {episode_time_s}s")

        # 尝试重置摄像头设备
        self.reset_camera_devices()

        listener, events = init_keyboard_listener()

        try:
            # 初始化摄像头
            self.setup_cameras()  # Initialize cameras (using config file ports)
            self.setup_dataset()
            self.start_teleoperation_thread()

            if self.show_webcam:
                self.webcam_server = WebServer()
                self.webcam_server.start()

            with VideoEncodingManager(self.dataset):
                recorded_episodes = 0
                current_episode = 0

                while current_episode < num_episodes and not (
                    events.get("stop_recording", False)
                ):
                    logger.info(f"\n{'=' * 60}")
                    logger.info(f"Episode {current_episode + 1}/{num_episodes}")
                    logger.info(f"{'=' * 60}")

                    logger.info("🎮 Teleoperation is running")
                    logger.info("📋 Setup your robot and environment for recording")
                    logger.info(f"⏱️  Recording will last {episode_time_s} seconds")

                    # 在每次开始录制前清理可能存在的OpenCV窗口
                    if self.show_display and self.windows_created:
                        import cv2

                        cv2.destroyAllWindows()
                        self.windows_created = False

                    print(
                        "▶️  Press ENTER when ready to start recording this episode..."
                    )
                    while True:
                        if events.get("continue", False):
                            events["continue"] = False
                            break
                        time.sleep(0.5)

                    # 在开始录制时创建窗口并确保获得焦点
                    # if self.show_display and not self.windows_created:
                    #     self.windows_created = True
                    #     import cv2

                    #     # 创建控制窗口
                    #     cv2.namedWindow("Recording Control", cv2.WINDOW_AUTOSIZE)
                    #     cv2.setWindowTitle(
                    #         "Recording Control",
                    #         "Recording in Progress - Click to focus",
                    #     )
                    #     # # 设置窗口位置到屏幕右侧
                    #     # cv2.moveWindow("Recording Control", 1000, 100)  # x=1000, y=100 将窗口移到右侧
                    #     # 获取屏幕尺寸并计算窗口位置
                    #     import tkinter as tk

                    #     root = tk.Tk()
                    #     screen_width = root.winfo_screenwidth()
                    #     screen_height = root.winfo_screenheight()
                    #     root.destroy()

                    #     # 将窗口放在右上角
                    #     window_x = screen_width - 650  # 600是窗口宽度+一些边距
                    #     window_y = 50
                    #     cv2.moveWindow("Recording Control", window_x, window_y)

                    #     # 显示提示信息
                    #     info_img = np.zeros((120, 600, 3), dtype=np.uint8)
                    #     cv2.putText(
                    #         info_img,
                    #         "RECORDING IN PROGRESS",
                    #         (10, 30),
                    #         cv2.FONT_HERSHEY_SIMPLEX,
                    #         1,
                    #         (0, 0, 255),
                    #         2,
                    #     )
                    #     cv2.putText(
                    #         info_img,
                    #         "ESC: Stop Session",
                    #         (10, 60),
                    #         cv2.FONT_HERSHEY_SIMPLEX,
                    #         0.6,
                    #         (255, 255, 255),
                    #         1,
                    #     )
                    #     cv2.putText(
                    #         info_img,
                    #         "Left Arrow: Re-record, Right Arrow: Exit Early",
                    #         (10, 90),
                    #         cv2.FONT_HERSHEY_SIMPLEX,
                    #         0.6,
                    #         (255, 255, 255),
                    #         1,
                    #     )
                    #     cv2.imshow("Recording Control", info_img)

                    #     # 强制将焦点转移到OpenCV窗口
                    #     cv2.waitKey(1)
                    #     # 再次调用提升焦点概率
                    #     cv2.setWindowProperty(
                    #         "Recording Control", cv2.WND_PROP_TOPMOST, 1
                    #     )
                    #     cv2.waitKey(100)  # 给一些时间让窗口获得焦点

                    episode_recorded = self.record_episode(task, episode_time_s, events)

                    # 每次录制结束后销毁窗口
                    if self.show_display and self.windows_created:
                        import cv2

                        cv2.destroyAllWindows()
                        self.windows_created = False

                    # 检查是否需要重新录制
                    if events.get("rerecord_episode", False):
                        logger.info("🔄 Re-recording episode...")
                        events["rerecord_episode"] = False
                        events["exit_early"] = False
                        continue  # 不增加episode计数，重新录制

                    if episode_recorded:
                        recorded_episodes += 1
                        current_episode += 1
                    else:
                        logger.info("⏭️  Episode skipped - continuing to next episode")
                        current_episode += 1

                    # 重置事件标志
                    events["exit_early"] = False
                    events["rerecord_episode"] = False
                    events["stop_recording"] = False
                    events["continue"] = False

                    if current_episode < num_episodes and not events.get(
                        "stop_recording", False
                    ):
                        logger.info(f"⏳ Reset time: {reset_time_s}s")
                        logger.info("   Please reset the environment for next episode")

                        reset_start = time.time()
                        while time.time() - reset_start < reset_time_s:
                            if events.get("stop_recording", False):
                                break
                            time.sleep(0.1)

                if events.get("stop_recording", False):
                    logger.info(f"\n🛑 Recording stopped by user")
                else:
                    logger.info(f"\n🎉 Dataset recording completed!")

                logger.info(f"   Episodes recorded: {recorded_episodes}")
                logger.info(f"   Location: {self.dataset.root}")

                if push_to_hub and recorded_episodes > 0:
                    logger.info("☁️  Uploading to HuggingFace Hub...")
                    self.dataset.push_to_hub()
                    logger.info(
                        f"✅ Uploaded: https://huggingface.co/datasets/{self.repo_id}"
                    )

        except KeyboardInterrupt:
            logger.info("\n⚠️  Recording interrupted by user")
        except Exception as e:
            logger.error(f"❌ Recording failed: {e}")
            raise
        finally:
            # 清理OpenCV窗口
            if self.show_display and self.windows_created:
                import cv2

                cv2.destroyAllWindows()
            if self.show_webcam:
                self.webcam_server.stop()
            cleanup_keyboard_listener(listener)
            # 断开摄像头
            self.disconnect_cameras()  # Disconnect cameras
            self.stop_teleoperation()


def main():
    parser = argparse.ArgumentParser(
        description="Record LeRobot dataset with integrated teleoperation"
    )

    parser.add_argument(
        "--repo-id",
        required=True,
        help="Dataset repository ID (e.g., 'username/dataset_name')",
    )
    parser.add_argument("--task", required=True, help="Task description")

    parser.add_argument(
        "--episodes", type=int, default=5, help="Number of episodes (default: 5)"
    )
    parser.add_argument(
        "--episode-time",
        type=float,
        default=30.0,
        help="Episode length in seconds (default: 30)",
    )
    parser.add_argument(
        "--reset-time",
        type=float,
        default=10.0,
        help="Reset time between episodes (default: 10)",
    )
    parser.add_argument(
        "--fps", type=int, default=30, help="Recording FPS (default: 30)"
    )
    parser.add_argument(
        "--root",
        default="./outputs",
        help="Local storage directory (default: ./outputs)",
    )
    parser.add_argument("--config", default="configs/so101.yaml",
                       help="Config file path (default: configs/so101.yaml)")
    parser.add_argument(
        "--push-to-hub",
        action="store_true",
        default=False,
        help="Upload to HuggingFace Hub",
    )
    parser.add_argument(
        "--cameras",
        type=str,
        default=None,
        help="Camera configuration JSON string (overrides config file)",
    )
    parser.add_argument(
        "--auto-detect-cameras",
        action="store_true",
        help="Auto-detect camera ports and update config file",
    )
    parser.add_argument(
        "--show-display",
        action="store_true",
        help="Enable real-time data visualization with opencv",
    )
    parser.add_argument(
        "--show-webcam",
        action="store_true",
        help="Enable real-time data visualization with webcam",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["INFO", "DEBUG", "WARNING", "ERROR"],
        help="log level",
    )
    parser.add_argument("--lang", default="EN", choices=["EN", "CN"], help="Language")
    parser.add_argument("--godview", "-gv", type=lambda x: x.lower() in ('yes', 'true', '1'),
                       default=True, help="Express poses in front camera frame (default: yes)")

    args = parser.parse_args()

    if is_headless():
        logging.warning("Running in headless mode. Set args.show_display=False.")
        args.show_display = False

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s - %(levelname)s - %(message)s",
    )
    logging.info(args)

    cameras_config = None
    if args.cameras:
        try:
            cameras_config = json.loads(args.cameras)
            logger.info(
                f"🎥 Using camera config from arguments: {list(cameras_config.keys())}"
            )
        except json.JSONDecodeError as e:
            logger.error(f"❌ Invalid camera configuration JSON: {e}")
            return 1

    try:
        if not (args.show_display or args.show_webcam):
            logger.info("📊 No visualization show...")
            # _init_rerun(session_name="recording")

        if args.auto_detect_cameras:
            logger.info("🔍 Running camera auto-detection...")
            from scripts.find_camera_port import find_cameras

            detected_cameras = find_cameras()
            logger.info(f"Detected cameras: {detected_cameras}")
            return 0

        recorder = RynnLeRobotRecorder(
            repo_id=args.repo_id,
            fps=args.fps,
            root=os.path.join(args.root, args.repo_id),
            config_path=args.config,
            cameras_config=cameras_config,
            show_display=args.show_display,
            show_webcam=args.show_webcam,
            log_level=args.log_level,
            godview=args.godview,
        )

        recorder.record_dataset(
            task=args.task,
            num_episodes=args.episodes,
            episode_time_s=args.episode_time,
            reset_time_s=args.reset_time,
            push_to_hub=args.push_to_hub,
        )

    except Exception as e:
        logger.error(f"❌ Recording failed: {e}")
        return 1

    logger.info("✅ Recording completed successfully")
    return 0


if __name__ == "__main__":
    sys.exit(main())
