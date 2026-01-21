#!/usr/bin/env python3
"""SO101 autonomous pick-and-place demonstration.

Usage:
    $ so101-pickplace
"""

import os
import sys
import time
import argparse
import logging
import yaml
import numpy as np

from RynnMotion.core.interface_base import robotinterface_factory
from RynnMotion.manager.robot_manager import RobotManager
from RynnMotion.utils.path_config import get_models_root
from RynnMotion.common.data.robot_state import RobotState
from RynnLeRobot.utils.cube_grabber import CubeGrabberLerp, CubeGrabberPControl


class SO101PickPlaceController:
    """SO101 autonomous pick-and-place demonstration controller (simulation only)."""

    def __init__(
        self,
        mode: str = "sim",
        frequency: int = 100,
        config_path: str = "configs/so101.yaml",
    ):
        if mode != "sim":
            raise ValueError("SO101 pick-and-place requires mode='sim'")

        self.mode = mode
        self.frequency = max(30, min(frequency, 250))
        self.timestep = 1.0 / self.frequency
        self.config_path = config_path
        self.config = self._load_config(config_path)

        self._init_logging()

        self.initial_joint_positions = None
        self.qPos_command = None
        self.robot_command = None
        self.interface = None
        self.robot_model = None
        self.cube_grabber = None
        self.cube_grabber_mode = None

        self._init_robot_model()
        self._init_interface()
        self._init_cube_grabber()

    def _load_config(self, config_path: str) -> dict:
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
            return config if config else {}
        except FileNotFoundError:
            return {}

    def _init_logging(self):
        from datetime import datetime

        log_dir = os.path.expanduser(self.config.get("logging", {}).get("log_dir", "~/logs/so101"))
        log_file = self.config.get("logging", {}).get("log_file", "so101_pickplace.log")
        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        log_path = os.path.join(log_dir, f"{log_file.replace('.log', '')}_{timestamp}.log")

        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s [%(levelname)s] %(message)s",
            handlers=[logging.FileHandler(log_path)],
        )
        self.logger = logging.getLogger(__name__)

    def get_robot_name(self) -> str:
        return "so101"

    def get_scene_config(self) -> dict:
        return {"scene_path": "3.robot_arm/24.so101/scene/scene_grip.xml", "scene_name": "so101_grip"}

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

    def _init_interface(self):
        self.interface = robotinterface_factory(
            "mujoco_sim_robot",
            self.robot_model,
            {"timestep": self.timestep},
        )
        self.interface.connect()

        self.initial_joint_positions = self.interface.get_joint_positions()
        self.robot_command = RobotState()
        self.robot_command.num_joints = self.interface.mdof

    def _init_cube_grabber(self):
        self.cube_grabber_mode = self.config.get("simulation", {}).get("cube_grabber_mode", "Pcontrol")

        models_root = get_models_root()
        model_mjcf_path = str(models_root / "3.robot_arm/24.so101/mjcf/so101_new_calib.xml")
        cube_names = ["cube1", "cube2", "cube3", "cube4", "cube5", "cube6"]

        mug_position = self.interface.data.body("mug").xpos.copy()
        mug_orientation = self.interface.data.body("mug").xquat
        mug_pose = np.concatenate((mug_position, mug_orientation))

        home_pose = {
            "position": self.interface.data.site("ee").xpos,
            "orientation": self.interface.data.site("ee").xmat,
        }

        if self.cube_grabber_mode == "lerp":
            self.cube_grabber = CubeGrabberLerp(
                home_joint_positions=self.initial_joint_positions,
                cube_names=cube_names,
                mug_pose=mug_pose,
                model_mjcf_path=model_mjcf_path,
            )
        elif self.cube_grabber_mode == "Pcontrol":
            self.cube_grabber = CubeGrabberPControl(
                home_pose=home_pose,
                home_joint_positions=self.initial_joint_positions,
                cube_names=cube_names,
                mug_pose=mug_pose,
                model_mjcf_path=model_mjcf_path,
                control_interval=self.timestep,
            )
        else:
            raise ValueError(f"Invalid cube_grabber_mode: {self.cube_grabber_mode}")

    def motion_planner(self, current_time: float) -> np.ndarray:
        self.cube_grabber.current_time = current_time
        self.cube_grabber.step()
        return self.cube_grabber.generated_joint_command.copy()

    def run(self):
        self.logger.info(f"SO101 Pick-and-Place Controller: mode={self.mode}, freq={self.frequency}Hz")

        try:
            while True:
                if not self.interface.is_viewer_alive():
                    break

                precall_wall_time = time.time()
                current_time = self.interface.get_current_time()

                qPos_feedback = self.interface.get_joint_positions()
                self.cube_grabber.current_joint_positions = qPos_feedback.copy()

                self.cube_grabber.all_cube_positions = {}
                for cube_name in self.cube_grabber.cube_names:
                    pose = self.interface.data.joint(cube_name).qpos
                    self.cube_grabber.all_cube_positions[cube_name] = pose[:3].copy()

                if self.cube_grabber.current_cube_name is not None:
                    self.cube_grabber.current_cube_pose = self.interface.data.joint(self.cube_grabber.current_cube_name).qpos

                if self.cube_grabber_mode == "Pcontrol":
                    self.cube_grabber.current_pose = self.cube_grabber.get_SE3_pose(
                        self.interface.data.site("ee").xpos,
                        self.interface.data.site("ee").xmat,
                    )

                self.qPos_command = self.motion_planner(current_time)

                if self.qPos_command is not None:
                    for i in range(len(self.qPos_command)):
                        self.robot_command.joint_pos[i] = self.qPos_command[i]
                    self.interface.set_robot_command(self.robot_command)

                self.interface.step()

                sleep_time = self.timestep - (time.time() - precall_wall_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def cleanup(self):
        if self.interface:
            self.interface.disconnect()


def main():
    parser = argparse.ArgumentParser(description="SO101 Pick-and-Place Controller (simulation only)")
    parser.add_argument("--frequency", type=int, default=100)
    parser.add_argument("--config", default="configs/so101.yaml")
    args = parser.parse_args()

    controller = SO101PickPlaceController(
        mode="sim",
        frequency=args.frequency,
        config_path=args.config,
    )
    controller.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
