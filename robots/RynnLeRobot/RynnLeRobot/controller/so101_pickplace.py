#!/usr/bin/env python3
"""SO101 autonomous pick-and-place demonstration.

Usage:
    $ python -m RynnLeRobot.controller.so101_pickplace --frequency 100
    $ python -m RynnLeRobot.controller.so101_pickplace --config configs/so101.yaml
"""

from __future__ import annotations

import sys
import argparse
import numpy as np

from RynnMotion.core.base_controller import ControllerBase
from RynnMotion.utils.path_config import get_models_root
from RynnLeRobot.utils.cube_grabber import CubeGrabberLerp, CubeGrabberPControl


class SO101PickPlaceController(ControllerBase):
    """SO101 pick-and-place demonstration controller (simulation only).

    Responsibilities:
        - Initialize cube grabber for pick-and-place tasks
        - Track cube positions from MuJoCo simulation
        - Generate pick-and-place motion commands

    Attributes:
        cube_grabber: Utility for computing pick-and-place trajectories.
        cube_grabber_mode: Control mode ("lerp" or "Pcontrol").
    """

    def __init__(
        self,
        mode: str = "sim",
        frequency: int = 100,
        config_path: str = "configs/so101.yaml",
    ):
        if mode != "sim":
            raise ValueError("SO101 pick-and-place requires mode='sim'")

        self.cube_grabber = None
        self.cube_grabber_mode = None
        super().__init__(mode, frequency, config_path)

    def get_robot_name(self) -> str:
        return "so101"

    def get_scene_config(self) -> dict:
        return {
            "scene_path": "3.robot_arm/24.so101/scene/scene_grip.xml",
            "scene_name": "so101_grip",
            "pino_path": "3.robot_arm/24.so101/mjcf/so101_pinocchio.xml",
        }

    def init_communicator(self) -> None:
        pass

    def init_trajgen(self) -> None:
        pass

    def init_robot_state_monitor(self) -> None:
        pass

    def init_fsm(self) -> None:
        pass

    def on_init_complete(self) -> None:
        self._init_cube_grabber()

    def _init_cube_grabber(self) -> None:
        self.cube_grabber_mode = "Pcontrol"

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

    def pre_step_hook(self) -> None:
        qPos_feedback = self.interface.get_joint_positions()
        self.cube_grabber.current_joint_positions = qPos_feedback.copy()

        self.cube_grabber.all_cube_positions = {}
        for cube_name in self.cube_grabber.cube_names:
            pose = self.interface.data.joint(cube_name).qpos
            self.cube_grabber.all_cube_positions[cube_name] = pose[:3].copy()

        if self.cube_grabber.current_cube_name is not None:
            self.cube_grabber.current_cube_pose = self.interface.data.joint(
                self.cube_grabber.current_cube_name
            ).qpos

        if self.cube_grabber_mode == "Pcontrol":
            self.cube_grabber.current_pose = self.cube_grabber.get_SE3_pose(
                self.interface.data.site("ee").xpos,
                self.interface.data.site("ee").xmat,
            )

    def motion_planner(self, current_time: float) -> np.ndarray:
        self.cube_grabber.current_time = current_time
        self.cube_grabber.step()
        return self.cube_grabber.generated_joint_command.copy()


def main() -> int:
    parser = argparse.ArgumentParser(description="SO101 pick-and-place controller (simulation only)")
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
