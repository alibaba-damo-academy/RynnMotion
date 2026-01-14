#!/usr/bin/env python3
"""SO101 motion controller with predefined patterns.

Usage:
    $ python -m RynnLeRobot.controller.so101_motion --mode sim --motion 1
    $ python -m RynnLeRobot.controller.so101_motion --mode real --motion 2
"""

from __future__ import annotations

import sys
import argparse
import numpy as np

from RynnMotion.core.base_controller import ControllerBase
from RynnMotion.algorithms.policy_interpolator import lerp


class SO101MotionController(ControllerBase):
    """SO101 controller for predefined motion patterns.

    Responsibilities:
        - Generate 5 predefined motion patterns
        - Support simulation and real robot modes
        - Provide smooth trajectory interpolation

    Attributes:
        motion: Selected motion pattern (1-5).
        movement_parameters: Duration, amplitude, and frequency settings.
    """

    def __init__(
        self,
        mode: str = "sim",
        motion: int = 0,
        frequency: int = 100,
    ):
        self.motion = motion
        self.movement_parameters = {
            "movement_duration": 2.0,
            "amplitude": 0.5,
            "frequency": 0.2,
        }
        super().__init__(mode, frequency, "configs/so101.yaml")

    def get_robot_name(self) -> str:
        return "so101"

    def get_scene_config(self) -> dict:
        return {
            "scene_path": "3.robot_arm/24.so101/scene/scene.xml",
            "scene_name": "so101_scene",
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
        padded_positions = np.zeros(6)  # 5 arm + 1 gripper
        padded_positions[:len(self.initial_joint_positions)] = self.initial_joint_positions
        self.initial_joint_positions = padded_positions

    def motion_planner(self, current_time: float) -> np.ndarray:
        duration = self.movement_parameters["movement_duration"]
        amplitude = self.movement_parameters["amplitude"]
        freq = self.movement_parameters["frequency"]

        if self.motion == 1:
            target = np.array([0.0, -np.pi / 4.0, np.pi / 10.0, 0.0, 0.0, 0.0])
            if current_time <= duration:
                return lerp(self.initial_joint_positions.copy(), target, current_time, duration)
            t = current_time - duration
            target[0] += 1.0 * amplitude * np.sin(2 * np.pi * freq * t)
            target[1] += 0.8 * amplitude * np.sin(3 * np.pi * freq * t)
            target[2] += 1.2 * amplitude * np.sin(4 * np.pi * freq * t)
            return target

        elif self.motion == 2:
            if current_time <= duration:
                target = np.array([amplitude, 0.0, 0.0, 0.0, 0.0, 0.0])
                return lerp(self.initial_joint_positions.copy(), target, current_time, duration)
            target = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            t = current_time - duration
            target[0] += amplitude * np.cos(2 * np.pi * freq * t)
            target[2] += amplitude * np.sin(2 * np.pi * freq * t)
            return target

        elif self.motion == 3:
            if current_time <= duration:
                target = np.array([
                    1.5 * amplitude,
                    -1.5 * amplitude + np.pi / 2.0,
                    amplitude - np.pi / 2.0,
                    amplitude,
                    0.0,
                    0.0,
                ])
                return lerp(self.initial_joint_positions.copy(), target, current_time, duration)
            target = np.array([0.0, -1.5 * amplitude + np.pi / 2.0, -np.pi / 2.0, 0.0, 0.0, 0.0])
            t = current_time - duration
            target[0] += 1.5 * amplitude * np.cos(2 * np.pi * freq * t)
            target[2] += amplitude * np.cos(4 * np.pi * freq * t)
            target[3] += amplitude * np.cos(4 * np.pi * freq * t)
            return target

        elif self.motion == 4:
            target = np.array([0.5 * np.pi, 0.0, -np.pi / 2.0, 0.0, 0.0, 30.0])
            if current_time <= duration:
                return lerp(self.initial_joint_positions.copy(), target, current_time, duration)
            t = current_time - duration
            target[1] += amplitude * np.sin(2 * np.pi * freq * t)
            target[3] += amplitude * np.sin(2 * np.pi * freq * t)
            target[4] += 2 * amplitude * np.sin(2 * np.pi * freq * t)
            return target

        elif self.motion == 5:
            target = np.array([
                0.25 * np.pi,
                -0.2 * np.pi,
                0.2 * np.pi,
                0.25 * np.pi,
                -0.5 * np.pi,
                30.0,
            ])
            return lerp(self.initial_joint_positions.copy(), target, current_time, duration)

        return self.initial_joint_positions.copy()


def main() -> int:
    parser = argparse.ArgumentParser(description="SO101 motion controller with patterns 1-5")
    parser.add_argument("--mode", choices=["sim", "real", "mock"], default="sim")
    parser.add_argument("--motion", type=int, choices=[1, 2, 3, 4, 5], default=1)
    parser.add_argument("--frequency", type=int, default=100)
    args = parser.parse_args()

    controller = SO101MotionController(
        mode=args.mode,
        motion=args.motion,
        frequency=args.frequency,
    )
    controller.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
