#!/usr/bin/env python3
"""Robot joint control UI viewer.

Usage:
    $ python3 examples/robot_jointui.py fr3
    $ python3 examples/robot_jointui.py rm75
    $ python3 examples/robot_jointui.py piper
"""

from __future__ import annotations

import argparse

import mujoco
import mujoco.viewer
import numpy as np

from RynnMotion.utils.path_config import get_models_root

ROBOTS = {
    "fr3": "3.robot_arm/20.fr3/scene/scene.xml",
    "rm75": "3.robot_arm/23.rm75/scene/scene.xml",
    "piper": "3.robot_arm/22.piper/scene/scene.xml",
    "ur5e": "3.robot_arm/21.ur5e/scene/scene.xml",
    "so101": "3.robot_arm/24.so101/scene/scene.xml",
    "rizon4s": "3.robot_arm/25.rizon4s/scene/scene.xml",
}


class RobotJointUI:
    """MuJoCo viewer for robot joint control."""

    def __init__(self, scene_path: str):
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.q_fb = np.zeros(self.model.nq)
        self.qd_fb = np.zeros(self.model.nv)

    def run(self) -> None:
        """Run simulation loop."""
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                self.q_fb = self.data.qpos.copy()
                self.qd_fb = self.data.qvel.copy()
                mujoco.mj_step(self.model, self.data)
                viewer.sync()


def main() -> None:
    """Entry point."""
    parser = argparse.ArgumentParser(description="Robot joint control UI")
    parser.add_argument("robot", choices=ROBOTS.keys(), help="Robot name")
    args = parser.parse_args()

    models_root = get_models_root()
    scene_path = str(models_root / ROBOTS[args.robot])

    try:
        ui = RobotJointUI(scene_path)
        ui.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
