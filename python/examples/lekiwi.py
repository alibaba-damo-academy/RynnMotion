#!/usr/bin/env python3
"""LeKiwi mobile manipulator viewer with omnidirectional wheel control.

Usage:
    $ python3 examples/lekiwi.py
"""

from __future__ import annotations

import time

import mujoco
import mujoco.viewer
import numpy as np

from RynnMotion.utils.path_config import get_models_root


class LeKiwiController:
    """Controller for LeKiwi omnidirectional mobile robot."""

    def __init__(self, scene_path: str):
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.q_fb = np.zeros(self.model.nu)
        self.qd_fb = np.zeros(self.model.nu)

        wheel_radius = 0.1
        self.vel2wheel = 20 * np.array([
            [0, 1, -wheel_radius],
            [-np.sqrt(3) * 0.5, -0.5, -wheel_radius],
            [np.sqrt(3) * 0.5, -0.5, -wheel_radius],
        ])

    def run(self) -> None:
        """Run simulation loop."""
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                self.q_fb = self.data.qpos.copy()
                self.qd_fb = self.data.qvel.copy()

                cart_vel = np.array([self.qd_fb[1], self.qd_fb[2], self.qd_fb[0]])
                self.data.ctrl[3:6] = self.vel2wheel @ cart_vel

                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                time.sleep(0.001)


def main() -> None:
    """Entry point."""
    models_root = get_models_root()
    scene_path = str(models_root / "6.mobile_manipulator/60.lekiwi/scene/scene.xml")

    try:
        controller = LeKiwiController(scene_path)
        controller.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
