#!/usr/bin/env python3
"""Motion simulation robot using ControllerBase.

Supports FSM-based control, trajectory generation, and LCM communication.

Usage:
    $ python scripts/motion_sim_robotv2.py --config config/config_so101.yaml
    $ python scripts/motion_sim_robotv2.py --config config/config_fr3.yaml
"""

from __future__ import annotations

import sys
import argparse
import numpy as np
from RynnLeRobot import teleop

from RynnMotion.core.base_controller import ControllerBase


class TeleopMotionSimRobot(ControllerBase):
    """Motion simulation robot with FSM-based control.

    Responsibilities:
        - Initialize all components from YAML config
        - Delegate control pipeline to FSM
        - Publish robot state for monitoring

    Attributes:
        fsm: Finite state machine managing the control pipeline.
        trajgen: Trajectory generator for motion commands.
        robot_state_monitor: State publisher for external monitoring.
    """

    def __init__(self, config_path: str = "config/config_so101.yaml"):
        super().__init__(mode="sim", frequency=100, config_path=config_path)

    def get_robot_name(self) -> str:
        return self.robotmodel_config.get("robot_name", "unknown")

    def motion_planner(self, current_time: float) -> np.ndarray:
        if self.fsm:
            self.fsm.update()

        if self.robot_state_monitor and self.trajgen:
            robot_state = self.interface.get_robot_state_without_update()
            robot_action = self.trajgen.get_trajectory_command_without_update()
            self.robot_state_monitor.update_robot_states(robot_state, robot_action)

        return None


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Motion simulation robot using ControllerBase"
    )
    parser.add_argument(
        "--config",
        type=str,
        default="config/config_fr3_so101leader.yaml",
        help="Path to configuration file",
    )
    args = parser.parse_args()

    controller = TeleopMotionSimRobot(config_path=args.config)
    controller.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
