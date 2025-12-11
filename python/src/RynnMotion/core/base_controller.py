#!/usr/bin/env python3
"""Base controller for robot control applications.

Provides unified initialization and control loop for all robot controllers.

Usage:
    class MyController(ControllerBase):
        def get_robot_name(self) -> str:
            return "my_robot"

        def motion_planner(self, current_time: float) -> np.ndarray:
            return self.initial_joint_positions

Example:
    controller = MyController(mode="sim", config_path="config/robot.yaml")
    controller.run()
"""

from __future__ import annotations

import os
import time
import logging
import yaml
import numpy as np
from abc import ABC, abstractmethod
from datetime import datetime
from typing import Optional

from RynnMotion.core.interface_base import robotinterface_factory
from RynnMotion.manager.robot_manager import RobotManager
from RynnMotion.utils.path_config import get_models_root
from RynnMotion.common.data.robot_state import RobotState


class ControllerBase(ABC):
    """Abstract base class for robot controllers.

    Responsibilities:
        - Load configuration and setup logging
        - Initialize robot model, interface, and optional components
        - Run main control loop with timing synchronization
        - Handle cleanup on shutdown

    Attributes:
        mode: Operating mode ("sim" or "real").
        frequency: Control frequency in Hz.
        timestep: Control period in seconds.
        config: Loaded YAML configuration dictionary.
        robot_model: RobotManager instance for robot configuration.
        interface: Robot interface for simulation or hardware.
        initial_joint_positions: Joint positions at startup.
    """

    def __init__(
        self,
        mode: str = "sim",
        frequency: int = 100,
        config_path: str = "config/config.yaml",
    ):
        self.mode = mode
        self.frequency = max(30, min(frequency, 250))
        self.timestep = 1.0 / self.frequency
        self.config_path = config_path

        self.config: dict = {}
        self.logger: Optional[logging.Logger] = None
        self.robot_model: Optional[RobotManager] = None
        self.interface = None
        self.initial_joint_positions: Optional[np.ndarray] = None
        self.robot_command: Optional[RobotState] = None

        self.communicator = None
        self.trajgen = None
        self.fsm = None
        self.robot_state_monitor = None

        self.init_config()
        self.init_logging()
        self.init_robotmodel()
        self.init_communicator()
        self.init_trajgen()
        self.init_robot_interface()
        self.init_robot_state_monitor()
        self.init_fsm()
        self.on_init_complete()

    def init_config(self) -> None:
        """Load configuration from YAML file."""
        try:
            with open(self.config_path, "r") as f:
                self.config = yaml.safe_load(f) or {}
        except FileNotFoundError:
            self.config = {}

        self.log_config = self.config.get("logging", {})
        self.robotmodel_config = self.config.get("robotmodel", {})
        self.communicator_config = self.config.get("communicator", {})
        self.trajgen_config = self.config.get("trajgen", {})
        self.robotinterface_config = self.config.get("robotinterface", {})
        self.fsm_config = self.config.get("fsm", {})
        self.robot_monitor_config = self.config.get("robot_monitor", {})

        if self.robotmodel_config.get("robot_control_freq"):
            self.frequency = self.robotmodel_config["robot_control_freq"]
            self.timestep = 1.0 / self.frequency

    def init_logging(self) -> None:
        """Setup logging with timestamped log files."""
        log_dir = os.path.expanduser(self.log_config.get("log_dir", "~/logs/rynn"))
        log_file = self.log_config.get("log_file", f"{self.get_robot_name()}_controller.log")

        os.makedirs(log_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        log_path = os.path.join(log_dir, f"{log_file.replace('.log', '')}_{timestamp}.log")

        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s [%(levelname)s] %(message)s",
            handlers=[logging.FileHandler(log_path)],
        )
        self.logger = logging.getLogger(__name__)

    def init_robotmodel(self) -> None:
        """Initialize robot model from MJCF."""
        if self.mode != "sim":
            return

        models_root = get_models_root()

        robot_mjcf = self.robotmodel_config.get("robot_mjcf", "")
        pino_mjcf = self.robotmodel_config.get("pino_mjcf", "")

        if robot_mjcf:
            robot_mjcf = self._resolve_model_path(robot_mjcf, models_root)
            pino_mjcf = self._resolve_model_path(pino_mjcf, models_root)
        else:
            scene_config = self.get_scene_config()
            robot_mjcf = str(models_root / scene_config.get("scene_path", ""))
            pino_mjcf = str(models_root / scene_config.get("pino_path", ""))

        resolved_config = {
            "robot_name": self.get_robot_name(),
            "robot_control_freq": self.frequency,
            "robot_mjcf": robot_mjcf,
            "pino_mjcf": pino_mjcf,
        }

        self.robot_model = RobotManager(resolved_config, self.logger)
        self.logger.info(f"Robot model ({self.get_robot_name()}) initialized")

    def _resolve_model_path(self, path: str, models_root) -> str:
        """Resolve relative model paths to absolute paths."""
        if not path:
            return ""
        if path.startswith("../models/"):
            return str(models_root / path[10:])
        elif path.startswith("../"):
            return str(models_root / path[3:])
        elif path.startswith("/"):
            return path
        else:
            return str(models_root / path)

    def init_communicator(self) -> None:
        """Initialize communicator for external communication."""
        if not self.communicator_config:
            return

        try:
            from RynnMotion.common.communicator_base import communicator_factory

            communicator_type = self.communicator_config.get("type", "lcm")
            self.communicator = communicator_factory(
                communicator_type,
                self.robot_model,
                self.communicator_config,
                self.logger,
            )
            self.logger.info(f"Communicator ({communicator_type}) initialized")
        except Exception as e:
            self.logger.warning(f"Communicator init skipped: {e}")

    def init_trajgen(self) -> None:
        """Initialize trajectory generator."""
        if not self.trajgen_config:
            return

        try:
            from RynnMotion.trajgen.trajgen_base import generator_factory

            trajgen_name = self.trajgen_config.get("trajmodule", "sim_motion")
            self.trajgen = generator_factory(
                trajgen_name,
                self.trajgen_config,
                self.robot_model,
                self.communicator,
                self.logger,
            )
            self.logger.info(f"Trajectory generator ({trajgen_name}) initialized")
        except Exception as e:
            self.logger.warning(f"Trajgen init skipped: {e}")

    def init_robot_interface(self) -> None:
        """Initialize robot interface for simulation or hardware."""
        if self.mode == "sim":
            self._init_sim_interface()
        else:
            self._init_real_interface()

        self.interface.connect()
        self._init_joint_state()
        self.logger.info(f"Robot interface initialized (mode={self.mode})")

    def _init_sim_interface(self) -> None:
        self.interface = robotinterface_factory(
            "mujoco_sim_robot",
            self.robot_model,
            self.robotinterface_config,
        )

    def _init_real_interface(self) -> None:
        raise NotImplementedError(
            f"Real robot interface not implemented for {self.get_robot_name()}. "
            "Override _init_real_interface() in your controller subclass."
        )

    def _init_joint_state(self) -> None:
        self.initial_joint_positions = self.interface.get_joint_positions()

        if self.mode == "sim":
            self.robot_command = RobotState()
            self.robot_command.num_joints = self.interface.mdof

    def init_robot_state_monitor(self) -> None:
        """Initialize robot state publisher for monitoring."""
        if not self.robot_monitor_config:
            return

        try:
            from RynnMotion.utils.robot_state_publisher import RobotStatePublisher

            self.robot_state_monitor = RobotStatePublisher(
                self.robot_model, self.robot_monitor_config
            )
            self.logger.info("Robot state monitor initialized")
        except Exception as e:
            self.logger.warning(f"Robot state monitor init skipped: {e}")

    def init_fsm(self) -> None:
        """Initialize finite state machine for task management."""
        if not self.fsm_config:
            return

        try:
            from RynnMotion.manager.fsm_base import fsm_factory

            fsm_type = self.fsm_config.get("fsm_type", "basic_fsm")
            self.fsm = fsm_factory(
                fsm_type,
                self.fsm_config,
                self.robot_model,
                self.communicator,
                self.trajgen,
                self.interface,
                self.logger,
            )
            self.logger.info(f"FSM ({fsm_type}) initialized")
        except Exception as e:
            self.logger.warning(f"FSM init skipped: {e}")

    @abstractmethod
    def get_robot_name(self) -> str:
        """Return the robot identifier string."""
        pass

    def get_scene_config(self) -> dict:
        """Return scene configuration for simulation."""
        return {
            "scene_path": "",
            "pino_path": "",
            "scene_name": self.get_robot_name(),
        }

    @abstractmethod
    def motion_planner(self, current_time: float) -> np.ndarray:
        """Compute joint commands for the current timestep.

        Args:
            current_time: Current simulation/wall time in seconds.

        Returns:
            Joint position command array, or None if FSM handles commands.
        """
        pass

    def on_init_complete(self) -> None:
        """Hook called after all initialization is complete."""
        pass

    def pre_step_hook(self) -> None:
        """Hook called before each control step."""
        pass

    def post_step_hook(self) -> None:
        """Hook called after each control step."""
        pass

    def should_continue(self) -> bool:
        """Check if the control loop should continue."""
        if self.mode == "sim":
            return self.interface.is_viewer_alive()
        return True

    def _send_command(self, command: np.ndarray) -> None:
        if command is None:
            return

        if self.mode == "sim":
            for i in range(min(len(command), self.interface.mdof)):
                self.robot_command.joint_pos[i] = command[i]
            self.interface.set_robot_command(self.robot_command)

            if len(command) > self.interface.mdof:
                gripper_idx = self.interface.mdof
                self.interface.mjData.ctrl[gripper_idx] = command[gripper_idx]
        else:
            self.interface.set_joint_positions(command)

    def prepare_run(self) -> None:
        """Prepare for running the control loop."""
        if self.robot_state_monitor:
            self.robot_state_monitor.start()

    def run(self) -> None:
        """Run main control loop."""
        self.logger.info(
            f"{self.__class__.__name__}: mode={self.mode}, freq={self.frequency}Hz"
        )

        self.prepare_run()

        try:
            while self.should_continue():
                precall_wall_time = time.time()
                current_time = self.interface.get_current_time()

                self.pre_step_hook()
                command = self.motion_planner(current_time)
                self._send_command(command)
                self.interface.step()
                self.post_step_hook()

                sleep_time = self.timestep - (time.time() - precall_wall_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -0.001:
                    self.logger.warning(f"Control loop behind by {-sleep_time:.3f}s")

        except KeyboardInterrupt:
            self.logger.info("Controller interrupted by user")
        finally:
            self.cleanup()

    def cleanup(self) -> None:
        """Clean up resources on shutdown."""
        self.logger.info("Cleaning up...")

        if self.interface:
            self.interface.disconnect()
            self.logger.info("Robot interface disconnected")

        if self.communicator:
            self.communicator.disconnect()
            self.logger.info("Communicator disconnected")

        if self.robot_state_monitor:
            self.robot_state_monitor.stop()
            self.logger.info("Robot state monitor stopped")

        self.logger.info("Cleanup completed")
