#!/usr/bin/env python3
"""
teleoperation Test Script for LeRobot Motion Scripts

This script simulate teleoperate in mujoco,
you can define your own scene for your task in simulation,
using LCM for communication with real robot.

Usage:
    cd robots/lerobot
    python -m scripts.teleop_simulation --mode [sim | real]
"""

import os
import sys
import argparse
import numpy as np
import math
import time

import logging
import yaml

from RynnMotion.manager.robot_manager import RobotManager
from RynnMotion.trajgen.trajgen_base import generator_factory
from RynnMotion.common.communicator_base import communicator_factory
from RynnMotion.core.robotinterface_base import robotinterface_factory
from RynnMotion.utils.robot_state_publisher import RobotStatePublisher


class MotionSimRobot:
    """
    simulate robot motion.
    """

    def __init__(
        self,
        config_path: str = "configs/config.yaml",
    ):
        """
        Initialize the motion simulator.

        Args:
            config_path: path to the configuration file
        """
        self.config_path = config_path

        self.init_config()
        self.init_logging()
        self.init_robotmodel()
        self.init_communicator()
        self.init_trajgen()
        self.init_robot_interface()
        self.init_robot_state_monitor()

    def init_config(self):
        """Initialize configuration."""
        self.config = self.load_config(self.config_path)
        self.log_config = self.config.get("logging", {})
        self.robotmodel_config = self.config.get("robotmodel", {})
        self.conmmunicator_config = self.config.get("communicator", {})
        self.trajgen_config = self.config.get("trajgen", {})
        self.robotinterface_config = self.config.get("robotinterface", {})

    def init_logging(self):
        """Setup logging configuration."""
        log_dir = os.path.expanduser(self.log_config.get("log_dir", {}))
        log_file = self.log_config.get("log_file", {})

        if not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        from datetime import datetime

        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        log_file_with_timestamp = f"{log_file.replace('.log', '')}_{timestamp}.log"
        log_path = os.path.join(log_dir, log_file_with_timestamp)

        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s [%(levelname)s] %(message)s",
            handlers=[
                logging.FileHandler(log_path),
            ],
        )

        self.logger = logging.getLogger(__name__)
        print(f"✓ Log file: {log_path}")

    def init_robotmodel(self):
        """Initialize robot model."""
        self.robot_model = RobotManager(
            self.robotmodel_config,
            self.logger,
        )
        self.control_freq = self.robot_model.get_robot_control_freq()
        self.timestep = 1.0 / self.control_freq
        self.logger.info("✓ Load robot model successfully!\n")

    def init_communicator(self):
        """Initialize communicator."""
        self.conmmunicator_type = self.conmmunicator_config.get("type", "lcm")
        self.communicator = communicator_factory(
            self.conmmunicator_type,
            self.robot_model,
            self.conmmunicator_config,
            self.logger,
        )
        self.logger.info(f"✓communicator class build seccessfully!\n")

    def init_trajgen(self):
        """Initialize trajectory generator."""
        self.trajgen_name = self.trajgen_config.get("trajmodule", "wrong_type")
        self.trajgen = generator_factory(
            self.trajgen_name,
            self.trajgen_config,
            self.robot_model,
            self.communicator,
            self.logger,
        )
        self.logger.info(f"✓trajecory generation build seccessfully!\n")

    def init_robot_interface(self):
        """Setup interface - now unified for all modes!"""
        self.robot_interface = robotinterface_factory(
            "mujoco_sim_robot", self.robot_model, self.robotinterface_config
        )
        self.logger.info(f"✓ robot interface build seccessfully!\n")

    def init_robot_state_monitor(self):
        """Setup robot monitor - now unified for all modes!"""
        robot_monitor_config = self.config.get("robot_monitor", {})
        self.robot_state_monitor = RobotStatePublisher(
            self.robot_model, robot_monitor_config
        )

    def load_config(self, config_path):
        """Load configuration from YAML file."""
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
            return config
        except FileNotFoundError:
            (
                self.logger.warning(
                    f"Config file {config_path} not found, using defaults"
                )
                if hasattr(self, "logger")
                else print(
                    f"Warning: Config file {config_path} not found, using defaults"
                )
            )
            return {}

    def _pre_time_handling(self):
        """Handle timing setup at the start of each control loop iteration."""
        precall_time = self.robot_interface.get_current_time()
        precall_wall_time = time.time()
        nextcall_time = precall_time + self.timestep
        return precall_time, precall_wall_time, nextcall_time

    def _post_time_handling(self, precall_wall_time):
        """Handle timing synchronization at the end of each control loop iteration."""
        consumed_time = time.time() - precall_wall_time
        sleep_time = self.timestep - consumed_time

        if sleep_time > 0:
            time.sleep(sleep_time)
        elif sleep_time < -0.001:
            self.logger.warning(f"Control loop running behind by {-sleep_time:.3f}s")

    def prepare_run(self):
        try:
            self.communicator.connect()
            self.robot_interface.connect()

            robot_state = self.robot_interface.get_robot_state_feedbacks()
            self.trajgen.reset_trajectory_state(robot_state)

            self.robot_state_monitor.update_robot_states(robot_state, robot_state)
            self.robot_state_monitor.start()

        except (KeyboardInterrupt, Exception) as e:
            if isinstance(e, KeyboardInterrupt):
                self.logger.info("\nStopping robot...")
            else:
                self.logger.error(f"Error prepare run: {e}")

    def run(self):
        """Run the robot controller."""

        try:
            while True:
                precall_time, precall_wall_time, nextcall_time = (
                    self._pre_time_handling()
                )

                # todo: fsm update
                robot_state = self.robot_interface.get_robot_state_feedbacks()
                self.trajgen.process_input_command()
                traj_point, traj_complete = self.trajgen.update_trajectory()
                self.robot_interface.set_robot_command(traj_point)
                self.robot_interface.step()
                self.communicator.process_publish_robot_state(robot_state)
                self.robot_state_monitor.update_robot_states(robot_state, traj_point)

                self._post_time_handling(precall_wall_time)

        except (KeyboardInterrupt, Exception) as e:
            if isinstance(e, KeyboardInterrupt):
                self.logger.info("Stopping robot simulation...")
            else:
                self.logger.error(f"Error running simulation: {e}")

        self.cleanup()

    def cleanup(self):
        """Cleanup resources."""
        self.logger.info("Cleaning up...")

        if self.robot_interface:
            self.robot_interface.disconnect()
            self.logger.info("✓ Robot disconnected")

        if self.communicator:
            self.communicator.disconnect()
            self.logger.info("✓ Communicator disconnected")

        self.robot_state_monitor.stop()
        self.logger.info("✓ Cleanup completed")

    def wait_for_process_realstop(self):
        try:
            while True:
                print("Use ctrl + c to stop the main process!")
                time.sleep(5)
        except KeyboardInterrupt:
            print("Process interrupted by user successfully.")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="motion sim robot script")

    parser.add_argument(
        "--config",
        type=str,
        default="config/config_so101.yaml",
        help="Path to configuration file",
    )

    args = parser.parse_args()

    motion_sim_robot = MotionSimRobot(
        config_path=args.config,
    )
    motion_sim_robot.prepare_run()
    motion_sim_robot.run()
    motion_sim_robot.wait_for_process_realstop()


if __name__ == "__main__":
    sys.exit(main())
