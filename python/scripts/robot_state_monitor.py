import os
import sys
import argparse

import cv2
import numpy as np
import time
import yaml
from collections import deque
from threading import Thread, Lock
import lcm
from RynnMotion.manager.robot_manager import RobotManager
from RynnMotion.common.lcm.lcmMotion.robot_observation import robot_observation
from RynnMotion.common.data.robot_state import RobotState
from RynnMotion.utils.data_monitor import DataMonitor


class RobotStateMonitor:
    def __init__(self, config_path):
        # self.robot_state = RobotState()
        self.control_freq = 100
        self.dt = 1.0 / self.control_freq

        self.config = self.load_config(config_path)
        self.monitor_config = self.config.get("robot_monitor", {})
        self.enable_joint_monitor = self.monitor_config.get("enable_joint_pos", False)
        self.enable_eepose_monitor = self.monitor_config.get("enable_ee_pose", False)

        self.robotmodel_config = self.config.get("robotmodel", {})
        self.init_robotmodel()
        self.init_robot_monitor_data()

        # Initialize monitors
        if self.enable_joint_monitor:
            self.init_joint_monitor()
        if self.enable_eepose_monitor:
            self.init_ee_pose_monitor()

        # Flag to control monitoring
        self.running = False
        self.init_lcm_topic()

    def load_config(self, config_path):
        """Load configuration from YAML file."""
        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
            return config
        except FileNotFoundError:
            (
                print(f"Config file {config_path} not found, using defaults")
                if hasattr(self, "logger")
                else print(
                    f"Warning: Config file {config_path} not found, using defaults"
                )
            )
            return {}

    def init_robotmodel(self):
        """Initialize robot model."""
        self.robot_model = RobotManager(
            self.robotmodel_config,
            None,
        )
        self.control_freq = self.robot_model.get_robot_control_freq()
        self.dt = 1.0 / self.control_freq
        print("✓ Load robot model successfully!\n")

    def init_robot_monitor_data(self):
        """Initialize robot state and action."""
        self.robot_joint_dim = self.robot_model.get_actuator_num()
        self.robot_ee_num = self.robot_model.get_ee_num()
        self.robot_state = RobotState()
        self.robot_state.num_joints = self.robot_joint_dim
        self.robot_state.num_end_effectors = self.robot_ee_num
        self.robot_action = RobotState()
        self.robot_action.num_joints = self.robot_joint_dim
        self.robot_action.num_end_effectors = self.robot_ee_num

    def init_lcm_topic(self):
        """Initialize LCM connection and subscribe to channels."""
        try:
            self.lcm_instance = lcm.LCM()
            self.lcm_instance.subscribe(
                "robot_state_monitor/robot_state", self.handle_lcm_robot_state
            )
            self.lcm_instance.subscribe(
                "robot_state_monitor/robot_action", self.handle_lcm_robot_action
            )
            print(f"robot monitor setup successful!")
        except Exception as e:
            print(f"robot monitor setup failed: {e}")
            self.lcm_instance = None

    def init_joint_monitor(self):
        self.robot_joint_monitor = DataMonitor(
            data_dim=self.robot_joint_dim,
            control_freq=self.control_freq,
            data_name="joint positions",
            history_len=2000,
        )

    def init_ee_pose_monitor(self):
        self.robot_eepose_monitor = DataMonitor(
            data_dim=self.robot_ee_num * 7,
            control_freq=self.control_freq,
            data_name="end-effector pose",
            history_len=2000,
        )

    def handle_lcm_robot_state(self, channel, data):
        msg = robot_observation.decode(data)

        self.robot_state.num_joints = msg.numJoint
        self.robot_state.joint_pos = msg.jointPos
        self.robot_state.joint_vel = msg.jointVel
        self.robot_state.joint_torques = msg.jointTorque

        self.robot_state.num_end_effectors = msg.numEE
        for i in range(msg.numEE):
            self.robot_state.ee_pose[i].pos = msg.eePose[i][:3]
            self.robot_state.ee_pose[i].quat = msg.eePose[i][3:7]

    def handle_lcm_robot_action(self, channel, data):
        msg = robot_observation.decode(data)

        self.robot_action.num_joints = msg.numJoint
        self.robot_action.joint_pos = msg.jointPos
        self.robot_action.joint_vel = msg.jointVel
        self.robot_action.joint_torques = msg.jointTorque

        self.robot_action.num_end_effectors = msg.numEE
        for i in range(msg.numEE):
            self.robot_action.ee_pose[i].pos = msg.eePose[i][:3]
            self.robot_action.ee_pose[i].quat = msg.eePose[i][3:7]

    def stop(self):
        """Stop monitoring"""
        self.running = False
        self.thread.join(timeout=1)
        print("robot monitor thead stopped!")

    def robot_data_callback(self):
        """Callback function"""
        while self.running:
            self.update_robot_states(self.robot_state, self.robot_action)
            time.sleep(self.dt)

    def update_robot_states(self, robot_state, robot_action):
        """
        Update all robot states. Call this in your main control loop.

        Args:
            robot_state: RobotState object containing current robot state
        """
        if not self.running:
            return

        if self.lcm_instance:
            self.lcm_instance.handle_timeout(1)

        # Update joint states
        if self.enable_joint_monitor:
            self._update_robot_joint_states(robot_state, robot_action)

        # Update end-effector poses
        if self.enable_eepose_monitor:
            self._update_robot_end_effector_poses(robot_state)

    def _update_robot_joint_states(self, robot_state, robot_action):
        """Update joint state information"""
        self.robot_joint_monitor.update(robot_action.joint_pos, robot_state.joint_pos)

    def _update_robot_end_effector_poses(self, robot_state):
        """Update end-effector pose information"""
        eepos = [v for pose in robot_state.ee_pose for v in (pose.pos + pose.quat)]
        # Use same data for action and state since we're just monitoring
        self.robot_eepose_monitor.update(eepos, eepos)

    def run(self):
        self.running = True
        self.thread = Thread(target=self.robot_data_callback, daemon=True)
        self.thread.start()
        try:
            if self.enable_joint_monitor:
                self.robot_joint_monitor.init_window()
            if self.enable_eepose_monitor:
                self.robot_eepose_monitor.init_window()

            while self.running:
                postcall_time = time.time()
                # self.update_robot_states(self.robot_state, self.robot_action)
                not_update = True
                if self.enable_joint_monitor:
                    not_update = self.robot_joint_monitor.run_once() and not_update
                if self.enable_eepose_monitor:
                    not_update = self.robot_eepose_monitor.run_once() and not_update
                postcall_wall_time = time.time()

                sleep_time = self.dt - (postcall_wall_time - postcall_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                key = cv2.waitKey(1)
                if key == ord("q"):
                    self.running = False

            cv2.destroyAllWindows()

        except Exception as e:
            print(
                "robot monitor thread stopped! Can't use cv2 in this thead, Error:",
                e,
            )
        finally:
            cv2.destroyAllWindows()
            self.stop()


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

    robot_monitor = RobotStateMonitor(
        config_path=args.config,
    )
    robot_monitor.run()


if __name__ == "__main__":
    sys.exit(main())
